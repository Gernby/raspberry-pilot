#!/usr/bin/env python3 
import capnp
import time
import sys
from cereal import car, log
from common.numpy_fast import clip
from common.params import Params
from common.profiler import Profiler
import selfdrive.messaging as messaging
from selfdrive.services import service_list
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_alert
from selfdrive.controls.lib.drive_helpers import get_events, \
                                                 create_event, \
                                                 EventTypes as ET, \
                                                 update_v_cruise, \
                                                 initialize_v_cruise
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.laterald import Lateral
from selfdrive.controls.lib.alertmanager import AlertManager
from setproctitle import setproctitle


ThermalStatus = log.ThermalData.ThermalStatus
State = log.ControlsState.OpenpilotState
#import sys
#sys.stderr = open('../controlsd.txt', 'w')

def isActive(state):
  """Check if the actuators are enabled"""
  return state in [State.enabled, State.softDisabling]


def isEnabled(state):
  """Check if openpilot is engaged"""
  return (isActive(state) or state == State.preEnabled)

def events_to_bytes(events):
  # optimization when comparing capnp structs: str() or tree traverse are much slower
  ret = []
  for e in events:
    if isinstance(e, capnp.lib.capnp._DynamicStructReader):
      e = e.as_builder()
    ret.append(e.to_bytes())
  return ret

def wait_for_can(logcan):
  print("Waiting for CAN messages...")
  while len(messaging.recv_sock(logcan, wait=True).can) == 0:
    pass

def data_sample(CI, CC, can_sock, carstate, lac_log, lateral, sm, profiler):
  """Receive data from sockets and create events for battery, temperature and disk space"""
 
  can_strs = [can_sock.recv()]
  profiler.checkpoint('can_recv', True)
  CS = CI.update(CC, can_strs, lac_log, profiler)
  profiler.checkpoint('carstate')
  lateral.update(CS, sm, 0, 1)
  profiler.checkpoint('lateral')
  if CS.canTime + 20 < CS.sysTime: 
    can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=False, limit=1)
    profiler.checkpoint('drain_can')
    if len(can_strs) > 0: 
      print("  Controls lagged by %d CAN packets at %d at %0.2f m/s!" % (len(can_strs), int(time.time()*1000), CS.vEgo), [len(x) for x in can_strs])
      for i in range(len(can_strs[-40:])):
        #time.sleep(0.00001)
        CS = CI.update(CC, [can_strs[i]], lac_log, profiler)
        profiler.checkpoint('drain_carstate')
        lateral.update(CS, sm, i, len(can_strs[-40:]))
        profiler.checkpoint('drain_lateral')
    else:
      #print("  CAN lagged!")
      CI.canTime += 20
  #else:
  #  time.sleep(0.00001)

  events = list(CS.events)

  return CS, events

def state_transition(frame, CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM):
  """Compute conditional state transitions and execute actions on state transitions"""
  enabled = isEnabled(state)

  v_cruise_kph_last = v_cruise_kph

  # if stock cruise is completely disabled, then we can use our own set speed logic
  if not CP.enableCruise:
    v_cruise_kph = update_v_cruise(v_cruise_kph, CS.buttonEvents, enabled)
  elif CP.enableCruise and CS.cruiseState.enabled:
    v_cruise_kph = CS.cruiseState.speed #* CV.MS_TO_KPH

  # decrease the soft disable timer at every step, as it's reset on
  # entrance in SOFT_DISABLING state
  soft_disable_timer = max(0, soft_disable_timer - 1)

  # DISABLED
  if state == State.disabled:
    if get_events(events, [ET.ENABLE]):
      if get_events(events, [ET.NO_ENTRY]):
        for e in get_events(events, [ET.NO_ENTRY]):
          AM.add(frame, str(e) + "NoEntry", enabled)

      else:
        if get_events(events, [ET.PRE_ENABLE]):
          state = State.preEnabled
        else:
          state = State.enabled
        AM.add(frame, "enable", enabled)
        v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, v_cruise_kph_last)

  # ENABLED
  elif state == State.enabled:
    if get_events(events, [ET.USER_DISABLE]):
      state = State.disabled
      AM.add(frame, "disable", enabled)

    elif get_events(events, [ET.IMMEDIATE_DISABLE]):
      state = State.disabled
      for e in get_events(events, [ET.IMMEDIATE_DISABLE]):
        AM.add(frame, e, enabled)

    elif get_events(events, [ET.SOFT_DISABLE]):
      state = State.softDisabling
      soft_disable_timer = 600   # 6s
      for e in get_events(events, [ET.SOFT_DISABLE]):
        AM.add(frame, e, enabled)

  # SOFT DISABLING
  elif state == State.softDisabling:
    if get_events(events, [ET.USER_DISABLE]):
      state = State.disabled
      AM.add(frame, "disable", enabled)

    elif get_events(events, [ET.IMMEDIATE_DISABLE]):
      state = State.disabled
      for e in get_events(events, [ET.IMMEDIATE_DISABLE]):
        AM.add(frame, e, enabled)

    elif not get_events(events, [ET.SOFT_DISABLE]):
      # no more soft disabling condition, so go back to ENABLED
      state = State.enabled

    elif get_events(events, [ET.SOFT_DISABLE]) and soft_disable_timer > 0:
      for e in get_events(events, [ET.SOFT_DISABLE]):
        AM.add(frame, e, enabled)

    elif soft_disable_timer <= 0:
      state = State.disabled

  # PRE ENABLING
  elif state == State.preEnabled:
    if get_events(events, [ET.USER_DISABLE]):
      state = State.disabled
      AM.add(frame, "disable", enabled)

    elif get_events(events, [ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE]):
      state = State.disabled
      for e in get_events(events, [ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE]):
        AM.add(frame, e, enabled)

    elif not get_events(events, [ET.PRE_ENABLE]):
      state = State.enabled

  return state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last


def state_control(frame, lkasMode, path_plan, CS, CP, state, events, AM, LaC, lac_log, profiler):
  """Given the state, this function returns an actuators packet"""

  actuators = car.CarControl.Actuators.new_message()

  enabled = isEnabled(state)
  active = isActive(state)

  # Steering PID loop and lateral MPC
  actuators.steer, actuators.steerAngle, lac_log = LaC.update(path_plan.paramsValid and CS.lkMode and (active or lkasMode), CS.brakePressed, CS.vEgo, CS.steeringAngle, CS.steeringRate, CS.steeringPressed, CP, path_plan, CS.canTime, CS.blinkers)
  profiler.checkpoint('lac_update')
    # parse warnings from car specific interface
  for e in get_events(events, [ET.WARNING]):
    extra_text = ""
    AM.add(frame, e, enabled, extra_text_2=extra_text)

  # Parse permanent warnings to display constantly
  for e in get_events(events, [ET.PERMANENT]):
    extra_text_1, extra_text_2 = "", ""
    AM.add(frame, str(e) + "Permanent", enabled, extra_text_1=extra_text_1, extra_text_2=extra_text_2)

  AM.process_alerts(frame)

  return actuators, lac_log

def data_send(sm, CS, CI, CP, state, events, actuators, carstate, carcontrol, carevents, carparams, controlsstate, sendcan, AM, LaC, start_time, lac_log, events_prev, profiler):
  """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

  CC = car.CarControl.new_message()
  CC.enabled = isEnabled(state)
  CC.actuators = actuators

  CC.cruiseControl.override = True
  CC.cruiseControl.cancel = not CP.enableCruise or (not isEnabled(state) and CS.cruiseState.enabled)

  #CC.hudControl.setSpeed = float(v_cruise_kph * CV.KPH_TO_MS)
  CC.hudControl.speedVisible = isEnabled(state)
  CC.hudControl.lanesVisible = isEnabled(state)

  right_lane_visible = sm['pathPlan'].rProb > 0.5
  left_lane_visible = sm['pathPlan'].lProb > 0.5

  CC.hudControl.rightLaneVisible = bool(right_lane_visible)
  CC.hudControl.leftLaneVisible = bool(left_lane_visible)

  CC.hudControl.visualAlert = AM.visual_alert
  CC.hudControl.audibleAlert = AM.audible_alert
  profiler.checkpoint('data_send')
  can_sends = CI.apply(CC)
  profiler.checkpoint('ci_apply')
  sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
  events_bytes = None

  return CC, events_bytes


def controlsd_thread(gctx=None):
  setproctitle('controlsd')
  params = Params()
  print(params)
  # Pub Sockets
  profiler = Profiler(True, 'controls')

  sendcan = messaging.pub_sock(service_list['sendcan'].port)
  controlsstate = messaging.pub_sock(service_list['controlsState'].port)
  carstate = None #messaging.pub_sock(service_list['carState'].port)
  carcontrol = messaging.pub_sock(service_list['carControl'].port)
  carevents = messaging.pub_sock(service_list['carEvents'].port)
  carparams = messaging.pub_sock(service_list['carParams'].port)

  sm = messaging.SubMaster(['pathPlan','health','gpsLocationExternal'])
  can_sock = messaging.sub_sock(service_list['can'].port)
  if len(sys.argv) >= 2 and sys.argv[1] == 'wait':
    print("waiting for transcoder")
    messaging.recv_one(sm.sock['pathPlan'])
    print("transcoder ready")
  else:
    print('not waiting for transcoder')
  hw_type = messaging.recv_one(sm.sock['health']).health.hwType
  is_panda_black = hw_type == log.HealthData.HwType.blackPanda  
  print("\npanda black: ", is_panda_black)
  wait_for_can(can_sock)
  
  CI, CP = get_car(can_sock, sendcan, is_panda_black)
  #logcan.close()

  # TODO: Use the logcan socket from above, but that will currenly break the tests
  #can_timeout = None #if os.environ.get('NO_CAN_TIMEOUT', False) else 100
  #can_sock = messaging.sub_sock(service_list['can'].port, timeout=can_timeout)

  # Write CarParams for radard and boardd safety mode
  params.put("CarParams", CP.to_bytes())
  params.put("LongitudinalControl", "1" if CP.openpilotLongitudinalControl else "0")

  CC = car.CarControl.new_message()
  AM = AlertManager()

  startup_alert = get_startup_alert(True, True)
  AM.add(sm.frame, startup_alert, False)    

  LaC = LatControlPID(CP)
  lateral = Lateral(CP)
  lkasMode = int(float(LaC.kegman.conf['lkasMode']))
  #CI.CS.lkasMode = (lkasMode == 0)
  lac_log = None #car.CarState.lateralControlState.pidState.new_message()

  state = State.disabled
  soft_disable_timer = 0
  v_cruise_kph = 255
  events_prev = []
  frame = 0

  sm['pathPlan'].sensorValid = True
  sm['pathPlan'].posenetValid = True

  while True:

    start_time = 0 # time.time()  #sec_since_boot()

    # Sample data and compute car events
    CS, events = data_sample(CI, CC, can_sock, carstate, lac_log, lateral, sm, profiler)
    profiler.checkpoint('data_sample')

    state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last = \
        state_transition(sm.frame, CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM)
    profiler.checkpoint('state_transition')
    # Compute actuators (runs PID loops and lateral MPC)
    sm.update(0)
    profiler.checkpoint('sm_update')

    actuators, lac_log = state_control(sm.frame, lkasMode, sm['pathPlan'], CS, CP, state, events, AM, LaC, lac_log, profiler)
    profiler.checkpoint('state_control')

    # Publish data
    CC, events_prev = data_send(sm, CS, CI, CP, state, events, actuators, carstate, carcontrol, carevents, carparams,
                    controlsstate, sendcan, AM, LaC, start_time, lac_log, events_prev, profiler)
    profiler.checkpoint('data_send')
    frame += 1
    if frame % 10000 == 0 and profiler.enabled:
      profiler.display()
      profiler.reset(True)

    
def main(gctx=None):
  controlsd_thread(gctx)

if __name__ == "__main__":
  main()
