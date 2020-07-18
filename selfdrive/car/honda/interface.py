#!/usr/bin/env python3 
import os
import time
import numpy as np
from cereal import car, log
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot, DT_CTRL
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET, get_events
#from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.honda.carstate import CarState, get_can_parser, get_cam_can_parser
from selfdrive.car.honda.values import CruiseButtons, CAR, HONDA_BOSCH, AUDIO_HUD, VISUAL_HUD, CAMERA_MSGS
from selfdrive.car import STD_CARGO_KG, CivicParams, scale_rot_inertia, scale_tire_stiffness
from selfdrive.car.interfaces import CarInterfaceBase
#from selfdrive.controls.lib.planner import _A_CRUISE_MAX_V_FOLLOWING

A_ACC_MAX = 0 #max(_A_CRUISE_MAX_V_FOLLOWING)


def compute_gb_honda(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  return float(accel) / 4.8 - creep_brake


def get_compute_gb_acura():
  # generate a function that takes in [desired_accel, current_speed] -> [-1.0, 1.0]
  # where -1.0 is max brake and 1.0 is max gas
  # see debug/dump_accel_from_fiber.py to see how those parameters were generated
  w0 = np.array([[ 1.22056961, -0.39625418,  0.67952657],
                 [ 1.03691769,  0.78210306, -0.41343188]])
  b0 = np.array([ 0.01536703, -0.14335321, -0.26932889])
  w2 = np.array([[-0.59124422,  0.42899439,  0.38660881],
                 [ 0.79973811,  0.13178682,  0.08550351],
                 [-0.15651935, -0.44360259,  0.76910877]])
  b2 = np.array([ 0.15624429,  0.02294923, -0.0341086 ])
  w4 = np.array([[-0.31521443],
                 [-0.38626176],
                 [ 0.52667892]])
  b4 = np.array([-0.02922216])

  def compute_output(dat, w0, b0, w2, b2, w4, b4):
    m0 = np.dot(dat, w0) + b0
    m0 = leakyrelu(m0, 0.1)
    m2 = np.dot(m0, w2) + b2
    m2 = leakyrelu(m2, 0.1)
    m4 = np.dot(m2, w4) + b4
    return m4

  def leakyrelu(x, alpha):
    return np.maximum(x, alpha * x)

  def _compute_gb_acura(accel, speed):
    # linearly extrap below v1 using v1 and v2 data
    v1 = 5.
    v2 = 10.
    dat = np.array([accel, speed])
    if speed > 5.:
      m4 = compute_output(dat, w0, b0, w2, b2, w4, b4)
    else:
      dat[1] = v1
      m4v1 = compute_output(dat, w0, b0, w2, b2, w4, b4)
      dat[1] = v2
      m4v2 = compute_output(dat, w0, b0, w2, b2, w4, b4)
      m4 = (speed - v1) * (m4v2 - m4v1) / (v2 - v1) + m4v1
    return float(m4)

  return _compute_gb_acura


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController):
    self.CP = CP

    self.frame = 0
    self.last_enable_pressed = 0
    self.last_enable_sent = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.stock_cam_frame_prev = 0

    self.cp = get_can_parser(CP)
    self.cp_cam = get_cam_can_parser(CP.isPandaBlack)

    # *** init the major players ***
    self.CS = CarState(CP)
    #self.VM = VehicleModel(CP)
    self.canTime = 0
    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name)

    if self.CS.CP.carFingerprint == CAR.ACURA_ILX:
      self.compute_gb = get_compute_gb_acura()
    else:
      self.compute_gb = compute_gb_honda

    if self.CS.CP.carFingerprint in HONDA_BOSCH and self.CS.CP.carFingerprint not in (CAR.CRV_HYBRID, CAR.CRV, CAR.CRV_5G):
      self.bosch_honda = True
    else:
      self.bosch_honda = False

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):

    # normalized max accel. Allowing max accel at low speed causes speed overshoots
    max_accel_bp = [10, 20]    # m/s
    max_accel_v = [0.714, 1.0] # unit of max accel
    max_accel = interp(v_ego, max_accel_bp, max_accel_v)

    # limit the pcm accel cmd if:
    # - v_ego exceeds v_target, or
    # - a_ego exceeds a_target and v_ego is close to v_target

    eA = a_ego - a_target
    valuesA = [1.0, 0.1]
    bpA = [0.3, 1.1]

    eV = v_ego - v_target
    valuesV = [1.0, 0.1]
    bpV = [0.0, 0.5]

    valuesRangeV = [1., 0.]
    bpRangeV = [-1., 0.]

    # only limit if v_ego is close to v_target
    speedLimiter = interp(eV, bpV, valuesV)
    accelLimiter = max(interp(eA, bpA, valuesA), interp(eV, bpRangeV, valuesRangeV))

    # accelOverride is more or less the max throttle allowed to pcm: usually set to a constant
    # unless aTargetMax is very high and then we scale with it; this help in quicker restart

    return float(max(max_accel, a_target / A_ACC_MAX)) * min(speedLimiter, accelLimiter)

  @staticmethod
  def get_params(candidate, fingerprint, vin="", is_panda_black=False):

    ret = car.CarParams.new_message()
    ret.carName = "honda"
    ret.carFingerprint = candidate
    ret.carVin = vin
    ret.isPandaBlack = is_panda_black

    if candidate in HONDA_BOSCH:
      ret.safetyModel = car.CarParams.SafetyModel.hondaBosch
      ret.enableCamera = True
      ret.radarOffCan = True
      ret.openpilotLongitudinalControl = False
    else:
      ret.safetyModel = car.CarParams.SafetyModel.honda
      ret.enableCamera = not any(x for x in CAMERA_MSGS if x in fingerprint) or is_panda_black
      ret.enableGasInterceptor = 0x201 in fingerprint
      ret.openpilotLongitudinalControl = ret.enableCamera

    cloudlog.warn("ECU Camera Simulated: %r", ret.enableCamera)
    cloudlog.warn("ECU Gas Interceptor: %r", ret.enableGasInterceptor)

    ret.enableCruise = not ret.enableGasInterceptor

    # Optimized car params: tire_stiffness_factor and steerRatio are a result of a vehicle
    # model optimization process. Certain Hondas have an extra steering sensor at the bottom
    # of the steering rack, which improves controls quality as it removes the steering column
    # torsion from feedback.
    # Tire stiffness factor fictitiously lower if it includes the steering column torsion effect.
    # For modeling details, see p.198-200 in "The Science of Vehicle Dynamics (2014), M. Guiggiani"
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kf = 0.00006 # conservative feed-forward
    ret.lateralTuning.pid.dampTime = 0.02
    ret.lateralTuning.pid.reactMPC = 0.0
    ret.lateralTuning.pid.dampMPC = 0.25
    ret.lateralTuning.pid.rateFFGain = 0.4
    ret.lateralTuning.pid.polyFactor = 0.01
    ret.lateralTuning.pid.polyDampTime = 0.15
    ret.lateralTuning.pid.polyReactTime = 0.5
    ret.steerAdvanceCycles = 9
    #ret.epsSteerRateFactor = -0.08

    if candidate in [CAR.CIVIC, CAR.CIVIC_BOSCH]:
      stop_and_go = True
      ret.mass = CivicParams.MASS
      ret.wheelbase = CivicParams.WHEELBASE
      ret.centerToFront = CivicParams.CENTER_TO_FRONT
      ret.steerRatio = 15.38  # 10.93 is end-to-end spec
      tire_stiffness_factor = 1.
      # Civic at comma has modified steering FW, so different tuning for the Neo in that car
      is_fw_modified = os.getenv("DONGLE_ID") in ['99c94dc769b5d96e']
      if is_fw_modified:
        ret.lateralTuning.pid.kf = 0.00004

      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.4], [0.12]] if is_fw_modified else [[0.8], [0.18]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.54, 0.36]
      ret.lateralTuning.pid.dampTime = 0.1
      ret.lateralTuning.pid.reactMPC = 0.0
      #ret.epsSteerRateFactor = -0.08

    elif candidate in (CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH):
      stop_and_go = True
      if not candidate == CAR.ACCORDH: # Hybrid uses same brake msg as hatch
        ret.safetyParam = 1 # Accord and CRV 5G use an alternate user brake msg
      ret.mass = 3279. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.83
      ret.centerToFront = ret.wheelbase * 0.39
      ret.steerRatio = 15.96  # 11.82 is spec end-to-end
      tire_stiffness_factor = 0.8467
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]
      ret.lateralTuning.pid.dampTime = 0.1
      ret.lateralTuning.pid.reactMPC = 0.0
      #ret.epsSteerRateFactor = -0.12
      #ret.lateralTuning.pid.steerPscale = [[1.0, 2.0, 10.0], [1.0, 0.5, 0.25], [1.0, 0.75, 0.5]]  # [abs angles, scale UP, scale DOWN]
      ret.steerLimitAlert = False

    elif candidate == CAR.ACURA_ILX:
      stop_and_go = False
      ret.mass = 3095. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.67
      ret.centerToFront = ret.wheelbase * 0.37
      ret.steerRatio = 18.61  # 15.3 is spec end-to-end
      tire_stiffness_factor = 0.72
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]
      ret.lateralTuning.pid.dampTime = 0.1
      ret.lateralTuning.pid.reactMPC = 0.0
      ret.lateralTuning.pid.rateFFGain = 0.4
      #ret.epsSteerRateFactor = -0.08

    elif candidate == CAR.CRV:
      stop_and_go = False
      ret.mass = 3572. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.62
      ret.centerToFront = ret.wheelbase * 0.41
      ret.steerRatio = 16.89         # as spec
      tire_stiffness_factor = 0.444 # not optimized yet
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]
      ret.lateralTuning.pid.dampTime = 0.1
      ret.lateralTuning.pid.reactMPC = 0.0
      #ret.epsSteerRateFactor = -0.1375

    elif candidate == CAR.CRV_5G:
      stop_and_go = True
      ret.safetyParam = 1 # Accord and CRV 5G use an alternate user brake msg
      ret.mass = 3410. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.66
      ret.centerToFront = ret.wheelbase * 0.41
      ret.steerRatio = 16.0   # 12.3 is spec end-to-end
      tire_stiffness_factor = 0.677
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]
      ret.lateralTuning.pid.dampTime = 0.1
      ret.lateralTuning.pid.reactMPC = 0.0

    elif candidate == CAR.CRV_HYBRID:
      stop_and_go = True
      ret.safetyParam = 1 # Accord and CRV 5G use an alternate user brake msg
      ret.mass = 1667. + STD_CARGO_KG # mean of 4 models in kg
      ret.wheelbase = 2.66
      ret.centerToFront = ret.wheelbase * 0.41
      ret.steerRatio = 16.0   # 12.3 is spec end-to-end
      tire_stiffness_factor = 0.677
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]
      ret.lateralTuning.pid.dampTime = 0.1
      ret.lateralTuning.pid.reactMPC = 0.0
      ret.steerLimitAlert = False

    elif candidate == CAR.ACURA_RDX:
      stop_and_go = False
      ret.mass = 3935. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.68
      ret.centerToFront = ret.wheelbase * 0.38
      ret.steerRatio = 15.0         # as spec
      tire_stiffness_factor = 0.444 # not optimized yet
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]

    elif candidate == CAR.INSIGHT:
      stop_and_go = True
      ret.mass = 2987. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.centerToFront = ret.wheelbase * 0.39
      ret.steerRatio = 15  # 12.58 is spec end-to-end
      tire_stiffness_factor = 0.82
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]

    elif candidate == CAR.ODYSSEY:
      stop_and_go = False
      ret.mass = 4471. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 3.00
      ret.centerToFront = ret.wheelbase * 0.41
      ret.steerRatio = 14.35        # as spec
      tire_stiffness_factor = 0.82
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.45], [0.135]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]

    elif candidate == CAR.ODYSSEY_CHN:
      stop_and_go = False
      ret.mass = 1849.2 + STD_CARGO_KG # mean of 4 models in kg
      ret.wheelbase = 2.90 # spec
      ret.centerToFront = ret.wheelbase * 0.41 # from CAR.ODYSSEY
      ret.steerRatio = 14.35 # from CAR.ODYSSEY
      tire_stiffness_factor = 0.82 # from CAR.ODYSSEY
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.45], [0.135]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]

    elif candidate in (CAR.PILOT, CAR.PILOT_2019):
      stop_and_go = False
      ret.mass = 4204. * CV.LB_TO_KG + STD_CARGO_KG # average weight
      ret.wheelbase = 2.82
      ret.steerRatio = 17.25
      ret.centerToFront = ret.wheelbase * 0.428 # average weight distribution
      tire_stiffness_factor = 0.444 # not optimized yet
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.38], [0.11]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]

    elif candidate == CAR.RIDGELINE:
      stop_and_go = False
      ret.mass = 4515. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 3.18
      ret.centerToFront = ret.wheelbase * 0.41
      ret.steerRatio = 15.59        # as spec
      tire_stiffness_factor = 0.82 # not optimized yet
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.40], [0.20]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]

    else:
      raise ValueError("unsupported car %s" % candidate)

    ret.steerControlType = car.CarParams.SteerControlType.torque

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter. Otherwise, add 0.5 mph margin to not
    # conflict with PCM acc
    ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else 25.5 * CV.MPH_TO_MS

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.

    # no max steer limit VS speed
    ret.steerMaxBP = [0.]  # m/s
    ret.steerMaxV = [1.]   # max steer allowed

    # prevent lurching when resuming
    if ret.enableGasInterceptor:
      ret.gasMaxBP = [0., 3, 8, 35]
      ret.gasMaxV = [0.2, 0.3, 0.5, 0.6]
    else:
      ret.gasMaxBP = [0.]  # m/s
      ret.gasMaxV = [0.] # max gas allowed

    #ret.gasMaxBP = [0.]  # m/s
    #ret.gasMaxV = [0.6] if ret.enableGasInterceptor else [0.] # max gas allowed
    ret.brakeMaxBP = [5., 20.]  # m/s
    ret.brakeMaxV = [1., 0.8]   # max brake allowed

    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]

    ret.stoppingControl = True
    ret.startAccel = 0.5

    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.3

    return ret

  # returns a car.CarState
  def update(self, c, can_strings, lac_log):
    # ******************* do can recv *******************
    ret = car.CarState.new_message()
    ret.lateralControlState.init('pidState')
    ret.sysTime = int(time.time() * 100) * 10
    if self.canTime == 0: 
      self.canTime = ret.sysTime
    elif self.canTime < ret.sysTime + 20:
      self.canTime = self.canTime + 10
    ret.canTime = self.canTime
    #self.canTime = max(int(time.time() * 100) * 10, self.canTime + 10)
    #if self.frame % 100 == 0: print(self.canTime)

    self.cp.update_strings(can_strings)
    #self.cp.update(0, False)
    if not self.cp_cam is None: 
      self.cp_cam.update_strings(can_strings)
      #self.cp_cam.update(0, False)

    self.CS.update(self.cp, self.cp_cam)
    # create message
    #print(len(can_strings), can_strings)
    #can_strings = log.Event.from_bytes(can_strings[0])
    ret.canValid = self.cp.can_valid
    if not lac_log is None:
      ret.torqueRequest = lac_log.output
      ret.lateralControlState.pidState = lac_log

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    #ret.curvatureFactor = self.VM.curvature_factor(ret.vEgo)
    #ret.curvature = ret.curvatureFactor * self.CS.angle_steers * CV.DEG_TO_RAD / self.VM.sR
    ret.yawRate = ret.curvature * ret.vEgo
    #ret.slipFactor = self.VM.sF
    #ret.steerRatio = self.VM.sR
    ret.lateralAccel = self.CS.lateral_accel
    ret.longAccel = self.CS.long_accel
    ret.yawRateCAN = self.CS.yaw_rate
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # gas pedal
    ret.gas = self.CS.car_gas / 256.0
    if not self.CP.enableGasInterceptor:
      ret.gasPressed = self.CS.pedal_gas > 0
    else:
      ret.gasPressed = self.CS.user_gas_pressed

    # brake pedal
    ret.brake = self.CS.user_brake
    ret.brakePressed = self.CS.brake_pressed != 0
    # FIXME: read sendcan for brakelights
    brakelights_threshold = 0.02 if self.CS.CP.carFingerprint == CAR.CIVIC else 0.1
    ret.brakeLights = bool(self.CS.brake_switch or
                           c.actuators.brake > brakelights_threshold)

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate
    ret.steeringTorqueEps = self.CS.steer_rate_motor
    #self.CS.steer_advance = self.AA.get_steer_advance(self.CS.steer_advance, self.CS.steer_rate_motor * DT_CTRL, self.CS.steer_override, self.frame, self.CS.CP)
    #ret.steeringAdvance = float(self.CS.steer_advance)

    # gear shifter lever
    ret.gearShifter = self.CS.gear_shifter

    ret.steeringTorque = self.CS.steer_torque_driver
    ret.steeringPressed = self.CS.steer_override

    # cruise state
    ret.cruiseState.enabled = self.CS.pcm_acc_status != 0
    ret.cruiseState.speed = self.CS.v_cruise_pcm * CV.KPH_TO_MS
    ret.cruiseState.available = bool(self.CS.main_on) and not bool(self.CS.cruise_mode)
    ret.cruiseState.speedOffset = self.CS.cruise_speed_offset
    ret.cruiseState.standstill = False

    #ret.readdistancelines = self.CS.read_distance_lines
    ret.lkMode = self.CS.lkMode

    if not self.cp_cam is None:
      ret.camLeft.full1 = self.CS.cam_left_1['FULL']
      ret.camLeft.full2 = self.CS.cam_left_2['FULL']
      ret.camRight.full1 = self.CS.cam_right_1['FULL']
      ret.camRight.full2 = self.CS.cam_right_2['FULL']
      ret.camFarLeft.full1 = self.CS.cam_far_left_1['FULL']
      ret.camFarLeft.full2 = self.CS.cam_far_left_2['FULL']
      ret.camFarRight.full1 = self.CS.cam_far_right_1['FULL']
      ret.camFarRight.full2 = self.CS.cam_far_right_2['FULL']
      ret.camLeft.parm1 = self.CS.cam_left_1['PARM_1']
      ret.camLeft.parm2 = self.CS.cam_left_1['PARM_2']
      ret.camLeft.parm3 = self.CS.cam_left_1['PARM_3']
      ret.camLeft.parm4 = self.CS.cam_left_1['PARM_4']
      ret.camLeft.parm5 = self.CS.cam_left_1['PARM_5']
      ret.camLeft.parm6 = self.CS.cam_left_2['PARM_6']
      ret.camLeft.parm7 = self.CS.cam_left_2['PARM_7']
      ret.camLeft.parm8 = self.CS.cam_left_2['PARM_8']
      ret.camLeft.parm9 = self.CS.cam_left_2['PARM_9']
      ret.camLeft.parm10 = self.CS.cam_left_2['PARM_10']
      ret.camLeft.parm11 = self.CS.cam_left_2['PARM_11']
      ret.camLeft.parm12 = self.CS.cam_left_2['PARM_12']
      ret.camLeft.parm13 = self.CS.cam_left_2['PARM_13']
      ret.camLeft.dashed = self.CS.cam_left_2['DASHED_LINE']
      ret.camLeft.solid = self.CS.cam_left_2['SOLID_LINE']
      ret.camLeft.frame = self.CS.cam_left_1['FRAME_ID'] + self.CS.cam_left_2['FRAME_ID']
      ret.camRight.parm1 = self.CS.cam_right_1['PARM_1']
      ret.camRight.parm2 = self.CS.cam_right_1['PARM_2']
      ret.camRight.parm3 = self.CS.cam_right_1['PARM_3']
      ret.camRight.parm4 = self.CS.cam_right_1['PARM_4']
      ret.camRight.parm5 = self.CS.cam_right_1['PARM_5']
      ret.camRight.parm6 = self.CS.cam_right_2['PARM_6']
      ret.camRight.parm7 = self.CS.cam_right_2['PARM_7']
      ret.camRight.parm8 = self.CS.cam_right_2['PARM_8']
      ret.camRight.parm9 = self.CS.cam_right_2['PARM_9']
      ret.camRight.parm10 = self.CS.cam_right_2['PARM_10']
      ret.camRight.parm11 = self.CS.cam_right_2['PARM_11']
      ret.camRight.parm12 = self.CS.cam_right_2['PARM_12']
      ret.camRight.parm13 = self.CS.cam_right_2['PARM_13']
      ret.camRight.dashed = self.CS.cam_right_2['DASHED_LINE']
      ret.camRight.solid = self.CS.cam_right_2['SOLID_LINE']
      ret.camRight.frame = self.CS.cam_right_1['FRAME_ID'] + self.CS.cam_right_2['FRAME_ID']
      ret.camFarLeft.parm1 = self.CS.cam_far_left_1['PARM_1']
      ret.camFarLeft.parm2 = self.CS.cam_far_left_1['PARM_2']
      ret.camFarLeft.parm3 = self.CS.cam_far_left_1['PARM_3']
      ret.camFarLeft.parm4 = self.CS.cam_far_left_1['PARM_4']
      ret.camFarLeft.parm5 = self.CS.cam_far_left_1['PARM_5']
      ret.camFarLeft.parm6 = self.CS.cam_far_left_2['PARM_6']
      ret.camFarLeft.parm7 = self.CS.cam_far_left_2['PARM_7']
      ret.camFarLeft.parm8 = self.CS.cam_far_left_2['PARM_8']
      ret.camFarLeft.parm9 = self.CS.cam_far_left_2['PARM_9']
      ret.camFarLeft.parm10 = self.CS.cam_far_left_2['PARM_10']
      ret.camFarLeft.parm11 = self.CS.cam_far_left_2['PARM_11']
      ret.camFarLeft.parm12 = self.CS.cam_far_left_2['PARM_12']
      ret.camFarLeft.parm13 = self.CS.cam_far_left_2['PARM_13']
      ret.camFarLeft.frame = self.CS.cam_far_left_1['FRAME_ID'] + self.CS.cam_far_left_2['FRAME_ID']
      ret.camFarRight.parm1 = self.CS.cam_far_right_1['PARM_1']
      ret.camFarRight.parm2 = self.CS.cam_far_right_1['PARM_2']
      ret.camFarRight.parm3 = self.CS.cam_far_right_1['PARM_3']
      ret.camFarRight.parm4 = self.CS.cam_far_right_1['PARM_4']
      ret.camFarRight.parm5 = self.CS.cam_far_right_1['PARM_5']
      ret.camFarRight.parm6 = self.CS.cam_far_right_2['PARM_6']
      ret.camFarRight.parm7 = self.CS.cam_far_right_2['PARM_7']
      ret.camFarRight.parm8 = self.CS.cam_far_right_2['PARM_8']
      ret.camFarRight.parm9 = self.CS.cam_far_right_2['PARM_9']
      ret.camFarRight.parm10 = self.CS.cam_far_right_2['PARM_10']
      ret.camFarRight.parm11 = self.CS.cam_far_right_2['PARM_11']
      ret.camFarRight.parm12 = self.CS.cam_far_right_2['PARM_12']
      ret.camFarRight.parm13 = self.CS.cam_far_right_2['PARM_13']
      ret.camFarRight.frame = self.CS.cam_far_right_1['FRAME_ID'] + self.CS.cam_far_right_2['FRAME_ID']

      # TODO: Fix these values in the DBC
      ret.camLeft.parm2 = ret.camLeft.parm2 if ret.camLeft.parm2 > -150 else ret.camLeft.parm2 + 1024
      ret.camLeft.parm10 = ret.camLeft.parm10 if ret.camLeft.parm10 > -10 else ret.camLeft.parm10 + 128
      ret.camFarLeft.parm2 = ret.camFarLeft.parm2 if ret.camFarLeft.parm2 > -150 else ret.camFarLeft.parm2 + 1024
      ret.camFarLeft.parm10 = ret.camFarLeft.parm10 if ret.camFarLeft.parm10 > -10 else ret.camFarLeft.parm10 + 128
      ret.camRight.parm2 = ret.camRight.parm2 if ret.camRight.parm2 < 150 else ret.camRight.parm2 - 1024
      ret.camRight.parm10 = ret.camRight.parm10 if ret.camRight.parm10 < 10 else ret.camRight.parm10 - 128
      ret.camFarRight.parm2 = ret.camFarRight.parm2 if ret.camFarRight.parm2 < 150 else ret.camFarRight.parm2 - 1024
      ret.camFarRight.parm10 = ret.camFarRight.parm10 if ret.camFarRight.parm10 < 10 else ret.camFarRight.parm10 - 128

      #if self.frame % 1000 == 0: print(self.CS.cam_left_1['FULL'],self.CS.cam_left_2['FULL'],self.CS.cam_right_1['FULL'],self.CS.cam_right_2['FULL'])
      #print(self.cp_cam.vl["CUR_LANE_LEFT_1"]['FRAME_ID'], )
    
    #if ret.camLeft.parm2 < -100:
    #  ret.camLeft.parm2 += 2048
    #ret.camLeft.parm2 -= 512
    #if ret.camRight.parm2 > 100:
    #  ret.camRight.parm2 -= 2048
    #ret.camRight.parm2 += 512
    #if ret.camFarLeft.parm2 < 0:
    #  ret.camFarLeft.parm2 += 2048
    #ret.camFarLeft.parm2 -= 1024 
    #if ret.camFarRight.parm2 > 0:
    #  ret.camFarRight.parm2 -= 2048
    #ret.camFarRight.parm2 += 1024


    # TODO: button presses
    buttonEvents = []
    ret.econMode = bool(self.CS.econ_on)
    ret.leftBlinker = bool(self.CS.left_blinker_on)
    ret.rightBlinker = bool(self.CS.right_blinker_on)
    ret.blinkers = bool(self.CS.blinker_on)

    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      be.pressed = self.CS.left_blinker_on != 0
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      be.pressed = self.CS.right_blinker_on != 0
      buttonEvents.append(be)

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'unknown'
      if self.CS.cruise_buttons != 0:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        be.type = 'accelCruise'
      elif but == CruiseButtons.DECEL_SET:
        be.type = 'decelCruise'
      elif but == CruiseButtons.CANCEL:
        be.type = 'cancel'
      elif but == CruiseButtons.MAIN:
        be.type = 'altButton3'
      buttonEvents.append(be)

    if self.CS.cruise_setting != self.CS.prev_cruise_setting:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'unknown'
      if self.CS.cruise_setting != 0:
        be.pressed = True
        but = self.CS.cruise_setting
      else:
        be.pressed = False
        but = self.CS.prev_cruise_setting
      if but == 1:
        be.type = 'altButton1'
      # TODO: more buttons?
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents

    # events
    events = []

    # wait 1.0s before throwing the alert to avoid it popping when you turn off the car
    if not self.cp_cam is None and self.cp_cam.can_invalid_cnt >= 100 and self.CS.CP.carFingerprint not in HONDA_BOSCH and self.CP.enableCamera:
      events.append(create_event('invalidGiraffeHonda', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))

    if not self.CS.lkMode:
      events.append(create_event('manualSteeringRequired', [ET.WARNING]))
    elif self.CS.steer_error:
      events.append(create_event('steerUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))
    elif self.CS.steer_warning:
      events.append(create_event('steerTempUnavailable', [ET.WARNING]))
    if self.CS.brake_error:
      events.append(create_event('brakeUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))
    if not ret.gearShifter in ['drive', 'sport', 'low']:
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if self.CS.esp_disabled:
      events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if not self.CS.main_on or self.CS.cruise_mode:
      events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if ret.gearShifter == 'reverse':
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    if self.CS.brake_hold and self.CS.CP.carFingerprint not in HONDA_BOSCH:
      events.append(create_event('brakeHold', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if self.CS.park_brake:
      events.append(create_event('parkBrake', [ET.NO_ENTRY, ET.USER_DISABLE]))

    if self.CP.enableCruise and ret.vEgo < self.CP.minEnableSpeed:
      events.append(create_event('speedTooLow', [ET.NO_ENTRY]))

    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    if (ret.gasPressed and not self.gas_pressed_prev and not self.bosch_honda) or \
       (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    if ret.gasPressed and not self.bosch_honda:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))

    # it can happen that car cruise disables while comma system is enabled: need to
    # keep braking if needed or if the speed is very low
    if self.CP.enableCruise and not ret.cruiseState.enabled and c.actuators.brake <= 0.:
      # non loud alert if cruise disbales below 25mph as expected (+ a little margin)
      if ret.vEgo < self.CP.minEnableSpeed + 2.:
      #  events.append(create_event('speedTooLow', [ET.IMMEDIATE_DISABLE]))
      #else:
        events.append(create_event("cruiseDisabled", [ET.IMMEDIATE_DISABLE]))   # send loud alert if slow and cruise disables during braking

    if self.CS.CP.minEnableSpeed > 0 and ret.vEgo < 0.001:
      events.append(create_event('manualRestart', [ET.WARNING]))

    cur_time = self.frame * DT_CTRL
    enable_pressed = False
    # handle button presses
    for b in ret.buttonEvents:

      # do enable on both accel and decel buttons
      if b.type in ["accelCruise", "decelCruise"] and not b.pressed:
        self.last_enable_pressed = cur_time
        enable_pressed = True

      # do disable on button down
      if b.type == "cancel" and b.pressed:
        events.append(create_event('buttonCancel', [ET.USER_DISABLE]))

    if self.CP.enableCruise:
      # KEEP THIS EVENT LAST! send enable event if button is pressed and there are
      # NO_ENTRY events, so controlsd will display alerts. Also not send enable events
      # too close in time, so a no_entry will not be followed by another one.
      # TODO: button press should be the only thing that triggers enble
      if ((cur_time - self.last_enable_pressed) < 0.2 and
          (cur_time - self.last_enable_sent) > 0.2 and
          ret.cruiseState.enabled) or \
         (enable_pressed and get_events(events, [ET.NO_ENTRY])):
        events.append(create_event('buttonEnable', [ET.ENABLE]))
        self.last_enable_sent = cur_time
    elif enable_pressed:
      events.append(create_event('buttonEnable', [ET.ENABLE]))

    ret.events = events

    # update previous brake/gas pressed
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed


    # cast to reader so it can't be modified
    return ret

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):
    if c.hudControl.speedVisible:
      hud_v_cruise = c.hudControl.setSpeed * CV.MS_TO_KPH
    else:
      hud_v_cruise = 255

    hud_alert = VISUAL_HUD[c.hudControl.visualAlert.raw]
    snd_beep, snd_chime = AUDIO_HUD[c.hudControl.audibleAlert.raw]

    pcm_accel = int(clip(c.cruiseControl.accelOverride, 0, 1) * 0xc6)

    time.sleep(0.00001)
    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators,
                               c.cruiseControl.speedOverride,
                               c.cruiseControl.override,
                               c.cruiseControl.cancel,
                               pcm_accel,
                               hud_v_cruise,
                               c.hudControl.lanesVisible,
                               hud_show_car=c.hudControl.leadVisible,
                               hud_alert=hud_alert,
                               snd_beep=snd_beep,
                               snd_chime=snd_chime)

    self.frame += 1
    return can_sends
