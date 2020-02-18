from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import gernterp, interp, clip
import numpy as np
from cereal import log
from common.realtime import sec_since_boot
from common.params import Params
from numpy import array

import json

class LatControlPID(object):
  def __init__(self, CP):
    self.kegman = kegman_conf(CP)
    self.frame = 0
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0)
    self.angle_steers_des = 0.
    self.polyReact = 1.  #max(0.0, CP.lateralTuning.pid.polyReactTime + CP.lateralTuning.pid.polyDampTime)
    self.poly_smoothing = max(1.0, CP.lateralTuning.pid.polyDampTime * 100.)
    self.poly_scale = CP.lateralTuning.pid.polyScale
    self.poly_factor = CP.lateralTuning.pid.polyFactor
    self.poly_scale = CP.lateralTuning.pid.polyScale
    self.path_error_comp = 0.0
    self.last_path_error = 0.0
    self.cur_poly_scale = 0.0
    self.p_poly = [0., 0., 0., 0.]
    self.s_poly = [0., 0., 0., 0.]
    self.p_prob = 0.
    self.damp_angle_steers = 0.
    self.damp_angle_rate = 0.
    self.damp_time = 0.1
    self.react_mpc = 0.0
    self.damp_mpc = 0.25
    self.angle_ff_ratio = 0.0
    self.gernbySteer = True
    self.standard_ff_ratio = 0.0
    self.angle_ff_gain = 1.0
    self.rate_ff_gain = CP.lateralTuning.pid.rateFFGain
    self.angle_ff_bp = [[0.5, 5.0],[0.0, 1.0]]
    self.steer_p_scale = CP.lateralTuning.pid.steerPscale
    self.calculate_rate = True
    self.prev_angle_steers = 0.0
    self.rough_steers_rate = 0.0
    self.steer_counter = 1
    self.lane_change_adjustment = 0.0
    self.lane_changing = 0.0
    self.starting_angle = 0.0
    self.half_lane_width = 0.0
    self.steer_counter_prev = 1
    self.params = Params()
    self.prev_override = False
    self.driver_assist_offset = 0.0
    self.driver_assist_hold = False
    self.angle_bias = 0.
    self.previous_integral = 0.0
    self.damp_angle_steers= 0.0
    self.damp_rate_steers_des = 0.0
    self.damp_angle_steers_des = 0.0
    self.old_plan_count = 0
    self.last_plan_time = 0
    self.angle_bias = 0.0
    self.path_age = 0
    self.lane_compensation = 0.
    self.future_centers = 0.
    self.plan_index = 0
    self.avg_plan_age = 0.
    self.lane_error = 0.
    self.min_index = 0
    self.max_index = 0
    #self.poly_range = np.concatenate((np.zeros((3)),np.ones((7)),np.zeros((4))))
    self.poly_range = np.concatenate((np.zeros((4)),np.arange((10.))))
    self.path_index = np.arrange((14))

    try:
      lateral_params = self.params.get("LateralParams")
      lateral_params = json.loads(lateral_params)
      self.angle_ff_gain = max(1.0, float(lateral_params['angle_ff_gain']))
    except:
      self.angle_ff_gain = 1.0

  def live_tune(self, CP):
    self.frame += 1
    if self.frame % 3600 == 0:
      self.params.put("LateralParams", json.dumps({'angle_ff_gain': self.angle_ff_gain}))
    if self.frame % 300 == 0:
      #print("plan age = %f" % self.plan_age)
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      #with open('~/openpilot/selfdrive/gernby.json', 'r') as f:
      #  kegman = json.load(f)
      try:
        self.kegman = kegman_conf()  #.read_config()
        #print(self.kegman.conf)
        self.pid._k_i = ([0.], [float(self.kegman.conf['Ki'])])
        self.pid._k_p = ([0.], [float(self.kegman.conf['Kp'])])
        self.pid.k_f = (float(self.kegman.conf['Kf']))
        self.damp_time = (float(self.kegman.conf['dampTime']))
        self.react_mpc = (float(self.kegman.conf['reactMPC']))
        self.damp_mpc = (float(self.kegman.conf['dampMPC']))
        self.polyReact =  max(0.0, float(self.kegman.conf['polyReact']) * 0.1)
        self.poly_smoothing = max(1.0, float(self.kegman.conf['polyDamp']) * 100.)
        self.poly_factor = max(0.0, float(self.kegman.conf['polyFactor']) * 0.001)
      except:
        print("   Kegman error")

  def reset(self):
    self.pid.reset()

  def adjust_angle_gain(self):
    if (self.pid.f > 0) == (self.pid.i > 0) and abs(self.pid.i) >= abs(self.previous_integral):
      if not abs(self.pid.f + self.pid.i) > 1: self.angle_ff_gain *= 1.0001
    elif self.angle_ff_gain > 1.0:
      self.angle_ff_gain *= 0.9999
    self.previous_integral = self.pid.i

  def update(self, active, v_ego, angle_steers, angle_steers_advance, angle_steers_rate, steer_override, blinkers_on, CP, VM, path_plan, live_mpc, logMonoTime):

    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steerAngle = float(angle_steers)

    if logMonoTime['pathPlan'] != self.last_plan_time:
      if self.path_age > 0.23: self.old_plan_count += 1
      self.last_plan_time = logMonoTime['pathPlan']
      path_age = sec_since_boot() - logMonoTime['pathPlan'] * 1e-9
      self.avg_plan_age += 0.01 * (path_age - self.avg_plan_age)
      self.plan_index = max(0, min(24, int(100 * (self.react_mpc + path_age))))
      #self.lane_error = self.poly_factor * path_plan.cPoly[0] + self.polyReact*(path_plan.cPoly[len(path_plan.cPoly)-1]-path_plan.cPoly[3])
      #if path_plan.cPoly[0] > 0:
      #  self.lane_error += self.poly_factor
      #elif path_plan.cPoly[0] < 0:
      #  self.lane_error -= self.poly_factor
      self.projected_lane_error = self.polyReact * 0.001 * sum(self.poly_range * self.poly_range * path_plan.cPoly)   #-path_plan.cPoly[3])
    else:
      self.plan_index += 1

    self.min_index = min(self.min_index, self.plan_index)
    self.max_index = max(self.max_index, self.plan_index)

    if self.frame % 300 == 0:
      #print(path_plan.mpcAngles)
      #print(path_plan.mpcRates)
      print("old plans:  %d  avg plan age:  %0.3f   min index:  %d  max_index:  %d  angles:  %d poly_react:  %d" % (self.old_plan_count, self.avg_plan_age, self.min_index, self.max_index, len(path_plan.cPoly), self.polyReact))
      self.min_index = 100
      self.max_index = 0

    self.frame += 1
    self.live_tune(CP)

    if (v_ego < 0.3 or not active):  # or self.plan_index > len(path_plan.mpcAngles)):
      output_steer = 0.0
      self.previous_integral = 0.0
      self.damp_angle_steers= 0.0
      self.damp_rate_steers_des = 0.0
      self.damp_angle_steers_des = 0.0
      self.angle_bias = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      try:
        self.angle_steers_des = path_plan.mpcAngles[self.plan_index//7]
        #self.angle_steers_des += ((path_plan.mpcAngles[self.plan_index//7] - path_plan.mpcAngles[(self.plan_index//7)-1])/7) * self.plan_index % 7
        #self.damp_angle_steers_des = self.angle_steers_des
        #print(" %0.1f  %0.1f   %0.1f   %0.1f  %d   %d"  % (self.damp_rate_steers_des, self.angle_steers_des, self.damp_angle_steers_des, path_plan.mpcAngles[self.plan_index//7], self.plan_index,self.plan_index//7 ))

        #self.angle_steers_des = path_plan.mpcAngles[self.plan_index]
        self.damp_angle_steers_des += (self.angle_steers_des - self.damp_angle_steers_des) / max(1.0, self.damp_mpc * 100.)
        #self.damp_rate_steers_des += ((path_plan.mpcAngles[self.plan_index//7] - path_plan.mpcAngles[(self.plan_index//7)-1]) - self.damp_rate_steers_des) / max(1.0, self.damp_mpc * 100.)
      except:
        pass
      self.damp_angle_steers = angle_steers  # += (angle_steers + angle_steers_rate * self.damp_time - self.damp_angle_steers) / max(1.0, self.damp_time * 100.)
      self.damp_angle_rate = angle_steers_rate  #+= (angle_steers_rate - self.damp_angle_rate) / max(1.0, self.damp_time * 100.)

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      angle_feedforward = float(self.damp_angle_steers_des - path_plan.angleOffset)
      self.angle_ff_ratio = float(gernterp(abs(angle_feedforward), self.angle_ff_bp[0], self.angle_ff_bp[1]))
      rate_feedforward = (1.0 - self.angle_ff_ratio) * self.rate_ff_gain * self.damp_rate_steers_des
      steer_feedforward = float(v_ego)**2 * (rate_feedforward + angle_feedforward * self.angle_ff_ratio * self.angle_ff_gain)

      #self.lane_compensation += self.poly_factor * path_plan.cPoly[0] * 1 if self.lane_compensation * path_plan.cPoly[0] > 1 else 10
      self.path_error_comp += (v_ego * self.projected_lane_error * self.angle_ff_gain - self.path_error_comp) / self.poly_smoothing
      #self.path_error_comp += ((self.projected_lane_error + self.lane_compensation) - self.path_error_comp) / self.poly_smoothing

      if self.gernbySteer and not steer_override and v_ego > 10.0:
        if abs(angle_steers) > (self.angle_ff_bp[0][1] / 2.0):
          self.adjust_angle_gain()
        else:
          self.previous_integral = self.pid.i
      p_scale = 1.0

      deadzone = 0.0
      driver_opposing_i = steer_override
      #output_steer = self.pid.update(self.angle_steers_des + self.path_error_comp, self.damp_angle_steers, check_saturation=(v_ego > 10), override=driver_opposing_i,
      #                              feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone, p_scale=p_scale)
      print("%0.3f  %0.3f   %0.3f  " % (self.path_error_comp, self.angle_steers_des, self.damp_angle_steers))

      output_steer = self.pid.update(self.damp_angle_steers_des, self.damp_angle_steers, check_saturation=(v_ego > 10), override=driver_opposing_i,
                                    add_error=float(self.path_error_comp), feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone, p_scale=p_scale)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_steer)
      pid_log.p2 = float(self.pid.p2)   #float(self.path_error_comp) * float(self.pid._k_p[1][0])
      pid_log.saturated = bool(self.pid.saturated)
      pid_log.angleFFRatio = self.angle_ff_ratio
      pid_log.angleBias = self.angle_bias

      #if self.frame % 100 == 0:
      #  print(path_plan)
    self.prev_angle_steers = angle_steers
    self.prev_override = steer_override
    self.sat_flag = self.pid.saturated
    return output_steer, float(self.angle_steers_des), pid_log
    #except:
    #  return 0, 0, pid_log
