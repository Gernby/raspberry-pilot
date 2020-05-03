from selfdrive.controls.lib.pid import PIController
from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import gernterp, interp, clip
import numpy as np
import time
from cereal import car
from common.params import Params
from numpy import array

import json

class LatControlPID(object):
  def __init__(self, CP):
    self.kegman = kegman_conf(CP)
    self.frame = 0
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf)
    self.angle_steers_des = 0.
    self.polyReact = 1.  #max(0.0, CP.lateralTuning.pid.polyReactTime + CP.lateralTuning.pid.polyDampTime)
    self.poly_smoothing = max(1.0, CP.lateralTuning.pid.polyDampTime * 100.)
    self.poly_factor = CP.lateralTuning.pid.polyFactor
    self.poly_scale = CP.lateralTuning.pid.polyScale
    self.path_error_comp = 0.0
    self.damp_angle_steers = 0.
    self.damp_angle_rate = 0.
    self.damp_time = 0.1
    self.react_mpc = 0.0
    self.damp_mpc = 0.25
    self.angle_ff_ratio = 0.0
    self.angle_ff_gain = 1.0
    self.rate_ff_gain = CP.lateralTuning.pid.rateFFGain
    self.angle_ff_bp = [[0.5, 5.0],[0.0, 1.0]]
    self.params = Params()
    self.lateral_offset = 0.0
    self.previous_integral = 0.0
    self.damp_angle_steers= 0.0
    self.damp_rate_steers_des = 0.0
    self.damp_angle_steers_des = 0.0
    self.old_plan_count = 0
    self.last_plan_time = 0
    self.angle_index = 0.
    self.avg_plan_age = 0.
    self.min_index = 0
    self.max_index = 0
    self.c_prob = 0.
    self.projected_lane_error = 0.
    self.prev_projected_lane_error = 0.
    self.path_index = None #np.arange((30.))*100.0/15.0

    try:
      lateral_params = self.params.get("LateralGain")
      lateral_params = json.loads(lateral_params)
      self.angle_ff_gain = max(1.0, float(lateral_params['angle_ff_gain']))
    except:
      self.angle_ff_gain = 1.0

  def live_tune(self, CP):
    if self.frame % 3600 == 0:
      self.params.put("LateralGain", json.dumps({'angle_ff_gain': self.angle_ff_gain}))
    if self.frame % 300 == 0:
      try:
        self.kegman = kegman_conf()  #.read_config()
        self.pid._k_i = ([0.], [float(self.kegman.conf['Ki'])])
        self.pid._k_p = ([0.], [float(self.kegman.conf['Kp'])])
        self.pid.k_f = (float(self.kegman.conf['Kf']))
        self.damp_time = (float(self.kegman.conf['dampTime']))
        self.react_mpc = (float(self.kegman.conf['reactMPC']))
        self.damp_mpc = (float(self.kegman.conf['dampMPC']))
        self.polyReact =  1.0 + np.arange(15) * float(self.kegman.conf['polyReact']) / 15
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

  def update(self, active, v_ego, angle_steers, angle_steers_rate, steer_override, CP, path_plan, canTime):
    pid_log = car.CarState.LateralPIDState.new_message()
    if path_plan.canTime != self.last_plan_time and len(path_plan.mpcAngles) > 1:
      path_age = (canTime - path_plan.canTime) * 1e-3
      if path_age > 0.23: self.old_plan_count += 1
      if self.path_index is None:
        self.avg_plan_age = path_age
        self.path_index = np.arange((len(path_plan.mpcAngles)))*100.0/15.0
      self.last_plan_time = path_plan.canTime
      self.avg_plan_age += 0.01 * (path_age - self.avg_plan_age)

      self.c_prob = path_plan.cProb
      self.projected_lane_error = (self.c_prob / max(1, v_ego)) * self.poly_factor * sum(np.array(path_plan.cPoly) * self.polyReact)
      if abs(self.projected_lane_error) < abs(self.prev_projected_lane_error) and (self.projected_lane_error > 0) == (self.prev_projected_lane_error > 0):
        self.projected_lane_error *= gernterp(angle_steers + path_plan.angleOffset, [0, 4], [0.1, 1.0])
      self.prev_projected_lane_error = self.projected_lane_error
      self.angle_index = max(0., 100. * (self.react_mpc + path_age))
    else:
      self.angle_index += 1.0

    self.min_index = min(self.min_index, self.angle_index)
    self.max_index = max(self.max_index, self.angle_index)

    if self.frame % 300 == 0 and self.frame > 0:
      print("old plans:  %d  avg plan age:  %0.3f   min index:  %d  max_index:  %d" % (self.old_plan_count, self.avg_plan_age, self.min_index, self.max_index))
      self.min_index = 100
      self.max_index = 0

    self.frame += 1
    self.live_tune(CP)

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.previous_integral = 0.0
      self.previous_lane_error = 0.0
      self.path_error_comp = 0.0
      self.damp_angle_steers= 0.0
      self.damp_rate_steers_des = 0.0
      self.damp_angle_steers_des = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      try:
        pid_log.active = True
        self.path_error_comp += (self.projected_lane_error - self.path_error_comp) / self.poly_smoothing
        self.damp_angle_steers += (angle_steers + angle_steers_rate * self.damp_time - self.damp_angle_steers) / max(1.0, 1 + self.damp_time * 100.)
        self.damp_angle_rate += (angle_steers_rate - self.damp_angle_rate) / max(1.0, self.damp_time * 100.)
        self.angle_steers_des = interp(self.angle_index, self.path_index, path_plan.mpcAngles) + float(self.path_error_comp)
        self.damp_angle_steers_des += (self.angle_steers_des - self.damp_angle_steers_des) / max(1.0, self.damp_mpc * 100.)
        self.damp_rate_steers_des += ((path_plan.mpcAngles[4] - path_plan.mpcAngles[3]) - self.damp_rate_steers_des) / max(1.0, self.damp_mpc * 100.)
      except:
        print("  angle error!")
        pass

      angle_feedforward = float(self.damp_angle_steers_des + path_plan.angleOffset)
      self.angle_ff_ratio = float(gernterp(abs(angle_feedforward), self.angle_ff_bp[0], self.angle_ff_bp[1]))
      rate_feedforward = (1.0 - self.angle_ff_ratio) * self.rate_ff_gain * self.damp_rate_steers_des
      steer_feedforward = float(v_ego)**2 * (rate_feedforward + angle_feedforward * self.angle_ff_ratio * self.angle_ff_gain)

      if not steer_override and v_ego > 10.0:
        if abs(angle_steers) > (self.angle_ff_bp[0][1] / 2.0):
          self.adjust_angle_gain()
        else:
          self.previous_integral = self.pid.i

      deadzone = 0.0 

      if path_plan.cProb == 0 or (angle_feedforward > 0) == (self.pid.p > 0) or (path_plan.cPoly[-1] > 0) == (self.pid.p > 0):
        p_scale = 1.0 
      else:
        p_scale = max(0.2, min(1.0, 1 / abs(angle_feedforward)))

      requested_angle = max(self.damp_angle_steers_des - 0.05, min(self.damp_angle_steers_des + 0.05, path_plan.angleSteers))
      output_steer = self.pid.update(requested_angle, self.damp_angle_steers, check_saturation=(v_ego > 10), override=steer_override, p_scale=p_scale,
                                    add_error=0, feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)

    pid_log.p = float(self.pid.p)
    pid_log.i = float(self.pid.i)
    pid_log.f = float(self.pid.f)
    pid_log.output = float(output_steer)
    pid_log.p2 = float(self.path_error_comp) * float(self.pid._k_p[1][0])
    pid_log.saturated = bool(self.pid.saturated)
    pid_log.angleFFRatio = self.angle_ff_ratio
    pid_log.steerAngle = float(self.damp_angle_steers)
    pid_log.steerAngleDes = float(self.damp_angle_steers_des)

    self.sat_flag = self.pid.saturated

    return output_steer, float(self.angle_steers_des), pid_log
