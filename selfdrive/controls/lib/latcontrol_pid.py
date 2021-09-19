from selfdrive.controls.lib.pid import PIController
from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import gernterp, interp, clip
from common.profiler import Profiler
import numpy as np
import os
import time
from cereal import car

from common.params import Params, put_nonblocking
from numpy import array

import json

STEER_HYSTERESIS = [5, 90]

class LatControlPID(object):
  def __init__(self, CP):
    self.kegman = kegman_conf(CP)
    self.kegtime_prev = 0
    self.profiler = Profiler(False, 'LaC')
    self.frame = 0
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf)
    self.angle_steers_des = 0.
    self.polyReact = 1.
    self.poly_damp = 0
    self.poly_factor = CP.lateralTuning.pid.polyFactor
    self.poly_scale = CP.lateralTuning.pid.polyScale
    self.damp_angle_steers = 0.
    self.damp_angle_rate = 0.
    self.damp_steer = 0.1
    self.react_steer = 0.01
    self.react_mpc = 0.0
    self.damp_mpc = 0.25
    self.angle_ff_ratio = 0.0
    self.angle_ff_gain = 0.8
    self.rate_ff_gain = CP.lateralTuning.pid.rateFFGain
    self.angle_ff_bp = [[0.5, 5.0],[0.0, 1.0]]
    self.lateral_offset = 0.0
    self.previous_integral = 0.0
    self.damp_angle_steers= 0.0
    self.damp_rate_steers_des = 0.0
    self.damp_angle_steers_des = 0.0
    self.limited_damp_angle_steers_des = 0.0
    self.old_plan_count = 0
    self.last_plan_time = 0.
    self.last_plan_recv = 0.
    self.lane_change_adjustment = 1.0
    self.angle_index = 0.
    self.avg_plan_age = 0.
    self.avg_plan_freq = 30.0
    self.min_index = 0
    self.max_index = 0
    self.wiggle_angle = 0.2
    self.prev_angle_steers = 0.
    self.c_prob = 0.
    self.deadzone = 0.
    self.use_deadzone = False
    self.starting_angle = 0.
    self.projected_lane_error = 0.
    self.prev_projected_lane_error = 0.
    self.path_index = None #np.arange((30.))*100.0/15.0
    self.accel_limit = 0.05      # 100x degrees/sec**2
    self.angle_rate_des = 0.0    # degrees/sec, rate dynamically limited by accel_limit
    self.fast_angles = [[]]
    self.center_angles = []
    self.last_model_index = 0
    self.live_tune(CP)
    self.react_index = 0.0
    self.next_params_put = 36000
    self.zero_poly_crossed = 0
    self.zero_steer_crossed = 0
    self.lane_changing = 0
    self.output_steer = 0.
    self.hysteresis_state = 1
    self.cPoints = np.arange(15)
    self.use_poly_angle = True

    try:
      self.params = Params()
      lateral_params = self.params.get("LateralGain")
      lateral_params = json.loads(lateral_params)
      try:
        self.angle_ff_gain = [max(0.1, float(lateral_params['angle_ff_gain'][0])), max(0.1, float(lateral_params['angle_ff_gain'][1]))]
        self.angle_ff_offset = float(lateral_params['angle_ff_offset'])
      except:
        self.angle_ff_gain = [max(0.1, float(lateral_params['angle_ff_gain'])), max(0.1, float(lateral_params['angle_ff_gain']))]
        self.angle_ff_offset = 0.
    except:
      self.params.put("LateralGain", json.dumps({'angle_ff_gain': self.angle_ff_gain}))
      self.angle_ff_gain = [1.0, 1.0]
      self.angle_ff_offset = 0.

  def live_tune(self, CP):
    if self.frame % 300 == 0:
      (mode, ino, dev, nlink, uid, gid, size, atime, mtime, self.kegtime) = os.stat(os.path.expanduser('~/kegman.json'))
      if self.kegtime != self.kegtime_prev:
        try:
          time.sleep(0.0001)
          self.kegman = kegman_conf() 
        except:
          print("   Kegman error")
        self.pid._k_i = ([0.], [float(self.kegman.conf['Ki'])])
        self.pid._k_p = ([0.], [float(self.kegman.conf['Kp'])])
        self.pid.k_f = (float(self.kegman.conf['Kf']))
        self.damp_steer = (float(self.kegman.conf['dampSteer']))
        self.react_steer = (float(self.kegman.conf['reactSteer']))
        self.react_mpc = (float(self.kegman.conf['reactMPC']))
        self.damp_mpc = (float(self.kegman.conf['dampMPC']))
        self.deadzone = float(self.kegman.conf['deadzone'])
        self.rate_ff_gain = float(self.kegman.conf['rateFFGain'])
        self.wiggle_angle = float(self.kegman.conf['wiggleAngle'])
        self.accel_limit = (float(self.kegman.conf['accelLimit']))
        self.polyReact = min(11, max(0, int(10 * float(self.kegman.conf['polyReact']))))
        self.poly_damp = min(1, max(0, float(self.kegman.conf['polyDamp'])))
        self.poly_factor = max(0.0, float(self.kegman.conf['polyFactor']) * 0.001)
        self.require_blinker = bool(int(self.kegman.conf['requireBlinker']))
        self.require_nudge = bool(int(self.kegman.conf['requireNudge']))
        self.use_poly_angle = bool(int(self.kegman.conf['usePolyAngle']))
        self.react_center = [max(0, float(self.kegman.conf['reactCenter0'])),max(0, float(self.kegman.conf['reactCenter1'])),max(0, float(self.kegman.conf['reactCenter2'])), 0]
        self.kegtime_prev = self.kegtime

  def update_lane_state(self, angle_steers, driver_opposing_lane, blinker_on, path_plan):
    if self.require_nudge:
      if self.lane_changing > 0.0: # and path_plan.cProb > 0:
        self.lane_changing += 0.01  # max(self.lane_changing + 0.01, 0.005 * abs(path_plan.lPoly[5] + path_plan.rPoly[5]))
        if self.lane_changing > 2.75 or (not blinker_on and self.lane_changing < 1.0 and abs(self.cPoints[5]) < 100 and min(abs(self.starting_angle - angle_steers), abs(self.angle_steers_des - angle_steers)) < 1.5 and self.cPoints[14] * self.cPoints[0] > 0):
          self.lane_changing = 0.0
          self.stage = "4"
        elif 2.25 <= self.lane_changing < 2.5 and self.cPoints[14] * self.cPoints[4] > 0:   # abs(path_plan.lPoly[5] + path_plan.rPoly[5]) < abs(self.cPoints[5]):
          self.lane_changing = 2.5
          self.stage = "3"
        elif 2.0 <= self.lane_changing < 2.25 and self.cPoints[14] * self.cPoints[9] > 0:      # (path_plan.lPoly[5] + path_plan.rPoly[5]) * self.cPoints[0] < 0:
          self.lane_changing = 2.25
          self.stage = "2"
        elif self.lane_changing < 2.0 and self.cPoints[14] * self.cPoints[0] < 0:     #path_plan.laneWidth < 1.2 * abs(path_plan.lPoly[5] + path_plan.rPoly[5]):
          self.lane_changing = 2.0
          self.stage = "1"
        elif self.lane_changing < 1.0 and abs(self.cPoints[14]) > abs(self.cPoints[7]):     #path_plan.laneWidth < 1.2 * abs(path_plan.lPoly[5] + path_plan.rPoly[5]):
          self.lane_changing = 0.98
          self.stage = "0"
        #else:
        #self.lane_changing = max(self.lane_changing + 0.01, 0.005 * abs(path_plan.lPoly[5] + path_plan.rPoly[5]))
        #if blinker_on:
        #  self.lane_change_adjustment = 0.0
        #else:
        self.lane_change_adjustment = interp(self.lane_changing, [0.0, 1.0, 2.0, 2.25, 2.5, 2.75], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #print("%0.2f lane_changing  %0.2f adjustment  %0.2f p_poly   %0.2f avg_poly   stage = %s    blinker %d   opposing %d  width_1  %0.2f  width_2  %0.2f  center_1  %0.2f  center_2  %0.2f" % (self.lane_changing, self.lane_change_adjustment, self.cPoints[5], path_plan.lPoly[5] + path_plan.rPoly[5], self.stage, blinker_on, driver_opposing_lane, path_plan.laneWidth, 0.6 * abs(path_plan.lPoly[5] - path_plan.rPoly[5]), self.cPoints[0], path_plan.lPoly[5] + path_plan.rPoly[5]))
      elif (blinker_on or not self.require_blinker) and driver_opposing_lane and path_plan.rProb > 0 and path_plan.lProb > 0 and (abs(self.cPoints[-1]) > 100 or min(abs(self.starting_angle - angle_steers), abs(self.angle_steers_des - angle_steers)) > 1.0): # and self.cPoints[14] * self.cPoints[0] > 0:
        print('starting lane change @ %0.2f' % self.lane_changing)
        self.lane_changing = 0.01 
        self.lane_change_adjustment = 1.0
      else:
        if self.lane_changing != 0: print('terminating lane change @ %0.2f' % self.lane_changing)
        self.lane_changing = 0
        self.stage = "0"
        self.starting_angle = angle_steers
        self.lane_change_adjustment = 1.0
    elif blinker_on:
      self.lane_change_adjustment = 0.0
    else:
      self.lane_change_adjustment = 1.0
      
  def reset(self):
    self.pid.reset()

  def adjust_angle_gain(self, ff_index):
    if (self.pid.f > 0) == (self.pid.i > 0) and abs(self.pid.i) >= abs(self.previous_integral):
      if not abs(self.pid.f + self.pid.i) > 1: self.angle_ff_gain[ff_index] *= 1.0001
    elif self.angle_ff_gain[ff_index] > 1.0:
      self.angle_ff_gain[ff_index] *= 0.9999
    if self.angle_ff_gain[0] > self.angle_ff_gain[1]:
      self.angle_ff_offset -= 0.00001
    else:
      self.angle_ff_offset += 0.00001
    self.previous_integral = self.pid.i

  def update(self, active, cruise_enabled, v_ego, angle_steers, angle_steers_rate, steer_override, CP, path_plan, canTime, blinker_on):
    self.profiler.checkpoint('controlsd')
    pid_log = car.CarState.LateralPIDState.new_message()
    cur_time = time.time()
    path_age = (cur_time - path_plan.sysTime * 1e-3)
    if (angle_steers - path_plan.angleOffset - self.angle_ff_offset >= 0) == (self.prev_angle_steers - path_plan.angleOffset - self.angle_ff_offset < 0):
      self.zero_steer_crossed = cur_time
    driver_opposing = steer_override and (angle_steers - self.prev_angle_steers) * self.output_steer < 0
    self.prev_angle_steers = angle_steers

    if (path_plan.canTime != self.last_plan_time or path_plan.modelIndex != self.last_model_index) and len(path_plan.fastAngles) > 1:
      if path_age > 0.23: self.old_plan_count += 1
      if self.path_index is None:
        self.avg_plan_age = path_age
        self.path_index = np.arange((len(path_plan.fastAngles)))*100.0/15.0
      self.avg_plan_freq += 0.01 * (1 / (cur_time - self.last_plan_recv) - self.avg_plan_freq)
      self.last_plan_recv = cur_time
      self.last_model_index = path_plan.modelIndex
      self.last_plan_time = path_plan.canTime
      self.avg_plan_age += 0.01 * (path_age - self.avg_plan_age)
      self.c_prob = path_plan.cProb
      self.fast_angles = np.array(path_plan.fastAngles)
      self.p_poly = np.poly1d(path_plan.pPoly)
      self.cPoints = (np.polyval(path_plan.dPoly, np.arange(8))) * 1000

      self.projected_lane_error = float(min(0.75, max(-0.75, self.c_prob * self.poly_factor * sum(np.array(self.cPoints)))))
      if np.sign(self.projected_lane_error) != np.sign(self.prev_projected_lane_error):
        self.zero_poly_crossed = cur_time

      if v_ego > 10 and self.c_prob > 0:
        if driver_opposing:
          self.zero_poly_crossed = 0
          self.zero_steer_crossed = 0
          self.use_deadzone = False
          self.projected_lane_error = 0.
        elif np.sign(self.projected_lane_error) == np.sign(self.prev_projected_lane_error) and \
           (abs(self.projected_lane_error) < abs(self.prev_projected_lane_error) or \
            np.sign(self.cPoints[7]) == -np.sign(self.cPoints[-1])) and \
           (np.sign(self.output_steer) == -np.sign(self.fast_angles[7,-1] - path_plan.angleOffset - self.angle_ff_offset) or \
            np.sign(self.output_steer) == -np.sign(self.fast_angles[7,-1] - self.fast_angles[0,-1])):
          self.zero_poly_crossed = max(cur_time - 4, self.zero_poly_crossed)
          self.zero_steer_crossed = max(cur_time - 4, self.zero_steer_crossed)
          self.use_deadzone = False
        else:
          self.use_deadzone = True
      self.prev_projected_lane_error = self.projected_lane_error

      self.center_angles.append(float(self.projected_lane_error))
      if len(self.center_angles) > 15: self.center_angles.pop(0)
      if cur_time - max(self.zero_poly_crossed, self.zero_steer_crossed) <= 4 and abs(self.damp_angle_steers - path_plan.angleOffset - self.angle_ff_offset) < 2:
        self.projected_lane_error -= (float(self.c_prob * self.poly_damp * self.center_angles[0]))
      self.profiler.checkpoint('path_plan')

    if path_plan.paramsValid: self.angle_index = max(0., 100. * (self.react_mpc + path_age))
    self.min_index = min(self.min_index, self.angle_index)
    self.max_index = max(self.max_index, self.angle_index)

    if self.frame % 300 == 0 and self.frame > 0:
      print("old plans:  %d  avg plan age:  %0.3f   avg plan freq:  %0.1f   min index:  %d  max_index:  %d   angle_ff_offset:  %0.5f" % (self.old_plan_count, self.avg_plan_age, self.avg_plan_freq, self.min_index, self.max_index, self.angle_ff_offset))
      self.min_index = 100
      self.max_index = 0

    self.profiler.checkpoint('update')
    self.frame += 1
    self.live_tune(CP)
    self.profiler.checkpoint('live_tune')

    if v_ego < 0.3 or not path_plan.paramsValid or not active:
      if self.frame > self.next_params_put and v_ego == 0 and not cruise_enabled:
        self.next_params_put = self.frame + 36000
        put_nonblocking("LateralGain", json.dumps({'angle_ff_gain': self.angle_ff_gain, 'angle_ff_offset': self.angle_ff_offset}))
        #self.params.put("LateralGain", json.dumps({'angle_ff_gain': self.angle_ff_gain}))
        self.profiler.checkpoint('params_put')
      self.output_steer = 0.0
      self.stage = "0"
      self.lane_changing = 0.0
      self.previous_integral = 0.0
      self.previous_lane_error = 0.0
      self.damp_angle_steers= 0.0
      self.damp_rate_steers_des = 0.0 
      self.damp_angle_steers_des = 0.0
      pid_log.active = False
      self.pid.reset()
      self.profiler.checkpoint('pid_reset')
    else:
      try:
        pid_log.active = True
        if False and blinker_on and steer_override:
          self.damp_angle_steers = angle_steers
          self.angle_steers_des = angle_steers
          self.damp_angle_steers_des = angle_steers
          self.limited_damp_angle_steers_des = angle_steers
          self.angle_rate_des = 0
        else:
          react_steer = self.react_steer + self.react_center[min(len(self.react_center)-1, int(abs(angle_steers - path_plan.angleOffset - self.angle_ff_offset)))]
          self.damp_angle_steers += (angle_steers + angle_steers_rate * (self.damp_steer + float(react_steer)) - self.damp_angle_steers) / max(1.0, self.damp_steer * 100.)
          if self.use_poly_angle:
            self.angle_steers_des = (self.p_poly(self.angle_index * 0.15 - 3)) * 54.938633 + path_plan.angleBias + path_plan.angleOffset + self.projected_lane_error
          else:
            self.angle_steers_des = interp(self.angle_index, self.path_index, self.fast_angles[min(len(self.fast_angles)-1, int(self.polyReact))]) + self.projected_lane_error
          self.damp_angle_steers_des += (self.angle_steers_des - self.damp_angle_steers_des) / max(1.0, self.damp_mpc * 100.)
          #self.damp_angle_steers_des += (self.angle_steers_des - self.damp_angle_steers_des + self.projected_lane_error) / max(1.0, self.damp_mpc * 100.)
          if (self.damp_angle_steers - self.damp_angle_steers_des) * (angle_steers - self.damp_angle_steers_des) < 0:
            self.damp_angle_steers = self.damp_angle_steers_des

        accel_factor = gernterp(abs(angle_steers - path_plan.angleOffset - self.angle_ff_offset), [0, 5], [1, 0.5]) * v_ego
        self.angle_rate_des = float(min(self.angle_rate_des + self.accel_limit * accel_factor, max(self.angle_rate_des - self.accel_limit * accel_factor, self.damp_angle_steers_des - self.limited_damp_angle_steers_des)))
        self.limited_damp_angle_steers_des += self.angle_rate_des
        wiggle_angle = gernterp(abs(angle_steers - path_plan.angleOffset - self.angle_ff_offset), [0, 1], [self.wiggle_angle, self.wiggle_angle * 0.25])
        requested_angle = min(self.limited_damp_angle_steers_des + wiggle_angle, max(self.limited_damp_angle_steers_des - wiggle_angle, self.angle_steers_des))

        angle_feedforward = float(requested_angle - path_plan.angleOffset - self.angle_ff_offset)
        ff_index = 0 if angle_feedforward > 0 else 1
        self.angle_ff_ratio = float(gernterp(abs(angle_feedforward), self.angle_ff_bp[0], self.angle_ff_bp[1]))
        rate_feedforward = float((1.0 - self.angle_ff_ratio) * self.rate_ff_gain * (requested_angle - self.damp_angle_steers))   #self.angle_rate_des
        steer_feedforward = float(v_ego)**2 * (rate_feedforward + angle_feedforward * self.angle_ff_ratio * self.angle_ff_gain[ff_index])

        if not steer_override and v_ego > 10.0:
          if abs(angle_steers) > (self.angle_ff_bp[0][1] / 2.0):
            self.adjust_angle_gain(ff_index)
          else:
            self.previous_integral = self.pid.i

        if (angle_feedforward > 0) == (self.pid.p > 0) or (path_plan.cProb > 0 and (self.cPoints[-1] > 0) == (self.pid.p > 0)):
          p_scale = 1.0 
        else:
          p_scale = float(gernterp(abs(angle_feedforward), [0., 10.], [max(0.3, min(1.0, 1 / (0.001 + abs(angle_steers_rate)))), max(0.3, min(1.0, 1 / (0.001 + abs(angle_feedforward)), 1 / (0.001 + abs(angle_steers_rate))))]))
        
        #if self.deadzone > 0 or (abs(angle_feedforward) < 1 and abs(angle_steers_rate) == 0 and self.use_deadzone) or np.sign(angle_steers_rate) == -np.sign(steer_feedforward):
        if (abs(angle_feedforward) < 1 and abs(angle_steers_rate) == 0 and self.use_deadzone) or (np.sign(angle_steers_rate) == -np.sign(steer_feedforward) and self.deadzone < 0):
          deadzone = self.deadzone
        else:
          deadzone = 0.0
        
        self.profiler.checkpoint('pre-pid')

        self.output_steer = self.pid.update(requested_angle, self.damp_angle_steers, check_saturation=(v_ego > 10), override=steer_override, p_scale=p_scale,
                                      add_error=0, feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)  #-self.deadzone))
        self.profiler.checkpoint('pid_update')

      except:
        self.output_steer = 0
        self.pid.reset()
        print("angle error!")
        pass
    
      #self.update_lane_state(angle_steers, driver_opposing, blinker_on, path_plan)
      self.profiler.checkpoint('lane_change')

    output_factor = self.lane_change_adjustment if pid_log.active else 0

    if self.lane_change_adjustment < 1 and self.lane_changing > 0:
      self.damp_angle_steers_des = self.angle_steers_des
      self.limited_damp_angle_steers_des = self.angle_steers_des

    self.prev_override = steer_override
    self.pid.f *= output_factor
    self.pid.i *= output_factor
    self.pid.p *= output_factor
    self.output_steer *= output_factor
    pid_log.p = float(self.pid.p)
    pid_log.i = float(self.pid.i)
    pid_log.f = float(self.pid.f)
    pid_log.output = float(self.output_steer)
    pid_log.p2 = float(self.projected_lane_error)
    pid_log.saturated = bool(self.pid.saturated)
    pid_log.angleFFRatio = self.angle_ff_ratio
    pid_log.steerAngle = float(self.damp_angle_steers)
    pid_log.steerAngleDes = float(self.damp_angle_steers_des)
    self.sat_flag = self.pid.saturated
    self.profiler.checkpoint('post_update')

    if abs(angle_steers) > STEER_HYSTERESIS[1] or (self.hysteresis_state == 0 and abs(angle_steers) > STEER_HYSTERESIS[0] and self.c_prob == 0):
      self.output_steer = 0
      self.hysteresis_state = 0
    else:
      self.hysteresis_state = 1


    if self.frame % 5000 == 1000 and self.profiler.enabled:
      self.profiler.display()
      self.profiler.reset(True)

    return self.output_steer, float(self.angle_steers_des), pid_log
