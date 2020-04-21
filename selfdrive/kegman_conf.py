import json
import os

class kegman_conf():
  def __init__(self, CP=None):
    if CP is not None:
      self.type = CP.lateralTuning.which()
    self.conf = self.read_config(CP)

    #print(self.conf)
    if CP is not None:
      try:
        self.init_config(CP)

      #print(self.conf)
      except:
        self.conf = self.read_config(CP, True)
        self.init_config(CP)

  def init_config(self, CP):
    write_conf = False
    if CP.lateralTuning.which() == 'pid':
      self.type = "pid"
      if self.conf['type'] == "-1":
        self.conf["type"] = "pid"
        write_conf = True
      if self.conf['Kp'] == "-1":
        self.conf['Kp'] = str(round(CP.lateralTuning.pid.kpV[0],3))
        write_conf = True
      if self.conf['Ki'] == "-1":
        self.conf['Ki'] = str(round(CP.lateralTuning.pid.kiV[0],3))
        write_conf = True
      if self.conf['Kf'] == "-1":
        self.conf['Kf'] = str(round(CP.lateralTuning.pid.kf,5))
        write_conf = True
      if self.conf['dampTime'] == "-1":
        self.conf['dampTime'] = str(round(CP.lateralTuning.pid.dampTime,3))
        write_conf = True
      if self.conf['reactMPC'] == "-1":
        self.conf['reactMPC'] = str(round(CP.lateralTuning.pid.reactMPC,3))
        write_conf = True
      if self.conf['dampMPC'] == "-1":
        self.conf['dampMPC'] = str(round(CP.lateralTuning.pid.dampMPC,3))
        write_conf = True
      if self.conf['rateFFGain'] == "-1":
        self.conf['rateFFGain'] = str(round(CP.lateralTuning.pid.rateFFGain,3))
        write_conf = True
      if self.conf['fingerprint'] == "-1":
        print("getting fingerprint!")
        fingerprint = self.get_fingerprint()
        if not fingerprint is None:
          self.conf['fingerprint'] = str(fingerprint)
          write_conf = True
      if self.conf['polyDamp'] == "-1":
        self.conf['polyReact'] = str(round(CP.lateralTuning.pid.polyReactTime,3))
        self.conf['polyDamp'] = str(round(CP.lateralTuning.pid.polyDampTime,3))
        self.conf['polyFactor'] = str(round(CP.lateralTuning.pid.polyFactor,3))
        write_conf = True
    else:
      self.type = "indi"
      if self.conf['type'] == "-1":
        self.conf["type"] = "indi"
        write_conf = True
      if self.conf['timeConst'] == "-1":
        self.conf['type'] = "indi"
        self.conf['timeConst'] = str(round(CP.lateralTuning.indi.timeConstant,3))
        self.conf['actEffect'] = str(round(CP.lateralTuning.indi.actuatorEffectiveness,3))
        self.conf['outerGain'] = str(round(CP.lateralTuning.indi.outerLoopGain,3))
        self.conf['innerGain'] = str(round(CP.lateralTuning.indi.innerLoopGain,3))
        write_conf = True
      if self.conf['reactMPC'] == "-1":
        self.conf['reactMPC'] = str(round(CP.lateralTuning.indi.reactMPC,3))
        write_conf = True

    if write_conf:
      self.write_config(self.config)

  def get_fingerprint(self):
    from common.params import Params
    params = Params()
    fingerprints = ['4973','060f','6f6d','2e00','6512','a2b8']
    try:
      this_car = str(params.get("PandaDongleId"))
    except:
      this_car = str(params.get("DongleId"))

    for i in range(len(fingerprints)):
      if fingerprints[i] in this_car:
        return i

  def read_config(self, CP=None, Reset=False):
    self.element_updated = False

    with open(os.path.expanduser('~/raspilot/selfdrive/gernby.json'), 'r') as f:
      base_config = json.load(f)

    if Reset or not os.path.isfile(os.path.expanduser('~/kegman.json')):
      self.config = {"Kp":"-1","Ki":"-1","Kf":"-1","fingerprint":"-1","rateFFGain":"-1","reactMPC":"-1","dampMPC":"-1","dampSteer":"-1","advanceSteer":"-1","angleFactor":"1.0","lkasMode":"0", "reactSteer":"-1","cameraOffset":"0.06"}
    else:
      with open(os.path.expanduser('~/kegman.json'), 'r') as f:
        self.config = json.load(f)

    if "battPercOff" not in self.config:
      self.config.update({"battPercOff":"25"})
      self.config.update({"carVoltageMinEonShutdown":"11800"})
      self.config.update({"brakeStoppingTarget":"0.25"})
      self.element_updated = True

    if "liveParams" not in self.config:
      self.config.update({"liveParams":"1"})
      self.element_updated = True

    if "leadDistance" not in self.config:
      self.config.update({"leadDistance":"5"})
      self.element_updated = True

    if "advanceSteer" not in self.config:
      self.config.update({"advanceSteer":"0.0"})
      self.element_updated = True

    if "angleFactor" not in self.config:
      self.config.update({"angleFactor":"1.0"})
      self.element_updated = True

    if "lkasMode" not in self.config:
      self.config.update({"lkasMode":"0"})
      self.element_updated = True

    if "lateralOffset" not in self.config:
      self.config.update({"lateralOffset":"0"})
      self.config.update({"angleOffset":"0"})
      self.element_updated = True

    if "fingerprint" not in self.config:
      self.config.update({"fingerprint":"-1"})
      self.element_updated = True

    if ("type" not in self.config or self.config['type'] == "-1") and CP != None:
      self.config.update({"type":CP.lateralTuning.which()})
      print(CP.lateralTuning.which())
      self.element_updated = True

    if "dashIP" not in self.config:
      self.config.update({"dashIP":"tcp://gernstation.synology.me"})
      self.config.update({"dashCapture":"controlsState,liveParameters,gpsLocation"})
      self.element_updated = True

    if "tuneRev" not in self.config or self.config['tuneRev'] != base_config['tuneRev']:
      for key, value in base_config.items():
        self.config.update({key: value})
        self.element_updated = True

    if self.element_updated:
      print("updated")
      self.write_config(self.config)

    return self.config

  def write_config(self, config):

    try:
      with open(os.path.expanduser('~/kegman.json'), 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod(os.path.expanduser("~/kegman.json"), 0o764)
    except IOError:
      os.mkdir(os.path.expanduser('~/raspilot/selfdrive/'))
      with open(os.path.expanduser('~/kegman.json'), 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod(os.path.expanduser("~/kegman.json"), 0o764)
