import json
import os
from cereal import log, car
from common.params import Params 

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
    self.type = "pid"
    if self.conf['Kp'] == "-1":
      self.conf['Kp'] = str(round(CP.lateralTuning.pid.kpV[0],3))
      write_conf = True
    if self.conf['Ki'] == "-1":
      self.conf['Ki'] = str(round(CP.lateralTuning.pid.kiV[0],3))
      write_conf = True
    if self.conf['Kf'] == "-1":
      self.conf['Kf'] = str(round(CP.lateralTuning.pid.kf,5))
      write_conf = True
    if self.conf['dampSteer'] == "-1":
      self.conf['dampSteer'] = str(round(CP.lateralTuning.pid.dampSteer,3))
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
    if self.conf['polyDamp'] == "-1":
      self.conf['polyReact'] = str(round(CP.lateralTuning.pid.polyReactTime,3))
      self.conf['polyDamp'] = str(round(CP.lateralTuning.pid.polyDampTime,3))
      self.conf['polyFactor'] = str(round(CP.lateralTuning.pid.polyFactor,3))
      write_conf = True

    if write_conf:
      self.write_config(self.config)

  def read_config(self, CP=None, Reset=False):
    self.element_updated = False

    if Reset or not os.path.isfile(os.path.expanduser('~/kegman.json')):
      self.config = {"Kp":"-1","Ki":"-1","Kf":"-1","rateFFGain":"-1","reactMPC":"-1","dampMPC":"-1","useAutoFlash": "0","useInfluxDB":"0","requireBlinker":"1","requireNudge":"1"}
      self.element_updated = True
    else:
      with open(os.path.expanduser('~/kegman.json'), 'r') as f:
        self.config = json.load(f)

    with open(os.path.expanduser('~/raspilot/selfdrive/gernby.json'), 'r') as f:
      base_config = json.load(f)

    if "advanceSteer" not in self.config:
      self.config.update({"advanceSteer":"0.0"})
      self.element_updated = True

    if "angleFactor" not in self.config:
      self.config.update({"angleFactor":"1.0"})
      self.element_updated = True

    if "lkasMode" not in self.config:
      self.config.update({"lkasMode":"0"})
      self.element_updated = True

    if "useInfluxDB" not in self.config:
      self.config.update({"useInfluxDB":"0"})
      self.element_updated = True

    if not CP is None and ("tuneRev" not in self.config or self.config['tuneRev'] != base_config['tuneRev']):
      for car in base_config:
        if car in CP.carFingerprint:
          print(car, base_config[car])
          break
      if "tuneRev" in self.config and base_config['tuneRev'] != self.config['tuneRev']:
        if os.path.exists(os.path.expanduser('~/kegman.json')):
          with open(os.path.expanduser('~/kegman.json'), 'r') as f1:
            json.load(f1)
            with open(os.path.expanduser('~/kegman_%s.json' % self.config['tuneRev']), 'w') as f2:
              json.dump(self.config, f2, indent=2, sort_keys=True)
              os.chmod(os.path.expanduser("~/kegman.json"), 0o764)
      self.config.update({'tuneRev': base_config['tuneRev']})
      for key, value in base_config[car].items():
        self.config.update({key: value})
        self.element_updated = True

    if "discreteAngle" not in self.config:
      self.config.update({"discreteAngle": "1"})
      self.element_updated = True

    if "useMinimize" not in self.config:
      self.config.update({"useMinimize": "0"})
      self.element_updated = True

    if "requireBlinker" not in self.config:
      self.config.update({"requireNudge": "1"})
      self.config.update({"requireBlinker": "1"})
      self.element_updated = True

    if "reactCenter0" not in self.config:
      self.config.update({"reactCenter0": "0.0"})
      self.config.update({"reactCenter1": "0.0"})
      self.config.update({"reactCenter2": "0.0"})
      self.element_updated = True

    if "reactSteer" not in self.config:
      self.config.update({"reactSteer": "0.0"})
      self.element_updated = True

    if "useLocalImport" not in self.config:
      self.config.update({"useLocalImport": "0"})
      self.element_updated = True

    if "deadzone" not in self.config:
      self.config.update({"deadzone": "-0.1"})
      self.element_updated = True

    if "firstModel" not in self.config:
      self.config.update({"firstModel": "0"})
      self.config.update({"lastModel": "6"})
      self.config.update({"modelFactor": "0.5"})
      self.element_updated = True

    if self.element_updated:
      self.write_config(self.config)

    return self.config

  def write_config(self, config):
    print('    WRITING Kegman!')
    try:
      with open(os.path.expanduser('~/kegman.json'), 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod(os.path.expanduser("~/kegman.json"), 0o764)
    except IOError:
      os.mkdir(os.path.expanduser('~/raspilot/selfdrive/'))
      with open(os.path.expanduser('~/kegman.json'), 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod(os.path.expanduser("~/kegman.json"), 0o764)
