from common.numpy_fast import interp
#from common.kalman.simple_kalman import KF1D
#from opendbc.can.can_define import CANDefine
from selfdrive.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.honda.values import CAR, DBC, STEER_THRESHOLD, SPEED_FACTOR, HONDA_BOSCH
from selfdrive.kegman_conf import kegman_conf

def get_can_parser(isPandaBlack):
  signals = [
      ("FRAME_ID", "CUR_LANE_LEFT_1", 0),
      ("FRAME_ID", "CUR_LANE_LEFT_2", 0),
      ("FRAME_ID", "CUR_LANE_RIGHT_1", 0),
      ("FRAME_ID", "CUR_LANE_RIGHT_2", 0),
      ("FRAME_ID", "ADJ_LANE_LEFT_1", 0),
      ("FRAME_ID", "ADJ_LANE_LEFT_2", 0),
      ("FRAME_ID", "ADJ_LANE_RIGHT_1", 0),
      ("FRAME_ID", "ADJ_LANE_RIGHT_2", 0),
      ("DASHED_LINE", "CUR_LANE_LEFT_2", 0),
      ("DASHED_LINE", "CUR_LANE_RIGHT_2", 0),
      ("DASHED_LINE", "ADJ_LANE_LEFT_2", 0),
      ("DASHED_LINE", "ADJ_LANE_RIGHT_2", 0),
      ("SOLID_LINE", "CUR_LANE_LEFT_2", 0),
      ("SOLID_LINE", "CUR_LANE_RIGHT_2", 0),
      ("SOLID_LINE", "ADJ_LANE_LEFT_2", 0),
      ("SOLID_LINE", "ADJ_LANE_RIGHT_2", 0),
      ("PARM_1", "CUR_LANE_LEFT_1", 0),
      ("PARM_2", "CUR_LANE_LEFT_1", 0),
      ("PARM_3", "CUR_LANE_LEFT_1", 0),
      ("PARM_4", "CUR_LANE_LEFT_1", 0),
      ("PARM_5", "CUR_LANE_LEFT_1", 0),
      ("PARM_6", "CUR_LANE_LEFT_2", 0),
      ("PARM_7", "CUR_LANE_LEFT_2", 0),
      ("PARM_8", "CUR_LANE_LEFT_2", 0),
      ("PARM_9", "CUR_LANE_LEFT_2", 0),
      ("PARM_10", "CUR_LANE_LEFT_2", 0),
      ("PARM_1", "CUR_LANE_RIGHT_1", 0),
      ("PARM_2", "CUR_LANE_RIGHT_1", 0),
      ("PARM_3", "CUR_LANE_RIGHT_1", 0),
      ("PARM_4", "CUR_LANE_RIGHT_1", 0),
      ("PARM_5", "CUR_LANE_RIGHT_1", 0),
      ("PARM_6", "CUR_LANE_RIGHT_2", 0),
      ("PARM_7", "CUR_LANE_RIGHT_2", 0),
      ("PARM_8", "CUR_LANE_RIGHT_2", 0),
      ("PARM_9", "CUR_LANE_RIGHT_2", 0),
      ("PARM_10", "CUR_LANE_RIGHT_2", 0),
      ("PARM_1", "ADJ_LANE_RIGHT_1", 0),
      ("PARM_2", "ADJ_LANE_RIGHT_1", 0),
      ("PARM_3", "ADJ_LANE_RIGHT_1", 0),
      ("PARM_4", "ADJ_LANE_RIGHT_1", 0),
      ("PARM_5", "ADJ_LANE_RIGHT_1", 0),
      ("PARM_6", "ADJ_LANE_RIGHT_2", 0),
      ("PARM_7", "ADJ_LANE_RIGHT_2", 0),
      ("PARM_8", "ADJ_LANE_RIGHT_2", 0),
      ("PARM_9", "ADJ_LANE_RIGHT_2", 0),
      ("PARM_10", "ADJ_LANE_RIGHT_2", 0),
      ("PARM_1", "ADJ_LANE_LEFT_1", 0),
      ("PARM_2", "ADJ_LANE_LEFT_1", 0),
      ("PARM_3", "ADJ_LANE_LEFT_1", 0),
      ("PARM_4", "ADJ_LANE_LEFT_1", 0),
      ("PARM_5", "ADJ_LANE_LEFT_1", 0),
      ("PARM_6", "ADJ_LANE_LEFT_2", 0),
      ("PARM_7", "ADJ_LANE_LEFT_2", 0),
      ("PARM_8", "ADJ_LANE_LEFT_2", 0),
      ("PARM_9", "ADJ_LANE_LEFT_2", 0),
      ("PARM_10", "ADJ_LANE_LEFT_2", 0),
  ]
  checks = [
      ("CUR_LANE_LEFT_1", 15),
      ("CUR_LANE_LEFT_2", 15),
      ("CUR_LANE_RIGHT_1", 15),
      ("CUR_LANE_RIGHT_2", 15),
      ("CUR_LANE_LEFT_1", 15),
      ("CUR_LANE_LEFT_2", 15),
      ("CUR_LANE_RIGHT_1", 15),
      ("CUR_LANE_RIGHT_2", 15),
      ("ADJ_LANE_LEFT_1", 15),
      ("ADJ_LANE_LEFT_2", 15),
      ("ADJ_LANE_RIGHT_1", 15),
      ("ADJ_LANE_RIGHT_2", 15),
      ("ADJ_LANE_LEFT_1", 15),
      ("ADJ_LANE_LEFT_2", 15),
      ("ADJ_LANE_RIGHT_1", 15),
      ("ADJ_LANE_RIGHT_2", 15),
  ]

  bus_cam = 1 if not isPandaBlack else 2
  return CANParser(b"bosch_camera", signals, checks, bus_cam)

class CameraState(object):
  def __init__(self, isPandaBlack):
    self.cp_cam = get_can_parser(isPandaBlack)

  def update(self, can_time, can_strings):
    self.cp_cam.update_strings(can_time, can_strings)
    self.cam_left_1 = self.cp_cam.vl["CUR_LANE_LEFT_1"]
    self.cam_left_2 = self.cp_cam.vl["CUR_LANE_LEFT_2"]
    self.cam_right_1 = self.cp_cam.vl["CUR_LANE_RIGHT_1"]
    self.cam_right_2 = self.cp_cam.vl["CUR_LANE_RIGHT_2"]
    self.cam_far_left_1 = self.cp_cam.vl["ADJ_LANE_LEFT_1"]
    self.cam_far_left_2 = self.cp_cam.vl["ADJ_LANE_LEFT_2"]
    self.cam_far_right_1 = self.cp_cam.vl["ADJ_LANE_RIGHT_1"]
    self.cam_far_right_2 = self.cp_cam.vl["ADJ_LANE_RIGHT_2"]
