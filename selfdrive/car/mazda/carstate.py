import numpy as np
from cereal import car
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.can.parser import CANParser
from selfdrive.car.mazda.values import DBC, CAR

def get_powertrain_can_parser(CP, canbus):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("LEFT_BLINK", "BLINK_INFO", 0), 
    ("RIGHT_BLINK", "BLINK_INFO", 0),
    ("STEER_ANGLE", "STEER", 0),
    ("STEER_ANGLE_RATE", "STEER_RATE", 0),
    ("STEER_TORQUE_SENSOR", "STEER_TORQUE", 0),
    ("FL", "WHEEL_SPEEDS", 0), 
    ("FR", "WHEEL_SPEEDS", 0),
    ("RL", "WHEEL_SPEEDS", 0), 
    ("RR", "WHEEL_SPEEDS", 0), 
    ("CRZ_ACTIVE", "CRZ_CTRL", 0),
    ("STANDSTILL","PEDALS", 0),
    ("BRAKE_ON","PEDALS", 0),
    ("GEAR","GEAR", 0),
    ("DRIVER_SEATBELT", "SEATBELT", 0),
    ("FL", "DOORS", 0),
    ("GAS_PEDAL_PRESSED", "CRZ_EVENTS", 0),
  ]
  
  checks = [
    # sig_address, frequency
    ("BLINK_INFO", 100),
    ("STEER", 20),
    ("STEER_RATE", 20),
    ("STEER_TORQUE", 20),
    ("WHEEL_SPEEDS", 20),
    ("CRZ_CTRL", 20),
    ("CRZ_EVENTS", 50),
    ("PEDALS", 20),
    ("SEATBELT", 100),
    ("DOORS", 100),
    ("GEAR", 50),
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.powertrain)
  
class CarState(object):
  def __init__(self, CP, canbus):
    # initialize can parser
    self.CP = CP
    
    self.car_fingerprint = CP.carFingerprint
    self.blinker_on = False
    self.prev_blinker_on = False
    self.left_blinker_on = False
    self.prev_left_blinker_on = False
    self.right_blinker_on = False
    self.prev_right_blinker_on = False

    self.steer_torque_driver = 0
    self.steer_not_allowed = False

    self.main_on = False

    # vEgo kalman filter
    dt = 0.01
    self.v_ego_kf = KF1D(x0=np.matrix([[0.], [0.]]),
                         A=np.matrix([[1., dt], [0., 1.]]),
                         C=np.matrix([1., 0.]),
                         K=np.matrix([[0.12287673], [0.29666309]]))
    self.v_ego = 0.

  def update(self, pt_cp):

    self.can_valid = pt_cp.can_valid
    self.can_valid = True
    
    self.v_wheel_fl = pt_cp.vl["WHEEL_SPEEDS"]['FL'] * CV.KPH_TO_MS
    self.v_wheel_fr = pt_cp.vl["WHEEL_SPEEDS"]['FR'] * CV.KPH_TO_MS
    self.v_wheel_rl = pt_cp.vl["WHEEL_SPEEDS"]['RL'] * CV.KPH_TO_MS
    self.v_wheel_rr = pt_cp.vl["WHEEL_SPEEDS"]['RR'] * CV.KPH_TO_MS
    speed_estimate = (self.v_wheel_fl + self.v_wheel_fr + self.v_wheel_rl + self.v_wheel_rr) / 4.0

    self.v_ego_raw = speed_estimate
    # FIXME
    v_ego_x = self.v_ego_kf.update(speed_estimate)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.prev_blinker_on = self.blinker_on
    self.left_blinker_on = pt_cp.vl["BLINK_INFO"]['LEFT_BLINK'] == 1
    self.right_blinker_on = pt_cp.vl["BLINK_INFO"]['RIGHT_BLINK'] == 1
    self.blinker_on = self.left_blinker_on or self.right_blinker_on

    self.acc_active = pt_cp.vl["CRZ_CTRL"]['CRZ_ACTIVE']
    self.main_on = pt_cp.vl["CRZ_CTRL"]['CRZ_ACTIVE']
      
    self.steer_torque_driver = pt_cp.vl["STEER_TORQUE"]['STEER_TORQUE_SENSOR']
    self.steer_override = abs(self.steer_torque_driver) > 150 #fixme

    self.angle_steers = pt_cp.vl["STEER"]['STEER_ANGLE'] 
    self.angle_steers_rate = pt_cp.vl["STEER_RATE"]['STEER_ANGLE_RATE']

    #self.standstill = pt_cp.vl["PEDALS"]['STANDSTILL'] == 1
    #self.brake_pressed = pt_cp.vl["PEDALS"]['BREAK_PEDAL_1'] == 1

    self.standstill = self.v_ego_raw < 0.01
    
