from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.mazda.carstate import CarState, get_powertrain_can_parser, get_cam_can_parser
from selfdrive.car.mazda import mazdacan
from selfdrive.car.mazda.values import CAR, DBC
from selfdrive.can.packer import CANPacker


class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.STEER_MAX = 256              # max_steer 2048
    self.STEER_STEP = 1    # 6        # how often we update the steer cmd
    self.STEER_DELTA_UP = 20           # torque increase per refresh
    self.STEER_DELTA_DOWN = 20         # torque decrease per refresh
    if car_fingerprint == CAR.CX5:
      self.STEER_DRIVER_ALLOWANCE = 5000   # allowed driver torque before start limiting
    else:
      self.STEER_DRIVER_ALLOWANCE = 250   # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 1   # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1     # from dbc
    


class CarController(object):
  def __init__(self, canbus, car_fingerprint, enable_camera):
    self.start_time = sec_since_boot()
    self.lkas_active = False
    self.steer_idx = 0
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.canbus = canbus
    self.params = CarControllerParams(car_fingerprint)
    print(DBC)
    self.packer_pt = CANPacker(DBC[car_fingerprint]['pt'])

    self.last_cam_ctr = -1

  def update(self, sendcan, enabled, CS, frame, actuators):
    """ Controls thread """

    P = self.params

    # Send CAN commands.
    can_sends = []
    canbus = self.canbus

    ### STEER ###

    if (frame % P.STEER_STEP) == 0:
    
      final_steer = actuators.steer if enabled else 0.
      apply_steer = final_steer * P.STEER_MAX
      # limits due to driver torque
      driver_max_torque = P.STEER_MAX + (P.STEER_DRIVER_ALLOWANCE + CS.steer_torque_driver * P.STEER_DRIVER_FACTOR) * P.STEER_DRIVER_MULTIPLIER
      driver_min_torque = -P.STEER_MAX + (-P.STEER_DRIVER_ALLOWANCE + CS.steer_torque_driver * P.STEER_DRIVER_FACTOR) * P.STEER_DRIVER_MULTIPLIER
      max_steer_allowed = max(min(P.STEER_MAX, driver_max_torque), 0)
      min_steer_allowed = min(max(-P.STEER_MAX, driver_min_torque), 0)
      apply_steer = clip(apply_steer, min_steer_allowed, max_steer_allowed)

      # slow rate if steer torque increases in magnitude
      if self.apply_steer_last > 0:
        apply_steer = clip(apply_steer, max(self.apply_steer_last - P.STEER_DELTA_DOWN, -P.STEER_DELTA_UP), self.apply_steer_last + P.STEER_DELTA_UP)
      else:
        apply_steer = clip(apply_steer, self.apply_steer_last - P.STEER_DELTA_UP, min(self.apply_steer_last + P.STEER_DELTA_DOWN, P.STEER_DELTA_UP))

      apply_steer = int(round(apply_steer))
      self.apply_steer_last = apply_steer
      
      
      lkas_enabled = enabled and not CS.steer_not_allowed and (CS.v_ego_raw > 10)

      if not lkas_enabled:
          apply_steer = 0

      if self.car_fingerprint == CAR.CX5:
        #counts from 0 to 15 then back to 0
        ctr = CS.CAM_LKAS.ctr #(frame / P.STEER_STEP) % 16

        if ctr != -1 and self.last_cam_ctr != ctr:
          self.last_cam_ctr = ctr
          line_not_visible = CS.CAM_LKAS.lnv
          e1 = CS.CAM_LKAS.err1
          e2 = CS.CAM_LKAS.err2

          #can_sends.append(mazdacan.create_steering_control(self.packer_pt, canbus.powertrain,
          #                                                  CS.CP.carFingerprint, ctr, apply_steer, line_not_visible, 1, 1, e1, e2))

          can_sends.append(mazdacan.create_lkas_msg(self.packer_pt, canbus.powertrain, CS.CP.carFingerprint, CS.CAM_LKAS))
          
          #can_sends.append(mazdacan.create_lane_track(self.packer_pt, canbus.powertrain, CS.CP.carFingerprint, CS.CAM_LT))
    
      sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
