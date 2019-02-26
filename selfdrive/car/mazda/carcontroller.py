from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.mazda.carstate import CarState, get_powertrain_can_parser
from selfdrive.car.mazda import mazdacan
from selfdrive.car.mazda.values import CAR, DBC
from selfdrive.can.packer import CANPacker


class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.STEER_MAX = 256              # max_steer 2048
    self.STEER_STEP = 3    # 6        # how often we update the steer cmd
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
      
      
      lkas_enabled = enabled and not CS.steer_not_allowed

      if not lkas_enabled:
          apply_steer = 0

      if CS.v_ego_raw < 10:
          apply_steer = 0

      lineval = 8
      linebit = 1

      if self.car_fingerprint == CAR.CX5:
        if CS.v_ego_raw > 10:
          linebit = 0
          lineval = 0
        
        #counts from 0 to 15 then back to 0
        ctr = CS.CAM_LKAS.ctr #(frame / P.STEER_STEP) % 16

        if ctr != -1 and self.last_cam_ctr != ctr and ctr == CS.CAM_LT.ctr:
          self.last_cam_ctr = ctr
          e1 = 0
          e2 = 0
          if CS.CAM_LT.ctr == 0 and CS.CAM_LT.chksum == 0xff:
            e1 = 1
            e2 = 1

          # The checksum is calculated by subtracting all byte values across the msg from 249
          # however, the first byte is devided in half and are the two halves
          # are subtracted separtaley. bytes 3 and 4 are constants at 32 and 2 repectively
          # for example
          # the checksum for the msg b8 00 00 20 02 00 00 c4 would be
          #  hex: checksum = f9 - b - 00 - 00 - 08 - 20 - 02 - 00 - 00 = c4
          #  dec: chechsum = 249 - 11 - 0 - 0 - 8 - 32 - 2  - 0 - 0   = 196
          checksum = 249 - ctr - (apply_steer >> 8) - (apply_steer & 0x0FF) - lineval - 32 - 2 - e1 - (e2 << 6)

          #can_sends.append(mazdacan.create_steering_control(self.packer_pt, canbus.powertrain,
          #                                                  CS.CP.carFingerprint, ctr, apply_steer, linebit, 1, 1, e1, e2, checksum))

          can_sends.append(mazdacan.create_steering_control(self.packer_pt, canbus.powertrain,
                                                            CS.CP.carFingerprint, CS.CAM_LKAS.ctr,
                                                            CS.CAM_LKAS.lkas, CS.CAM_LKAS.lnv,
                                                            CS.CAM_LKAS.bit1, CS.CAM_LKAS.bit2,
                                                            CS.CAM_LKAS.err1, CS.CAM_LKAS.err2,
                                                            CS.CAM_LKAS.chksum))
          
          can_sends.append(mazdacan.create_lane_track(self.packer_pt, canbus.powertrain, CS.CP.carFingerprint, CS.CAM_LT))
    
      sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
