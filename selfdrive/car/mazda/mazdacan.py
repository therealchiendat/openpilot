from selfdrive.car.mazda.values import CAR, DBC

def create_steering_control(packer, bus, car_fingerprint, idx, steer, linebit, checksum):
  if car_fingerprint == CAR.CX5:
    values = {
      "CTR": idx,
      "LKAS_REQUEST": steer,
      "NEW_SIGNAL_1": 1,
      "NEW_SIGNAL_5": 1,
      "NO_LANE_LINE": linebit,
      "CHKSUM": checksum
    }

  return packer.make_can_msg("CAM_LKAS", bus, values)
