from selfdrive.car.mazda.values import CAR, DBC

def create_steering_control(packer, bus, car_fingerprint, idx, steer, linebit, checksum):
  if car_fingerprint == CAR.CX5:
    values = {
      "CTR": idx,
      "LKAS_REQUEST": steer,
      "BIT_1": 1,
      "BIT_2": 1,
      "LINE_NOT_VISIBLE": linebit,
      "CHKSUM": checksum
    }

  return packer.make_can_msg("CAM_LKAS", bus, values)


def create_lane_track(packer, bus, car_fingerprint, CAM_LT):
  if car_fingerprint == CAR.CX5:
    values = {
      "LINE1":      CAM_LT.line1,
      "CTR":        CAM_LT.ctr,
      "LINE2":      CAM_LT.line2,
      "LANE_CURVE": CAM_LT.lane_curve,
      "SIG1":       CAM_LT.sig1,
      "SIG2":       CAM_LT.sig2,
      "ZERO":       CAM_LT.zero,
      "SIG3":       CAM_LT.sig3,
      "CHKSUM":     CAM_LT.chksum
    }

  return packer.make_can_msg("CAM_LANETRACK", bus, values)
