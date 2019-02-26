from selfdrive.car.mazda.values import CAR, DBC

def create_steering_control(packer, bus, car_fingerprint, ctr, steer, lnv, b1, b2, err1, err2, checksum):
  if car_fingerprint == CAR.CX5:
    values = {
      "CTR": ctr,
      "LKAS_REQUEST": steer,
      "BIT_1": b1,
      "BIT_2": b2,
      "LINE_NOT_VISIBLE": lnv,
      "ERR_BIT_1" : err1,
      "ERR_BIT_2" : err2,
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
