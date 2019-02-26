from selfdrive.car.mazda.values import CAR, DBC

def create_steering_control(packer, bus, car_fingerprint, ctr, lkas, lnv, b1, b2, err1, err2):
  # The checksum is calculated by subtracting all byte values across the msg from 249
  # however, the first byte is devided in half and are the two halves
  # are subtracted separtaley. bytes 3 and 4 are constants at 32 and 2 repectively
  # for example
  # the checksum for the msg b8 00 00 20 02 00 00 c4 would be
  #  hex: checksum = f9 - b - 00 - 00 - 08 - 20 - 02 - 00 - 00 = c4
  #  dec: chechsum = 249 - 11 - 0 - 0 - 8 - 32 - 2  - 0 - 0   = 196
  csum = 249 - ctr - (apply_steer >> 8) - (apply_steer & 0x0FF) - lineval - 32 - 2 - e1 - (e2 << 6)

  if car_fingerprint == CAR.CX5:
    values = {
      "CTR"              : ctr,
      "LKAS_REQUEST"     : lkas,
      "BIT_1"            : b1,
      "BIT_2"            : b2,
      "LINE_NOT_VISIBLE" : lnv,
      "ERR_BIT_1"        : err1,
      "ERR_BIT_2"        : err2,
      "CHKSUM"           : csum
    }

  return packer.make_can_msg("CAM_LKAS", bus, values)

def create_lkas_msg(packer, bus, car_fingerprint, CAM_LKAS):
  if car_fingerprint == CAR.CX5:
    values = {
      "LKAS_REQUEST"     : CAM_LKAS.lkas,
      "CTR"	         : CAM_LKAS.ctr,
      "ERR_BIT_1"        : CAM_LKAS.err1,
      "LINE_NOT_VISIBLE" : CAM_LKAS.lnv,
      "BIT_1"            : CAM_LKAS.bit1,
      "ERR_BIT_2"        : CAM_LKAS.err2,
      "BIT_2"            : CAM_LKAS.bit2,
      "CHKSUM"           : CAM_LKAS.chksum
    }

  return packer.make_can_msg("CAM_LKAS", bus, values)


def create_lane_track(packer, bus, car_fingerprint, CAM_LT):
  if car_fingerprint == CAR.CX5:
    values = {
      "LINE1"      : CAM_LT.line1,
      "CTR"        : CAM_LT.ctr,
      "LINE2"      : CAM_LT.line2,
      "LANE_CURVE" : CAM_LT.lane_curve,
      "SIG1"       : CAM_LT.sig1,
      "SIG2"       : CAM_LT.sig2,
      "ZERO"       : CAM_LT.zero,
      "SIG3"       : CAM_LT.sig3,
      "CHKSUM"     : CAM_LT.chksum
    }

  return packer.make_can_msg("CAM_LANETRACK", bus, values)
