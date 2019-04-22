from selfdrive.car.mazda.values import CAR, DBC

def create_steering_control(packer, bus, car_fingerprint, ctr, lkas, lnv, b1, b2, err1, err2, ldw):
  # The checksum is calculated by subtracting all byte values across the msg from 241
  # however, the first byte is devided in half and are the two halves
  # are subtracted separtaley the second half must be subtracted from 8 first.
  # bytes 3 and 4 are constants at 32 and 2 repectively
  # for example:
  # the checksum for the msg b8 00 00 20 02 00 00 c4 would be
  #  hex: checksum = f1 - b - (8-8) - 00 - 20 - 02 - 00 - 00 = c4
  #  dec: chechsum = 241 - 11 - (8-8) - 0 - 32 - 2  - 0 - 0   = 196

  tmp = lkas + 2048

  lkasl = tmp & 0xFF
  lkash = tmp >> 8

  csum = 241 - ctr - (lkash - 8) - lkasl - (lnv << 3) - (b1 << 5)  - (b2 << 1) - (ldw << 7)

  if csum < 0:
      csum = csum + 256

  csum = csum % 256

  if car_fingerprint == CAR.CX5:
    values = {
      "CTR"              : ctr,
      "LKAS_REQUEST"     : lkas,
      "BIT_1"            : b1,
      "BIT_2"            : b2,
      "LDW"              : ldw,
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

def create_cam_lane_info(packer, bus, car_fingerprint, lnv, cam_laneinfo, steer_lkas, ldwr, ldwl, lines):

  if steer_lkas.block == 1:
    lin = 0
  elif lines == 0:
    lin = 0
  else:
    lin = 2

  if car_fingerprint == CAR.CX5:
    values = {
        "LINE_VISIBLE"          : 1 if lnv == 0 else 0,
        "LINE_NOT_VISIBLE"      : lnv,
        "BIT1"                  : 1,
        "LANE_LINES"            : lin,
        "BIT2"                  : cam_laneinfo["BIT2"],
        "NO_ERR_BIT"            : cam_laneinfo["NO_ERR_BIT"],
        "ERR_BIT"               : 0,
        "HANDS_WARN_3_BITS"     : 0 , #if ldwl == 0 and ldwr == 0 else 7,
        "S1"                    : cam_laneinfo["S1"],
        "S1_NOT"                : cam_laneinfo["S1_NOT"],
        "HANDS_ON_STEER_WARN"   : 0, #ldwr+ldwl,
        "HANDS_ON_STEER_WARN_2" : 0, #ldwr+ldwl,
        "BIT3"                  : 1,
        "LDW_WARN_RL"           : 0, #ldwr,
        "LDW_WARN_LL"           : 0 #ldwl
    }

    return packer.make_can_msg("CAM_LANEINFO", bus, values)

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
