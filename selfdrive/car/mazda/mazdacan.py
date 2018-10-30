from selfdrive.car.mazda.values import CAR, DBC

def create_steering_control(packer, bus, car_fingerprint, idx, steer, checksum):
  if car_fingerprint == CAR.CX5:
    values = {
      "Counter": idx,
      "LKAS_Request": steer,
      "NEW_SIGNAL_1": 1,
      "NEW_SIGNAL_5": 1,
      "Checksum": checksum
    }
    
  return packer.make_can_msg("CAM_LKAS", 0, values)
