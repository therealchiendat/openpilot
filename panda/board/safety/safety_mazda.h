void mazda_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {}

int mazda_ign_hook() {
  return -1; 
}

// FIXME
// *** all output safety mode ***

static void mazda_init(int16_t param) {
  controls_allowed = 1;
}

static int mazda_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  return false;
}

static int mazda_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  return false;
}

static int mazda_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  
  // shifts bits 29 > 11
  int32_t addr = to_fwd->RIR >> 21;

  // forward CAN 0 > 1
  if (bus_num == 0) {
    return -1; // CAM CAN , for now  with stock OBDII don't forward anything
  }
  // forward CAN 1 > 0, except ES_LKAS
  else if (bus_num == 1) {
    if (addr == 0x243) {
      return -1;
    }

    return -1; // Main CAN, for now  with stock OBDII don't forward anything
  }

  // fallback to do not forward
  return -1;
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .tx_lin = mazda_tx_lin_hook,
  .ignition = mazda_ign_hook,
  .fwd = mazda_fwd_hook,
};
