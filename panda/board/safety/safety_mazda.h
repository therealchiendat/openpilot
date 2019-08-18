
#define LKAS 0x243
#define LANEINFO 0x440

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
int op_lkas_detected = 0;
int op_laneinfo_detected = 0;

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
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  // Check if msg is sent on BUS 0
  if (bus == 0) {
    if (addr == LKAS){
      op_lkas_detected = 1;
    }
    if (addr == LANEINFO){
      op_laneinfo_detected = 1;
    }
  }

  // 1 allows the message through
  return tx;
}

static int mazda_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  return false;
}

static int mazda_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  
  // shifts bits 29 > 11
  int32_t addr = to_fwd->RIR >> 21;

  // forward CAN 0 > 1
  if (bus_num == 0) {
    return 1; //  CAN Bus0 ==> CAN Bus1
  }
  // forward CAN 1 > 0
  else if (bus_num == 1) {
      // drop stock CAM_LKAS if OP is sending them
    if (addr == LKAS && op_lkas_detected) {
      return -1;
    }
    // drop stock CAM_LANEINFO if OP is sending them
    if (addr == LANEINFO && op_laneinfo_detected) {
      return -1;
    }
    return 0; // Main CAN
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
