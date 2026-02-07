#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/hyundai_common.h"

#define HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(bus)                               \
  {0x1CF, bus, 8, .check_relay = false}, /* CRUISE_BUTTON */

#define HYUNDAI_CANFD_LKA_STEERING_COMMON_TX_MSGS(a_can, e_can)                \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can){                                  \
      0x50, a_can, 16, .check_relay = (a_can) == 0},   /* LKAS */              \
      {0x2A4, a_can, 24, .check_relay = (a_can) == 0}, /* CAM_0x2A4 */

#define HYUNDAI_CANFD_LKA_STEERING_ALT_COMMON_TX_MSGS(a_can, e_can)            \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can){                                  \
      0x110, a_can, 32, .check_relay = (a_can) == 0},  /* LKAS_ALT */          \
      {0x362, a_can, 32, .check_relay = (a_can) == 0}, /* CAM_0x362 */

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e_can)                       \
  {0x12A, e_can, 16, .check_relay = (e_can) == 0},     /* LFA */               \
      {0x1E0, e_can, 16, .check_relay = (e_can) == 0}, /* LFAHDA_CLUSTER */    \
      {0x161, e_can, 32, .check_relay = (e_can) == 0}, /* MSG_161 */           \
      {0x162, e_can, 32, .check_relay = (e_can) == 0}, /* MSG_162 */

#define HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e_can, longitudinal)          \
  {0x1A0, e_can, 32, .check_relay = (longitudinal)}, /* SCC_CONTROL */

#define MAX_RX_CHECKS 16

static safety_config hyundai_canfd_init_safety_config(void) {
  static RxCheck hyundai_canfd_rx_checks[MAX_RX_CHECKS] = {0};
  safety_config ret = {.rx_checks = hyundai_canfd_rx_checks,
                       .rx_checks_len = 0,
                       .tx_msgs = NULL,
                       .tx_msgs_len = 0};
  return ret;
}

static void add_rx_check(safety_config *safetyConfig, RxCheck config) {
  const uint8_t index = safetyConfig->rx_checks_len;
  safetyConfig->rx_checks_len++;
  if (index < (uint8_t)MAX_RX_CHECKS) {
    // We can't copy the struct directly because of the const member.
    // So we have to copy it byte-by-byte.
    const uint8_t *src = (const uint8_t *)&config;
    uint8_t *dst = (uint8_t *)&safetyConfig->rx_checks[index];
    for (unsigned int i = 0; i < sizeof(RxCheck); i++) {
      dst[i] = src[i];
    }
  }
}

// SCC_CONTROL (from ADAS unit or camera)
#define HYUNDAI_CANFD_SCC_ADDR_CHECK(scc_bus)                                  \
  {                                                                            \
    .msg = {                                                                   \
      {0x1a0, (scc_bus), 32, 50U, .max_counter = 0xffU,                        \
       .ignore_quality_flag = true},                                           \
      {0},                                                                     \
      {0}                                                                      \
    }                                                                          \
  }

static bool hyundai_canfd_alt_buttons = false;
static bool hyundai_canfd_lka_steering_alt = false;

static unsigned int hyundai_canfd_get_lka_addr(void) {
  return hyundai_canfd_lka_steering_alt ? 0x110U : 0x50U;
}

static uint8_t hyundai_canfd_get_counter(const CANPacket_t *msg) {
  uint8_t ret = 0;
  if (GET_LEN(msg) == 8U) {
    ret = msg->data[1] >> 4;
  } else {
    ret = msg->data[2];
  }
  return ret;
}

static uint32_t hyundai_canfd_get_checksum(const CANPacket_t *msg) {
  uint32_t chksum = msg->data[0] | (msg->data[1] << 8);
  return chksum;
}

static void hyundai_canfd_rx_hook(const CANPacket_t *msg) {

  const unsigned pt_bus = hyundai_canfd_lka_steering ? 1U : 0U;
  const unsigned int scc_bus = hyundai_camera_scc ? 2U : pt_bus;

  if (msg->bus == pt_bus) {
    // driver torque
    if (msg->addr == 0xeaU) {
      int torque_driver_new = ((msg->data[11] & 0x1fU) << 8U) | msg->data[10];
      torque_driver_new -= 4095;
      update_sample(&torque_driver, torque_driver_new);
    }

    // cruise buttons
    const unsigned int button_addr =
        hyundai_canfd_alt_buttons ? 0x1aaU : 0x1cfU;
    if (msg->addr == button_addr) {
      bool main_button = false;
      int cruise_button = 0;
      if (msg->addr == 0x1cfU) {
        cruise_button = msg->data[2] & 0x7U;
        main_button = GET_BIT(msg, 19U);
        mads_button_press =
            GET_BIT(msg, 23U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      } else {
        cruise_button = (msg->data[4] >> 4) & 0x7U;
        main_button = GET_BIT(msg, 34U);
        mads_button_press =
            GET_BIT(msg, 39U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      }
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((msg->addr == 0x35U) && hyundai_ev_gas_signal) {
      gas_pressed = msg->data[5] != 0U;
    } else if ((msg->addr == 0x105U) && hyundai_hybrid_gas_signal) {
      gas_pressed =
          GET_BIT(msg, 103U) || (msg->data[13] != 0U) || GET_BIT(msg, 112U);
    } else if ((msg->addr == 0x100U) && !hyundai_ev_gas_signal &&
               !hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 176U);
    } else {
    }

    // brake press
    if (msg->addr == 0x175U) {
      brake_pressed = GET_BIT(msg, 81U);
    }

    // vehicle moving
    if (msg->addr == 0xa0U) {
      uint32_t fl = (GET_BYTES(msg, 8, 2)) & 0x3FFFU;
      uint32_t fr = (GET_BYTES(msg, 10, 2)) & 0x3FFFU;
      uint32_t rl = (GET_BYTES(msg, 12, 2)) & 0x3FFFU;
      uint32_t rr = (GET_BYTES(msg, 14, 2)) & 0x3FFFU;
      vehicle_moving = (fl > HYUNDAI_STANDSTILL_THRSLD) ||
                       (fr > HYUNDAI_STANDSTILL_THRSLD) ||
                       (rl > HYUNDAI_STANDSTILL_THRSLD) ||
                       (rr > HYUNDAI_STANDSTILL_THRSLD);

      // average of all 4 wheel speeds. Conversion: raw * 0.03125 / 3.6 = m/s
      UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.03125 * KPH_TO_MS);
    }
  }

  if (msg->bus == scc_bus) {
    // cruise state
    if ((msg->addr == 0x1a0U) && !hyundai_longitudinal) {
      // 1=enabled, 2=driver override
      int cruise_status = ((msg->data[8] >> 4) & 0x7U);
      bool cruise_engaged = (cruise_status == 1) || (cruise_status == 2);
      hyundai_common_cruise_state_check(cruise_engaged);
      acc_main_on = GET_BIT(msg, 66U);
    }
  }

  hyundai_common_reset_acc_main_on_mismatches();
}

static bool hyundai_canfd_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits HYUNDAI_CANFD_STEERING_LIMITS = {
      .max_torque = 270,
      .max_rt_delta = 250,
      .max_rate_up = 4,
      .max_rate_down = 6,
      .driver_torque_allowance = 250,
      .driver_torque_multiplier = 2,
      .type = TorqueDriverLimited,

      // the EPS faults when the steering angle is above a certain threshold for
      // too long. to prevent this, we allow setting torque actuation bit to 0
      // while maintaining the requested torque value for two consecutive frames
      .min_valid_request_frames = 89,
      .max_invalid_request_frames = 2,
      .min_valid_request_rt_interval =
          810000, // 810ms; a ~10% buffer on cutting every 90 frames
      .has_steer_req_tolerance = true,
  };

  bool tx = true;

  // steering
  const unsigned int steer_addr =
      (hyundai_canfd_lka_steering && !hyundai_longitudinal)
          ? hyundai_canfd_get_lka_addr()
          : 0x12aU;
  if (msg->addr == steer_addr) {
    int desired_torque =
        (((msg->data[6] & 0xFU) << 7U) | (msg->data[5] >> 1U)) - 1024U;
    bool steer_req = GET_BIT(msg, 52U);

    if (steer_torque_cmd_checks(desired_torque, steer_req,
                                HYUNDAI_CANFD_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // cruise buttons check
  const int button_addr = hyundai_canfd_alt_buttons ? 0x1aa : 0x1cf;
  if (msg->addr == button_addr) {
    int cruise_button = 0;
    if (msg->addr == 0x1cfU) {
      cruise_button = msg->data[2] & 0x7U;
    } else {
      cruise_button = (msg->data[4] >> 4) & 0x7U;
    }

    bool is_cancel = (cruise_button == HYUNDAI_BTN_CANCEL);
    bool is_resume = (cruise_button == HYUNDAI_BTN_RESUME);
    bool is_set = (cruise_button == HYUNDAI_BTN_SET);

    bool allowed = (is_cancel && cruise_engaged_prev) ||
                   ((is_resume || is_set) && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  // UDS: only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on
  // diagnostics address
  if (((msg->addr == 0x730U) && hyundai_canfd_lka_steering) ||
      ((msg->addr == 0x7D0U) && !hyundai_camera_scc)) {
    if ((GET_BYTES(msg, 0, 4) != 0x00803E02U) ||
        (GET_BYTES(msg, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  // ACCEL: safety check
  if (msg->addr == 0x1a0U) {
    int desired_accel_raw =
        (((msg->data[17] & 0x7U) << 8) | msg->data[16]) - 1023U;
    int desired_accel_val =
        ((msg->data[18] << 4) | (msg->data[17] >> 4)) - 1023U;

    bool violation = false;

    if (hyundai_longitudinal) {
      violation |=
          longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |=
          longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    } else {
      // only used to cancel on here
      const int acc_mode = (msg->data[8] >> 4) & 0x7U;
      if (acc_mode != 4) {
        violation = true;
      }

      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }

    acc_main_on_tx = GET_BIT(msg, 66U);
    hyundai_common_acc_main_on_sync();
  }

  return tx;
}

static bool hyundai_canfd_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;
  if ((bus_num == 0) || (bus_num == 2)) {
    bool is_lkas_msg = (addr == 0x50);
    bool is_lfa_msg = (addr == 0x12A);
    bool is_lfahda_msg = (addr == 0x1E0);
    bool is_scc_msg = ((addr == 0x1a0) && hyundai_longitudinal &&
                       !hyundai_canfd_lka_steering);
    bool is_ccnc_msg = (addr == 0x161) || (addr == 0x162);

    block_msg =
        is_lkas_msg || is_lfa_msg || is_lfahda_msg || is_scc_msg || is_ccnc_msg;
  }
  return block_msg;
}

static safety_config hyundai_canfd_init(uint16_t param) {
  const uint16_t HYUNDAI_PARAM_CANFD_LKA_STEERING_ALT = 128;
  const uint16_t HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;

  // TODO: Build these more precisely like RX_CHECKS.
  static const CanMsg HYUNDAI_CANFD_LKA_STEERING_TX_MSGS[] = {
      HYUNDAI_CANFD_LKA_STEERING_COMMON_TX_MSGS(0, 1){
          0x1AA, 1, 16, .check_relay = false}, // CRUISE_BUTTONS_ALT
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEERING_ALT_TX_MSGS[] = {
      HYUNDAI_CANFD_LKA_STEERING_ALT_COMMON_TX_MSGS(0, 1){
          0x1AA, 1, 16, .check_relay = false}, // CRUISE_BUTTONS_ALT
      {0x1A0, 1, 32, .check_relay = false},    // CRUISE_INFO
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEERING_LONG_TX_MSGS[] = {
      HYUNDAI_CANFD_LKA_STEERING_COMMON_TX_MSGS(0, 1)
          HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(1)
              HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(1, true){
                  0x1AA, 1, 16, .check_relay = false}, // CRUISE_BUTTONS_ALT
      {0x51, 0, 32, .check_relay = false},             // ADRV_0x51
      {0x730, 1, 8,
       .check_relay = false}, // tester present for ADAS ECU disable
      {0x160, 1, 16, .check_relay = false}, // ADRV_0x160
      {0x1EA, 1, 32, .check_relay = false}, // ADRV_0x1ea
      {0x200, 1, 8, .check_relay = false},  // ADRV_0x200
      {0x345, 1, 8, .check_relay = false},  // ADRV_0x345
      {0x1DA, 1, 32, .check_relay = false}, // ADRV_0x1da
      {0x38C, 1, 32, .check_relay = false}, // ADRV_0x38c
  };

  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_TX_MSGS[] = {
      HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2){
          0x1AA, 2, 16, .check_relay = false}, // CRUISE_BUTTONS_ALT
      HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
          HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)};

  // ADRV_0x160 is checked for radar liveness
  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS[] = {
      HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2){
          0x1AA, 2, 16, .check_relay = false}, // CRUISE_BUTTONS_ALT
      HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
          HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true){
              0x160, 0, 16, .check_relay = true}, // ADRV_0x160
      {0x7D0, 0, 8,
       .check_relay = false}, // tester present for radar ECU disable
      {0x38C, 1, 32, .check_relay = false}, // ADRV_0x38c
  };

  // ADRV_0x160 is checked for relay malfunction
#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(longitudinal)            \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2){                                      \
      0x1AA, 2, 16, .check_relay = false}, /* CRUISE_BUTTONS_ALT */            \
      HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)                             \
          HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)){         \
              0x160, 0, 16, .check_relay = (longitudinal)}, /* ADRV_0x160 */

  hyundai_common_init(param);

  gen_crc_lookup_table_16(0x1021, hyundai_canfd_crc_lut);
  hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
  hyundai_canfd_lka_steering_alt =
      GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEERING_ALT);

  safety_config ret = hyundai_canfd_init_safety_config();

  // RX Common checks.
  const int pt_bus = hyundai_canfd_lka_steering ? 1 : 0;
  add_rx_check(
      &ret, (RxCheck){.msg = {{0x175, (pt_bus), 24, 50U, .max_counter = 0xffU,
                               .ignore_quality_flag = true},
                              {0},
                              {0}}});
  add_rx_check(
      &ret, (RxCheck){.msg = {{0xa0, (pt_bus), 24, 100U, .max_counter = 0xffU,
                               .ignore_quality_flag = true},
                              {0},
                              {0}}});
  add_rx_check(
      &ret, (RxCheck){.msg = {{0xea, (pt_bus), 24, 100U, .max_counter = 0xffU,
                               .ignore_quality_flag = true},
                              {0},
                              {0}}});

  if (hyundai_ev_gas_signal) {
    add_rx_check(
        &ret, (RxCheck){.msg = {{0x35, (pt_bus), 32, 100U, .max_counter = 0xffU,
                                 .ignore_quality_flag = true},
                                {0},
                                {0}}});
  } else if (hyundai_hybrid_gas_signal) {
    add_rx_check(&ret, (RxCheck){.msg = {{0x105, (pt_bus), 32, 100U,
                                          .max_counter = 0xffU,
                                          .ignore_quality_flag = true},
                                         {0},
                                         {0}}});
  } else {
    add_rx_check(&ret, (RxCheck){.msg = {{0x100, (pt_bus), 32, 100U,
                                          .max_counter = 0xffU,
                                          .ignore_quality_flag = true},
                                         {0},
                                         {0}}});
  }

  // RX Cruise buttons.
  if (hyundai_canfd_alt_buttons) {
    add_rx_check(
        &ret, (RxCheck){.msg = {{0x1aa, (pt_bus), 16, 50U, .max_counter = 0xffU,
                                 .ignore_checksum = true,
                                 .ignore_quality_flag = true},
                                {0},
                                {0}}});
  } else {
    // 0x1cf check_checksum=false => ignore_checksum=true
    add_rx_check(
        &ret, (RxCheck){.msg = {{0x1cf, (pt_bus), 8, 50U, .max_counter = 0xfU,
                                 .ignore_checksum = true,
                                 .ignore_quality_flag = true},
                                {0},
                                {0}}});
  }

  if (hyundai_longitudinal) {
    // TX SCC_CONTROL.
  } else {
    // RX SCC_CONTROL.
    const int scc_bus = hyundai_canfd_lka_steering ? 1
                        : hyundai_camera_scc       ? 2
                                                   : 0;
    add_rx_check(&ret, (RxCheck){.msg = {{0x1a0, (scc_bus), 32, 50U,
                                          .max_counter = 0xffU,
                                          .ignore_quality_flag = true},
                                         {0},
                                         {0}}});
  }

  // TX messages
  if (hyundai_longitudinal) {
    if (hyundai_canfd_lka_steering) {
      // LKA Steering Long (HDA2 Long)
      // Checks: Common (done above)
      SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEERING_LONG_TX_MSGS, ret);
    } else {
      // LFA Steering Long (HDA1 Long)
      static const CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
          HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(true)};

      if (hyundai_camera_scc) {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS, ret);
      }
    }
  } else {
    if (hyundai_canfd_lka_steering) {
      if (hyundai_canfd_lka_steering_alt) {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEERING_ALT_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEERING_TX_MSGS, ret);
      }
    } else {
      static const CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
          HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(false)};
      // LFA Steering (HDA1)
      if (hyundai_camera_scc) {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_TX_MSGS, ret);
      }
    }
  }

  return ret;
}

const safety_hooks hyundai_canfd_hooks = {
    .init = hyundai_canfd_init,
    .rx = hyundai_canfd_rx_hook,
    .tx = hyundai_canfd_tx_hook,
    .fwd = hyundai_canfd_fwd_hook,
    .get_counter = hyundai_canfd_get_counter,
    .get_checksum = hyundai_canfd_get_checksum,
    .compute_checksum = hyundai_common_canfd_compute_checksum,
};
