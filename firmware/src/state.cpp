#include "state.h"

#include "flash.h"
#include "helper.h"
#include "comms_defs.h"

#include <cstring>

namespace motor_driver {
namespace state {

Results results;

Calibration calibration;

Parameters parameters;

Recorder recorder;

volatile bool should_copy_results = false;

volatile bool should_copy_parameters = false;

constexpr static uint16_t calib_buf_size = 
                      state::num_calibrations * state::num_bytes_per_calib + 
                      sizeof(state::Calibration) +
                      sizeof(consts::calib_ss); // For the start sequence
static uint8_t calib_buf[calib_buf_size];

void initState() {
}

void storeCalibration() {
  uint32_t addr = reinterpret_cast<uintptr_t>(consts::calibration_ptr);

  packCalibration(calib_buf, state::calibration);

  struct IWDG_Values save = pauseIWDG();
  flashErase(addr, sizeof(state::Calibration));
  resumeIWDG(save);

  flashWrite(addr, (char*) calib_buf, calib_buf_size);
}

void loadCalibration() {
  uint32_t addr = reinterpret_cast<uintptr_t>(consts::calibration_ptr);
  uint16_t start_sequence = 0;
  // Retry loading flash until successful load. This is critical to read.
  while (not (flashRead(addr, (char *)&start_sequence, sizeof(uint16_t)) == FLASH_RETURN_SUCCESS));
  if (start_sequence == consts::calib_ss) {
    while (not (flashRead(addr, (char*) calib_buf, calib_buf_size) == FLASH_RETURN_SUCCESS));
  }
}

void clearCalibration() {
  // Copy default values into calibration.
  state::Calibration temp_calib;
  std::memcpy(&state::calibration, &temp_calib, sizeof(state::Calibration));
}

static void write_buf(uint8_t *buf, const uint8_t *data, uint8_t num_bytes) {
  for (uint8_t i = 0; i < num_bytes; i++)
    buf[i] = data[i];
}

template <typename T>
static void write_calibration (uint8_t **buf, comms::comm_addr_t addr, uint16_t size, const T &data) {
  write_buf(*buf, (uint8_t*) &addr, sizeof(addr));
  *buf += sizeof(addr);
  write_buf(*buf, (uint8_t*) &size, sizeof(size));
  *buf += sizeof(size);
  write_buf(*buf, (uint8_t*) &data, size);
  *buf += size;
}

/** 
 * --Pack Calibration-- takes in a calibration struct and packs the data into a 
 * buffer in a recoverable way. This works by using the addresses defined in
 * consts.cpp to identify each data type. The packed data has the following fields:
 *
 *  d1 -> <address (2 bytes)> <size (2 bytes)> <data (n bytes)>
 *  d2 -> <address (2 bytes)> <size (2 bytes)> <data (n bytes)>
 *
 */
#define WC(ADDRESS, MEMBER) write_calibration(bpp, comms::ADDRESS, sizeof(calibration.MEMBER), calibration.MEMBER)
void packCalibration(uint8_t *buf, const state::Calibration &calibration) {
  uint8_t *bp = buf;
  uint8_t **bpp = &bp;

  write_buf(bp, (uint8_t*) &consts::calib_ss, sizeof(consts::calib_ss));
  bpp += sizeof(consts::calib_ss);

  WC(COMM_REG_CAL_REV_START, erev_start);
  WC(COMM_REG_CAL_EREVS_PER_MREV, erevs_per_mrev);
  WC(COMM_REG_CAL_INV_PHASES, flip_phases);

  WC(COMM_REG_CAL_DI_KP, foc_kp_d);
  WC(COMM_REG_CAL_DI_KI, foc_ki_d);
  WC(COMM_REG_CAL_QI_KP, foc_kp_q);
  WC(COMM_REG_CAL_QI_KI, foc_ki_q);

  WC(COMM_REG_CAL_V_KP, velocity_kp);
  WC(COMM_REG_CAL_V_KI, velocity_kd);
  WC(COMM_REG_CAL_P_KP, position_kp);
  WC(COMM_REG_CAL_P_KI, position_kd);

  WC(COMM_REG_CAL_I_LIMIT, current_limit);
  WC(COMM_REG_CAL_T_LIMIT, torque_limit);
  WC(COMM_REG_CAL_V_LIMIT, velocity_limit);

  WC(COMM_REG_CAL_POS_L_LIMIT, position_lower_limit);
  WC(COMM_REG_CAL_POS_U_LIMIT, position_upper_limit);
  WC(COMM_REG_CAL_POS_OFFSET, position_offset);
 
  WC(COMM_REG_CAL_MOTOR_R, motor_resistance);
  WC(COMM_REG_CAL_MOTOR_H, motor_inductance);
  WC(COMM_REG_CAL_MOTOR_T, motor_torque_const);

  WC(COMM_REG_CAL_WATCHDOG, control_timeout);
  WC(COMM_REG_CAL_HF_V_FILTER, hf_velocity_filter_param);
  WC(COMM_REG_CAL_LF_V_FILTER, lf_velocity_filter_param);

  WC(COMM_REG_CAL_IA_OFF, ia_offset);
  WC(COMM_REG_CAL_IB_OFF, ib_offset);
  WC(COMM_REG_CAL_IC_OFF, ic_offset);

  WC(COMM_REG_CAL_EAC_SCALE, enc_ang_corr_scale);
  WC(COMM_REG_CAL_EAC_OFFSET, enc_ang_corr_offset);
  write_calibration(bpp, comms::COMM_REG_CAL_EAC_TABLE, 
                    consts::enc_ang_corr_table_size * sizeof(calibration.enc_ang_corr_table_values[0]), 
                    calibration.enc_ang_corr_table_values[0]);

  comms::comm_addr_t null_addr = 0;
  write_buf(bp, (uint8_t*) &null_addr, sizeof(null_addr));
}

static void read_buf(const uint8_t *buf, uint8_t *data, uint8_t num_bytes) {
  for (uint8_t i = 0; i < num_bytes; i++)
    data[i] = buf[i];
}

// While reading in calibrations, don't read if the sizes don't match.
template <typename T>
static void read_calibration (const uint8_t** buf, uint16_t read_size, uint16_t expected_size, T &data) {
  if (expected_size == read_size)
    read_buf(*buf, (uint8_t*) &data, read_size);
  *buf += read_size;
}

/** 
 * --Unpack Calibration-- takes in a calibration buffer and unpacks the data into a 
 * calibration struct. This is more tricky than packing as it is assumed the order of 
 * packing is unknown.This works by using the addresses defined in
 * consts.cpp to identify each data type. The packed data has the following fields:
 *
 *  d1 -> <address (2 bytes)> <size (2 bytes)> <data (n bytes)>
 *  d2 -> <address (2 bytes)> <size (2 bytes)> <data (n bytes)>
 *
 */
 #define RC(MEMBER) read_calibration(bpp, size, sizeof(calibration.MEMBER), calibration.MEMBER)
bool unpackCalibration(const uint8_t *buf, state::Calibration &calibration) {
  uint16_t calib_check = 0;
  const uint8_t *bp = buf;
  const uint8_t **bpp = &bp;

  read_buf(bp, (uint8_t*) &calib_check, sizeof(calib_check));
  bp += sizeof(calib_check);

  // Check if the calibration buffer is valid.
  if (consts::calib_ss != calib_check)
    return false;

  comms::comm_addr_t addr;
  read_buf(bp, (uint8_t*) &addr, sizeof(addr));
  bp += sizeof(addr);

  uint16_t size;
  while (addr != 0) {
    read_buf(bp, (uint8_t*) &size, sizeof(size));
    bp += sizeof(size);

    switch (addr) {
      case comms::COMM_REG_CAL_REV_START:
        RC(erev_start);
        break;
      case comms::COMM_REG_CAL_EREVS_PER_MREV:
        RC(erevs_per_mrev);
        break;
      case comms::COMM_REG_CAL_INV_PHASES:
        RC(flip_phases);
        break;

      case comms::COMM_REG_CAL_DI_KP:
        RC(foc_kp_d);
        break;
      case comms::COMM_REG_CAL_DI_KI:
        RC(foc_ki_d);
        break;
      case comms::COMM_REG_CAL_QI_KP:
        RC(foc_kp_q);
        break;
      case comms::COMM_REG_CAL_QI_KI:
        RC(foc_ki_q);
        break;

      case comms::COMM_REG_CAL_V_KP:
        RC(velocity_kp);
        break;
      case comms::COMM_REG_CAL_V_KI:
        RC(velocity_kd);
        break;
      case comms::COMM_REG_CAL_P_KP:
        RC(position_kp);
        break;
      case comms::COMM_REG_CAL_P_KI:
        RC(position_kd);
        break;

      case comms::COMM_REG_CAL_I_LIMIT:
        RC(current_limit);
        break;
      case comms::COMM_REG_CAL_T_LIMIT:
        RC(torque_limit);
        break;
      case comms::COMM_REG_CAL_V_LIMIT:
        RC(velocity_limit);
        break;

      case comms::COMM_REG_CAL_POS_L_LIMIT:
        RC(position_lower_limit);
        break;
      case comms::COMM_REG_CAL_POS_U_LIMIT:
        RC(position_upper_limit);
        break;
      case comms::COMM_REG_CAL_POS_OFFSET:
        RC(position_offset);
        break;

      case comms::COMM_REG_CAL_MOTOR_R:
        RC(motor_resistance);
        break;
      case comms::COMM_REG_CAL_MOTOR_H:
        RC(motor_inductance);
        break;
      case comms::COMM_REG_CAL_MOTOR_T:
        RC(motor_torque_const);
        break;

      case comms::COMM_REG_CAL_WATCHDOG:
        RC(control_timeout);
        break;
      case comms::COMM_REG_CAL_HF_V_FILTER:
        RC(hf_velocity_filter_param);
        break;
      case comms::COMM_REG_CAL_LF_V_FILTER:
        RC(lf_velocity_filter_param);
        break;

      case comms::COMM_REG_CAL_IA_OFF:
        RC(ia_offset);
        break;
      case comms::COMM_REG_CAL_IB_OFF:
        RC(ib_offset);
        break;
      case comms::COMM_REG_CAL_IC_OFF:
        RC(ic_offset);
        break;

      case comms::COMM_REG_CAL_EAC_SCALE:
        RC(enc_ang_corr_scale);
        break;
      case comms::COMM_REG_CAL_EAC_OFFSET:
        RC(enc_ang_corr_offset);
        break;
      case comms::COMM_REG_CAL_EAC_TABLE:
        read_calibration(bpp, size, 
                        consts::enc_ang_corr_table_size * sizeof(calibration.enc_ang_corr_table_values[0]), 
                        calibration.enc_ang_corr_table_values[0]);
        break;

      /* If the address does not exist, jump over the stored amount 
         (compatibility support) */
      default:
        bp += size;
        break;
    }

    read_buf(bp, (uint8_t*) &addr, sizeof(addr));
    bp += sizeof(addr);
  }

  return true;
}

} // namespace state
} // namespace motor_driver
