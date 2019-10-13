#ifndef _COMMS_DEFS_H_
#define _COMMS_DEFS_H_

#include <stdint.h>

namespace motor_driver {
namespace comms {

constexpr uint8_t COMM_VERSION = 0xfe;

using comm_errors_t = uint16_t;
constexpr comm_errors_t COMM_ERRORS_NONE = 0;
constexpr comm_errors_t COMM_ERRORS_OP_FAILED = 1;
constexpr comm_errors_t COMM_ERRORS_MALFORMED = 2;
constexpr comm_errors_t COMM_ERRORS_INVALID_FC = 4;
constexpr comm_errors_t COMM_ERRORS_INVALID_ARGS = 8;
constexpr comm_errors_t COMM_ERRORS_BUF_LEN_MISMATCH = 16;

using comm_id_t = uint8_t;
constexpr comm_id_t COMM_ID_BROADCAST = 0;
constexpr comm_id_t COMM_ID_MIN = 1;
constexpr comm_id_t COMM_ID_MAX = UINT8_MAX;

// Flag Bits (These are toggle bits so each flag should only change one bit)
using comm_fg_t = uint8_t;
constexpr comm_fg_t COMM_FG_COMP    = 0b00000000;
constexpr comm_fg_t COMM_FG_BOARD   = 0b00000001;
constexpr comm_fg_t COMM_FG_RESET   = 0b00000010;
constexpr comm_fg_t COMM_FG_TIMEOUT = 0b00000100;

using comm_fc_t = uint8_t;
constexpr comm_fc_t COMM_FC_NOP = 0x00;
constexpr comm_fc_t COMM_FC_REG_READ = 0x01;
constexpr comm_fc_t COMM_FC_REG_WRITE = 0x02;
constexpr comm_fc_t COMM_FC_REG_READ_WRITE = 0x03;
constexpr comm_fc_t COMM_FC_CLEAR_IWDGRST = 0x10;
constexpr comm_fc_t COMM_FC_SYSTEM_RESET = 0x80;
constexpr comm_fc_t COMM_FC_JUMP_TO_ADDR = 0x81;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_COUNT = 0x82;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_START = 0x83;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_SIZE = 0x84;
constexpr comm_fc_t COMM_FC_FLASH_SECTOR_ERASE = 0x85;
constexpr comm_fc_t COMM_FC_FLASH_PROGRAM = 0x86;
constexpr comm_fc_t COMM_FC_FLASH_READ = 0x87;
constexpr comm_fc_t COMM_FC_FLASH_VERIFY = 0x88;
constexpr comm_fc_t COMM_FC_FLASH_VERIFY_ERASED = 0x89;
constexpr comm_fc_t COMM_FC_CONFIRM_ID = 0xFE;
constexpr comm_fc_t COMM_FC_ENUMERATE = 0xFF;

using comm_addr_t = uint16_t;
// Info/System Register (0x0***)
constexpr comm_addr_t COMM_REG_SYS_MAP_VERSION = 0x0000;
constexpr comm_addr_t COMM_REG_SYS_BOARD_ID = 0x0001;
constexpr comm_addr_t COMM_REG_SYS_FW_VERSION = 0x0002;
constexpr comm_addr_t COMM_REG_SYS_BL_VERSION = 0x0003;
constexpr comm_addr_t COMM_REG_SYS_STORE_CALIBRATION = 0x0004;
constexpr comm_addr_t COMM_REG_SYS_CLEAR_CALIBRATION = 0x0005;

// Calibration Register (0x1***)
constexpr comm_addr_t COMM_REG_CAL_REV_START = 0x1000;
constexpr comm_addr_t COMM_REG_CAL_EREVS_PER_MREV = 0x1001;
constexpr comm_addr_t COMM_REG_CAL_INV_PHASES = 0x1002;
constexpr comm_addr_t COMM_REG_CAL_DI_KP = 0x1003;
constexpr comm_addr_t COMM_REG_CAL_DI_KI = 0x1004;
constexpr comm_addr_t COMM_REG_CAL_QI_KP = 0x1005;
constexpr comm_addr_t COMM_REG_CAL_QI_KI = 0x1006;
constexpr comm_addr_t COMM_REG_CAL_V_KP = 0x1007;
constexpr comm_addr_t COMM_REG_CAL_V_KI = 0x1008;
constexpr comm_addr_t COMM_REG_CAL_P_KP = 0x1009;
constexpr comm_addr_t COMM_REG_CAL_P_KI = 0x100a;
constexpr comm_addr_t COMM_REG_CAL_I_LIMIT = 0x1010;
constexpr comm_addr_t COMM_REG_CAL_T_LIMIT = 0x1011;
constexpr comm_addr_t COMM_REG_CAL_V_LIMIT = 0x1012;
constexpr comm_addr_t COMM_REG_CAL_POS_L_LIMIT = 0x1013;
constexpr comm_addr_t COMM_REG_CAL_POS_U_LIMIT = 0x1014;
constexpr comm_addr_t COMM_REG_CAL_POS_OFFSET = 0x1015;
constexpr comm_addr_t COMM_REG_CAL_MOTOR_R = 0x1020; // Resistance
constexpr comm_addr_t COMM_REG_CAL_MOTOR_H = 0x1021; // Inductance
constexpr comm_addr_t COMM_REG_CAL_MOTOR_T = 0x1022; // Torque Constant
constexpr comm_addr_t COMM_REG_CAL_WATCHDOG = 0x1030;
constexpr comm_addr_t COMM_REG_CAL_V_FILTER = 0x1040;
constexpr comm_addr_t COMM_REG_CAL_EAC_SCALE = 0x1100;
constexpr comm_addr_t COMM_REG_CAL_EAC_OFFSET = 0x1101;
constexpr comm_addr_t COMM_REG_CAL_EAC_TABLE = 0x1200;
constexpr comm_addr_t COMM_REG_CAL_IA_OFF = 0x1050;
constexpr comm_addr_t COMM_REG_CAL_IB_OFF = 0x1051;
constexpr comm_addr_t COMM_REG_CAL_IC_OFF = 0x1052;

// Volatile Registers (0x2***)
constexpr comm_addr_t COMM_REG_VOL_CTRL_MODE = 0x2000;
constexpr comm_addr_t COMM_REG_VOL_DI_COMM = 0x2001;
constexpr comm_addr_t COMM_REG_VOL_QI_COMM = 0x2002;
constexpr comm_addr_t COMM_REG_VOL_PHASE_A_PWM = 0x2003;
constexpr comm_addr_t COMM_REG_VOL_PHASE_B_PWM = 0x2004;
constexpr comm_addr_t COMM_REG_VOL_PHASE_C_PWM = 0x2005;
constexpr comm_addr_t COMM_REG_VOL_SETPOINT_T = 0x2006;
constexpr comm_addr_t COMM_REG_VOL_SETPOINT_V = 0x2007;
constexpr comm_addr_t COMM_REG_VOL_SETPOINT_P = 0x2008;
constexpr comm_addr_t COMM_REG_VOL_SETPOINT_FF = 0x2009;
constexpr comm_addr_t COMM_REG_VOL_PWM_DRIVE = 0x200A;
// Read Only Registers (0x3***)
constexpr comm_addr_t COMM_REG_RO_ROTOR_P = 0x3000;
constexpr comm_addr_t COMM_REG_RO_ROTOT_V = 0x3001;
constexpr comm_addr_t COMM_REG_RO_DI_MEAS = 0x3002;
constexpr comm_addr_t COMM_REG_RO_QC_MEAS = 0x3003;
constexpr comm_addr_t COMM_REG_RO_DC_SUPPLY_V = 0x3004;
constexpr comm_addr_t COMM_REG_RO_TEMP = 0x3005;
constexpr comm_addr_t COMM_REG_RO_ACCEL_X = 0x3006;
constexpr comm_addr_t COMM_REG_RO_ACCEL_Y = 0x3007;
constexpr comm_addr_t COMM_REG_RO_ACCEL_Z = 0x3008;
constexpr comm_addr_t COMM_REG_RO_REC_START = 0x3009;
constexpr comm_addr_t COMM_REG_RO_REC_LEN = 0x300a;
constexpr comm_addr_t COMM_REG_RO_REC_RESET = 0x300b;
constexpr comm_addr_t COMM_REG_RO_ROTOR_P_RAW = 0x3010;
constexpr comm_addr_t COMM_REG_RO_ROTOR_REVS = 0x3011;



using comm_reg_count_t = uint8_t;

} // namespace comms
} // namespace motor_driver

#endif /* _COMMS_DEFS_H_ */
