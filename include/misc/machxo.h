/*
 * Definitions for programming Lattice MachXO2 FPGA's.
 * Copyright (c) 2013 Bjarne Steinsbo <bjarne at steinsbo dot com>
 * License: http://www.gnu.org/licenses/gpl.html GPL version 2 or higher
 */

#ifndef _COMMANDS_H
#define _COMMANDS_H 1

#define IDCODE_PUB 0xE0
#define ISC_ENABLE_X 0x74
#define ISC_ENABLE 0xC6
#define LSC_CHECK_BUSY 0xF0
#define LSC_READ_STATUS 0x3C
# define READ_STATUS_BUSY(x) ((x) & 0x00001000)
# define READ_STATUS_FAIL(x) ((x) & 0x00002000)
# define READ_STATUS_DONE(x) ((x) & 0x00000100)
#define ISC_ERASE 0x0E
# define ERASE_FEATURE_ROW 0x00020000
# define ERASE_CONFIGURATION 0x00040000
# define ERASE_USER_FLASH 0x00080000
#define LSC_ERASE_TAG 0xCB
#define LSC_INIT_ADDRESS 0x46
#define LSC_WRITE_ADDRESS 0xB4
#define LSC_PROG_INCR_NV 0x70
#define LSC_INIT_ADDR_UFM 0x47
#define LSC_PROG_TAG 0xC9
#define ISC_PROGRAM_USERCODE 0xC2
#define USERCODE 0xC0
#define LSC_PROG_FEATURE 0xE4
#define LSC_READ_FEATURE 0xE7
#define LSC_PROG_FEABITS 0xF8
#define LSC_READ_FEABITS 0xFB
#define LSC_READ_INCR_NV 0x73
#define LSC_READ_UFM 0xCA
#define ISC_PROGRAM_DONE 0x5E
#define LSC_PROG_OTP 0xF9
#define LSC_READ_OTP 0xFA
#define ISC_DISABLE 0x26 
#define ISC_NOOP 0xFF
#define LSC_REFRESH 0x79
#define ISC_PROGRAM_SECURITY 0xCE
#define ISC_PROGRAM_SECPLUS 0xCF
#define UIDCODE_PUB 0x19
#define LSC_BITSTREAM_BURST 0x7A
#define LSC_PROG_CTRL0 0x22
#define LSC_READ_CTRL0 0x20
#define LSC_PROG_INCR_RTI 0x82
#define KEY_ACTIVATION 0xA4
#define LSC_VERIFY_INCR_RTI 0x6A

#define DIRECTION_SEND 0
#define DIRECTION_RECEIVE 1

#define DEFAULT_I2C_DEV 0

#endif
