/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file .h
 *
 * Shared defines for the icm20602 driver.
 */

#pragma once

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define ICM_DEVICE_PATH_ACCEL		"/dev/ICM20602_accel"
#define ICM_DEVICE_PATH_GYRO		"/dev/ICM20602_gyro"
#define ICM_DEVICE_PATH_ACCEL_EXT	"/dev/ICM20602_accel_ext"
#define ICM_DEVICE_PATH_GYRO_EXT	"/dev/ICM20602_gyro_ext"

// ICM20602 registers
#define ICMREG_WHOAMI			0x75
#define ICMREG_SMPLRT_DIV		0x19
#define ICMREG_CONFIG			0x1A
#define ICMREG_GYRO_CONFIG		0x1B
#define ICMREG_ACCEL_CONFIG		0x1C
#define ICMREG_ACCEL_CONFIG2	0x1D

#define ICMREG_FIFO_EN			0x23
#define ICMREG_INT_PIN_CFG		0x37
#define ICMREG_INT_ENABLE		0x38
#define ICMREG_INT_STATUS		0x3A
#define ICMREG_ACCEL_XOUT_H		0x3B
#define ICMREG_ACCEL_XOUT_L		0x3C
#define ICMREG_ACCEL_YOUT_H		0x3D
#define ICMREG_ACCEL_YOUT_L		0x3E
#define ICMREG_ACCEL_ZOUT_H		0x3F
#define ICMREG_ACCEL_ZOUT_L		0x40
#define ICMREG_TEMP_OUT_H		0x41
#define ICMREG_TEMP_OUT_L		0x42
#define ICMREG_GYRO_XOUT_H		0x43
#define ICMREG_GYRO_XOUT_L		0x44
#define ICMREG_GYRO_YOUT_H		0x45
#define ICMREG_GYRO_YOUT_L		0x46
#define ICMREG_GYRO_ZOUT_H		0x47
#define ICMREG_GYRO_ZOUT_L		0x48
#define ICMREG_USER_CTRL		0x6A
#define ICMREG_PWR_MGMT_1		0x6B
#define ICMREG_PWR_MGMT_2		0x6C
#define ICMREG_FIFO_COUNTH		0x72
#define ICMREG_FIFO_COUNTL		0x73
#define ICMREG_FIFO_R_W			0x74
#define ICMREG_GYRO_SELFTEST_X	0x50
#define ICMREG_GYRO_SELFTEST_Y	0x51
#define ICMREG_GYRO_SELFTEST_Z	0x52
#define ICMREG_ACCEL_SELFTEST_X	0x0D
#define ICMREG_ACCEL_SELFTEST_Y	0x0E
#define ICMREG_ACCEL_SELFTEST_Z	0x0F

// Configuration bits ICM20602
#define BIT_SLEEP					0x40
#define BIT_H_RESET					0x80
#define BITS_CLKSEL					0x07
#define BITS_INTERNAL_20MHZ			0x00
#define BITS_BESTCLOCK_PLL			0x01
#define BITS_BESTCLOCK_PLL2			0x02
#define BITS_BESTCLOCK_PLL3			0x03
#define BITS_BESTCLOCK_PLL4			0x04
#define BITS_BESTCLOCK_PLL5			0x05
#define BITS_INTERNAL_20MHZ2		0x06
#define BITS_STOPCLOCK_RESET		0x07
#define BITS_FS_250DPS				0x00
#define BITS_FS_500DPS				0x08
#define BITS_FS_1000DPS				0x10
#define BITS_FS_2000DPS				0x18
#define BITS_FS_MASK				0x18
#define BITS_DLPF_CFG_4000HZ_NOLPF2	0x00
#define BITS_DLPF_CFG_188HZ			0x01
#define BITS_DLPF_CFG_98HZ			0x02
#define BITS_DLPF_CFG_42HZ			0x03
#define BITS_DLPF_CFG_20HZ			0x04
#define BITS_DLPF_CFG_10HZ			0x05
#define BITS_DLPF_CFG_5HZ			0x06
#define BITS_DLPF_CFG_4000HZ_NOLPF	0x07
#define BITS_DLPF_CFG_MASK			0x07
#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_DATA_RDY_INT_EN			0x00

#define ICM20602_WHO_AM_I			0x12


#define ICM20602_ACCEL_DEFAULT_RANGE_G			8
#define ICM20602_ACCEL_DEFAULT_RATE				1000
#define ICM20602_ACCEL_MAX_OUTPUT_RATE			280
#define ICM20602_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define ICM20602_GYRO_DEFAULT_RANGE_G			8
#define ICM20602_GYRO_DEFAULT_RATE				1000
/* rates need to be the same between accel and gyro */
#define ICM20602_GYRO_MAX_OUTPUT_RATE			ICM20602_ACCEL_MAX_OUTPUT_RATE
#define ICM20602_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define ICM20602_DEFAULT_ONCHIP_FILTER_FREQ		42

#define ICM20602_ONE_G					9.80665f

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

#pragma pack(push, 1)
/**
 * Report conversation within the ICM20602, including command byte and
 * interrupt status.
 */
struct ICMReport {
	uint8_t		cmd;
	uint8_t		status;
	uint8_t		accel_x[2];
	uint8_t		accel_y[2];
	uint8_t		accel_z[2];
	uint8_t		temp[2];
	uint8_t		gyro_x[2];
	uint8_t		gyro_y[2];
	uint8_t		gyro_z[2];
};
#pragma pack(pop)

#define ICM_MAX_READ_BUFFER_SIZE (sizeof(ICMReport) + 1)
#define ICM_MAX_WRITE_BUFFER_SIZE (2)
/*
  The ICM20602 can only handle high bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  Communication with all registers of the device is performed using SPI at 1MHz.
  For applications requiring faster communications,
  the sensor and interrupt registers may be read using SPI at 20MHz
 */
#define ICM20602_LOW_BUS_SPEED				0
#define ICM20602_HIGH_BUS_SPEED				0x8000
#  define ICM20602_IS_HIGH_SPEED(r) 		((r) & ICM20602_HIGH_BUS_SPEED)
#  define ICM20602_REG(r) 					((r) &~ICM20602_HIGH_BUS_SPEED)
#  define ICM20602_SET_SPEED(r, s) 			((r)|(s))
#  define ICM20602_HIGH_SPEED_OP(r) 		ICM20602_SET_SPEED((r), ICM20602_HIGH_BUS_SPEED)
#  define ICM20602_LOW_SPEED_OP(r)			ICM20602_REG((r))

/* interface factories */
extern device::Device *ICM20602_SPI_interface(int bus, int device_type, bool external_bus);
extern int ICM20602_probe(device::Device *dev, int device_type);

typedef device::Device *(*ICM20602_constructor)(int, int, bool);
