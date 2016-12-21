/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
 *         Author: David Sidrane <david_s5@nscdg.com>
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
 * @file board_config.h
 *
 * bat12-v1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <chip.h>
#include <stm32_gpio.h>
#include <arch/board/board.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */
/*                              Port Connector FMUv5 Delta */
#define GPIO_LED1              /* PB1 TIM3_CH4             */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_LED2              /* PC6 TIM3_CH1             */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define GPIO_LED3              /* PC7 TIM3_CH2             */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)

#define GPIO_LED_RED 	GPIO_LED1
#define GPIO_LED_GREEN 	GPIO_LED2
#define GPIO_LED_BLUE   GPIO_LED3

/*  Define the Chip Selects */

#define GPIO_SPI_CS_IMU         /* PF10 IMU         */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN10)
#define GPIO_SPI_CS_MS5611      /* PF3  MS5607      */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN3)
#define GPIO_SPI_CS_DPS310      /* PF2  DPS310      */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN2)
#define GPIO_SPI_CS_FRAM        /* PF5  Unknown Dev */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN5)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

#define GPIO_SPI_CS_OFF_IMU 		_PIN_OFF(GPIO_SPI_CS_IMU)
#define GPIO_SPI_CS_OFF_MS5611		_PIN_OFF(GPIO_SPI_CS_MS5611)
#define GPIO_SPI_CS_OFF_DPS310      _PIN_OFF(GPIO_SPI_CS_DPS310)


/* SPI1 off */
#define GPIO_SPI1_SCK_OFF   _PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF  _PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF  _PIN_OFF(GPIO_SPI1_MOSI)

/* SPI4 off */
#define GPIO_SPI4_SCK_OFF   _PIN_OFF(GPIO_SPI4_SCK)
#define GPIO_SPI4_MISO_OFF  _PIN_OFF(GPIO_SPI4_MISO)
#define GPIO_SPI4_MOSI_OFF  _PIN_OFF(GPIO_SPI4_MOSI)

/*
 * MS5611, DPS310 is on bus SPI1
 * FRAM is on bus SPI2
 * IMU is on SPI4
 */
#define PX4_SPI_BUS_BARO    1
#define PX4_SPI_BUS_RAMTRON 2
#define PX4_SPI_BUS_SENSORS 4

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 */
/*                              BUS:DEV For clarity and indexing
 */
#define PX4_MK_SPI_SEL(b,d)      ((((b) & 0xf) << 4) + ((d) & 0xf))
#define PX4_SPI_BUS_ID(bd)       (((bd) >> 4) & 0xf)
#define PX4_SPI_DEV_ID(bd)       ((bd) & 0xf)

#define PX4_SPIDEV_GYRO			 PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,0)
#define PX4_SPIDEV_ACCEL_MAG	 PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,1)
#define PX4_SPIDEV_MPU			 PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,2)
#define PX4_SPIDEV_IMU			 PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,2)

#define PX4_SENSOR_BUS_CS_GPIO   {0, 0, GPIO_SPI_CS_IMU}
#define PX4_SENSORS_BUS_FIRST_CS PX4_SPIDEV_GYRO
#define PX4_SENSORS_BUS_LAST_CS  PX4_SPIDEV_MPU

#define PX4_SPIDEV_FRAM          PX4_MK_SPI_SEL(PX4_SPI_BUS_RAMTRON,0)
#define PX4_RAMTRON_BUS_CS_GPIO  {GPIO_SPI_CS_FRAM}
#define PX4_RAMTRON_BUS_FIRST_CS PX4_SPIDEV_FRAM
#define PX4_RAMTRON_BUS_LAST_CS  PX4_SPIDEV_FRAM

#define PX4_SPIDEV_BARO          PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,0)
#define PX4_SPIDEV_BARO2         PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,1)
#define PX4_BARO_BUS_CS_GPIO     {GPIO_SPI_CS_MS5611, GPIO_SPI_CS_DPS310}
#define PX4_BARO_BUS_FIRST_CS    PX4_SPIDEV_BARO
#define PX4_BARO_BUS_LAST_CS     PX4_SPIDEV_BARO


/* I2C busses */
#define PX4_I2C_BUS_ONBOARD     2
#define PX4_I2C_BUS_EXPANSION   1
#define PX4_I2C_BUS_EXPANSION1  3
#define PX4_I2C_BUS_EXPANSION2  4
#define PX4_I2C_BUS_LED			PX4_I2C_BUS_EXPANSION

#define PX4_I2C_BUS_SET {PX4_I2C_BUS_ONBOARD, PX4_I2C_BUS_EXPANSION, PX4_I2C_BUS_EXPANSION1, PX4_I2C_BUS_EXPANSION2, -1}
/* Devices on the external bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_LED	    0x55
#define PX4_I2C_OBDEV_HMC5883	0x1e
#define PX4_I2C_OBDEV_LIS3MDL	0x1e

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 0) | (1 << 10) | (1 << 11)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL		0
#define ADC_BATTERY_CURRENT_CHANNEL ((uint8_t)(-1))
#define ADC_5V_RAIL_SENSE               10
#define ADC_3V3_RAIL_SENSE              11

/* CAN Silence
 *
 * Silent mode control input to TJA1057
 * Active HI is Silent
 */
//#define GPIO_CAN1_SILENCE  /* PH2  U1 CAN_S1 */(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
//#define GPIO_CAN2_SILENCE  /* PH3  U3 CAN_S1 */(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN3)
//#define GPIO_CAN3_SILENCE  /* PG10 U5 CAN_S1 */(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN10)


/* PWM
 *
 * 4  PWM outputs are configured.
 *
 * Pins:
 *
 * PWM_1 FMU_CH4 : PE9  : TIM1_CH1
 * PWM_2 FMU_CH3 : PE11 : TIM1_CH2
 * PWM_3 Buzzer  : PH6  : TIM12_CH1
 * PWM_4 FMU_CH1 : PE14 : TIM1_CH4
 *
 */
#define GPIO_TIM1_CH1OUT        /* PE9  JP1-5  T1C1  FMU4  PWM_1 */ GPIO_TIM1_CH1OUT_2
#define GPIO_TIM1_CH2OUT        /* PE11 JP1-7  T1C2  FMU3  PWM_2 */ GPIO_TIM1_CH2OUT_2
#define GPIO_TIM12_CH1OUT       /* PH6 JP1-9   T12C1 BUZ1  PWM_3 */ GPIO_TIM12_CH1OUT_2
#define GPIO_TIM1_CH4OUT        /* PE14 JP1-11 T1C4  FMU1  PWM_4 */ GPIO_TIM1_CH4OUT_2
#define DIRECT_PWM_OUTPUT_CHANNELS  4

#define GPIO_TIM1_CH1IN         /* PE9  JP1-5  T1C1  FMU4  PWM_1 */ GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN         /* PE11 JP1-7  T1C2  FMU3  PWM_2 */ GPIO_TIM1_CH2IN_2
#define GPIO_TIM12_CH1IN        /* PH6 JP1-9   T12C1 BUZ1  PWM_3 */ GPIO_TIM12_CH1IN_2
#define GPIO_TIM1_CH4IN         /* PE14 JP1-11 T1C4  FMU1  PWM_4 */ GPIO_TIM1_CH4IN_2
#define DIRECT_INPUT_TIMER_CHANNELS  4

/* User GPIOs
 *
 * GPIO0-5 are the PWM servo outputs.
 */

#define _MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))

#define GPIO_GPIO0_INPUT        /* PE9  JP1-5  T1C1  FMU4  PWM_1 */ _MK_GPIO_INPUT(GPIO_TIM1_CH1OUT)
#define GPIO_GPIO1_INPUT        /* PE11 JP1-7  T1C2  FMU3  PWM_2 */ _MK_GPIO_INPUT(GPIO_TIM1_CH2OUT)
#define GPIO_GPIO2_INPUT        /* PH6 JP1-9   T12C1 BUZ1  PWM_3 */ _MK_GPIO_INPUT(GPIO_TIM12_CH1OUT)
#define GPIO_GPIO3_INPUT        /* PE14 JP1-11 T1C4  FMU1  PWM_4 */ _MK_GPIO_INPUT(GPIO_TIM1_CH4OUT)

#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))
#define GPIO_GPIO0_OUTPUT       /* PE9  JP1-5  T1C1  FMU4  PWM_1 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH1OUT)
#define GPIO_GPIO1_OUTPUT       /* PE11 JP1-7  T1C2  FMU3  PWM_2 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH2OUT)
#define GPIO_GPIO2_OUTPUT       /* PH6 JP1-9   T12C1 BUZ1  PWM_3 */ _MK_GPIO_OUTPUT(GPIO_TIM12_CH1OUT)
#define GPIO_GPIO3_OUTPUT       /* PE14 JP1-11 T1C4  FMU1  PWM_4 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH4OUT)

#define BOARD_HAS_LED_PWM

#define GPIO_TIM5_CH1OUT        /* PH10 JP2-4  T5C1  MAIN_LED_RED   */ GPIO_TIM5_CH1OUT_2
#define GPIO_TIM5_CH2OUT        /* PH11 JP2-5  T5C2  MAIN_LED_GREEN */ GPIO_TIM5_CH2OUT_2
#define GPIO_TIM5_CH3OUT        /* PH12 JP2-6  T5C3  MAIN_LED_BLUE  */ GPIO_TIM5_CH3OUT_2

#define LED_TIM5_CH1OUT  GPIO_TIM5_CH1OUT
#define LED_TIM5_CH2OUT  GPIO_TIM5_CH2OUT
#define LED_TIM5_CH3OUT  GPIO_TIM5_CH3OUT

#define BOARD_PWM_DRIVE_ACTIVE_LOW 1


/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 J17-1 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER		    8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL   3	/* use capture/compare channel 3 */

/* PWM input driver. Use  pins attached to timer12 channel 1 */
#define PWMIN_TIMER			12
#define PWMIN_TIMER_CHANNEL     /* PH6 JP1-9  */ 1
#define GPIO_PWM_IN             /* PH6 JP1-9  */ GPIO_TIM12_CH1IN_2

#define RC_SERIAL_PORT      "/dev/ttyS2"
#define INVERT_RC_INPUT(_s)     while(0)


#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

#define	BOARD_NAME "BreezePro"

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK_VALID   (1)
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (0)
#define BOARD_ADC_HIPOWER_5V_OC (0)

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,       0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0}, }

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define MS_PWR_BUTTON_DOWN 200
#define KEY_DETECT_GPIO        (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTG|GPIO_PIN2)
#define POWER_CONTROL_ON_GPIO  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)
#define POWER_CONTROL_OFF_GPIO (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_100MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN4)

#define GPIO_PCON_RADIO (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define RF_RADIO_POWER_CONTROL(_on_true)    px4_arch_gpiowrite(GPIO_PCON_RADIO, !(_on_true))

#define GPIO_TEMP_CONT (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN13)
#define TEMP_CONTROL(_on_true)  px4_arch_gpiowrite(GPIO_TEMP_CONT, (_on_true))

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

extern int stm32_spi_bus_initialize(void);

void board_spi_reset(int ms);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);


/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

/************************************************************************************
 * Name: board_pwr_init()
 *
 * Description:
 *   Called to configure power control for the board.
 *
 * Input Parameters:
 *   stage- 0 for boot, 1 for board init
 *
 ************************************************************************************/

void board_pwr_init(int stage);

/****************************************************************************
 * Name: board_pwr_button_down
 *
 * Description:
 *   Called to Read the logical state of the power button
 ****************************************************************************/

bool board_pwr_button_down(void);

/****************************************************************************
 * Name: board_pwr
 *
 * Description:
 *   Called to turn on or off the board
 *
 ****************************************************************************/

void board_pwr(bool on_not_off);

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
