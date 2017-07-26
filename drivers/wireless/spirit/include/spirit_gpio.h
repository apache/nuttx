/******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_gpio.h
 * This file provides all the low level API to manage SPIRIT GPIO.
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 *   Adapted for NuttX by:
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_GPIO_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_GPIO_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_regs.h"
#include "spirit_types.h"


/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macros used in debug assertions */

#define IS_SPIRIT_GPIO(pin)  \
  ((pin == SPIRIT_GPIO_0) || (pin == SPIRIT_GPIO_1) || \
   (pin == SPIRIT_GPIO_2) || (pin == SPIRIT_GPIO_3))
#define IS_SPIRIT_GPIO_MODE(mode)  \
  ((mode == SPIRIT_GPIO_MODE_DIGITAL_INPUT) || \
   (mode == SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP) || \
   (mode == SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP))
#define IS_SPIRIT_GPIO_IO(iosel)  \
  ((iosel == SPIRIT_GPIO_DIG_OUT_IRQ) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_POR_INV) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_WUT_EXP) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_LBD) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_TX_DATA) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_TX_STATE) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_TX_FIFO_ALMOST_EMPTY) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_TX_FIFO_ALMOST_FULL) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_RX_DATA) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_RX_CLOCK) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_RX_STATE) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_RX_FIFO_ALMOST_FULL) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_RX_FIFO_ALMOST_EMPTY) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_ANTENNA_SWITCH) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_VALID_PREAMBLE) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_SYNC_DETECTED) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_RSSI_THRESHOLD) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_MCU_CLOCK) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_TX_RX_MODE) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_VDD) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_GND) || \
   (iosel == SPIRIT_GPIO_DIG_OUT_SMPS_EXT) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_SLEEP_OR_STANDBY) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_READY) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_LOCK) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_WAIT_FOR_LOCK_SIG) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_WAIT_FOR_TIMER_FOR_LOCK) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_WAIT_FOR_READY2_SIG) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_WAIT_FOR_TIMER_FOR_PM_SET) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_WAIT_VCO_CALIBRATION) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_ENABLE_SYNTH_FULL_CIRCUIT) ||\
   (iosel == SPIRIT_GPIO_DIG_OUT_WAIT_FOR_RCCAL_OK_SIG) ||\
   (iosel == SPIRIT_GPIO_DIG_IN_TX_COMMAND) ||\
   (iosel == SPIRIT_GPIO_DIG_IN_RX_COMMAND) ||\
   (iosel == SPIRIT_GPIO_DIG_IN_TX_DATA_INPUT_FOR_DIRECTRF) ||\
   (iosel == SPIRIT_GPIO_DIG_IN_DATA_WAKEUP) ||\
   (iosel == SPIRIT_GPIO_DIG_IN_EXT_CLOCK_AT_34_7KHZ))

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* SPIRIT GPIO pin enumeration. */

enum spirit_gpio_pin_e
{
  SPIRIT_GPIO_0 = GPIO0_CONF_BASE,  /* GPIO_0 selected */
  SPIRIT_GPIO_1 = GPIO1_CONF_BASE,  /* GPIO_1 selected */
  SPIRIT_GPIO_2 = GPIO2_CONF_BASE,  /* GPIO_2 selected */
  SPIRIT_GPIO_3 = GPIO3_CONF_BASE   /* GPIO_3 selected */
};

/* SPIRIT GPIO mode enumeration. */

enum spirit_gpio_mode_e
{
  SPIRIT_GPIO_MODE_DIGITAL_INPUT      = 0x01,  /* Digital Input on GPIO */
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP  = 0x02,  /* Digital Output on GPIO
                                                * (low current) */
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP  = 0x03   /* Digital Output on GPIO
                                                * (high current) */
};

/* SPIRIT I/O selection enumeration. */

enum spirit_gpio_io_e
{
  SPIRIT_GPIO_DIG_OUT_IRQ            = 0x00,  /* nIRQ (Interrupt Request,
                                               * active low) , default
                                               * configuration after POR */
  SPIRIT_GPIO_DIG_OUT_POR_INV        = 0x08,  /* POR inverted (active low) */
  SPIRIT_GPIO_DIG_OUT_WUT_EXP        = 0x10,  /* Wake-Up Timer expiration:
                                               * "1" when WUT has expired */
  SPIRIT_GPIO_DIG_OUT_LBD            = 0x18,  /* Low battery detection: "1"
                                               * when battery is below
                                               * threshold setting */
  SPIRIT_GPIO_DIG_OUT_TX_DATA        = 0x20,  /* TX data internal clock
                                               * output (TX data are sampled
                                               * on the rising edge of it) */
  SPIRIT_GPIO_DIG_OUT_TX_STATE       = 0x28,  /* TX state indication: "1"
                                               * when Spirit1 is passing in
                                               * the TX state */
  SPIRIT_GPIO_DIG_OUT_TX_FIFO_ALMOST_EMPTY = 0x30, /* TX FIFO Almost Empty Flag */
  SPIRIT_GPIO_DIG_OUT_TX_FIFO_ALMOST_FULL  = 0x38, /* TX FIFO Almost Full Flag */
  SPIRIT_GPIO_DIG_OUT_RX_DATA        = 0x40,  /* RX data output */
  SPIRIT_GPIO_DIG_OUT_RX_CLOCK       = 0x48,  /* RX clock output
                                               * (recovered from received
                                               * data) */
  SPIRIT_GPIO_DIG_OUT_RX_STATE       = 0x50,  /* RX state indication: "1"
                                               * when Spirit1 is passing in
                                               * the RX state */
  SPIRIT_GPIO_DIG_OUT_RX_FIFO_ALMOST_FULL  = 0x58, /* RX FIFO Almost Full Flag */
  SPIRIT_GPIO_DIG_OUT_RX_FIFO_ALMOST_EMPTY = 0x60, /* RX FIFO Almost Empty Flag */
  SPIRIT_GPIO_DIG_OUT_ANTENNA_SWITCH = 0x68,  /* Antenna switch used for
                                               * antenna diversity */
  SPIRIT_GPIO_DIG_OUT_VALID_PREAMBLE = 0x70,  /* Valid Preamble Detected Flag */
  SPIRIT_GPIO_DIG_OUT_SYNC_DETECTED  = 0x78,  /* Sync WordSync Word Detected
                                               * Flag */
  SPIRIT_GPIO_DIG_OUT_RSSI_THRESHOLD = 0x80,  /* RSSI above threshold */
  SPIRIT_GPIO_DIG_OUT_MCU_CLOCK      = 0x88,  /* MCU Clock */
  SPIRIT_GPIO_DIG_OUT_TX_RX_MODE     = 0x90,  /* TX or RX mode indicator
                                               * (to enable an external range
                                               * extender) */
  SPIRIT_GPIO_DIG_OUT_VDD            = 0x98,  /* VDD (to emulate an additional
                                               * GPIO of the MCU, programmable
                                               * by SPI) */
  SPIRIT_GPIO_DIG_OUT_GND            = 0xa0,  /* GND (to emulate an additional
                                               * GPIO of the MCU, programmable
                                               * by SPI) */
  SPIRIT_GPIO_DIG_OUT_SMPS_EXT       = 0xa8,  /* External SMPS enable
                                               * signal (active high) */

  SPIRIT_GPIO_DIG_OUT_SLEEP_OR_STANDBY          = 0xb0,
  SPIRIT_GPIO_DIG_OUT_READY                     = 0xb8,
  SPIRIT_GPIO_DIG_OUT_LOCK                      = 0xc0,
  SPIRIT_GPIO_DIG_OUT_WAIT_FOR_LOCK_SIG         = 0xc8,
  SPIRIT_GPIO_DIG_OUT_WAIT_FOR_TIMER_FOR_LOCK   = 0xd0,
  SPIRIT_GPIO_DIG_OUT_WAIT_FOR_READY2_SIG       = 0xd8,
  SPIRIT_GPIO_DIG_OUT_WAIT_FOR_TIMER_FOR_PM_SET = 0xe0,
  SPIRIT_GPIO_DIG_OUT_WAIT_VCO_CALIBRATION      = 0xe8,
  SPIRIT_GPIO_DIG_OUT_ENABLE_SYNTH_FULL_CIRCUIT = 0xf0,
  SPIRIT_GPIO_DIG_OUT_WAIT_FOR_RCCAL_OK_SIG     = 0xff,

  SPIRIT_GPIO_DIG_IN_TX_COMMAND                 = 0x00,
  SPIRIT_GPIO_DIG_IN_RX_COMMAND                 = 0x08,
  SPIRIT_GPIO_DIG_IN_TX_DATA_INPUT_FOR_DIRECTRF = 0x10,
  SPIRIT_GPIO_DIG_IN_DATA_WAKEUP                = 0x18,
  SPIRIT_GPIO_DIG_IN_EXT_CLOCK_AT_34_7KHZ       = 0x20
};

/* SPIRIT GPIO Init structure definition. */

struct spirit_gpio_init_s
{
  enum spirit_gpio_pin_e gpiopin;    /* Specifies the GPIO pins to be
                                      * configured. This parameter can be
                                      * any value of enum spirit_gpio_pin_e */
  enum spirit_gpio_mode_e gpiomode;  /* Specifies the operating mode for
                                      * the selected pins. This parameter
                                      * can be a value of enum
                                      * spirit_gpio_mode_e */
  enum spirit_gpio_io_e gpioio;      /* Specifies the I/O selection for
                                      * the selected pins. This parameter
                                      * can be a value of enum spirit_gpio_io_e */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name:
 *
 * Description:
 *   Initializes the Spirit GPIOx according to the specified parameters in
 *   the gpioinit parameter.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   gpioinit - A pointer to a struct spirit_gpio_init_s structure that
 *              contains the configuration information for the specified
 *              SPIRIT GPIO.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_initialize(FAR struct spirit_library_s *spirit,
                           FAR const struct spirit_gpio_init_s *gpioinit);

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_GPIO_H*/
