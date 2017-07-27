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

/* This module can be used to configure the Spirit GPIO pins to perform
 * specific functions.  The structure gpioIRQ can be used to specify these
 * features for one of the four Spirit GPIO pin.
 *
 * The following example shows how to configure a pin (GPIO 3) to be used as
 * an IRQ source for a microcontroller using the spirit_gpio_initialize()
 * function.
 *
 * Example:
 *
 * struct spirit_gpio_init_s g_gpio_init =
 * {
 *   SPIRIT_GPIO_3,
 *   SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
 *   SPIRIT_GPIO_DIG_OUT_IRQ
 * };
 *
 * ...
 *
 * spirit_gpio_initialize(&g_gpio_init);
 *
 * NOTE: Please read the functions documentation for the other GPIO
 * features.
 */

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
#define IS_SPIRIT_GPIO_LEVEL(level) \
  (((level) == LOW)            || ((level) == HIGH))
#define IS_SPIRIT_CLOCK_OUTPUT_XO(ratio) \
  (((ratio) == XO_RATIO_1)     || ((ratio) == XO_RATIO_2_3)  || \
   ((ratio) == XO_RATIO_1_2)   || ((ratio) == XO_RATIO_1_3)  || \
   ((ratio) == XO_RATIO_1_4)   || ((ratio) == XO_RATIO_1_6)  || \
   ((ratio) == XO_RATIO_1_8)   || ((ratio) == XO_RATIO_1_12) || \
   ((ratio) == XO_RATIO_1_16)  || ((ratio) == XO_RATIO_1_24) || \
   ((ratio) == XO_RATIO_1_36)  || ((ratio) == XO_RATIO_1_48) || \
   ((ratio) == XO_RATIO_1_64)  || ((ratio) == XO_RATIO_1_96) || \
   ((ratio) == XO_RATIO_1_128) || ((ratio) == XO_RATIO_1_192))
#define IS_SPIRIT_CLOCK_OUTPUT_RCO(ratio) \
  (((ratio) == RCO_RATIO_1)    || ((ratio) == RCO_RATIO_1_128))
#define IS_SPIRIT_CLOCK_OUTPUT_EXTRA_CYCLES(cycles) \
  (((cycles) == EXTRA_CLOCK_CYCLES_0)   || \
   ((cycles) == EXTRA_CLOCK_CYCLES_64)  || \
   ((cycles) == EXTRA_CLOCK_CYCLES_256) || \
   ((cycles) == EXTRA_CLOCK_CYCLES_512))

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

/* SPIRIT OutputLevel enumeration. */

enum spirit_outputlevel_e
{
  LOW  = 0,
  HIGH = 1
};

/* SPIRIT clock output XO prescaler enumeration. */

enum spirit_clockoutput_xoprescaler_e
{
  XO_RATIO_1     = 0x00,  /* XO Clock signal available on the GPIO divided by 1 */
  XO_RATIO_2_3   = 0x02,  /* XO Clock signal available on the GPIO divided by 2/3 */
  XO_RATIO_1_2   = 0x04,  /* XO Clock signal available on the GPIO divided by 1/2 */
  XO_RATIO_1_3   = 0x06,  /* XO Clock signal available on the GPIO divided by 1/3 */
  XO_RATIO_1_4   = 0x08,  /* XO Clock signal available on the GPIO divided by 1/4 */
  XO_RATIO_1_6   = 0x0a,  /* XO Clock signal available on the GPIO divided by 1/6 */
  XO_RATIO_1_8   = 0x0C,  /* XO Clock signal available on the GPIO divided by 1/8 */
  XO_RATIO_1_12  = 0x0e,  /* XO Clock signal available on the GPIO divided by 1/12 */
  XO_RATIO_1_16  = 0x10,  /* XO Clock signal available on the GPIO divided by 1/16 */
  XO_RATIO_1_24  = 0x12,  /* XO Clock signal available on the GPIO divided by 1/24 */
  XO_RATIO_1_36  = 0x14,  /* XO Clock signal available on the GPIO divided by 1/36 */
  XO_RATIO_1_48  = 0x16,  /* XO Clock signal available on the GPIO divided by 1/48 */
  XO_RATIO_1_64  = 0x18,  /* XO Clock signal available on the GPIO divided by 1/64 */
  XO_RATIO_1_96  = 0x1a,  /* XO Clock signal available on the GPIO divided by 1/96 */
  XO_RATIO_1_128 = 0x1c,  /* XO Clock signal available on the GPIO divided by 1/128 */
  XO_RATIO_1_192 = 0x1e   /* XO Clock signal available on the GPIO divided by 1/196 */
};

/* SPIRIT Clock Output RCO prescaler enumeration. */

enum spirit_clockoutput_rcoprescaler_e
{
  RCO_RATIO_1     = 0x00, /* RCO Clock signal available on the GPIO divided by 1 */
  RCO_RATIO_1_128 = 0x01  /* RCO Clock signal available on the GPIO divided by 1/128 */
};

/* SPIRIT Extra Clock Cycles enumeration. */

enum spirit_extra_clockcycles_e
{
  EXTRA_CLOCK_CYCLES_0   = 0x00,  /* 0 extra clock cycles provided to the MCU
                                   * before switching to STANDBY * state */
  EXTRA_CLOCK_CYCLES_64  = 0x20,  /* 64 extra clock cycles provided to the
                                   * MCU before switching to STANDBY state */
  EXTRA_CLOCK_CYCLES_256 = 0x40,  /* 256 extra clock cycles provided to the
                                   * MCU before switching to STANDBY state */
  EXTRA_CLOCK_CYCLES_512 = 0x60   /* 512 extra clock cycles provided to the
                                   * MCU before switching to STANDBY state */
};

/* SPIRIT Clock Output initialization structure definition. */

struct spirit_clockoutput_init_s
{
  uint8_t xoprescaler;   /* Specifies the XO Ratio as clock output. This
                          * parameter can be any value from enum
                          * spirit_clockoutput_xoprescaler_e */

  uint8_t rcoprescaler;  /* Specifies the RCO  Ratio as clock output. This
                          * parameter can be any value from enum
                          * spirit_clockoutput_rcoprescaler_e */

  uint8_t xtracycles;    /* Specifies the Extra Clock Cycles provided before
                          * entering in Standby State.  This parameter can
                          * be any value from enum spirit_extra_clockcycles_e */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_gpio_initialize
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


/******************************************************************************
 * Name: spirit_gpio_enable_tempsensor
 *
 * Description:
 *   Enables or Disables the output of temperature sensor on SPIRIT GPIO_0.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - Bew state for temperature sensor.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_enable_tempsensor(FAR struct spirit_library_s *spirit,
                                  enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_gpio_set_outputlevel
 *
 * Description:
 *   Forces SPIRIT GPIO_x configured as digital output, to VDD or GND.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   gpio   - Specifies the GPIO to be configured.  This parameter can be one
 *            of following values:
 *
 *              SPIRIT_GPIO_0: SPIRIT GPIO_0
 *              SPIRIT_GPIO_1: SPIRIT GPIO_1
 *              SPIRIT_GPIO_2: SPIRIT GPIO_2
 *              SPIRIT_GPIO_3: SPIRIT GPIO_3
 *
 *   level  - Specifies the level.  This parameter can be: HIGH or LOW.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_set_outputlevel(FAR struct spirit_library_s *spirit,
                                enum spirit_gpio_pin_e gpio,
                                enum spirit_outputlevel_e level);

/******************************************************************************
 * Name: spirit_gpio_get_outputlevel
 *
 * Description:
 *   Returns output value (VDD or GND) of SPIRIT GPIO_x, when it is configured
 *   as digital output.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   gpio   - Specifies the GPIO to be read.  This parameter can be one of
 *            following values:
 *
 *              SPIRIT_GPIO_0: SPIRIT GPIO_0
 *              SPIRIT_GPIO_1: SPIRIT GPIO_1
 *              SPIRIT_GPIO_2: SPIRIT GPIO_2
 *              SPIRIT_GPIO_3: SPIRIT GPIO_3
 *
 * Returned Value:
 *   Logical level of selected GPIO configured as digital output. This
 *   parameter can be: HIGH or LOW.
 *
 ******************************************************************************/

enum spirit_outputlevel_e
  spirit_gpio_get_outputlevel(FAR struct spirit_library_s *spirit,
                              enum spirit_gpio_pin_e gpio);

/******************************************************************************
 * Name: spirit_gpio_enable_clockoutput
 *
 * Description:
 *   Enables or Disables the MCU clock output.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for the MCU clock output.  This parameter can be:
 *              S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_enable_clockoutput(FAR struct spirit_library_s *spirit,
                                   enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_gpio_clockoutput_initialize
 *
 * Description:
 *   Initializes the SPIRIT Clock Output according to the specified parameters
 *   in the xClockOutputInitStruct.
 *
 *   NOTE: The function spirit_gpio_enable_clockoutput() must be called in order to
 *   enable or disable the MCU clock dividers.
 *
 * Input Parameters:
 *   spirit      - Reference to a Spirit library state structure instance
 *   clockoutput - Pointer to a struct spirit_clockoutput_init_s structure
 *                 that contains the configuration information for the SPIRIT
 *                 Clock Output.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_clockoutput_initialize(FAR struct spirit_library_s *spirit,
                                       FAR const struct spirit_clockoutput_init_s *clockoutput);

/******************************************************************************
 * Name: spirit_gpio_set_xoprescaler
 *
 * Description:
 *   Sets the XO ratio as clock output.
 *
 * Input Parameters:
 *   spirit      - Reference to a Spirit library state structure instance
 *   xoprescaler - the XO prescaler to be used as clock output.  This
 *                 parameter can be any value from enum
 *                 spirit_clockoutput_xoprescaler_e .
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_set_xoprescaler(FAR struct spirit_library_s *spirit,
                                enum spirit_clockoutput_xoprescaler_e xoprescaler);

/******************************************************************************
 * Name: spirit_gpio_get_xoprescaler
 *
 * Description:
 *   Returns the settled XO prescaler as clock output.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled XO prescaler used for clock output.
 *
 ******************************************************************************/

enum spirit_clockoutput_xoprescaler_e
  spirit_gpio_get_xoprescaler(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_gpio_set_rcoprescaler
 *
 * Description:
 *   Sets the RCO ratio as clock output
 *
 * Input Parameters:
 *   spirit       - Reference to a Spirit library state structure instance
 *   rcoprescaler - The RCO prescaler to be used as clock output. This
 *                  parameter can be any value from enum
 *                  spirit_clockoutput_rcoprescaler_e .
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_set_rcoprescaler(FAR struct spirit_library_s *spirit,
                                 enum spirit_clockoutput_rcoprescaler_e rcoprescaler);

/******************************************************************************
 * Name: spirit_gpio_get_rcoprescaler
 *
 * Description:
 *   Returns the settled RCO prescaler as clock output.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled RCO prescaler used for clock output.
 *
 ******************************************************************************/

enum spirit_clockoutput_rcoprescaler_e
  spirit_gpio_get_rcoprescaler(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_gpio_set_extracycles
 *
 * Description:
 *   Sets the RCO ratio as clock output.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   xtracycles - The number of extra clock cycles provided before switching
 *                to STANDBY state. This parameter can be any value of enum
 *                spirit_extra_clockcycles_e.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_set_extracycles(FAR struct spirit_library_s *spirit,
                                enum spirit_extra_clockcycles_e xtracycles);

/******************************************************************************
 * Name: spirit_gpio_get_extracycles
 *
 * Description:
 *   Returns the settled RCO prescaler as clock output.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled number of extra clock cycles provided before switching to STANDBY
 *   state.
 *
 ******************************************************************************/

enum spirit_extra_clockcycles_e
  spirit_gpio_get_extracycles(FAR struct spirit_library_s *spirit);

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_GPIO_H*/
