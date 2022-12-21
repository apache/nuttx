/****************************************************************************
 * include/nuttx/leds/lp503x.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LEDS_LP503X_H
#define __INCLUDE_NUTTX_LEDS_LP503X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Configuration
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_LP503X - Enables support for the LP503X driver
 */

#if defined(CONFIG_I2C) && defined(CONFIG_LP503X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C definitions */

#define MAX_LEDS                      35
#define MAX_RGB_LEDS                  11
#define MAX_BRIGHTNESS                255
#define MAX_RGB_COLOUR                0xffffff

#define LP503X_I2C_BUS_FREQ_HZ       (380000)

/* LP503X Register Map */

#define LP503X_DEVICE_CONFIG0         0x00
#define LP503X_DEVICE_CONFIG1         0x01

#define LP503X_LED_CONFIG0            0x02
#define LP503X_LED_CONFIG1            0x03

#define LP503X_BANK_BRIGHTNESS        0x04

#define LP503X_BANK_A_COLOUR          0x05 /* All red   LEDs */
#define LP503X_BANK_B_COLOUR          0x06 /* All green LEDs */
#define LP503X_BANK_C_COLOUR          0x07 /* All blue  LEDs */

#define LP503X_LED0_BRIGHTNESS        0x08
#define LP503X_LED1_BRIGHTNESS        0x09
#define LP503X_LED2_BRIGHTNESS        0x0a
#define LP503X_LED3_BRIGHTNESS        0x0b
#define LP503X_LED4_BRIGHTNESS        0x0c
#define LP503X_LED5_BRIGHTNESS        0x0d
#define LP503X_LED6_BRIGHTNESS        0x0e
#define LP503X_LED7_BRIGHTNESS        0x0f
#define LP503X_LED8_BRIGHTNESS        0x10
#define LP503X_LED9_BRIGHTNESS        0x11
#define LP503X_LED10_BRIGHTNESS       0x12
#define LP503X_LED11_BRIGHTNESS       0x13

#define LP503X_OUT0_COLOUR            0x14
#define LP503X_OUT1_COLOUR            0x15
#define LP503X_OUT2_COLOUR            0x16
#define LP503X_OUT3_COLOUR            0x17
#define LP503X_OUT4_COLOUR            0x18
#define LP503X_OUT5_COLOUR            0x19
#define LP503X_OUT6_COLOUR            0x1a
#define LP503X_OUT7_COLOUR            0x1b
#define LP503X_OUT8_COLOUR            0x1c
#define LP503X_OUT9_COLOUR            0x1d
#define LP503X_OUT10_COLOUR           0x1e
#define LP503X_OUT11_COLOUR           0x1f
#define LP503X_OUT12_COLOUR           0x20
#define LP503X_OUT13_COLOUR           0x21
#define LP503X_OUT14_COLOUR           0x22
#define LP503X_OUT15_COLOUR           0x23
#define LP503X_OUT16_COLOUR           0x24
#define LP503X_OUT17_COLOUR           0x25
#define LP503X_OUT18_COLOUR           0x26
#define LP503X_OUT19_COLOUR           0x27
#define LP503X_OUT20_COLOUR           0x28
#define LP503X_OUT21_COLOUR           0x29
#define LP503X_OUT22_COLOUR           0x2a
#define LP503X_OUT23_COLOUR           0x2b
#define LP503X_OUT24_COLOUR           0x2c
#define LP503X_OUT25_COLOUR           0x2d
#define LP503X_OUT26_COLOUR           0x2e
#define LP503X_OUT27_COLOUR           0x2f
#define LP503X_OUT28_COLOUR           0x30
#define LP503X_OUT29_COLOUR           0x31
#define LP503X_OUT30_COLOUR           0x32
#define LP503X_OUT31_COLOUR           0x33
#define LP503X_OUT32_COLOUR           0x34
#define LP503X_OUT33_COLOUR           0x35
#define LP503X_OUT34_COLOUR           0x36
#define LP503X_OUT35_COLOUR           0x37

#define LP503X_RESET                  0x38

/* LP503X Commands */

/* DEVICE_CONFIG0 */

#define LP503X_CHIP_ENABLE           (0x01 << 6)
#define LP503X_CHIP_DISABLE           0

/* DEVICE_CONFIG1 */

#define LP503X_GLOBAL_OFF            (0x01 << 0)
#define LP503X_MAX_CURRENT_OPTION    (0x01 << 1)
#define LP503X_PWM_DITHERING_ENABLE  (0x01 << 2)
#define LP503X_AUTO_INCR_ENABLE      (0x01 << 3)
#define LP503X_POWER_SAVE_ENABLE     (0x01 << 4)
#define LP503X_LOG_SCALE_ENABLE      (0x01 << 5)

/* LED_CONFIG0 */

#define LP503X_LED_BANK_MODE_ENABLED  1
#define LP503X_LED_BANK_MODE_DISABLED 0
#define LP503X_LED0_BANK_ENABLE      (0x01 << 0)
#define LP503X_LED1_BANK_ENABLE      (0x01 << 1)
#define LP503X_LED2_BANK_ENABLE      (0x01 << 2)
#define LP503X_LED3_BANK_ENABLE      (0x01 << 3)
#define LP503X_LED4_BANK_ENABLE      (0x01 << 4)
#define LP503X_LED5_BANK_ENABLE      (0x01 << 5)
#define LP503X_LED6_BANK_ENABLE      (0x01 << 6)
#define LP503X_LED7_BANK_ENABLE      (0x01 << 7)
#define LP503X_ALL_LEDS_BANK_MODE0    0xff

/* LED_CONFIG1 */

#define LP503X_LED8_BANK_ENABLE      (0x01 << 0)
#define LP503X_LED9_BANK_ENABLE      (0x01 << 1)
#define LP503X_LED10_BANK_ENABLE     (0x01 << 2)
#define LP503X_LED11_BANK_ENABLE     (0x01 << 3)
#define LP503X_ALL_LEDS_BANK_MODE1    0x0f

/* RESET ALL REGISTERS */

#define LP503X_RESET_ALL_REGISTERS    0xff

#define  LED_RED                      0xff0000
#define  LED_GREEN                    0x00ff00
#define  LED_BLUE                     0x0000ff
#define  LED_MAGENTA                  0xff00ff
#define  LED_YELLOW                   0xffff33
#define  LED_CYAN                     0x00ffff
#define  LED_WHITE                    0xffffff
#define  LED_ORANGE                   0xffa500

/* LP503X_CONFIG1 bit definitions, disable=0, enable=1 */

#define LP503X_CONFIG1_LOG_SCALE   (1 << 5) /* enable log scale not linear  */
#define LP503X_CONFIG1_PWRSAVE     (1 << 4) /* power save mode              */
#define LP503X_CONFIG1_AUTOINC     (1 << 3) /* register auto increment mode */
#define LP503X_CONFIG1_DITHERING   (1 << 2) /* power saving mode            */
#define LP503X_CONFIG1_MAX_CURRENT (1 << 1) /* responds to I2C subaddr1     */
#define LP503X_CONFIG1_GLOBAL_OFF  (1 << 0) /* enable GLOGAL LEDs off       */

/* IOCTL commands */

#define PWMIOC_ENABLE               _PWMIOC(0) /* true/false                */
#define PWMIOC_RESET                _PWMIOC(1) /* no args                   */
#define PWMIOC_CONFIG               _PWMIOC(2) /* lin/log
                                                * pwm dither
                                                * Imax
                                                * Power Save                */
#define PWMIOC_ENABLE_LED_BANK_MODE _PWMIOC(3) /* led(0..11)                */
#define PWMIOC_SET_LED_COLOUR       _PWMIOC(4) /* led(0..35), Colour(0..255)*/
#define PWMIOC_SET_RGB_COLOUR       _PWMIOC(5) /* led(0..11), Colour HTML RGB*/
#define PWMIOC_SET_RGB_BRIGHTNESS   _PWMIOC(6) /* led(0..11), level(0..255) */
#define PWMIOC_SET_BANK_MIX_COLOUR  _PWMIOC(7) /* bank(A/B/C),level(0-255)  */
#define PWMIOC_SET_BANK_BRIGHTNESS  _PWMIOC(8) /* level(0-255)              */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* struct use to set device operating modes
 * - log/lin scale
 * - power save enable/disable
 * - reg auto inc mode (set to FALSE (disable) for this driver
 * - pwm dithering enable/disable
 * - max current option
 * - global LED shutdown control

 */

struct lp503x_config_s
{
  bool enable_log_mode;
  bool enable_power_save;
  bool enable_auto_increment;
  bool enable_pwm_dithering;
  bool set_max_current_35ma;
  bool enable_all_led_shutdown;
  bool led_mode[12];
};

/* struct use for most/all ioctl calls */

struct ioctl_arg_s
{
  uint8_t lednum;
  uint32_t param;
};

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: lp503x_register
 *
 * Description:
 *   Register the LP503X device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leddrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   lp503x_i2c_addr
 *           - The I2C address of the LP503X.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lp503x_register(const char *devpath, struct i2c_master_s *i2c,
                    uint8_t const lp503x_i2c_addr, int const i2c_frequency);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_I2C_LP503X */
#endif /* __INCLUDE_NUTTX_LEDS_LP503X_H */
