/****************************************************************************
 * include/nuttx/leds/ncp5623c.h
 * based on include/nuttx/leds/pca9635pw.c
 *
 *   Author: Konstantin Berzenko <kpberezenko@gmail.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#ifndef __INCLUDE_NUTTX_LEDS_NCP5623C_H
#define __INCLUDE_NUTTX_LEDS_NCP5623C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Configuration
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_NCP5623C - Enables support for the NCP5623C driver
 */

#if defined(CONFIG_I2C) && defined(CONFIG_NCP5623C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C definitions */

#define I2C_BUS_FREQ_HZ        (400000)

/* NCP5623C register addresses */

#define NCP5623C_SHUTDOWN      (0x00)     /* System Shut Down */
#define NCP5623C_ILED          (0x1)      /* ILED current */
#define NCP5623C_PWM1          (0x2)      /* LED 1 brightness control */
#define NCP5623C_PWM2          (0x3)      /* LED 2 brightness control */
#define NCP5623C_PWM3          (0x4)      /* LED 3 brightness control */
#define NCP5623C_UPWARD        (0x5)      /* Set Up the IEND Upward */
#define NCP5623C_DWNWRD        (0x6)      /* Set Up the IEND Downward */
#define NCP5623C_GRAD          (0x7)      /* Set Up the Gradual Dimming */
#define NCP5623C_MAX_REG       (0x7)      /* Highest register */

#define NCP5623C_ADDRESS_MASK  (0xe0)     /* Address part of reg */
#define NCP5623C_VALUE_MASK    (0x1f)     /* Value part of reg */
#define NCP5623C_SHIFT_ADDRESS (5)
#define NCP5623C_MAX_VALUE     (0x1f)     /* Max value of all registers */

#define NCP5623C_SET_REG(addr, val) \
  (((addr << NCP5623C_SHIFT_ADDRESS) & NCP5623C_ADDRESS_MASK) | \
   (val & NCP5623C_VALUE_MASK))           /* combine addr and val */

/* IOCTL commands */

#define LEDIOC_SET_REG _ULEDIOC(1)        /* Arg: ncp5623c_set_reg_s * pointer */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum led_select_e
{
  LED_1 = NCP5623C_PWM1,
  LED_2 = NCP5623C_PWM2,
  LED_3 = NCP5623C_PWM3
};

/* This structure is used in an IOCTL command for setting the PWM of an individual
 * LED. The desired LED is selected by setting the 'led' parameter accordingly
 * whereas the 'led_pwm' field governs the brightness of the selected LED. A value
 * of 0 (0x00) leads to a duty cycle of 0 % = LED off while a value of 255 (0xFF)
 * leads to a duty cycle of 99.6 % = Maximum brightness.
 */

struct ncp5623c_set_reg_s
{
  uint8_t reg;
  uint8_t val;
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
 * Name: ncp5623c_register
 *
 * Description:
 *   Register the NCP5623C device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leddrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   ncp5623c_i2c_addr
 *           - The I2C address of the NCP5623C.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ncp5623c_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t const ncp5623c_i2c_addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_I2C_NCP5623C */
#endif /* __INCLUDE_NUTTX_LEDS_NCP5623C_H */
