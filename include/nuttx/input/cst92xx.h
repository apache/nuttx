/****************************************************************************
 * include/nuttx/input/cst92xx.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_INPUT_CST92XX_H
#define __INCLUDE_NUTTX_INPUT_CST92XX_H

/* The Hynitron CST92xx family (CST9217, CST9220) are I2C capacitive
 * multi-touch controllers that use 16-bit register addresses.  This header
 * exposes the board-facing registration API.  Once registered, the device
 * appears as /dev/inputN and reports samples through the common touchscreen
 * upper half.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#ifdef CONFIG_INPUT_CST92XX

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default 7-bit I2C address of the CST92xx */

#define CST92XX_I2C_ADDRESS  0x5a

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Board-specific configuration.  Memory for this structure is provided by
 * the caller, is not copied by the driver, and must persist while the
 * driver is active.
 *
 * Coordinate transformation: the raw controller coordinates are first
 * swapped (TOUCH_FLAG_SWAPXY) and then mirrored (TOUCH_FLAG_MIRRORX /
 * TOUCH_FLAG_MIRRORY) against the output resolution.  The transformation is
 * done by this driver; the flags are not passed on to the upper half.
 */

struct cst92xx_config_s
{
  uint8_t  address;   /* 7-bit I2C address, normally CST92XX_I2C_ADDRESS */
  uint32_t frequency; /* I2C frequency in Hz */

  /* Output resolution in pixels, after the optional swap.  If zero, the
   * resolution reported by the controller firmware is used.
   */

  uint16_t xres;
  uint16_t yres;

  /* Coordinate transformation, any of TOUCH_FLAG_SWAPXY,
   * TOUCH_FLAG_MIRRORX and TOUCH_FLAG_MIRRORY from
   * <nuttx/input/touchscreen.h>.
   */

  uint8_t  flags;

  /* IRQ/GPIO access callbacks.  These isolate the driver from the GPIO and
   * interrupt handling of the board and MCU.
   *
   * attach - Attach the driver interrupt handler to the INT GPIO (active
   *          low, falling edge).  The driver disables the interrupt right
   *          after attaching and enables it while the device is open.
   *          Required unless CONFIG_INPUT_CST92XX_POLLMODE is enabled.
   * enable - Enable or disable the INT GPIO interrupt.  Required unless
   *          CONFIG_INPUT_CST92XX_POLLMODE is enabled.
   * reset  - Drive the RST pin: true asserts reset (pin low), false
   *          releases it.  Optional; may be NULL if RST is not wired.
   */

  CODE int  (*attach)(FAR const struct cst92xx_config_s *config,
                      xcpt_t isr, FAR void *arg);
  CODE void (*enable)(FAR const struct cst92xx_config_s *config,
                      bool enable);
  CODE void (*reset)(FAR const struct cst92xx_config_s *config,
                     bool assert);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: cst92xx_register
 *
 * Description:
 *   Reset and probe the CST92xx over I2C, configure the touchscreen lower
 *   half and register it as /dev/inputN where N is the given minor number.
 *
 * Input Parameters:
 *   i2c    - I2C master used to talk to the controller
 *   config - Persistent board configuration
 *   minor  - Device minor number used to form /dev/inputN
 *
 * Returned Value:
 *   Zero (OK) on success.  Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

int cst92xx_register(FAR struct i2c_master_s *i2c,
                     FAR const struct cst92xx_config_s *config, int minor);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT_CST92XX */
#endif /* __INCLUDE_NUTTX_INPUT_CST92XX_H */
