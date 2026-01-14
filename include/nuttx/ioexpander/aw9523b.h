/****************************************************************************
 * include/nuttx/ioexpander/aw9523b.h
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

/* References:
 *   "16-bit I2C-bus and SMBus I/O port with interrupt product datasheet",
 *   Rev. 08 - 22 October 2009, NXP
 */

#ifndef __INCLUDE_NUTTX_IOEXPANDER_AW9523B_H
#define __INCLUDE_NUTTX_IOEXPANDER_AW9523B_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AW9523B_GPIO_NPINS 16   /* All pins can be used as GPIOs */
#define AW9523B_ID         0x23 /* Device ID */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the AW9523B
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the AW9523B and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct aw9523b_config_s
{
  /* Device characterization */

  uint8_t  sub_address; /* configured via the ad1 and ad0 pins. 0-3 */
  uint32_t frequency;   /* I2C or SPI frequency */

#ifdef CONFIG_AW9523B_INT_ENABLE
  uint8_t  irq;         /* IRQ number for the device */

  /* If multiple AW9523B devices are supported, then an IRQ number must
   * be provided for each so that their interrupts can be distinguished.
   */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the AW9523B driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the AW9523B interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   */

  CODE int  (*attach)(FAR struct aw9523b_config_s *state, xcpt_t isr,
                      FAR void *arg);
  CODE void (*enable)(FAR struct aw9523b_config_s *state, bool enable);
#endif
};

/* AW9523B-specific options */

enum aw9523b_option_e
{
  AW9523B_OPTION_SOFTWARE_RESET = 1,      /* Software reset the device */
  AW9523B_OPTION_PORT0_PUSH_PULL,         /* Set port 0 as push-pull */
  AW9523B_OPTION_READ_ID,                 /* Read the ID register */
#ifdef CONFIG_AW9523B_LED_ENABLE
  AW9523B_OPTION_DIMMING,                 /* Set the dimming range for LEDs */
  AW9523B_OPTION_LED_CURRENT,             /* Set the LED current for a pin */
#endif
};
typedef enum aw9523b_option_e aw9523b_option_t;

/* Values for AW9523B_OPTION_* */

#define AW9523B_OPTION_DIMMING_100 0
#define AW9523B_OPTION_DIMMING_75  1
#define AW9523B_OPTION_DIMMING_50  2
#define AW9523B_OPTION_DIMMING_25  3

#define AW9523B_OPTION_P0_PP_OPEN_DRAIN  0
#define AW9523B_OPTION_P0_PP_PUSH_PULL   1

#define AW9523B_OPTION_LED_CURRENT_MAX   255 /* Max. LED current setting */

/* aw9523b-specific option structure
 *
 * Options:
 *   port0_push_pull - Set port 0 as push-pull vs open-drain.
 *                     1 = push-pull, 0 = open-drain.
 *   id              - Returned by AW9523B_OPTION_READ_ID.
 *   dimming         - Dimming range for LEDs. 0-3.
 *                     0 = max. 1 = 0.75x. 2 = 0.5x. 3 = 0.25x.
 *   led_current     - LED current 0-255.
 *                     Total current = 37mA * dimming * led_current/255.
 */

struct aw9523b_nongeneric_option_s
{
  aw9523b_option_t command;     /* Option command */
  union
    {
      uint8_t port0_push_pull;
      uint8_t id;
#ifdef CONFIG_AW9523B_LED_ENABLE
      uint8_t dimming;
      uint8_t led_current;
#endif
    } value;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: aw9523b_initialize
 *
 * Description:
 *   Instantiate and configure the AW9523B device driver to use the provided
 *   I2C device
 *   instance.
 *
 * Input Parameters:
 *   dev     - An I2C driver instance
 *   minor   - The device i2c address
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *aw9523b_initialize(FAR struct i2c_master_s *dev,
                                        FAR struct aw9523b_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_AW9523B_H */
