/****************************************************************************
 * drivers/ioexpander/aw9523b.h
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
 *   "16 Multi-function LED Driver and GPIO Controller with I2C Interface",
 *   May 2016, v1.1.1, Shanghai Awinic Technology Co., Ltd.
 * Derived from the PCA9555 driver.
 */

#ifndef __DRIVERS_IOEXPANDER_AW9523B_H
#define __DRIVERS_IOEXPANDER_AW9523B_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/aw9523b.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_AW9523B)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *   CONFIG_I2C
 *     I2C support is required
 *   CONFIG_IOEXPANDER
 *     Enables support for the AW9523B I/O expander
 *
 * CONFIG_IOEXPANDER_AW9523B
 *   Enables support for the AW9523B driver (Needs CONFIG_INPUT)
 * CONFIG_AW9523B_MULTIPLE
 *   Can be defined to support multiple AW9523B devices on board.
 * CONFIG_AW9523B_INT_ENABLE
 *   Enables support for pin on-change interrupts.
 * CONFIG_AW9523B_INT_NCALLBACKS
 *   Maximum number of supported pin interrupt callbacks.
 */

#ifdef CONFIG_AW9523B_INT_ENABLE
#  ifndef CONFIG_AW9523B_INT_NCALLBACKS
#    define CONFIG_AW9523B_INT_NCALLBACKS 4
#  endif
#  ifndef CONFIG_SCHED_WORKQUEUE
#    error Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected.
#  endif
#endif

#undef CONFIG_AW9523B_REFCNT

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.
 *  It defined here
 * so that it will be used consistently in all places.
 */

/* AW9523B Resources ********************************************************/

#ifndef CONFIG_I2C
#error "CONFIG_I2C is required by AW9523B"
#endif

#define AW9523B_MAXDEVS             4

/* AW9523B Registers ********************************************************/

#define AW9523B_I2C_ADDR_BASE  0x58 /* 7-bit base address for AW9523B */

/* AW9523B register addresses */

#define AW9523B_REG_INPUT0     0x00
#define AW9523B_REG_INPUT1     0x01
#define AW9523B_REG_OUTPUT0    0x02
#define AW9523B_REG_OUTPUT1    0x03
#define AW9523B_REG_CONFIG0    0x04
#define AW9523B_REG_CONFIG1    0x05
#define AW9523B_REG_INT0       0x06
#define AW9523B_REG_INT1       0x07
#define AW9523B_REG_ID         0x10
#define AW9523B_REG_GCR        0x11
#define AW9523B_REG_LEDMODE0   0x12
#define AW9523B_REG_LEDMODE1   0x13
#define AW9523B_REG_DIM_P1_0   0x20
#define AW9523B_REG_DIM_P0_0   0x24
#define AW9523B_REG_DIM_P1_4   0x2c
#define AW9523B_REG_RESET      0x7f

#define AW9523B_REG_GCR_DIMMING_MASK 0x03

/* The number of registers to shadow if we're using shadow mode */

#define AW9523B_NUM_SHADOW_REGS 0x14

/* LED default current = 5mA. 5mA / 37mA * 255 = 34 */

#define AW9523B_LED_DEFAULT_DIMMING  34

/* Default output values for each of the subaddresses. */

#define AW9523B_DEFAULT_OUT_0       0x0000
#define AW9523B_DEFAULT_OUT_1       0x0f00
#define AW9523B_DEFAULT_OUT_2       0xf000
#define AW9523B_DEFAULT_OUT_3       0xff00

/* All pins are GPIOs by default, not LEDs. */

#define AW9523B_LEDMODE_ALL_GPIO    0xff

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_AW9523B_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct aw9523b_callback_s
{
  ioe_pinset_t pinset;                 /* Set of pin interrupts that will generate
                                        * the callback. */
  ioe_callback_t cbfunc;               /* The saved callback function pointer */
  FAR void *cbarg;                     /* Callback argument */
};
#endif

/* This structure represents the state of the AW9523B driver */

struct aw9523b_dev_s
{
  struct ioexpander_dev_s      dev;      /* Nested structure to allow casting as public gpio
                                          * expander. */
#ifdef CONFIG_AW9523B_SHADOW_MODE
  uint8_t sreg[AW9523B_NUM_SHADOW_REGS]; /* Shadowed registers of the AW9523B */
#endif
#ifdef CONFIG_AW9523B_MULTIPLE
  FAR struct aw9523b_dev_s    *flink;    /* Supports a singly linked list of drivers */
#endif
  FAR struct aw9523b_config_s *config;   /* Board configuration data */
  FAR struct i2c_master_s     *i2c;      /* Saved I2C driver instance */
  sem_t                        exclsem;  /* Mutual exclusion */

#ifdef CONFIG_AW9523B_INT_ENABLE
  struct work_s work;                    /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct aw9523b_callback_s cb[CONFIG_AW9523B_INT_NCALLBACKS];
#endif
#ifdef CONFIG_AW9523B_LED_ENABLE
  ioe_pinset_t output_is_on_bitset;        /* Indicates if each output is active */
  ioe_pinset_t is_led_bitset;              /* Indicates if each pin is a LED */
  uint8_t led_current[AW9523B_GPIO_NPINS]; /* LED current for each pin */
#endif
  ioe_pinset_t invert_pin;                 /* Pins that are inverted (either input or output) */
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_AW9523B */
#endif /* __DRIVERS_IOEXPANDER_AW9523B_H */
