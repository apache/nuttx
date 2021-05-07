/****************************************************************************
 * include/nuttx/input/ft5x06.h
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

/* The FT5x06 Series ICs are single-chip capacitive touch panel controller
 * ICs with a built-in 8 bit Micro-controller unit (MCU).  They adopt the
 * mutual capacitance approach, which supports true multi-touch capability.
 * In conjunction with a mutual capacitive touch panel, the FT5x06 have
 * user-friendly input functions, which can be applied on many portable
 * devices, such as cellular phones, MIDs, netbook and notebook personal
 * computers.
 */

#ifndef __INCLUDE_NUTTX_INPUT_FT5X06_H
#define __INCLUDE_NUTTX_INPUT_FT5X06_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_INPUT_FT5X06

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_FT5X06_NPOLLWAITERS
#  define CONFIG_FT5X06_NPOLLWAITERS 2
#endif

/* Check for some required settings.  This can save the user a lot of time
 * in getting the right configuration.
 */

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected."
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the FT5X06
 * driver.  This structure provides information about the configuration
 * of the FT5x06 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

struct ft5x06_config_s
{
  /* Device characterization */

  uint8_t  address;    /* 7-bit I2C address (only bits 0-6 used) */
  uint32_t frequency;  /* Default I2C frequency */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the FT5X06 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach an FT5x06 interrupt handler to a GPIO interrupt
   * enable  - Enable or disable a GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   * wakeup  - Issue WAKE interrupt to FT5x06 to change the FT5x06 from
   *           Hibernate to Active mode.
   * nreset  - Control the chip reset pin (active low)

   */

#ifndef CONFIG_FT5X06_POLLMODE
  int  (*attach)(FAR const struct ft5x06_config_s *config, xcpt_t isr,
                 FAR void *arg);
  void (*enable)(FAR const struct ft5x06_config_s *config, bool enable);
  void (*clear)(FAR const struct ft5x06_config_s *config);
#endif
  void (*wakeup)(FAR const struct ft5x06_config_s *config);
  void (*nreset)(FAR const struct ft5x06_config_s *config,
                 bool state);
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
 * Name: ft5x06_register
 *
 * Description:
 *   Configure the FT5x06 to use the provided I2C device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int ft5x06_register(FAR struct i2c_master_s *i2c,
                    FAR const struct ft5x06_config_s *config, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT_FT5X06 */
#endif /* __INCLUDE_NUTTX_INPUT_FT5X06_H */
