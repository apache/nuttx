/****************************************************************************
 * include/nuttx/input/tsc2007.h
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

/* The TSC2007 is an analog interface circuit for a human interface touch
 * screen device. All peripheral functions are controlled through the command
 * byte and onboard state machines.
 */

#ifndef __INCLUDE_NUTTX_INPUT_TSC2007_H
#define __INCLUDE_NUTTX_INPUT_TSC2007_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_TSC2007)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_TSC2007_NPOLLWAITERS
#  define CONFIG_TSC2007_NPOLLWAITERS 2
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

/* A reference to a structure of this type must be passed to the TSC2007
 * driver.  This structure provides information about the configuration
 * of the TSB2007 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

struct tsc2007_config_s
{
  /* Device characterization */

  uint8_t  address;    /* 7-bit I2C address (only bits 0-6 used) */
  uint16_t rxplate;    /* Calibrated X plate resistance */
  uint32_t frequency;  /* I2C frequency */

  /* If multiple TSC2007 devices are supported, then an IRQ number must
   * be provided for each so that their interrupts can be distinguished.
   */

#ifndef CONFIG_TSC2007_MULTIPLE
  int      irq;        /* IRQ number received by interrupt handler. */
#endif

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the TSC2007 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   * attach  - Attach the TSC2007 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   * pendown - Return the state of the pen down GPIO input
   */

  int  (*attach)(FAR struct tsc2007_config_s *state, xcpt_t isr);
  void (*enable)(FAR struct tsc2007_config_s *state, bool enable);
  void (*clear)(FAR struct tsc2007_config_s *state);
  bool (*pendown)(FAR struct tsc2007_config_s *state);
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
 * Name: tsc2007_register
 *
 * Description:
 *   Configure the TSC2007 to use the provided I2C device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   dev     - An I2C driver instance
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int tsc2007_register(FAR struct i2c_master_s *dev,
                     FAR struct tsc2007_config_s *config, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_TSC2007 */
#endif /* __INCLUDE_NUTTX_INPUT_TSC2007_H */
