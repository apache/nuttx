/****************************************************************************
 * include/nuttx/input/max11802.h
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
 *   "Low-Power, Ultra-Small Resistive Touch-Screen Controllers
 *    with I2C/SPI Interface" Maxim IC, Rev 3, 10/2010
 */

#ifndef __INCLUDE_NUTTX_INPUT_MAX11802_H
#define __INCLUDE_NUTTX_INPUT_MAX11802_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdbool.h>
#include <nuttx/irq.h>

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_MAX11802)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI Frequency.  Default:  100KHz */

#ifndef CONFIG_MAX11802_FREQUENCY
#  define CONFIG_MAX11802_FREQUENCY 100000
#endif

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_MAX11802_NPOLLWAITERS
#  define CONFIG_MAX11802_NPOLLWAITERS 2
#endif

#ifndef CONFIG_MAX11802_SPIMODE
#  define CONFIG_MAX11802_SPIMODE SPIDEV_MODE0
#endif

/* Thresholds */

#ifndef CONFIG_MAX11802_THRESHX
#  define CONFIG_MAX11802_THRESHX 12
#endif

#ifndef CONFIG_MAX11802_THRESHY
#  define CONFIG_MAX11802_THRESHY 12
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

/* A reference to a structure of this type must be passed to the MAX11802
 * driver.  This structure provides information about the configuration
 * of the MAX11802 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

struct max11802_config_s
{
  /* Device characterization */

  uint32_t frequency;  /* SPI frequency */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the MAX11802 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   * attach  - Attach the MAX11802 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   * pendown - Return the state of the pen down GPIO input
   */

  int  (*attach)(FAR struct max11802_config_s *state, xcpt_t isr);
  void (*enable)(FAR struct max11802_config_s *state, bool enable);
  void (*clear)(FAR struct max11802_config_s *state);
  bool (*pendown)(FAR struct max11802_config_s *state);
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
 * Name: max11802_register
 *
 * Description:
 *   Configure the MAX11802 to use the provided SPI device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   spi     - An SPI driver instance
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int max11802_register(FAR struct spi_dev_s *spi,
                       FAR struct max11802_config_s *config, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_MAX11802 */
#endif /* __INCLUDE_NUTTX_INPUT_MAX11802_H */
