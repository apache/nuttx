/****************************************************************************
 * include/nuttx/input/ads7843e.h
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
 *   "Touch Screen Controller, ADS7843," Burr-Brown Products from Texas
 *    Instruments, SBAS090B, September 2000, Revised May 2002"
 */

#ifndef __INCLUDE_NUTTX_INPUT_ADS7843E_H
#define __INCLUDE_NUTTX_INPUT_ADS7843E_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_ADS7843E)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI Frequency.  Default:  100KHz */

#ifndef CONFIG_ADS7843E_FREQUENCY
#  define CONFIG_ADS7843E_FREQUENCY 100000
#endif

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_ADS7843E_NPOLLWAITERS
#  define CONFIG_ADS7843E_NPOLLWAITERS 2
#endif

#ifndef CONFIG_ADS7843E_SPIMODE
#  define CONFIG_ADS7843E_SPIMODE SPIDEV_MODE0
#endif

/* Thresholds */

#ifndef CONFIG_ADS7843E_THRESHX
#  define CONFIG_ADS7843E_THRESHX 12
#endif

#ifndef CONFIG_ADS7843E_THRESHY
#  define CONFIG_ADS7843E_THRESHY 12
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

/* A reference to a structure of this type must be passed to the ADS7843E
 * driver.  This structure provides information about the configuration
 * of the TSB2007 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

struct ads7843e_config_s
{
  /* Device characterization */

  uint32_t frequency;  /* SPI frequency */

  /* If multiple ADS7843E devices are supported, then an IRQ number must
   * be provided for each so that their interrupts can be distinguished.
   */

#ifndef CONFIG_ADS7843E_MULTIPLE
  int      irq;        /* IRQ number received by interrupt handler. */
#endif

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the ADS7843E driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   *   attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
   *   enable  - Enable or disable the GPIO interrupt
   *   clear   - Acknowledge/clear any pending GPIO interrupt
   *   busy    - Return the state of the BUSY GPIO input (for debug only)
   *   pendown - Return the state of the pen down GPIO input
   *
   * NOTE: The busy() method is not currently used by the driver.  The BUSY
   * signal is only an informative signal that is raised after a command has
   * been send but before the sample data has been obtained.
   */

  int  (*attach)(FAR struct ads7843e_config_s *state, xcpt_t isr);
  void (*enable)(FAR struct ads7843e_config_s *state, bool enable);
  void (*clear)(FAR struct ads7843e_config_s *state);
  bool (*busy)(FAR struct ads7843e_config_s *state);
  bool (*pendown)(FAR struct ads7843e_config_s *state);
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
 * Name: ads7843e_register
 *
 * Description:
 *   Configure the ADS7843E to use the provided SPI device instance.  This
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

int ads7843e_register(FAR struct spi_dev_s *spi,
                      FAR struct ads7843e_config_s *config,
                      int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_ADS7843E */
#endif /* __INCLUDE_NUTTX_INPUT_ADS7843E_H */
