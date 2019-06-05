/****************************************************************************
 * include/nuttx/input/ft5x06.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "FT5x06", FocalTech Systems Co., Ltd, D-FT5x06-1212-V4.0, Revised
 *   Dec. 18, 2012
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
