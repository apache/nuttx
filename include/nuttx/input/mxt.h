/****************************************************************************
 * include/nuttx/input/mxt.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_INPUT_MXT_H
#define __INCLUDE_NUTTX_INPUT_MXT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_MXT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_MXT_NPOLLWAITERS
#  define CONFIG_MXT_NPOLLWAITERS 2
#endif

/* Thresholding
 *
 * New touch positions will only be reported when the X or Y data
 * changes by these thresholds. This trades reduced data rates for some
 * loss in dragging accuracy.  For 12-bit values the raw ranges are
 * 0-4095. So for example, if your display is 800x480, then THRESHX=5
 * and THRESHY=8 would correspond to a one pixel change.
 *
 * NOTE: This does nothing to reduce the interrupt rate.  It only
 * reduces the rate at which touch events are reports.
 */

#ifndef CONFIG_MXT_MXT_THRESHX
#  define CONFIG_MXT_MXT_THRESHX 5
#endif

#ifndef CONFIG_MXT_MXT_THRESHY
#  define CONFIG_MXT_MXT_THRESHY 8
#endif

/* Buttons are not supported */

#undef CONFIG_MXT_BUTTONS

/* Check for some required settings.  This can save the user a lot of time
 * in getting the right configuration.
 */

#ifdef CONFIG_DISABLE_SIGNALS
#  error "Signals are required.  CONFIG_DISABLE_SIGNALS must not be selected."
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected."
#endif

/* Helper macros ************************************************************/

#define MXT_ATTACH(s,isr,arg) ((s)->attach(s,isr,arg))
#define MXT_DETACH(s)         ((s)->attach(s,NULL,NULL))
#define MXT_ENABLE(s)         ((s)->enable(s,true))
#define MXT_DISABLE(s)        ((s)->enable(s,false))
#define MXT_CLEAR(s)          ((s)->clear(s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the MXT interrupt handler.  The lower level code will
 * intercept the interrupt and provide the upper level with the private data
 * that was provided when the interrupt was attached.
 */

struct mxt_lower_s;
typedef CODE int (*mxt_handler_t)(FAR const struct mxt_lower_s *lower,
                                  FAR void *arg);

/* A reference to a structure of this type must be passed to the MXT
 * driver.  This structure provides information about the configuration
 * of the MXT and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct mxt_lower_s
{
  /* Device characterization */

  uint32_t frequency;  /* Initial I2C frequency */
  uint8_t address;     /* 7-bit I2C address (only bits 0-6 used) */

  /* True: Swap X and Y values */

  bool swapxy;

#ifdef CONFIG_MXT_BUTTONS
  /* Buttons are not currently supported.  But if they were we
   * would need to know which which.
   */

  uint8_t bmask;       /* Bit encoded, see MXT_GPIOn_MASK */
#endif

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the MXT driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   * attach  - Attach the MXT interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   */

  int  (*attach)(FAR const struct mxt_lower_s *lower, mxt_handler_t isr,
                 FAR void *arg);
  void (*enable)(FAR const struct mxt_lower_s *lower, bool enable);
  void (*clear)(FAR const struct mxt_lower_s *lower);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mxt_register
 *
 * Description:
 *   Configure the maXTouch to use the provided I2C device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   lower   - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mxt_register(FAR struct i2c_master_s *i2c,
                 FAR const struct mxt_lower_s *lower, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_MXT */
#endif /* __INCLUDE_NUTTX_INPUT_MXT_H */
