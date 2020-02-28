/****************************************************************************
 * include/nuttx/lcd/memlcd.h
 * Common definitions for the Sharp Memory LCD driver
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_MEMLCD_H
#define __INCLUDE_NUTTX_MEMLCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the initialization
 * method. It provides some board-specific hooks used by driver to manage the
 * timer and gpio (IRQ and DISP).
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct memlcd_priv_s
{
  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the Memory LCD driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * irqattach   - Attach the driver interrupt handler to the GPIO interrupt
   *               If isr is NULL, detach and disable it.
   * dispcontrol - Enable or disable the DISP pin and EXTCOMIN interrupt.
   * setpolarity - Board specified method to set EXTCOMIN.
   *               Needed only when CONFIG_MEMLCD_EXTCOMIN_MODE_HW is not set.
   * setvcomfreq - Set timer frequency for EXTCOMIN.
   */

  int (*attachirq) (xcpt_t isr, void *arg);
  void (*dispcontrol) (bool on);
#ifndef CONFIG_MEMLCD_EXTCOMIN_MODE_HW
  void (*setpolarity) (bool pol);
#endif
  void (*setvcomfreq) (unsigned int freq);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  memlcd_initialize
 *
 * Description:
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_MEMLCD_NINTERFACES-1.
 *   This allows support for multiple devices.
 *   memlcd_priv_s - Board specific structure
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for the
 *   specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct lcd_dev_s;               /* see nuttx/lcd.h */
struct spi_dev_s;               /* see nuttx/spi/spi.h */
FAR struct lcd_dev_s *memlcd_initialize(FAR struct spi_dev_s *spi,
                                        FAR struct memlcd_priv_s *priv,
                                        unsigned int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_MEMLCD_H */
