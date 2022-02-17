/****************************************************************************
 * include/nuttx/lcd/memlcd.h
 * Common definitions for the Sharp Memory LCD driver
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

#ifndef __INCLUDE_NUTTX_LCD_MEMLCD_H
#define __INCLUDE_NUTTX_LCD_MEMLCD_H

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
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the
 * initialization method. It provides some board-specific hooks used by
 * driver to manage the timer and gpio (IRQ and DISP).
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
   * attachirq   - Attach the driver interrupt handler to the GPIO interrupt
   *               If isr is NULL, detach and disable it.
   * dispcontrol - Enable or disable the DISP pin and EXTCOMIN interrupt.
   * setpolarity - Board specified method to set EXTCOMIN.  Needed only when
   *               CONFIG_MEMLCD_EXTCOMIN_MODE_HW is not set.
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
 *   priv - Board specific structure
 *   devno - A value in the range of 0 through CONFIG_MEMLCD_NINTERFACES-1.
 *   This allows support for multiple devices.
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

#endif /* __INCLUDE_NUTTX_LCD_MEMLCD_H */
