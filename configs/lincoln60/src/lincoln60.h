/****************************************************************************
 * configs/lincoln60/src/lincoln60.h
 *
 *   Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
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

#ifndef _CONFIGS_LINCOLN60_SRC_LINCOLN60_H
#define _CONFIGS_LINCOLN60_SRC_LINCOLN60_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 *  LEDs GPIO                        PIN  SIGNAL NAME
 *  -------------------------------- ---- --------------
 *  P1[18]                            32  LED1
 *  P3[26]                            26  LED2
 ****************************************************************************/

#define LINCOLN60_LED1              (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN18)
#define LINCOLN60_LED1_OFF          LINCOLN60_LED1
#define LINCOLN60_LED1_ON           (LINCOLN60_LED1 | GPIO_VALUE_ONE)
#define LINCOLN60_LED2              (GPIO_OUTPUT | GPIO_PORT3 | GPIO_PIN26)
#define LINCOLN60_LED2_OFF          LINCOLN60_LED2
#define LINCOLN60_LED2_ON           (LINCOLN60_LED2 | GPIO_VALUE_ONE)

#define LINCOLN60_HEARTBEAT         LINCOLN60_LED2

/****************************************************************************
 *  Buttons GPIO                     PIN  SIGNAL NAME
 *  -------------------------------- ---- --------------
 *  P2[10]                            53  BTN1
 ****************************************************************************/

#define LINCOLN60_BUT1              (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | \
                                     GPIO_PIN10)

/* Button IRQ numbers */

#define LINCOLN60_BUT1_IRQ          LPC17_IRQ_P0p23

/****************************************************************************
 *  microSD                          PIN   SIGNAL NAME
 *  -------------------------------- ----- --------------
 *  P0[15]                           J12 3  SPI SCK
 *  P0[17]                           J12 4  SPI MISO
 *  P0[18]                           J12 5  SPI MOSI
 *  P0[16]                           J18 5  SPI slave select
 ****************************************************************************/

#define LINCOLN60_CS               (GPIO_OUTPUT | GPIO_VALUE_ONE | \
                                    GPIO_PORT0 | GPIO_PIN16)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lincoln60_sspinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Lincoln 60 board.
 *
 ****************************************************************************/

void weak_function lincoln60_sspinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_LINCOLN60_SRC_LINCOLN60_H */
