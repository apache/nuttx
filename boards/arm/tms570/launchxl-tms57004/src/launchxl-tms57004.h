/****************************************************************************
 * boards/arm/tms570/launchxl-tms57004/src/launchxl-tms57004.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_TMS570_LAUNCHXL_TMS57004_SRC_LAUNCHXL_TMS57004_H
#define __BOARDS_ARM_TMS570_LAUNCHXL_TMS57004_SRC_LAUNCHXL_TMS57004_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs
 *
 * The launchpad has several LEDs:
 *
 *   - LEd D1 (white) that connects to the USB +5V supply,
 *   - LED D10 (red) that connects to the TMS570's NERROR pin,
 *   - D5 (blue), D6 (blue), and D8 (blue) connect to the XDS100 FT2322,
 *   - D7 (blue) connects to the XSD100 CPLD, and
 *   - Two white, user LEDs labeled D12 that connects to the NHET08
 *     pin and D11 that connects to GIOA2.
 *
 * NHET08 is one of 32 N2HET pins than can be available to the user if not
 * used by N2HET.  This implementation, however, uses only the single LED
 * driven by GIOA2.  That LED is tied to ground and illuminated with a high
 * level output value.
 */

#define GIO_LED_D11   (GIO_OUTPUT | GIO_CFG_DEFAULT | GIO_OUTPUT_SET | \
                       GIO_PORT_GIOA | GIO_PIN2)

/* Buttons
 *
 * The launchpad has three mechanical buttons. Two of these are reset
 * buttons:  One button is labeled PORRST performs a power-on reset and one
 * labeled RST performs an MCU reset.  Only one button is available for
 * general software usage.  That button is labeled GIOA7 and is, obviously,
 * sensed on GIOA7.
 *
 * GIOA7 is tied to ground, but will be pulled high if the GIOA7 button is
 * depressed.
 */

#define GIO_BUTTON    (GIO_INPUT | GIO_CFG_PULLUP | GIO_INT_BOTHEDGES | \
                       GIO_PORT_GIOA | GIO_PIN7)
#define IRQ_BUTTON     TMS570_IRQ_GIOA7

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_bringup
 *
 * Description:
 *   Bring up simulated board features
 *
 ****************************************************************************/

int tms570_bringup(void);

#endif /* __BOARDS_ARM_TMS570_LAUNCHXL_TMS57004_SRC_LAUNCHXL_TMS57004_H */
