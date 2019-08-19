/****************************************************************************
 * boards/arm/tiva/launchxl-cc1312r1/src/launchxl-cc1312r1.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __BOARDS_ARM_TIVA_LAUNCHXL_CC1312R1_SRC_LAUNCHXL_CC1312R1_H
#define __BOARDS_ARM__TIVA_LAUNCHXL_CC1312R1_SRC_LAUNCHXL_CC1312R1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Button GPIO IRQ numbers
 *
 *   DIO13_BTN1  SW1  Low input sensed when depressed
 *   DIO14_BTN2  SW2  Low input sensed when depressed
 */

#define CC1312_SW1_IRQ  TIVA_IRQ_DIO_13
#define CC1312_SW2_IRQ  TIVA_IRQ_DIO_14

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct cc13xx_pinconfig_s; /* Forward reference */

/* The LaunchXL-cc1312R1 has two LEDs controlled by software: DIO7_GLED (CR1)
 * and DIO6_RLED (CR2).  A high output value illuminates an LED.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

extern const struct cc13xx_pinconfig_s g_gpio_gled;
extern const struct cc13xx_pinconfig_s g_gpio_rled;

/* The LaunchXL-CC1312R1 has two push-puttons:
 *
 *   DIO13_BTN1  SW1  Low input sensed when depressed
 *   DIO14_BTN2  SW2  Low input sensed when depressed
 */

extern const struct cc13xx_pinconfig_s g_gpio_sw1;
extern const struct cc13xx_pinconfig_s g_gpio_sw2;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cc1312_bringup
 *
 * Description:
 *   Bring up board features.
 *
 *   If CONFIG_BOARD_LATE_INITIALIZE=y, then this function will be called from
 *   board_late_initialize().
 *
 *   If CONFIG_BOARD_LATE_INITIALIZE is not selected,
 *   but CONFIG_LIB_BOARDCTL=y
 *   then this function will *probably* be called from application logic via
 *   boardctl().
 *
 *   Otherwise, this function will not be called (which is usually a bad
 *   thing)
 *
 ****************************************************************************/

int cc1312_bringup(void);

/****************************************************************************
 * Name: cc1312_ssidev_initialize
 *
 * Description:
 *   Called to configure SSI chip select GPIO pins for the LAUNCHXL-CC1312R1
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_SSI
void cc1312_ssidev_initialize(void);
#endif

#endif /* __BOARDS_ARM_TIVA_LAUNCHXL_CC1312R1_SRC_LAUNCHXL_CC1312R1_H */
