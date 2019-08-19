/****************************************************************************
 * boards/arm/imx6/sabre-6quad/src/sabre-6quad.h
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

#ifndef __BOARDS_ARM_IMX6_SABRE_6QUAD_SRC_SABRE_6QUAD_H
#define __BOARDS_ARM_IMX6_SABRE_6QUAD_SRC_SABRE_6QUAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "imx_gpio.h"
#include "imx_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SABRE-6QUAD GPIO Pin Definitions *****************************************/

/* LED
 *
 * A single LED is available driven GPIO1_IO02.
 * On the schematic this is USR_DEF_RED_LED signal to pin T1 (GPIO_2).
 * This signal is shared with KEY_ROW6 (ALT2).
 * A high value illuminates the LED.
 */

#define IOMUX_LED  (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                    IOMUX_DRIVE_40OHM | \
                    IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)
#define GPIO_LED   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                    GPIO_PORT1 | GPIO_PIN2 | \
                    IOMUX_LED)

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
 * Name: imx_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imx_bringup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SABRE_6QUAD_SRC_SABRE_6QUAD_H */
