/****************************************************************************
 * boards/arm/xmc4/xmc4500-relax/src/xmc4500-relax.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_XMC4_XMC4500_RELAX_SRC_XMC4500_RELAX_H
#define __BOARDS_ARM_XMC4_XMC4500_RELAX_SRC_XMC4500_RELAX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "xmc4_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs
 *
 * The XMC4500 Relax Lite v1 board has two LEDs:
 *
 * LED1 P1.1, Pad type A1+, High output illuminates
 * LED2 P1.0, Pad type A1+ High output illuminates
 */

#define GPIO_LED1    (GPIO_OUTPUT | GPIO_OUTPUT_PUSHPULL | \
                      GPIO_PADA1P_STRONGSOFT | GPIO_PINCTRL_SOFTWARE | \
                      GPIO_OUTPUT_CLEAR | GPIO_PORT1 | GPIO_PIN1)
#define GPIO_LED2    (GPIO_OUTPUT | GPIO_OUTPUT_PUSHPULL | \
                      GPIO_PADA1P_STRONGSOFT | GPIO_PINCTRL_SOFTWARE | \
                      GPIO_OUTPUT_CLEAR | GPIO_PORT1 | GPIO_PIN0)

/* BUTTONS
 *
 * The XMC4500 Relax Lite v1 board has two buttons:
 *
 * BUTTON1 P1.14, Pad type A2, Low input sensed when button pressed
 * BUTTON2 P1.15, Pad type A2, Low input sensed when button pressed
 */

#define GPIO_BUTTON1 (GPIO_INPUT | GPIO_PINCTRL_SOFTWARE | \
                      GPIO_PORT1 | GPIO_PIN14)
#define GPIO_BUTTON2 (GPIO_INPUT | GPIO_PINCTRL_SOFTWARE | \
                      GPIO_PORT1 | GPIO_PIN15)

/* SPIs Chip select */

#define GPIO_CS_MAX6675 (GPIO_OUTPUT | GPIO_OUTPUT_PUSHPULL | \
                         GPIO_PADA1P_STRONGSOFT | GPIO_PINCTRL_SOFTWARE | \
                         GPIO_OUTPUT_SET | GPIO_PORT0 | GPIO_PIN2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int xmc4_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_XMC4_XMC4500_RELAX_SRC_XMC4500_RELAX_H */
