/************************************************************************************
 * configs/detron/src/detron_internal.h
 * arch/arm/src/board/detron_internal.n
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef _CONFIGS_DETRON_SRC_DETRON_INTERNAL_H
#define _CONFIGS_DETRON_SRC_DETRON_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Detron GPIO Pin Definitions ******************************************************/
/* Board GPIO Usage: To be provided */

#define DETRON_LED1_A             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN0)
#define DETRON_LED1_B             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN1)
#define DETRON_LED2_A             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN2)
#define DETRON_LED2_B             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN3)
#define DETRON_232_ENABLE         (GPIO_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORT2 | GPIO_PIN5)
#define DETRON_232_POWERSAVE      (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN5)
#define DETRON_232_VALID          (GPIO_INPUT  | GPIO_PULLUP     | GPIO_PORT2 | GPIO_PIN5)
#define DETRON_HEARTBEAT          (GPIO_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORT2 | GPIO_PIN11)
#define DETRON_EXTRA_LED          (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN12)
#define DETRON_5V_ENABLE          (GPIO_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORT2 | GPIO_PIN13)
#define DETRON_5V_DISABLE         (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN13)

#define DETRON_MMCSD_CS           (GPIO_OUTPUT | GPIO_VALUE_ONE  |  GPIO_PORT0 | GPIO_PIN16)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_sspinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Detron board.
 *
 ************************************************************************************/

extern void weak_function lpc17_sspinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_DETRON_SRC_DETRON_INTERNAL_H */

