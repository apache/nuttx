/************************************************************************************
 * configs/sam3uek_eval/src/sam3uek_internal.h
 * arch/arm/src/board/sam3uek_internal.n
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_SAM3U_EK_SRC_SAM3UEK_INTERNAL_H
#define __CONFIGS_SAM3U_EK_SRC_SAM3UEK_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* SAM3U-EK GPIOs *******************************************************************/

/* GPIO pin definitions *************************************************************/

/* LEDs */

#define GPIO_LED0    (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_OUTPUT_CLEAR|GPIO_PIN0)
#define GPIO_LED1    (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_OUTPUT_SET|GPIO_PIN1)
#define GPIO_LED2    (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_OUTPUT_SET|GPIO_PIN2)

/* BUTTONS */

#define GPIO_BUTTON1 (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_CFG_DEGLITCH|GPIO_PORT_PIOA|GPIO_PIN18)
#define GPIO_BUTTON2 (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_CFG_DEGLITCH|GPIO_PORT_PIOA|GPIO_PIN19)

#define IRQ_BUTTON1  SAM3U_IRQ_PA18
#define IRQ_BUTTON2  SAM3U_IRQ_PA19

/* SD Card Detect */

#define GPIO_MCI_CD  (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN25)

/* SPI Chip Selects */

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
 * Name: sam3u_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM3U-EK board.
 *
 ************************************************************************************/

extern void weak_function sam3u_spiinitialize(void);

/************************************************************************************
 * Name: sam3u_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the SAM3U-EK board.
 *
 ************************************************************************************/

extern void weak_function sam3u_usbinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAM3U_EK_SRC_SAM3UEK_INTERNAL_H */

