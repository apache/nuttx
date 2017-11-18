/************************************************************************************
 * configs/mcb1700/src/mcb1700.h
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
 ************************************************************************************/

#ifndef _CONFIGS_MCB1700_SRC_MCB1700_H
#define _CONFIGS_MCB1700_SRC_MCB1700_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* MCB1700 GPIO Pin Definitions *****************************************************/

#define MCB1700_LED1             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN18)
#define MCB1700_LED1_OFF          MCB1700_LED1
#define MCB1700_LED1_ON          (MCB1700_LED1 | GPIO_VALUE_ONE)
#define MCB1700_LED2             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN20)
#define MCB1700_LED2_OFF          MCB1700_LED2
#define MCB1700_LED2_ON          (MCB1700_LED2 | GPIO_VALUE_ONE)
#define MCB1700_LED3             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN21)
#define MCB1700_LED3_OFF          MCB1700_LED3
#define MCB1700_LED3_ON          (MCB1700_LED3 | GPIO_VALUE_ONE)
#define MCB1700_LED4             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN23)
#define MCB1700_LED4_OFF         MCB1700_LED4
#define MCB1700_LED4_ON          (MCB1700_LED 4| GPIO_VALUE_ONE)

#define MCB1700_HEARTBEAT        MCB1700_LED4

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/****************************************************************************
 * Name: mcb1700_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int mcb1700_bringup(void);

/************************************************************************************
 * Name: mcb1700_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NUCLEUS-2G board.
 *
 ************************************************************************************/

void weak_function mcb1700_sspdev_initialize(void);

/************************************************************************************
 * Name: mcb1700_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ************************************************************************************/

#ifdef CONFIG_PWM
int mcb1700_pwm_setup(void);
#endif

/************************************************************************************
 * Name: mcb1700_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ************************************************************************************/

#ifdef CONFIG_ADC
int mcb1700_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_MCB1700_SRC_MCB1700_H */

