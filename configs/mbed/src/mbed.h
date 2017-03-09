/************************************************************************************
 * configs/mbed/src/mbed.h
 *
 *   Copyright (C) 2010, 2016 Gregory Nutt. All rights reserved.
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

#ifndef _CONFIGS_MBED_SRC_MBED_H
#define _CONFIGS_MBED_SRC_MBED_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* MBED GPIO Pin Definitions ********************************************************/

#define MBED_LED1             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN18)
#define MBED_LED1_OFF          MBED_LED1
#define MBED_LED1_ON          (MBED_LED1 | GPIO_VALUE_ONE)
#define MBED_LED2             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN20)
#define MBED_LED2_OFF          MBED_LED2
#define MBED_LED2_ON          (MBED_LED2 | GPIO_VALUE_ONE)
#define MBED_LED3             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN21)
#define MBED_LED3_OFF          MBED_LED3
#define MBED_LED3_ON          (MBED_LED3 | GPIO_VALUE_ONE)
#define MBED_LED4             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN23)
#define MBED_LED4_OFF         MBED_LED4
#define MBED_LED4_ON          (MBED_LED 4| GPIO_VALUE_ONE)

#define MBED_HEARTBEAT        MBED_LED4

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
 * Name: mbed_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NUCLEUS-2G board.
 *
 ************************************************************************************/

void weak_function mbed_sspdev_initialize(void);

/************************************************************************************
 * Name: mbed_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ************************************************************************************/

#ifdef CONFIG_PWM
int mbed_pwm_setup(void);
#endif

/************************************************************************************
 * Name: mbed_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ************************************************************************************/

#ifdef CONFIG_ADC
int mbed_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_MBED_SRC_MBED_H */

