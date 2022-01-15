/****************************************************************************
 * boards/arm/lpc17xx_40xx/mbed/src/mbed.h
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_MBED_SRC_MBED_H
#define __BOARDS_ARM_LPC17XX_40XX_MBED_SRC_MBED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MBED GPIO Pin Definitions ************************************************/

#define MBED_LED1        (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN18)
#define MBED_LED2        (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN20)
#define MBED_LED3        (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN21)
#define MBED_LED4        (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN23)
#define MBED_HEARTBEAT   MBED_LED4

/* LED bits for use with board_userled_all() */
#define MBED_NLEDS      4

#define MBED_LED1_BIT    (1 << 0)
#define MBED_LED2_BIT    (1 << 1)
#define MBED_LED3_BIT    (1 << 2)
#define MBED_LED4_BIT    (1 << 3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: mbed_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NUCLEUS-2G board.
 *
 ****************************************************************************/

void weak_function mbed_sspdev_initialize(void);

/****************************************************************************
 * Name: mbed_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int mbed_pwm_setup(void);
#endif

/****************************************************************************
 * Name: mbed_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int mbed_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC17XX_40XX_MBED_SRC_MBED_H */
