/****************************************************************************
 * boards/arm/lpc17xx_40xx/mcb1700/src/mcb1700.h
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

#ifndef __BOARDSS_ARM_LPC17XX_40XX_MCB1700_SRC_MCB1700_H
#define __BOARDSS_ARM_LPC17XX_40XX__MCB1700_SRC_MCB1700_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MCB1700 GPIO Pin Definitions *********************************************/

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mcb1700_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int mcb1700_bringup(void);

/****************************************************************************
 * Name: mcb1700_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NUCLEUS-2G board.
 *
 ****************************************************************************/

void weak_function mcb1700_sspdev_initialize(void);

/****************************************************************************
 * Name: mcb1700_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int mcb1700_pwm_setup(void);
#endif

/****************************************************************************
 * Name: mcb1700_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int mcb1700_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDSS_ARM_LPC17XX_40XX__MCB1700_SRC_MCB1700_H */
