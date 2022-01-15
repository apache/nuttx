/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p107/src/olimex-stm32-p107.h
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

#ifndef __BOARDS_ARM_STM32_OLIMEX_STM32_P107_SRC_OLIMEX_STM32_P107_H
#define __BOARDS_ARM_STM32_OLIMEX_STM32_P107_SRC_OLIMEX_STM32_P107_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Olimex MOD-ENC624J600 Module
 *
 * --- ------ -------------- ------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- ------------------------------------------------
 *
 * 54  PB15   PB15-CS_UEXT   ENCX24J600 #CS
 * 78  PC10   PC10-SPI3-SCK  ENCX24J600 SCK
 * 79  PC11   PC11-SPI3-MISO ENCX24J600 MISO
 * 80  PC12   PC12-SPI3-MOSI ENCX24J600 MOSI
 * 95  PB8    PB8            ENCX24J600 #Interrupt
 */

#ifdef CONFIG_ENCX24J600
#  define GPIO_ENCX24J600_CS    (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| \
                                 GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN15)
#  define GPIO_ENCX24J600_INTR  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT| \
                                 GPIO_EXTI|GPIO_PORTB|GPIO_PIN8)
#endif

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the M3 Wildfire board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_OLIMEX_STM32_P107_SRC_OLIMEX_STM32_P107_H */
