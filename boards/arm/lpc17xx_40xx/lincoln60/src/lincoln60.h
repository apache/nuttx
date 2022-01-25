/****************************************************************************
 * boards/arm/lpc17xx_40xx/lincoln60/src/lincoln60.h
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_LINCOLN60_SRC_LINCOLN60_H
#define __BOARDS_ARM_LPC17XX_40XX_LINCOLN60_SRC_LINCOLN60_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 *  LEDs GPIO                        PIN  SIGNAL NAME
 *  -------------------------------- ---- --------------
 *  P1[18]                            32  LED1
 *  P3[26]                            26  LED2
 ****************************************************************************/

#define LINCOLN60_LED1              (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN18)
#define LINCOLN60_LED1_OFF          LINCOLN60_LED1
#define LINCOLN60_LED1_ON           (LINCOLN60_LED1 | GPIO_VALUE_ONE)
#define LINCOLN60_LED2              (GPIO_OUTPUT | GPIO_PORT3 | GPIO_PIN26)
#define LINCOLN60_LED2_OFF          LINCOLN60_LED2
#define LINCOLN60_LED2_ON           (LINCOLN60_LED2 | GPIO_VALUE_ONE)

#define LINCOLN60_HEARTBEAT         LINCOLN60_LED2

/****************************************************************************
 *  Buttons GPIO                     PIN  SIGNAL NAME
 *  -------------------------------- ---- --------------
 *  P2[10]                            53  BTN1
 ****************************************************************************/

#define LINCOLN60_BUT1              (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | \
                                     GPIO_PIN10)

/* Button IRQ numbers */

#define LINCOLN60_BUT1_IRQ          LPC17_40_IRQ_P0p23

/****************************************************************************
 *  microSD                          PIN   SIGNAL NAME
 *  -------------------------------- ----- --------------
 *  P0[15]                           J12 3  SPI SCK
 *  P0[17]                           J12 4  SPI MISO
 *  P0[18]                           J12 5  SPI MOSI
 *  P0[16]                           J18 5  SPI slave select
 ****************************************************************************/

#define LINCOLN60_CS               (GPIO_OUTPUT | GPIO_VALUE_ONE | \
                                    GPIO_PORT0 | GPIO_PIN16)

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
 * Name: lincoln60_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Lincoln 60 board.
 *
 ****************************************************************************/

void weak_function lincoln60_sspdev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC17XX_40XX_LINCOLN60_SRC_LINCOLN60_H */
