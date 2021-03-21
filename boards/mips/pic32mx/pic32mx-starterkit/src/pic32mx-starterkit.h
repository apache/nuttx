/****************************************************************************
 * boards/mips/pic32mx/pic32mx-starterkit/src/pic32mx-starterkit.h
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

#ifndef __BOARDS_MIPS_PIC32MX_PIC32MX_STARTERKIT_SRC_PIC32MX_STARTERKIT_H
#define __BOARDS_MIPS_PIC32MX_PIC32MX_STARTERKIT_SRC_PIC32MX_STARTERKIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The PIC32 starter kit has 3 user LEDs
 *
 *   RD0          User LED D4 (high illuminates)
 *   RD2          User LED D5 (high illuminates)
 *   RD1          User LED D6 (high illuminates)
 *
 * There are 5 LEDs available on the MEB:
 *
 *   RD1          LED1
 *   RD2          LED2
 *   RD3          LED3
 *   RC1          LED4
 *   RC2          LED5
 */

/* The PIC32 starter kit has 3 switches:
 *
 *   RD7            Switch SW2 (low when closed)
 *   RD6            Switch SW1 (low when closed)
 *   RD13           Switch SW3 (low when closed)
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pic32mx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PCB Logic board.
 *
 ****************************************************************************/

#if defined(CONFIG_PIC32MX_SPI2)
void weak_function pic32mx_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: pic32mx_led_initialize
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pic32mx_led_initialize(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_MIPS_PIC32MX_PIC32MX_STARTERKIT_SRC_PIC32MX_STARTERKIT_H */
