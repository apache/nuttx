/****************************************************************************
 * boards/sparc/bm3823/xx3823/include/board.h
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

#ifndef __BOARDS_SPARC_BM3823_XX3823_INCLUDE_BOARD_H
#define __BOARDS_SPARC_BM3823_XX3823_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Clocking *****************************************************************/

/* Assume default CLKDIV8 fuse setting is overridden to CLKDIV1 */

#define BOARD_XTAL_FREQ        25000000L         /* 25MHz crystal */

/* F_CPU = 300MHz */

#define BOARD_CPU_CLOCK        ((uint32_t)((BOARD_XTAL_FREQ*8*3)/2))

/* F_peripheral = 100MHz */

#define BOARD_PERIPH_CLOCK     ((uint32_t)(BOARD_CPU_CLOCK/3))
#define CHIP_NUARTS            2

/* LED definitions **********************************************************/

/* ----- ----- -------------------------------------------------------------
 * LABEL COLOR CONTROL
 * ----- ----- -------------------------------------------------------------
 * USB   Green   RF3. This could be used by software if USB is not used.
 *                    Otherwise, RF3 is used as the USBID signal.
 * LED1  While   RE2, Pulled up.  Low value illuminates
 * LED2  Red     RE1, Pulled up.  Low value illuminates
 * LED3  Yellow  RE0, Pulled up.  Low value illuminates
 * PWR   Blue    Illuminated when 5V is present, not controlled by software
 */

/* LED index values for use with board_userled() */

#define BM3823_XX3823_LED1     0
#define BM3823_XX3823_LED2     1
#define BM3823_XX3823_LED3     2
#define BM3823_XX3823_NLEDS    3

/* LED bits for use with board_userled_all() */

#define BM3823_XX3823_LED1_BIT (1 << BM3823_XX3823_LED1)
#define BM3823_XX3823_LED2_BIT (1 << BM3823_XX3823_LED2)
#define BM3823_XX3823_LED3_BIT (1 << BM3823_XX3823_LED3)

/* If CONFIG_ARCH_LEDS is defined,
 * then NuttX will control these LEDs as follows:
 *
 *                           ON                  OFF
 * ------------------------- ---- ---- ---- ---- ---- ----
 *                           LED1 LED2 LED3 LED1 LED2 LED3
 * ------------------------- ---- ---- ---- ---- ---- ----
 * LED_STARTED            0  OFF  OFF  OFF  ---  ---  ---
 * LED_HEAPALLOCATE       1  ON   OFF  N/C  ---  ---  ---
 * LED_IRQSENABLED        2  OFF  ON   N/C  ---  ---  ---
 * LED_STACKCREATED       3  ON   ON   N/C  ---  ---  ---
 * LED_INIRQ              4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_SIGNAL             4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_ASSERTION          4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_PANIC              5  ON   N/C  N/C  OFF  N/C  N/C
 */

#define LED_STARTED            0
#define LED_HEAPALLOCATE       1
#define LED_IRQSENABLED        2
#define LED_STACKCREATED       3
#define LED_INIRQ              4
#define LED_SIGNAL             4
#define LED_ASSERTION          4
#define LED_PANIC              5

#define LED_NVALUES            6
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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_SPARC_BM3823_XX3823_INCLUDE_BOARD_H */
