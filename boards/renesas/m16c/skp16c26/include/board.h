/****************************************************************************
 * boards/renesas/m16c/skp16c26/include/board.h
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

#ifndef __BOARDS_RENESAS_M16C_SKP16C26_INCLUDE_BOARD_H
#define __BOARDS_RENESAS_M16C_SKP16C26_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* User configuration checks ************************************************/

/* According to SKP16C26 documentation, "SIO/UART1 pins are used for
 * communication between the SKP16C26 board kernel and KD30 Debugger through
 * the ICD. Do not connect these pins to any other circuit, as UART1 cannot
 * be used in the user program.
 * For details, please see ICD (RTA-FoUSB-MON) User Manual on Target M16C
 * ROM Monitor Resources or related ICD application notes."
 *
 * However, the schematic appears to show that SIO/UART2 is actual
 * connection.
 * To be safe, we will error out on either selection:
 */

#if defined(CONFIG_M16C_UART1) || defined(CONFIG_M16C_UART2)
#  error "UART1/2 should not be used on SKP16C26"
#endif

/* Hardware defintitions ****************************************************/

/* Xin Freq */

#define	M16C_XIN_FREQ	20000000	/* 20MHz */

/* Interrupt Priority Levels ************************************************/

/* IPL settings */

#define M16C_DEFAULT_IPL   0     /* Default M16C Interrupt priority level */
#undef  M16C_INTERRUPT_IPL       /* Default interrupt IPL to enabled nested interrupts */

/* Define any of the following to specify interrupt priorities.  A default
 * value of 5 will be used for any unspecified values
 */

#undef  M16C_INT3_PRIO           /* INT3 interrupt priority  */
#undef  M16C_INT5_PRIO           /* INT5 interrupt priority */
#undef  M16C_INT4_PRIO           /* INT4 interrupt priority */
#undef  M16C_BCN_PRIO            /* Bus collision detection interrupt priority  */
#undef  M16C_DM0_PRIO            /* DMA0 interrupt priority */
#undef  M16C_DM1_PRIO            /* DMA1 interrupt priority */
#undef  M16C_KUP_PRIO            /* Key input interrupt priority */
#undef  M16C_AD_PRIO             /* A-D conversion interrupt priority */
#undef  M16C_S2T_PRIO            /* UART2 transmit interrupt priority */
#undef  M16C_S2R_PRIO            /* UART2 receive interrupt priority */
#undef  M16C_S0T_PRIO            /* UART0 transmit interrupt priority */
#undef  M16C_S0R_PRIO            /* UART0 receive interrupt priority */
#undef  M16C_S1T_PRIO            /* UART1 transmit interrupt priority */
#undef  M16C_S1R_PRIO            /* UART1 receive interrupt priority */
#define M16C_TA0_PRIO   5        /* Timer A0 interrupt priority */
#undef  M16C_TA1_PRIO            /* Timer A1 interrupt priority */
#undef  M16C_TA2_PRIO            /* Timer A2 interrupt priority */
#undef  M16C_TA3_PRIO            /* Timer A3 interrupt priority */
#undef  M16C_TA4_PRIO            /* Timer A4 interrupt priority */
#undef  M16C_TB0_PRIO            /* Timer B0 interrupt priority */
#undef  M16C_TB1_PRIO            /* Timer B1 interrupt priority */
#undef  M16C_TB2_PRIO            /* Timer B2 interrupt priority */
#undef  M16C_INT0_PRIO           /* INT0 interrupt priority */
#undef  M16C_INT1_PRIO           /* INT1 interrupt priority */

/* LED definitions **********************************************************/

                                 /* GREEN YELLOW RED */
#define LED_STARTED       0      /*  OFF   OFF   OFF */
#define LED_HEAPALLOCATE  1      /*  ON    OFF   OFF */
#define LED_IRQSENABLED   2      /*  OFF   ON    OFF */
#define LED_STACKCREATED  3      /*  ON    ON    OFF */
#define LED_INIRQ         4      /*  ON    OFF   ON  */
#define LED_SIGNAL        5      /*  OFF   ON    ON  */
#define LED_ASSERTION     6      /*  ON    ON    ON  */
#define LED_PANIC         7      /*  NC**  NC**  ON* */

/* *=FLASHING **=if INIRQ, SIGNAL, or ASSERTION will be flashing */

/* BUTTON definitions *******************************************************/

#define SW1_PRESSED       0x01   /* Bit 0: 1=SW1 pressed */
#define SW2_PRESSED       0x02   /* Bit 1: 1=SW2 pressed */
#define SW3_PRESSED       0x04   /* Bit 2: 1=SW3 pressed */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif /* __BOARDS_RENESAS_M16C_SKP16C26_INCLUDE_BOARD_H */
