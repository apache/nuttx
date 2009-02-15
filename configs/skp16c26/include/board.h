/************************************************************************************
 * configs/skp16c26/include/board.h 
 * arch/board/board.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_SKP16C26_INCLUDE_BOARD_H
#define __CONFIGS_SKP16C26_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
# include <sys/types.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Xin Freq */

#define	M16C_XIN_FREQ	20000000	/* 20MHz */

/* Switches */

#define	S1 		p8_3 
#define S2 		p8_2 
#define S3 		p8_1 
#define S1_DDR		pd8_3
#define S2_DDR		pd8_2
#define S3_DDR		pd8_1

/* LEDs */

#define	RED_LED		p8_0
#define	YLW_LED		p7_4
#define	GRN_LED		p7_2

#define	RED_DDR 	pd8_0		/* LED port direction register */
#define	YLW_DDR 	pd7_4
#define	GRN_DDR 	pd7_2

/* IPL settings */

#define M16C_DEFAULT_IPL   0     /* Default M16C Interrupt priority level */
#undef  M16C_INTERRUPT_IPL       /* Default interrupt IPL to enabled nested interrupts */

/* Define any of the following to specify interrupt priorities.  A default
 * value of 5 will be used for any unspecified values
 */

#undef  M16C_INT3_PRIO                  /* INT3 interrupt priority  */
#undef  M16C_INT5_PRIO                  /* INT5 interrupt priority */
#undef  M16C_INT4_PRIO                  /* INT4 interrupt priority */
#undef  M16C_BCN_PRIO                   /* Bus collision detection interrupt priority  */
#undef  M16C_DM0_PRIO                   /* DMA0 interrupt priority */
#undef  M16C_DM1_PRIO                   /* DMA1 interrupt priority */
#undef  M16C_KUP_PRIO                   /* Key input interrupt priority */
#undef  M16C_AD_PRIO                    /* A-D conversion interrupt priority */
#undef  M16C_S2T_PRIO                   /* UART2 transmit interrupt priority    */
#undef  M16C_S2R_PRIO                   /* UART2 receive interrupt priority    */
#undef  M16C_S0T_PRIO                   /* UART0 transmit interrupt priority */
#undef  M16C_S0R_PRIO                   /* UART0 receive interrupt priority */
#undef  M16C_S1T_PRIO                   /* UART1 transmit interrupt priority */
#undef  M16C_S1R_PRIO                   /* UART1 receive interrupt priority */
#define M16C_TA0_PRIO   5               /* Timer A0 interrupt priority */
#undef  M16C_TA1_PRIO                   /* Timer A1 interrupt priority */
#undef  M16C_TA2_PRIO                   /* Timer A2 interrupt priority */
#undef  M16C_TA3_PRIO                   /* Timer A3 interrupt priority */
#undef  M16C_TA4_PRIO                   /* Timer A4 interrupt priority */
#undef  M16C_TB0_PRIO                   /* Timer B0 interrupt priority */
#undef  M16C_TB1_PRIO                   /* Timer B1 interrupt priority */
#undef  M16C_TB2_PRIO                   /* Timer B2 interrupt priority */
#undef  M16C_INT0_PRIO                  /* INT0 interrupt priority */
#undef  M16C_INT1_PRIO                  /* INT1 interrupt priority */

/********************************************************************************/
/* Macro Definitions 															*/
/********************************************************************************/

#define LED_ON      	0
#define LED_OFF     	1

/* Use these macros for switch inputs */

#define ENABLE_SWITCHES {S1_DDR = 0; S2_DDR = 0; S3_DDR = 0;}

/* Use these macros to control the LEDs */

#define LED(led, state) ((led) = !state)
#define ENABLE_LEDS {RED_LED = LED_OFF; YLW_LED = LED_OFF; GRN_LED = LED_OFF; RED_DDR = 1; YLW_DDR = 1; GRN_DDR = 1; }

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif  /* __CONFIGS_SKP16C26_INCLUDE_BOARD_H */
