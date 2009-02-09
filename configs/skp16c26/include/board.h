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

#define	XIN_FREQ	20e6	/* 20MHz */

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

#define	RED_DDR 	pd8_0		// LED port direction register
#define	YLW_DDR 	pd7_4
#define	GRN_DDR 	pd7_2

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
