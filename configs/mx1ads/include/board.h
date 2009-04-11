/************************************************************************************
 * configs/mx1ads/include/board.h
 * include/arch/board/board.h
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
# include <sys/types.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clock settings */

#define IMX_SYS_CLK_FREQ    16780000
#define IMX_SYSPLL_CLK_FREQ 16000000
#define IMX_PERCLK1_FREQ    96000000

/* MPCTL */

#define IMX_MPCTL0_VALUE    0x04632410 /* For 150MHz MCU PLL clock */
#define IMX_MPCTL0_VALUE    0x03AA11B9 /* For 150 MHz ARM clock with 32.768 KHz crystal */

/* SPCTL */

#define IMX_SPCTL0_VALUE    0x07AA16A6; /* For 96MHz peripheral clock with 32.768 KHz crystal */

/* PDCR */

#define IMX_PCDR_VALUE      0x00000055

/* LED definitions ******************************************************************/

/* The MX1ADS has only one usable LED: Port A, bit 2 */

                                /* ON   OFF */
#define LED_STARTED       0     /* OFF  OFF */
#define LED_HEAPALLOCATE  1     /* OFF  OFF */
#define LED_IRQSENABLED   2     /* OFF  OFF */
#define LED_STACKCREATED  3     /* OFF  OFF */
#define LED_INIRQ         4     /* ON   OFF */
#define LED_SIGNAL        5     /* ON   OFF */
#define LED_ASSERTION     6     /* ON   OFF */
#define LED_PANIC         7     /* ON   OFF */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif  /* __ARCH_BOARD_BOARD_H */
