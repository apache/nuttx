/************************************************************************************
 * configs/olimex-lpc-h3131/include/board.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __CONFIGS_OLIMEX_LPC_H3131_INCLUDE_BOARD_H
#define __CONFIGS_OLIMEX_LPC_H3131_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
# include "lpc31_cgudrvr.h"
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* Frequency of the FFAST input */

#define BOARD_FREQIN_FFAST     (12000000) /* ffast (12 MHz crystal) */

/* HPLL0 configuration */

#define BOARD_HPLL0_FINSEL CGU_HPFINSEL_FFAST /* Frequency input selection */
#define BOARD_HPLL0_NDEC   131                /* PLL N-divider value */
#define BOARD_HPLL0_MDEC   29784              /* PLL M-divider value */
#define BOARD_HPLL0_PDEC   7                  /* PLL P-divider value */
#define BOARD_HPLL0_SELR   0                  /* SELR bandwidth selection */
#define BOARD_HPLL0_SELI   8                  /* SELI bandwidth selection */
#define BOARD_HPLL0_SELP   31                 /* SELP bandwidth selection */
#define BOARD_HPLL0_MODE   0                  /* PLL mode */
#define BOARD_HPLL0_FREQ   406425600          /* Frequency of the PLL in MHz */

/* HPLL1 configuration */

#define BOARD_HPLL1_FINSEL CGU_HPFINSEL_FFAST /* Frequency input selection */
#define BOARD_HPLL1_NDEC   770                /* PLL N-divider value */
#define BOARD_HPLL1_MDEC   8191               /* PLL M-divider value */
#define BOARD_HPLL1_PDEC   98                 /* PLL P-divider value */
#define BOARD_HPLL1_SELR   0                  /* SELR bandwidth selection */
#define BOARD_HPLL1_SELI   16                 /* SELI bandwidth selection */
#define BOARD_HPLL1_SELP   8                  /* SELP bandwidth selection */
#define BOARD_HPLL1_MODE   0                  /* PLL mode */
#define BOARD_HPLL1_FREQ  180000000           /* Frequency of the PLL in MHz */

/* The following 3 bitsets determine which clocks will be enabled at initialization
 * time.
 */

#define BOARD_CLKS_0_31 \
 (_RBIT(CLKID_APB0CLK,0)|_RBIT(CLKID_APB1CLK,0)|_RBIT(CLKID_APB2CLK,0)|\
  _RBIT(CLKID_APB3CLK,0)|_RBIT(CLKID_APB4CLK,0)|_RBIT(CLKID_AHB2INTCCLK,0)|\
  _RBIT(CLKID_AHB0CLK,0)|_RBIT(CLKID_ARM926CORECLK,0)|_RBIT(CLKID_ARM926BUSIFCLK,0)|\
  _RBIT(CLKID_ARM926RETIMECLK,0)|_RBIT(CLKID_ISRAM0CLK,0)|_RBIT(CLKID_ISRAM1CLK,0)|\
  _RBIT(CLKID_ISROMCLK,0)|_RBIT(CLKID_INTCCLK,0)|_RBIT(CLKID_AHB2APB0PCLK,0)|\
  _RBIT(CLKID_EVENTROUTERPCLK,0)|_RBIT(CLKID_CLOCKOUT,0))

#define BOARD_CLKS_32_63 \
 (_RBIT(CLKID_IOCONFPCLK,32)|_RBIT(CLKID_CGUPCLK,32)|_RBIT(CLKID_SYSCREGPCLK,32)|\
  _RBIT(CLKID_OTPPCLK,32)|_RBIT(CLKID_AHB2APB1PCLK,32)|_RBIT(CLKID_AHB2APB2PCLK,32)|\
  _RBIT(CLKID_AHB2APB3PCLK,32)|_RBIT(CLKID_EDGEDETPCLK,32))

#define BOARD_CLKS_64_92 \
 (0)

/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with lpc31_setled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

/* LED bits for use with lpc31_setleds() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is as follows:
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                         LED2   LED1
 *   ------------------------  --------------------------  ------ ------ */

#define LED_STARTED          0 /* NuttX has been started   OFF    OFF    */
#define LED_HEAPALLOCATE     0 /* Heap has been allocated  OFF    OFF    */
#define LED_IRQSENABLED      0 /* Interrupts enabled       OFF    OFF    */
#define LED_STACKCREATED     1 /* Idle stack created       ON     OFF    */
#define LED_INIRQ            2 /* In an interrupt          N/C    N/C    */
#define LED_SIGNAL           2 /* In a signal handler      N/C    N/C    */
#define LED_ASSERTION        2 /* An assertion failed      N/C    N/C    */
#define LED_PANIC            3 /* The system has crashed   N/C  Blinking */
#undef  LED_IDLE               /* MCU is is sleep mode       Not used    */

/* Thus if LED2 is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.  If LED1 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 *
 * NOTE: That LED2 is not used after completion of booting and may
 * be used by other board-specific logic.
 */

/* Button definitions ***************************************************************/
/* The Olimex LPC_H3131 has no user buttons */

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: lpc31_boardinitialize
 *
 * Description:
 *   All LPC31XX architectures must provide the following entry point.  This entry
 *   point is called early in the intitialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void lpc31_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

/************************************************************************************
 * Name:  lpc31_setled, and lpc31_setleds
 *
 * Description:
 *   These interfaces allow user control of the board LEDs.
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control both on-board LEDs up
 *   until the completion of boot.  The it will continue to control LED1; LED2 is
 *   avaiable for application use.
 *
 *   If CONFIG_ARCH_LEDS is not defined, then both LEDs are available for application
 *   use.
 *
 ************************************************************************************/

void lpc31_setled(int led, bool ledon);
void lpc31_setleds(uint8_t ledset);

/************************************************************************************
 * Name: lpc31_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an overcurrent condition is
 *   detected.
 *
 * Input parameter:
 *   handler - New overcurrent interrupt handler
 *
 * Returned value:
 *   Old overcurrent interrupt handler
 *
 ************************************************************************************/

#if 0 /* Not ready yet */
xcpt_t lpc31_setup_overcurrent(xcpt_t handler);
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_OLIMEX_LPC_H3131_INCLUDE_BOARD_H */
