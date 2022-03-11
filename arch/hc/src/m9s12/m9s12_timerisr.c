/****************************************************************************
 * arch/hc/src/m9s12/m9s12_timerisr.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "clock/clock.h"
#include "up_internal.h"
#include "chip.h"
#include "m9s12.h"
#include "m9s12_crg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 */

/* The timer frequency is the OSCCLK divided down.  The divisor is
 * (A+1)*(2**(B+9)) where:
 *
 *   A = MODCNT RTR[0:3]
 *   B = PREP   RTR[4:6]
 *
 * Maximum and minimum values:
 */

#define MIN_PRER    1024 /* 2**10, B=1 */
#define MAX_PRER   65536 /* 2**16, B=7 */

#define MIN_MODCNT     1 /* A=0 */
#define MAX_MODCNT    16 /* A=15 */

/* Pick the smallest value of B for which:
 *
 *   OSCCLK/(MAX_MODCNT*(2**(B+9))) >= CLK_TCK >=
  *                                            OSCCLK/(MIN_MODCNT*(2**(B+9)))
 */

#if CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*1024) && HCS12_OSCCLK/(MIN_MODCNT*1024)
#  define PRER_VALUE      1
#  define PRER_DIV     1024
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*2048) && HCS12_OSCCLK/(MIN_MODCNT*2048)
#  define PRER_VALUE      2
#  define PRER_DIV     2048
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*4096) && HCS12_OSCCLK/(MIN_MODCNT*4096)
#  define PRER_VALUE      3
#  define PRER_DIV     4096
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*8192) && HCS12_OSCCLK/(MIN_MODCNT*8192)
#  define PRER_VALUE      4
#  define PRER_DIV     8192
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*16384) && HCS12_OSCCLK/(MIN_MODCNT*16384)
#  define PRER_VALUE      5
#  define PRER_DIV    16384
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*32768) && HCS12_OSCCLK/(MIN_MODCNT*32768)
#  define PRER_VALUE      6
#  define PRER_DIV    32768
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*65536) && HCS12_OSCCLK/(MIN_MODCNT*65536)
#  define PRER_VALUE      7
#  define PRER_DIV    65536
#else
#  error "Cannot generate CLK_TCK from HCSCLK_OSCCLK"
#endif

/* Now we can simply calculate A from:
 *
 * CLK_TCK = OSCCLK/((A+1)*PRER_DIV)
 * OSCCLK / (CLK_TCK * PRER_DIV) - 1
 */

#define MODCNT_DENOM  ((uint32_t)CLK_TCK * (uint32_t)PRER_DIV)
#define MODCNT_VALUE  ((((uint32_t)HCS12_OSCCLK  + (MODCNT_DENOM/2))/ MODCNT_DENOM) - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  m9s12_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int m9s12_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear real time interrupt flag */

  putreg8(CRG_CRGFLG_RTIF, HCS12_CRG_CRGFLG);

  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the system timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t tmp;
  uint8_t  regval;

  /* Configure hardware RTI timer (with a hack to avoid and integer overflow
   * error at compile time.. hopefully the optimizer will eliminate these
   * uint32_t operations).
   */

  tmp = MODCNT_VALUE << CRG_RTICTL_MODCNT_SHIFT |
                        PRER_VALUE << CRG_RTICTL_PRER_SHIFT;
  putreg8((uint8_t)tmp, HCS12_CRG_RTICTL);

  /* Attach the timer interrupt vector */

  irq_attach(HCS12_IRQ_VRTI, (xcpt_t)m9s12_timerisr, NULL);

  /* Enable RTI interrupt by setting the RTIE bit */

  regval  = getreg8(HCS12_CRG_CRGINT);
  regval |= CRG_CRGINT_RTIE;
  putreg8(regval, HCS12_CRG_CRGINT);
}
