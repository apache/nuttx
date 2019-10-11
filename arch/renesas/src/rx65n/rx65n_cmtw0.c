/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_cmtw0.c
 *
 *   Copyright (C) 2008-2019 Gregory Nutt. All rights reserved.
 *   Author:Surya Prakash <surya.prakash@tataelxsi.co.in>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <queue.h>
#include <errno.h>

#include "rx65n_cmtw0.h"
#include "rx65n/iodefine.h"
#include "rx65n/irq.h"
#include "rx65n_definitions.h"
#include <nuttx/irq.h>
#include "rx65n_eth.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_cmtw0_create
 *
 * Description:
 * CMTW0 Timer Initialization
 ****************************************************************************/

void rx65n_cmtw0_create(uint32_t txpoll_time, uint32_t txtimeout_time)
{
  /* Disable OC0I0 interrupt */

  IEN(PERIB, INTB170) = 0U;

  /* Disable OC1I0 interrupt */

  IEN(PERIB, INTB171) = 0U;

  /* Cancel CMTW stop state in LPC */

  MSTP(CMTW0) = 0U;

  /* Stop CMTW0 count */

  CMTW0.CMWSTR.BIT.STR = 0U;

  /* Set timer I/O control register */

  CMTW0.CMWIOR.WORD =  _1000_CMTW_CMWIOR_OC0E_ENABLE |
                        _0000_CMTW_CMWIOR_OC0_RETAIN |
                        _2000_CMTW_CMWIOR_OC1E_ENABLE |
                        _0000_CMTW_CMWIOR_OC1_RETAIN |
                        _8000_CMTW_CMWIOR_CMWE_ENABLE;

  /* Set compare match register */

  CMTW0.CMWCOR = _000000BB_CMTW0_CMWCOR_VALUE;

  /* Set output compare register 0 */

  CMTW0.CMWOCR0 = txpoll_time;

  /* Set output compare register 1 */

  CMTW0.CMWOCR1 = txtimeout_time;

  /* Set control registers */

  CMTW0.CMWCR.WORD = _0001_CMTW_CMWCR_CLOCK_PCLK32 |
                     _0040_CMTW_CMWCR_OC0IE_ENABLE |
                     _0080_CMTW_CMWCR_OC1IE_ENABLE |
                     _0000_CMTW_CMWCR_COUNTER_SIZE_32 |
                     _0000_CMTW_CMWCR_CCLR_ENABLE_CMWCOR;

  /* Set OC0I0 interrupt and priority level */

  ICU.SLIBR170.BYTE = 0x2bu;
  IPR(PERIB, INTB170) = _0F_CMTW_PRIORITY_LEVEL15;

  /* Set OC1I0 interrupt and priority level */

  ICU.SLIBR171.BYTE = 0x2cu;
  IPR(PERIB, INTB171) = _0F_CMTW_PRIORITY_LEVEL15;

  /* Set TIC0 pin */

  MPC.PC6PFS.BYTE = 0x1du;
  PORTC.PMR.BYTE |= 0x40u;

  /* Set TIC1 pin */

  MPC.PE6PFS.BYTE = 0x1du;
  PORTE.PMR.BYTE |= 0x40u;

  /* Set TOC0 pin */

  MPC.PC7PFS.BYTE = 0x1du;
  PORTC.PMR.BYTE |= 0x80u;

  /* Set TOC1 pin */

  MPC.PE7PFS.BYTE = 0x1du;
  PORTE.PMR.BYTE |= 0x80u;

  /* Attach the IRQ for poll expiry */

  irq_attach(RX65N_INTB170_IRQ, (xcpt_t)rx65n_poll_expiry, NULL);

  /* Attach the IRQ for tx timeout */

  irq_attach(RX65N_INTB171_IRQ, (xcpt_t)rx65n_txtimeout_expiry, NULL);
}

/****************************************************************************
 * Name: rx65n_cmtw0_start
 *
 * Description:
 * CMTW0 Timer Initialization
 ****************************************************************************/

void rx65n_cmtw0_start(uint8_t type, uint32_t timeout)
{
  /* Update OC0I0 for polling */

  if (type == rx65n_cmtw0_txpoll)
    {
      CMTW0.CMWOCR0 = CMTW0.CMWCNT + timeout;

      /* Enable OC0I0 interrupt in ICU */

      IEN(PERIB, INTB170) = 1U;

      /* Enabling OC0 */

      CMTW0.CMWIOR.WORD |=  _1000_CMTW_CMWIOR_OC0E_ENABLE;
    }

  /* Update OC0I1 for timeout */

  if (type == rx65n_cmtw0_timeout)
    {
      CMTW0.CMWOCR1 = CMTW0.CMWCNT + timeout;

      /* Enable OC1I0 interrupt in ICU */

      IEN(PERIB, INTB171) = 1U;

      /* Enabling OC1 */

     CMTW0.CMWIOR.WORD |=  _2000_CMTW_CMWIOR_OC1E_ENABLE;
    }

  /* Start CMTW0 count */

    CMTW0.CMWSTR.BIT.STR = 1U;
}

/****************************************************************************
 * Name: rx65n_cmtw0_stop
 *
 * Description:
 * CMTW0 Timer Initialization
 ****************************************************************************/

void rx65n_cmtw0_stop(uint8_t type)
{
  /* STOP OC0I0 fot polling */

  if (type == rx65n_cmtw0_txpoll)
    {
      /* Disable OC0I0 interrupt in ICU */

      IEN(PERIB, INTB170) = 0U;

      /* Enabling OC0 */

      CMTW0.CMWIOR.WORD &=  ~(_1000_CMTW_CMWIOR_OC0E_ENABLE);
    }

  /* Stop OC0I1 fot timeout */

  if (type == rx65n_cmtw0_timeout)
    {
      /* Disable OC1I0 interrupt in ICU */

      IEN(PERIB, INTB171) = 0U;

      /* Disabling OC1 */

     CMTW0.CMWIOR.WORD &=  ~(_2000_CMTW_CMWIOR_OC1E_ENABLE);
    }
}
