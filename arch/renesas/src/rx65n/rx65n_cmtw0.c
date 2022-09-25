/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_cmtw0.c
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

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include "rx65n_cmtw0.h"
#include "arch/rx65n/iodefine.h"
#include "arch/rx65n/irq.h"
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
  /* STOP OC0I0 for polling */

  if (type == rx65n_cmtw0_txpoll)
    {
      /* Disable OC0I0 interrupt in ICU */

      IEN(PERIB, INTB170) = 0U;

      /* Enabling OC0 */

      CMTW0.CMWIOR.WORD &=  ~(_1000_CMTW_CMWIOR_OC0E_ENABLE);
    }

  /* Stop OC0I1 for timeout */

  if (type == rx65n_cmtw0_timeout)
    {
      /* Disable OC1I0 interrupt in ICU */

      IEN(PERIB, INTB171) = 0U;

      /* Disabling OC1 */

     CMTW0.CMWIOR.WORD &=  ~(_2000_CMTW_CMWIOR_OC1E_ENABLE);
    }
}
