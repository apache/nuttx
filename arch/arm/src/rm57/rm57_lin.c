/****************************************************************************
 * arch/arm/src/rm57/rm57_lin.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* See rm57_lin.h for the scope of this LIN 2.x scaffold and what is
 * explicitly not yet implemented. Configuration sequence follows TRM
 * SPNU562A section 28.3.4; register field positions are the ones added
 * to hardware/rm57_sci.h alongside this file (LIN registers were not
 * previously defined there - only SCI-mode registers were).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_sci.h"
#include "hardware/rm57l843_memorymap.h"
#include "rm57_lin.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Crude polling timeout: a bounded iteration count rather than a
 * calibrated delay, since this scaffold does not depend on
 * CONFIG_BOARD_LOOPSPERMSEC being calibrated.
 */

#define RM57_LIN_POLL_LIMIT  1000000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_lin_base
 ****************************************************************************/

static uint32_t rm57_lin_base(int intf)
{
  switch (intf)
    {
      case 1:
        return RM57_SCI1_BASE;

      case 2:
        return RM57_SCI2_BASE;

      default:
        return 0;
    }
}

/****************************************************************************
 * Name: rm57_lin_waitflag
 *
 * Description:
 *   Poll FLR until (flr & mask) == mask, or the poll limit is reached.
 *
 ****************************************************************************/

static int rm57_lin_waitflag(uint32_t base, uint32_t mask)
{
  int i;

  for (i = 0; i < RM57_LIN_POLL_LIMIT; i++)
    {
      if ((getreg32(base + RM57_SCI_FLR_OFFSET) & mask) == mask)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_lin_initialize
 ****************************************************************************/

int rm57_lin_initialize(int intf, const struct rm57_lin_config_s *config)
{
  uint32_t base;
  uint32_t gcr1;
  uint32_t divisor;
  uint32_t p;

  base = rm57_lin_base(intf);
  if (base == 0 || config == NULL || config->length < 1 ||
      config->length > 8 || config->baud == 0)
    {
      return -EINVAL;
    }

  /* Enable the module, then hold it in software reset while it is being
   * configured (TRM 28.3.4).
   */

  putreg32(0, base + RM57_SCI_GCR0_OFFSET);
  putreg32(SCI_GCR0_RESET, base + RM57_SCI_GCR0_OFFSET);

  putreg32(SCI_INT_ALLINTS, base + RM57_SCI_CLEARINT_OFFSET);
  putreg32(SCI_INT_ALLINTS, base + RM57_SCI_CLEARINTLVL_OFFSET);

  gcr1 = SCI_GCR1_LIN_MODE | SCI_GCR1_TIMING_MODE | SCI_GCR1_MBUF_MODE |
         SCI_GCR1_RXENA | SCI_GCR1_TXENA;

  if (config->master)
    {
      gcr1 |= SCI_GCR1_CLOCK;
    }

  if (!config->classic_cs)
    {
      gcr1 |= SCI_GCR1_CTYPE;
    }

  if (config->hgen)
    {
      gcr1 |= SCI_GCR1_HGEN_CTRL;
    }

  putreg32(gcr1, base + RM57_SCI_GCR1_OFFSET);

  /* RX/TX pin function */

  putreg32(SCI_PIO_RX | SCI_PIO_TX, base + RM57_SCI_PIO0_OFFSET);

  /* Baud rate: same BRS integer-divisor scheme used for SCI mode
   * (rm57_sci_configure() in rm57_lowputc.c) - the SCI/LIN module shares
   * one baud rate generator (TRM 28.2.1.3/28.3.1.4). Adaptive baud rate
   * detection (GCR1 ADAPT, MBRS) is not used by this scaffold, so a
   * slave node here must be configured with the same fixed baud rate as
   * the master.
   */

  divisor = BOARD_VCLK_FREQUENCY / (config->baud * 16);
  p = divisor - 1;
  putreg32(SCI_BRS_P(p), base + RM57_SCI_BRS_OFFSET);

  /* Response frame length */

  putreg32(SCI_FORMAT_CHAR(config->length - 1),
           base + RM57_SCI_FORMAT_OFFSET);

  /* ID masks (TRM 28.7.26): a 0 bit compares, a 1 bit is filtered out */

  putreg32(SCI_LINMASK_TXIDMASK(config->idmask_tx) |
           SCI_LINMASK_RXIDMASK(config->idmask_rx),
           base + RM57_SCI_LINMASK_OFFSET);

  /* Release from software reset - the module is now operational */

  gcr1 |= SCI_GCR1_SWRST;
  putreg32(gcr1, base + RM57_SCI_GCR1_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: rm57_lin_uninitialize
 ****************************************************************************/

void rm57_lin_uninitialize(int intf)
{
  uint32_t base = rm57_lin_base(intf);

  if (base != 0)
    {
      putreg32(0, base + RM57_SCI_GCR1_OFFSET);
      putreg32(SCI_INT_ALLINTS, base + RM57_SCI_CLEARINT_OFFSET);
    }
}

/****************************************************************************
 * Name: rm57_lin_send
 ****************************************************************************/

int rm57_lin_send(int intf, uint8_t id, const uint8_t *data, size_t len)
{
  uint32_t base;
  uint32_t td0 = 0;
  uint32_t td1 = 0;
  int ret;

  base = rm57_lin_base(intf);
  if (base == 0 || data == NULL || len < 1 || len > 8)
    {
      return -EINVAL;
    }

  /* Wait for the transmitter to be ready for a new response, then load
   * the response data bytes (TD<n-1> holds LIN frame data byte <n>, TRM
   * 28.7.28/28.7.29), enable hardware checksum generation, and finally
   * write the ID byte - on a master node this triggers header (and,
   * with HW response generation, the response field) transmission.
   */

  ret = rm57_lin_waitflag(base, SCI_FLR_TXRDY);
  if (ret < 0)
    {
      return ret;
    }

  if (len > 0)
    {
      td0 |= (uint32_t)data[0] << SCI_LINTD_BYTE0_SHIFT;
    }

  if (len > 1)
    {
      td0 |= (uint32_t)data[1] << SCI_LINTD_BYTE1_SHIFT;
    }

  if (len > 2)
    {
      td0 |= (uint32_t)data[2] << SCI_LINTD_BYTE2_SHIFT;
    }

  if (len > 3)
    {
      td0 |= (uint32_t)data[3] << SCI_LINTD_BYTE3_SHIFT;
    }

  if (len > 4)
    {
      td1 |= (uint32_t)data[4] << SCI_LINTD_BYTE0_SHIFT;
    }

  if (len > 5)
    {
      td1 |= (uint32_t)data[5] << SCI_LINTD_BYTE1_SHIFT;
    }

  if (len > 6)
    {
      td1 |= (uint32_t)data[6] << SCI_LINTD_BYTE2_SHIFT;
    }

  if (len > 7)
    {
      td1 |= (uint32_t)data[7] << SCI_LINTD_BYTE3_SHIFT;
    }

  putreg32(td0, base + RM57_SCI_LINTD0_OFFSET);
  if (len > 4)
    {
      putreg32(td1, base + RM57_SCI_LINTD1_OFFSET);
    }

  putreg32(getreg32(base + RM57_SCI_GCR2_OFFSET) | SCI_GCR2_SC,
           base + RM57_SCI_GCR2_OFFSET);

  putreg32(SCI_LINID_IDBYTE(id), base + RM57_SCI_LINID_OFFSET);

  /* Wait for the response (and checksum) to finish transmitting */

  return rm57_lin_waitflag(base, SCI_FLR_TXRDY | SCI_FLR_TXEMPTY);
}

/****************************************************************************
 * Name: rm57_lin_recv
 ****************************************************************************/

int rm57_lin_recv(int intf, uint8_t *id, uint8_t *data, size_t len)
{
  uint32_t base;
  uint32_t flr;
  uint32_t rd0;
  uint32_t rd1;
  int ret;

  base = rm57_lin_base(intf);
  if (base == 0 || id == NULL || data == NULL || len < 1 || len > 8)
    {
      return -EINVAL;
    }

  putreg32(getreg32(base + RM57_SCI_GCR2_OFFSET) | SCI_GCR2_CC,
           base + RM57_SCI_GCR2_OFFSET);

  /* Wait for a header with a matching identifier */

  ret = rm57_lin_waitflag(base, SCI_FLR_IDRX);
  if (ret < 0)
    {
      return ret;
    }

  *id = (uint8_t)(getreg32(base + RM57_SCI_LINID_OFFSET) >>
                  SCI_LINID_RECEIVEDID_SHIFT);

  /* Wait for the response (and checksum) to finish arriving. Per TRM
   * 28.3.1.10, for LENGTH &lt;= 4 a read of LINRD0 clears RXRDY; for
   * LENGTH &gt; 4, LINRD1 must be read instead.
   */

  ret = rm57_lin_waitflag(base, SCI_FLR_RXRDY);
  if (ret < 0)
    {
      return ret;
    }

  flr = getreg32(base + RM57_SCI_FLR_OFFSET);
  if ((flr & (SCI_FLR_CE | SCI_FLR_PBE | SCI_FLR_ISFE | SCI_FLR_NRE |
              SCI_FLR_BE | SCI_FLR_FE | SCI_FLR_OE)) != 0)
    {
      return -EIO;
    }

  rd0 = getreg32(base + RM57_SCI_LINRD0_OFFSET);
  rd1 = (len > 4) ? getreg32(base + RM57_SCI_LINRD1_OFFSET) : 0;

  if (len > 0)
    {
      data[0] = (uint8_t)(rd0 >> SCI_LINRD_BYTE0_SHIFT);
    }

  if (len > 1)
    {
      data[1] = (uint8_t)(rd0 >> SCI_LINRD_BYTE1_SHIFT);
    }

  if (len > 2)
    {
      data[2] = (uint8_t)(rd0 >> SCI_LINRD_BYTE2_SHIFT);
    }

  if (len > 3)
    {
      data[3] = (uint8_t)(rd0 >> SCI_LINRD_BYTE3_SHIFT);
    }

  if (len > 4)
    {
      data[4] = (uint8_t)(rd1 >> SCI_LINRD_BYTE0_SHIFT);
    }

  if (len > 5)
    {
      data[5] = (uint8_t)(rd1 >> SCI_LINRD_BYTE1_SHIFT);
    }

  if (len > 6)
    {
      data[6] = (uint8_t)(rd1 >> SCI_LINRD_BYTE2_SHIFT);
    }

  if (len > 7)
    {
      data[7] = (uint8_t)(rd1 >> SCI_LINRD_BYTE3_SHIFT);
    }

  return OK;
}
