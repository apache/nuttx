/****************************************************************************
 * arch/rh850/src/u2c/u2c_lowputc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "rh850_internal.h"
#include "hardware/u2c_rlin3.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RH850_RLIN311
#  define CONSOLE_BASE  U2C_RLIN311_BASE
#  define CONSOLE_BAUD  CONFIG_UART0_BAUD
#endif

/* LBRP01 = round(clock / (16 x baud)) - 1 with LWBR = 0.  A rate whose
 * divider is out of range, or that is more than RLIN3_BAUD_TOL percent off,
 * is rejected.
 */

#define RLIN3_CLK       CONFIG_RH850_CLK_HSB_FREQUENCY
#define RLIN3_BAUD_TOL  3
#define RLIN3_DIV(b)    ((RLIN3_CLK + RLIN3_SAMPLES * (b) / 2) / \
                         (RLIN3_SAMPLES * (b)))
#define RLIN3_REAL(b)   (RLIN3_SAMPLES * (b) * RLIN3_DIV(b))
#define RLIN3_DIFF(b)   (RLIN3_REAL(b) > RLIN3_CLK ? \
                         RLIN3_REAL(b) - RLIN3_CLK : \
                         RLIN3_CLK - RLIN3_REAL(b))
#define RLIN3_BAUD_OK(b) \
  ((b) > 0 && RLIN3_SAMPLES * (b) <= RLIN3_CLK && \
   RLIN3_DIV(b) <= 0x10000 && \
   RLIN3_DIFF(b) * 100 <= RLIN3_REAL(b) * RLIN3_BAUD_TOL)

#if defined(CONSOLE_BAUD) && !RLIN3_BAUD_OK(CONSOLE_BAUD)
#  error CONFIG_UART0_BAUD does not fit CONFIG_RH850_CLK_HSB_FREQUENCY
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: u2c_rlin3_setup
 *
 * Description:
 *   RLIN3 UART mode, 8N1, LSB first, no error detection.  The unit clock
 *   and module stop are set up by rh850_clockconfig().  Returns -EINVAL
 *   for a rate that RLIN3_BAUD_OK() rejects.
 *
 ****************************************************************************/

int u2c_rlin3_setup(uintptr_t base, uint32_t baud)
{
  uint64_t b = baud;

  if (!RLIN3_BAUD_OK(b))
    {
      return -EINVAL;
    }

  putreg8(0, base + RLIN3_LCUC_OFFSET);
  while ((getreg8(base + RLIN3_LMST_OFFSET) & RLIN3_LMST_OMM0) != 0)
    {
    }

  putreg8(0, base + RLIN3_LWBR_OFFSET);
  putreg16((uint16_t)(RLIN3_DIV(b) - 1), base + RLIN3_LBRP01_OFFSET);
  putreg8(RLIN3_LMD_UART, base + RLIN3_LMD_OFFSET);
  putreg8(0, base + RLIN3_LBFC_OFFSET);
  putreg8(0, base + RLIN3_LEDE_OFFSET);
  putreg8(0, base + RLIN3_LUOR1_OFFSET);

  putreg8(RLIN3_LCUC_OM0, base + RLIN3_LCUC_OFFSET);
  while ((getreg8(base + RLIN3_LMST_OFFSET) & RLIN3_LMST_OMM0) == 0)
    {
    }

  putreg8(RLIN3_LUOER_UTOE | RLIN3_LUOER_UROE, base + RLIN3_LUOER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: u2c_rlin3_putc
 *
 * Description:
 *   Send one character and wait for the end of the transmission.
 *
 ****************************************************************************/

void u2c_rlin3_putc(uintptr_t base, int ch)
{
  putreg8((uint8_t)ch, base + RLIN3_LUTDR_OFFSET);
  while ((getreg8(base + RLIN3_LST_OFFSET) & RLIN3_LST_UTS) != 0)
    {
    }
}

/****************************************************************************
 * Name: rh850_lowputc
 ****************************************************************************/

void rh850_lowputc(int ch)
{
#ifdef CONSOLE_BASE
  u2c_rlin3_putc(CONSOLE_BASE, ch);
#endif
}

/****************************************************************************
 * Name: rh850_lowsetup
 *
 * Description:
 *   Console setup, after rh850_clockconfig().
 *
 ****************************************************************************/

void rh850_lowsetup(void)
{
#ifdef CONSOLE_BASE
  u2c_rlin3_setup(CONSOLE_BASE, CONSOLE_BAUD);
#endif
}

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

void up_putc(int ch)
{
  irqstate_t flags = up_irq_save();

  rh850_lowputc(ch);
  up_irq_restore(flags);
}
