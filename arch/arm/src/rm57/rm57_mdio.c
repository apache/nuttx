/****************************************************************************
 * arch/arm/src/rm57/rm57_mdio.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/rm57_ethernet.h"
#include "rm57_mdio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USERACCESS0 GO-bit poll bound.  A single MDIO transaction (with
 * preamble) takes on the order of tens of bit times at the configured
 * MDIO clock; this bound is generous even at the slowest permitted MDIO
 * clock and guards against a stuck bus (e.g. no PHY present, SWD-halted
 * peer) hanging the caller forever.
 */

#define RM57_MDIO_GO_TIMEOUT   100000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_rm57mdio_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_mdio_wait_go
 *
 * Description:
 *   Poll USERACCESS0.GO until the hardware clears it (transaction done)
 *   or the timeout bound is hit.  Caller holds g_rm57mdio_lock.
 *
 * Returned Value:
 *   The final USERACCESS0 register value (so callers can inspect DATA/
 *   ACK without an extra read), or 0xffffffff on timeout - a value that
 *   can never occur naturally since bit 30 (WRITE) and bit 31 (GO) can't
 *   both be genuinely set with ACK also set on a real read/write.
 *
 ****************************************************************************/

static uint32_t rm57_mdio_wait_go(void)
{
  uint32_t regval;
  int timeout = RM57_MDIO_GO_TIMEOUT;

  do
    {
      regval = getreg32(RM57_MDIO_USERACCESS0);
    }
  while ((regval & MDIO_USERACCESS_GO) != 0 && --timeout > 0);

  if (timeout <= 0)
    {
      return 0xffffffff;
    }

  return regval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_mdio_initialize
 ****************************************************************************/

int rm57_mdio_initialize(uint32_t vclk3_freq, uint32_t mdio_freq)
{
  uint32_t clkdiv;

  if (mdio_freq == 0 || mdio_freq > vclk3_freq)
    {
      return -EINVAL;
    }

  clkdiv = vclk3_freq / mdio_freq;
  if (clkdiv > 0)
    {
      clkdiv--;
    }

  nxmutex_lock(&g_rm57mdio_lock);

  putreg32(MDIO_CONTROL_CLKDIV(clkdiv) | MDIO_CONTROL_ENABLE |
           MDIO_CONTROL_FAULTENB,
           RM57_MDIO_CONTROL);

  nxmutex_unlock(&g_rm57mdio_lock);
  return OK;
}

/****************************************************************************
 * Name: rm57_mdio_shutdown
 ****************************************************************************/

void rm57_mdio_shutdown(void)
{
  nxmutex_lock(&g_rm57mdio_lock);
  putreg32(0, RM57_MDIO_CONTROL);
  nxmutex_unlock(&g_rm57mdio_lock);
}

/****************************************************************************
 * Name: rm57_mdio_read
 ****************************************************************************/

int rm57_mdio_read(uint8_t phyaddr, uint8_t regaddr, uint16_t *value)
{
  uint32_t regval;

  nxmutex_lock(&g_rm57mdio_lock);

  putreg32(MDIO_USERACCESS_GO | MDIO_USERACCESS_READ |
           MDIO_USERACCESS_PHYADR(phyaddr) |
           MDIO_USERACCESS_REGADR(regaddr),
           RM57_MDIO_USERACCESS0);

  regval = rm57_mdio_wait_go();

  nxmutex_unlock(&g_rm57mdio_lock);

  if (regval == 0xffffffff)
    {
      return -ETIMEDOUT;
    }

  if ((regval & MDIO_USERACCESS_ACK) == 0)
    {
      return -ENXIO;
    }

  *value = (uint16_t)(regval & MDIO_USERACCESS_DATA_MASK);
  return OK;
}

/****************************************************************************
 * Name: rm57_mdio_write
 ****************************************************************************/

int rm57_mdio_write(uint8_t phyaddr, uint8_t regaddr, uint16_t value)
{
  uint32_t regval;

  nxmutex_lock(&g_rm57mdio_lock);

  putreg32(MDIO_USERACCESS_GO | MDIO_USERACCESS_WRITE |
           MDIO_USERACCESS_PHYADR(phyaddr) |
           MDIO_USERACCESS_REGADR(regaddr) |
           (value & MDIO_USERACCESS_DATA_MASK),
           RM57_MDIO_USERACCESS0);

  regval = rm57_mdio_wait_go();

  nxmutex_unlock(&g_rm57mdio_lock);

  if (regval == 0xffffffff)
    {
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: rm57_mdio_alive
 ****************************************************************************/

uint32_t rm57_mdio_alive(void)
{
  return getreg32(RM57_MDIO_ALIVE);
}

/****************************************************************************
 * Name: rm57_mdio_linkstatus
 ****************************************************************************/

bool rm57_mdio_linkstatus(uint8_t phyaddr)
{
  return (getreg32(RM57_MDIO_LINK) & MDIO_LINK_PHY(phyaddr)) != 0;
}

/****************************************************************************
 * Name: rm57_mdio_linkint
 ****************************************************************************/

void rm57_mdio_linkint(uint8_t phyaddr, bool enable)
{
  uint32_t regval = MDIO_USERPHYSEL_PHYADRMON(phyaddr);

  if (enable)
    {
      regval |= MDIO_USERPHYSEL_LINKINTENB;
    }

  nxmutex_lock(&g_rm57mdio_lock);
  putreg32(regval, RM57_MDIO_USERPHYSEL0);
  nxmutex_unlock(&g_rm57mdio_lock);
}
