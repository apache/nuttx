/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_tsi.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include "arm_internal.h"
#include "chip.h"
#include "gd32w51x_tsi.h"
#include "hardware/gd32w51x_rcu.h"

#ifdef CONFIG_GD32W51X_TSI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pulse generator: CTCLK = HCLK / 64 (2.8 MHz at 180 MHz), 4 CTCLK of
 * charge and 8 CTCLK of transfer per cycle, at most 8191 cycles (about
 * 35 ms) per sequence.
 */

#define TSI_CTCDIV                6   /* HCLK / 2^6 */
#define TSI_CHARGE_CTCLK          4
#define TSI_TRANSFER_CTCLK        8
#define TSI_MCN                   5   /* 8191 cycles */

/* Time for the electrode and the sampling capacitor to discharge before a
 * sequence, and timeout of a sequence.
 */

#define TSI_DISCHARGE_US          1000
#define TSI_POLL_US               500
#define TSI_TIMEOUT_US            100000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_tsi_lock = NXMUTEX_INITIALIZER;
static uint32_t g_tsi_samplepins;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_tsi_initialize
 *
 * Description:
 *   Enable the touch sensing interface in software trigger mode.
 *
 ****************************************************************************/

int gd32_tsi_initialize(uint32_t samplepins)
{
  uint32_t groups = 0;
  int group;

  for (group = 0; group < GD32W51X_TSI_NGROUPS; group++)
    {
      if ((samplepins & (0xf << (group * 4))) != 0)
        {
          groups |= TSI_GCTL_GE(group);
        }
    }

  if (groups == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_tsi_lock);

  /* Enable and reset the peripheral */

  modifyreg32(GD32W51X_RCU_AHB1EN, 0, RCU_AHB1EN_TSIEN);
  modifyreg32(GD32W51X_RCU_AHB1RST, 0, RCU_AHB1RST_TSIRST);
  modifyreg32(GD32W51X_RCU_AHB1RST, RCU_AHB1RST_TSIRST, 0);

  /* Software trigger mode, pins driven low when idle so that the
   * capacitors discharge between the sequences.
   */

  putreg32(TSI_CTL0_CDT(TSI_CHARGE_CTCLK) |
           TSI_CTL0_CTDT(TSI_TRANSFER_CTCLK) |
           ((TSI_CTCDIV & 7) << TSI_CTL0_CTCDIV_SHIFT) |
           TSI_CTL0_MCN(TSI_MCN), GD32W51X_TSI_CTL0);
  putreg32((TSI_CTCDIV & 8) != 0 ? TSI_CTL1_CTCDIV : 0, GD32W51X_TSI_CTL1);

  /* No Schmitt trigger on the TSI pins, as recommended for analog use */

  modifyreg32(GD32W51X_TSI_PHM, samplepins, 0);

  putreg32(samplepins, GD32W51X_TSI_SAMPCFG);
  putreg32(0, GD32W51X_TSI_CHCFG);
  putreg32(groups, GD32W51X_TSI_GCTL);
  putreg32(0, GD32W51X_TSI_INTEN);

  modifyreg32(GD32W51X_TSI_CTL0, 0, TSI_CTL0_TSIEN);
  g_tsi_samplepins = samplepins;

  nxmutex_unlock(&g_tsi_lock);
  return OK;
}

/****************************************************************************
 * Name: gd32_tsi_acquire
 *
 * Description:
 *   Run one charge transfer sequence on an electrode and return its cycle
 *   count.
 *
 ****************************************************************************/

int gd32_tsi_acquire(int group, int pin, FAR uint16_t *count)
{
  uint32_t chpin;
  uint32_t intf = 0;
  int elapsed;
  int ret;

  DEBUGASSERT(group >= 0 && group < GD32W51X_TSI_NGROUPS &&
              pin >= 0 && pin < 4 && count != NULL);

  chpin = GD32W51X_TSI_PIN(group, pin);
  if (g_tsi_samplepins == 0 || (g_tsi_samplepins & chpin) != 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_tsi_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Select the electrode and let the capacitors discharge */

  modifyreg32(GD32W51X_TSI_PHM, chpin, 0);
  putreg32(chpin, GD32W51X_TSI_CHCFG);
  nxsig_usleep(TSI_DISCHARGE_US);

  /* Start the sequence and wait for its end */

  putreg32(TSI_INT_CTCF | TSI_INT_MNERR, GD32W51X_TSI_INTC);
  modifyreg32(GD32W51X_TSI_CTL0, 0, TSI_CTL0_TSIS);

  for (elapsed = 0; elapsed < TSI_TIMEOUT_US; elapsed += TSI_POLL_US)
    {
      intf = getreg32(GD32W51X_TSI_INTF);
      if ((intf & (TSI_INT_CTCF | TSI_INT_MNERR)) != 0)
        {
          break;
        }

      nxsig_usleep(TSI_POLL_US);
    }

  if ((intf & TSI_INT_MNERR) != 0)
    {
      ret = -ERANGE;
    }
  else if ((intf & TSI_INT_CTCF) == 0)
    {
      modifyreg32(GD32W51X_TSI_CTL0, TSI_CTL0_TSIS, 0);
      ret = -ETIMEDOUT;
    }
  else
    {
      *count = getreg32(GD32W51X_TSI_GCYCN(group)) & TSI_GCYCN_MASK;
      ret = OK;
    }

  putreg32(TSI_INT_CTCF | TSI_INT_MNERR, GD32W51X_TSI_INTC);
  putreg32(0, GD32W51X_TSI_CHCFG);

  nxmutex_unlock(&g_tsi_lock);
  return ret;
}

#endif /* CONFIG_GD32W51X_TSI */
