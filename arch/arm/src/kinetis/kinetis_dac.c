/****************************************************************************
 * arch/arm/src/kinetis/kinetis_dac.c
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
 * 12-bit Digital-to-Analog Converter (DAC)
 *
 * From the Reference Manual:
 *
 *  The 12-bit digital-to-analog converter (DAC) is a low-power,
 *  general-purpose DAC. The output of the DAC can be placed on an external
 *  pin or set as one of the inputs to the analog comparator, op-amps,
 *  or ADC.
 *
 * This file provides DAC driver for the Kinetis' 12-bit DAC.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <nuttx/analog/dac.h>

#include "arm_internal.h"             /* putreg... */
#include "hardware/kinetis_dac.h"     /* KINETIS_DAC... */
#include "hardware/kinetis_sim.h"     /* SIM_... */
#include "hardware/kinetis_vrefv1.h"  /* VREF_... */

#if defined(CONFIG_KINETIS_DAC0) || defined(CONFIG_KINETIS_DAC1)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void dac_reset(struct dac_dev_s *dev);
static int  dac_setup(struct dac_dev_s *dev);
static void dac_shutdown(struct dac_dev_s *dev);
static void dac_txint(struct dac_dev_s *dev, bool enable);
static int  dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg);
static int  dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dac_ops_s const g_dacops =
{
  .ao_reset    = dac_reset,
  .ao_setup    = dac_setup,
  .ao_shutdown = dac_shutdown,
  .ao_txint    = dac_txint,
  .ao_send     = dac_send,
  .ao_ioctl    = dac_ioctl,
};

#if defined(CONFIG_KINETIS_DAC0)
static struct dac_dev_s g_dac0_dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = NULL,
};
#endif

#if defined(CONFIG_KINETIS_DAC1)
static struct dac_dev_s g_dac1_dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = NULL,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dac_reset(struct dac_dev_s *dev)
{
  uint8_t  reg;
  uint32_t addr;

  dac_shutdown(dev);
  dac_setup(dev);

#if defined(CONFIG_KINETIS_DAC0)
  if (&g_dac0_dev == dev)
    {
      reg  = 0x00;
      addr = KINETIS_DAC0_DAT0L;
      while (addr < KINETIS_DAC0_SR)
        {
          putreg8(reg, addr);
          addr++;
        }

      putreg8(0x02, KINETIS_DAC0_SR);
      putreg8(0x00, KINETIS_DAC0_C0);
      putreg8(0x00, KINETIS_DAC0_C1);
      putreg8(0x0f, KINETIS_DAC0_C2);
    }
#endif /* defined(CONFIG_KINETIS_DAC0) */

#if defined(CONFIG_KINETIS_DAC1)
  if (&g_dac1_dev == dev)
    {
      reg  = 0x00;
      addr = KINETIS_DAC1_DAT0L;
      while (addr < KINETIS_DAC1_SR)
        {
          putreg8(reg, addr);
          addr++;
        }

      putreg8(0x02, KINETIS_DAC1_SR);
      putreg8(0x00, KINETIS_DAC1_C0);
      putreg8(0x00, KINETIS_DAC1_C1);
      putreg8(0x0f, KINETIS_DAC1_C2);
    }
#endif /* defined(CONFIG_KINETIS_DAC1) */
}

static int dac_setup(struct dac_dev_s *dev)
{
  uint32_t reg32;
  uint8_t  reg;

#if defined(CONFIG_KINETIS_DAC0)
  if (&g_dac0_dev == dev)
    {
      /* Enable VREF (for DAC0) */

      reg32 = getreg32(KINETIS_SIM_SCGC4);
      reg32 |= SIM_SCGC4_VREF;
      putreg32(reg32, KINETIS_SIM_SCGC4);

      /* Configure VREF (for DAC0) */

      reg = VREF_SC_VREFEN | VREF_SC_REGEN | VREF_SC_MODE_LV_LOWPWR;
      putreg8(reg, KINETIS_VREF_SC);

      /* Enable DAC0 */

      reg32 = getreg32(KINETIS_SIM_SCGC2);
      reg32 |= SIM_SCGC2_DAC0;
      putreg32(reg32, KINETIS_SIM_SCGC2);

      putreg8(DAC_C0_DACEN, KINETIS_DAC0_C0);
    }
#endif /* defined(CONFIG_KINETIS_DAC0) */

#if defined(CONFIG_KINETIS_DAC1)
  if (&g_dac1_dev == dev)
    {
      /* Enable VREF (for DAC1) */

      reg32 = getreg32(KINETIS_SIM_SCGC4);
      reg32 |= SIM_SCGC4_VREF;
      putreg32(reg32, KINETIS_SIM_SCGC4);

      /* Configure VREF (for DAC1) */

      reg = VREF_SC_VREFEN | VREF_SC_REGEN | VREF_SC_MODE_LV_LOWPWR;
      putreg8(reg, KINETIS_VREF_SC);

      /* Enable DAC1 */

      reg32 = getreg32(KINETIS_SIM_SCGC2);
      reg32 |= SIM_SCGC2_DAC1;
      putreg32(reg32, KINETIS_SIM_SCGC2);

      putreg8(DAC_C0_DACEN, KINETIS_DAC1_C0);
    }
#endif /* defined(CONFIG_KINETIS_DAC1) */

  return OK;
}

static void dac_shutdown(struct dac_dev_s *dev)
{
  uint32_t reg32;

#if defined(CONFIG_KINETIS_DAC0)
  if (&g_dac0_dev == dev)
    {
      /* Is DAC0 module enabled? */

      reg32 = getreg32(KINETIS_SIM_SCGC2);
      if (reg32 & SIM_SCGC2_DAC0)
        {
          /* Disable DAC0 */

          putreg8(DAC_C0_DACEN, KINETIS_DAC0_C0);
        }

      /* Disable DAC0 module */

      reg32 &= ~SIM_SCGC2_DAC0;
      putreg32(reg32, KINETIS_SIM_SCGC2);

      /* Do not disable VREF (for DAC0) because someone else may use it. */
    }
#endif /* defined(CONFIG_KINETIS_DAC0) */

#if defined(CONFIG_KINETIS_DAC1)
  if (&g_dac1_dev == dev)
    {
      /* Is DAC1 module enabled? */

      reg32 = getreg32(KINETIS_SIM_SCGC2);
      if (reg32 & SIM_SCGC2_DAC1)
        {
          /* Disable DAC1 */

          putreg8(DAC_C0_DACEN, KINETIS_DAC1_C0);
        }

      /* Disable DAC1 module */

      reg32 &= ~SIM_SCGC2_DAC1;
      putreg32(reg32, KINETIS_SIM_SCGC2);

      /* Do not disable VREF (for DAC1) because someone else may use it. */
    }
#endif /* defined(CONFIG_KINETIS_DAC1) */
}

static void dac_txint(struct dac_dev_s *dev, bool enable)
{
  UNUSED(dev);
  UNUSED(enable);

  awarn("Interrupts not implemented for Kinetis' DAC");
}

static int dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg)
{
#if defined(CONFIG_KINETIS_DAC0)
  if (&g_dac0_dev == dev)
    {
      putreg8((uint8_t)((msg->am_data & 0x00ff) >> 0), KINETIS_DAC0_DAT0L);
      putreg8((uint8_t)((msg->am_data & 0xff00) >> 8), KINETIS_DAC0_DAT0H);
    }
#endif /* defined(CONFIG_KINETIS_DAC0) */

#if defined(CONFIG_KINETIS_DAC1)
  if (&g_dac1_dev == dev)
    {
      putreg8((uint8_t)((msg->am_data & 0x00ff) >> 0), KINETIS_DAC1_DAT0L);
      putreg8((uint8_t)((msg->am_data & 0xff00) >> 8), KINETIS_DAC1_DAT0H);
    }
#endif /* defined(CONFIG_KINETIS_DAC1) */

  dac_txdone(dev);
  return OK;
}

static int dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  UNUSED(dev);
  UNUSED(cmd);
  UNUSED(arg);
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct dac_dev_s *kinetis_dac_initialize(int which)
{
  switch (which)
    {
      case 0:
#if defined(CONFIG_KINETIS_DAC0)
        return &g_dac0_dev;
#else
        awarn("CONFIG_KINETIS_DAC0 must be defined");
        return NULL;
#endif
        break;
      case 1:
#if defined(CONFIG_KINETIS_DAC1)
        return &g_dac1_dev;
#else
        awarn("CONFIG_KINETIS_DAC1 must be defined");
        return NULL;
#endif
        break;
      default:
        awarn("Bad device specified, must be 0 <= %d <= 1", which);
    }

  return NULL;
}

#endif /* defined(CONFIG_KINETIS_DAC0) || defined(CONFIG_KINETIS_DAC1) */
