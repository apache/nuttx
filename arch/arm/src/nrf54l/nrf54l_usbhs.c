/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_usbhs.c
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

#include "arm_internal.h"
#include "nrf54l_usbhs.h"
#include "hardware/nrf54l_clock.h"
#include "hardware/nrf54l_usbhs.h"
#include "hardware/nrf54l_vregusb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nordic USBHS Type 2 PHY defaults for nRF54LM20A/B. */

#define NRF54L_USBHS_PHY_CONFIG_VALUE \
  ((12 << USBHS_PHY_CONFIG_PLLPTUNE_SHIFT) | \
   (3 << USBHS_PHY_CONFIG_COMPDISTUNE_SHIFT) | \
   (3 << USBHS_PHY_CONFIG_SQRXTUNE_SHIFT) | \
   (1 << USBHS_PHY_CONFIG_VDATREFTUNE_SHIFT) | \
   (3 << USBHS_PHY_CONFIG_TXHSXVTUNE_SHIFT) | \
   (3 << USBHS_PHY_CONFIG_TXFSLSTUNE_SHIFT) | \
   (3 << USBHS_PHY_CONFIG_TXVREFTUNE_SHIFT) | \
   (1 << USBHS_PHY_CONFIG_TXRISETUNE_SHIFT) | \
   (2 << USBHS_PHY_CONFIG_TXRESTUNE_SHIFT))

#define NRF54L_USBHS_CLOCK_TIMEOUT 5000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static xcpt_t g_vbus_handler;
static void *g_vbus_arg;
static volatile uint32_t g_vbus_generation;
static bool g_initialized;
static bool g_enabled;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_usbhs_interrupt
 *
 * Description:
 *   Record every VBUS transition, including removal followed by insertion
 *   while the PHY is starting. The device driver handles the new bus state.
 *
 ****************************************************************************/

static int nrf54l_usbhs_interrupt(int irq, void *context, void *arg)
{
  uint32_t detected = getreg32(NRF54L_VREGUSB_EVENTS_VBUSDETECTED);
  uint32_t removed = getreg32(NRF54L_VREGUSB_EVENTS_VBUSREMOVED);

  if (detected == 0 && removed == 0)
    {
      return OK;
    }

  if (detected != 0)
    {
      putreg32(0, NRF54L_VREGUSB_EVENTS_VBUSDETECTED);
      getreg32(NRF54L_VREGUSB_EVENTS_VBUSDETECTED);
    }

  if (removed != 0)
    {
      putreg32(0, NRF54L_VREGUSB_EVENTS_VBUSREMOVED);
      getreg32(NRF54L_VREGUSB_EVENTS_VBUSREMOVED);
    }

  g_vbus_generation++;
  return g_vbus_handler(irq, context, g_vbus_arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_usbhs_initialize
 *
 * Description:
 *   Reset the PHY and start VBUS detection without accessing the DWC2 core.
 *
 ****************************************************************************/

int nrf54l_usbhs_initialize(xcpt_t handler, void *arg)
{
  int ret;

  if (handler == NULL)
    {
      return -EINVAL;
    }

  if (g_initialized)
    {
      return -EBUSY;
    }

  up_disable_irq(NRF54L_IRQ_VREGUSB);
  ret = irq_attach(NRF54L_IRQ_VREGUSB, nrf54l_usbhs_interrupt, NULL);
  if (ret < 0)
    {
      return ret;
    }

  g_vbus_handler = handler;
  g_vbus_arg = arg;
  g_vbus_generation = 0;
  g_initialized = true;
  nrf54l_usbhs_disable();
  putreg32(VREGUSB_INT_VBUSDETECTED | VREGUSB_INT_VBUSREMOVED,
           NRF54L_VREGUSB_INTENCLR);
  putreg32(0, NRF54L_VREGUSB_EVENTS_VBUSDETECTED);
  putreg32(0, NRF54L_VREGUSB_EVENTS_VBUSREMOVED);
  putreg32(VREGUSB_INT_VBUSDETECTED | VREGUSB_INT_VBUSREMOVED,
           NRF54L_VREGUSB_INTENSET);
  putreg32(VREGUSB_TASKS_START, NRF54L_VREGUSB_TASKS_START);
  up_enable_irq(NRF54L_IRQ_VREGUSB);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_usbhs_vbus
 *
 * Description:
 *   Sample VBUS, including after a warm reset with the cable connected.
 *
 ****************************************************************************/

bool nrf54l_usbhs_vbus(void)
{
  return (getreg32(NRF54L_VREGUSB_STATUS) & VREGUSB_STATUS_VBUSDET) != 0;
}

/****************************************************************************
 * Name: nrf54l_usbhs_enable
 *
 * Description:
 *   Start the 24 MHz clock and Type 2 PHY before releasing DWC2 reset.
 *   Keep the D+ pull-up off until the device driver configures the core.
 *
 ****************************************************************************/

int nrf54l_usbhs_enable(void)
{
  uint32_t generation;
  unsigned int timeout;
  irqstate_t flags;
  int ret = OK;

  if (!g_initialized)
    {
      return -ENODEV;
    }

  if (g_enabled)
    {
      return -EBUSY;
    }

  flags = enter_critical_section();
  generation = g_vbus_generation;
  if (!nrf54l_usbhs_vbus() ||
      getreg32(NRF54L_VREGUSB_EVENTS_VBUSREMOVED) != 0)
    {
      ret = -ENODEV;
    }

  leave_critical_section(flags);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(0, NRF54L_CLOCK_EVENTS_XO24MSTARTED);
  putreg32(1, NRF54L_CLOCK_TASKS_XO24MSTART);
  for (timeout = 0; timeout < NRF54L_USBHS_CLOCK_TIMEOUT; timeout++)
    {
      if (getreg32(NRF54L_CLOCK_EVENTS_XO24MSTARTED) != 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (timeout == NRF54L_USBHS_CLOCK_TIMEOUT)
    {
      putreg32(1, NRF54L_CLOCK_TASKS_XO24MSTOP);
      return -ETIMEDOUT;
    }

  putreg32(USBHS_ENABLE_CORE, NRF54L_USBHS_ENABLE);
  putreg32(NRF54L_USBHS_PHY_CONFIG_VALUE, NRF54L_USBHS_PHY_CONFIG);
  putreg32(USBHS_PHY_OVERRIDEVALUES_ID, NRF54L_USBHS_PHY_OVERRIDEVALUES);
  putreg32(USBHS_PHY_INPUTOVERRIDE_ID | USBHS_PHY_INPUTOVERRIDE_VBUSVALID |
           USBHS_PHY_INPUTOVERRIDE_SUSPENDM, NRF54L_USBHS_PHY_INPUTOVERRIDE);
  putreg32(USBHS_ENABLE_CORE | USBHS_ENABLE_PHY, NRF54L_USBHS_ENABLE);
  putreg32(USBHS_PHY_INPUTOVERRIDE_ID | USBHS_PHY_INPUTOVERRIDE_VBUSVALID,
           NRF54L_USBHS_PHY_INPUTOVERRIDE);
  up_udelay(45);
  putreg32(USBHS_TASKS_START, NRF54L_USBHS_TASKS_START);

  /* Reading a DWC2 register before this delay can hang the CPU. Interrupts
   * remain enabled so VBUS changes during startup are recorded.
   */

  up_mdelay(1);
  flags = enter_critical_section();
  if (generation != g_vbus_generation || !nrf54l_usbhs_vbus() ||
      getreg32(NRF54L_VREGUSB_EVENTS_VBUSREMOVED) != 0)
    {
      ret = -EAGAIN;
    }
  else
    {
      g_enabled = true;
    }

  leave_critical_section(flags);
  if (ret < 0)
    {
      nrf54l_usbhs_disable();
    }

  return ret;
}

/****************************************************************************
 * Name: nrf54l_usbhs_pullup
 *
 * Description:
 *   Allow the PHY to observe VBUS after the core is ready to enumerate.
 *
 ****************************************************************************/

int nrf54l_usbhs_pullup(bool enable)
{
  uint32_t regval = USBHS_PHY_INPUTOVERRIDE_ID;

  if (!g_enabled || (enable && !nrf54l_usbhs_vbus()))
    {
      return -ENODEV;
    }

  if (!enable)
    {
      regval |= USBHS_PHY_INPUTOVERRIDE_VBUSVALID;
    }

  putreg32(regval, NRF54L_USBHS_PHY_INPUTOVERRIDE);
  putreg32(USBHS_PHY_OVERRIDEVALUES_ID, NRF54L_USBHS_PHY_OVERRIDEVALUES);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_usbhs_disable
 *
 * Description:
 *   Disconnect the PHY, assert reset, then release the 24 MHz clock.
 *
 ****************************************************************************/

void nrf54l_usbhs_disable(void)
{
  putreg32(USBHS_PHY_INPUTOVERRIDE_ID | USBHS_PHY_INPUTOVERRIDE_VBUSVALID |
           USBHS_PHY_INPUTOVERRIDE_SUSPENDM, NRF54L_USBHS_PHY_INPUTOVERRIDE);
  putreg32(USBHS_PHY_OVERRIDEVALUES_ID, NRF54L_USBHS_PHY_OVERRIDEVALUES);
  putreg32(0, NRF54L_USBHS_ENABLE);
  up_udelay(10);
  putreg32(1, NRF54L_CLOCK_TASKS_XO24MSTOP);
  g_enabled = false;
}

/****************************************************************************
 * Name: nrf54l_usbhs_uninitialize
 *
 * Description:
 *   Stop VBUS detection and release the PHY and its clock.
 *
 ****************************************************************************/

void nrf54l_usbhs_uninitialize(void)
{
  if (!g_initialized)
    {
      return;
    }

  up_disable_irq(NRF54L_IRQ_VREGUSB);
  putreg32(VREGUSB_INT_VBUSDETECTED | VREGUSB_INT_VBUSREMOVED,
           NRF54L_VREGUSB_INTENCLR);
  irq_detach(NRF54L_IRQ_VREGUSB);
  nrf54l_usbhs_disable();
  putreg32(VREGUSB_TASKS_STOP, NRF54L_VREGUSB_TASKS_STOP);
  g_initialized = false;
  g_vbus_handler = NULL;
  g_vbus_arg = NULL;
}
