/****************************************************************************
 * arch/arm/src/nrf52/nrf52832_errdata.c
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
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/nrf52_utils.h"
#include "hardware/nrf52_rng.h"
#include "hardware/nrf52_ficr.h"
#include "hardware/nrf52_temp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool errata_16(void)
{
  if (((getreg32(0xf0000fe0) & 0xff) == 0x6) &&
      ((getreg32(0xf0000fe4) & 0x0f) == 0x0))
    {
      if ((getreg32(0xf0000fe8) & 0xf0) == 0x30)
        {
          return true;
        }
    }

  return false;
}

static bool errata_66(void)
{
  /* This piece of code is modified from Nordic SDK , there is no document to
   * describe how to get the errdata information
   */

  if (((getreg32(0xf0000fe0) & 0xff) == 0x6) &&
      ((getreg32(0xf0000fe4) & 0x0f) == 0x0))
    {
      if ((getreg32(0xf0000fe8) & 0xf0) == 0x50)
        {
          return true;
        }
    }

  return false;
}

static void nrf52832_errdata_16(void)
{
  if (errata_16())
    {
      *(volatile uint32_t *)0x4007c074 = 3131961357ul;
    }
}

static void nrf52832_errdata_102(void)
{
  uint32_t regval = getreg32(NRF52_FICR_INFO_VARIANT);

  /* Also addresses erratas 106, 146. Revision 1 chips are affected */

  if (regval == NRF52_FICR_INFO_VARIANT_AAB0 ||
      regval == NRF52_FICR_INFO_VARIANT_ABB0)
    {
      *(volatile uint32_t *) 0x40001774 =
          ((*(volatile uint32_t *) 0x40001774) & 0xfffffffe) | 0x01000000;
    }
}

static void nrf52832_errdata_66_temp(void)
{
  /* Workaround for Errata 66 "TEMP: Linearity specification not met with
   * default settings found at the Errata document for your device located
   * at https://infocenter.nordicsemi.com/
   */

  if (errata_66())
    {
      int i;

      /* slot A : 6 totals */

      for (i = 0; i < 6; i++)
        {
          putreg32(getreg32(NRF52_FICR_TEMP_A0 + i * 4),
                   NRF52_TEMP_A0 + i * 4);
        }

      /* Slot B : 6 totals */

      for (i = 0 ; i < 6; i++)
        {
          putreg32(getreg32(NRF52_FICR_TEMP_B0 + i * 4),
                   NRF52_TEMP_B0 + i * 4);
        }

      /* slot C : 5 totals */

      for (i = 0; i < 5; i++)
        {
          putreg32(getreg32(NRF52_FICR_TEMP_T0 + i * 4),
                   NRF52_TEMP_T0 + i * 4);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nrf52832_errdata_init(void)
{
  nrf52832_errdata_16();

  nrf52832_errdata_66_temp();

  nrf52832_errdata_102();
}
