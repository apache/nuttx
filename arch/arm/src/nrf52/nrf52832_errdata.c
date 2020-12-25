/****************************************************************************
 * arch/arm/src/nrf52/nrf52832_errdata.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Levin Li <levin.li@outlook.com>
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

#include "arm_arch.h"
#include "chip.h"
#include "hardware/nrf52_utils.h"
#include "hardware/nrf52_rng.h"
#include "hardware/nrf52_ficr.h"
#include "hardware/nrf52_temp.h"
#include "arm_internal.h"

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
