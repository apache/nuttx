/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_oscconfig.c
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
#include <assert.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "nrf54l_oscconfig.h"
#include "hardware/nrf54l_osc.h"
#include "hardware/nrf54l_ficr.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_hfxo_intcap
 *
 * Description:
 *   Configure HFXO load capacitance using the signed factory trim.
 *
 ****************************************************************************/

static void nrf54l_hfxo_intcap(void)
{
  int32_t  slope;
  int32_t  offset;
  int32_t  capvalue;
  uint32_t trim;

  trim = getreg32(NRF54L_FICR_XOSC32MTRIM);
  if (trim == UINT32_MAX)
    {
      return;
    }

  slope = (int32_t)((trim & 0x1ff) ^ 0x100) - 0x100;
  offset = (trim >> 16) & 0x3ff;

  /* Board capacitance is expressed in femtofarads to avoid floating point.
   */

  capvalue = ((BOARD_HFXO_CAPACITANCE - 5500) * (slope + 791) +
              4000 * offset + 128000) / 256000;
  if (capvalue < 0 || capvalue > 63)
    {
      PANIC();
    }

  putreg32(capvalue, NRF54L_OSC_XOSC32M_INTCAP);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_oscconfig
 ****************************************************************************/

void nrf54l_oscconfig(void)
{
  /* Configure internal capacitors for HFXO */

  nrf54l_hfxo_intcap();
}
