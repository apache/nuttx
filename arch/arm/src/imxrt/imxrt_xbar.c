/****************************************************************************
 * arch/arm/src/imxrt/imxrt_xbar.c
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

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include "chip.h"
#include "arm_internal.h"
#include "imxrt_xbar.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_xbars_masks[] = IMXRT_XBAR_SEL_MASKS;

static const uintptr_t g_xbars_addresses[] =
{
  IMXRT_XBAR1_BASE,
  IMXRT_XBAR2_BASE,
#if (defined(IMXRT_XBAR3_BASE))
  IMXRT_XBAR3_BASE
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_xbar_connect
 *
 * Description:
 *   This function maps the input_index of the cross bar to the output.
 *
 * input_index Parameters:
 *   mux_index_out   - XBAR Output and mux_select choice.
 *   mux_index_input - XBAR Input and input_index choice.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int imxrt_xbar_connect(uint16_t mux_index_out, uint16_t mux_index_input)
{
  uintptr_t address;
  uint16_t mux_select;
  uint16_t mux_input;
  uint16_t xbar_index;
  uint16_t clearbits;
  int retval;

  retval     = -EINVAL;
  mux_select = IMXRT_SEL(mux_index_out);
  mux_input  = IMXRT_SEL(mux_index_input);
  xbar_index = IMXRT_XBAR(mux_index_out);
  clearbits  = g_xbars_masks[xbar_index];

  switch (xbar_index)
    {
      case 0:
        imxrt_clockall_xbar1();
        break;
      case 1:
        imxrt_clockall_xbar2();
        break;
      case 2:
#if defined(IMXRT_XBAR3_BASE)
        imxrt_clockall_xbar3();
        break;
#endif
      default:
        break;
    }

  /* Verify:
   * 1) The Xbar index is valid.
   * 2) In and out are on the same Xbar.
   * 3) Output index is an output.
   * 4) Input index is input.
   */

  if (xbar_index < sizeof(g_xbars_addresses) /
      sizeof(g_xbars_addresses[0]) &&
      (mux_index_out & XBAR_OUTPUT) == XBAR_OUTPUT &&
      (mux_index_input & XBAR_INPUT) == XBAR_INPUT)
    {
      address = g_xbars_addresses[xbar_index];
      address += (mux_select / IMXRT_SEL_PER_REG) * sizeof(uint16_t);

      /* There are 2 selects per Register LSB is even selects and
       * MSB is odd
       */

      if (mux_select & 1)
        {
          clearbits <<= IMXRT_SEL1_SHIFTS;
          mux_input <<= IMXRT_SEL1_SHIFTS;
        }

      modifyreg16(address, clearbits, mux_input);
      retval = OK;
    }

  return retval;
}
