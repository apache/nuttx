/****************************************************************************
 * arch/arm/src/imxrt/imxrt_xbar.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: David Sidrane <david_s5@nscdg.com>
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include "chip.h"
#include "arm_arch.h"
#include "imxrt_xbar.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uintptr_t g_xbars_addresses[] =
{
  IMXRT_XBAR1_BASE,
  IMXRT_XBAR2_BASE,
#if (defined(CONFIG_ARCH_FAMILY_IMXRT105x) || defined (CONFIG_ARCH_FAMILY_IMXRT106x))
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
  clearbits  = IMXRT_SEL0_MASK;

  /* Verify:
   * 1) The Xbar index is valid.
   * 2) In and out are on the same Xbar.
   * 3) Output index is an output.
   * 4) Input index is input.
   */

  if (xbar_index < sizeof(g_xbars_addresses) / sizeof(g_xbars_addresses[0]) &&
      (mux_index_out & XBAR_OUTPUT) == XBAR_OUTPUT &&
      (mux_index_input & XBAR_INPUT) == XBAR_INPUT)
    {
      address = g_xbars_addresses[xbar_index];
      address += (mux_select / IMXRT_SEL_PER_REG) * sizeof(uint16_t);

      /* There are 2 selects per Register LSB is even selects and MSB is odd */

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
