/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_xosc.c
 *
 * Based upon the software originally developed by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "arm_internal.h"

#include "rp23xx_xosc.h"
#include "hardware/address_mapped.h"
#include "hardware/structs/xosc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_xosc_init
 *
 * Description:
 *   Initialize Crystal Oscillator (XOSC).
 *
 ****************************************************************************/

void rp23xx_xosc_init(void)
{
  /* Assumes 1-15 MHz input */

  ASSERT(BOARD_XOSC_FREQ <= (15 * MHZ));
  putreg32(XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ<<XOSC_CTRL_FREQ_RANGE_LSB, &xosc_hw->ctrl);

  /* Set xosc startup delay */

  uint32_t startup_delay = (((12 * MHZ) / 1000) + 128) / 256;
  putreg32(startup_delay, &xosc_hw->startup);

  /* Set the enable bit now that we have set freq range and startup delay */

  hw_set_bits(&xosc_hw->ctrl, XOSC_CTRL_ENABLE_VALUE_ENABLE<<XOSC_CTRL_ENABLE_LSB);

  /* Wait for XOSC to be stable */

  while (!(getreg32(&xosc_hw->status) & XOSC_STATUS_STABLE_BITS))
    ;
}
