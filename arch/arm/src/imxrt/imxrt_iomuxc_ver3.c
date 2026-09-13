/****************************************************************************
 * arch/arm/src/imxrt/imxrt_iomuxc_ver3.c
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

#include "arm_internal.h"
#include "hardware/imxrt_memorymap.h"
#include "imxrt_iomuxc_ver3.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Descriptors for the two IOMUXC instances on RT1180.  These four numbers
 * per bank are all we need to derive MUX_CTL, PAD_CTL and DAISY register
 * addresses from a packed IMXRT_PADCFG value using pure arithmetic.
 *
 * ctl_first : offset of MUX_CTL[0] within the bank
 * pad_delta : PAD_CTL[i] = MUX_CTL[i] + pad_delta   (constant per bank)
 * dsy_first : offset of DAISY[0] within the bank
 */

static const struct imxrt_iomux_bank_s g_bank[IMXRT_PADCFG_NBANKS] =
{
  [IMXRT_PADCFG_BANK_MAIN] =
    {
      .base      = IMXRT_IOMUXC_WKUP_BASE,   /* 0x42a1_0000 */
      .ctl_first = 0x010u,
      .pad_delta = 0x248u,
      .dsy_first = 0x4a0u,
    },
  [IMXRT_PADCFG_BANK_AON] =
    {
      .base      = IMXRT_IOMUXC_AON_BASE,    /* 0x443c_0000 */
      .ctl_first = 0x000u,
      .pad_delta = 0x074u,
      .dsy_first = 0x0e8u,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_iomux_ctlreg
 *
 * Description:
 *   Compute the SW_MUX_CTL register address for a packed padcfg value.
 *
 ****************************************************************************/

static inline uintptr_t imxrt_iomux_ctlreg(uint32_t padcfg)
{
  const struct imxrt_iomux_bank_s *b = &g_bank[IMXRT_PADCFG_PADBANK(padcfg)];
  return (uintptr_t)b->base + b->ctl_first +
         IMXRT_PADCFG_PADIDX(padcfg) * 4u;
}

/****************************************************************************
 * Name: imxrt_iomux_padreg
 *
 * Description:
 *   Compute the SW_PAD_CTL register address for a packed padcfg value.
 *
 ****************************************************************************/

static inline uintptr_t imxrt_iomux_padreg(uint32_t padcfg)
{
  const struct imxrt_iomux_bank_s *b = &g_bank[IMXRT_PADCFG_PADBANK(padcfg)];
  return imxrt_iomux_ctlreg(padcfg) + b->pad_delta;
}

/****************************************************************************
 * Name: imxrt_iomux_dsyreg
 *
 * Description:
 *   Compute the DAISY (input) register address for a packed padcfg value,
 *   or 0 if the pad/alt has no input daisy.  Note that the daisy bank
 *   may differ from the pad bank (a pin on one IOMUXC can route to a
 *   peripheral whose input mux lives on the other IOMUXC).
 *
 ****************************************************************************/

static inline uintptr_t imxrt_iomux_dsyreg(uint32_t padcfg)
{
  uint32_t dsy_idx = IMXRT_PADCFG_DSYIDX(padcfg);
  const struct imxrt_iomux_bank_s *b;

  if (dsy_idx == IMXRT_PADCFG_NO_DSYIDX)
    {
      return 0;
    }

  b = &g_bank[IMXRT_PADCFG_DSYBANK(padcfg)];
  return (uintptr_t)b->base + b->dsy_first + dsy_idx * 4u;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_iomux_configure
 ****************************************************************************/

int imxrt_iomux_configure(imxrt_pinset_t pinset)
{
  uint32_t  padcfg   = IMXRT_PINSET_PADCFG(pinset);
  uintptr_t ctlreg   = imxrt_iomux_ctlreg(padcfg);
  uintptr_t padreg   = imxrt_iomux_padreg(padcfg);
  uintptr_t dsyreg   = imxrt_iomux_dsyreg(padcfg);
  uint32_t  mux      = IMXRT_PADCFG_MUX(padcfg);
  uint32_t  mux_side = IMXRT_PINSET_MUXSIDE(pinset);

  /* Write MUX_CTL: ALT | mux side-band bits (currently only SION) */

  putreg32(IOMUXC_MUX_MODE_ALT(mux) | mux_side, ctlreg);

  /* Write PAD_CTL */

  putreg32(IMXRT_PINSET_PADCTL(pinset), padreg);

  /* Route the peripheral input mux (daisy) if this alt uses one */

  if (dsyreg != 0)
    {
      putreg32(IMXRT_PADCFG_DSY(padcfg), dsyreg);
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_iomux_gpio
 *
 * Description:
 *   Force a pad to GPIO mode.  On RT1180 dedicated GPIO_IOxx pads on the
 *   main IOMUXC take ALT0 for GPIO; all other pads (including AON pads)
 *   use ALT5.
 *
 ****************************************************************************/

int imxrt_iomux_gpio(imxrt_pinset_t pinset, bool sion)
{
  uint32_t  padcfg   = IMXRT_PINSET_PADCFG(pinset);
  uintptr_t ctlreg   = imxrt_iomux_ctlreg(padcfg);
  uintptr_t padreg   = imxrt_iomux_padreg(padcfg);
  uint32_t  reg_sion = sion ? IOMUXC_MUX_SION_ON : 0;
  uint32_t  alt;

  /* On the main IOMUXC, the dedicated GPIO_IO<n> pads (there is no other
   * function on ALT0) select GPIO via ALT0.  Every other pad selects GPIO
   * via ALT5.  The IOMUXC_PAD_..._GPIOx_IOn macros generated from the SDK
   * already carry the correct ALT (0 or 5) in the padcfg field, so we
   * simply write the pre-encoded ALT and SION.
   */

  alt = IMXRT_PADCFG_MUX(padcfg);

  putreg32(IOMUXC_MUX_MODE_ALT(alt) | reg_sion, ctlreg);
  putreg32(IMXRT_PINSET_PADCTL(pinset), padreg);

  return OK;
}
