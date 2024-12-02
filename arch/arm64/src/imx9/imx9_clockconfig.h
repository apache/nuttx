/****************************************************************************
 * arch/arm64/src/imx9/imx9_clockconfig.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_CLOCKCONFIG_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PLL_PARMS(_rdiv, _odiv, _mfi, _mfn, _mfd) \
  {                                               \
    .rdiv = (_rdiv),                              \
    .odiv = (_odiv),                              \
    .mfi  = (_mfi),                               \
    .mfn  = (_mfn),                               \
    .mfd  = (_mfd),                               \
  }

#define PLL_CFG(_reg, _frac, _parms) \
  {                                  \
    .reg   = (_reg),                 \
    .frac  = (_frac),                \
    .parms = _parms,                 \
  }

#define PFD_PARMS(_mfi, _mfn, _div2) \
  {                                  \
    .mfi       = (_mfi),             \
    .mfn       = (_mfn),             \
    .divby2_en = (_div2)             \
  }

#define PFD_CFG(_reg, _pfd, _parms) \
  {                                 \
    .reg   = (_reg),                \
    .pfd   = (_pfd),                \
    .parms = _parms,                \
  }

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pll_parms_s
{
  /* Integer part (DIV) */

  struct
  {
    uint32_t rdiv; /* Input clock divider */
    uint32_t odiv; /* PLL output divider */
    uint32_t mfi;  /* PLL integer divider */
  };

  /* Fractional part (NUMERATOR / DENOMINATOR) */

  struct
  {
    uint32_t mfn;  /* PLL fractional divider numerator */
    uint32_t mfd;  /* PLL fractional divider denominator */
  };
};

struct pfd_parms_s
{
  uint32_t mfi;       /* PLL integer divider */
  uint32_t mfn;       /* PLL fractional divider numerator */
  bool     divby2_en; /* Enable the divide-by-2 output */
};

struct imx9_pll_cfg_s
{
  uintptr_t          reg;   /* The PLL register base */
  bool               frac;  /* Fractional PLL ? */
  struct pll_parms_s parms; /* The PLL parameters */
};

struct imx9_pfd_cfg_s
{
  uintptr_t          reg;   /* The PLL register base */
  int                pfd;   /* The PFD number */
  struct pfd_parms_s parms; /* The PFD parameters */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_clockconfig
 *
 * Description:
 *   Called to initialize the i.IMX9.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void imx9_clockconfig(void);

/****************************************************************************
 * Name: imx9_get_clock
 *
 * Description:
 *   This function returns the clock frequency of the specified functional
 *   clock.
 *
 * Input Parameters:
 *   clkname   - Identifies the clock of interest
 *   frequency - The location where the peripheral clock frequency will be
 *              returned
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int imx9_get_clock(int clkname, uint32_t *frequency);

/****************************************************************************
 * Name: imx9_get_rootclock
 *
 * Description:
 *   This function returns the clock frequency of the specified root
 *   functional clock.
 *
 * Input Parameters:
 *   clkroot   - Identifies the peripheral clock of interest
 *   frequency - The location where the peripheral clock frequency will be
 *              returned
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int imx9_get_rootclock(int clkroot, uint32_t *frequency);

#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_CLOCKCONFIG_H */
