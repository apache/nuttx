/****************************************************************************
 * arch/arm64/src/imx9/imx9_iomuxc.c
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

#include "arm64_internal.h"
#include "imx9_iomuxc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_iomux_configure
 *
 * Description:
 *   This function writes the encoded pad configuration to the Pad Control
 *   register.
 *
 * Input Parameters:
 *   cfg - The IOMUX configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int imx9_iomux_configure(iomux_cfg_t cfg)
{
  if (!cfg.padcfg.ctlreg)
    {
      return -EINVAL;
    }

  putreg32(cfg.padcfg.mode | cfg.mux, cfg.padcfg.ctlreg);

  if (cfg.padcfg.dsyreg)
    {
      putreg32(cfg.padcfg.dsy, cfg.padcfg.dsyreg);
    }

  if (cfg.padcfg.padreg)
    {
      putreg32(cfg.pad, cfg.padcfg.padreg);
    }

  return OK;
}

/****************************************************************************
 * Name: imx9_iomux_configure
 *
 * Description:
 *   This can be used to forcibly set a pad to GPIO mode. This overrides and
 *   disconnects any peripheral using the pin.
 *
 * Input Parameters:
 *   cfg  - The IOMUX configuration.
 *   sion - if true; sets SION, otherwise clears it.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int imx9_iomux_gpio(iomux_cfg_t cfg, bool sion)
{
  uint32_t reg_sion;

  if (!cfg.padcfg.ctlreg)
    {
      return -EINVAL;
    }

  /* Set sion if requested to do so */

  reg_sion = sion ? IOMUXC_MUX_SION_ON : 0;

  /* Based on pad number, either ALT0/ALT5 sets the pad as GPIO */

  if ((cfg.padcfg.ctlreg >= IOMUXC_MUX_CTL_GPIO_IO00_OFFSET) &&
      (cfg.padcfg.ctlreg <= IOMUXC_MUX_CTL_GPIO_IO29_OFFSET))
    {
      putreg32(IOMUXC_MUX_MODE_ALT0 | reg_sion, cfg.padcfg.ctlreg);
    }
  else
    {
      putreg32(IOMUXC_MUX_MODE_ALT5 | reg_sion, cfg.padcfg.ctlreg);
    }

  return OK;
}
