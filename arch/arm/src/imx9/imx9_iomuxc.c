/****************************************************************************
 * arch/arm/src/imx9/imx9_iomuxc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm_internal.h"
#include "hardware/imx9_memorymap.h"
#include "imx9_iomuxc.h"

#ifdef CONFIG_IMX9_IOMUX_OVER_SCMI
#include "imx9_scmi.h"
#endif

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

#ifdef CONFIG_IMX9_IOMUX_OVER_SCMI

/* Sets the IOMUXC pin mux mode.
 */

void imx9_sm_iomucx_configure(sm_pinctrl_t *sm_pinctrl)
{
  scmi_pin_config_t configs[4];
  uint32_t num_configs     = 0;
  uint32_t channel         = sm_pinctrl->channel;
  uint32_t mux_register    = sm_pinctrl->mux_register;
  uint32_t mux_mode        = sm_pinctrl->mux_mode;
  uint32_t input_register  = sm_pinctrl->input_register;
  uint32_t input_daisy     = sm_pinctrl->input_daisy;
  uint32_t input_on_field  = sm_pinctrl->input_on_field;
  uint32_t config_register = sm_pinctrl->config_register;
  uint32_t config_value    = sm_pinctrl->config_value;

  if (mux_register)
    {
      configs[num_configs].type  = SCMI_PINCTRL_TYPE_MUX;
      configs[num_configs].value = SM_PLATFORM_PINCTRL_MUX_MODE(mux_mode)
                                   | SM_PLATFORM_PINCTRL_SION(
                                        input_on_field);
      num_configs++;
    }

  if (input_register & 0xffff)
    {
      configs[num_configs].type = SCMI_PINCTRL_TYPE_DAISY_ID;
      configs[num_configs].value =
          (input_register - SM_PLATFORM_PINCTRL_DAISYREG_OFF) / 4;
      num_configs++;
      configs[num_configs].type  = SCMI_PINCTRL_TYPE_DAISY_CFG;
      configs[num_configs].value = input_daisy;
      num_configs++;
    }

  if (config_register)
    {
      configs[num_configs].type  = SCMI_PINCTRL_TYPE_CONFIG;
      configs[num_configs].value = config_value;
      num_configs++;
    }

  if (mux_register || input_register)
    {
      uint32_t attributes = SCMI_PINCTRL_SET_ATTR_SELECTOR(
                                  SCMI_PINCTRL_SEL_PIN)
                            | SCMI_PINCTRL_SET_ATTR_NUM_CONFIGS(
                                  num_configs);

      int32_t ret = imx9_scmi_pinctrlconfigset(
          channel, (mux_register - SM_PLATFORM_PINCTRL_MUXREG_OFF) / 4,
          attributes, configs);
      if (ret != 0)
        {
          assert("Unable to set iomux over SCMI");
        }
    }
}

#endif

int imx9_iomux_configure(iomux_cfg_t cfg)
{
#ifdef CONFIG_IMX9_IOMUX_OVER_SCMI
  sm_pinctrl_t sm_pinctrl =
    {
      0
    };

  sm_pinctrl.channel         = SM_PLATFORM_A2P;
  sm_pinctrl.mux_register    = IMX9_IOMUXC_BASE + cfg.padcfg.ctlregoff;
  sm_pinctrl.mux_mode        = cfg.padcfg.mode;
  sm_pinctrl.input_register  = IMX9_IOMUXC_BASE + cfg.padcfg.dsyregoff;
  sm_pinctrl.input_daisy     = cfg.padcfg.dsy;
  sm_pinctrl.config_register = IMX9_IOMUXC_BASE + cfg.padcfg.padregoff;
  sm_pinctrl.config_value    = cfg.pad;
  sm_pinctrl.input_on_field  = cfg.sion;

  imx9_sm_iomucx_configure(&sm_pinctrl);
#else
  if (!cfg.padcfg.ctlregoff)
    {
      return -EINVAL;
    }

  putreg32(cfg.padcfg.mode | (cfg.sion << IOMUXC_MUX_SION_SHIFT),
           IMX9_IOMUXC_BASE + cfg.padcfg.ctlregoff);

  if (cfg.padcfg.dsyregoff)
    {
      putreg32(cfg.padcfg.dsy, IMX9_IOMUXC_BASE + cfg.padcfg.dsyregoff);
    }

  if (cfg.padcfg.padregoff)
    {
      putreg32(cfg.pad << IOMUXC_PAD_CONFIG_SHIFT,
               IMX9_IOMUXC_BASE + cfg.padcfg.padregoff);
    }

#endif
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

  if (!cfg.padcfg.ctlregoff)
    {
      return -EINVAL;
    }

  /* Set sion if requested to do so */

  reg_sion = sion ? IOMUXC_MUX_SION_ON : 0;

  /* Based on pad number, either ALT0/ALT5 sets the pad as GPIO */

  if ((cfg.padcfg.ctlregoff >= IOMUXC_MUX_CTL_GPIO_IO00_OFFSET)
      && (cfg.padcfg.ctlregoff <= IOMUXC_MUX_CTL_GPIO_IO37_OFFSET))
    {
      putreg32(IOMUXC_MUX_MODE_ALT0 | reg_sion,
               IMX9_IOMUXC_BASE + cfg.padcfg.ctlregoff);
    }

  else
    {
      putreg32(IOMUXC_MUX_MODE_ALT5 | reg_sion,
               IMX9_IOMUXC_BASE + cfg.padcfg.ctlregoff);
    }

  return OK;
}
