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

void imx9_sm_iomucx_configure(scmi_pinctrl_t *pinctrl)
{
  scmi_pin_config_t configs[4];
  uint32_t num_configs     = 0;
  uint32_t channel         = pinctrl->channel;
  uint32_t mux_register    = pinctrl->mux_register;
  uint32_t mux_mode        = pinctrl->mux_mode;
  uint32_t input_register  = pinctrl->input_register;
  uint32_t input_daisy     = pinctrl->input_daisy;
  uint32_t input_on_field  = pinctrl->input_on_field;
  uint32_t config_register = pinctrl->config_register;
  uint32_t config_value    = pinctrl->config_value;

  if (mux_register)
    {
      configs[num_configs].type  = SCMI_PINCTRL_TYPE_MUX;
      configs[num_configs].value = SCMI_PLATFORM_PINCTRL_MUX_MODE(mux_mode)
                                   | SCMI_PLATFORM_PINCTRL_SION(
                                        input_on_field);
      num_configs++;
    }

  if (input_register & 0xffff)
    {
      configs[num_configs].type = SCMI_PINCTRL_TYPE_DAISY_ID;
      configs[num_configs].value =
          (input_register - SCMI_PLATFORM_PINCTRL_DAISYREG_OFF) / 4;
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

      int32_t ret = imx9_scmi_set_pinctrl_config(
          channel, (mux_register - SCMI_PLATFORM_PINCTRL_MUXREG_OFF) / 4,
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

  scmi_pinctrl_t pinctrl =
    {
      0
    };

  pinctrl.channel         = SCMI_PLATFORM_A2P;
  pinctrl.mux_register    = cfg.padcfg.ctlreg;
  pinctrl.mux_mode        = cfg.padcfg.mode;
  pinctrl.input_register  = cfg.padcfg.dsy;
  pinctrl.input_daisy     = cfg.padcfg.dsy;
  pinctrl.config_register = cfg.padcfg.padreg;
  pinctrl.config_value    = cfg.pad;
  pinctrl.input_on_field  = cfg.mux;
  imx9_sm_iomucx_configure(&pinctrl);

#else

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

#endif

  return OK;
}

#ifndef CONFIG_IMX9_IOMUX_OVER_SCMI

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

#endif
