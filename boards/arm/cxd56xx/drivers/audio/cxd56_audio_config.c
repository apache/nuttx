/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_config.c
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
#include "cxd56_audio_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIC_CH_BITNUM  4
#define MIC_CH_BITMAP  0xf

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct cxd56_audio_cfg_s
{
  uint8_t mic_num;
  uint8_t mic_dev;
  uint8_t mic_mode;
  uint32_t mic_map;
  cxd56_audio_clkmode_t clk_mode;
  cxd56_audio_sp_drv_t  sp_driver;
};

static struct cxd56_audio_cfg_s g_audio_cfg =
{
  1,
  CXD56_AUDIO_CFG_MIC_DEV_ANADIG,
  CXD56_AUDIO_CFG_MIC_MODE_64FS,
  CXD56_AUDIO_CFG_MIC,
  CXD56_AUDIO_CLKMODE_NORMAL,
  CXD56_AUDIO_CFG_SP_DRIVER
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void set_miccfg(void)
{
  bool is_amic = false;
  bool is_dmic = false;
  uint8_t mic_num = 0;
  uint8_t mic_sel = 0;
  uint8_t i;

  for (i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
    {
      mic_sel = (g_audio_cfg.mic_map >> (i * MIC_CH_BITNUM)) &
                MIC_CH_BITMAP;
      if ((mic_sel >= 1) && (mic_sel <= 4))
        {
          is_amic = true;
          mic_num++;
        }
      else if ((mic_sel >= 5) && (mic_sel <= 12))
        {
          is_dmic = true;
          mic_num++;
        }
    }

  /* Set mic number. */

  g_audio_cfg.mic_num = mic_num;

  /* Set mic device type and mode. */

  if (is_amic)
    {
      if (is_dmic)
        {
          g_audio_cfg.mic_dev  = CXD56_AUDIO_CFG_MIC_DEV_ANADIG;
          g_audio_cfg.mic_mode = CXD56_AUDIO_CFG_MIC_MODE_64FS;
        }
      else
        {
          g_audio_cfg.mic_dev  = CXD56_AUDIO_CFG_MIC_DEV_ANALOG;
          g_audio_cfg.mic_mode = CXD56_AUDIO_CFG_MIC_MODE_128FS;
        }
    }
  else
    {
      if (is_dmic)
        {
          g_audio_cfg.mic_dev  = CXD56_AUDIO_CFG_MIC_DEV_DIGITAL;
          g_audio_cfg.mic_mode = CXD56_AUDIO_CFG_MIC_MODE_64FS;
        }
      else
        {
          g_audio_cfg.mic_dev  = CXD56_AUDIO_CFG_MIC_DEV_NONE;
          g_audio_cfg.mic_mode = CXD56_AUDIO_CFG_MIC_MODE_64FS;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cxd56_audio_config_init(void)
{
  /* Set mic config */

  set_miccfg();
}

uint8_t cxd56_audio_config_get_micmode(void)
{
  return g_audio_cfg.mic_mode;
}

uint8_t cxd56_audio_config_get_micdev(void)
{
  return g_audio_cfg.mic_dev;
}

uint8_t cxd56_audio_config_get_micnum(void)
{
  return g_audio_cfg.mic_num;
}

void cxd56_audio_config_set_spdriver(cxd56_audio_sp_drv_t sp_driver)
{
  g_audio_cfg.sp_driver = sp_driver;
}

cxd56_audio_sp_drv_t cxd56_audio_config_get_spdriver(void)
{
  return g_audio_cfg.sp_driver;
}

void cxd56_audio_config_set_clkmode(cxd56_audio_clkmode_t mode)
{
  g_audio_cfg.clk_mode = mode;
}

cxd56_audio_clkmode_t cxd56_audio_config_get_clkmode(void)
{
  return g_audio_cfg.clk_mode;
}

void cxd56_audio_config_set_micmap(uint32_t map)
{
  g_audio_cfg.mic_map = map;
}

uint32_t cxd56_audio_config_get_micmap(void)
{
  return g_audio_cfg.mic_map;
}
