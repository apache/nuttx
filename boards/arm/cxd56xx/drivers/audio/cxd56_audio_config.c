/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_config.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
