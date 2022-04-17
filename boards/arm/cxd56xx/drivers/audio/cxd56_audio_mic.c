/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_mic.c
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
#include <arch/chip/audio.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_mic.h"
#include "cxd56_audio_ac_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CIC_NUM           4
#define CIC_MIC_CH_NUM    2

#define MIC_CH_BITNUM  4
#define MIC_CH_BITMAP  0xf

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_mic_enable(cxd56_audio_mic_gain_t *gain)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;
  uint8_t mic_num;
  uint8_t cic_num;
  uint8_t mic_sel;
  uint8_t mic_mode;
  uint32_t mic_map;
  cxd56_audio_mic_gain_t cic_gain;
  cxd56_audio_clkmode_t clk_mode;
  uint8_t i;

  /* Get mic number. */

  mic_num = cxd56_audio_config_get_micnum();

  /* Get CIC filter number. */

  cic_num = (mic_num + 1) / CIC_MIC_CH_NUM;

  /* Get mic mode. */

  mic_mode = cxd56_audio_config_get_micmode();

  /* Set cic gain. */

  mic_map = cxd56_audio_config_get_micmap();

  for (i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
    {
      mic_sel = (mic_map >> (i * MIC_CH_BITNUM)) &
                MIC_CH_BITMAP;
      if ((mic_sel >= 1) && (mic_sel <= 4))
        {
          cic_gain.gain[i] = 0;
        }
      else
        {
          cic_gain.gain[i] = gain->gain[i];
        }
    }

  ret = cxd56_audio_ac_reg_poweron_cic(CXD56_AUDIO_CFG_CIC_IN,
                                       mic_mode,
                                       cic_num,
                                       &cic_gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  clk_mode = cxd56_audio_config_get_clkmode();
  ret = cxd56_audio_ac_reg_poweron_decim(mic_mode, clk_mode);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_mic_disable(void)
{
  /* Disable DECIM. */

  cxd56_audio_ac_reg_poweroff_decim();

  /* Power off CIC. */

  cxd56_audio_ac_reg_poweroff_cic();

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_mic_set_gain(cxd56_audio_mic_gain_t *gain)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;
  uint8_t mic_num;
  uint8_t cic_num;
  uint8_t mic_sel;
  uint32_t mic_map;
  cxd56_audio_mic_gain_t cic_gain;
  uint8_t i;

  /* Get mic number. */

  mic_num = cxd56_audio_config_get_micnum();

  /* Get CIC filter number. */

  cic_num = (mic_num + 1) / CIC_MIC_CH_NUM;

  /* Set cic gain. */

  mic_map = cxd56_audio_config_get_micmap();

  for (i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
    {
      mic_sel = (mic_map >> (i * MIC_CH_BITNUM)) &
                MIC_CH_BITMAP;
      if ((mic_sel >= 1) && (mic_sel <= 4))
        {
          cic_gain.gain[i] = 0;
        }
      else
        {
          cic_gain.gain[i] = gain->gain[i];
        }
    }

  cxd56_audio_ac_reg_set_cicgain(cic_num, &cic_gain);

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_mic_set_seloutch(uint8_t dma_mic_num,
                                               cxd56_audio_samp_fmt_t format)
{
  uint8_t i;
  cxd56_audio_ac_reg_seloutch_t seloutch;

  if ((format == CXD56_AUDIO_SAMP_FMT_16) &&
      (CXD56_AUDIO_CFG_DMA_FORMAT == CXD56_AUDIO_CFG_DMA_FORMAT_RL))
    {
      for (i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
        {
          seloutch.ch[i] = (i & 1) ? i - 1 : i + 1;
        }
    }
  else
    {
      for (i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
        {
          seloutch.ch[i] = i;
        }
    }

  if ((format == CXD56_AUDIO_SAMP_FMT_16) && ((dma_mic_num & 1) == 1))
    {
      if (CXD56_AUDIO_CFG_DMA_FORMAT == CXD56_AUDIO_CFG_DMA_FORMAT_LR)
        {
          seloutch.ch[dma_mic_num] = seloutch.ch[dma_mic_num - 1];
        }
      else
        {
          seloutch.ch[dma_mic_num - 1] = seloutch.ch[dma_mic_num];
        }
    }

  cxd56_audio_ac_reg_set_seloutch(&seloutch);

  return CXD56_AUDIO_ECODE_OK;
}
