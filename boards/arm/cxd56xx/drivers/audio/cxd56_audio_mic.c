/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_mic.c
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

CXD56_AUDIO_ECODE cxd56_audio_mic_enable(FAR cxd56_audio_mic_gain_t *gain)
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

CXD56_AUDIO_ECODE cxd56_audio_mic_set_gain(FAR cxd56_audio_mic_gain_t *gain)
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
