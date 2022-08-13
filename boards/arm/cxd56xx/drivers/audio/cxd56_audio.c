/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <debug.h>

#include <arch/board/board.h>
#include <arch/chip/audio.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_power.h"
#include "cxd56_audio_filter.h"
#include "cxd56_audio_mic.h"
#include "cxd56_audio_volume.h"
#include "cxd56_audio_digital.h"
#include "cxd56_audio_beep.h"
#include "cxd56_audio_irq.h"
#include "cxd56_audio_dma.h"
#include "cxd56_audio_pin.h"
#include "cxd56_audio_analog.h"
#include "cxd56_audio_ac_reg.h"
#include "cxd56_audio_bca_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AUDIO_DNC_ID_NUM      (CXD56_AUDIO_DNC_ID_FF + 1)
#define AUDIO_VOL_ID_NUM      (CXD56_AUDIO_VOLID_MIXER_OUT + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct clear_stereo_param_s
{
  bool    en;
  bool    sign_inv;
  int16_t vol;
};

struct power_param_s
{
  bool dnc;
};

struct dnc_param_s
{
  bool en;
  cxd56_audio_dnc_id_t   id;
  cxd56_audio_dnc_bin_t *bin;
};

struct deq_param_s
{
  bool en;
  cxd56_audio_deq_coef_t *coef;
};

struct input_param_s
{
  bool en;
  cxd56_audio_mic_gain_t gain;
};

struct output_param_s
{
  bool en;
  bool ana_en;
};

struct vol_param_s
{
  cxd56_audio_volid_t id;
  int16_t             vol;
  bool                mute;
};

struct vol_beep_s
{
  bool     en;
  int16_t  vol;
  bool     mute;
  uint16_t freq;
};

struct data_path_s
{
  cxd56_audio_signal_t sig;
  cxd56_audio_sel_t    sel;
};

struct power_on_param_s
{
  struct clear_stereo_param_s cs;
  struct power_param_s        pw;
  struct dnc_param_s          dnc[AUDIO_DNC_ID_NUM];
  struct deq_param_s          deq;
  struct input_param_s        input;
  struct output_param_s       output;
  struct vol_param_s          vol[AUDIO_VOL_ID_NUM];
  struct vol_beep_s           beep;
  struct data_path_s          path;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Status of audio driver. */

static cxd56_audio_state_t g_status = CXD56_AUDIO_POWER_STATE_OFF;

/* Parameter of poweron setting. */

static struct power_on_param_s g_pwon_param =
{
  .cs =
    {
      .en       = false,
      .sign_inv = false,
      .vol      = -825,
    },
  .pw =
    {
      .dnc = false,
    },
  .dnc[0] =
    {
      .en  = false,
      .id  = 0,
      .bin = NULL,
    },
  .dnc[1] =
    {
      .en  = false,
      .id  = 1,
      .bin = NULL,
    },
  .deq =
    {
      .en   = false,
      .coef = NULL,
    },
  .input =
    {
      .en = false,
    },
  .output =
    {
      .en     = false,
      .ana_en = false,
    },
  .vol[0] =
    {
      .id   = 0,
      .vol  = -1025,
      .mute = false,
    },
  .vol[1] =
    {
      .id   = 1,
      .vol  = -1025,
      .mute = false,
    },
  .vol[2] =
    {
      .id   = 2,
      .vol  = -1025,
      .mute = false,
    },
  .path =
    {
      .sig = CXD56_AUDIO_SIG_MIC1,
      .sel.au_dat_sel1 = false,
      .sel.au_dat_sel2 = false,
      .sel.cod_insel2  = false,
      .sel.cod_insel3  = false,
      .sel.src1in_sel  = false,
      .sel.src2in_sel  = false,
    }
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_poweron(void)
{
  int i;

  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  if (CXD56_AUDIO_POWER_STATE_OFF != g_status)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  /* Initialize board I/O. */

  board_audio_initialize();

  /* Initialize config. */

  cxd56_audio_config_init();

  /* Power On analog block. */

  ret = cxd56_audio_analog_poweron();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Power On audio codec block. */

  ret = cxd56_audio_power_on();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Enable interrupt. */

  cxd56_audio_irq_attach();
  cxd56_audio_irq_enable();

  /* Update status. */

  g_status = CXD56_AUDIO_POWER_STATE_ON;

  /* Set initialize parameter. */

  if (g_pwon_param.cs.en)
    {
      ret = cxd56_audio_en_cstereo(g_pwon_param.cs.sign_inv,
                                   g_pwon_param.cs.vol);
    }

  if (g_pwon_param.pw.dnc)
    {
      for (i = 0; i < AUDIO_DNC_ID_NUM; i++)
        {
          if (g_pwon_param.dnc[i].en)
            {
              ret = cxd56_audio_en_dnc(g_pwon_param.dnc[i].id,
                                       g_pwon_param.dnc[i].bin);
            }
        }
    }

  if (g_pwon_param.deq.en)
    {
      cxd56_audio_filter_set_deq(g_pwon_param.deq.en,
                                 g_pwon_param.deq.coef);
    }

  if (g_pwon_param.input.en)
    {
      ret = cxd56_audio_en_input();
    }

  if (g_pwon_param.output.en)
    {
      ret = cxd56_audio_en_output();
    }

  for (i = 0; i < AUDIO_VOL_ID_NUM; i++)
    {
      ret = cxd56_audio_set_vol(g_pwon_param.vol[i].id,
                                g_pwon_param.vol[i].vol);
      if (g_pwon_param.vol[i].mute)
        {
          ret = cxd56_audio_mute_vol(g_pwon_param.vol[i].id);
        }
    }

  ret = cxd56_audio_set_datapath(g_pwon_param.path.sig,
                                 g_pwon_param.path.sel);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      g_status = CXD56_AUDIO_POWER_STATE_OFF;
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_poweroff(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check status */

  if (CXD56_AUDIO_POWER_STATE_ON != g_status)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  /* Power off audio codec block. */

  ret = cxd56_audio_power_off();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Power off analog block. */

  ret = cxd56_audio_analog_poweroff();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Disable interrupt. */

  cxd56_audio_irq_disable();
  cxd56_audio_irq_detach();

  /* Finalize board I/O. */

  board_audio_finalize();

  /* Update status. */

  g_status = CXD56_AUDIO_POWER_STATE_OFF;

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_en_cstereo(bool sign_inv, int16_t vol)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.cs.en       = true;
  g_pwon_param.cs.sign_inv = sign_inv;
  g_pwon_param.cs.vol      = vol;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_filter_set_cstereo(true, sign_inv, vol);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_dis_cstereo(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.cs.en = false;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_filter_set_cstereo(false, false, 0);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_poweron_dnc(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.pw.dnc = true;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  cxd56_audio_filter_poweron_dnc();

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_poweroff_dnc(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.pw.dnc = false;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  cxd56_audio_filter_poweroff_dnc();

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_en_dnc(cxd56_audio_dnc_id_t id,
                                     cxd56_audio_dnc_bin_t *bin)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.dnc[id].en  = true;
  g_pwon_param.dnc[id].id  = id;
  g_pwon_param.dnc[id].bin = bin;

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  cxd56_audio_filter_set_dnc(id, true, bin);

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_dis_dnc(cxd56_audio_dnc_id_t id)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.dnc[id].en  = false;
  g_pwon_param.dnc[id].id  = id;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  cxd56_audio_filter_set_dnc(id, false, NULL);

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_en_deq(cxd56_audio_deq_coef_t *coef)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.deq.en = true;
  if (coef != NULL)
    {
      g_pwon_param.deq.coef = coef;
    }

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  cxd56_audio_filter_set_deq(true, coef);

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_dis_deq(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.deq.en = false;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  cxd56_audio_filter_set_deq(false, NULL);

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_en_input(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_NONE
  return CXD56_AUDIO_ECODE_MIC_NO_ANA;
#endif

  /* Save power on parameters. */

  g_pwon_param.input.en = true;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_analog_poweron_input(&g_pwon_param.input.gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  ret = cxd56_audio_mic_enable(&g_pwon_param.input.gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_en_output(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.output.en = true;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  if (g_pwon_param.output.ana_en)
    {
      ret = cxd56_audio_analog_poweron_output();
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }

      /* Enable S-Master. */

      cxd56_audio_clkmode_t clk_mode = cxd56_audio_config_get_clkmode();
      cxd56_audio_ac_reg_poweron_smaster(clk_mode);
      cxd56_audio_bca_reg_set_smaster();
      cxd56_audio_ac_reg_enable_smaster();
    }
  else
    {
      ret = cxd56_audio_analog_poweroff_output();
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }
#endif

  ret = cxd56_audio_volume_unmute(CXD56_AUDIO_VOLID_MIXER_OUT);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_dis_input(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

#ifdef CONFIG_CXD56_AUDIO_ANALOG_NONE
  return CXD56_AUDIO_ECODE_MIC_NO_ANA;
#endif

  /* Save power on parameters. */

  g_pwon_param.input.en = false;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_mic_disable();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  ret = cxd56_audio_analog_poweroff_input();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_dis_output(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.output.en = false;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  /* Mute output. */

  ret = cxd56_audio_volume_mute(CXD56_AUDIO_VOLID_MIXER_OUT);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

#ifndef CONFIG_CXD56_AUDIO_ANALOG_NONE
  if (g_pwon_param.output.ana_en)
    {
      cxd56_audio_ac_reg_disable_smaster();
      cxd56_audio_ac_reg_poweroff_smaster();

      ret = cxd56_audio_analog_poweroff_output();
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

#endif
  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_set_spout(bool sp_out_en)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      /* Save power on parameters. */

      g_pwon_param.output.ana_en = sp_out_en;
      return ret;
    }

  g_pwon_param.output.ana_en = sp_out_en;

  /* Actual switching timing is when cxd56_audio_en_output() is executed.
   * When calling this function, execute cxd56_audio_en_output().
   */

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_set_vol(cxd56_audio_volid_t id, int16_t vol)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.vol[id].id  = id;
  g_pwon_param.vol[id].vol = vol;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_volume_set(id, vol);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_mute_vol(cxd56_audio_volid_t id)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.vol[id].mute = true;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_volume_mute(id);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_unmute_vol(cxd56_audio_volid_t id)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.vol[id].mute = false;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_volume_unmute(id);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_mute_vol_fade(cxd56_audio_volid_t id,
                                            bool wait)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of state. PowerON state only. */

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  ret = cxd56_audio_volume_mute_fade(id, wait);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_unmute_vol_fade(cxd56_audio_volid_t id,
                                              bool wait)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of state. PowerON state only. */

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  ret = cxd56_audio_volume_unmute_fade(id, wait);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_set_beep_freq(uint16_t freq)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.beep.freq = freq;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_beep_set_freq(freq);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_set_beep_vol(int16_t vol)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.beep.vol = vol;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_beep_set_vol(vol);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_play_beep(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.beep.en = true;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  cxd56_audio_beep_play();

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_stop_beep(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.beep.en = false;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  cxd56_audio_beep_stop();

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_set_micgain(cxd56_audio_mic_gain_t *gain)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  if (gain == NULL)
    {
      return CXD56_AUDIO_ECODE_MIC_ARG_NULL;
    }

  /* Save power on parameters. */

  g_pwon_param.input.gain = *gain;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  /* Set analog mic gain. */

  ret = cxd56_audio_analog_set_micgain(gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  /* Set digital mic gain. */

  ret = cxd56_audio_mic_set_gain(gain);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_get_dmahandle(cxd56_audio_dma_path_t path,
                                            cxd56_audio_dma_t *handle)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of argument. */

  if (handle == NULL)
    {
      return CXD56_AUDIO_ECODE_DMA_ARG_NULL;
    }

  ret = cxd56_audio_dma_get_handle(path, handle);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  if (CXD56_AUDIO_DMA_PATH_MIC_TO_MEM == path)
    {
      ret = cxd56_audio_analog_wait_input_standby();
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_free_dmahandle(cxd56_audio_dma_t handle)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_dma_free_handle(handle);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_set_datapath(cxd56_audio_signal_t sig,
                                           cxd56_audio_sel_t sel)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Save power on parameters. */

  g_pwon_param.path.sig = sig;
  g_pwon_param.path.sel = sel;
  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return ret;
    }

  ret = cxd56_audio_ac_reg_set_selector(sig, sel);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  if (CXD56_AUDIO_SIG_MIC1 == sig ||
      CXD56_AUDIO_SIG_MIC2 == sig ||
      CXD56_AUDIO_SIG_MIC3 == sig ||
      CXD56_AUDIO_SIG_MIC4 == sig)
    {
      ret = cxd56_audio_analog_wait_input_standby();
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_init_dma(cxd56_audio_dma_t handle,
                                       cxd56_audio_samp_fmt_t fmt,
                                       uint8_t *ch_num)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of argument. */

  if (ch_num == NULL)
    {
      return CXD56_AUDIO_ECODE_DMA_ARG_NULL;
    }

  /* Check error of state. PowerON state only. */

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  ret = cxd56_audio_dma_init(handle, fmt, ch_num);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_set_dmacb(cxd56_audio_dma_t handle,
                                        cxd56_audio_dma_cb_t cb)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of argument. */

  if (cb == NULL)
    {
      return CXD56_AUDIO_ECODE_DMA_ARG_NULL;
    }

  ret = cxd56_audio_dma_set_cb(handle, cb);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE
cxd56_audio_get_dmamstate(cxd56_audio_dma_t handle,
                          cxd56_audio_dma_mstate_t *state)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of argument. */

  if (state == NULL)
    {
      return CXD56_AUDIO_ECODE_DMA_ARG_NULL;
    }

  /* Check error of state. PowerON state only. */

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  ret = cxd56_audio_dma_get_mstate(handle, state);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_en_dmaint(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_dma_en_dmaint();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_dis_dmaint(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  ret = cxd56_audio_dma_dis_dmaint();
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_clear_dmaerrint(cxd56_audio_dma_t handle)
{
  cxd56_audio_bca_reg_clear_err_int(handle);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_mask_dmaerrint(cxd56_audio_dma_t handle)
{
  cxd56_audio_bca_reg_mask_err_int(handle);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_unmask_dmaerrint(cxd56_audio_dma_t handle)
{
  cxd56_audio_bca_reg_unmask_err_int(handle);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_start_dma(cxd56_audio_dma_t handle,
                                        uint32_t addr,
                                        uint32_t sample)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of state. PowerON state only. */

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  ret = cxd56_audio_dma_start(handle, addr, sample);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_stop_dma(cxd56_audio_dma_t handle)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of state. PowerON state only. */

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  ret = cxd56_audio_dma_stop(handle);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_set_clkmode(cxd56_audio_clkmode_t mode)
{
  if ((CXD56_AUDIO_CFG_MCLK == CXD56_AUDIO_CFG_XTAL_24_576MHZ) &&
      (mode == CXD56_AUDIO_CLKMODE_HIRES))
    {
      return CXD56_AUDIO_ECODE_CFG_CLK_MODE;
    }

  cxd56_audio_config_set_clkmode(mode);

  return CXD56_AUDIO_ECODE_OK;
}

cxd56_audio_clkmode_t cxd56_audio_get_clkmode(void)
{
  return cxd56_audio_config_get_clkmode();
}

cxd56_audio_dmafmt_t cxd56_audio_get_dmafmt(void)
{
  cxd56_audio_dmafmt_t fmt;

  if (CXD56_AUDIO_CFG_DMA_FORMAT == CXD56_AUDIO_CFG_DMA_FORMAT_LR)
    {
      fmt = CXD56_AUDIO_DMA_FMT_LR;
    }
  else
    {
      fmt = CXD56_AUDIO_DMA_FMT_RL;
    }

  return fmt;
}

cxd56_audio_micdev_t cxd56_audio_get_micdev(void)
{
  cxd56_audio_micdev_t micdev;
  uint8_t cfg_micdev = cxd56_audio_config_get_micdev();

  switch (cfg_micdev)
    {
      case CXD56_AUDIO_CFG_MIC_DEV_ANALOG:
        micdev = CXD56_AUDIO_MIC_DEV_ANALOG;
        break;

      case CXD56_AUDIO_CFG_MIC_DEV_DIGITAL:
        micdev = CXD56_AUDIO_MIC_DEV_DIGITAL;
        break;

      case CXD56_AUDIO_CFG_MIC_DEV_ANADIG:
        micdev = CXD56_AUDIO_MIC_DEV_ANADIG;
        break;

      default:
        micdev = CXD56_AUDIO_MIC_DEV_NONE;
        break;
    }

  return micdev;
}

CXD56_AUDIO_ECODE cxd56_audio_en_digsft(cxd56_audio_dsr_rate_t rate)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of state. PowerON state only. */

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  cxd56_audio_ac_reg_set_dsrrate(rate);
  cxd56_audio_ac_reg_enable_digsft();

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_dis_digsft(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of state. PowerON state only. */

  if (g_status == CXD56_AUDIO_POWER_STATE_OFF)
    {
      return CXD56_AUDIO_ECODE_POW_STATE;
    }

  cxd56_audio_ac_reg_disable_digsft();

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_en_i2s_io(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Enable I2S pin. */

  cxd56_audio_pin_i2s_set();

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_dis_i2s_io(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Disable I2S pin. */

  cxd56_audio_pin_i2s_unset();

  return ret;
}

cxd56_audio_state_t cxd56_audio_get_status(void)
{
  return g_status;
}

CXD56_AUDIO_ECODE cxd56_audio_set_spdriver(cxd56_audio_sp_drv_t sp_driver)
{
  cxd56_audio_config_set_spdriver(sp_driver);

  return CXD56_AUDIO_ECODE_OK;
}

cxd56_audio_sp_drv_t cxd56_audio_get_spdriver(void)
{
  return cxd56_audio_config_get_spdriver();
}

CXD56_AUDIO_ECODE cxd56_audio_set_micmap(uint32_t map)
{
  cxd56_audio_config_set_micmap(map);

  return CXD56_AUDIO_ECODE_OK;
}

uint32_t cxd56_audio_get_micmap(void)
{
  return cxd56_audio_config_get_micmap();
}

bool board_audio_tone_generator(bool en, int16_t vol, uint16_t freq)
{
  if (!en)
    {
      /* Stop beep */

      if (cxd56_audio_stop_beep() != 0)
        {
          return false;
        }
    }

  if (0 != freq)
    {
      /* Set beep frequency parameter */

      if (cxd56_audio_set_beep_freq(freq) != 0)
        {
          return false;
        }
    }

  if (255 != vol)
    {
      /* Set beep volume parameter */

      if (cxd56_audio_set_beep_vol(vol) != 0)
        {
          return false;
        }
    }

  if (en)
    {
      /* Play beep */

      if (cxd56_audio_play_beep() != 0)
        {
          return false;
        }
    }

  return true;
}
