/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_aca.c
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

#include <string.h>
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <debug.h>

#include <arch/board/board.h>
#include <arch/chip/audio.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_aca.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIC_CH_BITNUM  4
#define MIC_CH_BITMAP  0xf

#define MIC_GAIN_MAX   150
#define MIC_GAIN_MIN   0

#define PGA_GAIN_MAX   60
#define PGA_GAIN_MIN   0

#define AS_VGAIN_MAX   60
#define AS_VGAIN_MIN   -95

typedef enum
{
  AS_ACA_OSC_UNKNOWN,
  AS_ACA_OSC_24_576MHZ,        /* 24.576MHz */
  AS_ACA_OSC_24_576MHZ_HIRES,  /* 24.576MHz,Hi-Res */
  AS_ACA_OSC_49_152MHZ,        /* 49.152MHz */
  AS_ACA_OSC_49_152MHZ_HIRES,  /* 49.152MHz,Hi-Res */
  AS_ACA_OSC_MAX_ENTRY
} as_aca_pulco_osc_mode_id_t;

typedef enum
{
  AS_ACA_MIC_UNKNOWN,
  AS_ACA_MIC_AMIC,             /* Analog MIC */
  AS_ACA_MIC_DMIC,             /* Digital MIC */
  AS_ACA_MIC_BOTH,             /* Analog MIC and Digital MIC */
  AS_ACA_MIC_MAX_ENTRY
} as_aca_pulco_mic_device_id_t;

typedef enum
{
  AS_ACA_MICBIAS_SEL_UNKNOWN,
  AS_ACA_MICBIAS_SEL_2_0V,     /* 2.0V */
  AS_ACA_MICBIAS_SEL_2_8V,     /* 2.8V */
  AS_ACA_MICBIAS_SEL_MAX_ENTRY
} as_aca_pulco_mic_bias_sel_id_t;

typedef enum
{
  AS_ACA_IO_DS_UNKNOWN,
  AS_ACA_IO_DS_WEAKEST,        /* Weakest */
  AS_ACA_IO_DS_WEAKER,         /* Weaker */
  AS_ACA_IO_DS_STRONGER,       /* Stronger */
  AS_ACA_IO_DS_STRONGEST,      /* Strongest */
  AS_ACA_IO_DS_MAX_ENTRY
} as_aca_pulco_io_ds_id_t;

typedef enum
{
  AS_SMSTR_MODE_FS_UNKNOWN,
  AS_SMSTR_MODE_FS_16,            /* 16fs */
  AS_SMSTR_MODE_FS_32,            /* 32fs */
  AS_SMSTR_MODE_FS_MAX_ENTRY
} as_smstr_mode_id_t;

typedef enum
{
  AS_SMSTR_MCK_FS_UNKNOWN,
  AS_SMSTR_MCK_FS_512,            /* 512fs */
  AS_SMSTR_MCK_FS_1024,           /* 1024fs */
  AS_SMSTR_MCK_FS_MAX_ENTRY
} as_smaster_mck_id_t;

typedef enum
{
  AS_SMSTR_PWMMD_UNKNOWN,
  AS_SMSTR_PWMMD_SINGLE,          /* Single side */
  AS_SMSTR_PWMMD_BOTH,            /* Both side */
  AS_SMSTR_PWMMD_SINGLE_ALTER,    /* Single side alternating */
  AS_SMSTR_PWMMD_BOTH_ALTER,      /* Both side alternating */
  AS_SMSTR_PWMMD_MAX_ENTRY
} as_smaster_pwm_mode_id_t;

typedef enum
{
  AS_SMSTR_CHSEL_UNKNOWN,
  AS_SMSTR_CHSEL_NORMAL,          /* Normal */
  AS_SMSTR_CHSEL_EXCHANGE,        /* Exchange L and R */
  AS_SMSTR_CHSEL_MAX_ENTRY
} as_smaster_ch_sel_id_t;

typedef enum
{
  AS_ACA_OUT_UNKNOWN,
  AS_ACA_OUT_HP,               /* Headphone output */
  AS_ACA_OUT_EP,               /* Ear Speaker output */
  AS_ACA_OUT_PWM,              /* PWM output */
  AS_ACA_OUT_HP_PWM,           /* Headphone and PWM output */
  AS_ACA_OUT_EP_PWM,           /* Ear Speaker and PWM output */
  AS_ACA_OUT_OFF,              /* Disable output */
  AS_ACA_OUT_MAX_ENTRY
} as_aca_pulco_out_dev_id_t;

typedef enum
{
  AS_ACA_PWMOUT_UNKNOWN,
  AS_ACA_PWMOUT_OFF,           /* Disable */
  AS_ACA_PWMOUT_LN,            /* LN */
  AS_ACA_PWMOUT_LP,            /* LP */
  AS_ACA_PWMOUT_RN,            /* RN */
  AS_ACA_PWMOUT_RP,            /* RP */
  AS_ACA_PWMOUT_MAX_ENTRY
} as_aca_pulco_pwm_out_id_t;

typedef enum
{
  AS_ACA_SP_LOOP_MODE_UNKNOWN,
  AS_ACA_SP_LOOP_MODE_ENABLE,
  AS_ACA_SP_LOOP_MODE_DISABLE,
  AS_ACA_SP_LOOP_MODE_MAX_ENTRY
} as_aca_sp_loop_mode_id_t;

typedef enum
{
  AS_ACA_SP_DELAY_SEL_UNKNOWN,
  AS_ACA_SP_DELAY_SEL_NON,
  AS_ACA_SP_DELAY_SEL_SHORT,
  AS_ACA_SP_DELAY_SEL_MIDDLE,
  AS_ACA_SP_DELAY_SEL_LONG,
  AS_ACA_SP_DELAY_SEL_MAX_ENTRY
} as_aca_sp_delay_sel_id_t;

typedef enum
{
  AS_ACA_SP_DLY_FREE_UNKNOWN,
  AS_ACA_SP_DLY_FREE_OFF,
  AS_ACA_SP_DLY_FREE_ON,
  AS_ACA_SP_DLY_FREE_MAX_ENTRY
} as_aca_sp_dly_free_id_t;

typedef enum
{
  AS_ACA_SP_SPLITON_SEL_UNKNOWN,
  AS_ACA_SP_SPLITON_SEL_SHORTEST,
  AS_ACA_SP_SPLITON_SEL_SHORT,
  AS_ACA_SP_SPLITON_SEL_LONG,
  AS_ACA_SP_SPLITON_SEL_LONGEST,
  AS_ACA_SP_SPLITON_SEL_MAX_ENTRY
} as_aca_sp_spliton_sel_id_t;

typedef enum
{
  AS_ACA_SP_DRV_SEL_UNKNOWN,
  AS_ACA_SP_DRV_SEL_4DRIVER,
  AS_ACA_SP_DRV_SEL_2DRIVER,
  AS_ACA_SP_DRV_SEL_1DRIVER,
  AS_ACA_SP_DRV_SEL_LINEOUT,
  AS_ACA_SP_DRV_SEL_MAX_ENTRY
} as_aca_sp_drv_sel_id_t;

typedef enum
{
  AS_ACA_SER_MODE_UNKNOWN,
  AS_ACA_SER_MODE_8CH,         /* 8ch */
  AS_ACA_SER_MODE_4CH,         /* 4ch */
  AS_ACA_SER_MODE_MAX_ENTRY
} as_aca_pulco_ser_mode_id_t;

typedef enum
{
  AS_ACA_SER_FS_UNKNOWN,
  AS_ACA_SER_FS_128,           /* 128fs */
  AS_ACA_SER_FS_64,            /* 64fs */
  AS_ACA_SER_FS_MAX_ENTRY
} as_aca_pulco_ser_fs_id_t;

typedef enum
{
  AS_ACA_SER_SEL_FIX0 = 0,
  AS_ACA_SER_SEL_AMIC1 = 1,
  AS_ACA_SER_SEL_AMIC2 = 2,
  AS_ACA_SER_SEL_AMIC3 = 3,
  AS_ACA_SER_SEL_AMIC4 = 4,
  AS_ACA_SER_SEL_DMIC1 = 5,
  AS_ACA_SER_SEL_DMIC2 = 6,
  AS_ACA_SER_SEL_DMIC3 = 7,
  AS_ACA_SER_SEL_DMIC4 = 8,
  AS_ACA_SER_SEL_DMIC5 = 9,
  AS_ACA_SER_SEL_DMIC6 = 10,
  AS_ACA_SER_SEL_DMIC7 = 11,
  AS_ACA_SER_SEL_DMIC8 = 12,
  AS_ACA_SER_SEL_UNKNOWN = 15,
  AS_ACA_SER_SEL_MAX_ENTRY = 16
} as_aca_pulco_ser_sel_ch_id_t;

typedef enum
{
  AS_SDES_DES_SEL_UNKNOWN,
  AS_SDES_DES_SEL_CH1,
  AS_SDES_DES_SEL_CH2,
  AS_SDES_DES_SEL_CH3,
  AS_SDES_DES_SEL_CH4,
  AS_SDES_DES_SEL_CH5,
  AS_SDES_DES_SEL_CH6,
  AS_SDES_DES_SEL_CH7,
  AS_SDES_DES_SEL_CH8,
  AS_SDES_DES_SEL_MAX_ENTRY
} as_sdes_des_sel_out_id_t;

typedef enum
{
  AS_ACA_CHECK_ID,
  AS_ACA_POWER_ON_COMMON,
  AS_ACA_POWER_ON_INPUT,
  AS_ACA_POWER_ON_OUTPUT,
  AS_ACA_SET_SERDES,
  AS_ACA_SET_SMASTER,
  AS_ACA_POWER_OFF_COMMON,
  AS_ACA_POWER_OFF_INPUT,
  AS_ACA_POWER_OFF_OUTPUT,
  AS_ACA_POWER_ON_MICBIAS,
  AS_ACA_POWER_OFF_MICBIAS,
  AS_ACA_INIT_AMIC,
  AS_ACA_SET_AMIC_BOOT_DONE,
  AS_ACA_SET_OUTPUT_DEVICE,
  AS_ACA_GET_REGISTER,
  AS_ACA_SET_REGISTER,
  AS_ACA_CONTROL_TYPE_NUM
} as_aca_control_t;

typedef struct
{
  as_aca_pulco_osc_mode_id_t   osc_mode;
  as_aca_pulco_mic_device_id_t mic_dev;
  as_aca_pulco_io_ds_id_t      gpo_ds;
  as_aca_pulco_io_ds_id_t      ad_data_ds;
  as_aca_pulco_io_ds_id_t      dmic_clk_ds;
  as_aca_pulco_io_ds_id_t      mclk_ds;
} as_aca_pulco_param_t;

typedef struct
{
  as_aca_pulco_mic_device_id_t    mic_dev;
  as_aca_pulco_mic_bias_sel_id_t  mic_bias_sel;
  uint32_t                        mic_gain[4];
  uint32_t                        pga_gain[4];
  int32_t                         vgain[4];
} as_aca_pulco_in_param_t;

typedef struct
{
  as_smstr_mode_id_t         mode;
  as_smaster_mck_id_t        mck_fs;
  as_smaster_pwm_mode_id_t   pwm_mode;
  as_smaster_ch_sel_id_t     ch_sel;
  uint8_t                    out2dly;
} as_aca_pulco_smaster_param_t;

typedef struct
{
  as_aca_pulco_out_dev_id_t    out_dev;
  as_aca_pulco_pwm_out_id_t    pwm_out[2];
  as_aca_sp_delay_sel_id_t     sp_delay;
  as_aca_sp_loop_mode_id_t     loop_mode;
  as_smstr_mode_id_t           mode;
  as_aca_sp_dly_free_id_t      sp_dly_free;
  as_aca_sp_spliton_sel_id_t   sp_spliton;
  as_aca_sp_drv_sel_id_t       sp_drv;
} as_aca_pulco_out_param_t;

typedef struct
{
  uint32_t bank;
  uint32_t addr;
  uint32_t value;
} as_aca_pulco_reg_param_t;

typedef struct
{
  as_aca_pulco_ser_mode_id_t   ser_mode;
  as_aca_pulco_ser_fs_id_t     ser_fs;
  union
  {
    as_aca_pulco_ser_sel_ch_id_t  in[CXD56_AUDIO_MIC_CH_MAX];
    as_sdes_des_sel_out_id_t      out[CXD56_AUDIO_MIC_CH_MAX];
  } sel_ch;
} as_ser_des_param_t;

/* Select output device ID */

typedef enum
{
  /* output device none */

  AS_OUT_DEV_OFF,

  /* output device speaker */

  AS_OUT_DEV_SP,

  /* output device i2s */

  AS_OUT_DEV_I2S,
  AS_OUT_DEV_NUM
} as_out_dev_id_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern uint32_t fw_as_acacontrol(uint8_t type, uint32_t param);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void get_osc_mode(uint8_t cfg_mclk,
                         as_aca_pulco_osc_mode_id_t *osc)
{
  cxd56_audio_clkmode_t clk_mode = cxd56_audio_config_get_clkmode();

  if (cfg_mclk == CXD56_AUDIO_CFG_XTAL_24_576MHZ)
    {
      if (clk_mode == CXD56_AUDIO_CLKMODE_HIRES)
        {
          *osc = AS_ACA_OSC_24_576MHZ_HIRES;
        }
      else
        {
          *osc = AS_ACA_OSC_24_576MHZ;
        }
    }
  else
    {
      if (clk_mode == CXD56_AUDIO_CLKMODE_HIRES)
        {
          *osc = AS_ACA_OSC_49_152MHZ_HIRES;
        }
      else
        {
          *osc = AS_ACA_OSC_49_152MHZ;
        }
    }
}

static void get_mic_dev(uint32_t cfg_mic,
                        as_aca_pulco_mic_device_id_t *dev)
{
  bool is_amic = false;
  bool is_dmic = false;
  uint8_t i;

  for (i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
    {
      uint8_t mic_sel;
      mic_sel = (cfg_mic >> (i * MIC_CH_BITNUM)) & MIC_CH_BITMAP;
      if ((mic_sel >= 1) && (mic_sel <= 4))
        {
          is_amic = true;
        }
      else if ((mic_sel >= 5) && (mic_sel <= 12))
        {
          is_dmic = true;
        }
    }

  if (is_amic && is_dmic)
    {
      *dev = AS_ACA_MIC_BOTH;
    }
  else if(is_amic)
    {
      *dev = AS_ACA_MIC_AMIC;
    }
  else
    {
      *dev = AS_ACA_MIC_DMIC;
    }
}

static void get_drv_str(uint8_t cfg_ds,
                        as_aca_pulco_io_ds_id_t *ds)
{
  switch (cfg_ds)
    {
      case CXD56_AUDIO_CFG_DS_WEAKEST:
        *ds = AS_ACA_IO_DS_WEAKEST;
        break;

      case CXD56_AUDIO_CFG_DS_WEAKER:
        *ds = AS_ACA_IO_DS_WEAKER;
        break;

      case CXD56_AUDIO_CFG_DS_STRONGER:
        *ds = AS_ACA_IO_DS_STRONGER;
        break;

      case CXD56_AUDIO_CFG_DS_STRONGEST:
        *ds = AS_ACA_IO_DS_STRONGEST;
        break;

      default:
        *ds = AS_ACA_IO_DS_WEAKEST;
        break;
    }
}

static void get_mic_bias(uint8_t cfg_bsel,
                         as_aca_pulco_mic_bias_sel_id_t *bsel)
{
  switch (cfg_bsel)
    {
      case CXD56_AUDIO_CFG_MIC_BIAS_20V:
        *bsel = AS_ACA_MICBIAS_SEL_2_0V;
        break;

      case CXD56_AUDIO_CFG_MIC_BIAS_28V:
        *bsel = AS_ACA_MICBIAS_SEL_2_8V;
        break;

      default:
        *bsel = AS_ACA_MICBIAS_SEL_2_0V;
        break;
    }
}

static void get_sp_split_on(uint8_t cf_sp_spliton,
                            as_aca_sp_spliton_sel_id_t *sp_spliton)
{
  switch (cf_sp_spliton)
    {
      case CXD56_AUDIO_CFG_SP_SPLITON_LONGEST:
        *sp_spliton = AS_ACA_SP_SPLITON_SEL_LONGEST;
        break;

      case CXD56_AUDIO_CFG_SP_SPLITON_LONG:
        *sp_spliton = AS_ACA_SP_SPLITON_SEL_LONG;
        break;

      case CXD56_AUDIO_CFG_SP_SPLITON_SHORT:
        *sp_spliton = AS_ACA_SP_SPLITON_SEL_SHORT;
        break;

      default:
        *sp_spliton = AS_ACA_SP_SPLITON_SEL_SHORTEST;
        break;
    }
}

static void get_sp_driver(uint8_t cfg_sp_drv,
                          as_aca_sp_drv_sel_id_t *sp_drv)
{
  switch (cfg_sp_drv)
    {
      case CXD56_AUDIO_SP_DRV_LINEOUT:
        *sp_drv = AS_ACA_SP_DRV_SEL_LINEOUT;
        break;

      case CXD56_AUDIO_SP_DRV_1DRIVER:
        *sp_drv = AS_ACA_SP_DRV_SEL_1DRIVER;
        break;

      case CXD56_AUDIO_SP_DRV_2DRIVER:
        *sp_drv = AS_ACA_SP_DRV_SEL_2DRIVER;
        break;

      default:
        *sp_drv = AS_ACA_SP_DRV_SEL_4DRIVER;
        break;
    }
}

void get_pwon_param(as_aca_pulco_param_t *param)
{
  uint32_t mic_map = cxd56_audio_config_get_micmap();

  get_osc_mode((uint8_t)CXD56_AUDIO_CFG_MCLK,       &param->osc_mode);
  get_mic_dev(mic_map,                              &param->mic_dev);
  get_drv_str((uint8_t)CXD56_AUDIO_CFG_GPO_A_DS,    &param->gpo_ds);
  get_drv_str((uint8_t)CXD56_AUDIO_CFG_DA_DS,       &param->ad_data_ds);
  get_drv_str((uint8_t)CXD56_AUDIO_CFG_DMIC_CLK_DS, &param->dmic_clk_ds);
  get_drv_str((uint8_t)CXD56_AUDIO_CFG_MCLKOUT_DS,  &param->mclk_ds);
}

void get_serial_param(as_ser_des_param_t *param)
{
  uint8_t mic_mode = cxd56_audio_config_get_micmode();
  uint32_t mic_map = cxd56_audio_config_get_micmap();
  uint8_t mic_sel  = 0;
  uint8_t i;

  if (CXD56_AUDIO_CFG_MIC_MODE_128FS == mic_mode)
    {
      param->ser_mode = AS_ACA_SER_MODE_4CH;
      param->ser_fs   = AS_ACA_SER_FS_128;
    }
  else
    {
      param->ser_mode = AS_ACA_SER_MODE_8CH;
      param->ser_fs   = AS_ACA_SER_FS_64;
    }

  for (i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
    {
      mic_sel = (mic_map >> (i * MIC_CH_BITNUM)) & MIC_CH_BITMAP;
      param->sel_ch.in[i]  = (as_aca_pulco_ser_sel_ch_id_t)mic_sel;
    }
}

void get_input_param(as_aca_pulco_in_param_t *param,
                     cxd56_audio_mic_gain_t *gain)
{
  uint8_t mic_sel;
  uint8_t mic_id;
  uint32_t mic_map = cxd56_audio_config_get_micmap();
  uint32_t pga_gain;
  uint8_t i;

  memset((void *)param, 0, sizeof(as_aca_pulco_in_param_t));

  get_mic_dev(mic_map, &param->mic_dev);

  get_mic_bias((uint8_t)CXD56_AUDIO_CFG_MIC_BIAS, &param->mic_bias_sel);

  for (i = 0; i < CXD56_AUDIO_MIC_CH_MAX; i++)
    {
      mic_sel = (mic_map >> (i * MIC_CH_BITNUM)) & MIC_CH_BITMAP;
      if ((mic_sel >= 1) && (mic_sel <= 4))
        {
          mic_id = mic_sel - 1;
          param->mic_gain[mic_id] = (gain->gain[i] >= MIC_GAIN_MAX) ?
                                     MIC_GAIN_MAX :
                                    (gain->gain[i] / 30) * 30;

          pga_gain = gain->gain[i] - param->mic_gain[mic_id];
          param->pga_gain[mic_id] = (pga_gain >= PGA_GAIN_MAX) ?
                                     PGA_GAIN_MAX : pga_gain;
        }
    }
}

void get_smaster_param(as_aca_pulco_smaster_param_t *param)
{
  cxd56_audio_clkmode_t clk_mode = cxd56_audio_config_get_clkmode();

  if (clk_mode == CXD56_AUDIO_CLKMODE_HIRES)
    {
      param->mode  = AS_SMSTR_MODE_FS_32;
      param->mck_fs = AS_SMSTR_MCK_FS_1024;
    }
  else
    {
      param->mode  = AS_SMSTR_MODE_FS_16;
      param->mck_fs = AS_SMSTR_MCK_FS_512;
    }

  param->ch_sel   = AS_SMSTR_CHSEL_NORMAL;
  param->out2dly = 0x00;

  param->pwm_mode = AS_SMSTR_PWMMD_BOTH;
}

void get_pwon_out_param(as_aca_pulco_out_param_t *param)
{
  cxd56_audio_clkmode_t clk_mode = cxd56_audio_config_get_clkmode();

  if (clk_mode == CXD56_AUDIO_CLKMODE_HIRES)
    {
      param->mode = AS_SMSTR_MODE_FS_32;
    }
  else
    {
      param->mode = AS_SMSTR_MODE_FS_16;
    }

  param->out_dev    = AS_ACA_OUT_OFF;
  param->pwm_out[0] = AS_ACA_PWMOUT_UNKNOWN;
  param->pwm_out[1] = AS_ACA_PWMOUT_UNKNOWN;
  param->sp_delay   = AS_ACA_SP_DELAY_SEL_UNKNOWN;
  param->loop_mode  = AS_ACA_SP_LOOP_MODE_UNKNOWN;
  param->sp_dly_free = AS_ACA_SP_DLY_FREE_UNKNOWN;

  get_sp_split_on((uint8_t)CXD56_AUDIO_CFG_SP_SPLIT_ON, &param->sp_spliton);
  get_sp_driver((uint8_t)cxd56_audio_config_get_spdriver(), &param->sp_drv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_aca_poweron(void)
{
  if (fw_as_acacontrol(AS_ACA_CHECK_ID, 0) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_CHKID;
    }

  as_aca_pulco_param_t pwon_param;
  get_pwon_param(&pwon_param);

  if (fw_as_acacontrol(AS_ACA_POWER_ON_COMMON, (uint32_t)&pwon_param) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWON;
    }

  as_ser_des_param_t serial_param;
  get_serial_param(&serial_param);

  if (fw_as_acacontrol(AS_ACA_SET_SERDES, (uint32_t)&serial_param) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_SERIAL;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_poweroff(void)
{
  if (fw_as_acacontrol(AS_ACA_POWER_OFF_COMMON, 0) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWOFF;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_poweron_micbias(void)
{
  if (fw_as_acacontrol(AS_ACA_POWER_ON_MICBIAS, 0) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWON_MBIAS;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_poweron_input(
                                 cxd56_audio_mic_gain_t *gain)
{
  as_aca_pulco_in_param_t pwon_input_param;

  get_input_param(&pwon_input_param, gain);

  if (fw_as_acacontrol(AS_ACA_POWER_ON_INPUT,
     (uint32_t)&pwon_input_param) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWON_INPUT;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_set_smaster(void)
{
  as_aca_pulco_smaster_param_t smaster_param;

  get_smaster_param(&smaster_param);

  if (fw_as_acacontrol(AS_ACA_SET_SMASTER, (uint32_t)&smaster_param) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_SET_SMASTER;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_poweron_output(void)
{
  as_aca_pulco_out_param_t pwon_output_param;

  get_pwon_out_param(&pwon_output_param);

  if (fw_as_acacontrol(AS_ACA_POWER_ON_OUTPUT,
     (uint32_t)&pwon_output_param) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWON_OUTPUT;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_poweroff_input(void)
{
  if (fw_as_acacontrol(AS_ACA_POWER_OFF_INPUT, 0) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWOFF_INPUT;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_poweroff_output(void)
{
  if (fw_as_acacontrol(AS_ACA_POWER_OFF_OUTPUT, 0) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWOFF_OUTPUT;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_enable_output(void)
{
  if (fw_as_acacontrol(AS_ACA_SET_OUTPUT_DEVICE,
                       (uint32_t)AS_OUT_DEV_SP) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_ENABLE_OUTPUT;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_disable_output(void)
{
  if (fw_as_acacontrol(AS_ACA_SET_OUTPUT_DEVICE,
     (uint32_t)AS_OUT_DEV_OFF) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_DISABLE_OUTPUT;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_set_micgain(
                             cxd56_audio_mic_gain_t *gain)
{
  as_aca_pulco_in_param_t mic_gain_param;

  get_input_param(&mic_gain_param, gain);

  if (fw_as_acacontrol(AS_ACA_INIT_AMIC, (uint32_t)&mic_gain_param) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_SET_MICGAIN;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_notify_micbootdone(void)
{
  if (fw_as_acacontrol(AS_ACA_SET_AMIC_BOOT_DONE, 0) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_NOTIFY_MICBOOT;
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_read_reg(as_aca_pulco_reg_param_t *param)
{
  fw_as_acacontrol(AS_ACA_GET_REGISTER, (uint32_t)param);
  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_aca_write_reg(as_aca_pulco_reg_param_t *param)
{
  fw_as_acacontrol(AS_ACA_SET_REGISTER, (uint32_t)param);
  return CXD56_AUDIO_ECODE_OK;
}
