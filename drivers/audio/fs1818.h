/****************************************************************************
 * drivers/audio/fs1818.h
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

#ifndef __DRIVERS_AUDIO_FS1818U_H__
#define __DRIVERS_AUDIO_FS1818U_H__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FSM_DISABLE            0
#define FSM_ENABLE             1
#define FSM_MONO_MODE          1
#define MAGNIF_FACTOR          1024
#define R0_DEFAULT             (7 * MAGNIF_FACTOR)

#define FS1818_I2C_RETRY       5
#define FS1818_SLVAEADDR       0x34
#define FS1818_DEF_SPKERR      0x5052
#define FS1818_DEF_SPKM24      0xA0A4
#define FS1818_DEF_SPKM6       0xA55E
#define FS1818_DEF_SPKREV      0xAA60

#define FS1818_TEMP_ERR        130  /* tmax = 110 */
#define FS1818_TEMP_M24        104
#define FS1818_TEMP_M6         99
#define FS1818_TEMP_REC        93

#define FS1818_DEV_ID          0x06

#define R0_ALLOWANCE           30   /* 30% */
#define CALIB_MAGNIF_FACTOR    100
#define R0_ALLOWANCE_LOW       ((R0_DEFAULT * (100 - R0_ALLOWANCE)) / CALIB_MAGNIF_FACTOR)
#define R0_ALLOWANCE_HIGH      ((R0_DEFAULT * (100 + R0_ALLOWANCE)) / CALIB_MAGNIF_FACTOR)

#define CALIB_OTP_R25_STEP     ((int)(R0_ALLOWANCE * R0_DEFAULT) / 0x7F)
#define OTP_MIN_DELTA          (int)(R0_DEFAULT * 0.05f)

#define DATA_SIZE(arr)         (sizeof(arr)/sizeof(arr[0]))
#define FS1818_ZMDELTA_MAX     (0x150)
#define RS_TRIM_DEFAULT        (0x8F)
#define RS2RL_RATIO            (2300)
#define MAGNIF_TEMPR_COEF      (0xFFFF)
#define TEMPR_COEF             (0.0035)
#define EXT_TEMPERATURE        (25)
#define CALIB_MAX_USER_TRY     (13)

#define PRESET_MUSIC           (0)
#define PRESET_VOICE           (1)

#define AUD_SAMPRATE_8000       (8000)
#define AUD_SAMPRATE_16000      (16000)
#define AUD_SAMPRATE_32000      (32000)
#define AUD_SAMPRATE_44100      (44100)
#define AUD_SAMPRATE_48000      (48000)

#define AUD_BITS_16             (16)
#define AUD_BITS_24             (24)

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

enum fsm_pll_state
{
  FSM_PLL_OFF = 0,
  FSM_PLL_ON = 1,
};

struct audio_out_format
{
  uint16_t sample_rate;
  uint16_t bits;
  uint16_t channel_num;
};
typedef struct audio_out_format  audio_out_format_t;

enum fsm_wait_type
{
  FSM_WAIT_AMP_ON,
  FSM_WAIT_AMP_OFF,
  FSM_WAIT_AMP_ADC_OFF,
  FSM_WAIT_AMP_ADC_PLL_OFF,
  FSM_WAIT_TSIGNAL_ON,
  FSM_WAIT_TSIGNAL_OFF,
  FSM_WAIT_OTP_READY,
};

enum fsm_mute_type
{
  FSM_MUTE_UNKNOW = -1,
  FSM_UNMUTE = 0,
  FSM_MUTE = 1,
};

struct fsm_pll_config
{
  uint32_t bclk;
  uint16_t c1;
  uint16_t c2;
  uint16_t c3;
};
typedef struct fsm_pll_config fsm_pll_config_t;

struct fs1818u_dev_s
{
  struct audio_lowerhalf_s dev;

  /* Our specific driver data goes here */

  const FAR struct fs1818u_lower_s *lower;   /* Pointer to the board lower functions */
  FAR struct i2c_master_s *i2c;              /* I2C driver to use */

  uint16_t                samprate;         /* Configured samprate (samples/sec) */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  uint16_t                balance;          /* Current balance level (b16) */
#endif  /* CONFIG_AUDIO_EXCLUDE_BALANCE */
  uint8_t                 nor_volume;       /* Current volume level {0..63} */
  uint8_t                 dsp_volume;
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */
  uint8_t                 nchannels;        /* Number of channels (1 or 2) */
  uint8_t                 bpsamp;           /* Bits per sample (8 or 16) */
  uint32_t                bclk;             /* i2s blck */
  uint8_t                 scenario_mode;    /* scenario mode */
  bool                    dump_info;
  int                     mode;
  bool                    caliberate_done;
  int                     caliberate_result;
  bool                    store_cali_value;
  int                     caliberate_count;
};

#endif