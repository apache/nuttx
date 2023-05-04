/****************************************************************************
 * drivers/audio/esxxxx_common.h
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

#ifndef __DRIVERS_AUDIO_ESXXXX_COMMON_H
#define __DRIVERS_AUDIO_ESXXXX_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  ES_WORD_LENGTH_16BITS = 0x03,
  ES_WORD_LENGTH_18BITS = 0x02,
  ES_WORD_LENGTH_20BITS = 0x01,
  ES_WORD_LENGTH_24BITS = 0x00,
  ES_WORD_LENGTH_32BITS = 0x04
} es_word_length_e;

typedef enum
{
  ES_MCLK_DIV_AUTO,
  ES_MCLK_DIV_1,
  ES_MCLK_DIV_2,
  ES_MCLK_DIV_3,
  ES_MCLK_DIV_4,
  ES_MCLK_DIV_6,
  ES_MCLK_DIV_8,
  ES_MCLK_DIV_9,
  ES_MCLK_DIV_11,
  ES_MCLK_DIV_12,
  ES_MCLK_DIV_16,
  ES_MCLK_DIV_18,
  ES_MCLK_DIV_22,
  ES_MCLK_DIV_24,
  ES_MCLK_DIV_33,
  ES_MCLK_DIV_36,
  ES_MCLK_DIV_44,
  ES_MCLK_DIV_48,
  ES_MCLK_DIV_66,
  ES_MCLK_DIV_72,
  ES_MCLK_DIV_5,
  ES_MCLK_DIV_10,
  ES_MCLK_DIV_15,
  ES_MCLK_DIV_17,
  ES_MCLK_DIV_20,
  ES_MCLK_DIV_25,
  ES_MCLK_DIV_30,
  ES_MCLK_DIV_32,
  ES_MCLK_DIV_34,
  ES_MCLK_DIV_7,
  ES_MCLK_DIV_13,
  ES_MCLK_DIV_14
} es_sclk_div_e;

typedef enum
{
  ES_LCLK_DIV_128 = 0,
  ES_LCLK_DIV_192 = 1,
  ES_LCLK_DIV_256 = 2,
  ES_LCLK_DIV_384 = 3,
  ES_LCLK_DIV_512 = 4,
  ES_LCLK_DIV_576 = 5,
  ES_LCLK_DIV_768 = 6,
  ES_LCLK_DIV_1024 = 7,
  ES_LCLK_DIV_1152 = 8,
  ES_LCLK_DIV_1408 = 9,
  ES_LCLK_DIV_1536 = 10,
  ES_LCLK_DIV_2112 = 11,
  ES_LCLK_DIV_2304 = 12,
  ES_LCLK_DIV_125 = 16,
  ES_LCLK_DIV_136 = 17,
  ES_LCLK_DIV_250 = 18,
  ES_LCLK_DIV_272 = 19,
  ES_LCLK_DIV_375 = 20,
  ES_LCLK_DIV_500 = 21,
  ES_LCLK_DIV_544 = 22,
  ES_LCLK_DIV_750 = 23,
  ES_LCLK_DIV_1000 = 24,
  ES_LCLK_DIV_1088 = 25,
  ES_LCLK_DIV_1496 = 26,
  ES_LCLK_DIV_1500 = 27
} es_lclk_div_e;

typedef enum
{
  ES_D2SE_PGA_GAIN_DIS,
  ES_D2SE_PGA_GAIN_EN
} es_d2se_pga_e;

typedef enum
{
  ES_ADC_CHANNEL_LINPUT1_RINPUT1 = 0x00,
  ES_ADC_CHANNEL_MIC1 = 0x05,
  ES_ADC_CHANNEL_MIC2 = 0x06,
  ES_ADC_CHANNEL_LINPUT2_RINPUT2 = 0x50,
  ES_ADC_CHANNEL_DIFFERENCE = 0xf0
} es_adc_channel_e;

typedef enum
{
  ES_DAC_CHANNEL_LOUT1 = 0x04,
  ES_DAC_CHANNEL_LOUT2 = 0x08,
  ES_DAC_CHANNEL_SPK = 0x09,
  ES_DAC_CHANNEL_ROUT1 = 0x10,
  ES_DAC_CHANNEL_ROUT2 = 0x20,
  ES_DAC_CHANNEL_ALL = 0x3c
} es_dac_channel_e;

typedef enum
{
  ES_MIC_GAIN_0DB,
  ES_MIC_GAIN_3DB,
  ES_MIC_GAIN_6DB,
  ES_MIC_GAIN_9DB,
  ES_MIC_GAIN_12DB,
  ES_MIC_GAIN_15DB,
  ES_MIC_GAIN_18DB,
  ES_MIC_GAIN_21DB,
  ES_MIC_GAIN_24DB
} es_mic_gain_e;

typedef enum
{
  ES_MODULE_ADC = 1,
  ES_MODULE_DAC,
  ES_MODULE_ADC_DAC,
  ES_MODULE_LINE
} es_module_e;

typedef enum
{
  ES_MODE_SLAVE,
  ES_MODE_MASTER
} es_mode_e;

typedef enum
{
  ES_I2S_NORMAL,
  ES_I2S_LEFT,
  ES_I2S_RIGHT,
  ES_I2S_DSP
} es_i2s_fmt_e;

#endif /* __DRIVERS_AUDIO_ESXXXX_COMMON_H */
