/****************************************************************************
 * arch/arm/src/imxrt/imxrt105x_daisy.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: David Sidrane <david_s5@nscdg.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/* Based on chip selection this file is included in imxrt_daisy.c */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imxrt_daisy_t g_daisy_select[] =
{
  /* index:0 GPIO_EMC_00 */

  {
    {
      /* Index:0 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:0 Alt:1  GPIO EMC 00 FLEXPWM4 PWMA00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM4_PWMA0_OFFSET),
      [ALT1].sel   = 0,

      /* Index:0 Alt:2  GPIO EMC 00 LPSPI2 SCK */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI2_SCK_OFFSET),
      [ALT2].sel   = 1,

      /* Index:0 Alt:3  GPIO EMC 00 XBAR1 XBAR IN02 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN02_OFFSET),
      [ALT3].sel   = 0,

      /* Index:0 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:0 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:0 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:0 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:0 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:0 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:1 GPIO_EMC_01 */

  {
    {
      /* Index:1 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:1 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:1 Alt:2  GPIO EMC 01 LPSPI2 PCS0 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI2_PCS0_OFFSET),
      [ALT2].sel   = 1,

      /* Index:1 Alt:3  GPIO EMC 01 XBAR1 IN03 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN03_OFFSET),
      [ALT3].sel   = 0,

      /* Index:1 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:1 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:1 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:1 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:1 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:1 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:2 GPIO_EMC_02 */

  {
    {
      /* Index:2 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:2 Alt:1  GPIO EMC 02 FLEXPWM4 PWMA01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM4_PWMA1_OFFSET),
      [ALT1].sel   = 0,

      /* Index:2 Alt:2  GPIO EMC 02 LPSPI2 SDO */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI2_SDO_OFFSET),
      [ALT2].sel   = 1,

      /* Index:2 Alt:3  GPIO EMC 02 XBAR1 INOUT04 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN04_OFFSET),
      [ALT3].sel   = 0,

      /* Index:2 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:2 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:2 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:2 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:2 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:2 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:3 GPIO_EMC_03 */

  {
    {
      /* Index:3 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:3 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:3 Alt:2  GPIO EMC 03 LPSPI2 SDI */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI2_SDI_OFFSET),
      [ALT2].sel   = 1,

      /* Index:3 Alt:3  GPIO EMC 03 XBAR1 INOUT05 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN05_OFFSET),
      [ALT3].sel   = 0,

      /* Index:3 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:3 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:3 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:3 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:3 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:3 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:4 GPIO_EMC_04 */

  {
    {
      /* Index:4 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:4 Alt:1  GPIO EMC 04 FLEXPWM4 PWMA02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM4_PWMA2_OFFSET),
      [ALT1].sel   = 0,

      /* Index:4 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:4 Alt:3  GPIO EMC 04 XBAR1 INOUT06 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN06_OFFSET),
      [ALT3].sel   = 0,

      /* Index:4 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:4 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:4 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:4 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:4 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:4 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:5 GPIO_EMC_05 */

  {
    {
      /* Index:5 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:5 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:5 Alt:2  GPIO EMC 05 SAI2 TX SYNC */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_TX_SYNC_OFFSET),
      [ALT2].sel   = 0,

      /* Index:5 Alt:3  GPIO EMC 05 XBAR1 INOUT07 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN07_OFFSET),
      [ALT3].sel   = 0,

      /* Index:5 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:5 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:5 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:5 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:5 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:5 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:6 GPIO_EMC_06 */

  {
    {
      /* Index:6 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:6 Alt:1  GPIO EMC 06 FLEXPWM2 PWMA00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA0_OFFSET),
      [ALT1].sel   = 0,

      /* Index:6 Alt:2  GPIO EMC 06 SAI2 TX BCLK */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_TX_BCLK_OFFSET),
      [ALT2].sel   = 0,

      /* Index:6 Alt:3  GPIO EMC 06 XBAR1 INOUT08 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN08_OFFSET),
      [ALT3].sel   = 0,

      /* Index:6 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:6 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:6 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:6 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:6 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:6 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:7 GPIO_EMC_07 */

  {
    {
      /* Index:7 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:7 Alt:1  GPIO EMC 07 FLEXPWM2 PWMB00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB0_OFFSET),
      [ALT1].sel   = 0,

      /* Index:7 Alt:2  GPIO EMC 07 SAI2 MCLK */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_MCLK2_OFFSET),
      [ALT2].sel   = 0,

      /* Index:7 Alt:3  GPIO EMC 07 XBAR1 INOUT09 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN09_OFFSET),
      [ALT3].sel   = 0,

      /* Index:7 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:7 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:7 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:7 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:7 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:7 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:8 GPIO_EMC_08 */

  {
    {
      /* Index:8 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:8 Alt:1  GPIO EMC 08 FLEXPWM2 PWMA01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA1_OFFSET),
      [ALT1].sel   = 0,

      /* Index:8 Alt:2  GPIO EMC 08 SAI2 RX DATA */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_RX_DATA0_OFFSET),
      [ALT2].sel   = 0,

      /* Index:8 Alt:3  GPIO EMC 08 XBAR1 INOUT17 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN17_OFFSET),
      [ALT3].sel   = 0,

      /* Index:8 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:8 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:8 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:8 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:8 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:8 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:9 GPIO_EMC_09 */

  {
    {
      /* Index:9 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:9 Alt:1  GPIO EMC 09 FLEXPWM2 PWMB01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB1_OFFSET),
      [ALT1].sel   = 0,

      /* Index:9 Alt:2  GPIO EMC 09 SAI2 RX SYNC */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_RX_SYNC_OFFSET),
      [ALT2].sel   = 0,

      /* Index:9 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:9 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:9 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:9 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:9 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:9 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:9 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:10 GPIO_EMC_10 */

  {
    {
      /* Index:10 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:10 Alt:1  GPIO EMC 10 FLEXPWM2 PWMA02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA2_OFFSET),
      [ALT1].sel   = 0,

      /* Index:10 Alt:2  GPIO EMC 10 SAI2 RX BCLK */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_RX_BCLK_OFFSET),
      [ALT2].sel   = 0,

      /* Index:10 Alt:3  GPIO EMC 10 FLEXCAN2 RX */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXCAN2_RX_OFFSET),
      [ALT3].sel   = 0,

      /* Index:10 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:10 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:10 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:10 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:10 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:10 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:11 GPIO_EMC_11 */

  {
    {
      /* Index:11 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:11 Alt:1  GPIO EMC 11 FLEXPWM2 PWMB02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB2_OFFSET),
      [ALT1].sel   = 0,

      /* Index:11 Alt:2  GPIO EMC 11 LPI2C4 SDA */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C4_SDA_OFFSET),
      [ALT2].sel   = 0,

      /* Index:11 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:11 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:11 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:11 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:11 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:11 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:11 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:12 GPIO_EMC_12 */

  {
    {
      /* Index:12 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:12 Alt:1  GPIO EMC 12 XBAR1 IN24 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN24_OFFSET),
      [ALT1].sel   = 0,

      /* Index:12 Alt:2  GPIO EMC 12 LPI2C4 SCL */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C4_SCL_OFFSET),
      [ALT2].sel   = 0,

      /* Index:12 Alt:3  GPIO EMC 12 USDHC1 WP */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC1_WP_OFFSET),
      [ALT3].sel   = 0,

      /* Index:12 Alt:4  GPIO EMC 12 FLEXPWM1 PWMA03 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA3_OFFSET),
      [ALT4].sel   = 1,

      /* Index:12 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:12 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:12 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:12 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:12 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:13 GPIO_EMC_13 */

  {
    {
      /* Index:13 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:13 Alt:1  GPIO EMC 13 XBAR1 IN25 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN25_OFFSET),
      [ALT1].sel   = 1,

      /* Index:13 Alt:2  GPIO EMC 13 LPUART3 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART3_TX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:13 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:13 Alt:4  GPIO EMC 13 FLEXPWM1 PWMB03 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB3_OFFSET),
      [ALT4].sel   = 1,

      /* Index:13 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:13 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:13 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:13 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:13 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:14 GPIO_EMC_14 */

  {
    {
      /* Index:14 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:14 Alt:1  GPIO EMC 14 XBAR1 INOUT19 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN19_OFFSET),
      [ALT1].sel   = 0,

      /* Index:14 Alt:2  GPIO EMC 14 LPUART3 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART3_RX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:14 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:14 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:14 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:14 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:14 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:14 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:14 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:15 GPIO_EMC_15 */

  {
    {
      /* Index:15 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:15 Alt:1  GPIO EMC 15 XBAR1 IN20 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN20_OFFSET),
      [ALT1].sel   = 0,

      /* Index:15 Alt:2  GPIO EMC 15 LPUART3 CTS B */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART3_CTS_B_OFFSET),
      [ALT2].sel   = 0,

      /* Index:15 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:15 Alt:4  GPIO EMC 15 QTIMER3 TIMER0 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER0_OFFSET),
      [ALT4].sel   = 0,

      /* Index:15 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:15 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:15 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:15 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:15 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:16 GPIO_EMC_16 */

  {
    {
      /* Index:16 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:16 Alt:1  GPIO EMC 16 XBAR1 IN21 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN21_OFFSET),
      [ALT1].sel   = 0,

      /* Index:16 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:16 Alt:3  GPIO EMC 16 SPDIF IN */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SPDIF_IN_OFFSET),
      [ALT3].sel   = 1,

      /* Index:16 Alt:4  GPIO EMC 16 QTIMER3 TIMER1 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER1_OFFSET),
      [ALT4].sel   = 1,

      /* Index:16 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:16 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:16 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:16 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:16 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:17 GPIO_EMC_17 */

  {
    {
      /* Index:17 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:17 Alt:1  GPIO EMC 17 FLEXPWM4 PWMA03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM4_PWMA3_OFFSET),
      [ALT1].sel   = 0,

      /* Index:17 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:17 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:17 Alt:4  GPIO EMC 17 QTIMER3 TIMER2 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER2_OFFSET),
      [ALT4].sel   = 0,

      /* Index:17 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:17 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:17 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:17 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:17 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:18 GPIO_EMC_18 */

  {
    {
      /* Index:18 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:18 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:18 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:18 Alt:3  GPIO EMC 18 FLEXCAN1 RX */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXCAN1_RX_OFFSET),
      [ALT3].sel   = 1,

      /* Index:18 Alt:4  GPIO EMC 18 QTIMER3 TIMER3 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER3_OFFSET),
      [ALT4].sel   = 0,

      /* Index:18 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:18 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:18 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:18 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:18 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:19 GPIO_EMC_19 */

  {
    {
      /* Index:19 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:19 Alt:1  GPIO EMC 19 FLEXPWM2 PWMA03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA3_OFFSET),
      [ALT1].sel   = 1,

      /* Index:19 Alt:2  GPIO EMC 19 LPUART4 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART4_TX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:19 Alt:3  GPIO EMC 19 ENET RDATA01 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET1_RXDATA_OFFSET),
      [ALT3].sel   = 0,

      /* Index:19 Alt:4  GPIO EMC 19 QTIMER2 TIMER0 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER2_TIMER0_OFFSET),
      [ALT4].sel   = 0,

      /* Index:19 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:19 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:19 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:19 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:19 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:20 GPIO_EMC_20 */

  {
    {
      /* Index:20 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:20 Alt:1  GPIO EMC 20 FLEXPWM2 PWMB03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB3_OFFSET),
      [ALT1].sel   = 1,

      /* Index:20 Alt:2  GPIO EMC 20 LPUART4 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART4_RX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:20 Alt:3  GPIO EMC 20 ENET RDATA00 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET0_RXDATA_OFFSET),
      [ALT3].sel   = 0,

      /* Index:20 Alt:4  GPIO EMC 20 QTIMER2 TIMER1 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER2_TIMER1_OFFSET),
      [ALT4].sel   = 0,

      /* Index:20 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:20 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:20 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:20 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:20 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:21 GPIO_EMC_21 */

  {
    {
      /* Index:21 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:21 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:21 Alt:2  GPIO EMC 21 LPI2C3 SDA */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C3_SDA_OFFSET),
      [ALT2].sel   = 0,

      /* Index:21 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:21 Alt:4  GPIO EMC 21 QTIMER2 TIMER2 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER2_TIMER2_OFFSET),
      [ALT4].sel   = 0,

      /* Index:21 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:21 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:21 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:21 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:21 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:22 GPIO_EMC_22 */

  {
    {
      /* Index:22 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:22 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:22 Alt:2  GPIO EMC 22 LPI2C3 SCL */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C3_SCL_OFFSET),
      [ALT2].sel   = 0,

      /* Index:22 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:22 Alt:4  GPIO EMC 22 QTIMER2 TIMER3 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER2_TIMER3_OFFSET),
      [ALT4].sel   = 0,

      /* Index:22 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:22 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:22 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:22 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:22 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:23 GPIO_EMC_23 */

  {
    {
      /* Index:23 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:23 Alt:1  GPIO EMC 23 FLEXPWM1 PWMA00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA0_OFFSET),
      [ALT1].sel   = 0,

      /* Index:23 Alt:2  GPIO EMC 23 LPUART5 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART5_TX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:23 Alt:3  GPIO EMC 23 ENET RX EN */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_RXEN_OFFSET),
      [ALT3].sel   = 0,

      /* Index:23 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:23 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:23 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:23 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:23 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:23 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:24 GPIO_EMC_24 */

  {
    {
      /* Index:24 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:24 Alt:1  GPIO EMC 24 FLEXPWM1 PWMB00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB0_OFFSET),
      [ALT1].sel   = 0,

      /* Index:24 Alt:2  GPIO EMC 24 LPUART5 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART5_RX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:24 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:24 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:24 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:24 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:24 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:24 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:24 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:25 GPIO_EMC_25 */

  {
    {
      /* Index:25 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:25 Alt:1  GPIO EMC 25 FLEXPWM1 PWMA01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA1_OFFSET),
      [ALT1].sel   = 0,

      /* Index:25 Alt:2  GPIO EMC 25 LPUART6 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART6_TX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:25 Alt:3  GPIO EMC 25 ENET TX CLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_TXCLK_OFFSET),
      [ALT3].sel   = 0,

      /* Index:25 Alt:4  GPIO EMC 25 ENET REF CLK */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_IPG_CLK_RMII_OFFSET),
      [ALT4].sel   = 0,

      /* Index:25 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:25 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:25 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:25 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:25 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:26 GPIO_EMC_26 */

  {
    {
      /* Index:26 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:26 Alt:1  GPIO EMC 26 FLEXPWM1 PWMB01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB1_OFFSET),
      [ALT1].sel   = 0,

      /* Index:26 Alt:2  GPIO EMC 26 LPUART6 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART6_RX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:26 Alt:3  GPIO EMC 26 ENET RX ER */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_RXERR_OFFSET),
      [ALT3].sel   = 0,

      /* Index:26 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:26 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:26 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:26 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:26 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:26 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:27 GPIO_EMC_27 */

  {
    {
      /* Index:27 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:27 Alt:1  GPIO EMC 27 FLEXPWM1 PWMA02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA2_OFFSET),
      [ALT1].sel   = 0,

      /* Index:27 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:27 Alt:3  GPIO EMC 27 LPSPI1 SCK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI1_SCK_OFFSET),
      [ALT3].sel   = 0,

      /* Index:27 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:27 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:27 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:27 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:27 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:27 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:28 GPIO_EMC_28 */

  {
    {
      /* Index:28 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:28 Alt:1  GPIO EMC 28 FLEXPWM1 PWMB02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB2_OFFSET),
      [ALT1].sel   = 0,

      /* Index:28 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:28 Alt:3  GPIO EMC 28 LPSPI1 SDO */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI1_SDO_OFFSET),
      [ALT3].sel   = 0,

      /* Index:28 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:28 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:28 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:28 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:28 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:28 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:29 GPIO_EMC_29 */

  {
    {
      /* Index:29 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:29 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:29 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:29 Alt:3  GPIO EMC 29 LPSPI1 SDI */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI1_SDI_OFFSET),
      [ALT3].sel   = 0,

      /* Index:29 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:29 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:29 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:29 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:29 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:29 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:30 GPIO_EMC_30 */

  {
    {
      /* Index:30 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:30 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:30 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:30 Alt:3  GPIO EMC 30 LPSPI1 PCS0 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI1_PCS0_OFFSET),
      [ALT3].sel   = 1,

      /* Index:30 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:30 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:30 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:30 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:30 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:30 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:31 GPIO_EMC_31 */

  {
    {
      /* Index:31 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:31 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:31 Alt:2  GPIO EMC 31 LPUART7 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART7_TX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:31 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:31 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:31 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:31 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:31 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:31 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:31 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:32 GPIO_EMC_32 */

  {
    {
      /* Index:32 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:32 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:32 Alt:2  GPIO EMC 32 LPUART7 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART7_RX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:32 Alt:3  GPIO EMC 32 CCM PMIC RDY */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CCM_PMIC_READY_OFFSET),
      [ALT3].sel   = 4,

      /* Index:32 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:32 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:32 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:32 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:32 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:32 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:33 GPIO_EMC_33 */

  {
    {
      /* Index:33 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:33 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:33 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:33 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:33 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:33 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:33 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:33 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:33 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:33 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:34 GPIO_EMC_34 */

  {
    {
      /* Index:34 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:34 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:34 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:34 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:34 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:34 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:34 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:34 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:34 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:34 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:35 GPIO_EMC_35 */

  {
    {
      /* Index:35 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:35 Alt:1  GPIO EMC 35 XBAR1 INOUT18 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN18_OFFSET),
      [ALT1].sel   = 0,

      /* Index:35 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:35 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:35 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:35 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:35 Alt:6  GPIO EMC 35 USDHC1 CD B */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC1_CD_B_OFFSET),
      [ALT6].sel   = 0,

      /* Index:35 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:35 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:35 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:36 GPIO_EMC_36 */

  {
    {
      /* Index:36 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:36 Alt:1  GPIO EMC 36 XBAR1 IN22 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN22_OFFSET),
      [ALT1].sel   = 0,

      /* Index:36 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:36 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:36 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:36 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:36 Alt:6  GPIO EMC 36 USDHC1 WP */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC1_WP_OFFSET),
      [ALT6].sel   = 1,

      /* Index:36 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:36 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:36 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:37 GPIO_EMC_37 */

  {
    {
      /* Index:37 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:37 Alt:1  GPIO EMC 37 XBAR1 IN23 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN23_OFFSET),
      [ALT1].sel   = 0,

      /* Index:37 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:37 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:37 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:37 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:37 Alt:6  GPIO EMC 37 USDHC2 WP */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_WP_OFFSET),
      [ALT6].sel   = 0,

      /* Index:37 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:37 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:37 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:38 GPIO_EMC_38 */

  {
    {
      /* Index:38 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:38 Alt:1  GPIO EMC 38 FLEXPWM1 PWMA03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA3_OFFSET),
      [ALT1].sel   = 2,

      /* Index:38 Alt:2  GPIO EMC 38 LPUART8 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART8_TX_OFFSET),
      [ALT2].sel   = 2,

      /* Index:38 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:38 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:38 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:38 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:38 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:38 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:38 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:39 GPIO_EMC_39 */

  {
    {
      /* Index:39 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:39 Alt:1  GPIO EMC 39 FLEXPWM1 PWMB03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB3_OFFSET),
      [ALT1].sel   = 2,

      /* Index:39 Alt:2  GPIO EMC 39 LPUART8 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART8_RX_OFFSET),
      [ALT2].sel   = 2,

      /* Index:39 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:39 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:39 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:39 Alt:6  GPIO EMC 39 USDHC2 CD B */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_CD_B_OFFSET),
      [ALT6].sel   = 1,

      /* Index:39 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:39 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:39 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:40 GPIO_EMC_40 */

  {
    {
      /* Index:40 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:40 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:40 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:40 Alt:3  GPIO EMC 40 USB OTG2 OC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USB_OTG2_OC_OFFSET),
      [ALT3].sel   = 1,

      /* Index:40 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:40 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:40 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:40 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:40 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:40 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:41 GPIO_EMC_41 */

  {
    {
      /* Index:41 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:41 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:41 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:41 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:41 Alt:4  GPIO EMC 41 ENET MDIO */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_MDIO_OFFSET),
      [ALT4].sel   = 1,

      /* Index:41 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:41 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:41 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:41 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:41 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:42 GPIO_AD_B0_00 */

  {
    {
      /* Index:42 Alt:0  GPIO AD B0 00 FLEXPWM2 PWMA03 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA3_OFFSET),
      [ALT0].sel   = 2,

      /* Index:42 Alt:1  GPIO AD B0 00 XBAR1 INOUT14 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN14_OFFSET),
      [ALT1].sel   = 0,

      /* Index:42 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:42 Alt:3  GPIO AD B0 00 USB OTG2 ID */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ANATOP_USB_OTG2_ID_OFFSET),
      [ALT3].sel   = 0,

      /* Index:42 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:42 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:42 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:42 Alt:7  GPIO AD B0 00 LPSPI3 SCK */

      [ALT7].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI3_SCK_OFFSET),
      [ALT7].sel   = 0,

      /* Index:42 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:42 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:43 GPIO_AD_B0_01 */

  {
    {
      /* Index:43 Alt:0  GPIO AD B0 01 FLEXPWM2 PWMB03 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB3_OFFSET),
      [ALT0].sel   = 2,

      /* Index:43 Alt:1  GPIO AD B0 01 XBAR1 INOUT15 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN15_OFFSET),
      [ALT1].sel   = 0,

      /* Index:43 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:43 Alt:3  GPIO AD B0 01 USB OTG1 ID */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ANATOP_USB_OTG1_ID_OFFSET),
      [ALT3].sel   = 0,

      /* Index:43 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:43 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:43 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:43 Alt:7  GPIO AD B0 01 LPSPI3 SDO */

      [ALT7].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI3_SDO_OFFSET),
      [ALT7].sel   = 0,

      /* Index:43 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:43 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:44 GPIO_AD_B0_02 */

  {
    {
      /* Index:44 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:44 Alt:1  GPIO AD B0 02 XBAR1 INOUT16 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN16_OFFSET),
      [ALT1].sel   = 0,

      /* Index:44 Alt:2  GPIO AD B0 02 LPUART6 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART6_TX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:44 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:44 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:44 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:44 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:44 Alt:7  GPIO AD B0 02 LPSPI3 SDI */

      [ALT7].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI3_SDI_OFFSET),
      [ALT7].sel   = 0,

      /* Index:44 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:44 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:45 GPIO_AD_B0_03 */

  {
    {
      /* Index:45 Alt:0  GPIO AD B0 03 FLEXCAN2 RX */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXCAN2_RX_OFFSET),
      [ALT0].sel   = 1,

      /* Index:45 Alt:1  GPIO AD B0 03 XBAR1 INOUT17 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN17_OFFSET),
      [ALT1].sel   = 1,

      /* Index:45 Alt:2  GPIO AD B0 03 LPUART6 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART6_RX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:45 Alt:3  GPIO AD B0 03 USB OTG1 OC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USB_OTG1_OC_OFFSET),
      [ALT3].sel   = 0,

      /* Index:45 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:45 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:45 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:45 Alt:7  GPIO AD B0 03 LPSPI3 PCS0 */

      [ALT7].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI3_PCS0_OFFSET),
      [ALT7].sel   = 0,

      /* Index:45 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:45 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:46 GPIO_AD_B0_04 */

  {
    {
      /* Index:46 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:46 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:46 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:46 Alt:3  GPIO AD B0 04 SAI2 TX SYNC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_TX_SYNC_OFFSET),
      [ALT3].sel   = 1,

      /* Index:46 Alt:4  GPIO AD B0 04 CSI DATA09 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA09_OFFSET),
      [ALT4].sel   = 1,

      /* Index:46 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:46 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:46 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:46 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:46 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:47 GPIO_AD_B0_05 */

  {
    {
      /* Index:47 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:47 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:47 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:47 Alt:3  GPIO AD B0 05 SAI2 TX BCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_TX_BCLK_OFFSET),
      [ALT3].sel   = 1,

      /* Index:47 Alt:4  GPIO AD B0 05 CSI DATA08 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA08_OFFSET),
      [ALT4].sel   = 1,

      /* Index:47 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:47 Alt:6  GPIO AD B0 05 XBAR1 INOUT17 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN17_OFFSET),
      [ALT6].sel   = 2,

      /* Index:47 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:47 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:47 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:48 GPIO_AD_B0_06 */

  {
    {
      /* Index:48 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:48 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:48 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:48 Alt:3  GPIO AD B0 06 SAI2 RX BCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_RX_BCLK_OFFSET),
      [ALT3].sel   = 1,

      /* Index:48 Alt:4  GPIO AD B0 06 CSI DATA07 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA07_OFFSET),
      [ALT4].sel   = 1,

      /* Index:48 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:48 Alt:6  GPIO AD B0 06 XBAR1 INOUT18 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN18_OFFSET),
      [ALT6].sel   = 1,

      /* Index:48 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:48 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:48 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:49 GPIO_AD_B0_07 */

  {
    {
      /* Index:49 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:49 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:49 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:49 Alt:3  GPIO AD B0 07 SAI2 RX SYNC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_RX_SYNC_OFFSET),
      [ALT3].sel   = 1,

      /* Index:49 Alt:4  GPIO AD B0 07 CSI DATA06 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA06_OFFSET),
      [ALT4].sel   = 1,

      /* Index:49 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:49 Alt:6  GPIO AD B0 07 XBAR1 INOUT19 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN19_OFFSET),
      [ALT6].sel   = 1,

      /* Index:49 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:49 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:49 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:50 GPIO_AD_B0_08 */

  {
    {
      /* Index:50 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:50 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:50 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:50 Alt:3  GPIO AD B0 08 SAI2 RX DATA */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_RX_DATA0_OFFSET),
      [ALT3].sel   = 1,

      /* Index:50 Alt:4  GPIO AD B0 08 CSI DATA05 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA05_OFFSET),
      [ALT4].sel   = 1,

      /* Index:50 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:50 Alt:6  GPIO AD B0 08 XBAR1 IN20 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN20_OFFSET),
      [ALT6].sel   = 1,

      /* Index:50 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:50 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:50 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:51 GPIO_AD_B0_09 */

  {
    {
      /* Index:51 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:51 Alt:1  GPIO AD B0 09 FLEXPWM2 PWMA03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA3_OFFSET),
      [ALT1].sel   = 3,

      /* Index:51 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:51 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:51 Alt:4  GPIO AD B0 09 CSI DATA04 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA04_OFFSET),
      [ALT4].sel   = 1,

      /* Index:51 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:51 Alt:6  GPIO AD B0 09 XBAR1 IN21 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN21_OFFSET),
      [ALT6].sel   = 1,

      /* Index:51 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:51 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:51 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:52 GPIO_AD_B0_10 */

  {
    {
      /* Index:52 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:52 Alt:1  GPIO AD B0 10 FLEXPWM1 PWMA03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA3_OFFSET),
      [ALT1].sel   = 3,

      /* Index:52 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:52 Alt:3  GPIO AD B0 10 SAI2 MCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI2_MCLK2_OFFSET),
      [ALT3].sel   = 1,

      /* Index:52 Alt:4  GPIO AD B0 10 CSI DATA03 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA03_OFFSET),
      [ALT4].sel   = 1,

      /* Index:52 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:52 Alt:6  GPIO AD B0 10 XBAR1 IN22 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN22_OFFSET),
      [ALT6].sel   = 1,

      /* Index:52 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:52 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:52 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:53 GPIO_AD_B0_11 */

  {
    {
      /* Index:53 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:53 Alt:1  GPIO AD B0 11 FLEXPWM1 PWMB03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB3_OFFSET),
      [ALT1].sel   = 3,

      /* Index:53 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:53 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:53 Alt:4  GPIO AD B0 11 CSI DATA02 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA02_OFFSET),
      [ALT4].sel   = 1,

      /* Index:53 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:53 Alt:6  GPIO AD B0 11 XBAR1 IN23 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN23_OFFSET),
      [ALT6].sel   = 1,

      /* Index:53 Alt:7  GPIO AD B0 11 ENET 1588 EVENT0 IN */

      [ALT7].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET0_TIMER_OFFSET),
      [ALT7].sel   = 1,

      /* Index:53 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:53 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:54 GPIO_AD_B0_12 */

  {
    {
      /* Index:54 Alt:0  GPIO AD B0 12 LPI2C4 SCL */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C4_SCL_OFFSET),
      [ALT0].sel   = 1,

      /* Index:54 Alt:1  GPIO AD B0 12 CCM PMIC READY */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CCM_PMIC_READY_OFFSET),
      [ALT1].sel   = 1,

      /* Index:54 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:54 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:54 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:54 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:54 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:54 Alt:7  GPIO AD B0 12 NMI GLUE NMI */

      [ALT7].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_NMI_GLUE_NMI_OFFSET),
      [ALT7].sel   = 0,

      /* Index:54 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:54 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:55 GPIO_AD_B0_13 */

  {
    {
      /* Index:55 Alt:0  GPIO AD B0 13 LPI2C4 SDA */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C4_SDA_OFFSET),
      [ALT0].sel   = 1,

      /* Index:55 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:55 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:55 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:55 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:55 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:55 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:55 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:55 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:55 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:56 GPIO_AD_B0_14 */

  {
    {
      /* Index:56 Alt:0  GPIO AD B0 14 USB OTG2 OC */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USB_OTG2_OC_OFFSET),
      [ALT0].sel   = 0,

      /* Index:56 Alt:1  GPIO AD B0 14 XBAR1 IN24 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN24_OFFSET),
      [ALT1].sel   = 1,

      /* Index:56 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:56 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:56 Alt:4  GPIO AD B0 14 CSI VSYNC */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_VSYNC_OFFSET),
      [ALT4].sel   = 0,

      /* Index:56 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:56 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:56 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:56 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:56 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:57 GPIO_AD_B0_15 */

  {
    {
      /* Index:57 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:57 Alt:1  GPIO AD B0 15 XBAR1 IN25 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN25_OFFSET),
      [ALT1].sel   = 0,

      /* Index:57 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:57 Alt:3  GPIO AD B0 15 ENET 1588 EVENT0 IN */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET0_TIMER_OFFSET),
      [ALT3].sel   = 0,

      /* Index:57 Alt:4  GPIO AD B0 15 CSI HSYNC */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_HSYNC_OFFSET),
      [ALT4].sel   = 0,

      /* Index:57 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:57 Alt:6  GPIO AD B0 15 FLEXCAN2 RX */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXCAN2_RX_OFFSET),
      [ALT6].sel   = 2,

      /* Index:57 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:57 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:57 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:58 GPIO_AD_B1_00 */

  {
    {
      /* Index:58 Alt:0  GPIO AD B1 00 USB OTG2 ID */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ANATOP_USB_OTG2_ID_OFFSET),
      [ALT0].sel   = 1,

      /* Index:58 Alt:1  GPIO AD B1 00 QTIMER3 TIMER0 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER0_OFFSET),
      [ALT1].sel   = 1,

      /* Index:58 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:58 Alt:3  GPIO AD B1 00 LPI2C1 SCL */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C1_SCL_OFFSET),
      [ALT3].sel   = 1,

      /* Index:58 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:58 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:58 Alt:6  GPIO AD B1 00 USDHC1 WP */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC1_WP_OFFSET),
      [ALT6].sel   = 2,

      /* Index:58 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:58 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:58 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:59 GPIO_AD_B1_01 */

  {
    {
      /* Index:59 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:59 Alt:1  GPIO AD B1 01 QTIMER3 TIMER1 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER1_OFFSET),
      [ALT1].sel   = 0,

      /* Index:59 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:59 Alt:3  GPIO AD B1 01 LPI2C1 SDA */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C1_SDA_OFFSET),
      [ALT3].sel   = 1,

      /* Index:59 Alt:4  GPIO AD B1 01 CCM PMIC READY */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CCM_PMIC_READY_OFFSET),
      [ALT4].sel   = 2,

      /* Index:59 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:59 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:59 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:59 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:59 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:60 GPIO_AD_B1_02 */

  {
    {
      /* Index:60 Alt:0  GPIO AD B1 02 USB OTG1 ID */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ANATOP_USB_OTG1_ID_OFFSET),
      [ALT0].sel   = 1,

      /* Index:60 Alt:1  GPIO AD B1 02 QTIMER3 TIMER2 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER2_OFFSET),
      [ALT1].sel   = 1,

      /* Index:60 Alt:2  GPIO AD B1 02 LPUART2 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART2_TX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:60 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:60 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:60 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:60 Alt:6  GPIO AD B1 02 USDHC1 CD B */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC1_CD_B_OFFSET),
      [ALT6].sel   = 1,

      /* Index:60 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:60 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:60 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:61 GPIO_AD_B1_03 */

  {
    {
      /* Index:61 Alt:0  GPIO AD B1 03 USB OTG1 OC */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USB_OTG1_OC_OFFSET),
      [ALT0].sel   = 1,

      /* Index:61 Alt:1  GPIO AD B1 03 QTIMER3 TIMER3 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER3_OFFSET),
      [ALT1].sel   = 1,

      /* Index:61 Alt:2  GPIO AD B1 03 LPUART2 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART2_RX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:61 Alt:3  GPIO AD B1 03 SPDIF IN */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SPDIF_IN_OFFSET),
      [ALT3].sel   = 0,

      /* Index:61 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:61 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:61 Alt:6  GPIO AD B1 03 USDHC2 CD B */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_CD_B_OFFSET),
      [ALT6].sel   = 0,

      /* Index:61 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:61 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:61 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:62 GPIO_AD_B1_04 */

  {
    {
      /* Index:62 Alt:0  GPIO AD B1 04 FLEXSPIB DATA03 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIB_DATA3_OFFSET),
      [ALT0].sel   = 1,

      /* Index:62 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:62 Alt:2  GPIO AD B1 04 LPUART3 CTS B */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART3_CTS_B_OFFSET),
      [ALT2].sel   = 1,

      /* Index:62 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:62 Alt:4  GPIO AD B1 04 CSI PIXCLK */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_PIXCLK_OFFSET),
      [ALT4].sel   = 0,

      /* Index:62 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:62 Alt:6  GPIO AD B1 04 USDHC2 DATA0 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA0_OFFSET),
      [ALT6].sel   = 1,

      /* Index:62 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:62 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:62 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:63 GPIO_AD_B1_05 */

  {
    {
      /* Index:63 Alt:0  GPIO AD B1 05 FLEXSPIB DATA02 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIB_DATA2_OFFSET),
      [ALT0].sel   = 1,

      /* Index:63 Alt:1  GPIO AD B1 05 ENET MDIO */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_MDIO_OFFSET),
      [ALT1].sel   = 0,

      /* Index:63 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:63 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:63 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:63 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:63 Alt:6  GPIO AD B1 05 USDHC2 DATA1 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA1_OFFSET),
      [ALT6].sel   = 1,

      /* Index:63 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:63 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:63 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:64 GPIO_AD_B1_06 */

  {
    {
      /* Index:64 Alt:0  GPIO AD B1 06 FLEXSPIB DATA01 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIB_DATA1_OFFSET),
      [ALT0].sel   = 1,

      /* Index:64 Alt:1  GPIO AD B1 06 LPI2C3 SDA */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C3_SDA_OFFSET),
      [ALT1].sel   = 2,

      /* Index:64 Alt:2  GPIO AD B1 06 LPUART3 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART3_TX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:64 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:64 Alt:4  GPIO AD B1 06 CSI VSYNC */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_VSYNC_OFFSET),
      [ALT4].sel   = 1,

      /* Index:64 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:64 Alt:6  GPIO AD B1 06 USDHC2 DATA2 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA2_OFFSET),
      [ALT6].sel   = 1,

      /* Index:64 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:64 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:64 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:65 GPIO_AD_B1_07 */

  {
    {
      /* Index:65 Alt:0  GPIO AD B1 07 FLEXSPIB DATA00 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIB_DATA0_OFFSET),
      [ALT0].sel   = 1,

      /* Index:65 Alt:1  GPIO AD B1 07 LPI2C3 SCL */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C3_SCL_OFFSET),
      [ALT1].sel   = 2,

      /* Index:65 Alt:2  GPIO AD B1 07 LPUART3 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART3_RX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:65 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:65 Alt:4  GPIO AD B1 07 CSI HSYNC */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_HSYNC_OFFSET),
      [ALT4].sel   = 1,

      /* Index:65 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:65 Alt:6  GPIO AD B1 07 USDHC2 DATA3 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA3_OFFSET),
      [ALT6].sel   = 1,

      /* Index:65 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:65 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:65 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:66 GPIO_AD_B1_08 */

  {
    {
      /* Index:66 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:66 Alt:1  GPIO AD B1 08 FLEXPWM4 PWMA00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM4_PWMA0_OFFSET),
      [ALT1].sel   = 1,

      /* Index:66 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:66 Alt:3  GPIO AD B1 08 CCM PMIC READY */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CCM_PMIC_READY_OFFSET),
      [ALT3].sel   = 3,

      /* Index:66 Alt:4  GPIO AD B1 08 CSI DATA09 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA09_OFFSET),
      [ALT4].sel   = 0,

      /* Index:66 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:66 Alt:6  GPIO AD B1 08 USDHC2 CMD */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_CMD_OFFSET),
      [ALT6].sel   = 1,

      /* Index:66 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:66 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:66 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:67 GPIO_AD_B1_09 */

  {
    {
      /* Index:67 Alt:0  GPIO AD B1 09 FLEXSPIA DQS */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DQS_OFFSET),
      [ALT0].sel   = 1,

      /* Index:67 Alt:1  GPIO AD B1 09 FLEXPWM4 PWMA01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM4_PWMA1_OFFSET),
      [ALT1].sel   = 1,

      /* Index:67 Alt:2  GPIO AD B1 09 FLEXCAN1 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXCAN1_RX_OFFSET),
      [ALT2].sel   = 2,

      /* Index:67 Alt:3  GPIO AD B1 09 SAI1 MCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_MCLK2_OFFSET),
      [ALT3].sel   = 1,

      /* Index:67 Alt:4  GPIO AD B1 09 CSI DATA08 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA08_OFFSET),
      [ALT4].sel   = 0,

      /* Index:67 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:67 Alt:6  GPIO AD B1 09 USDHC2 CLK */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_CLK_OFFSET),
      [ALT6].sel   = 1,

      /* Index:67 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:67 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:67 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:68 GPIO_AD_B1_10 */

  {
    {
      /* Index:68 Alt:0  GPIO AD B1 10 FLEXSPIA DATA03 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DATA3_OFFSET),
      [ALT0].sel   = 1,

      /* Index:68 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:68 Alt:2  GPIO AD B1 10 LPUART8 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART8_TX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:68 Alt:3  GPIO AD B1 10 SAI1 RX SYNC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_SYNC_OFFSET),
      [ALT3].sel   = 1,

      /* Index:68 Alt:4  GPIO AD B1 10 CSI DATA07 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA07_OFFSET),
      [ALT4].sel   = 0,

      /* Index:68 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:68 Alt:6  GPIO AD B1 10 USDHC2 WP */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_WP_OFFSET),
      [ALT6].sel   = 1,

      /* Index:68 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:68 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:68 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:69 GPIO_AD_B1_11 */

  {
    {
      /* Index:69 Alt:0  GPIO AD B1 11 FLEXSPIA DATA02 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DATA2_OFFSET),
      [ALT0].sel   = 1,

      /* Index:69 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:69 Alt:2  GPIO AD B1 11 LPUART8 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART8_RX_OFFSET),
      [ALT2].sel   = 1,

      /* Index:69 Alt:3  GPIO AD B1 11 SAI1 RX BCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_BCLK_OFFSET),
      [ALT3].sel   = 1,

      /* Index:69 Alt:4  GPIO AD B1 11 CSI DATA06 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA06_OFFSET),
      [ALT4].sel   = 0,

      /* Index:69 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:69 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:69 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:69 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:69 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:70 GPIO_AD_B1_12 */

  {
    {
      /* Index:70 Alt:0  GPIO AD B1 12 FLEXSPIA DATA01 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DATA1_OFFSET),
      [ALT0].sel   = 1,

      /* Index:70 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:70 Alt:2  GPIO AD B1 12 LPSPI3 PCS0 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI3_PCS0_OFFSET),
      [ALT2].sel   = 1,

      /* Index:70 Alt:3  GPIO AD B1 12 SAI1 RX DATA00 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_DATA0_OFFSET),
      [ALT3].sel   = 1,

      /* Index:70 Alt:4  GPIO AD B1 12 CSI DATA05 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA05_OFFSET),
      [ALT4].sel   = 0,

      /* Index:70 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:70 Alt:6  GPIO AD B1 12 USDHC2 DATA4 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA4_OFFSET),
      [ALT6].sel   = 1,

      /* Index:70 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:70 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:70 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:71 GPIO_AD_B1_13 */

  {
    {
      /* Index:71 Alt:0  GPIO AD B1 13 FLEXSPIA DATA00 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DATA0_OFFSET),
      [ALT0].sel   = 1,

      /* Index:71 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:71 Alt:2  GPIO AD B1 13 LPSPI3 SDI */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI3_SDI_OFFSET),
      [ALT2].sel   = 1,

      /* Index:71 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:71 Alt:4  GPIO AD B1 13 CSI DATA04 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA04_OFFSET),
      [ALT4].sel   = 0,

      /* Index:71 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:71 Alt:6  GPIO AD B1 13 USDHC2 DATA5 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA5_OFFSET),
      [ALT6].sel   = 1,

      /* Index:71 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:71 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:71 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:72 GPIO_AD_B1_14 */

  {
    {
      /* Index:72 Alt:0  GPIO AD B1 14 FLEXSPIA SCLK */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_SCK_OFFSET),
      [ALT0].sel   = 1,

      /* Index:72 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:72 Alt:2  GPIO AD B1 14 LPSPI3 SDO */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI3_SDO_OFFSET),
      [ALT2].sel   = 1,

      /* Index:72 Alt:3  GPIO AD B1 14 SAI1 TX BCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_TX_BCLK_OFFSET),
      [ALT3].sel   = 1,

      /* Index:72 Alt:4  GPIO AD B1 14 CSI DATA03 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA03_OFFSET),
      [ALT4].sel   = 0,

      /* Index:72 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:72 Alt:6  GPIO AD B1 14 USDHC2 DATA6 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA6_OFFSET),
      [ALT6].sel   = 1,

      /* Index:72 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:72 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:72 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:73 GPIO_AD_B1_15 */

  {
    {
      /* Index:73 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:73 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:73 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:73 Alt:3  GPIO AD B1 15 SAI1 TX SYNC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_TX_SYNC_OFFSET),
      [ALT3].sel   = 1,

      /* Index:73 Alt:4  GPIO AD B1 15 CSI DATA02 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_DATA02_OFFSET),
      [ALT4].sel   = 0,

      /* Index:73 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:73 Alt:6  GPIO AD B1 15 USDHC2 DATA7 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA7_OFFSET),
      [ALT6].sel   = 1,

      /* Index:73 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:73 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:73 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:74 GPIO_B0_00 */

  {
    {
      /* Index:74 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:74 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:74 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:74 Alt:3  GPIO B0 00 LPSPI4 PCS0 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI4_PCS0_OFFSET),
      [ALT3].sel   = 0,

      /* Index:74 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:74 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:74 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:74 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:74 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:74 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:75 GPIO_B0_01 */

  {
    {
      /* Index:75 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:75 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:75 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:75 Alt:3  GPIO B0 01 LPSPI4 SDI */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI4_SDI_OFFSET),
      [ALT3].sel   = 0,

      /* Index:75 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:75 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:75 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:75 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:75 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:75 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:76 GPIO_B0_02 */

  {
    {
      /* Index:76 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:76 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:76 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:76 Alt:3  GPIO B0 02 LPSPI4 SDO */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI4_SDO_OFFSET),
      [ALT3].sel   = 0,

      /* Index:76 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:76 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:76 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:76 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:76 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:76 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:77 GPIO_B0_03 */

  {
    {
      /* Index:77 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:77 Alt:1  GPIO B0 03 QTIMER2 TIMER0 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER2_TIMER0_OFFSET),
      [ALT1].sel   = 1,

      /* Index:77 Alt:2  GPIO B0 03 FLEXCAN1 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXCAN1_RX_OFFSET),
      [ALT2].sel   = 3,

      /* Index:77 Alt:3  GPIO B0 03 LPSPI4 SCK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI4_SCK_OFFSET),
      [ALT3].sel   = 0,

      /* Index:77 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:77 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:77 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:77 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:77 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:77 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:78 GPIO_B0_04 */

  {
    {
      /* Index:78 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:78 Alt:1  GPIO B0 04 QTIMER2 TIMER1 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER2_TIMER1_OFFSET),
      [ALT1].sel   = 1,

      /* Index:78 Alt:2  GPIO B0 04 LPI2C2 SCL */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C2_SCL_OFFSET),
      [ALT2].sel   = 1,

      /* Index:78 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:78 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:78 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:78 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:78 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:78 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:78 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:79 GPIO_B0_05 */

  {
    {
      /* Index:79 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:79 Alt:1  GPIO B0 05 QTIMER2 TIMER2 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER2_TIMER2_OFFSET),
      [ALT1].sel   = 1,

      /* Index:79 Alt:2  GPIO B0 05 LPI2C2 SDA */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C2_SDA_OFFSET),
      [ALT2].sel   = 1,

      /* Index:79 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:79 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:79 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:79 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:79 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:79 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:79 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:80 GPIO_B0_06 */

  {
    {
      /* Index:80 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:80 Alt:1  GPIO B0 06 QTIMER3 TIMER0 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER0_OFFSET),
      [ALT1].sel   = 2,

      /* Index:80 Alt:2  GPIO B0 06 FLEXPWM2 PWMA00 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA0_OFFSET),
      [ALT2].sel   = 1,

      /* Index:80 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:80 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:80 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:80 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:80 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:80 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:80 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:81 GPIO_B0_07 */

  {
    {
      /* Index:81 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:81 Alt:1  GPIO B0 07 QTIMER3 TIMER1 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER1_OFFSET),
      [ALT1].sel   = 2,

      /* Index:81 Alt:2  GPIO B0 07 FLEXPWM2 PWMB00 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB0_OFFSET),
      [ALT2].sel   = 1,

      /* Index:81 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:81 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:81 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:81 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:81 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:81 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:81 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:82 GPIO_B0_08 */

  {
    {
      /* Index:82 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:82 Alt:1  GPIO B0 08 QTIMER3 TIMER2 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER2_OFFSET),
      [ALT1].sel   = 2,

      /* Index:82 Alt:2  GPIO B0 08 FLEXPWM2 PWMA01 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA1_OFFSET),
      [ALT2].sel   = 1,

      /* Index:82 Alt:3  GPIO B0 08 LPUART3 TX */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART3_TX_OFFSET),
      [ALT3].sel   = 2,

      /* Index:82 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:82 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:82 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:82 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:82 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:82 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:83 GPIO_B0_09 */

  {
    {
      /* Index:83 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:83 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:83 Alt:2  GPIO B0 09 FLEXPWM2 PWMB01 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB1_OFFSET),
      [ALT2].sel   = 1,

      /* Index:83 Alt:3  GPIO B0 09 LPUART3 RX */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART3_RX_OFFSET),
      [ALT3].sel   = 2,

      /* Index:83 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:83 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:83 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:83 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:83 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:83 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:84 GPIO_B0_10 */

  {
    {
      /* Index:84 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:84 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:84 Alt:2  GPIO B0 10 FLEXPWM2 PWMA02 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA2_OFFSET),
      [ALT2].sel   = 1,

      /* Index:84 Alt:3  GPIO B0 10 SAI1 TX DATA03 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_DATA1_OFFSET),
      [ALT3].sel   = 1,

      /* Index:84 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:84 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:84 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:84 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:84 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:84 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:85 GPIO_B0_11 */

  {
    {
      /* Index:85 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:85 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:85 Alt:2  GPIO B0 11 FLEXPWM2 PWMB02 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB2_OFFSET),
      [ALT2].sel   = 1,

      /* Index:85 Alt:3  GPIO B0 11 SAI1 TX DATA02 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_DATA2_OFFSET),
      [ALT3].sel   = 1,

      /* Index:85 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:85 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:85 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:85 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:85 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:85 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:86 GPIO_B0_12 */

  {
    {
      /* Index:86 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:86 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:86 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:86 Alt:3  GPIO B0 12 SAI1 TX DATA01 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_DATA3_OFFSET),
      [ALT3].sel   = 1,

      /* Index:86 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:86 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:86 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:86 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:86 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:86 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:87 GPIO_B0_13 */

  {
    {
      /* Index:87 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:87 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:87 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:87 Alt:3  GPIO B0 13 SAI1 MCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_MCLK2_OFFSET),
      [ALT3].sel   = 2,

      /* Index:87 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:87 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:87 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:87 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:87 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:87 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:88 GPIO_B0_14 */

  {
    {
      /* Index:88 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:88 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:88 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:88 Alt:3  GPIO B0 14 SAI1 RX SYNC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_SYNC_OFFSET),
      [ALT3].sel   = 2,

      /* Index:88 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:88 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:88 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:88 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:88 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:88 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:89 GPIO_B0_15 */

  {
    {
      /* Index:89 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:89 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:89 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:89 Alt:3  GPIO B0 15 SAI1 RX BCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_BCLK_OFFSET),
      [ALT3].sel   = 2,

      /* Index:89 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:89 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:89 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:89 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:89 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:89 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:90 GPIO_B1_00 */

  {
    {
      /* Index:90 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:90 Alt:1  GPIO B1 00 XBAR1 INOUT14 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN14_OFFSET),
      [ALT1].sel   = 1,

      /* Index:90 Alt:2  GPIO B1 00 LPUART4 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART4_TX_OFFSET),
      [ALT2].sel   = 2,

      /* Index:90 Alt:3  GPIO B1 00 SAI1 RX DATA00 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_DATA0_OFFSET),
      [ALT3].sel   = 2,

      /* Index:90 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:90 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:90 Alt:6  GPIO B1 00 FLEXPWM1 PWMA03 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA3_OFFSET),
      [ALT6].sel   = 4,

      /* Index:90 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:90 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:90 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:91 GPIO_B1_01 */

  {
    {
      /* Index:91 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:91 Alt:1  GPIO B1 01 XBAR1 INOUT15 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN15_OFFSET),
      [ALT1].sel   = 1,

      /* Index:91 Alt:2  GPIO B1 01 LPUART4 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART4_RX_OFFSET),
      [ALT2].sel   = 2,

      /* Index:91 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:91 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:91 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:91 Alt:6  GPIO B1 01 FLEXPWM1 PWMB03 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB3_OFFSET),
      [ALT6].sel   = 4,

      /* Index:91 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:91 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:91 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:92 GPIO_B1_02 */

  {
    {
      /* Index:92 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:92 Alt:1  GPIO B1 02 XBAR1 INOUT16 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN16_OFFSET),
      [ALT1].sel   = 1,

      /* Index:92 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:92 Alt:3  GPIO B1 02 SAI1 TX BCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_TX_BCLK_OFFSET),
      [ALT3].sel   = 2,

      /* Index:92 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:92 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:92 Alt:6  GPIO B1 02 FLEXPWM2 PWMA03 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA3_OFFSET),
      [ALT6].sel   = 4,

      /* Index:92 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:92 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:92 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:93 GPIO_B1_03 */

  {
    {
      /* Index:93 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:93 Alt:1  GPIO B1 03 XBAR1 INOUT17 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN17_OFFSET),
      [ALT1].sel   = 3,

      /* Index:93 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:93 Alt:3  GPIO B1 03 SAI1 TX SYNC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_TX_SYNC_OFFSET),
      [ALT3].sel   = 2,

      /* Index:93 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:93 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:93 Alt:6  GPIO B1 03 FLEXPWM2 PWMB03 */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB3_OFFSET),
      [ALT6].sel   = 3,

      /* Index:93 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:93 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:93 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:94 GPIO_B1_04 */

  {
    {
      /* Index:94 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:94 Alt:1  GPIO B1 04 LPSPI4 PCS0 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI4_PCS0_OFFSET),
      [ALT1].sel   = 1,

      /* Index:94 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:94 Alt:3  GPIO B1 04 ENET RX DATA00 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET0_RXDATA_OFFSET),
      [ALT3].sel   = 1,

      /* Index:94 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:94 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:94 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:94 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:94 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:94 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:95 GPIO_B1_05 */

  {
    {
      /* Index:95 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:95 Alt:1  GPIO B1 05 LPSPI4 SDI */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI4_SDI_OFFSET),
      [ALT1].sel   = 1,

      /* Index:95 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:95 Alt:3  GPIO B1 05 ENET RX DATA01 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET1_RXDATA_OFFSET),
      [ALT3].sel   = 1,

      /* Index:95 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:95 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:95 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:95 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:95 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:95 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:96 GPIO_B1_06 */

  {
    {
      /* Index:96 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:96 Alt:1  GPIO B1 06 LPSPI4 SDO */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI4_SDO_OFFSET),
      [ALT1].sel   = 1,

      /* Index:96 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:96 Alt:3  GPIO B1 06 ENET RX EN */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_RXEN_OFFSET),
      [ALT3].sel   = 1,

      /* Index:96 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:96 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:96 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:96 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:96 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:96 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:97 GPIO_B1_07 */

  {
    {
      /* Index:97 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:97 Alt:1  GPIO B1 07 LPSPI4 SCK */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI4_SCK_OFFSET),
      [ALT1].sel   = 1,

      /* Index:97 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:97 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:97 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:97 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:97 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:97 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:97 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:97 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:98 GPIO_B1_08 */

  {
    {
      /* Index:98 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:98 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:98 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:98 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:98 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:98 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:98 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:98 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:98 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:98 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:99 GPIO_B1_09 */

  {
    {
      /* Index:99 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:99 Alt:1  GPIO B1 09 QTIMER2 TIMER3 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER2_TIMER3_OFFSET),
      [ALT1].sel   = 1,

      /* Index:99 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:99 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:99 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:99 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:99 Alt:6  GPIO B1 09 FLEXCAN2 RX */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXCAN2_RX_OFFSET),
      [ALT6].sel   = 3,

      /* Index:99 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:99 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:99 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:100 GPIO_B1_10 */

  {
    {
      /* Index:100 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:100 Alt:1  GPIO B1 10 QTIMER3 TIMER3 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_QTIMER3_TIMER3_OFFSET),
      [ALT1].sel   = 2,

      /* Index:100 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:100 Alt:3  GPIO B1 10 ENET TX CLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_TXCLK_OFFSET),
      [ALT3].sel   = 1,

      /* Index:100 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:100 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:100 Alt:6  GPIO B1 10 ENET REF CLK */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_IPG_CLK_RMII_OFFSET),
      [ALT6].sel   = 1,

      /* Index:100 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:100 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:100 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:101 GPIO_B1_11 */

  {
    {
      /* Index:101 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:101 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:101 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:101 Alt:3  GPIO B1 11 ENET RX ER */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_RXERR_OFFSET),
      [ALT3].sel   = 1,

      /* Index:101 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:101 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:101 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:101 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:101 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:101 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:102 GPIO_B1_12 */

  {
    {
      /* Index:102 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:102 Alt:1  GPIO B1 12 LPUART5 TX */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART5_TX_OFFSET),
      [ALT1].sel   = 1,

      /* Index:102 Alt:2  GPIO B1 12 CSI PIXCLK */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_PIXCLK_OFFSET),
      [ALT2].sel   = 1,

      /* Index:102 Alt:3  GPIO B1 12 ENET 1588 EVENT0 IN */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET0_TIMER_OFFSET),
      [ALT3].sel   = 2,

      /* Index:102 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:102 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:102 Alt:6  GPIO B1 12 USDHC1 CD B */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC1_CD_B_OFFSET),
      [ALT6].sel   = 2,

      /* Index:102 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:102 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:102 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:103 GPIO_B1_13 */

  {
    {
      /* Index:103 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:103 Alt:1  GPIO B1 13 LPUART5 RX */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART5_RX_OFFSET),
      [ALT1].sel   = 1,

      /* Index:103 Alt:2  GPIO B1 13 CSI VSYNC */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_VSYNC_OFFSET),
      [ALT2].sel   = 2,

      /* Index:103 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:103 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:103 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:103 Alt:6  GPIO B1 13 USDHC1 WP */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC1_WP_OFFSET),
      [ALT6].sel   = 3,

      /* Index:103 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:103 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:103 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:104 GPIO_B1_14 */

  {
    {
      /* Index:104 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:104 Alt:1  GPIO B1 14 FLEXPWM4 PWMA02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM4_PWMA2_OFFSET),
      [ALT1].sel   = 1,

      /* Index:104 Alt:2  GPIO B1 14 CSI HSYNC */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CSI_HSYNC_OFFSET),
      [ALT2].sel   = 2,

      /* Index:104 Alt:3  GPIO B1 14 XBAR1 IN02 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN02_OFFSET),
      [ALT3].sel   = 1,

      /* Index:104 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:104 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:104 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:104 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:104 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:104 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:105 GPIO_B1_15 */

  {
    {
      /* Index:105 Alt:0  GPIO B1 15 ENET MDIO */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_ENET_MDIO_OFFSET),
      [ALT0].sel   = 2,

      /* Index:105 Alt:1  GPIO B1 15 FLEXPWM4 PWMA03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM4_PWMA3_OFFSET),
      [ALT1].sel   = 1,

      /* Index:105 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:105 Alt:3  GPIO B1 15 XBAR1 IN03 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN03_OFFSET),
      [ALT3].sel   = 1,

      /* Index:105 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:105 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:105 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:105 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:105 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:105 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:106 GPIO_SD_B0_00 */

  {
    {
      /* Index:106 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:106 Alt:1  GPIO SD B0 00 FLEXPWM1 PWMA00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA0_OFFSET),
      [ALT1].sel   = 1,

      /* Index:106 Alt:2  GPIO SD B0 00 LPI2C3 SCL */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C3_SCL_OFFSET),
      [ALT2].sel   = 1,

      /* Index:106 Alt:3  GPIO SD B0 00 XBAR1 INOUT04 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN04_OFFSET),
      [ALT3].sel   = 1,

      /* Index:106 Alt:4  GPIO SD B0 00 LPSPI1 SCK */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI1_SCK_OFFSET),
      [ALT4].sel   = 1,

      /* Index:106 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:106 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:106 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:106 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:106 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:107 GPIO_SD_B0_01 */

  {
    {
      /* Index:107 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:107 Alt:1  GPIO SD B0 01 FLEXPWM1 PWMB00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB0_OFFSET),
      [ALT1].sel   = 1,

      /* Index:107 Alt:2  GPIO SD B0 01 LPI2C3 SDA */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C3_SDA_OFFSET),
      [ALT2].sel   = 1,

      /* Index:107 Alt:3  GPIO SD B0 01 XBAR1 INOUT05 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN05_OFFSET),
      [ALT3].sel   = 1,

      /* Index:107 Alt:4  GPIO SD B0 01 LPSPI1 PCS0 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI1_PCS0_OFFSET),
      [ALT4].sel   = 0,

      /* Index:107 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:107 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:107 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:107 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:107 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:108 GPIO_SD_B0_02 */

  {
    {
      /* Index:108 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:108 Alt:1  GPIO SD B0 02 FLEXPWM1 PWMA01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA1_OFFSET),
      [ALT1].sel   = 1,

      /* Index:108 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:108 Alt:3  GPIO SD B0 02 XBAR1 INOUT06 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN06_OFFSET),
      [ALT3].sel   = 1,

      /* Index:108 Alt:4  GPIO SD B0 02 LPSPI1 SDO */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI1_SDO_OFFSET),
      [ALT4].sel   = 1,

      /* Index:108 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:108 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:108 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:108 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:108 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:109 GPIO_SD_B0_03 */

  {
    {
      /* Index:109 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:109 Alt:1  GPIO SD B0 03 FLEXPWM1 PWMB01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB1_OFFSET),
      [ALT1].sel   = 1,

      /* Index:109 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:109 Alt:3  GPIO SD B0 03 XBAR1 INOUT07 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN07_OFFSET),
      [ALT3].sel   = 1,

      /* Index:109 Alt:4  GPIO SD B0 03 LPSPI1 SDI */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI1_SDI_OFFSET),
      [ALT4].sel   = 1,

      /* Index:109 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:109 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:109 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:109 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:109 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:110 GPIO_SD_B0_04 */

  {
    {
      /* Index:110 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:110 Alt:1  GPIO SD B0 04 FLEXPWM1 PWMA02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA2_OFFSET),
      [ALT1].sel   = 1,

      /* Index:110 Alt:2  GPIO SD B0 04 LPUART8 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART8_TX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:110 Alt:3  GPIO SD B0 04 XBAR1 INOUT08 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN08_OFFSET),
      [ALT3].sel   = 1,

      /* Index:110 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:110 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:110 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:110 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:110 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:110 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:111 GPIO_SD_B0_05 */

  {
    {
      /* Index:111 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:111 Alt:1  GPIO SD B0 05 FLEXPWM1 PWMB02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB2_OFFSET),
      [ALT1].sel   = 1,

      /* Index:111 Alt:2  GPIO SD B0 05 LPUART8 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART8_RX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:111 Alt:3  GPIO SD B0 05 XBAR1 INOUT09 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_XBAR1_IN09_OFFSET),
      [ALT3].sel   = 1,

      /* Index:111 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:111 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:111 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:111 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:111 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:111 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:112 GPIO_SD_B1_00 */

  {
    {
      /* Index:112 Alt:0  GPIO SD B1 00 USDHC2 DATA3 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA3_OFFSET),
      [ALT0].sel   = 0,

      /* Index:112 Alt:1  GPIO SD B1 00 FLEXSPIB DATA03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIB_DATA3_OFFSET),
      [ALT1].sel   = 0,

      /* Index:112 Alt:2  GPIO SD B1 00 FLEXPWM1 PWMA03 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMA3_OFFSET),
      [ALT2].sel   = 0,

      /* Index:112 Alt:3  GPIO SD B1 00 SAI1 TX DATA03 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_DATA1_OFFSET),
      [ALT3].sel   = 0,

      /* Index:112 Alt:4  GPIO SD B1 00 LPUART4 TX */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART4_TX_OFFSET),
      [ALT4].sel   = 0,

      /* Index:112 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:112 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:112 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:112 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:112 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:113 GPIO_SD_B1_01 */

  {
    {
      /* Index:113 Alt:0  GPIO SD B1 01 USDHC2 DATA2 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA2_OFFSET),
      [ALT0].sel   = 0,

      /* Index:113 Alt:1  GPIO SD B1 01 FLEXSPIB DATA02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIB_DATA2_OFFSET),
      [ALT1].sel   = 0,

      /* Index:113 Alt:2  GPIO SD B1 01 FLEXPWM1 PWMB03 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM1_PWMB3_OFFSET),
      [ALT2].sel   = 0,

      /* Index:113 Alt:3  GPIO SD B1 01 SAI1 TX DATA02 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_DATA2_OFFSET),
      [ALT3].sel   = 0,

      /* Index:113 Alt:4  GPIO SD B1 01 LPUART4 RX */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART4_RX_OFFSET),
      [ALT4].sel   = 0,

      /* Index:113 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:113 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:113 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:113 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:113 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:114 GPIO_SD_B1_02 */

  {
    {
      /* Index:114 Alt:0  GPIO SD B1 02 USDHC2 DATA1 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA1_OFFSET),
      [ALT0].sel   = 0,

      /* Index:114 Alt:1  GPIO SD B1 02 FLEXSPIB DATA01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIB_DATA1_OFFSET),
      [ALT1].sel   = 0,

      /* Index:114 Alt:2  GPIO SD B1 02 FLEXPWM2 PWMA03 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMA3_OFFSET),
      [ALT2].sel   = 0,

      /* Index:114 Alt:3  GPIO SD B1 02 SAI1 TX DATA01 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_DATA3_OFFSET),
      [ALT3].sel   = 0,

      /* Index:114 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:114 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:114 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:114 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:114 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:114 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:115 GPIO_SD_B1_03 */

  {
    {
      /* Index:115 Alt:0  GPIO SD B1 03 USDHC2 DATA0 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA0_OFFSET),
      [ALT0].sel   = 0,

      /* Index:115 Alt:1  GPIO SD B1 03 FLEXSPIB DATA00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIB_DATA0_OFFSET),
      [ALT1].sel   = 0,

      /* Index:115 Alt:2  GPIO SD B1 03 FLEXPWM2 PWMB03 */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXPWM2_PWMB3_OFFSET),
      [ALT2].sel   = 0,

      /* Index:115 Alt:3  GPIO SD B1 03 SAI1 MCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_MCLK2_OFFSET),
      [ALT3].sel   = 0,

      /* Index:115 Alt:4  GPIO SD B1 03 FLEXCAN1 RX */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXCAN1_RX_OFFSET),
      [ALT4].sel   = 0,

      /* Index:115 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:115 Alt:6  GPIO SD B1 03 CCM PMIC READY */

      [ALT6].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_CCM_PMIC_READY_OFFSET),
      [ALT6].sel   = 0,

      /* Index:115 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:115 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:115 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:116 GPIO_SD_B1_04 */

  {
    {
      /* Index:116 Alt:0  GPIO SD B1 04 USDHC2 CLK */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_CLK_OFFSET),
      [ALT0].sel   = 0,

      /* Index:116 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:116 Alt:2  GPIO SD B1 04 LPI2C1 SCL */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C1_SCL_OFFSET),
      [ALT2].sel   = 0,

      /* Index:116 Alt:3  GPIO SD B1 04 SAI1 RX SYNC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_SYNC_OFFSET),
      [ALT3].sel   = 0,

      /* Index:116 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:116 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:116 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:116 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:116 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:116 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:117 GPIO_SD_B1_05 */

  {
    {
      /* Index:117 Alt:0  GPIO SD B1 05 USDHC2 CMD */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_CMD_OFFSET),
      [ALT0].sel   = 0,

      /* Index:117 Alt:1  GPIO SD B1 05 FLEXSPIA DQS */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DQS_OFFSET),
      [ALT1].sel   = 0,

      /* Index:117 Alt:2  GPIO SD B1 05 LPI2C1 SDA */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C1_SDA_OFFSET),
      [ALT2].sel   = 0,

      /* Index:117 Alt:3  GPIO SD B1 05 SAI1 RX BCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_BCLK_OFFSET),
      [ALT3].sel   = 0,

      /* Index:117 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:117 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:117 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:117 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:117 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:117 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:118 GPIO_SD_B1_06 */

  {
    {
      /* Index:118 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:118 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:118 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:118 Alt:3  GPIO SD B1 06 SAI1 RX DATA00 */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_RX_DATA0_OFFSET),
      [ALT3].sel   = 0,

      /* Index:118 Alt:4  GPIO SD B1 06 LPSPI2 PCS0 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI2_PCS0_OFFSET),
      [ALT4].sel   = 0,

      /* Index:118 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:118 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:118 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:118 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:118 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:119 GPIO_SD_B1_07 */

  {
    {
      /* Index:119 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:119 Alt:1  GPIO SD B1 07 FLEXSPIA SCLK */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_SCK_OFFSET),
      [ALT1].sel   = 0,

      /* Index:119 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:119 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:119 Alt:4  GPIO SD B1 07 LPSPI2 SCK */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI2_SCK_OFFSET),
      [ALT4].sel   = 0,

      /* Index:119 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:119 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:119 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:119 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:119 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:120 GPIO_SD_B1_08 */

  {
    {
      /* Index:120 Alt:0  GPIO SD B1 08 USDHC2 DATA4 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA4_OFFSET),
      [ALT0].sel   = 0,

      /* Index:120 Alt:1  GPIO SD B1 08 FLEXSPIA DATA00 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DATA0_OFFSET),
      [ALT1].sel   = 0,

      /* Index:120 Alt:2  GPIO SD B1 08 LPUART7 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART7_TX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:120 Alt:3  GPIO SD B1 08 SAI1 TX BCLK */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_TX_BCLK_OFFSET),
      [ALT3].sel   = 0,

      /* Index:120 Alt:4  GPIO SD B1 08 LPSPI2 SD0 */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI2_SDO_OFFSET),
      [ALT4].sel   = 0,

      /* Index:120 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:120 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:120 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:120 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:120 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:121 GPIO_SD_B1_09 */

  {
    {
      /* Index:121 Alt:0  GPIO SD B1 09 USDHC2 DATA5 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA5_OFFSET),
      [ALT0].sel   = 0,

      /* Index:121 Alt:1  GPIO SD B1 09 FLEXSPIA DATA01 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DATA1_OFFSET),
      [ALT1].sel   = 0,

      /* Index:121 Alt:2  GPIO SD B1 09 LPUART7 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART7_RX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:121 Alt:3  GPIO SD B1 09 SAI1 TX SYNC */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_SAI1_TX_SYNC_OFFSET),
      [ALT3].sel   = 0,

      /* Index:121 Alt:4  GPIO SD B1 09 LPSPI2 SDI */

      [ALT4].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPSPI2_SDI_OFFSET),
      [ALT4].sel   = 0,

      /* Index:121 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:121 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:121 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:121 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:121 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:122 GPIO_SD_B1_10 */

  {
    {
      /* Index:122 Alt:0  GPIO SD B1 10 USDHC2 DATA6 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA6_OFFSET),
      [ALT0].sel   = 0,

      /* Index:122 Alt:1  GPIO SD B1 10 FLEXSPIA DATA02 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DATA2_OFFSET),
      [ALT1].sel   = 0,

      /* Index:122 Alt:2  GPIO SD B1 10 LPUART2 RX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART2_RX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:122 Alt:3  GPIO SD B1 10 LPI2C2 SDA */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C2_SDA_OFFSET),
      [ALT3].sel   = 0,

      /* Index:122 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:122 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:122 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:122 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:122 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:122 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:123 GPIO_SD_B1_11 */

  {
    {
      /* Index:123 Alt:0  GPIO SD B1 11 USDHC2 DATA7 */

      [ALT0].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_USDHC2_DATA7_OFFSET),
      [ALT0].sel   = 0,

      /* Index:123 Alt:1  GPIO SD B1 11 FLEXSPIA DATA03 */

      [ALT1].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_FLEXSPIA_DATA3_OFFSET),
      [ALT1].sel   = 0,

      /* Index:123 Alt:2  GPIO SD B1 11 LPUART2 TX */

      [ALT2].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPUART2_TX_OFFSET),
      [ALT2].sel   = 0,

      /* Index:123 Alt:3  GPIO SD B1 11 LPI2C2 SCL */

      [ALT3].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_LPI2C2_SCL_OFFSET),
      [ALT3].sel   = 0,

      /* Index:123 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:123 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:123 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:123 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:123 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:123 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:124 WAKEUP */

  {
    {
      /* Index:124 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:124 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:124 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:124 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:124 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:124 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:124 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:124 Alt:7  SNVS WAKEUP NMI GLUE NMI */

      [ALT7].index = IMXRT_INPUT_OFFSET2INDEX(IMXRT_INPUT_NMI_GLUE_NMI_OFFSET),
      [ALT7].sel   = 1,

      /* Index:124 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:124 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },

  /* index:125 PMIC_ON_REQ */

  {
    {
      /* Index:125 Alt:0 No input selection */

      [ALT0].index = DAISY_INDEX_INVALID,
      [ALT0].sel   = DAISY_SEL_INVALID,

      /* Index:125 Alt:1 No input selection */

      [ALT1].index = DAISY_INDEX_INVALID,
      [ALT1].sel   = DAISY_SEL_INVALID,

      /* Index:125 Alt:2 No input selection */

      [ALT2].index = DAISY_INDEX_INVALID,
      [ALT2].sel   = DAISY_SEL_INVALID,

      /* Index:125 Alt:3 No input selection */

      [ALT3].index = DAISY_INDEX_INVALID,
      [ALT3].sel   = DAISY_SEL_INVALID,

      /* Index:125 Alt:4 No input selection */

      [ALT4].index = DAISY_INDEX_INVALID,
      [ALT4].sel   = DAISY_SEL_INVALID,

      /* Index:125 Alt:5 No input selection */

      [ALT5].index = DAISY_INDEX_INVALID,
      [ALT5].sel   = DAISY_SEL_INVALID,

      /* Index:125 Alt:6 No input selection */

      [ALT6].index = DAISY_INDEX_INVALID,
      [ALT6].sel   = DAISY_SEL_INVALID,

      /* Index:125 Alt:7 No input selection */

      [ALT7].index = DAISY_INDEX_INVALID,
      [ALT7].sel   = DAISY_SEL_INVALID,

      /* Index:125 Alt:8 No input selection */

      [ALT8].index = DAISY_INDEX_INVALID,
      [ALT8].sel   = DAISY_SEL_INVALID,

      /* Index:125 Alt:9 No input selection */

      [ALT9].index = DAISY_INDEX_INVALID,
      [ALT9].sel   = DAISY_SEL_INVALID,
    },
  },
};
