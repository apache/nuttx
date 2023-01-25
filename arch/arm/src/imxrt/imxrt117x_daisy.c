/****************************************************************************
 * arch/arm/src/imxrt/imxrt117x_daisy.c
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

/*  Based on chip selection this file is included in imxrt_daisy.c */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DAISY_OFFSET_INVALID    0xffff
#define DAISY_SEL_INVALID       255
#define ALT0                    0
#define ALT1                    1
#define ALT2                    2
#define ALT3                    3
#define ALT4                    4
#define ALT5                    5
#define ALT6                    6
#define ALT7                    7
#define ALT8                    8
#define ALT9                    9
#define ALT10                   10
#define ALT11                   11

#define IMXRT_DAISY_SELECT_PROVIDED

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct imxrt_daisy_entry_t
{
  uint16_t  offset;
  uint8_t   sel;
};

struct imxrt_daisy_t
{
  struct imxrt_daisy_entry_t alts[12];
};

static const struct imxrt_daisy_t g_daisy_select[] =
{
  /* index:0 GPIO_EMC_B1_00 */

  {
    {
      /* Index:0 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:0 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:1 GPIO_EMC_B1_01 */

  {
    {
      /* Index:1 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:1 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:2 GPIO_EMC_B1_02 */

  {
    {
      /* Index:2 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:2 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:3 GPIO_EMC_B1_03 */

  {
    {
      /* Index:3 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:3 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:4 GPIO_EMC_B1_04 */

  {
    {
      /* Index:4 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:4 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:5 GPIO_EMC_B1_05 */

  {
    {
      /* Index:5 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:5 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:6 GPIO_EMC_B1_06 */

  {
    {
      /* Index:6 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:1  GPIO EMC B1 06 FLEXPWM2 PWM0 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM2_PWMA0_OFFSET,
      [ALT1].sel    = 0,

      /* Index:6 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:6 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:7 GPIO_EMC_B1_07 */

  {
    {
      /* Index:7 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:1  GPIO EMC B1 07 FLEXPWM2 PWM0 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM2_PWMB0_OFFSET,
      [ALT1].sel    = 0,

      /* Index:7 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:7 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:8 GPIO_EMC_B1_08 */

  {
    {
      /* Index:8 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:1  GPIO EMC B1 08 FLEXPWM2 PWM1 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM2_PWMA1_OFFSET,
      [ALT1].sel    = 0,

      /* Index:8 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:8 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:9 GPIO_EMC_B1_09 */

  {
    {
      /* Index:9 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:1  GPIO EMC B1 09 FLEXPWM2 PWM1 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM2_PWMB1_OFFSET,
      [ALT1].sel    = 0,

      /* Index:9 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:9 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:10 GPIO_EMC_B1_10 */

  {
    {
      /* Index:10 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:1  GPIO EMC B1 10 FLEXPWM2 PWM2 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM2_PWMA2_OFFSET,
      [ALT1].sel    = 0,

      /* Index:10 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:10 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:11 GPIO_EMC_B1_11 */

  {
    {
      /* Index:11 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:1  GPIO EMC B1 11 FLEXPWM2 PWM2 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM2_PWMB2_OFFSET,
      [ALT1].sel    = 0,

      /* Index:11 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:11 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:12 GPIO_EMC_B1_12 */

  {
    {
      /* Index:12 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:12 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:13 GPIO_EMC_B1_13 */

  {
    {
      /* Index:13 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:13 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:14 GPIO_EMC_B1_14 */

  {
    {
      /* Index:14 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:14 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:15 GPIO_EMC_B1_15 */

  {
    {
      /* Index:15 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:15 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:16 GPIO_EMC_B1_16 */

  {
    {
      /* Index:16 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:16 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:17 GPIO_EMC_B1_17 */

  {
    {
      /* Index:17 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:2  GPIO EMC B1 17 TMR1 TIMER0 */

      [ALT2].offset = IMXRT_INPUT_QTIMER1_TIMER0_OFFSET,
      [ALT2].sel    = 0,

      /* Index:17 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:17 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:18 GPIO_EMC_B1_18 */

  {
    {
      /* Index:18 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:2  GPIO EMC B1 18 TMR2 TIMER0 */

      [ALT2].offset = IMXRT_INPUT_QTIMER2_TIMER0_OFFSET,
      [ALT2].sel    = 0,

      /* Index:18 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:18 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:19 GPIO_EMC_B1_19 */

  {
    {
      /* Index:19 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:2  GPIO EMC B1 19 TMR3 TIMER0 */

      [ALT2].offset = IMXRT_INPUT_QTIMER3_TIMER0_OFFSET,
      [ALT2].sel    = 0,

      /* Index:19 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:19 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:20 GPIO_EMC_B1_20 */

  {
    {
      /* Index:20 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:2  GPIO EMC B1 20 TMR4 TIMER0 */

      [ALT2].offset = IMXRT_INPUT_QTIMER4_TIMER0_OFFSET,
      [ALT2].sel    = 0,

      /* Index:20 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:20 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:21 GPIO_EMC_B1_21 */

  {
    {
      /* Index:21 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:1  GPIO EMC B1 21 FLEXPWM3 PWM3 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM3_PWMA3_OFFSET,
      [ALT1].sel    = 0,

      /* Index:21 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:21 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:22 GPIO_EMC_B1_22 */

  {
    {
      /* Index:22 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:1  GPIO EMC B1 22 FLEXPWM3 PWM3 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM3_PWMB3_OFFSET,
      [ALT1].sel    = 0,

      /* Index:22 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:22 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:23 GPIO_EMC_B1_23 */

  {
    {
      /* Index:23 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:1  GPIO EMC B1 23 FLEXPWM1 PWM0 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM1_PWMA0_OFFSET,
      [ALT1].sel    = 0,

      /* Index:23 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:23 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:24 GPIO_EMC_B1_24 */

  {
    {
      /* Index:24 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:1  GPIO EMC B1 24 FLEXPWM1 PWM0 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM1_PWMB0_OFFSET,
      [ALT1].sel    = 0,

      /* Index:24 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:24 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:25 GPIO_EMC_B1_25 */

  {
    {
      /* Index:25 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:1  GPIO EMC B1 25 FLEXPWM1 PWM1 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM1_PWMA1_OFFSET,
      [ALT1].sel    = 0,

      /* Index:25 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:25 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:26 GPIO_EMC_B1_26 */

  {
    {
      /* Index:26 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:1  GPIO EMC B1 26 FLEXPWM1 PWM1 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM1_PWMB1_OFFSET,
      [ALT1].sel    = 0,

      /* Index:26 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:26 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:27 GPIO_EMC_B1_27 */

  {
    {
      /* Index:27 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:1  GPIO EMC B1 27 FLEXPWM1 PWM2 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM1_PWMA2_OFFSET,
      [ALT1].sel    = 0,

      /* Index:27 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:27 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:28 GPIO_EMC_B1_28 */

  {
    {
      /* Index:28 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:1  GPIO EMC B1 28 FLEXPWM1 PWM2 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM1_PWMB2_OFFSET,
      [ALT1].sel    = 0,

      /* Index:28 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:28 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:29 GPIO_EMC_B1_29 */

  {
    {
      /* Index:29 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:1  GPIO EMC B1 29 FLEXPWM3 PWM0 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM3_PWMA0_OFFSET,
      [ALT1].sel    = 0,

      /* Index:29 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:29 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:30 GPIO_EMC_B1_30 */

  {
    {
      /* Index:30 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:1  GPIO EMC B1 30 FLEXPWM3 PWM0 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM3_PWMB0_OFFSET,
      [ALT1].sel    = 0,

      /* Index:30 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:30 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:31 GPIO_EMC_B1_31 */

  {
    {
      /* Index:31 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:1  GPIO EMC B1 31 FLEXPWM3 PWM1 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM3_PWMA1_OFFSET,
      [ALT1].sel    = 0,

      /* Index:31 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:31 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:32 GPIO_EMC_B1_32 */

  {
    {
      /* Index:32 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:1  GPIO EMC B1 32 FLEXPWM3 PWM1 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM3_PWMB1_OFFSET,
      [ALT1].sel    = 0,

      /* Index:32 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:32 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:33 GPIO_EMC_B1_33 */

  {
    {
      /* Index:33 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:1  GPIO EMC B1 33 FLEXPWM3 PWM2 A */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM3_PWMA2_OFFSET,
      [ALT1].sel    = 0,

      /* Index:33 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:33 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:34 GPIO_EMC_B1_34 */

  {
    {
      /* Index:34 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:1  GPIO EMC B1 34 FLEXPWM3 PWM2 B */

      [ALT1].offset = IMXRT_INPUT_FLEXPWM3_PWMB2_OFFSET,
      [ALT1].sel    = 0,

      /* Index:34 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:34 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:35 GPIO_EMC_B1_35 */

  {
    {
      /* Index:35 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:35 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:36 GPIO_EMC_B1_36 */

  {
    {
      /* Index:36 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:36 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:37 GPIO_EMC_B1_37 */

  {
    {
      /* Index:37 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:37 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:38 GPIO_EMC_B1_38 */

  {
    {
      /* Index:38 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:2  GPIO EMC B1 38 TMR1 TIMER1 */

      [ALT2].offset = IMXRT_INPUT_QTIMER1_TIMER1_OFFSET,
      [ALT2].sel    = 0,

      /* Index:38 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:38 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:39 GPIO_EMC_B1_39 */

  {
    {
      /* Index:39 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:2  GPIO EMC B1 39 TMR2 TIMER1 */

      [ALT2].offset = IMXRT_INPUT_QTIMER2_TIMER1_OFFSET,
      [ALT2].sel    = 0,

      /* Index:39 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:39 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:40 GPIO_EMC_B1_40 */

  {
    {
      /* Index:40 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:40 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:41 GPIO_EMC_B1_41 */

  {
    {
      /* Index:41 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:7  GPIO EMC B1 41 ENET 1G MDIO */

      [ALT7].offset = IMXRT_INPUT_ENET_1G_MDIO_OFFSET,
      [ALT7].sel    = 0,

      /* Index:41 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:41 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:42 GPIO_EMC_B2_00 */

  {
    {
      /* Index:42 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:42 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:42 Alt:2  GPIO EMC B2 00 TMR3 TIMER1 */

      [ALT2].offset = IMXRT_INPUT_QTIMER3_TIMER1_OFFSET,
      [ALT2].sel    = 0,

      /* Index:42 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:42 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:42 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:42 Alt:6  GPIO EMC B2 00 XBAR1 INOUT20 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN20_OFFSET,
      [ALT6].sel    = 0,

      /* Index:42 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:42 Alt:8  GPIO EMC B2 00 LPSPI1 SCK */

      [ALT8].offset = IMXRT_INPUT_LPSPI1_SCK_OFFSET,
      [ALT8].sel    = 0,

      /* Index:42 Alt:9  GPIO EMC B2 00 LPI2C2 SCL */

      [ALT9].offset = IMXRT_INPUT_LPI2C2_SCL_OFFSET,
      [ALT9].sel    = 0,

      /* Index:42 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:42 Alt:11  GPIO EMC B2 00 FLEXPWM3 PWM0 A */

      [ALT11].offset = IMXRT_INPUT_FLEXPWM3_PWMA0_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:43 GPIO_EMC_B2_01 */

  {
    {
      /* Index:43 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:43 Alt:1  GPIO EMC B2 01 USDHC2 CD B */

      [ALT1].offset = IMXRT_INPUT_USDHC2_CD_B_OFFSET,
      [ALT1].sel    = 0,

      /* Index:43 Alt:2  GPIO EMC B2 01 TMR4 TIMER1 */

      [ALT2].offset = IMXRT_INPUT_QTIMER4_TIMER1_OFFSET,
      [ALT2].sel    = 0,

      /* Index:43 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:43 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:43 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:43 Alt:6  GPIO EMC B2 01 XBAR1 INOUT21 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN21_OFFSET,
      [ALT6].sel    = 0,

      /* Index:43 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:43 Alt:8  GPIO EMC B2 01 LPSPI1 PCS0 */

      [ALT8].offset = IMXRT_INPUT_LPSPI1_PCS0_OFFSET,
      [ALT8].sel    = 0,

      /* Index:43 Alt:9  GPIO EMC B2 01 LPI2C2 SDA */

      [ALT9].offset = IMXRT_INPUT_LPI2C2_SDA_OFFSET,
      [ALT9].sel    = 0,

      /* Index:43 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:43 Alt:11  GPIO EMC B2 01 FLEXPWM3 PWM0 B */

      [ALT11].offset = IMXRT_INPUT_FLEXPWM3_PWMB0_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:44 GPIO_EMC_B2_02 */

  {
    {
      /* Index:44 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:44 Alt:1  GPIO EMC B2 02 USDHC2 WP */

      [ALT1].offset = IMXRT_INPUT_USDHC2_WP_OFFSET,
      [ALT1].sel    = 0,

      /* Index:44 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:44 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:44 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:44 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:44 Alt:6  GPIO EMC B2 02 XBAR1 INOUT22 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN22_OFFSET,
      [ALT6].sel    = 0,

      /* Index:44 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:44 Alt:8  GPIO EMC B2 02 LPSPI1 SOUT */

      [ALT8].offset = IMXRT_INPUT_LPSPI1_SDO_OFFSET,
      [ALT8].sel    = 0,

      /* Index:44 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:44 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:44 Alt:11  GPIO EMC B2 02 FLEXPWM3 PWM1 A */

      [ALT11].offset = IMXRT_INPUT_FLEXPWM3_PWMA1_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:45 GPIO_EMC_B2_03 */

  {
    {
      /* Index:45 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:45 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:45 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:45 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:45 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:45 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:45 Alt:6  GPIO EMC B2 03 XBAR1 INOUT23 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN23_OFFSET,
      [ALT6].sel    = 0,

      /* Index:45 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:45 Alt:8  GPIO EMC B2 03 LPSPI1 SIN */

      [ALT8].offset = IMXRT_INPUT_LPSPI1_SDI_OFFSET,
      [ALT8].sel    = 0,

      /* Index:45 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:45 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:45 Alt:11  GPIO EMC B2 03 FLEXPWM3 PWM1 B */

      [ALT11].offset = IMXRT_INPUT_FLEXPWM3_PWMB1_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:46 GPIO_EMC_B2_04 */

  {
    {
      /* Index:46 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:46 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:46 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:46 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:46 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:46 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:46 Alt:6  GPIO EMC B2 04 XBAR1 INOUT24 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN24_OFFSET,
      [ALT6].sel    = 0,

      /* Index:46 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:46 Alt:8  GPIO EMC B2 04 LPSPI3 SCK */

      [ALT8].offset = IMXRT_INPUT_LPSPI3_SCK_OFFSET,
      [ALT8].sel    = 0,

      /* Index:46 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:46 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:46 Alt:11  GPIO EMC B2 04 FLEXPWM3 PWM2 A */

      [ALT11].offset = IMXRT_INPUT_FLEXPWM3_PWMA2_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:47 GPIO_EMC_B2_05 */

  {
    {
      /* Index:47 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:47 Alt:1  GPIO EMC B2 05 GPT3 CLK */

      [ALT1].offset = IMXRT_INPUT_GPT3_CLKIN_OFFSET,
      [ALT1].sel    = 0,

      /* Index:47 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:47 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:47 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:47 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:47 Alt:6  GPIO EMC B2 05 XBAR1 INOUT25 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN25_OFFSET,
      [ALT6].sel    = 0,

      /* Index:47 Alt:7  GPIO EMC B2 05 ENET 1G RX CLK */

      [ALT7].offset = IMXRT_INPUT_ENET_1G_RXCLK_OFFSET,
      [ALT7].sel    = 0,

      /* Index:47 Alt:8  GPIO EMC B2 05 LPSPI3 PCS0 */

      [ALT8].offset = IMXRT_INPUT_LPSPI3_PCS0_OFFSET,
      [ALT8].sel    = 0,

      /* Index:47 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:47 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:47 Alt:11  GPIO EMC B2 05 FLEXPWM3 PWM2 B */

      [ALT11].offset = IMXRT_INPUT_FLEXPWM3_PWMB2_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:48 GPIO_EMC_B2_06 */

  {
    {
      /* Index:48 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:48 Alt:1  GPIO EMC B2 06 GPT3 CAPTURE1 */

      [ALT1].offset = IMXRT_INPUT_GPT3_CAPIN1_OFFSET,
      [ALT1].sel    = 0,

      /* Index:48 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:48 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:48 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:48 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:48 Alt:6  GPIO EMC B2 06 XBAR1 INOUT26 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN26_OFFSET,
      [ALT6].sel    = 0,

      /* Index:48 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:48 Alt:8  GPIO EMC B2 06 LPSPI3 SOUT */

      [ALT8].offset = IMXRT_INPUT_LPSPI3_SDO_OFFSET,
      [ALT8].sel    = 0,

      /* Index:48 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:48 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:48 Alt:11  GPIO EMC B2 06 FLEXPWM3 PWM3 A */

      [ALT11].offset = IMXRT_INPUT_FLEXPWM3_PWMA3_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:49 GPIO_EMC_B2_07 */

  {
    {
      /* Index:49 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:49 Alt:1  GPIO EMC B2 07 GPT3 CAPTURE2 */

      [ALT1].offset = IMXRT_INPUT_GPT3_CAPIN2_OFFSET,
      [ALT1].sel    = 0,

      /* Index:49 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:49 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:49 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:49 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:49 Alt:6  GPIO EMC B2 07 XBAR1 INOUT27 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN27_OFFSET,
      [ALT6].sel    = 0,

      /* Index:49 Alt:7  GPIO EMC B2 07 ENET 1G RX DATA03 */

      [ALT7].offset = IMXRT_INPUT_ENET_1G_RXDATA3_OFFSET,
      [ALT7].sel    = 0,

      /* Index:49 Alt:8  GPIO EMC B2 07 LPSPI3 SIN */

      [ALT8].offset = IMXRT_INPUT_LPSPI3_SDI_OFFSET,
      [ALT8].sel    = 0,

      /* Index:49 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:49 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:49 Alt:11  GPIO EMC B2 07 FLEXPWM3 PWM3 B */

      [ALT11].offset = IMXRT_INPUT_FLEXPWM3_PWMB3_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:50 GPIO_EMC_B2_08 */

  {
    {
      /* Index:50 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:50 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:50 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:50 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:50 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:50 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:50 Alt:6  GPIO EMC B2 08 XBAR1 INOUT28 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN28_OFFSET,
      [ALT6].sel    = 0,

      /* Index:50 Alt:7  GPIO EMC B2 08 ENET 1G RX DATA02 */

      [ALT7].offset = IMXRT_INPUT_ENET_1G_RXDATA2_OFFSET,
      [ALT7].sel    = 0,

      /* Index:50 Alt:8  GPIO EMC B2 08 LPSPI3 PCS1 */

      [ALT8].offset = IMXRT_INPUT_LPSPI3_PCS1_OFFSET,
      [ALT8].sel    = 0,

      /* Index:50 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:50 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:50 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:51 GPIO_EMC_B2_09 */

  {
    {
      /* Index:51 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:51 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:51 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:51 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:51 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:51 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:51 Alt:6  GPIO EMC B2 09 XBAR1 INOUT29 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN29_OFFSET,
      [ALT6].sel    = 0,

      /* Index:51 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:51 Alt:8  GPIO EMC B2 09 LPSPI3 PCS2 */

      [ALT8].offset = IMXRT_INPUT_LPSPI3_PCS2_OFFSET,
      [ALT8].sel    = 0,

      /* Index:51 Alt:9  GPIO EMC B2 09 TMR1 TIMER0 */

      [ALT9].offset = IMXRT_INPUT_QTIMER1_TIMER0_OFFSET,
      [ALT9].sel    = 1,

      /* Index:51 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:51 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:52 GPIO_EMC_B2_10 */

  {
    {
      /* Index:52 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:52 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:52 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:52 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:52 Alt:4  GPIO EMC B2 10 FLEXSPI2 A SCLK */

      [ALT4].offset = IMXRT_INPUT_FLEXSPI2A_SCK_OFFSET,
      [ALT4].sel    = 0,

      /* Index:52 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:52 Alt:6  GPIO EMC B2 10 XBAR1 INOUT30 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN30_OFFSET,
      [ALT6].sel    = 0,

      /* Index:52 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:52 Alt:8  GPIO EMC B2 10 LPSPI3 PCS3 */

      [ALT8].offset = IMXRT_INPUT_LPSPI3_PCS3_OFFSET,
      [ALT8].sel    = 0,

      /* Index:52 Alt:9  GPIO EMC B2 10 TMR1 TIMER1 */

      [ALT9].offset = IMXRT_INPUT_QTIMER1_TIMER1_OFFSET,
      [ALT9].sel    = 1,

      /* Index:52 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:52 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:53 GPIO_EMC_B2_11 */

  {
    {
      /* Index:53 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:53 Alt:1  GPIO EMC B2 11 SPDIF IN */

      [ALT1].offset = IMXRT_INPUT_SPDIF_IN_OFFSET,
      [ALT1].sel    = 0,

      /* Index:53 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:53 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:53 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:53 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:53 Alt:6  GPIO EMC B2 11 XBAR1 INOUT31 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN31_OFFSET,
      [ALT6].sel    = 0,

      /* Index:53 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:53 Alt:8  GPIO EMC B2 11 EMVSIM1 IO */

      [ALT8].offset = IMXRT_INPUT_EMVSIM1_SIO_OFFSET,
      [ALT8].sel    = 0,

      /* Index:53 Alt:9  GPIO EMC B2 11 TMR1 TIMER2 */

      [ALT9].offset = IMXRT_INPUT_QTIMER1_TIMER2_OFFSET,
      [ALT9].sel    = 0,

      /* Index:53 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:53 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:54 GPIO_EMC_B2_12 */

  {
    {
      /* Index:54 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:6  GPIO EMC B2 12 XBAR1 INOUT32 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN32_OFFSET,
      [ALT6].sel    = 0,

      /* Index:54 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:54 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:55 GPIO_EMC_B2_13 */

  {
    {
      /* Index:55 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:55 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:55 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:55 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:55 Alt:4  GPIO EMC B2 13 FLEXSPI2 A DATA00 */

      [ALT4].offset = IMXRT_INPUT_FLEXSPI2A_DATA0_OFFSET,
      [ALT4].sel    = 0,

      /* Index:55 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:55 Alt:6  GPIO EMC B2 13 XBAR1 INOUT33 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN33_OFFSET,
      [ALT6].sel    = 0,

      /* Index:55 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:55 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:55 Alt:9  GPIO EMC B2 13 TMR2 TIMER0 */

      [ALT9].offset = IMXRT_INPUT_QTIMER2_TIMER0_OFFSET,
      [ALT9].sel    = 1,

      /* Index:55 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:55 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:56 GPIO_EMC_B2_14 */

  {
    {
      /* Index:56 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:56 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:56 Alt:2  GPIO EMC B2 14 ENET 1G TX CLK IO */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_TXCLK_OFFSET,
      [ALT2].sel    = 0,

      /* Index:56 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:56 Alt:4  GPIO EMC B2 14 FLEXSPI2 A DATA01 */

      [ALT4].offset = IMXRT_INPUT_FLEXSPI2A_DATA1_OFFSET,
      [ALT4].sel    = 0,

      /* Index:56 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:56 Alt:6  GPIO EMC B2 14 XBAR1 INOUT34 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN34_OFFSET,
      [ALT6].sel    = 0,

      /* Index:56 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:56 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:56 Alt:9  GPIO EMC B2 14 TMR2 TIMER1 */

      [ALT9].offset = IMXRT_INPUT_QTIMER2_TIMER1_OFFSET,
      [ALT9].sel    = 1,

      /* Index:56 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:56 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:57 GPIO_EMC_B2_15 */

  {
    {
      /* Index:57 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:57 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:57 Alt:2  GPIO EMC B2 15 ENET 1G RX DATA00 */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXDATA0_OFFSET,
      [ALT2].sel    = 0,

      /* Index:57 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:57 Alt:4  GPIO EMC B2 15 FLEXSPI2 A DATA02 */

      [ALT4].offset = IMXRT_INPUT_FLEXSPI2A_DATA2_OFFSET,
      [ALT4].sel    = 0,

      /* Index:57 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:57 Alt:6  GPIO EMC B2 15 XBAR1 INOUT35 */

      [ALT6].offset = IMXRT_INPUT_XBAR1_IN35_OFFSET,
      [ALT6].sel    = 0,

      /* Index:57 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:57 Alt:8  GPIO EMC B2 15 EMVSIM1 PD */

      [ALT8].offset = IMXRT_INPUT_EMVSIM1_SIMPD_OFFSET,
      [ALT8].sel    = 0,

      /* Index:57 Alt:9  GPIO EMC B2 15 TMR2 TIMER2 */

      [ALT9].offset = IMXRT_INPUT_QTIMER2_TIMER2_OFFSET,
      [ALT9].sel    = 0,

      /* Index:57 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:57 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:58 GPIO_EMC_B2_16 */

  {
    {
      /* Index:58 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:58 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:58 Alt:2  GPIO EMC B2 16 ENET 1G RX DATA01 */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXDATA1_OFFSET,
      [ALT2].sel    = 0,

      /* Index:58 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:58 Alt:4  GPIO EMC B2 16 FLEXSPI2 A DATA03 */

      [ALT4].offset = IMXRT_INPUT_FLEXSPI2A_DATA3_OFFSET,
      [ALT4].sel    = 0,

      /* Index:58 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:58 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:58 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:58 Alt:8  GPIO EMC B2 16 EMVSIM1 POWER FAIL */

      [ALT8].offset = IMXRT_INPUT_EMVSIM1_POWER_FAIL_OFFSET,
      [ALT8].sel    = 0,

      /* Index:58 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:58 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:58 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:59 GPIO_EMC_B2_17 */

  {
    {
      /* Index:59 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:59 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:59 Alt:2  GPIO EMC B2 17 ENET 1G RX EN */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXEN_OFFSET,
      [ALT2].sel    = 0,

      /* Index:59 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:59 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:59 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:59 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:59 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:59 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:59 Alt:9  GPIO EMC B2 17 TMR3 TIMER0 */

      [ALT9].offset = IMXRT_INPUT_QTIMER3_TIMER0_OFFSET,
      [ALT9].sel    = 1,

      /* Index:59 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:59 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:60 GPIO_EMC_B2_18 */

  {
    {
      /* Index:60 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:60 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:60 Alt:2  GPIO EMC B2 18 ENET 1G RX ER */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXERR_OFFSET,
      [ALT2].sel    = 0,

      /* Index:60 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:60 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:60 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:60 Alt:6  GPIO EMC B2 18 FLEXSPI1 A DQS */

      [ALT6].offset = IMXRT_INPUT_FLEXSPI1A_DQS_OFFSET,
      [ALT6].sel    = 0,

      /* Index:60 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:60 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:60 Alt:9  GPIO EMC B2 18 TMR3 TIMER1 */

      [ALT9].offset = IMXRT_INPUT_QTIMER3_TIMER1_OFFSET,
      [ALT9].sel    = 1,

      /* Index:60 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:60 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:61 GPIO_EMC_B2_19 */

  {
    {
      /* Index:61 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:61 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:61 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:61 Alt:3  GPIO EMC B2 19 ENET 1G REF CLK */

      [ALT3].offset = IMXRT_INPUT_ENET_1G_IPG_CLK_RMII_OFFSET,
      [ALT3].sel    = 0,

      /* Index:61 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:61 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:61 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:61 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:61 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:61 Alt:9  GPIO EMC B2 19 TMR3 TIMER2 */

      [ALT9].offset = IMXRT_INPUT_QTIMER3_TIMER2_OFFSET,
      [ALT9].sel    = 0,

      /* Index:61 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:61 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:62 GPIO_EMC_B2_20 */

  {
    {
      /* Index:62 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:62 Alt:1  GPIO EMC B2 20 ENET MDIO */

      [ALT1].offset = IMXRT_INPUT_ENET_MDIO_OFFSET,
      [ALT1].sel    = 0,

      /* Index:62 Alt:2  GPIO EMC B2 20 ENET 1G MDIO */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_MDIO_OFFSET,
      [ALT2].sel    = 1,

      /* Index:62 Alt:3  GPIO EMC B2 20 ENET QOS REF CLK */

      [ALT3].offset = IMXRT_INPUT_CCM_ENET_QOS_REFCLK_OFFSET,
      [ALT3].sel    = 0,

      /* Index:62 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:62 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:62 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:62 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:62 Alt:8  GPIO EMC B2 20 ENET QOS MDIO */

      [ALT8].offset = IMXRT_INPUT_ENET_QOS_MDIO_OFFSET,
      [ALT8].sel    = 0,

      /* Index:62 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:62 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:62 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:63 GPIO_AD_00 */

  {
    {
      /* Index:63 Alt:0  GPIO AD 00 EMVSIM1 IO */

      [ALT0].offset = IMXRT_INPUT_EMVSIM1_SIO_OFFSET,
      [ALT0].sel    = 1,

      /* Index:63 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:63 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:63 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:63 Alt:4  GPIO AD 00 FLEXPWM1 PWM0 A */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM1_PWMA0_OFFSET,
      [ALT4].sel    = 1,

      /* Index:63 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:63 Alt:6  GPIO AD 00 LPUART7 TXD */

      [ALT6].offset = IMXRT_INPUT_LPUART7_TX_OFFSET,
      [ALT6].sel    = 0,

      /* Index:63 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:63 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:63 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:63 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:63 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:64 GPIO_AD_01 */

  {
    {
      /* Index:64 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:64 Alt:1  GPIO AD 01 FLEXCAN2 RX */

      [ALT1].offset = IMXRT_INPUT_FLEXCAN2_RX_OFFSET,
      [ALT1].sel    = 0,

      /* Index:64 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:64 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:64 Alt:4  GPIO AD 01 FLEXPWM1 PWM0 B */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM1_PWMB0_OFFSET,
      [ALT4].sel    = 1,

      /* Index:64 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:64 Alt:6  GPIO AD 01 LPUART7 RXD */

      [ALT6].offset = IMXRT_INPUT_LPUART7_RX_OFFSET,
      [ALT6].sel    = 0,

      /* Index:64 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:64 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:64 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:64 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:64 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:65 GPIO_AD_02 */

  {
    {
      /* Index:65 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:65 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:65 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:65 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:65 Alt:4  GPIO AD 02 FLEXPWM1 PWM1 A */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM1_PWMA1_OFFSET,
      [ALT4].sel    = 1,

      /* Index:65 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:65 Alt:6  GPIO AD 02 LPUART8 TXD */

      [ALT6].offset = IMXRT_INPUT_LPUART8_TX_OFFSET,
      [ALT6].sel    = 0,

      /* Index:65 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:65 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:65 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:65 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:65 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:66 GPIO_AD_03 */

  {
    {
      /* Index:66 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:66 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:66 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:66 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:66 Alt:4  GPIO AD 03 FLEXPWM1 PWM1 B */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM1_PWMB1_OFFSET,
      [ALT4].sel    = 1,

      /* Index:66 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:66 Alt:6  GPIO AD 03 LPUART8 RXD */

      [ALT6].offset = IMXRT_INPUT_LPUART8_RX_OFFSET,
      [ALT6].sel    = 0,

      /* Index:66 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:66 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:66 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:66 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:66 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:67 GPIO_AD_04 */

  {
    {
      /* Index:67 Alt:0  GPIO AD 04 EMVSIM1 PD */

      [ALT0].offset = IMXRT_INPUT_EMVSIM1_SIMPD_OFFSET,
      [ALT0].sel    = 1,

      /* Index:67 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:67 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:67 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:67 Alt:4  GPIO AD 04 FLEXPWM1 PWM2 A */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM1_PWMA2_OFFSET,
      [ALT4].sel    = 1,

      /* Index:67 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:67 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:67 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:67 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:67 Alt:9  GPIO AD 04 TMR4 TIMER0 */

      [ALT9].offset = IMXRT_INPUT_QTIMER4_TIMER0_OFFSET,
      [ALT9].sel    = 1,

      /* Index:67 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:67 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:68 GPIO_AD_05 */

  {
    {
      /* Index:68 Alt:0  GPIO AD 05 EMVSIM1 POWER FAIL */

      [ALT0].offset = IMXRT_INPUT_EMVSIM1_POWER_FAIL_OFFSET,
      [ALT0].sel    = 1,

      /* Index:68 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:68 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:68 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:68 Alt:4  GPIO AD 05 FLEXPWM1 PWM2 B */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM1_PWMB2_OFFSET,
      [ALT4].sel    = 1,

      /* Index:68 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:68 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:68 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:68 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:68 Alt:9  GPIO AD 05 TMR4 TIMER1 */

      [ALT9].offset = IMXRT_INPUT_QTIMER4_TIMER1_OFFSET,
      [ALT9].sel    = 1,

      /* Index:68 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:68 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:69 GPIO_AD_06 */

  {
    {
      /* Index:69 Alt:0  GPIO AD 06 USB OTG2 OC */

      [ALT0].offset = IMXRT_INPUT_USB_OTG2_OC_OFFSET,
      [ALT0].sel    = 0,

      /* Index:69 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:69 Alt:2  GPIO AD 06 EMVSIM2 IO */

      [ALT2].offset = IMXRT_INPUT_EMVSIM2_SIO_OFFSET,
      [ALT2].sel    = 0,

      /* Index:69 Alt:3  GPIO AD 06 GPT3 CAPTURE1 */

      [ALT3].offset = IMXRT_INPUT_GPT3_CAPIN1_OFFSET,
      [ALT3].sel    = 1,

      /* Index:69 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:69 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:69 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:69 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:69 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:69 Alt:9  GPIO AD 06 TMR4 TIMER2 */

      [ALT9].offset = IMXRT_INPUT_QTIMER4_TIMER2_OFFSET,
      [ALT9].sel    = 0,

      /* Index:69 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:69 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:70 GPIO_AD_07 */

  {
    {
      /* Index:70 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:70 Alt:1  GPIO AD 07 FLEXCAN1 RX */

      [ALT1].offset = IMXRT_INPUT_FLEXCAN1_RX_OFFSET,
      [ALT1].sel    = 0,

      /* Index:70 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:70 Alt:3  GPIO AD 07 GPT3 CAPTURE2 */

      [ALT3].offset = IMXRT_INPUT_GPT3_CAPIN2_OFFSET,
      [ALT3].sel    = 1,

      /* Index:70 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:70 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:70 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:70 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:70 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:70 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:70 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:70 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:71 GPIO_AD_08 */

  {
    {
      /* Index:71 Alt:0  GPIO AD 08 USBPHY2 OTG ID */

      [ALT0].offset = IMXRT_INPUT_USBPHY2_USB_ID_OFFSET,
      [ALT0].sel    = 0,

      /* Index:71 Alt:1  GPIO AD 08 LPI2C1 SCL */

      [ALT1].offset = IMXRT_INPUT_LPI2C1_SCL_OFFSET,
      [ALT1].sel    = 0,

      /* Index:71 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:71 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:71 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:71 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:71 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:71 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:71 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:71 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:71 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:71 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:72 GPIO_AD_09 */

  {
    {
      /* Index:72 Alt:0  GPIO AD 09 USBPHY1 OTG ID */

      [ALT0].offset = IMXRT_INPUT_USBPHY1_USB_ID_OFFSET,
      [ALT0].sel    = 0,

      /* Index:72 Alt:1  GPIO AD 09 LPI2C1 SDA */

      [ALT1].offset = IMXRT_INPUT_LPI2C1_SDA_OFFSET,
      [ALT1].sel    = 0,

      /* Index:72 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:72 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:72 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:72 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:72 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:72 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:72 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:72 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:72 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:72 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:73 GPIO_AD_10 */

  {
    {
      /* Index:73 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:2  GPIO AD 10 EMVSIM2 PD */

      [ALT2].offset = IMXRT_INPUT_EMVSIM2_SIMPD_OFFSET,
      [ALT2].sel    = 0,

      /* Index:73 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:73 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:74 GPIO_AD_11 */

  {
    {
      /* Index:74 Alt:0  GPIO AD 11 USB OTG1 OC */

      [ALT0].offset = IMXRT_INPUT_USB_OTG1_OC_OFFSET,
      [ALT0].sel    = 0,

      /* Index:74 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:74 Alt:2  GPIO AD 11 EMVSIM2 POWER FAIL */

      [ALT2].offset = IMXRT_INPUT_EMVSIM2_POWER_FAIL_OFFSET,
      [ALT2].sel    = 0,

      /* Index:74 Alt:3  GPIO AD 11 GPT3 CLK */

      [ALT3].offset = IMXRT_INPUT_GPT3_CLKIN_OFFSET,
      [ALT3].sel    = 1,

      /* Index:74 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:74 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:74 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:74 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:74 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:74 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:74 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:74 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:75 GPIO_AD_12 */

  {
    {
      /* Index:75 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:3  GPIO AD 12 FLEXSPI1 B DATA03 */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1B_DATA3_OFFSET,
      [ALT3].sel    = 0,

      /* Index:75 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:75 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:76 GPIO_AD_13 */

  {
    {
      /* Index:76 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:3  GPIO AD 13 FLEXSPI1 B DATA02 */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1B_DATA2_OFFSET,
      [ALT3].sel    = 0,

      /* Index:76 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:76 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:77 GPIO_AD_14 */

  {
    {
      /* Index:77 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:3  GPIO AD 14 FLEXSPI1 B DATA01 */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1B_DATA1_OFFSET,
      [ALT3].sel    = 0,

      /* Index:77 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:77 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:78 GPIO_AD_15 */

  {
    {
      /* Index:78 Alt:0  GPIO AD 15 SPDIF IN */

      [ALT0].offset = IMXRT_INPUT_SPDIF_IN_OFFSET,
      [ALT0].sel    = 1,

      /* Index:78 Alt:1  GPIO AD 15 LPUART10 TXD */

      [ALT1].offset = IMXRT_INPUT_LPUART10_TX_OFFSET,
      [ALT1].sel    = 0,

      /* Index:78 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:78 Alt:3  GPIO AD 15 FLEXSPI1 B DATA00 */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1B_DATA0_OFFSET,
      [ALT3].sel    = 0,

      /* Index:78 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:78 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:78 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:78 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:78 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:78 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:78 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:78 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:79 GPIO_AD_16 */

  {
    {
      /* Index:79 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:79 Alt:1  GPIO AD 16 LPUART10 RXD */

      [ALT1].offset = IMXRT_INPUT_LPUART10_RX_OFFSET,
      [ALT1].sel    = 0,

      /* Index:79 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:79 Alt:3  GPIO AD 16 FLEXSPI1 B SCLK */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1B_SCK_OFFSET,
      [ALT3].sel    = 0,

      /* Index:79 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:79 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:79 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:79 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:79 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:79 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:79 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:79 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:80 GPIO_AD_17 */

  {
    {
      /* Index:80 Alt:0  GPIO AD 17 SAI1 MCLK */

      [ALT0].offset = IMXRT_INPUT_SAI1_MCLK_OFFSET,
      [ALT0].sel    = 0,

      /* Index:80 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:80 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:80 Alt:3  GPIO AD 17 FLEXSPI1 A DQS */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1A_DQS_OFFSET,
      [ALT3].sel    = 1,

      /* Index:80 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:80 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:80 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:80 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:80 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:80 Alt:9  GPIO AD 17 ENET 1G MDIO */

      [ALT9].offset = IMXRT_INPUT_ENET_1G_MDIO_OFFSET,
      [ALT9].sel    = 2,

      /* Index:80 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:80 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:81 GPIO_AD_18 */

  {
    {
      /* Index:81 Alt:0  GPIO AD 18 SAI1 RX SYNC */

      [ALT0].offset = IMXRT_INPUT_SAI1_RX_SYNC_OFFSET,
      [ALT0].sel    = 0,

      /* Index:81 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:81 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:81 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:81 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:81 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:81 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:81 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:81 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:81 Alt:9  GPIO AD 18 LPI2C2 SCL */

      [ALT9].offset = IMXRT_INPUT_LPI2C2_SCL_OFFSET,
      [ALT9].sel    = 1,

      /* Index:81 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:81 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:82 GPIO_AD_19 */

  {
    {
      /* Index:82 Alt:0  GPIO AD 19 SAI1 RX BCLK */

      [ALT0].offset = IMXRT_INPUT_SAI1_RX_BCLK_OFFSET,
      [ALT0].sel    = 0,

      /* Index:82 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:82 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:82 Alt:3  GPIO AD 19 FLEXSPI1 A SCLK */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1A_SCK_OFFSET,
      [ALT3].sel    = 0,

      /* Index:82 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:82 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:82 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:82 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:82 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:82 Alt:9  GPIO AD 19 LPI2C2 SDA */

      [ALT9].offset = IMXRT_INPUT_LPI2C2_SDA_OFFSET,
      [ALT9].sel    = 1,

      /* Index:82 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:82 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:83 GPIO_AD_20 */

  {
    {
      /* Index:83 Alt:0  GPIO AD 20 SAI1 RX DATA00 */

      [ALT0].offset = IMXRT_INPUT_SAI1_RX_DATA0_OFFSET,
      [ALT0].sel    = 0,

      /* Index:83 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:83 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:83 Alt:3  GPIO AD 20 FLEXSPI1 A DATA00 */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1A_DATA0_OFFSET,
      [ALT3].sel    = 0,

      /* Index:83 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:83 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:83 Alt:6  GPIO AD 20 KPP ROW07 */

      [ALT6].offset = IMXRT_INPUT_KPP_ROW7_OFFSET,
      [ALT6].sel    = 0,

      /* Index:83 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:83 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:83 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:83 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:83 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:84 GPIO_AD_21 */

  {
    {
      /* Index:84 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:84 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:84 Alt:2  GPIO AD 21 LPSPI2 PCS1 */

      [ALT2].offset = IMXRT_INPUT_LPSPI2_PCS1_OFFSET,
      [ALT2].sel    = 0,

      /* Index:84 Alt:3  GPIO AD 21 FLEXSPI1 A DATA01 */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1A_DATA1_OFFSET,
      [ALT3].sel    = 0,

      /* Index:84 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:84 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:84 Alt:6  GPIO AD 21 KPP COL07 */

      [ALT6].offset = IMXRT_INPUT_KPP_COL7_OFFSET,
      [ALT6].sel    = 0,

      /* Index:84 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:84 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:84 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:84 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:84 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:85 GPIO_AD_22 */

  {
    {
      /* Index:85 Alt:0  GPIO AD 22 SAI1 TX BCLK */

      [ALT0].offset = IMXRT_INPUT_SAI1_TX_BCLK_OFFSET,
      [ALT0].sel    = 0,

      /* Index:85 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:85 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:85 Alt:3  GPIO AD 22 FLEXSPI1 A DATA02 */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1A_DATA2_OFFSET,
      [ALT3].sel    = 0,

      /* Index:85 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:85 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:85 Alt:6  GPIO AD 22 KPP ROW06 */

      [ALT6].offset = IMXRT_INPUT_KPP_ROW6_OFFSET,
      [ALT6].sel    = 0,

      /* Index:85 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:85 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:85 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:85 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:85 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:86 GPIO_AD_23 */

  {
    {
      /* Index:86 Alt:0  GPIO AD 23 SAI1 TX SYNC */

      [ALT0].offset = IMXRT_INPUT_SAI1_TX_SYNC_OFFSET,
      [ALT0].sel    = 0,

      /* Index:86 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:86 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:86 Alt:3  GPIO AD 23 FLEXSPI1 A DATA03 */

      [ALT3].offset = IMXRT_INPUT_FLEXSPI1A_DATA3_OFFSET,
      [ALT3].sel    = 0,

      /* Index:86 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:86 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:86 Alt:6  GPIO AD 23 KPP COL06 */

      [ALT6].offset = IMXRT_INPUT_KPP_COL6_OFFSET,
      [ALT6].sel    = 0,

      /* Index:86 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:86 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:86 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:86 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:86 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:87 GPIO_AD_24 */

  {
    {
      /* Index:87 Alt:0  GPIO AD 24 LPUART1 TXD */

      [ALT0].offset = IMXRT_INPUT_LPUART1_TX_OFFSET,
      [ALT0].sel    = 0,

      /* Index:87 Alt:1  GPIO AD 24 LPSPI2 SCK */

      [ALT1].offset = IMXRT_INPUT_LPSPI2_SCK_OFFSET,
      [ALT1].sel    = 0,

      /* Index:87 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:87 Alt:3  GPIO AD 24 ENET RX EN */

      [ALT3].offset = IMXRT_INPUT_ENET_RXEN_OFFSET,
      [ALT3].sel    = 0,

      /* Index:87 Alt:4  GPIO AD 24 FLEXPWM2 PWM0 A */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM2_PWMA0_OFFSET,
      [ALT4].sel    = 1,

      /* Index:87 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:87 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:87 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:87 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:87 Alt:9  GPIO AD 24 LPI2C4 SCL */

      [ALT9].offset = IMXRT_INPUT_LPI2C4_SCL_OFFSET,
      [ALT9].sel    = 0,

      /* Index:87 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:87 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:88 GPIO_AD_25 */

  {
    {
      /* Index:88 Alt:0  GPIO AD 25 LPUART1 RXD */

      [ALT0].offset = IMXRT_INPUT_LPUART1_RX_OFFSET,
      [ALT0].sel    = 0,

      /* Index:88 Alt:1  GPIO AD 25 LPSPI2 PCS0 */

      [ALT1].offset = IMXRT_INPUT_LPSPI2_PCS0_OFFSET,
      [ALT1].sel    = 0,

      /* Index:88 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:88 Alt:3  GPIO AD 25 ENET RX ER */

      [ALT3].offset = IMXRT_INPUT_ENET_RXERR_OFFSET,
      [ALT3].sel    = 0,

      /* Index:88 Alt:4  GPIO AD 25 FLEXPWM2 PWM0 B */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM2_PWMB0_OFFSET,
      [ALT4].sel    = 1,

      /* Index:88 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:88 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:88 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:88 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:88 Alt:9  GPIO AD 25 LPI2C4 SDA */

      [ALT9].offset = IMXRT_INPUT_LPI2C4_SDA_OFFSET,
      [ALT9].sel    = 0,

      /* Index:88 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:88 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:89 GPIO_AD_26 */

  {
    {
      /* Index:89 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:89 Alt:1  GPIO AD 26 LPSPI2 SOUT */

      [ALT1].offset = IMXRT_INPUT_LPSPI2_SDO_OFFSET,
      [ALT1].sel    = 0,

      /* Index:89 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:89 Alt:3  GPIO AD 26 ENET RX DATA00 */

      [ALT3].offset = IMXRT_INPUT_ENET_RXDATA0_OFFSET,
      [ALT3].sel    = 0,

      /* Index:89 Alt:4  GPIO AD 26 FLEXPWM2 PWM1 A */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM2_PWMA1_OFFSET,
      [ALT4].sel    = 1,

      /* Index:89 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:89 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:89 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:89 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:89 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:89 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:89 Alt:11  GPIO AD 26 USDHC2 CD B */

      [ALT11].offset = IMXRT_INPUT_USDHC2_CD_B_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:90 GPIO_AD_27 */

  {
    {
      /* Index:90 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:90 Alt:1  GPIO AD 27 LPSPI2 SIN */

      [ALT1].offset = IMXRT_INPUT_LPSPI2_SDI_OFFSET,
      [ALT1].sel    = 0,

      /* Index:90 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:90 Alt:3  GPIO AD 27 ENET RX DATA01 */

      [ALT3].offset = IMXRT_INPUT_ENET_RXDATA1_OFFSET,
      [ALT3].sel    = 0,

      /* Index:90 Alt:4  GPIO AD 27 FLEXPWM2 PWM1 B */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM2_PWMB1_OFFSET,
      [ALT4].sel    = 1,

      /* Index:90 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:90 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:90 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:90 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:90 Alt:9  GPIO AD 27 ENET QOS MDIO */

      [ALT9].offset = IMXRT_INPUT_ENET_QOS_MDIO_OFFSET,
      [ALT9].sel    = 1,

      /* Index:90 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:90 Alt:11  GPIO AD 27 USDHC2 WP */

      [ALT11].offset = IMXRT_INPUT_USDHC2_WP_OFFSET,
      [ALT11].sel    = 1,
    },
  },

  /* index:91 GPIO_AD_28 */

  {
    {
      /* Index:91 Alt:0  GPIO AD 28 LPSPI1 SCK */

      [ALT0].offset = IMXRT_INPUT_LPSPI1_SCK_OFFSET,
      [ALT0].sel    = 1,

      /* Index:91 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:91 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:91 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:91 Alt:4  GPIO AD 28 FLEXPWM2 PWM2 A */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM2_PWMA2_OFFSET,
      [ALT4].sel    = 1,

      /* Index:91 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:91 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:91 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:91 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:91 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:91 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:91 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:92 GPIO_AD_29 */

  {
    {
      /* Index:92 Alt:0  GPIO AD 29 LPSPI1 PCS0 */

      [ALT0].offset = IMXRT_INPUT_LPSPI1_PCS0_OFFSET,
      [ALT0].sel    = 1,

      /* Index:92 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:92 Alt:2  GPIO AD 29 ENET REF CLK */

      [ALT2].offset = IMXRT_INPUT_ENET_IPG_CLK_RMII_OFFSET,
      [ALT2].sel    = 0,

      /* Index:92 Alt:3  GPIO AD 29 ENET TX CLK */

      [ALT3].offset = IMXRT_INPUT_ENET_TXCLK_OFFSET,
      [ALT3].sel    = 0,

      /* Index:92 Alt:4  GPIO AD 29 FLEXPWM2 PWM2 B */

      [ALT4].offset = IMXRT_INPUT_FLEXPWM2_PWMB2_OFFSET,
      [ALT4].sel    = 1,

      /* Index:92 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:92 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:92 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:92 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:92 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:92 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:92 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:93 GPIO_AD_30 */

  {
    {
      /* Index:93 Alt:0  GPIO AD 30 LPSPI1 SOUT */

      [ALT0].offset = IMXRT_INPUT_LPSPI1_SDO_OFFSET,
      [ALT0].sel    = 1,

      /* Index:93 Alt:1  GPIO AD 30 USB OTG2 OC */

      [ALT1].offset = IMXRT_INPUT_USB_OTG2_OC_OFFSET,
      [ALT1].sel    = 1,

      /* Index:93 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:93 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:93 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:93 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:93 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:93 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:93 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:93 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:93 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:93 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:94 GPIO_AD_31 */

  {
    {
      /* Index:94 Alt:0  GPIO AD 31 LPSPI1 SIN */

      [ALT0].offset = IMXRT_INPUT_LPSPI1_SDI_OFFSET,
      [ALT0].sel    = 1,

      /* Index:94 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:94 Alt:2  GPIO AD 31 FLEXCAN2 RX */

      [ALT2].offset = IMXRT_INPUT_FLEXCAN2_RX_OFFSET,
      [ALT2].sel    = 1,

      /* Index:94 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:94 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:94 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:94 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:94 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:94 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:94 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:94 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:94 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:95 GPIO_AD_32 */

  {
    {
      /* Index:95 Alt:0  GPIO AD 32 LPI2C1 SCL */

      [ALT0].offset = IMXRT_INPUT_LPI2C1_SCL_OFFSET,
      [ALT0].sel    = 1,

      /* Index:95 Alt:1  GPIO AD 32 USBPHY2 OTG ID */

      [ALT1].offset = IMXRT_INPUT_USBPHY2_USB_ID_OFFSET,
      [ALT1].sel    = 1,

      /* Index:95 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:95 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:95 Alt:4  GPIO AD 32 USDHC1 CD B */

      [ALT4].offset = IMXRT_INPUT_USDHC1_CD_B_OFFSET,
      [ALT4].sel    = 0,

      /* Index:95 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:95 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:95 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:95 Alt:8  GPIO AD 32 LPUART10 TXD */

      [ALT8].offset = IMXRT_INPUT_LPUART10_TX_OFFSET,
      [ALT8].sel    = 1,

      /* Index:95 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:95 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:95 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:96 GPIO_AD_33 */

  {
    {
      /* Index:96 Alt:0  GPIO AD 33 LPI2C1 SDA */

      [ALT0].offset = IMXRT_INPUT_LPI2C1_SDA_OFFSET,
      [ALT0].sel    = 1,

      /* Index:96 Alt:1  GPIO AD 33 USBPHY1 OTG ID */

      [ALT1].offset = IMXRT_INPUT_USBPHY1_USB_ID_OFFSET,
      [ALT1].sel    = 1,

      /* Index:96 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:96 Alt:3  GPIO AD 33 ENET MDIO */

      [ALT3].offset = IMXRT_INPUT_ENET_MDIO_OFFSET,
      [ALT3].sel    = 1,

      /* Index:96 Alt:4  GPIO AD 33 USDHC1 WP */

      [ALT4].offset = IMXRT_INPUT_USDHC1_WP_OFFSET,
      [ALT4].sel    = 0,

      /* Index:96 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:96 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:96 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:96 Alt:8  GPIO AD 33 LPUART10 RXD */

      [ALT8].offset = IMXRT_INPUT_LPUART10_RX_OFFSET,
      [ALT8].sel    = 1,

      /* Index:96 Alt:9  GPIO AD 33 ENET 1G MDIO */

      [ALT9].offset = IMXRT_INPUT_ENET_1G_MDIO_OFFSET,
      [ALT9].sel    = 3,

      /* Index:96 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:96 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:97 GPIO_AD_34 */

  {
    {
      /* Index:97 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:97 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:98 GPIO_AD_35 */

  {
    {
      /* Index:98 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:1  GPIO AD 35 USB OTG1 OC */

      [ALT1].offset = IMXRT_INPUT_USB_OTG1_OC_OFFSET,
      [ALT1].sel    = 1,

      /* Index:98 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:98 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:99 GPIO_SD_B1_00 */

  {
    {
      /* Index:99 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:99 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:99 Alt:2  GPIO SD B1 00 XBAR1 INOUT20 */

      [ALT2].offset = IMXRT_INPUT_XBAR1_IN20_OFFSET,
      [ALT2].sel    = 1,

      /* Index:99 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:99 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:99 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:99 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:99 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:99 Alt:8  GPIO SD B1 00 KPP ROW07 */

      [ALT8].offset = IMXRT_INPUT_KPP_ROW7_OFFSET,
      [ALT8].sel    = 1,

      /* Index:99 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:99 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:99 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:100 GPIO_SD_B1_01 */

  {
    {
      /* Index:100 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:100 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:100 Alt:2  GPIO SD B1 01 XBAR1 INOUT21 */

      [ALT2].offset = IMXRT_INPUT_XBAR1_IN21_OFFSET,
      [ALT2].sel    = 1,

      /* Index:100 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:100 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:100 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:100 Alt:6  GPIO SD B1 01 FLEXSPI2 A SCLK */

      [ALT6].offset = IMXRT_INPUT_FLEXSPI2A_SCK_OFFSET,
      [ALT6].sel    = 1,

      /* Index:100 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:100 Alt:8  GPIO SD B1 01 KPP COL07 */

      [ALT8].offset = IMXRT_INPUT_KPP_COL7_OFFSET,
      [ALT8].sel    = 1,

      /* Index:100 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:100 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:100 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:101 GPIO_SD_B1_02 */

  {
    {
      /* Index:101 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:101 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:101 Alt:2  GPIO SD B1 02 XBAR1 INOUT22 */

      [ALT2].offset = IMXRT_INPUT_XBAR1_IN22_OFFSET,
      [ALT2].sel    = 1,

      /* Index:101 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:101 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:101 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:101 Alt:6  GPIO SD B1 02 FLEXSPI2 A DATA00 */

      [ALT6].offset = IMXRT_INPUT_FLEXSPI2A_DATA0_OFFSET,
      [ALT6].sel    = 1,

      /* Index:101 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:101 Alt:8  GPIO SD B1 02 KPP ROW06 */

      [ALT8].offset = IMXRT_INPUT_KPP_ROW6_OFFSET,
      [ALT8].sel    = 1,

      /* Index:101 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:101 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:101 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:102 GPIO_SD_B1_03 */

  {
    {
      /* Index:102 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:102 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:102 Alt:2  GPIO SD B1 03 XBAR1 INOUT23 */

      [ALT2].offset = IMXRT_INPUT_XBAR1_IN23_OFFSET,
      [ALT2].sel    = 1,

      /* Index:102 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:102 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:102 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:102 Alt:6  GPIO SD B1 03 FLEXSPI2 A DATA01 */

      [ALT6].offset = IMXRT_INPUT_FLEXSPI2A_DATA1_OFFSET,
      [ALT6].sel    = 1,

      /* Index:102 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:102 Alt:8  GPIO SD B1 03 KPP COL06 */

      [ALT8].offset = IMXRT_INPUT_KPP_COL6_OFFSET,
      [ALT8].sel    = 1,

      /* Index:102 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:102 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:102 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:103 GPIO_SD_B1_04 */

  {
    {
      /* Index:103 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:103 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:103 Alt:2  GPIO SD B1 04 XBAR1 INOUT24 */

      [ALT2].offset = IMXRT_INPUT_XBAR1_IN24_OFFSET,
      [ALT2].sel    = 1,

      /* Index:103 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:103 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:103 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:103 Alt:6  GPIO SD B1 04 FLEXSPI2 A DATA02 */

      [ALT6].offset = IMXRT_INPUT_FLEXSPI2A_DATA2_OFFSET,
      [ALT6].sel    = 1,

      /* Index:103 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:103 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:103 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:103 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:103 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:104 GPIO_SD_B1_05 */

  {
    {
      /* Index:104 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:104 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:104 Alt:2  GPIO SD B1 05 XBAR1 INOUT25 */

      [ALT2].offset = IMXRT_INPUT_XBAR1_IN25_OFFSET,
      [ALT2].sel    = 1,

      /* Index:104 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:104 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:104 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:104 Alt:6  GPIO SD B1 05 FLEXSPI2 A DATA03 */

      [ALT6].offset = IMXRT_INPUT_FLEXSPI2A_DATA3_OFFSET,
      [ALT6].sel    = 1,

      /* Index:104 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:104 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:104 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:104 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:104 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:105 GPIO_SD_B2_00 */

  {
    {
      /* Index:105 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:105 Alt:1  GPIO SD B2 00 FLEXSPI1 B DATA03 */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1B_DATA3_OFFSET,
      [ALT1].sel    = 1,

      /* Index:105 Alt:2  GPIO SD B2 00 ENET 1G RX EN */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXEN_OFFSET,
      [ALT2].sel    = 1,

      /* Index:105 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:105 Alt:4  GPIO SD B2 00 LPSPI4 SCK */

      [ALT4].offset = IMXRT_INPUT_LPSPI4_SCK_OFFSET,
      [ALT4].sel    = 0,

      /* Index:105 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:105 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:105 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:105 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:105 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:105 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:105 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:106 GPIO_SD_B2_01 */

  {
    {
      /* Index:106 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:106 Alt:1  GPIO SD B2 01 FLEXSPI1 B DATA02 */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1B_DATA2_OFFSET,
      [ALT1].sel    = 1,

      /* Index:106 Alt:2  GPIO SD B2 01 ENET 1G RX CLK */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXCLK_OFFSET,
      [ALT2].sel    = 1,

      /* Index:106 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:106 Alt:4  GPIO SD B2 01 LPSPI4 PCS0 */

      [ALT4].offset = IMXRT_INPUT_LPSPI4_PCS0_OFFSET,
      [ALT4].sel    = 0,

      /* Index:106 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:106 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:106 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:106 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:106 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:106 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:106 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:107 GPIO_SD_B2_02 */

  {
    {
      /* Index:107 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:107 Alt:1  GPIO SD B2 02 FLEXSPI1 B DATA01 */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1B_DATA1_OFFSET,
      [ALT1].sel    = 1,

      /* Index:107 Alt:2  GPIO SD B2 02 ENET 1G RX DATA00 */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXDATA0_OFFSET,
      [ALT2].sel    = 1,

      /* Index:107 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:107 Alt:4  GPIO SD B2 02 LPSPI4 SOUT */

      [ALT4].offset = IMXRT_INPUT_LPSPI4_SDO_OFFSET,
      [ALT4].sel    = 0,

      /* Index:107 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:107 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:107 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:107 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:107 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:107 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:107 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:108 GPIO_SD_B2_03 */

  {
    {
      /* Index:108 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:108 Alt:1  GPIO SD B2 03 FLEXSPI1 B DATA00 */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1B_DATA0_OFFSET,
      [ALT1].sel    = 1,

      /* Index:108 Alt:2  GPIO SD B2 03 ENET 1G RX DATA01 */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXDATA1_OFFSET,
      [ALT2].sel    = 1,

      /* Index:108 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:108 Alt:4  GPIO SD B2 03 LPSPI4 SIN */

      [ALT4].offset = IMXRT_INPUT_LPSPI4_SDI_OFFSET,
      [ALT4].sel    = 0,

      /* Index:108 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:108 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:108 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:108 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:108 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:108 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:108 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:109 GPIO_SD_B2_04 */

  {
    {
      /* Index:109 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:109 Alt:1  GPIO SD B2 04 FLEXSPI1 B SCLK */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1B_SCK_OFFSET,
      [ALT1].sel    = 1,

      /* Index:109 Alt:2  GPIO SD B2 04 ENET 1G RX DATA02 */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXDATA2_OFFSET,
      [ALT2].sel    = 1,

      /* Index:109 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:109 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:109 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:109 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:109 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:109 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:109 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:109 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:109 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:110 GPIO_SD_B2_05 */

  {
    {
      /* Index:110 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:110 Alt:1  GPIO SD B2 05 FLEXSPI1 A DQS */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1A_DQS_OFFSET,
      [ALT1].sel    = 2,

      /* Index:110 Alt:2  GPIO SD B2 05 ENET 1G RX DATA03 */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXDATA3_OFFSET,
      [ALT2].sel    = 1,

      /* Index:110 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:110 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:110 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:110 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:110 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:110 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:110 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:110 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:110 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:111 GPIO_SD_B2_06 */

  {
    {
      /* Index:111 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:111 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:112 GPIO_SD_B2_07 */

  {
    {
      /* Index:112 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:112 Alt:1  GPIO SD B2 07 FLEXSPI1 A SCLK */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1A_SCK_OFFSET,
      [ALT1].sel    = 1,

      /* Index:112 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:112 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:112 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:112 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:112 Alt:6  GPIO SD B2 07 LPSPI2 SCK */

      [ALT6].offset = IMXRT_INPUT_LPSPI2_SCK_OFFSET,
      [ALT6].sel    = 1,

      /* Index:112 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:112 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:112 Alt:9  GPIO SD B2 07 ENET QOS REF CLK */

      [ALT9].offset = IMXRT_INPUT_CCM_ENET_QOS_REFCLK_OFFSET,
      [ALT9].sel    = 1,

      /* Index:112 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:112 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:113 GPIO_SD_B2_08 */

  {
    {
      /* Index:113 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:113 Alt:1  GPIO SD B2 08 FLEXSPI1 A DATA00 */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1A_DATA0_OFFSET,
      [ALT1].sel    = 1,

      /* Index:113 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:113 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:113 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:113 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:113 Alt:6  GPIO SD B2 08 LPSPI2 PCS0 */

      [ALT6].offset = IMXRT_INPUT_LPSPI2_PCS0_OFFSET,
      [ALT6].sel    = 1,

      /* Index:113 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:113 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:113 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:113 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:113 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:114 GPIO_SD_B2_09 */

  {
    {
      /* Index:114 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:114 Alt:1  GPIO SD B2 09 FLEXSPI1 A DATA01 */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1A_DATA1_OFFSET,
      [ALT1].sel    = 1,

      /* Index:114 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:114 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:114 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:114 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:114 Alt:6  GPIO SD B2 09 LPSPI2 SOUT */

      [ALT6].offset = IMXRT_INPUT_LPSPI2_SDO_OFFSET,
      [ALT6].sel    = 1,

      /* Index:114 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:114 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:114 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:114 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:114 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:115 GPIO_SD_B2_10 */

  {
    {
      /* Index:115 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:115 Alt:1  GPIO SD B2 10 FLEXSPI1 A DATA02 */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1A_DATA2_OFFSET,
      [ALT1].sel    = 1,

      /* Index:115 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:115 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:115 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:115 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:115 Alt:6  GPIO SD B2 10 LPSPI2 SIN */

      [ALT6].offset = IMXRT_INPUT_LPSPI2_SDI_OFFSET,
      [ALT6].sel    = 1,

      /* Index:115 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:115 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:115 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:115 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:115 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:116 GPIO_SD_B2_11 */

  {
    {
      /* Index:116 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:116 Alt:1  GPIO SD B2 11 FLEXSPI1 A DATA03 */

      [ALT1].offset = IMXRT_INPUT_FLEXSPI1A_DATA3_OFFSET,
      [ALT1].sel    = 1,

      /* Index:116 Alt:2  GPIO SD B2 11 ENET 1G TX CLK IO */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_TXCLK_OFFSET,
      [ALT2].sel    = 1,

      /* Index:116 Alt:3  GPIO SD B2 11 ENET 1G REF CLK */

      [ALT3].offset = IMXRT_INPUT_ENET_1G_IPG_CLK_RMII_OFFSET,
      [ALT3].sel    = 1,

      /* Index:116 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:116 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:116 Alt:6  GPIO SD B2 11 LPSPI2 PCS1 */

      [ALT6].offset = IMXRT_INPUT_LPSPI2_PCS1_OFFSET,
      [ALT6].sel    = 1,

      /* Index:116 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:116 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:116 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:116 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:116 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:117 GPIO_DISP_B1_00 */

  {
    {
      /* Index:117 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:117 Alt:1  GPIO DISP B1 00 ENET 1G RX EN */

      [ALT1].offset = IMXRT_INPUT_ENET_1G_RXEN_OFFSET,
      [ALT1].sel    = 2,

      /* Index:117 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:117 Alt:3  GPIO DISP B1 00 TMR1 TIMER0 */

      [ALT3].offset = IMXRT_INPUT_QTIMER1_TIMER0_OFFSET,
      [ALT3].sel    = 2,

      /* Index:117 Alt:4  GPIO DISP B1 00 XBAR1 INOUT26 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN26_OFFSET,
      [ALT4].sel    = 1,

      /* Index:117 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:117 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:117 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:117 Alt:8  GPIO DISP B1 00 ENET QOS RX EN */

      [ALT8].offset = IMXRT_INPUT_ENET_QOS_RXDV_OFFSET,
      [ALT8].sel    = 0,

      /* Index:117 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:117 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:117 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:118 GPIO_DISP_B1_01 */

  {
    {
      /* Index:118 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:118 Alt:1  GPIO DISP B1 01 ENET 1G RX CLK */

      [ALT1].offset = IMXRT_INPUT_ENET_1G_RXCLK_OFFSET,
      [ALT1].sel    = 2,

      /* Index:118 Alt:2  GPIO DISP B1 01 ENET 1G RX ER */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_RXERR_OFFSET,
      [ALT2].sel    = 1,

      /* Index:118 Alt:3  GPIO DISP B1 01 TMR1 TIMER1 */

      [ALT3].offset = IMXRT_INPUT_QTIMER1_TIMER1_OFFSET,
      [ALT3].sel    = 2,

      /* Index:118 Alt:4  GPIO DISP B1 01 XBAR1 INOUT27 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN27_OFFSET,
      [ALT4].sel    = 1,

      /* Index:118 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:118 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:118 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:118 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:118 Alt:9  GPIO DISP B1 01 ENET QOS RX ER */

      [ALT9].offset = IMXRT_INPUT_ENET_QOS_RXERR_OFFSET,
      [ALT9].sel    = 0,

      /* Index:118 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:118 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:119 GPIO_DISP_B1_02 */

  {
    {
      /* Index:119 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:119 Alt:1  GPIO DISP B1 02 ENET 1G RX DATA00 */

      [ALT1].offset = IMXRT_INPUT_ENET_1G_RXDATA0_OFFSET,
      [ALT1].sel    = 2,

      /* Index:119 Alt:2  GPIO DISP B1 02 LPI2C3 SCL */

      [ALT2].offset = IMXRT_INPUT_LPI2C3_SCL_OFFSET,
      [ALT2].sel    = 0,

      /* Index:119 Alt:3  GPIO DISP B1 02 TMR1 TIMER2 */

      [ALT3].offset = IMXRT_INPUT_QTIMER1_TIMER2_OFFSET,
      [ALT3].sel    = 1,

      /* Index:119 Alt:4  GPIO DISP B1 02 XBAR1 INOUT28 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN28_OFFSET,
      [ALT4].sel    = 1,

      /* Index:119 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:119 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:119 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:119 Alt:8  GPIO DISP B1 02 ENET QOS RX DATA00 */

      [ALT8].offset = IMXRT_INPUT_ENET_QOS_RXDATA0_OFFSET,
      [ALT8].sel    = 0,

      /* Index:119 Alt:9  GPIO DISP B1 02 LPUART1 TXD */

      [ALT9].offset = IMXRT_INPUT_LPUART1_TX_OFFSET,
      [ALT9].sel    = 1,

      /* Index:119 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:119 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:120 GPIO_DISP_B1_03 */

  {
    {
      /* Index:120 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:120 Alt:1  GPIO DISP B1 03 ENET 1G RX DATA01 */

      [ALT1].offset = IMXRT_INPUT_ENET_1G_RXDATA1_OFFSET,
      [ALT1].sel    = 2,

      /* Index:120 Alt:2  GPIO DISP B1 03 LPI2C3 SDA */

      [ALT2].offset = IMXRT_INPUT_LPI2C3_SDA_OFFSET,
      [ALT2].sel    = 0,

      /* Index:120 Alt:3  GPIO DISP B1 03 TMR2 TIMER0 */

      [ALT3].offset = IMXRT_INPUT_QTIMER2_TIMER0_OFFSET,
      [ALT3].sel    = 2,

      /* Index:120 Alt:4  GPIO DISP B1 03 XBAR1 INOUT29 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN29_OFFSET,
      [ALT4].sel    = 1,

      /* Index:120 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:120 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:120 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:120 Alt:8  GPIO DISP B1 03 ENET QOS RX DATA01 */

      [ALT8].offset = IMXRT_INPUT_ENET_QOS_RXDATA1_OFFSET,
      [ALT8].sel    = 0,

      /* Index:120 Alt:9  GPIO DISP B1 03 LPUART1 RXD */

      [ALT9].offset = IMXRT_INPUT_LPUART1_RX_OFFSET,
      [ALT9].sel    = 1,

      /* Index:120 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:120 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:121 GPIO_DISP_B1_04 */

  {
    {
      /* Index:121 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:121 Alt:1  GPIO DISP B1 04 ENET 1G RX DATA02 */

      [ALT1].offset = IMXRT_INPUT_ENET_1G_RXDATA2_OFFSET,
      [ALT1].sel    = 2,

      /* Index:121 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:121 Alt:3  GPIO DISP B1 04 TMR2 TIMER1 */

      [ALT3].offset = IMXRT_INPUT_QTIMER2_TIMER1_OFFSET,
      [ALT3].sel    = 2,

      /* Index:121 Alt:4  GPIO DISP B1 04 XBAR1 INOUT30 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN30_OFFSET,
      [ALT4].sel    = 1,

      /* Index:121 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:121 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:121 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:121 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:121 Alt:9  GPIO DISP B1 04 LPSPI3 SCK */

      [ALT9].offset = IMXRT_INPUT_LPSPI3_SCK_OFFSET,
      [ALT9].sel    = 1,

      /* Index:121 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:121 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:122 GPIO_DISP_B1_05 */

  {
    {
      /* Index:122 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:122 Alt:1  GPIO DISP B1 05 ENET 1G RX DATA03 */

      [ALT1].offset = IMXRT_INPUT_ENET_1G_RXDATA3_OFFSET,
      [ALT1].sel    = 2,

      /* Index:122 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:122 Alt:3  GPIO DISP B1 05 TMR2 TIMER2 */

      [ALT3].offset = IMXRT_INPUT_QTIMER2_TIMER2_OFFSET,
      [ALT3].sel    = 1,

      /* Index:122 Alt:4  GPIO DISP B1 05 XBAR1 INOUT31 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN31_OFFSET,
      [ALT4].sel    = 1,

      /* Index:122 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:122 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:122 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:122 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:122 Alt:9  GPIO DISP B1 05 LPSPI3 SIN */

      [ALT9].offset = IMXRT_INPUT_LPSPI3_SDI_OFFSET,
      [ALT9].sel    = 1,

      /* Index:122 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:122 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:123 GPIO_DISP_B1_06 */

  {
    {
      /* Index:123 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:123 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:123 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:123 Alt:3  GPIO DISP B1 06 TMR3 TIMER0 */

      [ALT3].offset = IMXRT_INPUT_QTIMER3_TIMER0_OFFSET,
      [ALT3].sel    = 2,

      /* Index:123 Alt:4  GPIO DISP B1 06 XBAR1 INOUT32 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN32_OFFSET,
      [ALT4].sel    = 1,

      /* Index:123 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:123 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:123 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:123 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:123 Alt:9  GPIO DISP B1 06 LPSPI3 SOUT */

      [ALT9].offset = IMXRT_INPUT_LPSPI3_SDO_OFFSET,
      [ALT9].sel    = 1,

      /* Index:123 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:123 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:124 GPIO_DISP_B1_07 */

  {
    {
      /* Index:124 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:124 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:124 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:124 Alt:3  GPIO DISP B1 07 TMR3 TIMER1 */

      [ALT3].offset = IMXRT_INPUT_QTIMER3_TIMER1_OFFSET,
      [ALT3].sel    = 2,

      /* Index:124 Alt:4  GPIO DISP B1 07 XBAR1 INOUT33 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN33_OFFSET,
      [ALT4].sel    = 1,

      /* Index:124 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:124 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:124 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:124 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:124 Alt:9  GPIO DISP B1 07 LPSPI3 PCS0 */

      [ALT9].offset = IMXRT_INPUT_LPSPI3_PCS0_OFFSET,
      [ALT9].sel    = 1,

      /* Index:124 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:124 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:125 GPIO_DISP_B1_08 */

  {
    {
      /* Index:125 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:125 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:125 Alt:2  GPIO DISP B1 08 USDHC1 CD B */

      [ALT2].offset = IMXRT_INPUT_USDHC1_CD_B_OFFSET,
      [ALT2].sel    = 1,

      /* Index:125 Alt:3  GPIO DISP B1 08 TMR3 TIMER2 */

      [ALT3].offset = IMXRT_INPUT_QTIMER3_TIMER2_OFFSET,
      [ALT3].sel    = 1,

      /* Index:125 Alt:4  GPIO DISP B1 08 XBAR1 INOUT34 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN34_OFFSET,
      [ALT4].sel    = 1,

      /* Index:125 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:125 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:125 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:125 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:125 Alt:9  GPIO DISP B1 08 LPSPI3 PCS1 */

      [ALT9].offset = IMXRT_INPUT_LPSPI3_PCS1_OFFSET,
      [ALT9].sel    = 1,

      /* Index:125 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:125 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:126 GPIO_DISP_B1_09 */

  {
    {
      /* Index:126 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:126 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:126 Alt:2  GPIO DISP B1 09 USDHC1 WP */

      [ALT2].offset = IMXRT_INPUT_USDHC1_WP_OFFSET,
      [ALT2].sel    = 1,

      /* Index:126 Alt:3  GPIO DISP B1 09 TMR4 TIMER0 */

      [ALT3].offset = IMXRT_INPUT_QTIMER4_TIMER0_OFFSET,
      [ALT3].sel    = 2,

      /* Index:126 Alt:4  GPIO DISP B1 09 XBAR1 INOUT35 */

      [ALT4].offset = IMXRT_INPUT_XBAR1_IN35_OFFSET,
      [ALT4].sel    = 1,

      /* Index:126 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:126 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:126 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:126 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:126 Alt:9  GPIO DISP B1 09 LPSPI3 PCS2 */

      [ALT9].offset = IMXRT_INPUT_LPSPI3_PCS2_OFFSET,
      [ALT9].sel    = 1,

      /* Index:126 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:126 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:127 GPIO_DISP_B1_10 */

  {
    {
      /* Index:127 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:127 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:127 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:127 Alt:3  GPIO DISP B1 10 TMR4 TIMER1 */

      [ALT3].offset = IMXRT_INPUT_QTIMER4_TIMER1_OFFSET,
      [ALT3].sel    = 2,

      /* Index:127 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:127 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:127 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:127 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:127 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:127 Alt:9  GPIO DISP B1 10 LPSPI3 PCS3 */

      [ALT9].offset = IMXRT_INPUT_LPSPI3_PCS3_OFFSET,
      [ALT9].sel    = 1,

      /* Index:127 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:127 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:128 GPIO_DISP_B1_11 */

  {
    {
      /* Index:128 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:128 Alt:1  GPIO DISP B1 11 ENET 1G TX CLK IO */

      [ALT1].offset = IMXRT_INPUT_ENET_1G_TXCLK_OFFSET,
      [ALT1].sel    = 2,

      /* Index:128 Alt:2  GPIO DISP B1 11 ENET 1G REF CLK */

      [ALT2].offset = IMXRT_INPUT_ENET_1G_IPG_CLK_RMII_OFFSET,
      [ALT2].sel    = 2,

      /* Index:128 Alt:3  GPIO DISP B1 11 TMR4 TIMER2 */

      [ALT3].offset = IMXRT_INPUT_QTIMER4_TIMER2_OFFSET,
      [ALT3].sel    = 1,

      /* Index:128 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:128 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:128 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:128 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:128 Alt:8  GPIO DISP B1 11 ENET QOS TX CLK */

      [ALT8].offset = IMXRT_INPUT_CCM_ENET_QOS_TXCLK_OFFSET,
      [ALT8].sel    = 0,

      /* Index:128 Alt:9  GPIO DISP B1 11 ENET QOS REF CLK */

      [ALT9].offset = IMXRT_INPUT_CCM_ENET_QOS_REFCLK_OFFSET,
      [ALT9].sel    = 2,

      /* Index:128 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:128 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:129 GPIO_DISP_B2_00 */

  {
    {
      /* Index:129 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:129 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:130 GPIO_DISP_B2_01 */

  {
    {
      /* Index:130 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:130 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:131 GPIO_DISP_B2_02 */

  {
    {
      /* Index:131 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:131 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:132 GPIO_DISP_B2_03 */

  {
    {
      /* Index:132 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:4  GPIO DISP B2 03 SAI1 MCLK */

      [ALT4].offset = IMXRT_INPUT_SAI1_MCLK_OFFSET,
      [ALT4].sel    = 1,

      /* Index:132 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:132 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:133 GPIO_DISP_B2_04 */

  {
    {
      /* Index:133 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:4  GPIO DISP B2 04 SAI1 RX SYNC */

      [ALT4].offset = IMXRT_INPUT_SAI1_RX_SYNC_OFFSET,
      [ALT4].sel    = 1,

      /* Index:133 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:133 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:134 GPIO_DISP_B2_05 */

  {
    {
      /* Index:134 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:134 Alt:1  GPIO DISP B2 05 ENET TX CLK */

      [ALT1].offset = IMXRT_INPUT_ENET_TXCLK_OFFSET,
      [ALT1].sel    = 1,

      /* Index:134 Alt:2  GPIO DISP B2 05 ENET REF CLK */

      [ALT2].offset = IMXRT_INPUT_ENET_IPG_CLK_RMII_OFFSET,
      [ALT2].sel    = 1,

      /* Index:134 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:134 Alt:4  GPIO DISP B2 05 SAI1 RX BCLK */

      [ALT4].offset = IMXRT_INPUT_SAI1_RX_BCLK_OFFSET,
      [ALT4].sel    = 1,

      /* Index:134 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:134 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:134 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:134 Alt:8  GPIO DISP B2 05 ENET QOS TX CLK */

      [ALT8].offset = IMXRT_INPUT_CCM_ENET_QOS_TXCLK_OFFSET,
      [ALT8].sel    = 1,

      /* Index:134 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:134 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:134 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:135 GPIO_DISP_B2_06 */

  {
    {
      /* Index:135 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:135 Alt:1  GPIO DISP B2 06 ENET RX DATA00 */

      [ALT1].offset = IMXRT_INPUT_ENET_RXDATA0_OFFSET,
      [ALT1].sel    = 1,

      /* Index:135 Alt:2  GPIO DISP B2 06 LPUART7 TXD */

      [ALT2].offset = IMXRT_INPUT_LPUART7_TX_OFFSET,
      [ALT2].sel    = 1,

      /* Index:135 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:135 Alt:4  GPIO DISP B2 06 SAI1 RX DATA00 */

      [ALT4].offset = IMXRT_INPUT_SAI1_RX_DATA0_OFFSET,
      [ALT4].sel    = 1,

      /* Index:135 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:135 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:135 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:135 Alt:8  GPIO DISP B2 06 ENET QOS RX DATA00 */

      [ALT8].offset = IMXRT_INPUT_ENET_QOS_RXDATA0_OFFSET,
      [ALT8].sel    = 1,

      /* Index:135 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:135 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:135 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:136 GPIO_DISP_B2_07 */

  {
    {
      /* Index:136 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:136 Alt:1  GPIO DISP B2 07 ENET RX DATA01 */

      [ALT1].offset = IMXRT_INPUT_ENET_RXDATA1_OFFSET,
      [ALT1].sel    = 1,

      /* Index:136 Alt:2  GPIO DISP B2 07 LPUART7 RXD */

      [ALT2].offset = IMXRT_INPUT_LPUART7_RX_OFFSET,
      [ALT2].sel    = 1,

      /* Index:136 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:136 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:136 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:136 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:136 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:136 Alt:8  GPIO DISP B2 07 ENET QOS RX DATA01 */

      [ALT8].offset = IMXRT_INPUT_ENET_QOS_RXDATA1_OFFSET,
      [ALT8].sel    = 1,

      /* Index:136 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:136 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:136 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:137 GPIO_DISP_B2_08 */

  {
    {
      /* Index:137 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:137 Alt:1  GPIO DISP B2 08 ENET RX EN */

      [ALT1].offset = IMXRT_INPUT_ENET_RXEN_OFFSET,
      [ALT1].sel    = 1,

      /* Index:137 Alt:2  GPIO DISP B2 08 LPUART8 TXD */

      [ALT2].offset = IMXRT_INPUT_LPUART8_TX_OFFSET,
      [ALT2].sel    = 1,

      /* Index:137 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:137 Alt:4  GPIO DISP B2 08 SAI1 TX BCLK */

      [ALT4].offset = IMXRT_INPUT_SAI1_TX_BCLK_OFFSET,
      [ALT4].sel    = 1,

      /* Index:137 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:137 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:137 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:137 Alt:8  GPIO DISP B2 08 ENET QOS RX EN */

      [ALT8].offset = IMXRT_INPUT_ENET_QOS_RXDV_OFFSET,
      [ALT8].sel    = 1,

      /* Index:137 Alt:9  GPIO DISP B2 08 LPUART1 TXD */

      [ALT9].offset = IMXRT_INPUT_LPUART1_TX_OFFSET,
      [ALT9].sel    = 2,

      /* Index:137 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:137 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:138 GPIO_DISP_B2_09 */

  {
    {
      /* Index:138 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:138 Alt:1  GPIO DISP B2 09 ENET RX ER */

      [ALT1].offset = IMXRT_INPUT_ENET_RXERR_OFFSET,
      [ALT1].sel    = 1,

      /* Index:138 Alt:2  GPIO DISP B2 09 LPUART8 RXD */

      [ALT2].offset = IMXRT_INPUT_LPUART8_RX_OFFSET,
      [ALT2].sel    = 1,

      /* Index:138 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:138 Alt:4  GPIO DISP B2 09 SAI1 TX SYNC */

      [ALT4].offset = IMXRT_INPUT_SAI1_TX_SYNC_OFFSET,
      [ALT4].sel    = 1,

      /* Index:138 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:138 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:138 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:138 Alt:8  GPIO DISP B2 09 ENET QOS RX ER */

      [ALT8].offset = IMXRT_INPUT_ENET_QOS_RXERR_OFFSET,
      [ALT8].sel    = 1,

      /* Index:138 Alt:9  GPIO DISP B2 09 LPUART1 RXD */

      [ALT9].offset = IMXRT_INPUT_LPUART1_RX_OFFSET,
      [ALT9].sel    = 2,

      /* Index:138 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:138 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:139 GPIO_DISP_B2_10 */

  {
    {
      /* Index:139 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:139 Alt:1  GPIO DISP B2 10 EMVSIM2 IO */

      [ALT1].offset = IMXRT_INPUT_EMVSIM2_SIO_OFFSET,
      [ALT1].sel    = 1,

      /* Index:139 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:139 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:139 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:139 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:139 Alt:6  GPIO DISP B2 10 LPI2C3 SCL */

      [ALT6].offset = IMXRT_INPUT_LPI2C3_SCL_OFFSET,
      [ALT6].sel    = 1,

      /* Index:139 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:139 Alt:8  GPIO DISP B2 10 ENET QOS RX ER */

      [ALT8].offset = IMXRT_INPUT_ENET_QOS_RXERR_OFFSET,
      [ALT8].sel    = 2,

      /* Index:139 Alt:9  GPIO DISP B2 10 SPDIF IN */

      [ALT9].offset = IMXRT_INPUT_SPDIF_IN_OFFSET,
      [ALT9].sel    = 2,

      /* Index:139 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:139 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:140 GPIO_DISP_B2_11 */

  {
    {
      /* Index:140 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:6  GPIO DISP B2 11 LPI2C3 SDA */

      [ALT6].offset = IMXRT_INPUT_LPI2C3_SDA_OFFSET,
      [ALT6].sel    = 1,

      /* Index:140 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:140 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:141 GPIO_DISP_B2_12 */

  {
    {
      /* Index:141 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:141 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:141 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:141 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:141 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:141 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:141 Alt:6  GPIO DISP B2 12 LPI2C4 SCL */

      [ALT6].offset = IMXRT_INPUT_LPI2C4_SCL_OFFSET,
      [ALT6].sel    = 1,

      /* Index:141 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:141 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:141 Alt:9  GPIO DISP B2 12 LPSPI4 SCK */

      [ALT9].offset = IMXRT_INPUT_LPSPI4_SCK_OFFSET,
      [ALT9].sel    = 1,

      /* Index:141 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:141 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:142 GPIO_DISP_B2_13 */

  {
    {
      /* Index:142 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:142 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:142 Alt:2  GPIO DISP B2 13 FLEXCAN1 RX */

      [ALT2].offset = IMXRT_INPUT_FLEXCAN1_RX_OFFSET,
      [ALT2].sel    = 1,

      /* Index:142 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:142 Alt:4  GPIO DISP B2 13 ENET REF CLK */

      [ALT4].offset = IMXRT_INPUT_ENET_IPG_CLK_RMII_OFFSET,
      [ALT4].sel    = 2,

      /* Index:142 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:142 Alt:6  GPIO DISP B2 13 LPI2C4 SDA */

      [ALT6].offset = IMXRT_INPUT_LPI2C4_SDA_OFFSET,
      [ALT6].sel    = 1,

      /* Index:142 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:142 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:142 Alt:9  GPIO DISP B2 13 LPSPI4 SIN */

      [ALT9].offset = IMXRT_INPUT_LPSPI4_SDI_OFFSET,
      [ALT9].sel    = 1,

      /* Index:142 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:142 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:143 GPIO_DISP_B2_14 */

  {
    {
      /* Index:143 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:143 Alt:1  GPIO DISP B2 14 EMVSIM2 PD */

      [ALT1].offset = IMXRT_INPUT_EMVSIM2_SIMPD_OFFSET,
      [ALT1].sel    = 1,

      /* Index:143 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:143 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:143 Alt:4  GPIO DISP B2 14 ENET 1G REF CLK */

      [ALT4].offset = IMXRT_INPUT_ENET_1G_IPG_CLK_RMII_OFFSET,
      [ALT4].sel    = 3,

      /* Index:143 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:143 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:143 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:143 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:143 Alt:9  GPIO DISP B2 14 LPSPI4 SOUT */

      [ALT9].offset = IMXRT_INPUT_LPSPI4_SDO_OFFSET,
      [ALT9].sel    = 1,

      /* Index:143 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:143 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:144 GPIO_DISP_B2_15 */

  {
    {
      /* Index:144 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:144 Alt:1  GPIO DISP B2 15 EMVSIM2 POWER FAIL */

      [ALT1].offset = IMXRT_INPUT_EMVSIM2_POWER_FAIL_OFFSET,
      [ALT1].sel    = 1,

      /* Index:144 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:144 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:144 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:144 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:144 Alt:6  GPIO DISP B2 15 FLEXCAN1 RX */

      [ALT6].offset = IMXRT_INPUT_FLEXCAN1_RX_OFFSET,
      [ALT6].sel    = 2,

      /* Index:144 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:144 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:144 Alt:9  GPIO DISP B2 15 LPSPI4 PCS0 */

      [ALT9].offset = IMXRT_INPUT_LPSPI4_PCS0_OFFSET,
      [ALT9].sel    = 1,

      /* Index:144 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:144 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:145 WAKEUP */

  {
    {
      /* Index:145 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:7  WAKEUP DIG NMI GLUE NMI */

      [ALT7].offset = IMXRT_INPUT_LPSR_NMI_GLUE_NMI_OFFSET,
      [ALT7].sel    = 1,

      /* Index:145 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:145 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:146 PMIC_ON_REQ */

  {
    {
      /* Index:146 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:146 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:147 PMIC_STBY_REQ */

  {
    {
      /* Index:147 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:147 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:148 GPIO_SNVS_00 */

  {
    {
      /* Index:148 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:148 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:149 GPIO_SNVS_01 */

  {
    {
      /* Index:149 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:149 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:150 GPIO_SNVS_02 */

  {
    {
      /* Index:150 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:150 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:151 GPIO_SNVS_03 */

  {
    {
      /* Index:151 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:151 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:152 GPIO_SNVS_04 */

  {
    {
      /* Index:152 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:152 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:153 GPIO_SNVS_05 */

  {
    {
      /* Index:153 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:153 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:154 GPIO_SNVS_06 */

  {
    {
      /* Index:154 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:154 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:155 GPIO_SNVS_07 */

  {
    {
      /* Index:155 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:155 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:156 GPIO_SNVS_08 */

  {
    {
      /* Index:156 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:156 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:157 GPIO_SNVS_09 */

  {
    {
      /* Index:157 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:157 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:158 GPIO_LPSR_00 */

  {
    {
      /* Index:158 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:158 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:158 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:158 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:158 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:158 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:158 Alt:6  GPIO LPSR 00 LPUART12 TXD */

      [ALT6].offset = IMXRT_INPUT_LPSR_LPUART12_TX_OFFSET,
      [ALT6].sel    = 0,

      /* Index:158 Alt:7  GPIO LPSR 00 SAI4 MCLK */

      [ALT7].offset = IMXRT_INPUT_LPSR_SAI4_MCLK_OFFSET,
      [ALT7].sel    = 0,

      /* Index:158 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:158 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:158 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:158 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:159 GPIO_LPSR_01 */

  {
    {
      /* Index:159 Alt:0  GPIO LPSR 01 FLEXCAN3 RX */

      [ALT0].offset = IMXRT_INPUT_LPSR_FLEXCAN3_RX_OFFSET,
      [ALT0].sel    = 0,

      /* Index:159 Alt:1  GPIO LPSR 01 MIC BITSTREAM0 */

      [ALT1].offset = IMXRT_INPUT_LPSR_PDM_BITSTREAM0_OFFSET,
      [ALT1].sel    = 0,

      /* Index:159 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:159 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:159 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:159 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:159 Alt:6  GPIO LPSR 01 LPUART12 RXD */

      [ALT6].offset = IMXRT_INPUT_LPSR_LPUART12_RX_OFFSET,
      [ALT6].sel    = 0,

      /* Index:159 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:159 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:159 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:159 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:159 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:160 GPIO_LPSR_02 */

  {
    {
      /* Index:160 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:1  GPIO LPSR 02 LPSPI5 SCK */

      [ALT1].offset = IMXRT_INPUT_LPSR_LPSPI5_SCK_OFFSET,
      [ALT1].sel    = 0,

      /* Index:160 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:160 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:161 GPIO_LPSR_03 */

  {
    {
      /* Index:161 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:161 Alt:1  GPIO LPSR 03 LPSPI5 PCS0 */

      [ALT1].offset = IMXRT_INPUT_LPSR_LPSPI5_PCS0_OFFSET,
      [ALT1].sel    = 0,

      /* Index:161 Alt:2  GPIO LPSR 03 SAI4 TX SYNC */

      [ALT2].offset = IMXRT_INPUT_LPSR_SAI4_TX_SYNC_OFFSET,
      [ALT2].sel    = 0,

      /* Index:161 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:161 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:161 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:161 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:161 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:161 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:161 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:161 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:161 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:162 GPIO_LPSR_04 */

  {
    {
      /* Index:162 Alt:0  GPIO LPSR 04 LPI2C5 SDA */

      [ALT0].offset = IMXRT_INPUT_LPSR_LPI2C5_SDA_OFFSET,
      [ALT0].sel    = 0,

      /* Index:162 Alt:1  GPIO LPSR 04 LPSPI5 SOUT */

      [ALT1].offset = IMXRT_INPUT_LPSR_LPSPI5_SDO_OFFSET,
      [ALT1].sel    = 0,

      /* Index:162 Alt:2  GPIO LPSR 04 SAI4 TX BCLK */

      [ALT2].offset = IMXRT_INPUT_LPSR_SAI4_TX_BCLK_OFFSET,
      [ALT2].sel    = 0,

      /* Index:162 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:162 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:162 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:162 Alt:6  GPIO LPSR 04 LPUART11 TXD */

      [ALT6].offset = IMXRT_INPUT_LPSR_LPUART11_TX_OFFSET,
      [ALT6].sel    = 0,

      /* Index:162 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:162 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:162 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:162 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:162 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:163 GPIO_LPSR_05 */

  {
    {
      /* Index:163 Alt:0  GPIO LPSR 05 LPI2C5 SCL */

      [ALT0].offset = IMXRT_INPUT_LPSR_LPI2C5_SCL_OFFSET,
      [ALT0].sel    = 0,

      /* Index:163 Alt:1  GPIO LPSR 05 LPSPI5 SIN */

      [ALT1].offset = IMXRT_INPUT_LPSR_LPSPI5_SDI_OFFSET,
      [ALT1].sel    = 0,

      /* Index:163 Alt:2  GPIO LPSR 05 SAI4 MCLK */

      [ALT2].offset = IMXRT_INPUT_LPSR_SAI4_MCLK_OFFSET,
      [ALT2].sel    = 1,

      /* Index:163 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:163 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:163 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:163 Alt:6  GPIO LPSR 05 LPUART11 RXD */

      [ALT6].offset = IMXRT_INPUT_LPSR_LPUART11_RX_OFFSET,
      [ALT6].sel    = 0,

      /* Index:163 Alt:7  GPIO LPSR 05 NMI GLUE NMI */

      [ALT7].offset = IMXRT_INPUT_LPSR_NMI_GLUE_NMI_OFFSET,
      [ALT7].sel    = 0,

      /* Index:163 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:163 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:163 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:163 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:164 GPIO_LPSR_06 */

  {
    {
      /* Index:164 Alt:0  GPIO LPSR 06 LPI2C6 SDA */

      [ALT0].offset = IMXRT_INPUT_LPSR_LPI2C6_SDA_OFFSET,
      [ALT0].sel    = 0,

      /* Index:164 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:164 Alt:2  GPIO LPSR 06 SAI4 RX DATA */

      [ALT2].offset = IMXRT_INPUT_LPSR_SAI4_RX_DATA0_OFFSET,
      [ALT2].sel    = 0,

      /* Index:164 Alt:3  GPIO LPSR 06 LPUART12 TXD */

      [ALT3].offset = IMXRT_INPUT_LPSR_LPUART12_TX_OFFSET,
      [ALT3].sel    = 1,

      /* Index:164 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:164 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:164 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:164 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:164 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:164 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:164 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:164 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:165 GPIO_LPSR_07 */

  {
    {
      /* Index:165 Alt:0  GPIO LPSR 07 LPI2C6 SCL */

      [ALT0].offset = IMXRT_INPUT_LPSR_LPI2C6_SCL_OFFSET,
      [ALT0].sel    = 0,

      /* Index:165 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:165 Alt:2  GPIO LPSR 07 SAI4 RX BCLK */

      [ALT2].offset = IMXRT_INPUT_LPSR_SAI4_RX_BCLK_OFFSET,
      [ALT2].sel    = 0,

      /* Index:165 Alt:3  GPIO LPSR 07 LPUART12 RXD */

      [ALT3].offset = IMXRT_INPUT_LPSR_LPUART12_RX_OFFSET,
      [ALT3].sel    = 1,

      /* Index:165 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:165 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:165 Alt:6  GPIO LPSR 07 FLEXCAN3 RX */

      [ALT6].offset = IMXRT_INPUT_LPSR_FLEXCAN3_RX_OFFSET,
      [ALT6].sel    = 1,

      /* Index:165 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:165 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:165 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:165 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:165 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:166 GPIO_LPSR_08 */

  {
    {
      /* Index:166 Alt:0  GPIO LPSR 08 LPUART11 TXD */

      [ALT0].offset = IMXRT_INPUT_LPSR_LPUART11_TX_OFFSET,
      [ALT0].sel    = 1,

      /* Index:166 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:166 Alt:2  GPIO LPSR 08 SAI4 RX SYNC */

      [ALT2].offset = IMXRT_INPUT_LPSR_SAI4_RX_SYNC_OFFSET,
      [ALT2].sel    = 0,

      /* Index:166 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:166 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:166 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:166 Alt:6  GPIO LPSR 08 LPI2C5 SDA */

      [ALT6].offset = IMXRT_INPUT_LPSR_LPI2C5_SDA_OFFSET,
      [ALT6].sel    = 1,

      /* Index:166 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:166 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:166 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:166 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:166 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:167 GPIO_LPSR_09 */

  {
    {
      /* Index:167 Alt:0  GPIO LPSR 09 LPUART11 RXD */

      [ALT0].offset = IMXRT_INPUT_LPSR_LPUART11_RX_OFFSET,
      [ALT0].sel    = 1,

      /* Index:167 Alt:1  GPIO LPSR 09 FLEXCAN3 RX */

      [ALT1].offset = IMXRT_INPUT_LPSR_FLEXCAN3_RX_OFFSET,
      [ALT1].sel    = 2,

      /* Index:167 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:167 Alt:3  GPIO LPSR 09 MIC BITSTREAM0 */

      [ALT3].offset = IMXRT_INPUT_LPSR_PDM_BITSTREAM0_OFFSET,
      [ALT3].sel    = 1,

      /* Index:167 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:167 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:167 Alt:6  GPIO LPSR 09 LPI2C5 SCL */

      [ALT6].offset = IMXRT_INPUT_LPSR_LPI2C5_SCL_OFFSET,
      [ALT6].sel    = 1,

      /* Index:167 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:167 Alt:8 No input selection */

      [ALT8].offset = DAISY_OFFSET_INVALID,
      [ALT8].sel    = DAISY_SEL_INVALID,

      /* Index:167 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:167 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:167 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:168 GPIO_LPSR_10 */

  {
    {
      /* Index:168 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:168 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:168 Alt:2  GPIO LPSR 10 LPI2C6 SDA */

      [ALT2].offset = IMXRT_INPUT_LPSR_LPI2C6_SDA_OFFSET,
      [ALT2].sel    = 1,

      /* Index:168 Alt:3  GPIO LPSR 10 MIC BITSTREAM1 */

      [ALT3].offset = IMXRT_INPUT_LPSR_PDM_BITSTREAM1_OFFSET,
      [ALT3].sel    = 0,

      /* Index:168 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:168 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:168 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:168 Alt:7  GPIO LPSR 10 SAI4 TX SYNC */

      [ALT7].offset = IMXRT_INPUT_LPSR_SAI4_TX_SYNC_OFFSET,
      [ALT7].sel    = 1,

      /* Index:168 Alt:8  GPIO LPSR 10 LPUART12 TXD */

      [ALT8].offset = IMXRT_INPUT_LPSR_LPUART12_TX_OFFSET,
      [ALT8].sel    = 2,

      /* Index:168 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:168 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:168 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:169 GPIO_LPSR_11 */

  {
    {
      /* Index:169 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:169 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:169 Alt:2  GPIO LPSR 11 LPI2C6 SCL */

      [ALT2].offset = IMXRT_INPUT_LPSR_LPI2C6_SCL_OFFSET,
      [ALT2].sel    = 1,

      /* Index:169 Alt:3  GPIO LPSR 11 MIC BITSTREAM2 */

      [ALT3].offset = IMXRT_INPUT_LPSR_PDM_BITSTREAM2_OFFSET,
      [ALT3].sel    = 0,

      /* Index:169 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:169 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:169 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:169 Alt:7 No input selection */

      [ALT7].offset = DAISY_OFFSET_INVALID,
      [ALT7].sel    = DAISY_SEL_INVALID,

      /* Index:169 Alt:8  GPIO LPSR 11 LPUART12 RXD */

      [ALT8].offset = IMXRT_INPUT_LPSR_LPUART12_RX_OFFSET,
      [ALT8].sel    = 2,

      /* Index:169 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:169 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:169 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:170 GPIO_LPSR_12 */

  {
    {
      /* Index:170 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:170 Alt:1 No input selection */

      [ALT1].offset = DAISY_OFFSET_INVALID,
      [ALT1].sel    = DAISY_SEL_INVALID,

      /* Index:170 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:170 Alt:3  GPIO LPSR 12 MIC BITSTREAM3 */

      [ALT3].offset = IMXRT_INPUT_LPSR_PDM_BITSTREAM3_OFFSET,
      [ALT3].sel    = 0,

      /* Index:170 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:170 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:170 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:170 Alt:7  GPIO LPSR 12 SAI4 TX BCLK */

      [ALT7].offset = IMXRT_INPUT_LPSR_SAI4_TX_BCLK_OFFSET,
      [ALT7].sel    = 1,

      /* Index:170 Alt:8  GPIO LPSR 12 LPSPI5 SCK */

      [ALT8].offset = IMXRT_INPUT_LPSR_LPSPI5_SCK_OFFSET,
      [ALT8].sel    = 1,

      /* Index:170 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:170 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:170 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:171 GPIO_LPSR_13 */

  {
    {
      /* Index:171 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:171 Alt:1  GPIO LPSR 13 MIC BITSTREAM1 */

      [ALT1].offset = IMXRT_INPUT_LPSR_PDM_BITSTREAM1_OFFSET,
      [ALT1].sel    = 1,

      /* Index:171 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:171 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:171 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:171 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:171 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:171 Alt:7  GPIO LPSR 13 SAI4 RX DATA */

      [ALT7].offset = IMXRT_INPUT_LPSR_SAI4_RX_DATA0_OFFSET,
      [ALT7].sel    = 1,

      /* Index:171 Alt:8  GPIO LPSR 13 LPSPI5 PCS0 */

      [ALT8].offset = IMXRT_INPUT_LPSR_LPSPI5_PCS0_OFFSET,
      [ALT8].sel    = 1,

      /* Index:171 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:171 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:171 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:172 GPIO_LPSR_14 */

  {
    {
      /* Index:172 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:172 Alt:1  GPIO LPSR 14 MIC BITSTREAM2 */

      [ALT1].offset = IMXRT_INPUT_LPSR_PDM_BITSTREAM2_OFFSET,
      [ALT1].sel    = 1,

      /* Index:172 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:172 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:172 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:172 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:172 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:172 Alt:7  GPIO LPSR 14 SAI4 RX BCLK */

      [ALT7].offset = IMXRT_INPUT_LPSR_SAI4_RX_BCLK_OFFSET,
      [ALT7].sel    = 1,

      /* Index:172 Alt:8  GPIO LPSR 14 LPSPI5 SOUT */

      [ALT8].offset = IMXRT_INPUT_LPSR_LPSPI5_SDO_OFFSET,
      [ALT8].sel    = 1,

      /* Index:172 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:172 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:172 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },

  /* index:173 GPIO_LPSR_15 */

  {
    {
      /* Index:173 Alt:0 No input selection */

      [ALT0].offset = DAISY_OFFSET_INVALID,
      [ALT0].sel    = DAISY_SEL_INVALID,

      /* Index:173 Alt:1  GPIO LPSR 15 MIC BITSTREAM3 */

      [ALT1].offset = IMXRT_INPUT_LPSR_PDM_BITSTREAM3_OFFSET,
      [ALT1].sel    = 1,

      /* Index:173 Alt:2 No input selection */

      [ALT2].offset = DAISY_OFFSET_INVALID,
      [ALT2].sel    = DAISY_SEL_INVALID,

      /* Index:173 Alt:3 No input selection */

      [ALT3].offset = DAISY_OFFSET_INVALID,
      [ALT3].sel    = DAISY_SEL_INVALID,

      /* Index:173 Alt:4 No input selection */

      [ALT4].offset = DAISY_OFFSET_INVALID,
      [ALT4].sel    = DAISY_SEL_INVALID,

      /* Index:173 Alt:5 No input selection */

      [ALT5].offset = DAISY_OFFSET_INVALID,
      [ALT5].sel    = DAISY_SEL_INVALID,

      /* Index:173 Alt:6 No input selection */

      [ALT6].offset = DAISY_OFFSET_INVALID,
      [ALT6].sel    = DAISY_SEL_INVALID,

      /* Index:173 Alt:7  GPIO LPSR 15 SAI4 RX SYNC */

      [ALT7].offset = IMXRT_INPUT_LPSR_SAI4_RX_SYNC_OFFSET,
      [ALT7].sel    = 1,

      /* Index:173 Alt:8  GPIO LPSR 15 LPSPI5 SIN */

      [ALT8].offset = IMXRT_INPUT_LPSR_LPSPI5_SDI_OFFSET,
      [ALT8].sel    = 1,

      /* Index:173 Alt:9 No input selection */

      [ALT9].offset = DAISY_OFFSET_INVALID,
      [ALT9].sel    = DAISY_SEL_INVALID,

      /* Index:173 Alt:10 No input selection */

      [ALT10].offset = DAISY_OFFSET_INVALID,
      [ALT10].sel    = DAISY_SEL_INVALID,

      /* Index:173 Alt:11 No input selection */

      [ALT11].offset = DAISY_OFFSET_INVALID,
      [ALT11].sel    = DAISY_SEL_INVALID,
    },
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imxrt_daisy_select(unsigned int index, unsigned int alt)
{
  uintptr_t address;

  DEBUGASSERT(index < sizeof(g_daisy_select) / sizeof(g_daisy_select[0]));

  const struct imxrt_daisy_t *daisy = &g_daisy_select[index];

  address = daisy->alts[alt].offset;
  if (address != DAISY_OFFSET_INVALID)
    {
      alt = daisy->alts[alt].sel;
      address += address >= IMXRT_INPUT_FLEXCAN1_RX_OFFSET ?
                 IMXRT_IOMUXC_BASE : IMXRT_IOMUXCLPSR_BASE;

      putreg32(alt, address);
    }
}
