/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_freqin.c
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

#include <stdint.h>

#include <arch/board/board.h>

#include "lpc31_cgudrvr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This array provides the programmed frequency of every input source.  The
 * board FFAST input crystal frequency is the only value known initially.
 * Additional frequencies will be added to the table as they are determined
 */

uint32_t g_boardfreqin[CGU_NFREQIN] =
{
  BOARD_FREQIN_FFAST,       /* Index=CGU_FREQIN_FFAST */
  0,                        /* Index=CGU_FREQIN_I2SRXBCK0 */
  0,                        /* Index=CGU_FREQIN_I2SRXWS0 */
  0,                        /* Index=CGU_FREQIN_I2SRXBCK1 */
  0,                        /* Index=CGU_FREQIN_I2SRXWS1 */
  0,                        /* Index=CGU_FREQIN_HPPLL0 (Audio/I2S PLL) */
  0                         /* Index=CGU_FREQIN_HPPLL1 (System PLL) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
