/****************************************************************************
 * drivers/audio/cs35l41b_debug.c
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
#include <nuttx/arch.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/cs35l41b.h>
#include "cs35l41b.h"
#include "cs35l41b_fw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAY_SIZE(array)           sizeof(array) / sizeof(array[0])
#define CS35L41B_DEBUG_DSP_MODE     0x01
#define CS35L41B_DEBUG_ASP_MODE     0x02

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

#ifdef CONFIG_AUDIO_CS35L41B_DEBUG
struct cs35l41b_regdump_s
{
  FAR const char *regname;
  uint32_t regaddr;
  uint32_t regval;
};
#endif
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_AUDIO_CS35L41B_DEBUG
static struct cs35l41b_regdump_s g_cs35l41b_debug_reg[] =
{
  {"CHIP_ID",             0x00000000},
  {"GLOBAL_EN",           0x00002014},
  {"REFCLK_INPUT",        0x00002c04},
  {"SAMPLE_RATE",         0x00002c0c},
  {"ASP_ENABLES1",        0x00004800},
  {"ASP_CONTROL2",        0x00004804},
  {"ASP_FRAME_CONTROL1",  0x00004810},
  {"ASP_FRAME_CONTROL5",  0x00004820},
  {"ASP_DATA_CONTROL1",   0x00004830},
  {"ASP_DATA_CONTROL5",   0x00004840},
  {"DACPCM1_INPUT",       0x00004c00},
  {"DSP1RX1_INPUT",       0x00004c40},
  {"DSP1RX2_INPUT",       0x00004c44},
  {"AMP_GAIN",            0x00006c04},
  {"IRQ1_EINT_1_REG",     0x00010010},
  {"IRQ1_EINT_2_REG",     0x00010014},
  {"CAL_VALUE",           0x02800268},
  {"CAL_AMBIENT",         0x0280026c},
  {"CAL_STATUS",          0x02800270},
  {"CAL_CHECKSUM",        0x02800274},
  {"CAL_R_SEL",           0x02800278},
  {"CAL_SET_STATUS",      0x0280027c},
  {"CPSL_HALO_STATUS",    0x028007e8},
  {"CPSL_HALO_HEARTBEAT", 0x028007ec},
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs35l41b_dump_registers
 *
 * Description:
 *   cs35l41b dump registers
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_CS35L41B_DEBUG
void cs35l41b_dump_registers(FAR struct cs35l41b_dev_s *priv)
{
  int i;
  for (i = 0; i < ARRAY_SIZE(g_cs35l41b_debug_reg); i++)
    {
      cs35l41b_read_register(priv, &g_cs35l41b_debug_reg[i].regval,
                            g_cs35l41b_debug_reg[i].regaddr);
      syslog(LOG_WARNING, "-%25s[%08x]: %08x\n",
             g_cs35l41b_debug_reg[i].regname,
             g_cs35l41b_debug_reg[i].regaddr,
             g_cs35l41b_debug_reg[i].regval);
    }
}
#endif
