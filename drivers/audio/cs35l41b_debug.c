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

struct cs35l41b_debug_msg_s
{
  struct cs35l41b_regdump_s *dump_ptr;
  uint32_t                   regdump_size;
  uint32_t                   gain;
  int                        mode;
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
void cs35l41b_dump_registers(FAR struct cs35l41b_dev_s *priv,
                             unsigned long arg)
{
  int i;
  struct cs35l41b_debug_msg_s *dump_reg;

  if (arg)
    {
      dump_reg                = (struct cs35l41b_debug_msg_s *)arg;
      dump_reg->dump_ptr      = g_cs35l41b_debug_reg;
      dump_reg->regdump_size  = ARRAY_SIZE(g_cs35l41b_debug_reg);
    }

  for (i = 0; i < ARRAY_SIZE(g_cs35l41b_debug_reg); i++)
    {
      cs35l41b_read_register(priv, &g_cs35l41b_debug_reg[i].regval,
                            g_cs35l41b_debug_reg[i].regaddr);
      if (priv->dump_dsp_info)
        {
          syslog(LOG_WARNING, "-%25s[%08lx]: %08lx\n",
                g_cs35l41b_debug_reg[i].regname,
                g_cs35l41b_debug_reg[i].regaddr,
                g_cs35l41b_debug_reg[i].regval);
        }
    }

}
#endif

/****************************************************************************
 * Name: cs35l41b_debug_set_gain
 *
 * Description:
 *   cs35l41b debug set gain
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_CS35L41B_DEBUG
int cs35l41b_debug_set_gain(FAR struct cs35l41b_dev_s *priv,
                            unsigned long arg)
{
  uint32_t regval;
  uint32_t temp;
  uint32_t gain;
  struct cs35l41b_debug_msg_s *msg = (struct cs35l41b_debug_msg_s *)arg;
  int ret;

  if (msg->gain > 20)
    {
      syslog(LOG_ERR, "gain parameter is invaild!\n");
      return ERROR;
    }

  if (priv->mode == CS35L41_ASP_MODE)
    {
      priv->asp_gain = msg->gain << CS35L41B_AMP_GAIN_PCM_SHIFT;
    }
  else
    {
      priv->dsp_gain = msg->gain << CS35L41B_AMP_GAIN_PCM_SHIFT;
    }

  if ((priv->state == CS35L41_STATE_POWER_UP) ||
      (priv->state == CS35L41_STATE_DSP_POWER_UP))
    {
      gain = msg->gain << CS35L41B_AMP_GAIN_PCM_SHIFT;

      ret = cs35l41b_read_register(priv, &regval, CS35L41B_AMP_GAIN_REG);
      if (ret < 0)
        {
          syslog(LOG_ERR, "read CS35L41B_AMP_GAIN_REG error!\n");
          return ERROR;
        }

      temp = ~CS35L41B_AMP_GAIN_PCM_MASK;
      temp &= regval;

      temp |= gain;

      if (cs35l41b_write_register(priv, CS35L41B_AMP_GAIN_REG, temp) < 0)
        {
          syslog(LOG_ERR, "write CS35L41B_AMP_GAIN_REG error!\n");
          return ERROR;
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: cs35l41b_debug_get_gain
 *
 * Description:
 *   cs35l41b debug get gain
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_CS35L41B_DEBUG
int cs35l41b_debug_get_gain(FAR struct cs35l41b_dev_s *priv,
                            unsigned long arg)
{
  struct cs35l41b_debug_msg_s *msg = (struct cs35l41b_debug_msg_s *)arg;

  if (priv->mode == CS35L41_ASP_MODE)
    {
      msg->gain = (priv->asp_gain >> CS35L41B_AMP_GAIN_PCM_SHIFT);
    }
  else
    {
      msg->gain = (priv->dsp_gain >> CS35L41B_AMP_GAIN_PCM_SHIFT);
    }

  syslog(LOG_INFO, "cs35l41b mode:%d, gain:%ld \n", priv->mode, msg->gain);

  return OK;
}
#endif

/****************************************************************************
 * Name: cs35l41b_debug_get_gain
 *
 * Description:
 *   cs35l41b debug get gain
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_CS35L41B_DEBUG
int cs35l41b_debug_get_mode(FAR struct cs35l41b_dev_s *priv,
                            unsigned long arg)
{
  struct cs35l41b_debug_msg_s *msg = (struct cs35l41b_debug_msg_s *)arg;

  msg->mode = priv->mode;

  syslog(LOG_INFO, "cs35l41b mode:%d", msg->mode);

  return OK;
}
#endif