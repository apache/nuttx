/****************************************************************************
 * include/nuttx/analog/mcp48xx.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_MCP48XX_H
#define __INCLUDE_NUTTX_ANALOG_MCP48XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* Cmd: ANIOC_MCP48XX_DAC_SET_GAIN         Arg: mcp48xx_set_gain_e value
 * Cmd: ANIOC_MCP48XX_DAC_ENABLE           Arg: mcp48xx_dac_channel_e channel
 * Cmd: ANIOC_MCP48XX_DAC_DISABLE          Arg: mcp48xx_dac_channel_e channel
 */

#define ANIOC_MCP48XX_DAC_SET_GAIN         _ANIOC(AN_MCP48XX_FIRST + 0)
#define ANIOC_MCP48XX_DAC_ENABLE           _ANIOC(AN_MCP48XX_FIRST + 1)
#define ANIOC_MCP48XX_DAC_DISABLE          _ANIOC(AN_MCP48XX_FIRST + 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum mcp48xx_gain_select_e
{
  MCP48XX_GAIN_2X  = 0 << 8U,  /* VOUT = 2 * VREF * D/4096 */
  MCP48XX_GAIN_1X  = 1 << 8U   /* VOUT = VREF * D/4096 */
};

enum mcp48xx_dac_channel_e
{
  MCP48XX_DAC_CHA  = 1 << 0U,  /* Select channel A */
  MCP48XX_DAC_CHB  = 1 << 1U,  /* Select channel B */
  MCP48XX_DAC_ALL  = 0x3u      /* Select both channels A and B */
};

enum mcp48xx_set_gain_e
{
  MCP48XX_DAC_CHA_GAIN_2X = MCP48XX_DAC_CHA | MCP48XX_GAIN_2X,
  MCP48XX_DAC_CHB_GAIN_2X = MCP48XX_DAC_CHB | MCP48XX_GAIN_2X,
  MCP48XX_DAC_CHA_GAIN_1X = MCP48XX_DAC_CHA | MCP48XX_GAIN_1X,
  MCP48XX_DAC_CHB_GAIN_1X = MCP48XX_DAC_CHB | MCP48XX_GAIN_1X,
  MCP48XX_DAC_ALL_GAIN_2X = MCP48XX_DAC_ALL | MCP48XX_GAIN_2X,
  MCP48XX_DAC_ALL_GAIN_1X = MCP48XX_DAC_ALL | MCP48XX_GAIN_1X
};

#endif /* __INCLUDE_NUTTX_ANALOG_MCP48XX_H */
