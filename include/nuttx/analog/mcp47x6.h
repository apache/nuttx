/****************************************************************************
 * include/nuttx/analog/mcp47x6.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_ANALOG_MCP47X6_H
#define __INCLUDE_NUTTX_ANALOG_MCP47X6_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* Cmd: ANIOC_MCP47X6_DAC_SET_GAIN         Arg: mcp47x6_gain_e value
 * Cmd: ANIOC_MCP47X6_DAC_SET_POWER_DOWN   Arg: mcp47x6_power_down_e value
 * Cmd: ANIOC_MCP47X6_DAC_SET_REFERENCE    Arg: mcp47x6_reference_e value
 */

#define ANIOC_MCP47X6_DAC_SET_GAIN         _ANIOC(AN_MCP47X6_FIRST + 0)
#define ANIOC_MCP47X6_DAC_SET_POWER_DOWN   _ANIOC(AN_MCP47X6_FIRST + 1)
#define ANIOC_MCP47X6_DAC_SET_REFERENCE    _ANIOC(AN_MCP47X6_FIRST + 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum mcp47x6_command_e
{
  MCP47X6_COMMAND_WRITE_DAC    = 0 << 5U,
  MCP47X6_COMMAND_WRITE_CONFIG = 4 << 5U,
};

enum mcp47x6_reference_e
{
  MCP47X6_REFERENCE_VDD_UNBUFFERED  = 0 << 3U,
  MCP47X6_REFERENCE_VREF_UNBUFFERED = 2 << 3U,
  MCP47X6_REFERENCE_VREF_BUFFERED   = 3 << 3U,
};

enum mcp47x6_power_down_e
{
  MCP47X6_POWER_DOWN_DISABLED = 0 << 1U,
  MCP47X6_POWER_DOWN_1K       = 1 << 1U,
  MCP47X6_POWER_DOWN_100K     = 2 << 1U,
  MCP47X6_POWER_DOWN_500K     = 3 << 1U,
};

/* MCP47X6_GAIN_2X is ignored for MCP47X6_REFERENCE_VDD_UNBUFFERED */

enum mcp47x6_gain_e
{
  MCP47X6_GAIN_1X = 0 << 0U,
  MCP47X6_GAIN_2X = 1 << 0U
};

#endif /* __INCLUDE_NUTTX_ANALOG_MCP47X6_H */
