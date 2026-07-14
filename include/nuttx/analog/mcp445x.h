/****************************************************************************
 * include/nuttx/analog/mcp445x.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_MCP445X_H
#define __INCLUDE_NUTTX_ANALOG_MCP445X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* Cmd: ANIOC_MCP445X_SET_TCON   Arg: struct mcp445x_tcon_s *tcon
 * Cmd: ANIOC_MCP445X_GET_TCON   Arg: struct mcp445x_tcon_s *tcon
 */

#define ANIOC_MCP445X_SET_TCON    _ANIOC(AN_MCP445X_FIRST + 0)
#define ANIOC_MCP445X_GET_TCON    _ANIOC(AN_MCP445X_FIRST + 1)

/* Terminal control flags. A set bit means the given terminal is connected
 * (B, W, A) or the wiper is not forced into shutdown (HW).
 */

#define MCP445X_TCON_B            (1 << 0)  /* Terminal B connected */
#define MCP445X_TCON_W            (1 << 1)  /* Wiper terminal connected */
#define MCP445X_TCON_A            (1 << 2)  /* Terminal A connected */
#define MCP445X_TCON_HW           (1 << 3)  /* Wiper not in shutdown */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Terminal control for ANIOC_MCP445X_SET_TCON and ANIOC_MCP445X_GET_TCON */

struct mcp445x_tcon_s
{
  uint8_t wiper;                 /* Wiper index */
  uint8_t flags;                 /* Terminal control flags, see
                                  * MCP445X_TCON_* */
};

#endif /* __INCLUDE_NUTTX_ANALOG_MCP445X_H */
