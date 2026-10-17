/****************************************************************************
 * net/bt/mesh/shell/od_priv_proxy.c
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/* Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <nuttx/net/bluetooth/compat/shell/shell.h>
#include <nuttx/net/bluetooth/mesh.h>
#include <nuttx/net/bluetooth/mesh/shell.h>

#include "utils.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cmd_od_priv_gatt_proxy_set(const struct shell *sh,
                                      size_t argc, char *argv[])
{
  uint8_t val;
  uint8_t val_rsp;
  uint16_t net_idx = bt_mesh_shell_target_ctx.net_idx;
  uint16_t addr = bt_mesh_shell_target_ctx.dst;
  int err = 0;

  if (argc < 2)
    {
      err = bt_mesh_od_priv_proxy_cli_get(net_idx, addr, &val_rsp);
    }
  else
    {
      val = shell_strtoul(argv[1], 0, &err);

      if (err)
        {
          shell_warn(sh, "Unable to parse input string argument");
          return err;
        }

      err = bt_mesh_od_priv_proxy_cli_set(net_idx, addr, val, &val_rsp);
    }

  if (err)
    {
      shell_print(sh, "Unable to send On-Demand Private GATT Proxy Get/Set "
                      "(err %d)",
              err);
      return 0;
    }

  shell_print(sh, "On-Demand Private GATT Proxy is set to 0x%02x", val_rsp);

  return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(od_priv_proxy_cmds,
                               SHELL_CMD_ARG(gatt - proxy, NULL, "[Dur(s)]",
                                             cmd_od_priv_gatt_proxy_set, 1,
                                                 1),
                               SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((mesh, models), od_priv_proxy, &od_priv_proxy_cmds,
                 "On-Demand Private Proxy Cli "
                 "commands", bt_mesh_shell_mdl_cmds_help, 1, 1);
