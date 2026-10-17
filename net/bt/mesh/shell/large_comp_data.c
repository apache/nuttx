/****************************************************************************
 * net/bt/mesh/shell/large_comp_data.c
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

#include "common/bt_shell_private.h"
#include "utils.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void status_print(int err,
                         char *msg, uint16_t addr,
                         struct bt_mesh_large_comp_data_rsp *rsp)
{
  if (err)
    {
      bt_shell_error("Failed to send %s Get message (err %d)", msg, err);
      return;
    }

  bt_shell_print("%s [0x%04x]: page: %u offset: %u total size: "
                 "%u", msg, addr,
          rsp->page,
                 rsp->offset, rsp->total_size);
  bt_shell_hexdump(rsp->data->data, rsp->data->len);
}

static int cmd_large_comp_data_get(const struct shell *sh,
                                   size_t argc, char *argv[])
{
  NET_BUF_SIMPLE_DEFINE(comp, 64);
  struct bt_mesh_large_comp_data_rsp rsp =
    {
          .data = &comp,
    };

  uint8_t page;
  uint16_t offset;
  int err = 0;

  net_buf_simple_init(&comp, 0);

  page = shell_strtoul(argv[1], 0, &err);
  offset = shell_strtoul(argv[2], 0, &err);

  if (err)
    {
      shell_warn(sh, "Unable to parse input string argument");
      return err;
    }

  err = bt_mesh_large_comp_data_get(bt_mesh_shell_target_ctx.net_idx,
          bt_mesh_shell_target_ctx.dst,
                                    page, offset, &rsp);
  status_print(err, "Composition Data", bt_mesh_shell_target_ctx.dst, &rsp);

  return err;
}

static int cmd_models_metadata_get(const struct shell *sh,
                                   size_t argc, char *argv[])
{
  NET_BUF_SIMPLE_DEFINE(metadata, 64);
  struct bt_mesh_large_comp_data_rsp rsp =
    {
          .data = &metadata,
    };

  uint8_t page;
  uint16_t offset;
  int err = 0;

  net_buf_simple_init(&metadata, 0);

  page = shell_strtoul(argv[1], 0, &err);
  offset = shell_strtoul(argv[2], 0, &err);

  if (err)
    {
      shell_warn(sh, "Unable to parse input string argument");
      return err;
    }

  err = bt_mesh_models_metadata_get(bt_mesh_shell_target_ctx.net_idx,
          bt_mesh_shell_target_ctx.dst,
                                    page, offset, &rsp);
  status_print(err, "Models Metadata", bt_mesh_shell_target_ctx.dst, &rsp);

  return err;
}

SHELL_STATIC_SUBCMD_SET_CREATE(large_comp_data_cmds,
                               SHELL_CMD_ARG(large - comp - data - get,
                                   NULL,
                                   "<page> <offset>",

                                             cmd_large_comp_data_get, 3, 0),
                               SHELL_CMD_ARG(models - metadata - get,
                                   NULL,
                                   "<page> <offset>",

                                             cmd_models_metadata_get, 3, 0),
                               SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((mesh, models), lcd, &large_comp_data_cmds, "Large Comp "
                                                             "Data Cli "
                                                             "commands",
                 bt_mesh_shell_mdl_cmds_help, 1, 1);
