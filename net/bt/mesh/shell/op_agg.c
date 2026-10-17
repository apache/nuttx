/****************************************************************************
 * net/bt/mesh/shell/op_agg.c
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

static int cmd_seq_start(const struct shell *sh, size_t argc, char *argv[])
{
  uint16_t elem_addr;
  int err = 0;

  elem_addr = shell_strtoul(argv[1], 0, &err);
  if (err)
    {
      shell_warn(sh, "Unable to parse input string argument");
      return err;
    }

  bt_mesh_shell_target_ctx.dst = elem_addr;
  shell_print(sh, "mesh dst set to 0x%04x", elem_addr);

  err = bt_mesh_op_agg_cli_seq_start(bt_mesh_shell_target_ctx.net_idx,
                                     bt_mesh_shell_target_ctx.app_idx,
                                     bt_mesh_shell_target_ctx.dst,
                                     elem_addr);
  if (err)
    {
      shell_error(sh,
              "Failed to configure Opcodes Aggregator Context (err %d)",
              err);
    }

  return 0;
}

static int cmd_seq_send(const struct shell *sh, size_t argc, char *argv[])
{
  int err;

  err = bt_mesh_op_agg_cli_seq_send();
  if (err)
    {
      shell_error(sh, "Failed to send Opcodes Aggregator Sequence message "
                      "(err %d)",
              err);
    }

  return 0;
}

static int cmd_seq_abort(const struct shell *sh, size_t argc, char *argv[])
{
  bt_mesh_op_agg_cli_seq_abort();

  return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(op_agg_cmds,
                               SHELL_CMD_ARG(seq - start,
                                             NULL,
                                             "<ElemAddr>",
                                             cmd_seq_start,
                                             2,
                                             0),
                               SHELL_CMD_ARG(seq - send,
                                             NULL,
                                             NULL,
                                             cmd_seq_send,
                                             1,
                                             0),
                               SHELL_CMD_ARG(seq - abort,
                                             NULL,
                                             NULL,
                                             cmd_seq_abort,
                                             1,
                                             0),
                               SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((mesh,
    models),
    opagg,
    &op_agg_cmds,
    "Opcode Aggregator Cli "
                                                      "commands",
                 bt_mesh_shell_mdl_cmds_help, 1, 1);
