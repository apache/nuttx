/****************************************************************************
 * net/bt/mesh/shell/sol_pdu_rpl.c
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

static int cmd_srpl_clear(const struct shell *sh, size_t argc, char *argv[])
{
  int err = 0;
  uint8_t len;

  struct bt_mesh_msg_ctx ctx =
          BT_MESH_MSG_CTX_INIT_APP(bt_mesh_shell_target_ctx.app_idx,
          bt_mesh_shell_target_ctx.dst);

  uint16_t range_start = shell_strtoul(argv[1], 0, &err);
  bool acked = shell_strtobool(argv[2], 0, &err);

  if (argc > 3)
    {
      len = shell_strtoul(argv[3], 0, &err);
    }
  else
    {
      len = 0;
    }

  if (err < 0)
    {
      shell_error(sh, "Invalid command parameter (err %d)", err);
      return err;
    }

  if (acked)
    {
      uint16_t start_rsp;
      uint8_t len_rsp;

      err = bt_mesh_sol_pdu_rpl_clear(&ctx, range_start, len, &start_rsp,
              &len_rsp);

      if (err)
        {
          shell_error(sh, "Failed to clear Solicitation PDU RPL (err %d)",
                  err);
        }
      else
        {
          shell_print(sh, "Cleared Solicitation PDU RPL with range start=%u "
                          "len=%u",
                  start_rsp,
                      len_rsp);
        }

      return err;
    }

  err = bt_mesh_sol_pdu_rpl_clear_unack(&ctx, range_start, len);
  if (err)
    {
      shell_error(sh, "Failed to clear Solicitation PDU RPL (err %d)", err);
    }

  return err;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sol_pdu_rpl_cmds,
                               SHELL_CMD_ARG(clear,
                                   NULL,
                                   "<RngStart> <Ackd> "
                                                          "[RngLen]",
                                             cmd_srpl_clear, 3, 1),
                               SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((mesh, models), sol_pdu_rpl, &sol_pdu_rpl_cmds,
                 "Solicitation PDU RPL Cli "
                 "commands", bt_mesh_shell_mdl_cmds_help, 1, 1);
