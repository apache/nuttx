/****************************************************************************
 * net/bt/mesh/shell/sar.c
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
#include "../sar_cfg_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cmd_tx_get(const struct shell *sh, size_t argc, char *argv[])
{
  struct bt_mesh_sar_tx rsp;
  int err;

  err = bt_mesh_sar_cfg_cli_transmitter_get(bt_mesh_shell_target_ctx.net_idx,
                                            bt_mesh_shell_target_ctx.dst,
                                            &rsp);
  if (err)
    {
      shell_error(sh, "Failed to send SAR Transmitter Get (err %d)", err);
      return 0;
    }

  shell_print(sh, "Transmitter Get: %u %u %u %u %u %u %u", rsp.seg_int_step,
              rsp.unicast_retrans_count,
              rsp.unicast_retrans_without_prog_count,
              rsp.unicast_retrans_int_step, rsp.unicast_retrans_int_inc,
              rsp.multicast_retrans_count, rsp.multicast_retrans_int);

  return 0;
}

static int cmd_tx_set(const struct shell *sh, size_t argc, char *argv[])
{
  struct bt_mesh_sar_tx set;
  struct bt_mesh_sar_tx rsp;
  int err = 0;

  set.seg_int_step = shell_strtoul(argv[1], 0, &err);
  set.unicast_retrans_count = shell_strtoul(argv[2], 0, &err);
  set.unicast_retrans_without_prog_count = shell_strtoul(argv[3], 0, &err);
  set.unicast_retrans_int_step = shell_strtoul(argv[4], 0, &err);
  set.unicast_retrans_int_inc = shell_strtoul(argv[5], 0, &err);
  set.multicast_retrans_count = shell_strtoul(argv[6], 0, &err);
  set.multicast_retrans_int = shell_strtoul(argv[7], 0, &err);

  if (err)
    {
      shell_warn(sh, "Unable to parse input string argument");
      return err;
    }

  err = bt_mesh_sar_cfg_cli_transmitter_set(bt_mesh_shell_target_ctx.net_idx,
                                            bt_mesh_shell_target_ctx.dst,
                                            &set, &rsp);
  if (err)
    {
      shell_error(sh, "Failed to send SAR Transmitter Set (err %d)", err);
      return 0;
    }

  shell_print(sh, "Transmitter Set: %u %u %u %u %u %u %u", rsp.seg_int_step,
              rsp.unicast_retrans_count,
              rsp.unicast_retrans_without_prog_count,
              rsp.unicast_retrans_int_step, rsp.unicast_retrans_int_inc,
              rsp.multicast_retrans_count, rsp.multicast_retrans_int);

  return 0;
}

static int cmd_rx_get(const struct shell *sh, size_t argc, char *argv[])
{
  struct bt_mesh_sar_rx rsp;
  int err;

  err = bt_mesh_sar_cfg_cli_receiver_get(bt_mesh_shell_target_ctx.net_idx,
                                         bt_mesh_shell_target_ctx.dst, &rsp);
  if (err)
    {
      shell_error(sh, "Failed to send SAR Receiver Get (err %d)", err);
      return 0;
    }

  shell_print(sh, "Receiver Get: %u %u %u %u %u", rsp.seg_thresh,
          rsp.ack_delay_inc,
              rsp.ack_retrans_count, rsp.discard_timeout,
              rsp.rx_seg_int_step);

  return 0;
}

static int cmd_rx_set(const struct shell *sh, size_t argc, char *argv[])
{
  struct bt_mesh_sar_rx set;
  struct bt_mesh_sar_rx rsp;
  int err = 0;

  set.seg_thresh = shell_strtoul(argv[1], 0, &err);
  set.ack_delay_inc = shell_strtoul(argv[2], 0, &err);
  set.ack_retrans_count = shell_strtoul(argv[3], 0, &err);
  set.discard_timeout = shell_strtoul(argv[4], 0, &err);
  set.rx_seg_int_step = shell_strtoul(argv[5], 0, &err);

  if (err)
    {
      shell_warn(sh, "Unable to parse input string argument");
      return err;
    }

  err = bt_mesh_sar_cfg_cli_receiver_set(bt_mesh_shell_target_ctx.net_idx,
                                         bt_mesh_shell_target_ctx.dst, &set,
                                         &rsp);
  if (err)
    {
      shell_error(sh, "Failed to send SAR Receiver Set (err %d)", err);
      return 0;
    }

  shell_print(sh, "Receiver Set: %u %u %u %u %u", rsp.seg_thresh,
          rsp.ack_delay_inc,
              rsp.ack_retrans_count, rsp.discard_timeout,
              rsp.rx_seg_int_step);

  return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
        sar_cfg_cli_cmds, SHELL_CMD_ARG(tx - get,
                                        NULL,
                                        NULL,
                                        cmd_tx_get,
                                        1,
                                        0),
        SHELL_CMD_ARG(tx - set, NULL,
                      "<SegIntStep> <UniRetransCnt> "
                      "<UniRetransWithoutProgCnt> "
                      "<UniRetransIntStep> <UniRetransIntInc> "
                      "<MultiRetransCnt> "
                      "<MultiRetransInt>",
                      cmd_tx_set, 8, 0),
        SHELL_CMD_ARG(rx - get, NULL, NULL, cmd_rx_get, 1, 0),
        SHELL_CMD_ARG(rx - set, NULL,
                      "<SegThresh> <AckDelayInc> <DiscardTimeout> "
                      "<RxSegIntStep> <AckRetransCount>",
                      cmd_rx_set, 6, 0),
        SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((mesh,
    models),
    sar,
    &sar_cfg_cli_cmds,
    "Sar Cfg Cli commands",

                 bt_mesh_shell_mdl_cmds_help, 1, 1);
