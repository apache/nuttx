/****************************************************************************
 * net/bt/services/ias/shell/ias_client.c
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

/**
 * @file
 * @brief Shell APIs for Bluetooth IAS
 *
 * Copyright (c) 2022 Codecoup
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>
#include <nuttx/net/bluetooth/compat/shell/shell.h>
#include <stdlib.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/services/ias.h>

#include "host/shell/bt.h"
#include "common/bt_shell_private.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ias_discover_cb(struct bt_conn *conn, int err)
{
  if (err != 0)
    {
      bt_shell_error("Failed to discover IAS err: %d\n", err);
    }
  else
    {
      bt_shell_print("IAS discover success\n");
    }
}

static struct bt_ias_client_cb ias_client_callbacks =
{
        .discover = ias_discover_cb,
};

static int cmd_ias_client_init(const struct shell *sh,
                               size_t argc, char **argv)
{
  int err;

  err = bt_ias_client_cb_register(&ias_client_callbacks);
  if (err != 0)
    {
      shell_print(sh, "IAS client init failed");
    }
  else
    {
      shell_print(sh, "IAS client initialized");
    }

  return err;
}

static int cmd_ias_client_discover(const struct shell *sh,
                                   size_t argc, char **argv)
{
  int err;

  err = bt_ias_discover(default_conn);
  if (err != 0)
    {
      shell_print(sh, "IAS discover failed");
    }

  return err;
}

static int cmd_ias_client_set_alert(const struct shell *sh,
                                    size_t argc, char **argv)
{
  int err = 0;

  if (strcmp(argv[1], "stop") == 0)
    {
      err = bt_ias_client_alert_write(default_conn,
              BT_IAS_ALERT_LVL_NO_ALERT);
    }
  else if (strcmp(argv[1], "mild") == 0)
    {
      err = bt_ias_client_alert_write(default_conn,
              BT_IAS_ALERT_LVL_MILD_ALERT);
    }
  else if (strcmp(argv[1], "high") == 0)
    {
      err = bt_ias_client_alert_write(default_conn,
              BT_IAS_ALERT_LVL_HIGH_ALERT);
    }
  else
    {
      shell_error(sh, "Invalid alert level %s", argv[1]);
      return -EINVAL;
    }

  if (err != 0)
    {
      shell_error(sh, "Failed to send %s alert %d", argv[1], err);
    }
  else
    {
      shell_print(sh, "Sent alert %s", argv[1]);
    }

  return err;
}

static int cmd_ias_client(const struct shell *sh, size_t argc, char **argv)
{
  if (argc > 1)
    {
      shell_error(sh, "%s unknown parameter: %s", argv[0], argv[1]);
    }
  else
    {
      shell_error(sh, "%s Missing subcommand", argv[0]);
    }

  return -ENOEXEC;
}

SHELL_STATIC_SUBCMD_SET_CREATE(ias_cli_cmds,
                               SHELL_CMD_ARG(init, NULL,
                                             "Initialize the client and "
                                             "register callbacks",
                                             cmd_ias_client_init, 1, 0),
                               SHELL_CMD_ARG(discover, NULL, "Discover IAS",
                                             cmd_ias_client_discover, 1, 0),
                               SHELL_CMD_ARG(set_alert, NULL, "Send alert "
    "<stop/mild/high>",
                                             cmd_ias_client_set_alert, 2, 0),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_ARG_REGISTER(ias_client, &ias_cli_cmds, "Bluetooth IAS client "
                                                  "shell commands",
                       cmd_ias_client, 1, 1);
