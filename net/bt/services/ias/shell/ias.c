/****************************************************************************
 * net/bt/services/ias/shell/ias.c
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

#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <nuttx/net/bluetooth/conn.h>

#include <nuttx/net/bluetooth/compat/shell/shell.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/services/ias.h>

#include "host/shell/bt.h"
#include "common/bt_shell_private.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void alert_stop(void)
{
  bt_shell_print("Alert stopped\n");
}

static void alert_start(void)
{
  bt_shell_print("Mild alert started\n");
}

static void alert_high_start(void)
{
  bt_shell_print("High alert started\n");
}

BT_IAS_CB_DEFINE(ias_callbacks) =
{
        .no_alert = alert_stop,
        .mild_alert = alert_start,
        .high_alert = alert_high_start,
};

static int cmd_ias_local_alert_stop(const struct shell *sh,
                                    size_t argc, char **argv)
{
  const int result = bt_ias_local_alert_stop();

  if (result)
    {
      shell_print(sh, "Local alert stop failed: %d", result);
    }
  else
    {
      shell_print(sh, "Local alert stopped");
    }

  return result;
}

static int cmd_ias(const struct shell *sh, size_t argc, char **argv)
{
  shell_error(sh, "%s unknown parameter: %s", argv[0], argv[1]);

  return -ENOEXEC;
}

SHELL_STATIC_SUBCMD_SET_CREATE(ias_cmds,
                               SHELL_CMD_ARG(local_alert_stop,
                                   NULL,
                                   "Stop alert locally",

                                             cmd_ias_local_alert_stop, 1, 0),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_ARG_REGISTER(ias,
                    &ias_cmds,
                    "Bluetooth IAS shell commands",
                    cmd_ias,
                    1,
                    1);
