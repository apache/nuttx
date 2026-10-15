/****************************************************************************
 * net/bt/common/bt_shell_private.c
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
 * @file bt_shell_private.c
 * @brief Bluetooth shell private module
 *
 * Provide common function which can be shared using privately inside
 * Bluetooth shell.
 */

/* Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bt_shell_private.h"

uint32_t shell_backend_count_get(void);
const struct shell *shell_backend_get(uint32_t idx);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void wall_vfprintf(enum shell_vt100_color color,
                          const char *fmt, va_list args)
{
  uint32_t count;
  const struct shell *sh;

  count = shell_backend_count_get();
  for (uint32_t i = 0; i < count; i++)
    {
      va_list args_copy;

      va_copy(args_copy, args); /* Create a copy of 'args' to safely reuse */
      sh = shell_backend_get(i);
      shell_vfprintf(sh, color, fmt, args_copy);
      va_end(args_copy); /* Clean up to prevent resource leaks */
    }
}

void bt_shell_hexdump(const uint8_t *data, size_t len)
{
  uint32_t count;
  const struct shell *sh;

  count = shell_backend_count_get();
  for (uint32_t i = 0; i < count; i++)
    {
      sh = shell_backend_get(i);
      shell_hexdump(sh, data, len);
    }
}

void bt_shell_fprintf(enum shell_vt100_color color, const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  wall_vfprintf(color, fmt, args);
  va_end(args);
}

void bt_shell_fprintf_info(const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  wall_vfprintf(SHELL_INFO, fmt, args);
  va_end(args);
}

void bt_shell_fprintf_print(const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  wall_vfprintf(SHELL_NORMAL, fmt, args);
  va_end(args);
}

void bt_shell_fprintf_warn(const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  wall_vfprintf(SHELL_WARNING, fmt, args);
  va_end(args);
}

void bt_shell_fprintf_error(const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  wall_vfprintf(SHELL_ERROR, fmt, args);
  va_end(args);
}
