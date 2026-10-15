/****************************************************************************
 * include/nuttx/net/bluetooth/compat/device.h
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

/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DEVICE_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DEVICE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <nuttx/compiler.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Minimal device model for BT HCI driver binding */

struct device_state
{
  bool initialized;
  int init_res;
};

struct device
{
  const char *name;
  const void *config;
  const void *api;
  void *data;
  struct device_state *state;
};

static inline bool device_is_ready(const struct device *dev)
{
  if (dev == NULL)
    {
      return true;
    }

  return dev->state && dev->state->initialized && dev->state->init_res == 0;
}

#define DEVICE_DECLARE(name) extern const struct device name
#define DEVICE_NAME_GET(dev) ((dev)->name)

/* NuttX-compatible HCI driver interface */

struct bt_hci_driver
{
  const struct device dev;
  int (*open)(void);
  int (*send)(const uint8_t *data, size_t len);
  int (*close)(void);
  void *priv;
};

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DEVICE_H */
