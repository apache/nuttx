/****************************************************************************
 * include/nuttx/net/bluetooth/compat/settings/settings.h
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
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SETTINGS_SETTINGS_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SETTINGS_SETTINGS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Minimal settings API for BT bonding key storage.
 * Backed by filesystem or MTD flash.
 */

#define SETTINGS_MAX_DIR_DEPTH 8

typedef int (*settings_read_cb)(void *cb_arg, void *data, size_t len);
typedef int (*settings_load_direct_cb)(
  const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
  void *param);

int settings_subsys_init(void);
int settings_save_one(const char *name, const void *value, size_t val_len);
int settings_load_subtree(const char *subtree);
int settings_load_subtree_direct(const char *subtree,
                                 settings_load_direct_cb cb, void *param);
int settings_delete(const char *name);
int settings_commit(void);

/* Callback registration */

struct settings_handler
{
  const char *name;
  int (*h_set)(const char *key, size_t len, settings_read_cb read_cb,
               void *cb_arg);
  int (*h_commit)(void);
  int (*h_export)(
    int (*cb)(const char *name, const void *value, size_t val_len));
  int (*h_rename)(const char *from, const char *to);
};

#define SETTINGS_STATIC_HANDLER_DEFINE(_hname, _tree, _hget, _hset, _hcommit, _hexport)            \
static Z_DECL_ALIGN(struct settings_handler) _hname __attribute__((__used__))                    \
__attribute__((__section__(".settings_handlers"))) = {                                           \
        .name = (_tree),                                                                         \
        .h_set = (_hset),                                                                        \
        .h_commit = (_hcommit),                                                                  \
        .h_export = (_hexport),                                                                  \
}

int settings_register(struct settings_handler *handler);

/* Internal helpers */

bool settings_name_steq(const char *name,
                        const char *key, const char **next);
int settings_name_next(const char *name, const char **next);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SETTINGS_SETTINGS_H */
