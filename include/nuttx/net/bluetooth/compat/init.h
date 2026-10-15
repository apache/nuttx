/****************************************************************************
 * include/nuttx/net/bluetooth/compat/init.h
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
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_INIT_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_INIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "sys/iterable_sections.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Init levels */
#define PRE_KERNEL_1 0
#define PRE_KERNEL_2 1
#define POST_KERNEL 2
#define APPLICATION 3
#define SMP_BOOT 4

struct init_entry
{
  int (*init)(void);
  int level;
  int prio;
  const char *name;
};

/* Default priorities used by NuttX Bluetooth code when NuttX config does not
 * provide an equivalent symbol.
 */
#ifndef CONFIG_KERNEL_INIT_PRIORITY_DEFAULT
#define CONFIG_KERNEL_INIT_PRIORITY_DEFAULT 40
#endif

#ifndef CONFIG_APPLICATION_INIT_PRIORITY
#define CONFIG_APPLICATION_INIT_PRIORITY 90
#endif

/* SYS_INIT macro - creates an init entry collected by defines.c */
#define SYS_INIT(init_fn, init_level, init_prio)                                                   \
const struct init_entry _bt_init_##init_fn __attribute__((__used__)) = {                         \
        .init = init_fn,                                                                         \
        .level = init_level,                                                                     \
        .prio = init_prio,                                                                       \
        .name = #init_fn,                                                                        \
}

/* SYS_INIT_NAMED - same with explicit name */
#define SYS_INIT_NAMED(name, init_fn, init_level, init_prio)                                       \
const struct init_entry _bt_init_##name __attribute__((__used__)) = {                            \
        .init = init_fn,                                                                         \
        .level = init_level,                                                                     \
        .prio = init_prio,                                                                       \
        .name = #name,                                                                           \
}

int bt_sys_init_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_INIT_H */
