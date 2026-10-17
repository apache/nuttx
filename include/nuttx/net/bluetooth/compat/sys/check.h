/****************************************************************************
 * include/nuttx/net/bluetooth/compat/sys/check.h
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

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SYS_CHECK_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SYS_CHECK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>

#if defined(CONFIG_ASSERT_ON_ERRORS)
#define CHECKIF(expr)                                                                              \
  DEBUGASSERT(!(expr));                                                                            \
  if (0)
#elif defined(CONFIG_NO_RUNTIME_CHECKS)
#define CHECKIF(...) if (0)
#else
#define CHECKIF(expr) if (expr)
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SYS_CHECK_H */
