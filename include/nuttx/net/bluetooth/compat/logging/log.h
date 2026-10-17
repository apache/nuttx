/****************************************************************************
 * include/nuttx/net/bluetooth/compat/logging/log.h
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
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_LOGGING_LOG_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_LOGGING_LOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <syslog.h>

#ifndef CONFIG_BT_LOG_LEVEL
#  define CONFIG_BT_LOG_LEVEL 3
#endif

enum
{
  ZEPHYR_LOG_ERR_PRIORITY = LOG_ERR,
  ZEPHYR_LOG_WRN_PRIORITY = LOG_WARNING,
  ZEPHYR_LOG_INF_PRIORITY = LOG_INFO,
  ZEPHYR_LOG_DBG_PRIORITY = LOG_DEBUG,
};

/* NuttX syslog.h defines LOG_ERR (3), LOG_WARNING (4), LOG_INFO (6),
 * LOG_DEBUG (7) as priority constants. These conflict with Zephyr's
 * function-like LOG_ERR macro. Undefine LOG_ERR so Zephyr code works.
 */
#undef LOG_ERR

/* Module registration stubs */
#define LOG_MODULE_REGISTER(name, ...)
#define LOG_MODULE_DECLARE(name, ...)

/* Log levels (map to NuttX syslog priorities) */
#if CONFIG_BT_LOG_LEVEL >= 1
#  define LOG_ERR(fmt, ...)  syslog(ZEPHYR_LOG_ERR_PRIORITY, \
                                    "<err> %s: " fmt "\n", __func__, \
                                    ##__VA_ARGS__)
#else
#  define LOG_ERR(fmt, ...)  do { if (0) { syslog(ZEPHYR_LOG_ERR_PRIORITY, \
                                    "<err> %s: " fmt "\n", __func__, \
                                    ##__VA_ARGS__); } } while (0)
#endif

#if CONFIG_BT_LOG_LEVEL >= 2
#  define LOG_WRN(fmt, ...)  syslog(ZEPHYR_LOG_WRN_PRIORITY, \
                                    "<wrn> %s: " fmt "\n", __func__, \
                                    ##__VA_ARGS__)
#else
#  define LOG_WRN(fmt, ...)  do { if (0) { syslog(ZEPHYR_LOG_WRN_PRIORITY, \
                                    "<wrn> %s: " fmt "\n", __func__, \
                                    ##__VA_ARGS__); } } while (0)
#endif

#if CONFIG_BT_LOG_LEVEL >= 3
#  define LOG_INF(fmt, ...)  syslog(ZEPHYR_LOG_INF_PRIORITY, \
                                    "<inf> %s: " fmt "\n", __func__, \
                                    ##__VA_ARGS__)
#else
#  define LOG_INF(fmt, ...)  do { if (0) { syslog(ZEPHYR_LOG_INF_PRIORITY, \
                                    "<inf> %s: " fmt "\n", __func__, \
                                    ##__VA_ARGS__); } } while (0)
#endif

#if CONFIG_BT_LOG_LEVEL >= 4
#  define LOG_DBG(fmt, ...)  syslog(ZEPHYR_LOG_DBG_PRIORITY, \
                                    "<dbg> %s: " fmt "\n", __func__, \
                                    ##__VA_ARGS__)
#else
#  define LOG_DBG(fmt, ...)  do { if (0) { syslog(ZEPHYR_LOG_DBG_PRIORITY, \
                                    "<dbg> %s: " fmt "\n", __func__, \
                                    ##__VA_ARGS__); } } while (0)
#endif

/* Rate-limited logging compatibility.  The current NuttX Bluetooth
 * compatibility layer does not provide Zephyr's log-rate limiter state, so
 * keep these as normal log calls rather than leaving them as unresolved
 * function-like symbols.
 */
#define LOG_ERR_RATELIMIT(fmt, ...) LOG_ERR(fmt, ##__VA_ARGS__)
#define LOG_WRN_RATELIMIT(fmt, ...) LOG_WRN(fmt, ##__VA_ARGS__)
#define LOG_INF_RATELIMIT(fmt, ...) LOG_INF(fmt, ##__VA_ARGS__)
#define LOG_DBG_RATELIMIT(fmt, ...) LOG_DBG(fmt, ##__VA_ARGS__)

#define LOG_ERR_RATELIMIT_RATE(rate, fmt, ...) LOG_ERR(fmt, ##__VA_ARGS__)
#define LOG_WRN_RATELIMIT_RATE(rate, fmt, ...) LOG_WRN(fmt, ##__VA_ARGS__)
#define LOG_INF_RATELIMIT_RATE(rate, fmt, ...) LOG_INF(fmt, ##__VA_ARGS__)
#define LOG_DBG_RATELIMIT_RATE(rate, fmt, ...) LOG_DBG(fmt, ##__VA_ARGS__)

/* Hex dump */
#define LOG_HEXDUMP_DBG(_data, _length, _str)
#define LOG_HEXDUMP_INF(_data, _length, _str)
#define LOG_HEXDUMP_WRN(_data, _length, _str)
#define LOG_HEXDUMP_ERR(_data, _length, _str)

/* Printk fallback */
#define LOG_PRINTK(fmt, ...) syslog(LOG_INFO, fmt, ##__VA_ARGS__)

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_LOGGING_LOG_H */
