/****************************************************************************
 * drivers/crypto/pnt/sm_port.h
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

/* Copyright 2023 NXP */

#ifndef __INCLUDE_NUTTX_CRYPTO_PNT_SM_PORT_H_
#define __INCLUDE_NUTTX_CRYPTO_PNT_SM_PORT_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SE05X_LOG_ERROR
#define SE05X_LOG_LEVEL LOG_ERR
#elif defined(CONFIG_SE05X_LOG_WARNING)
#define SE05X_LOG_LEVEL LOG_WARNING
#elif defined(CONFIG_SE05X_LOG_INFO)
#define SE05X_LOG_LEVEL LOG_INFO
#elif defined(CONFIG_SE05X_LOG_DEBUG)
#define SE05X_LOG_LEVEL LOG_DEBUG
#else
#define SE05X_LOG_LEVEL LOG_EMERG
#endif

#if SE05X_LOG_LEVEL >= LOG_ERR
#define SMLOG_E(...) syslog(LOG_ERR, __VA_ARGS__);
#else
#define SMLOG_E(...)
#endif

#if SE05X_LOG_LEVEL >= LOG_WARNING
#define SMLOG_W(...) syslog(LOG_WARNING, __VA_ARGS__);
#else
#define SMLOG_W(...)
#endif

#if SE05X_LOG_LEVEL >= LOG_INFO
#define SMLOG_I(...) syslog(LOG_INFO, __VA_ARGS__);
#else
#define SMLOG_I(...)
#endif

#if SE05X_LOG_LEVEL >= LOG_DEBUG
#define SMLOG_D(...) syslog(LOG_DEBUG, __VA_ARGS__);
#define SMLOG_AU8_D(BUF, LEN)                                                \
  syslog(LOG_DEBUG, " :");                                                   \
  for (size_t bufIndex = 0; bufIndex < LEN; bufIndex++)                      \
    {                                                                        \
      syslog(LOG_DEBUG, "%02x", BUF[bufIndex]);                              \
    }                                                                        \
  syslog(LOG_DEBUG, "\n")
#define SMLOG_MAU8_D(MSG, BUF, LEN)                                          \
  syslog(LOG_DEBUG, MSG);                                                    \
  syslog(LOG_DEBUG, " :");                                                   \
  for (size_t bufIndex = 0; bufIndex < LEN; bufIndex++)                      \
    {                                                                        \
      syslog(LOG_DEBUG, "%02x", BUF[bufIndex]);                              \
    }                                                                        \
  syslog(LOG_DEBUG, "\n")
#else
#define SMLOG_D(...)
#define SMLOG_AU8_D(BUF, LEN)
#define SMLOG_MAU8_D(MSG, BUF, LEN)
#endif

#define sm_malloc kmm_malloc
#define sm_free kmm_free

#define SM_MUTEX_DEFINE(x)
#define SM_MUTEX_INIT(x)
#define SM_MUTEX_DEINIT(x)
#define SM_MUTEX_LOCK(x)
#define SM_MUTEX_UNLOCK(x)

#endif /* __INCLUDE_NUTTX_CRYPTO_PNT_SM_PORT_H_ */
