/****************************************************************************
 * include/nuttx/rpmsg/rpmsg.h
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

#ifndef __INCLUDE_NUTTX_RPMSG_RPMSG_H
#define __INCLUDE_NUTTX_RPMSG_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <nuttx/atomic.h>
#include <nuttx/compiler.h>

#include <alloca.h>

#ifdef CONFIG_RPMSG

#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <openamp/rpmsg.h>
#include <openamp/rpmsg_internal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSGIOC_PANIC                  _RPMSGIOC(1)
#define RPMSGIOC_DUMP                   _RPMSGIOC(2)
#define RPMSGIOC_PING                   _RPMSGIOC(3)
#define RPMSGIOC_TEST                   _RPMSGIOC(4)
#define RPMSGIOC_RUNNING                _RPMSGIOC(5)
#define RPMSGIOC_ACQUIRE_WAKELOCK       _RPMSGIOC(6)
#define RPMSGIOC_RELEASE_WAKELOCK       _RPMSGIOC(7)

/* Rpmsg IOCTL for the rpmsg ctrldev */

#define RPMSG_CREATE_EPT_IOCTL          _RPMSGIOC(8)
#define RPMSG_CREATE_DEV_IOCTL          _RPMSGIOC(9)
#define RPMSG_RELEASE_DEV_IOCTL         _RPMSGIOC(10)

/* Rpmsg IOCTL for the rpmsg eptdev */

#define RPMSG_DESTROY_EPT_IOCTL         _RPMSGIOC(11)
#define RPMSG_GET_OUTGOING_FLOWCONTROL  _RPMSGIOC(12)
#define RPMSG_SET_INCOMING_FLOWCONTROL  _RPMSGIOC(13)

#define RPMSG_SIGNAL_RUNNING            TIOCM_CD

#if CONFIG_RPMSG_POOL_COUNT <= 0
#  define rpmsg_pool_alloc(size)        alloca(size)
#  define rpmsg_pool_free(addr)
#endif

#define rpmsg_wait(e, s)                rpmsg_tickwait(e, s, CLOCK_MAX)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Used for ioctl RPMSGIOC_PING */

struct rpmsg_ping_s
{
  int times;
  int len;
  int cmd;
  int sleep; /* unit: ms */
};

/* Used for ioctl RPMSG_CREATE_EPT_IOCTL, RPMSG_CREATE_DEV_IOCTL and
 * RPMSG_RELEASE_DEV_IOCTL
 */

struct rpmsg_endpoint_info
{
  /* Name of rpmsg service */

  char name[RPMSG_NAME_SIZE];

  /* Local address. To set to RPMSG_ADDR_ANY if not used. */

  uint32_t src;

  /* Destination address. To set to RPMSG_ADDR_ANY if not used. */

  uint32_t dst;
};

struct rpmsg_timestamp_s
{
  uint64_t tx_nsec;
  uint64_t rx_nsec;
};

CODE typedef void (*rpmsg_dev_cb_t)(FAR struct rpmsg_device *rdev,
                                    FAR void *priv);
CODE typedef bool (*rpmsg_match_cb_t)(FAR struct rpmsg_device *rdev,
                                      FAR void *priv, FAR const char *name,
                                      uint32_t dest);
CODE typedef void (*rpmsg_bind_cb_t)(FAR struct rpmsg_device *rdev,
                                     FAR void *priv, FAR const char *name,
                                     uint32_t dest);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

void rpmsg_initialize(void);

int rpmsg_tickwait(FAR struct rpmsg_endpoint *ept, FAR sem_t *sem,
                   clock_t delay);
int rpmsg_post(FAR struct rpmsg_endpoint *ept, FAR sem_t *sem);

FAR const char *rpmsg_get_local_cpuname(FAR struct rpmsg_device *rdev);
FAR const char *rpmsg_get_cpuname(FAR struct rpmsg_device *rdev);
int rpmsg_get_signals(FAR struct rpmsg_device *rdev);
int rpmsg_get_timestamp(FAR struct rpmsg_device *rdev, FAR const void *data,
                        FAR struct rpmsg_timestamp_s *ts);
bool rpmsg_support_alloc_buf(FAR struct rpmsg_device *rdev);
FAR void *rpmsg_alloc_buf(FAR struct rpmsg_device *rdev, size_t size,
                          size_t align);
void rpmsg_free_buf(FAR struct rpmsg_device *rdev, FAR void *addr);

static inline_function bool rpmsg_is_running(FAR struct rpmsg_device *rdev)
{
  return rpmsg_get_signals(rdev) & RPMSG_SIGNAL_RUNNING;
}

#if CONFIG_RPMSG_POOL_COUNT > 0
FAR void *rpmsg_pool_alloc(size_t size);
void rpmsg_pool_free(FAR void *addr);
#endif

int rpmsg_register_callback(FAR void *priv,
                            rpmsg_dev_cb_t device_created,
                            rpmsg_dev_cb_t device_destroy,
                            rpmsg_match_cb_t ns_match,
                            rpmsg_bind_cb_t ns_bind);
void rpmsg_unregister_callback(FAR void *priv,
                               rpmsg_dev_cb_t device_created,
                               rpmsg_dev_cb_t device_destroy,
                               rpmsg_match_cb_t ns_match,
                               rpmsg_bind_cb_t ns_bind);

int rpmsg_ioctl(FAR const char *cpuname, int cmd, unsigned long arg);
int rpmsg_panic(FAR const char *cpuname);
void rpmsg_dump_all(void);

#ifdef CONFIG_RPMSG_TRACE
void rpmsg_trace(FAR const char *name, bool bt, FAR const void *buf,
                 size_t len, FAR const char *format, ...)
                 printf_like(5, 6);
void rpmsg_vtrace(FAR const char *name, bool bt, FAR const void *buf,
                  size_t len, FAR const char *format, va_list ap)
                  printf_like(5, 0);
#else
#  define rpmsg_trace(name, bt, buf, len, format, ...) \
     do \
       { \
         (void)(name); \
         (void)(buf); \
         (void)(len); \
       } \
     while (0)
#  define rpmsg_vtrace(name, bt, buf, len, format, ap) \
     do \
       { \
         (void)(name); \
         (void)(buf); \
         (void)(len); \
       } \
     while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RPMSG */
#endif /* __INCLUDE_NUTTX_RPMSG_RPMSG_H */
