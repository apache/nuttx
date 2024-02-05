/****************************************************************************
 * include/nuttx/rpmsg/rpmsg.h
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

#ifdef CONFIG_RPMSG

#include <nuttx/fs/ioctl.h>
#include <nuttx/rpmsg/rpmsg_ping.h>
#include <openamp/rpmsg.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSGIOC_START              _RPMSGIOC(1)
#define RPMSGIOC_STOP               _RPMSGIOC(2)
#define RPMSGIOC_RESET              _RPMSGIOC(3)
#define RPMSGIOC_PANIC              _RPMSGIOC(4)
#define RPMSGIOC_DUMP               _RPMSGIOC(5)
#define RPMSGIOC_PING               _RPMSGIOC(6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rpmsg_s
{
  struct metal_list            bind;
  rmutex_t                     lock;
  struct metal_list            node;
  FAR const struct rpmsg_ops_s *ops;
#ifdef CONFIG_RPMSG_PING
  struct rpmsg_endpoint        ping;
#endif
  struct rpmsg_device          rdev[0];
};

/**
 * struct rpmsg_ops_s - Rpmsg device operations
 * wait: wait sem.
 * post: post sem.
 * get_cpuname: get cpu name.
 * get_tx_buffer_size: get tx buffer size.
 * get_rx_buffer_size: get rx buffer size.
 */

struct rpmsg_ops_s
{
  CODE int (*wait)(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
  CODE int (*post)(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
  CODE int (*ioctl)(FAR struct rpmsg_s *rpmsg, int cmd, unsigned long arg);
  CODE FAR const char *(*get_cpuname)(FAR struct rpmsg_s *rpmsg);
  CODE int (*get_tx_buffer_size)(FAR struct rpmsg_s *rpmsg);
  CODE int (*get_rx_buffer_size)(FAR struct rpmsg_s *rpmsg);
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
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int rpmsg_wait(FAR struct rpmsg_endpoint *ept, FAR sem_t *sem);
int rpmsg_post(FAR struct rpmsg_endpoint *ept, FAR sem_t *sem);

FAR const char *rpmsg_get_cpuname(FAR struct rpmsg_device *rdev);

int rpmsg_get_tx_buffer_size(FAR struct rpmsg_device *rdev);
int rpmsg_get_rx_buffer_size(FAR struct rpmsg_device *rdev);

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
void rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                   FAR const char *name, uint32_t dest);
void rpmsg_ns_unbind(FAR struct rpmsg_device *rdev,
                     FAR const char *name, uint32_t dest);
void rpmsg_device_created(FAR struct rpmsg_s *rpmsg);
void rpmsg_device_destory(FAR struct rpmsg_s *rpmsg);
int rpmsg_register(FAR const char *path, FAR struct rpmsg_s *rpmsg,
                   FAR const struct rpmsg_ops_s *ops);
void rpmsg_unregister(FAR const char *path, FAR struct rpmsg_s *rpmsg);
int rpmsg_ioctl(FAR const char *cpuname, int cmd, unsigned long arg);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RPMSG */
#endif /* __INCLUDE_NUTTX_RPMSG_RPMSG_H */
