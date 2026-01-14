/****************************************************************************
 * include/nuttx/rpmsg/rpmsg_virtio_lite.h
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

#ifndef __INCLUDE_NUTTX_RPMSG_RPMSG_VIRTIO_LITE_H
#define __INCLUDE_NUTTX_RPMSG_RPMSG_VIRTIO_LITE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RPMSG_VIRTIO_LITE

#include <nuttx/rpmsg/rpmsg.h>
#include <openamp/rpmsg_virtio.h>
#include <openamp/remoteproc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_VIRTIO_LITE_NOTIFY_ALL UINT32_MAX

#define RPMSG_VIRTIO_LITE_CMD_PANIC  0x1
#define RPMSG_VIRTIO_LITE_CMD_MASK   0xffff
#define RPMSG_VIRTIO_LITE_CMD_SHIFT  16

#define RPMSG_VIRTIO_LITE_CMD(c,v)   (((c) << RPMSG_VIRTIO_LITE_CMD_SHIFT) | \
                                      ((v) & RPMSG_VIRTIO_LITE_CMD_MASK))
#define RPMSG_VIRTIO_LITE_GET_CMD(c) ((c) >> RPMSG_VIRTIO_LITE_CMD_SHIFT)

#define RPMSG_VIRTIO_LITE_RSC2CMD(r) \
  ((FAR struct rpmsg_virtio_lite_cmd_s *) \
   &((FAR struct resource_table *)(r))->reserved[0])

/* Access macros ************************************************************/

/****************************************************************************
 * Name: RPMSG_VIRTIO_LITE_GET_LOCAL_CPUNAME
 *
 * Description:
 *   Get remote cpu name
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   Cpu name on success, NULL on failure.
 *
 ****************************************************************************/

#define RPMSG_VIRTIO_LITE_GET_LOCAL_CPUNAME(d) \
  ((d)->ops->get_local_cpuname ? (d)->ops->get_local_cpuname(d) : "")

/****************************************************************************
 * Name: RPMSG_VIRTIO_LITE_GET_CPUNAME
 *
 * Description:
 *   Get remote cpu name
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   Cpu name on success, NULL on failure.
 *
 ****************************************************************************/

#define RPMSG_VIRTIO_LITE_GET_CPUNAME(d) \
  ((d)->ops->get_cpuname ? (d)->ops->get_cpuname(d) : "")

/****************************************************************************
 * Name: RPMSG_VIRTIO_LITE_GET_RESOURCE
 *
 * Description:
 *   Get rpmsg virtio resource
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   Resource pointer on success, NULL on failure
 *
 ****************************************************************************/

#define RPMSG_VIRTIO_LITE_GET_RESOURCE(d) \
  ((d)->ops->get_resource ? (d)->ops->get_resource(d) : NULL)

/****************************************************************************
 * Name: RPMSG_VIRTIO_LITE_IS_MASTER
 *
 * Description:
 *   Is master or not
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   True master, false remote
 *
 ****************************************************************************/

#define RPMSG_VIRTIO_LITE_IS_MASTER(d) \
  ((d)->ops->is_master ? (d)->ops->is_master(d) : false)

/****************************************************************************
 * Name: RPMSG_VIRTIO_LITE_REGISTER_CALLBACK
 *
 * Description:
 *   Attach to receive a callback when something is received on RPTUN
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   callback - The function to be called when something has been received
 *   arg      - A caller provided value to return with the callback
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPMSG_VIRTIO_LITE_REGISTER_CALLBACK(d,c,a) \
  ((d)->ops->register_callback ? (d)->ops->register_callback(d,c,a) : -ENOSYS)

/****************************************************************************
 * Name: RPMSG_VIRTIO_LITE_NOTIFY
 *
 * Description:
 *   Notify remote core there is a message to get.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   vqid - Message to notify
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPMSG_VIRTIO_LITE_NOTIFY(d,v) \
  ((d)->ops->notify ? (d)->ops->notify(d,v) : -ENOSYS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE int (*rpmsg_virtio_callback_t)(FAR void *arg, uint32_t vqid);

begin_packed_struct struct rpmsg_virtio_lite_cmd_s
{
  uint32_t cmd_master;
  uint32_t cmd_slave;
} end_packed_struct;

struct aligned_data(8) rpmsg_virtio_lite_rsc_s
{
  struct resource_table    rsc_tbl_hdr;
  uint32_t                 offset[2];
  struct fw_rsc_trace      log_trace;
  struct fw_rsc_vdev       rpmsg_vdev;
  struct fw_rsc_vdev_vring rpmsg_vring0;
  struct fw_rsc_vdev_vring rpmsg_vring1;
  struct fw_rsc_config     config;
};

struct rpmsg_virtio_lite_s;
struct rpmsg_virtio_lite_ops_s
{
  CODE FAR const char *
  (*get_local_cpuname)(FAR struct rpmsg_virtio_lite_s *dev);
  CODE FAR const char *(*get_cpuname)(FAR struct rpmsg_virtio_lite_s *dev);
  CODE FAR struct rpmsg_virtio_lite_rsc_s *
  (*get_resource)(FAR struct rpmsg_virtio_lite_s *dev);
  CODE int (*is_master)(FAR struct rpmsg_virtio_lite_s *dev);
  CODE int (*notify)(FAR struct rpmsg_virtio_lite_s *dev, uint32_t vqid);
  CODE int (*register_callback)(FAR struct rpmsg_virtio_lite_s *dev,
                                rpmsg_virtio_callback_t callback,
                                FAR void *arg);
};

struct rpmsg_virtio_lite_s
{
  FAR const struct rpmsg_virtio_lite_ops_s *ops;
};

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

int rpmsg_virtio_lite_initialize(FAR struct rpmsg_virtio_lite_s *dev);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RPMSG_VIRTIO_LITE */

#endif /* __INCLUDE_NUTTX_RPMSG_RPMSG_VIRTIO_LITE_H */
