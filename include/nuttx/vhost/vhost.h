/****************************************************************************
 * include/nuttx/vhost/vhost.h
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

#ifndef __INCLUDE_NUTTX_VHOST_VHOST_H
#define __INCLUDE_NUTTX_VHOST_VHOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_DRIVERS_VHOST

#include <nuttx/compiler.h>
#include <nuttx/list.h>
#include <openamp/open_amp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Wrapper the vhost API to virtio API */

#define vhost_create_virtqueues virtio_create_virtqueues
#define vhost_delete_virtqueues virtio_delete_virtqueues
#define vhost_set_status        virtio_set_status
#define vhost_get_status        virtio_get_status
#define vhost_set_features      virtio_set_features
#define vhost_get_features      virtio_get_features
#define vhost_read_config       virtio_read_config
#define vhost_write_config      virtio_write_config

/* Wrapper the struct vhost_device to struct virtio_device */

#define vhost_device            virtio_device

struct vhost_driver
{
  struct list_node   node;
  uint32_t           device;   /* device id */
  CODE int         (*probe)(FAR struct vhost_device *hdev);
  CODE void        (*remove)(FAR struct vhost_device *hdev);
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

int vhost_register_device(FAR struct vhost_device *hdev);
int vhost_register_driver(FAR struct vhost_driver *hdrv);
int vhost_unregister_driver(FAR struct vhost_driver *hdrv);
int vhost_unregister_device(FAR struct vhost_device *hdev);

/****************************************************************************
 * Name: vhost_register_drivers
 ****************************************************************************/

void vhost_register_drivers(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VHOST */

#endif /* __INCLUDE_NUTTX_VHOST_VHOST_H */
