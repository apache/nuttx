/****************************************************************************
 * drivers/virtio/virtio-rpmsg.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/rpmsg/rpmsg_virtio.h>
#include <nuttx/virtio/virtio.h>

#include "virtio-rpmsg.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_rpmsg_driver =
{
  LIST_INITIAL_VALUE(g_virtio_rpmsg_driver.node), /* Node */
  VIRTIO_ID_RPMSG,                                /* Device id */
  rpmsg_virtio_probe,                             /* Probe */
  rpmsg_virtio_remove,                            /* Remove */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_rpmsg_driver
 ****************************************************************************/

int virtio_register_rpmsg_driver(void)
{
  return virtio_register_driver(&g_virtio_rpmsg_driver);
}
