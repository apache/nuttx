/****************************************************************************
 * include/nuttx/virtio/virtio-vsock.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_VSOCK_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_VSOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The feature bitmap for virtio vsock */

#define VIRTIO_VSOCK_F_STREAM            0 /* SOCK_STREAM supported */
#define VIRTIO_VSOCK_F_SEQPACKET         1 /* SOCK_SEQPACKET supported */
#define VIRTIO_VSOCK_F_NO_IMPLIED_STREAM 2 /* SOCK_STREAM is not implied */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

begin_packed_struct struct virtio_vsock_config_s
{
  uint64_t guest_cid;
} end_packed_struct;

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_VSOCK_H */
