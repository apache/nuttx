/****************************************************************************
 * net/vsock/vsock_virtio.h
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

#ifndef __NET_VSOCK_VSOCK_VIRTIO_H
#define __NET_VSOCK_VSOCK_VIRTIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/virtio/virtio-vsock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VirtIO Vsock event */

#define VIRTIO_VSOCK_EVENT_TRANSPORT_RESET 0

/* VirtIO Vsock Type: stream or seqpacket */

#define VIRTIO_VSOCK_TYPE_STREAM           1
#define VIRTIO_VSOCK_TYPE_SEQPACKET        2

/* Invalid operation */

#define VIRTIO_VSOCK_OP_INVALID            0

/* Connect operations */

#define VIRTIO_VSOCK_OP_REQUEST            1
#define VIRTIO_VSOCK_OP_RESPONSE           2
#define VIRTIO_VSOCK_OP_RST                3
#define VIRTIO_VSOCK_OP_SHUTDOWN           4

/* To send payload */

#define VIRTIO_VSOCK_OP_RW                 5

/* Tell the peer our credit info */

#define VIRTIO_VSOCK_OP_CREDIT_UPDATE      6

/* Request the peer to send the credit info to us */

#define VIRTIO_VSOCK_OP_CREDIT_REQUEST     7

/* VIRTIO_VSOCK_OP_SHUTDOWN flags values */

#define VIRTIO_VSOCK_SHUTDOWN_F_RECV       0
#define VIRTIO_VSOCK_SHUTDOWN_F_SEND       1
#define VIRTIO_VSOCK_SHUTDOWN_RECV         (1 << VIRTIO_VSOCK_SHUTDOWN_F_RECV)
#define VIRTIO_VSOCK_SHUTDOWN_SEND         (1 << VIRTIO_VSOCK_SHUTDOWN_F_SEND)
#define VIRTIO_VSOCK_SHUTDOWN              (VIRTIO_VSOCK_SHUTDOWN_RECV | VIRTIO_VSOCK_SHUTDOWN_SEND)

/* VIRTIO_VSOCK_OP_RW flags values */

#define VIRTIO_VSOCK_SEQ_EOM               (1 << 0)
#define VIRTIO_VSOCK_SEQ_EOR               (1 << 1)

#define VIRTIO_VSOCK_EVENT_LEN             sizeof(struct virtio_vsock_event_s)
#define VIRTIO_VSOCK_HDR_LEN               sizeof(struct vsock_hdr_s)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

begin_packed_struct struct vsock_hdr_s
{
  uint64_t src_cid;
  uint64_t dst_cid;
  uint32_t src_port;
  uint32_t dst_port;
  uint32_t len;
  uint16_t type;
  uint16_t op;
  uint32_t flags;
  uint32_t buf_alloc;
  uint32_t fwd_cnt;
} end_packed_struct;

begin_packed_struct struct virtio_vsock_event_s
{
  uint32_t id;
} end_packed_struct;

#endif /* __NET_VSOCK_VSOCK_VIRTIO_H */
