/****************************************************************************
 * net/vsock/vsock.h
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

#ifndef __NET_VSOCK_VSOCK_H
#define __NET_VSOCK_VSOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_VSOCK

#include <debug.h>
#include <sys/vm_sockets.h>

#include <nuttx/list.h>
#include <nuttx/net/net.h>
#include <nuttx/virtio/virtio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VSOCK_PKT_LEN   sizeof(struct vsock_pkt_s)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct vsock_pkt_s
{
  size_t                  vbidx;
  size_t                  vboff;
  size_t                  vbcnt;
  size_t                  len;
  FAR void               *priv;
  FAR struct vsock_pkt_s *next;
  struct virtqueue_buf    vb[CONFIG_NET_VSOCK_PKT_BUFCOUNT];
};

struct vsock_transport_s;
struct vsock_transport_ops_s
{
  /* Virtio/Vhost/Local transport operations */

  CODE uint64_t (*get_localcid)(FAR struct vsock_transport_s *t);
  CODE uint64_t (*get_remotecid)(FAR struct vsock_transport_s *t);
  CODE int (*alloc_pkt)(FAR struct vsock_transport_s *t,
                        FAR struct vsock_pkt_s *pkt,
                        size_t data_len);
  CODE ssize_t (*send_pkt)(FAR struct vsock_transport_s *t,
                           FAR struct vsock_pkt_s *pkt);
};

struct vsock_transport_s
{
  FAR const struct vsock_transport_ops_s *ops;
  FAR struct list_node                    node;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

extern const struct sock_intf_s g_vsock_sockif;

/****************************************************************************
 * Public Function
 ****************************************************************************/

static inline_function FAR struct vsock_hdr_s *
vsock_pkt2hdr(FAR struct vsock_pkt_s *pkt)
{
  return (FAR struct vsock_hdr_s *)pkt->vb[0].buf;
}

void vsock_recv_pkt(FAR struct vsock_transport_s *t,
                    FAR struct vsock_pkt_s *pkt);

int vsock_queue_work(FAR struct work_s *work, worker_t worker, FAR void *arg,
                     clock_t delay);

/* Register vsock transport layer to the vsock interface layer, should be
 * called by the vsock transport layer only.
 */

void vsock_transport_register(FAR struct vsock_transport_s *t);

/* Virtual socket address operation, these functions implemented in
 * vsock_addr.c
 */

int vsock_addr_is_valid(FAR const struct sockaddr *addr, socklen_t addrlen);
int vsock_addr_set(FAR struct sockaddr_vm *vmaddr,
                   FAR const struct sockaddr *addr,
                   socklen_t addrlen);
int vsock_addr_get(FAR const struct sockaddr_vm *vmaddr,
                   FAR struct sockaddr *addr,
                   FAR socklen_t *addrlen);
void vsock_addr_init(FAR struct sockaddr_vm *addr, uint64_t cid,
                     uint32_t port);
bool vsock_addr_equal(FAR const struct sockaddr_vm *addr1,
                      FAR const struct sockaddr_vm *addr2);

/* Virtual socket initialize */

int vsock_initialize(void);

#ifdef CONFIG_NET_VSOCK_VIRTIO
int vsock_virtio_initialize(void);
#endif

#ifdef CONFIG_NET_VSOCK_VHOST
int vsock_vhost_initialize(void);
#endif

#ifdef CONFIG_NET_VSOCK_LOCAL
int vsock_local_initialize(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_VSOCK */

#endif /* __NET_VSOCK_VSOCK_H */
