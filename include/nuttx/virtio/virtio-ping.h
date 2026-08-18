/****************************************************************************
 * include/nuttx/virtio/virtio-ping.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_PING_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_PING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <inttypes.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/debug.h>
#include <nuttx/virtio/virtio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ping device id.  NOT assigned by the VirtIO specification, which hands out
 * ids sequentially upwards, so picking one next to the assigned range would
 * collide with the next device the spec takes.  This sits at the top of the
 * range the modern PCI transport can express instead: that transport derives
 * the id as pci_device_id - 0x1040 and claims 0x1000..0x107f, so 63 is the
 * highest id still reachable there.  Kept beside the rest of the ping
 * definitions, as virtio-spi.h does for VIRTIO_ID_SPI.
 */

#define VIRTIO_ID_PING              63

/* Virtqueue layout, named from the driver's point of view; the device side
 * mirrors the roles.  Two queues are needed because only the driver may
 * produce into an available ring, so the device could never initiate a ping
 * on a single shared queue.
 */

#define VIRTIO_PING_VQ_TX           0  /* Driver -> device messages */
#define VIRTIO_PING_VQ_RX           1  /* Device -> driver messages */
#define VIRTIO_PING_VQ_NUM          2

/* Size of one buffer, pre-posted as writable by the receiving side.  The
 * sender clamps itself to the room it is handed.
 */

#define VIRTIO_PING_BUFLEN          2048

/* Tells a fresh peer request from the response to one of our own, so the
 * responder does not answer an answer.
 */

#define VIRTIO_PING_CMD_REQ         0
#define VIRTIO_PING_CMD_RSP         1

#define VIRTIO_PING_HDRSIZE         sizeof(struct virtio_ping_hdr_s)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Ping config space, carrying the instance name.  Kept in the ping device's
 * own config space so a ping device is self describing on every transport.
 */

begin_packed_struct struct virtio_ping_config_s
{
  char name[VIRTIO_PING_NAMELEN];  /* NUL terminated instance name */
} end_packed_struct;

/* Request/response header, shared by the driver and device sides.
 *
 * NOTE: no timestamp field on purpose.  The two cores do not necessarily
 * share a perf counter source, so all timing is done single ended by
 * whichever side initiated the ping.
 */

begin_packed_struct struct virtio_ping_hdr_s
{
  uint64_t cookie;                 /* Initiator's semaphore, echoed back */
  uint32_t cmd;                    /* VIRTIO_PING_CMD_REQ or _RSP */
  uint8_t  data[1];                /* Flexible payload tail */
} end_packed_struct;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_ping_logout
 ****************************************************************************/

static inline_function
void virtio_ping_logout(FAR const char *s, clock_t value)
{
  struct timespec ts;

  perf_convert(value, &ts);

  _emerg("%s: %jd s, %ld ns\n",
         s, (intmax_t)ts.tv_sec, ts.tv_nsec);
}

/****************************************************************************
 * Name: virtio_ping_logout_rate
 ****************************************************************************/

static inline_function
void virtio_ping_logout_rate(uint32_t len, clock_t avg)
{
  struct timespec ts;
  uint64_t ratebits;
  uint64_t nsec;

  perf_convert(avg, &ts);

  nsec = (uint64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
  if (nsec == 0)
    {
      _emerg("rate: n/a (measured interval too small)\n");
      return;
    }

  ratebits = (uint64_t)len * 8 * NSEC_PER_SEC / nsec;
  _emerg("rate: %" PRIu64 ".%06" PRIu64 " Mbits/sec\n",
         ratebits / 1000000, ratebits % 1000000);
}

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_PING_H */
