/****************************************************************************
 * drivers/rptun/rptun_dump.c
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

#include <nuttx/rptun/openamp.h>
#include <nuttx/rptun/rptun.h>
#include <metal/utilities.h>

#include <rpmsg/rpmsg_internal.h>

#include "rptun.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rptun_dump_addr(FAR struct rpmsg_device *rdev,
                            FAR void *addr, bool rx)
{
  FAR struct rpmsg_hdr *hdr = addr;
  FAR struct rpmsg_endpoint *ept;

  ept = rpmsg_get_ept_from_addr(rdev, rx ? hdr->dst : hdr->src);
  if (ept)
    {
      metal_log(METAL_LOG_EMERGENCY,
                "      %s buffer %p hold by %s\n",
                rx ? "RX" : "TX", hdr, ept->name);
    }
}

static void rptun_dump_buffer(FAR struct rpmsg_virtio_device *rvdev,
                              bool rx)
{
  FAR struct virtqueue *vq = rx ? rvdev->rvq : rvdev->svq;
  FAR void *addr;
  int desc_idx;
  int num;
  int i;

  num = rptun_buffer_nused(rvdev, rx);
  metal_log(METAL_LOG_EMERGENCY,
            "    %s buffer, total %d, pending %d\n",
            rx ? "RX" : "TX", vq->vq_nentries, num);

  for (i = 0; i < num; i++)
    {
      if ((rpmsg_virtio_get_role(rvdev) == RPMSG_HOST) ^ rx)
        {
          desc_idx = (vq->vq_ring.used->idx + i) & (vq->vq_nentries - 1);
          desc_idx = vq->vq_ring.avail->ring[desc_idx];
        }
      else
        {
          desc_idx = (vq->vq_ring.avail->idx + i) & (vq->vq_nentries - 1);
          desc_idx = vq->vq_ring.used->ring[desc_idx].id;
        }

      addr = metal_io_phys_to_virt(vq->shm_io,
                                   vq->vq_ring.desc[desc_idx].addr);
      if (addr)
        {
          rptun_dump_addr(&rvdev->rdev, addr, rx);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rptun_dump(FAR struct rpmsg_virtio_device *rvdev)
{
  FAR struct rpmsg_device *rdev = &rvdev->rdev;
  FAR struct rpmsg_endpoint *ept;
  FAR struct metal_list *node;

  if (!rvdev->vdev)
    {
      return;
    }

  if (!up_interrupt_context() && !sched_idletask())
    {
      metal_mutex_acquire(&rdev->lock);
    }

  metal_log(METAL_LOG_EMERGENCY,
            "Dump rpmsg info between cpu (master: %s)%s <==> %s:\n",
            rpmsg_virtio_get_role(rvdev) == RPMSG_HOST ? "yes" : "no",
            CONFIG_RPTUN_LOCAL_CPUNAME, rpmsg_get_cpuname(rdev));

  metal_log(METAL_LOG_EMERGENCY, "rpmsg vq RX:\n");
  virtqueue_dump(rvdev->rvq);
  metal_log(METAL_LOG_EMERGENCY, "rpmsg vq TX:\n");
  virtqueue_dump(rvdev->svq);

  metal_log(METAL_LOG_EMERGENCY, "  rpmsg ept list:\n");

  metal_list_for_each(&rdev->endpoints, node)
    {
      ept = metal_container_of(node, struct rpmsg_endpoint, node);
      metal_log(METAL_LOG_EMERGENCY, "    ept %s\n", ept->name);
    }

  metal_log(METAL_LOG_EMERGENCY, "  rpmsg buffer list:\n");

  rptun_dump_buffer(rvdev, true);
  rptun_dump_buffer(rvdev, false);

  if (!up_interrupt_context() && !sched_idletask())
    {
      metal_mutex_release(&rdev->lock);
    }
}
