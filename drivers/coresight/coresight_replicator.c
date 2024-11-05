/****************************************************************************
 * drivers/coresight/coresight_replicator.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>

#include <nuttx/coresight/coresight_replicator.h>

#include "coresight_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Replicator registers */

#define REPLICATOR_IDFILTER0      0x000
#define REPLICATOR_IDFILTER1      0x004

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int replicator_enable(FAR struct coresight_dev_s *csdev,
                             int iport, int oport);
static void replicator_disable(FAR struct coresight_dev_s *csdev,
                               int iport, int oport);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct coresight_link_ops_s g_replicator_link_ops =
{
  .enable  = replicator_enable,
  .disable = replicator_disable,
};

static const struct coresight_ops_s g_replicator_ops =
{
  .link_ops = &g_replicator_link_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: replicator_hw_enable
 ****************************************************************************/

static int
replicator_hw_enable(FAR struct coresight_replicator_dev_s *repdev,
                     int port)
{
  uint32_t id0val;
  uint32_t id1val;
  int ret = 0;

  if (port != 0 && port != 1)
    {
      return -EINVAL;
    }

  coresight_unlock(repdev->csdev.addr);
  id0val = coresight_get32(repdev->csdev.addr + REPLICATOR_IDFILTER0);
  id1val = coresight_get32(repdev->csdev.addr + REPLICATOR_IDFILTER1);

  /* Only claim the device when the first slave port is enabled */

  if (id0val == 0xff && id1val == 0xff)
    {
      ret = coresight_claim_device(repdev->csdev.addr);
      if (ret < 0)
        {
          cserr("%s claim failed\n", repdev->csdev.name);
          coresight_lock(repdev->csdev.addr);
          return ret;
        }
    }

  switch (port)
    {
      case 0:
        coresight_put32(0x00, repdev->csdev.addr + REPLICATOR_IDFILTER0);
        break;

      case 1:
        coresight_put32(0x00, repdev->csdev.addr + REPLICATOR_IDFILTER1);
        break;

      default:
        break;
    }

  coresight_lock(repdev->csdev.addr);
  return ret;
}

/****************************************************************************
 * Name: replicator_hw_disable
 ****************************************************************************/

static void
replicator_hw_disable(FAR struct coresight_replicator_dev_s *repdev,
                      int port)
{
  uint32_t id0val;
  uint32_t id1val;
  uint32_t off;

  switch (port)
    {
      case 0:
        off = REPLICATOR_IDFILTER0;
        break;

      case 1:
        off = REPLICATOR_IDFILTER1;
        break;

      default:
        return;
    }

  coresight_unlock(repdev->csdev.addr);
  coresight_put32(0xff, repdev->csdev.addr + off);
  id0val = coresight_get32(repdev->csdev.addr + REPLICATOR_IDFILTER0);
  id1val = coresight_get32(repdev->csdev.addr + REPLICATOR_IDFILTER1);
  coresight_lock(repdev->csdev.addr);

  if (id0val == 0xff && id1val == 0xff)
    {
      coresight_disclaim_device(repdev->csdev.addr);
    }
}

/****************************************************************************
 * Name: replicator_enable
 ****************************************************************************/

static int replicator_enable(FAR struct coresight_dev_s *csdev,
                             int iport, int oport)
{
  FAR struct coresight_replicator_dev_s *repdev =
    (FAR struct coresight_replicator_dev_s *)csdev;
  int ret = 0;

  if (repdev->port_refcnt[oport]++ == 0)
    {
      ret = replicator_hw_enable(repdev, oport);
      if (ret < 0)
        {
          repdev->port_refcnt[oport]--;
          cserr("%s inport %d enabled failed\n", csdev->name, oport);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: replicator_disable
 ****************************************************************************/

static void replicator_disable(FAR struct coresight_dev_s *csdev,
                               int iport, int oport)
{
  FAR struct coresight_replicator_dev_s *repdev =
    (FAR struct coresight_replicator_dev_s *)csdev;

  if (--repdev->port_refcnt[oport] == 0)
    {
      replicator_hw_disable(repdev, oport);
      csinfo("%s inport %d disabled\n", csdev->name, oport);
    }
}

/****************************************************************************
 * Name: replicator_reset
 ****************************************************************************/

static void replicator_reset(FAR struct coresight_replicator_dev_s *repdev)
{
  if (coresight_claim_device(repdev->csdev.addr) == 0)
    {
      coresight_unlock(repdev->csdev.addr);
      coresight_put32(0xff, repdev->csdev.addr + REPLICATOR_IDFILTER0);
      coresight_put32(0xff, repdev->csdev.addr + REPLICATOR_IDFILTER1);
      coresight_lock(repdev->csdev.addr);
      coresight_disclaim_device(repdev->csdev.addr);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: replicator_register
 *
 * Description:
 *   Register a replicator devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to a replicator device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_replicator_dev_s *
replicator_register(FAR const struct coresight_desc_s *desc)
{
  FAR struct coresight_replicator_dev_s *repdev;
  FAR struct coresight_dev_s *csdev;
  int ret;

  repdev = kmm_zalloc(sizeof(struct coresight_replicator_dev_s) +
                      sizeof(uint8_t) * desc->outport_num);
  if (repdev == NULL)
    {
      cserr("%s:malloc failed!\n", desc->name);
      return NULL;
    }

  csdev = &repdev->csdev;
  csdev->ops = &g_replicator_ops;
  ret = coresight_register(csdev, desc);
  if (ret < 0)
    {
      kmm_free(repdev);
      cserr("%s: register failed\n", desc->name);
      return NULL;
    }

  replicator_reset(repdev);
  return repdev;
}

/****************************************************************************
 * Name: replicator_unregister
 *
 * Description:
 *   Unregister a replicator devices.
 *
 * Input Parameters:
 *   fundev  - Pointer to the replicator device.
 *
 ****************************************************************************/

void replicator_unregister(FAR struct coresight_replicator_dev_s *repdev)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (repdev->csdev.refcnt > 0)
    {
      int i;

      for (i = 0; i < repdev->csdev.outport_num; i++)
        {
          if (repdev->port_refcnt[i] > 0)
            {
              replicator_hw_disable(repdev, i);
            }
        }
    }

  leave_critical_section(flags);
  coresight_unregister(&repdev->csdev);

  kmm_free(repdev);
}
