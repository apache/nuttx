/****************************************************************************
 * drivers/coresight/coresight_funnel.c
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

#include <nuttx/coresight/coresight_funnel.h>

#include "coresight_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Funnel registers */

#define FUNNEL_FUNCTL           0x000
#define FUNNEL_PRICTL           0x004

#define FUNNEL_HOLDTIME_MASK    0xf00
#define FUNNEL_HOLDTIME_SHFT    0x8
#define FUNNEL_HOLDTIME         (0x7 << FUNNEL_HOLDTIME_SHFT)
#define FUNNEL_ENS_MASK         0xff

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int funnel_enable(FAR struct coresight_dev_s *csdev,
                         int iport, int oport);
static void funnel_disable(FAR struct coresight_dev_s *csdev,
                           int iport, int oport);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct coresight_link_ops_s g_funnel_link_ops =
{
  .enable  = funnel_enable,
  .disable = funnel_disable,
};

static const struct coresight_ops_s g_funnel_ops =
{
  .link_ops = &g_funnel_link_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: funnel_hw_enable
 ****************************************************************************/

static int funnel_hw_enable(FAR struct coresight_funnel_dev_s *fundev,
                            int port)
{
  uint32_t functl;
  int ret = 0;

  coresight_unlock(fundev->csdev.addr);
  functl = coresight_get32(fundev->csdev.addr + FUNNEL_FUNCTL);
  coresight_lock(fundev->csdev.addr);

  /* Only claim the device when the first slave port is enabled */

  if (!(functl & FUNNEL_ENS_MASK))
    {
      ret = coresight_claim_device(fundev->csdev.addr);
      if (ret < 0)
        {
          cserr("%s claim failed\n", fundev->csdev.name);
          return ret;
        }
    }

  coresight_unlock(fundev->csdev.addr);
  functl &= ~FUNNEL_HOLDTIME_MASK;
  functl |= FUNNEL_HOLDTIME;
  functl |= 1 << port;
  coresight_put32(functl, fundev->csdev.addr + FUNNEL_FUNCTL);
  coresight_put32(fundev->priority, fundev->csdev.addr + FUNNEL_PRICTL);
  coresight_lock(fundev->csdev.addr);

  return ret;
}

/****************************************************************************
 * Name: funnel_hw_disable
 ****************************************************************************/

static void funnel_hw_disable(FAR struct coresight_funnel_dev_s *fundev,
                              int port)
{
  uint32_t functl;

  coresight_unlock(fundev->csdev.addr);
  functl = coresight_get32(fundev->csdev.addr + FUNNEL_FUNCTL);
  functl &= ~(1 << port);
  coresight_put32(functl, fundev->csdev.addr + FUNNEL_FUNCTL);
  coresight_lock(fundev->csdev.addr);

  if (!(functl & FUNNEL_ENS_MASK))
    {
      coresight_disclaim_device(fundev->csdev.addr);
    }
}

/****************************************************************************
 * Name: funnel_enable
 ****************************************************************************/

static int funnel_enable(FAR struct coresight_dev_s *csdev,
                         int iport, int oport)
{
  FAR struct coresight_funnel_dev_s *fundev =
    (FAR struct coresight_funnel_dev_s *)csdev;
  int ret = 0;

  if (fundev->port_refcnt[iport]++ == 0)
    {
      ret = funnel_hw_enable(fundev, iport);
      if (ret < 0)
        {
          fundev->port_refcnt[iport]--;
          cserr("%s inport %d enabled failed\n", csdev->name, iport);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: funnel_disable
 ****************************************************************************/

static void funnel_disable(FAR struct coresight_dev_s *csdev,
                           int iport, int oport)
{
  FAR struct coresight_funnel_dev_s *fundev =
    (FAR struct coresight_funnel_dev_s *)csdev;

  if (--fundev->port_refcnt[iport] == 0)
    {
      funnel_hw_disable(fundev, iport);
      csinfo("%s inport %d disabled\n", csdev->name, iport);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: set_funnel_priority
 *
 * Description:
 *   Set funnel ports priority. It should to be called when port has not been
 *   enabled.
 *
 * Input Parameters:
 *   fundev  - Pointer to the funnel coresight device.
 *   priority- Priority to set.
 *
 ****************************************************************************/

void set_funnel_priority(FAR struct coresight_funnel_dev_s *fundev,
                         uint32_t priority)
{
  fundev->priority = priority;
}

/****************************************************************************
 * Name: funnel_register
 *
 * Description:
 *   Register a funnel devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to a funnel device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_funnel_dev_s *
funnel_register(FAR const struct coresight_desc_s *desc)
{
  FAR struct coresight_funnel_dev_s *fundev;
  FAR struct coresight_dev_s *csdev;
  int ret;

  fundev = kmm_zalloc(sizeof(struct coresight_funnel_dev_s) +
                      sizeof(uint8_t) * desc->inport_num);
  if (fundev == NULL)
    {
      cserr("%s:malloc failed!\n", desc->name);
      return NULL;
    }

  fundev->port_num = desc->inport_num;
  csdev = &fundev->csdev;
  csdev->ops = &g_funnel_ops;
  ret = coresight_register(csdev, desc);
  if (ret < 0)
    {
      kmm_free(fundev);
      cserr("%s: register failed\n", desc->name);
      return NULL;
    }

  return fundev;
}

/****************************************************************************
 * Name: funnel_unregister
 *
 * Description:
 *   Unregister a funnel devices.
 *
 * Input Parameters:
 *   fundev  - Pointer to the funnel device.
 *
 ****************************************************************************/

void funnel_unregister(FAR struct coresight_funnel_dev_s *fundev)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (fundev->csdev.refcnt > 0)
    {
      int i;

      for (i = 0; i < fundev->port_num; i++)
        {
          if (fundev->port_refcnt[i] > 0)
            {
              funnel_hw_disable(fundev, i);
            }
        }
    }

  leave_critical_section(flags);
  coresight_unregister(&fundev->csdev);

  kmm_free(fundev);
}
