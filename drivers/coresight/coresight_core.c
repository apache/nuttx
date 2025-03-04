/****************************************************************************
 * drivers/coresight/coresight_core.c
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
#include <stdbool.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>

#include <nuttx/coresight/coresight.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Used for build path */

struct coresight_node_s
{
  FAR struct coresight_dev_s *csdev;
  struct list_node link;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node g_csdev_list = LIST_INITIAL_VALUE(g_csdev_list);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PM

/****************************************************************************
 * Name: coresight_notify_pm
 ****************************************************************************/

static void coresight_notify_pm(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate)
{
  FAR struct coresight_dev_s *csdev =
    container_of(cb, struct coresight_dev_s, pmcb);
  enum pm_state_e oldstate;

  if (csdev->refcnt == 0 || domain != PM_IDLE_DOMAIN)
    {
      return;
    }

  oldstate = pm_querystate(PM_IDLE_DOMAIN);
  switch (oldstate)
    {
      case PM_NORMAL:
      case PM_IDLE:
      case PM_STANDBY:
        if (pmstate == PM_SLEEP)
          {
            clk_disable(csdev->clk);
          }
        break;

      case PM_SLEEP:
        if (pmstate == PM_NORMAL || pmstate == PM_IDLE ||
            pmstate == PM_STANDBY)
          {
            if (clk_enable(csdev->clk) <= 0)
              {
                cserr("clk enable failed when pm state change\n");
              }
          }
        break;

      default:
        break;
    }
}

#endif

#ifdef CONFIG_CLK

/****************************************************************************
 * Name: coresight_enable_clk
 ****************************************************************************/

static int coresight_enable_clk(FAR struct coresight_dev_s *csdev)
{
  int ret;

  if (csdev->clk == NULL)
    {
      return 0;
    }

  ret = clk_enable(csdev->clk);
  if (ret < 0)
    {
      cserr("%s clk enable failed\n", csdev->name);
      return ret;
    }

#ifdef CONFIG_PM
  if (csdev->pmcb.notify == NULL)
    {
      csdev->pmcb.notify = coresight_notify_pm;
    }

  ret = pm_register(&csdev->pmcb);
  if (ret < 0)
    {
      clk_disable(csdev->clk);
      cserr("%s register pm failed\n", csdev->name);
      return ret;
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: coresight_disable_clk
 ****************************************************************************/

static void coresight_disable_clk(FAR struct coresight_dev_s *csdev)
{
  if (csdev->clk == NULL)
    {
      return;
    }

#ifdef CONFIG_PM
  pm_unregister(&csdev->pmcb);
#endif
  clk_disable(csdev->clk);
}

#else
#  define coresight_enable_clk(csdev) (0)
#  define coresight_disable_clk(csdev)
#endif

/****************************************************************************
 * Name: coresight_enable_sink
 ****************************************************************************/

static int coresight_enable_sink(FAR struct coresight_dev_s *csdev)
{
  int ret;

  if (csdev->ops->sink_ops->enable == NULL)
    {
      return -EINVAL;
    }

  if (csdev->refcnt++ != 0)
    {
      return 0;
    }

  ret = coresight_enable_clk(csdev);
  if (ret < 0)
    {
      csdev->refcnt--;
      return ret;
    }

  ret = csdev->ops->sink_ops->enable(csdev);
  if (ret >= 0)
    {
      return ret;
    }

  csdev->refcnt--;
  coresight_disable_clk(csdev);
  cserr("%s enable failed\n", csdev->name);
  return ret;
}

/****************************************************************************
 * Name: coresight_disable_sink
 ****************************************************************************/

static void coresight_disable_sink(FAR struct coresight_dev_s *csdev)
{
  if (csdev->ops->sink_ops->disable == NULL)
    {
      return;
    }

  if (--csdev->refcnt != 0)
    {
      return;
    }

  csdev->ops->sink_ops->disable(csdev);
  coresight_disable_clk(csdev);
}

/****************************************************************************
 * Name: coresight_find_link_inport
 ****************************************************************************/

static int coresight_find_link_inport(FAR struct coresight_dev_s *csdev,
                                      FAR struct coresight_dev_s *prev)
{
  FAR struct coresight_connect_s *conn;
  int i;

  for (i = 0; i < prev->outport_num; i++)
    {
      conn = &prev->outconns[i];
      if (conn->destdev == csdev)
        {
          return conn->destport;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: coresight_find_link_outport
 ****************************************************************************/

static int coresight_find_link_outport(FAR struct coresight_dev_s *csdev,
                                       FAR struct coresight_dev_s *next)
{
  FAR struct coresight_connect_s *conn;
  int i;

  for (i = 0; i < csdev->outport_num; i++)
    {
      conn = &csdev->outconns[i];
      if (conn->destdev == next)
        {
          return conn->srcport;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: coresight_enable_link
 ****************************************************************************/

static int coresight_enable_link(FAR struct coresight_dev_s *csdev,
                                 FAR struct coresight_dev_s *prev,
                                 FAR struct coresight_dev_s *next)
{
  int inport = 0;
  int outport = 0;
  int ret;

  if (csdev->ops->link_ops->enable == NULL)
    {
      return -EINVAL;
    }

  if (csdev->subtype.link_subtype == CORESIGHT_DEV_SUBTYPE_LINK_MERG)
    {
      inport = coresight_find_link_inport(csdev, prev);
      if (inport < 0)
        {
          return inport;
        }
    }

  if (csdev->subtype.link_subtype == CORESIGHT_DEV_SUBTYPE_LINK_SPLIT)
    {
      outport = coresight_find_link_outport(csdev, next);
      if (outport < 0)
        {
          return outport;
        }
    }

  if (csdev->refcnt++ == 0)
    {
      ret = coresight_enable_clk(csdev);
      if (ret < 0)
        {
          csdev->refcnt--;
          return ret;
        }
    }

  ret = csdev->ops->link_ops->enable(csdev, inport, outport);
  if (ret < 0)
    {
      if (--csdev->refcnt == 0)
        {
          coresight_disable_clk(csdev);
        }

      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: coresight_disable_link
 ****************************************************************************/

static void coresight_disable_link(FAR struct coresight_dev_s *csdev,
                                   FAR struct coresight_dev_s *prev,
                                   FAR struct coresight_dev_s *next)
{
  int inport;
  int outport;

  if (csdev->ops->sink_ops->disable == NULL)
    {
      return;
    }

  inport = coresight_find_link_inport(csdev, prev);
  outport = coresight_find_link_outport(csdev, next);
  csdev->ops->link_ops->disable(csdev, inport, outport);

  if (--csdev->refcnt == 0)
    {
      coresight_disable_clk(csdev);
    }
}

/****************************************************************************
 * Name: coresight_enable_source
 ****************************************************************************/

static int coresight_enable_source(FAR struct coresight_dev_s *csdev)
{
  int ret;

  if (csdev->ops->source_ops->enable == NULL)
    {
      return -EINVAL;
    }

  if (csdev->refcnt++ != 0)
    {
      return 0;
    }

  ret = coresight_enable_clk(csdev);
  if (ret < 0)
    {
      csdev->refcnt--;
      return ret;
    }

  ret = csdev->ops->source_ops->enable(csdev);
  if (ret >= 0)
    {
      return ret;
    }

  csdev->refcnt--;
  coresight_disable_clk(csdev);
  cserr("%s enable failed\n", csdev->name);
  return ret;
}

/****************************************************************************
 * Name: coresight_disable_source
 ****************************************************************************/

static void coresight_disable_source(FAR struct coresight_dev_s *csdev)
{
  if (csdev->ops->source_ops->disable == NULL)
    {
      return;
    }

  if (--csdev->refcnt != 0)
    {
      return;
    }

  csdev->ops->source_ops->disable(csdev);
  coresight_disable_clk(csdev);
}

/****************************************************************************
 * Name: coresight_validate_source
 *
 * Description:
 *   Indicate if this coresight device is a valid source device.
 *
 ****************************************************************************/

static int coresight_validate_source(FAR struct coresight_dev_s *csdev)
{
  uint8_t type = csdev->type;
  uint8_t subtype = csdev->subtype.source_subtype;

  if (type != CORESIGHT_DEV_TYPE_SOURCE)
    {
      cserr("not a source coresight device\n");
      return -EINVAL;
    }

  if (subtype != CORESIGHT_DEV_SUBTYPE_SOURCE_PROC &&
      subtype != CORESIGHT_DEV_SUBTYPE_SOURCE_SOFTWARE)
    {
      cserr("not a supported subtype of source device\n");
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: coresight_build_path
 *
 * Description:
 *   Build path from srcdev to destdev.
 *
 * Input Parameters:
 *   srcdev  - Pointer to the source device.
 *   destdev - Pointer to the destination device.
 *   path    - Pointer to the path which will save all the coresight devices
 *             through source device to destination device.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

static int coresight_build_path(FAR struct coresight_dev_s *srcdev,
                                FAR struct coresight_dev_s *destdev,
                                FAR struct list_node *path)
{
  FAR struct coresight_node_s *node;
  bool found = false;
  int i;

  if (srcdev == destdev)
    {
      goto out;
    }

  for (i = 0; i < srcdev->outport_num; i++)
    {
      struct coresight_dev_s *csdev = srcdev->outconns[i].destdev;
      if (csdev && coresight_build_path(csdev, destdev, path) == 0)
        {
          found = true;
          break;
        }
    }

  if (!found)
    {
      return -ENODEV;
    }

out:
  node = kmm_malloc(sizeof(struct coresight_node_s));
  if (node == NULL)
    {
      return -ENOMEM;
    }

  node->csdev = srcdev;
  list_add_head(path, &node->link);
  return 0;
}

/****************************************************************************
 * Name: coresight_release_path
 *
 * Description:
 *   Release memory malloced through the path.
 *
 ****************************************************************************/

static void coresight_release_path(FAR struct list_node *path)
{
  FAR struct coresight_node_s *node;
  FAR struct coresight_node_s *next;

  list_for_every_entry_safe(path, node, next, struct coresight_node_s, link)
    {
      list_delete(&node->link);
      kmm_free(node);
    }
}

/****************************************************************************
 * Name: coresight_disable_dev
 ****************************************************************************/

static void coresight_disable_dev(FAR struct coresight_node_s *node)
{
  switch (node->csdev->type)
    {
      case CORESIGHT_DEV_TYPE_SINK:
        coresight_disable_sink(node->csdev);
        break;

      case CORESIGHT_DEV_TYPE_LINK:
        {
          FAR struct coresight_node_s *prev =
            list_prev_entry(node, struct coresight_node_s, link);
          FAR struct coresight_node_s *next =
            list_next_entry(node, struct coresight_node_s, link);
          coresight_disable_link(node->csdev, prev->csdev, next->csdev);
        }
        break;

      /* We skip the first node in the path assuming that it is the source，
       * and it will be disabled in coresight_disable. So we don't expect a
       * source device in the middle of a path.
       */

      default:
        DEBUGASSERT(0);
        break;
    }
}

/****************************************************************************
 * Name: coresight_disable_path_from
 *
 * Description:
 *   Disable coresight devices from specific node.
 *
 * Input Parameters:
 *   path  - Head of the path.
 *   node  - Start position to search, it begins from next of this node to
 *           disable.
 *
 ****************************************************************************/

static void coresight_disable_path_from(FAR struct list_node *path,
                                        FAR struct coresight_node_s *node)
{
  list_for_every_entry_continue(node, path, struct coresight_node_s, link)
    {
      coresight_disable_dev(node);
    }
}

/****************************************************************************
 * Name: coresight_disable_path
 *
 * Description:
 *   Disable all the coresight devices through the path except source device.
 *   Source device will be disabled by coresight_disable or perf end
 *   function.
 *
 ****************************************************************************/

static void coresight_disable_path(FAR struct list_node *path)
{
  coresight_disable_path_from(path,
    container_of(path->next, struct coresight_node_s, link));
}

/****************************************************************************
 * Name: coresight_enable_path
 *
 * Description:
 *   Enable all coresight devices through the path in reverse order.
 *
 * Input Parameters:
 *   path  - path from source device to sink device.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

static int coresight_enable_path(FAR struct list_node *path)
{
  FAR struct coresight_node_s *node;
  int ret = 0;

  list_for_every_entry_reverse(path, node, struct coresight_node_s, link)
    {
      switch (node->csdev->type)
        {
          /* Sink device is the first device to be enable. No need to disable
           * other coresight device in the path if it enabled failed.
           */

          case CORESIGHT_DEV_TYPE_SINK:
            ret = coresight_enable_sink(node->csdev);
            if (ret < 0)
              {
                cserr("enalbe sink: %s failed ret: %d\n",
                      node->csdev->name, ret);
                return ret;
              }
            break;

          case CORESIGHT_DEV_TYPE_LINK:
            {
              FAR struct coresight_node_s *prev =
                list_prev_entry(node, struct coresight_node_s, link);
              FAR struct coresight_node_s *next =
                list_next_entry(node, struct coresight_node_s, link);
              ret = coresight_enable_link(node->csdev, prev->csdev,
                                          next->csdev);
              if (ret < 0)
                {
                  cserr("enalbe link: %s failed ret: %d\n",
                        node->csdev->name, ret);
                  goto err;
                }
            }
            break;

          /* Source device will be enabled in coresight_enable or
           * perf start function.
           */

          case CORESIGHT_DEV_TYPE_SOURCE:
            break;

          default:
            cserr("invalid coresight device type through the path\n");
            DEBUGASSERT(0);
            goto err;
      }
    }

  return ret;

err:
  coresight_disable_path_from(path, node);
  return ret;
}

/****************************************************************************
 * Name: coresight_find_dev
 ****************************************************************************/

static FAR struct coresight_dev_s *coresight_find_dev(FAR const char *name)
{
  FAR struct coresight_dev_s *tempdev;
  irqstate_t flags;

  flags = enter_critical_section();
  list_for_every_entry(&g_csdev_list, tempdev, struct coresight_dev_s, node)
    {
      if (strcmp(tempdev->name, name) == 0)
        {
          leave_critical_section(flags);
          return tempdev;
        }
    }

  leave_critical_section(flags);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: coresight_register
 *
 * Description:
 *   Register a coresight device to the coresight bus.
 *
 * Input Parameters:
 *   csdev  - Pointer to the coresight device that needs to be registered.
 *   desc   - Pointer to the attribute description of this coresight device.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_register(FAR struct coresight_dev_s *csdev,
                       FAR const struct coresight_desc_s *desc)
{
  FAR struct coresight_dev_s *tempdev;
  irqstate_t flags;
  int i;

  if (coresight_find_dev(desc->name) != NULL)
    {
      cserr("device has been registered!\n");
      return -EEXIST;
    }

  csdev->name = desc->name;
  csdev->addr = desc->addr;
  csdev->type = desc->type;
  csdev->subtype = desc->subtype;
  csdev->outport_num = desc->outport_num;
  list_initialize(&csdev->path);

#ifdef CONFIG_CLK
  if (desc->clkname != NULL)
    {
      csdev->clk = clk_get(desc->clkname);
      if (csdev->clk == NULL)
        {
          cserr("get device clk failed\n");
          return -ENODEV;
        }
    }
#endif

  if (csdev->outport_num > 0)
    {
      csdev->outconns =
        kmm_zalloc(sizeof(struct coresight_connect_s) * desc->outport_num);
      if (csdev->outconns == NULL)
        {
          return -ENOMEM;
        }

      for (i = 0; i < csdev->outport_num; i++)
        {
          FAR struct coresight_connect_s *conn = &csdev->outconns[i];
          FAR const struct coresight_portdesc_s *portdesc =
            &desc->outports[i];

          conn->srcport = i;
          conn->destport = portdesc->port;
          conn->destname = portdesc->remote;
          conn->srcdev = csdev;
        }
    }

  flags = enter_critical_section();
  list_for_every_entry(&g_csdev_list, tempdev, struct coresight_dev_s, node)
    {
      for (i = 0; i < tempdev->outport_num; i++)
        {
          FAR struct coresight_connect_s *conn = &tempdev->outconns[i];
          if (strcmp(conn->destname, csdev->name) == 0)
            {
              conn->destdev = csdev;
            }
        }

      for (i = 0; i < csdev->outport_num; i++)
        {
          FAR struct coresight_connect_s *conn = &csdev->outconns[i];
          if (strcmp(conn->destname, tempdev->name) == 0)
            {
              conn->destdev = tempdev;
            }
        }
    }

  list_add_tail(&g_csdev_list, &csdev->node);
  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: coresight_unregister
 *
 * Description:
 *   Unregister a coresight device from coresight bus.
 *
 * Input Parameters:
 *   csdev  - Pointer to the coresight device that needs to be unregistered.
 *
 ****************************************************************************/

void coresight_unregister(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_dev_s *tempdev;
  irqstate_t flags;
  int i;

  flags = enter_critical_section();
  list_for_every_entry(&g_csdev_list, tempdev, struct coresight_dev_s, node)
    {
      if (csdev == tempdev)
        {
          continue;
        }

      for (i = 0; i < tempdev->outport_num; i++)
        {
          FAR struct coresight_connect_s *conn = &tempdev->outconns[i];
          if (conn->destdev == csdev)
            {
              conn->destdev = NULL;
            }
        }
    }

  if (csdev->refcnt > 0)
    {
      switch (csdev->type)
        {
          case CORESIGHT_DEV_TYPE_SINK:
            if (csdev->ops->sink_ops->disable != NULL)
              {
                csdev->ops->sink_ops->disable(csdev);
              }
            break;

          case CORESIGHT_DEV_TYPE_SOURCE:
            if (csdev->ops->source_ops->disable != NULL)
              {
                csdev->ops->source_ops->disable(csdev);
              }
            break;

          /* Link devices may have multiple inport or outport, it can
           * not be distinguished here which one of them has been enabled.
           * so disable inport/outports in its own unregister function.
           */

          default:
            break;
        }

      coresight_disable_clk(csdev);
    }

  list_delete(&csdev->node);
  leave_critical_section(flags);

  if (csdev->outport_num > 0)
    {
      kmm_free(csdev->outconns);
      csdev->outconns = NULL;
    }
}

/****************************************************************************
 * Name: coresight_enable
 *
 * Description:
 *   Enable trace start from srcdev to destdev.
 *
 * Input Parameters:
 *   srcdev  - Source device that generates trace data.
 *   destdev - Sink device that finally accepts the trace data.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_enable(FAR struct coresight_dev_s *srcdev,
                     FAR struct coresight_dev_s *destdev)
{
  FAR struct coresight_node_s *node;
  irqstate_t flags;
  int ret;

  ret = coresight_validate_source(srcdev);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();

  if (list_is_empty(&srcdev->path))
    {
      ret = coresight_build_path(srcdev, destdev, &srcdev->path);
      if (ret < 0)
        {
          cserr("build path failed from %s ret: %d\n", srcdev->name, ret);
          goto err_path;
        }
    }

  ret = coresight_enable_path(&srcdev->path);
  if (ret < 0)
    {
      cserr("enable path failed from %s ret: %d\n", srcdev->name, ret);
      goto err_path;
    }

  ret = coresight_enable_source(srcdev);
  if (ret < 0)
    {
      cserr("enable source failed %s ret: %d\n", srcdev->name, ret);
      goto err_source;
    }

  csinfo("trace enabled success while devices are:");
  list_for_every_entry(&srcdev->path, node, struct coresight_node_s, link)
    {
      csinfo("-> %s", node->csdev->name);
    }

out:
  leave_critical_section(flags);
  return ret;

err_source:
  coresight_disable_path(&srcdev->path);

err_path:
  coresight_release_path(&srcdev->path);
  goto out;
}

/****************************************************************************
 * Name: coresight_disable
 *
 * Description:
 *   Disable the trace start from srcdev to destdev.
 *
 * Input Parameters:
 *   srcdev  - Source device that generates trace data.
 *
 ****************************************************************************/

void coresight_disable(FAR struct coresight_dev_s *srcdev)
{
  irqstate_t flags;

  flags = enter_critical_section();

  coresight_disable_source(srcdev);
  coresight_disable_path(&srcdev->path);
  coresight_release_path(&srcdev->path);

  leave_critical_section(flags);
}
