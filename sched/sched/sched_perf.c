/****************************************************************************
 * sched/sched/sched_perf.c
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

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdatomic.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/perf.h>
#include <nuttx/sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PERF_GET_COUNT(event) ((event)->count + (event)->child_count)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*perf_func_t)(FAR struct perf_event_s *event);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int perf_close(FAR struct file *filep);
static ssize_t perf_read(FAR struct file *filep, FAR char *buffer,
                         size_t len);
static int perf_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int perf_poll(FAR struct file *filep, FAR struct pollfd *fds,
                     bool setup);

static int perf_cpuclock_event_init(FAR struct perf_event_s *event);
static int perf_cpuclock_event_add(FAR struct perf_event_s *event,
                                   int flags);
static void perf_cpuclock_event_del(FAR struct perf_event_s *event,
                                    int flags);
static int perf_cpuclock_event_start(FAR struct perf_event_s *event,
                                     int flags);
static int perf_cpuclock_event_stop(FAR struct perf_event_s *event,
                                    int flags);
static int perf_cpuclock_event_read(FAR struct perf_event_s *event);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct perf_event_context_s g_perf_cpu_ctx[CONFIG_SMP_NCPUS];
static struct list_node g_perf_pmus = LIST_INITIAL_VALUE(g_perf_pmus);
static mutex_t g_perf_pmus_lock = NXMUTEX_INITIALIZER;
volatile static uint64_t g_perf_eventid;

static const struct file_operations g_perf_fops =
{
  .close = perf_close, /* close */
  .read  = perf_read,  /* read */
  .ioctl = perf_ioctl, /* ioctl */
  .poll  = perf_poll   /* poll */
};

static struct inode g_perf_inode =
{
  NULL,                   /* i_parent */
  NULL,                   /* i_peer */
  NULL,                   /* i_child */
  1,                      /* i_crefs */
  FSNODEFLAG_TYPE_DRIVER, /* i_flags */
  {
    &g_perf_fops          /* u */
  }
};

static struct pmu_ops_s g_perf_cpu_clock_ops =
{
  .event_init  = perf_cpuclock_event_init,
  .event_add   = perf_cpuclock_event_add,
  .event_del   = perf_cpuclock_event_del,
  .event_start = perf_cpuclock_event_start,
  .event_stop  = perf_cpuclock_event_stop,
  .event_read  = perf_cpuclock_event_read,
};

static struct pmu_s g_perf_cpu_clock =
{
  .ops = &g_perf_cpu_clock_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: perf_assign_eventid
 *
 * Description:
 *   Assign the perf event id
 *
 * Returned Value:
 *   Event id
 *
 ****************************************************************************/

static uint64_t perf_assign_eventid(void)
{
  return atomic_fetch_add(&g_perf_eventid, 1);
}

/****************************************************************************
 * Name: perf_get_eventid
 *
 * Description:
 *   Get the perf event id
 *
 * Input Parameters:
 *   event - Perf event
 *
 * Returned Value:
 *   Event id
 *
 ****************************************************************************/

static uint64_t perf_get_eventid(FAR struct perf_event_s *event)
{
  if (event->parent_event != NULL)
    {
      return event->parent_event->id;
    }
  else
    {
      return event->id;
    }
}

/****************************************************************************
 * Name: perf_pmu_enable
 *
 * Description:
 *   Enable pmu
 *
 * Input Parameters:
 *   pmu - PMU entry
 *
 ****************************************************************************/

static void perf_pmu_enable(FAR struct pmu_s *pmu)
{
  if (pmu->ops->pmu_enable)
    {
      pmu->ops->pmu_enable(pmu);
    }
}

/****************************************************************************
 * Name: perf_pmu_disable
 *
 * Description:
 *   Disable pmu
 *
 * Input Parameters:
 *   pmu - PMU entry
 *
 ****************************************************************************/

static void perf_pmu_disable(FAR struct pmu_s *pmu)
{
  if (pmu->ops->pmu_disable)
    {
      pmu->ops->pmu_disable(pmu);
    }
}

/****************************************************************************
 * Name: perf_get_pmu
 *
 * Description:
 *   Find the suitable pmu from the event
 *
 * Input Parameters:
 *   event - Perf event
 *
 * Returned Value:
 *   Perf pmu entry
 *
 ****************************************************************************/

static FAR struct pmu_s *perf_get_pmu(FAR struct perf_event_s *event)
{
  FAR struct pmu_s *pmu;

  if (event->parent_event && event->parent_event->pmu)
    {
      pmu = event->parent_event->pmu;
      event->pmu = pmu;
      if (pmu->ops->event_init(event) == 0)
        {
          return pmu;
        }
    }

  list_for_every_entry(&g_perf_pmus, pmu, struct pmu_s, node)
    {
      event->pmu = pmu;
      if (pmu->ops->event_init(event) == 0)
        {
          return pmu;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: perf_init_context
 *
 * Description:
 *   Perf event context initialization
 *
 * Input Parameters:
 *   ctx - Perf event context for init
 *
 * Returned Value:
 *   Initialization result
 *
 ****************************************************************************/

static void perf_init_context(FAR struct perf_event_context_s *ctx)
{
  spin_initialize(&ctx->lock, SP_UNLOCKED);
  list_initialize(&ctx->event_list);
  list_initialize(&ctx->group_list);
  list_initialize(&ctx->pmu_ctx_list);
}

/****************************************************************************
 * Name: perf_get_context
 *
 * Description:
 *   Find the suitable perf context for the event
 *
 * Input Parameters:
 *   event - Perf event
 *   tcb   - tcb the event bind
 *
 * Returned Value:
 *   Perf context entry
 *
 ****************************************************************************/

static FAR struct perf_event_context_s *
perf_get_context(FAR struct perf_event_s *event,
                 FAR struct tcb_s *tcb)
{
  FAR struct perf_event_context_s *ctx = NULL;

  /* No tcb is specified bound to cpu, else bound to task */

  if (tcb == NULL)
    {
      ctx = &g_perf_cpu_ctx[event->cpu];
      return ctx;
    }

  nxmutex_lock(&tcb->perf_event_mutex);

  ctx = tcb->perf_event_ctx;
  if (ctx == NULL)
    {
      ctx = kmm_zalloc(sizeof(struct perf_event_context_s));
      if (ctx == NULL)
        {
          serr("task perf event alloc fail\n");
          return NULL;
        }

      perf_init_context(ctx);
      tcb->perf_event_ctx = ctx;
      ctx->tcb = tcb;
    }

  nxmutex_unlock(&tcb->perf_event_mutex);
  return ctx;
}

/****************************************************************************
 * Name: perf_free_context
 *
 * Description:
 *   Free the perf event context for the event
 *
 * Input Parameters:
 *   ctx - perf event context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_free_context(FAR struct perf_event_context_s *ctx)
{
  FAR struct tcb_s *tcb = ctx->tcb;

  if (tcb == NULL)
    {
      return;
    }

  nxmutex_lock(&tcb->perf_event_mutex);

  if (list_is_empty(&ctx->event_list))
    {
      tcb->perf_event_ctx = NULL;
      ASSERT(ctx->active_num == 0);
      ASSERT(list_is_empty(&ctx->event_list));
      ASSERT(list_is_empty(&ctx->group_list));
      ASSERT(list_is_empty(&ctx->pmu_ctx_list));
      kmm_free(ctx);
    }

  nxmutex_unlock(&tcb->perf_event_mutex);
}

/****************************************************************************
 * Name: perf_init_pmu_context
 *
 * Description:
 *   Init pmu context for the event
 *
 * Input Parameters:
 *   pmu_ctx - Pmu context
 *   pmu     - Pmu entry
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void perf_init_pmu_context(FAR struct pmu_event_context_s *pmu_ctx,
                           FAR struct pmu_s *pmu)
{
  pmu_ctx->pmu = pmu;
  list_clear_node(&pmu_ctx->pmu_ctx_node);
  pmu_ctx->refcount = 0;
}

/****************************************************************************
 * Name: perf_get_pmu_context
 *
 * Description:
 *   Find the suitable pmu context for the event
 *
 * Input Parameters:
 *   pmu   - Pmu entry
 *   event - Perf event
 *   tcb   - Tcb the event bind
 *
 * Returned Value:
 *   Pmu context entry
 *
 ****************************************************************************/

static FAR struct pmu_event_context_s *
perf_get_pmu_context(FAR struct pmu_s *pmu, FAR struct perf_event_s *event,
                     FAR struct perf_event_context_s *ctx)
{
  FAR struct pmu_event_context_s *pmu_ctx;
  irqstate_t flags;

  /* This event is bound to cpu, get pmu cpu context */

  if (ctx->tcb == NULL)
    {
      pmu_ctx = &pmu->cpu_pmu_ctx[event->cpu].pmuctx;

      flags = spin_lock_irqsave(&ctx->lock);

      if (pmu_ctx->ctx == NULL)
        {
          pmu_ctx->ctx = ctx;
          list_add_tail(&ctx->pmu_ctx_list, &pmu_ctx->pmu_ctx_node);
        }

      pmu_ctx->refcount++;
      goto out;
    }

  /* This event is bound to task, found the task ctx or malloc new one */

  flags = spin_lock_irqsave(&ctx->lock);

  list_for_every_entry(&ctx->pmu_ctx_list, pmu_ctx,
                       struct pmu_event_context_s, pmu_ctx_node)
    {
      if (pmu_ctx->pmu == pmu)
        {
          pmu_ctx->refcount++;
          goto out;
        }
    }

  spin_unlock_irqrestore(&ctx->lock, flags);

  pmu_ctx = kmm_malloc(sizeof(struct pmu_event_context_s));
  if (pmu_ctx == NULL)
    {
      serr("malloc pmu_event_context_s fail\n");
      return NULL;
    }

  perf_init_pmu_context(pmu_ctx, pmu);

  flags = spin_lock_irqsave(&ctx->lock);

  pmu_ctx->ctx = ctx;
  pmu_ctx->refcount++;
  list_add_tail(&ctx->pmu_ctx_list, &pmu_ctx->pmu_ctx_node);

out:
  spin_unlock_irqrestore(&ctx->lock, flags);
  return pmu_ctx;
}

/****************************************************************************
 * Name: perf_free_pmu_context
 *
 * Description:
 *   Free the pmu context for the event
 *
 * Input Parameters:
 *   pmu_ctx - PMU context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_free_pmu_context(FAR struct pmu_event_context_s *pmu_ctx)
{
  bool free_flag = false;

  spin_lock(&pmu_ctx->ctx->lock);

  pmu_ctx->refcount--;

  if (pmu_ctx->refcount == 0)
    {
      list_delete(&pmu_ctx->pmu_ctx_node);

      if (pmu_ctx->ctx->tcb != NULL)
        {
          free_flag = true;
        }
    }

  spin_unlock(&pmu_ctx->ctx->lock);

  if (pmu_ctx->refcount == 0)
    {
      pmu_ctx->ctx = NULL;
    }

  if (free_flag)
    {
      kmm_free(pmu_ctx);
    }
}

/****************************************************************************
 * Name: perf_event_alloc
 *
 * Description:
 *   Create perf event
 *
 * Input Parameters:
 *   attr         - Perf event attribute
 *   cpu          - Cpu id perf event bind
 *   tcb          - Task perf event bind
 *   group_leader - Group leader
 *   parent_event - Parent event
 *
 * Returned Value:
 *   Perf event
 *
 ****************************************************************************/

static FAR struct perf_event_s *
perf_event_alloc(FAR struct perf_event_attr_s *attr,
                 int cpu, FAR struct tcb_s *tcb,
                 FAR struct perf_event_s *group_leader,
                 FAR struct perf_event_s *parent_event)
{
  FAR struct perf_event_s *event;
  FAR struct pmu_s *pmu;

  event = kmm_zalloc(sizeof(struct perf_event_s));
  if (event == NULL)
    {
      serr("perf event alloc fail\n");
      return NULL;
    }

  event->attr  = *attr;
  event->cpu   = cpu;
  event->oncpu = -1;
  event->parent_event = parent_event;

  if (group_leader == NULL)
    {
      event->group_leader = event;
    }
  else
    {
      event->group_leader = group_leader;
    }

  list_initialize(&event->sibling_list);
  list_initialize(&event->child_list);
  nxmutex_init(&event->child_mutex);

  /* Find the suitable pmu */

  pmu = perf_get_pmu(event);
  if (pmu == NULL)
    {
      serr("perf event get pmu fail\n");
      goto nxmutex_destroy;
    }

  event->id = perf_assign_eventid();
  return event;

nxmutex_destroy:
  kmm_free(event);
  return NULL;
}

/****************************************************************************
 * Name: perf_free_event
 *
 * Description:
 *   Free perf event
 *
 * Input Parameters:
 *   event - Free perf event
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_free_event(FAR struct perf_event_s *event)
{
  if (event->pmuctx != NULL)
    {
      perf_free_pmu_context(event->pmuctx);
    }

  if (event->ctx != NULL)
    {
      perf_free_context(event->ctx);
    }

  nxmutex_destroy(&event->child_mutex);
  kmm_free(event);
}

/****************************************************************************
 * Name: perf_add_event_to_count
 *
 * Description:
 *   Add perf event to context
 *
 * Input Parameters:
 *   event - Perf event
 *   ctx   - Perf context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_add_event_to_count(FAR struct perf_event_s *event,
                                  FAR struct perf_event_context_s *ctx)
{
  irqstate_t flags = spin_lock_irqsave(&ctx->lock);

  /* Add perf event to group if it has group_leader */

  if (event->group_leader != event)
    {
      list_add_tail(&event->group_leader->sibling_list,
                    &event->sibling_list);
      event->group_leader->sibling_num++;
    }
  else
    {
      list_add_tail(&ctx->group_list, &event->group_node);
    }

  /* Add perf event to event list */

  list_add_tail(&ctx->event_list, &event->event_node);

  spin_unlock_irqrestore(&ctx->lock, flags);
}

/****************************************************************************
 * Name: perf_inherit_event
 *
 * Description:
 *   Inherit one perf event
 *
 * Input Parameters:
 *   parent_event - Inherit parent event
 *   group_leader - Group leader for this event
 *   parent       - Parent tcb for this event
 *   child        - Current tcb for this event
 *
 * Returned Value:
 *   Perf event
 *
 ****************************************************************************/

static FAR struct perf_event_s *
perf_inherit_event(FAR struct perf_event_s *parent_event,
                   FAR struct perf_event_s *group_leader,
                   FAR struct tcb_s *parent,
                   FAR struct tcb_s *child)
{
  FAR struct perf_event_s *event;

  event = perf_event_alloc(&parent_event->attr, parent_event->cpu,
                           child, group_leader, parent_event);
  if (event == NULL)
    {
      serr("perf_inherit_event alloc perf event fail\n");
      return NULL;
    }

  nxmutex_lock(&parent_event->child_mutex);

  if (parent_event->state >= PERF_EVENT_STATE_INACTIVE)
    {
      event->state = PERF_EVENT_STATE_INACTIVE;
    }
  else
    {
      event->state = PERF_EVENT_STATE_OFF;
    }

  list_add_tail(&parent_event->child_list, &event->child_list);
  parent_event->child_num++;
  event->attach_state |= PERF_ATTACH_CHILD;
  event->ctx = child->perf_event_ctx;

  nxmutex_unlock(&parent_event->child_mutex);

  perf_add_event_to_count(event, event->ctx);

  return event;
}

/****************************************************************************
 * Name: perf_inherit_group_events
 *
 * Description:
 *   Inherit perf event from parent group
 *
 * Input Parameters:
 *   group_leader  - Perf event group leader
 *   parent        - Parent tcb for this event
 *   child         - Current tcb for this event
 *
 * Returned Value:
 *   Inherit result
 *
 ****************************************************************************/

static int perf_inherit_group_events(FAR struct perf_event_s *group_leader,
                                     FAR struct tcb_s *parent,
                                     FAR struct tcb_s *child)
{
  FAR struct perf_event_s *sub_event;
  FAR struct perf_event_s *child_leader;

  if (group_leader->attr.inherit == 0)
    {
      return OK;
    }

  if (child->perf_event_ctx == NULL)
    {
      child->perf_event_ctx =
        kmm_zalloc(sizeof(struct perf_event_context_s));
      if (child->perf_event_ctx == NULL)
        {
          serr("malloc task perf event fail\n");
          return -ENOMEM;
        }

      perf_init_context(child->perf_event_ctx);
      child->perf_event_ctx->tcb = child;
    }

  child_leader = perf_inherit_event(group_leader, NULL, parent, child);
  if (child_leader == NULL)
    {
      return -EINVAL;
    }

  list_for_every_entry(&group_leader->sibling_list, sub_event,
                       struct perf_event_s, group_node)
    {
      FAR struct perf_event_s *child_event;
      child_event = perf_inherit_event(sub_event, child_leader,
                                       parent, child);
      if (child_event == NULL)
        {
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: perf_event_sched_in
 *
 * Description:
 *   Perf event sched in
 *
 * Input Parameters:
 *   event - Perf event
 *
 * Returned Value:
 *   Sched result
 *
 ****************************************************************************/

static int perf_event_sched_in(FAR struct perf_event_s *event)
{
  FAR struct perf_event_context_s *ctx = event->ctx;
  FAR struct pmu_s *pmu = event->pmu;
  int ret;

  if (event->state == PERF_EVENT_STATE_OFF ||
      event->state == PERF_EVENT_STATE_ACTIVE)
    {
      return -EPERM;
    }

  if (ctx->tcb != NULL && ctx->tcb != this_task())
    {
      return -EAGAIN;
    }

#ifdef CONFIG_SMP
  event->oncpu = this_cpu();
#endif
  event->state = PERF_EVENT_STATE_ACTIVE;

  perf_pmu_disable(pmu);

  ret = pmu->ops->event_add(event, 0);
  if (ret < 0)
    {
      serr("pmu event add fail\n");
      goto out;
    }

  ret = pmu->ops->event_start(event, 0);
  if (ret < 0)
    {
      pmu->ops->event_del(event, 0);
      goto out;
    }

  ctx->active_num++;

out:
  perf_pmu_enable(pmu);
  return ret;
}

/****************************************************************************
 * Name: perf_event_sched_out
 *
 * Description:
 *   Perf event sched out
 *
 * Input Parameters:
 *   event - Perf event
 *
 * Returned Value:
 *   Sched result
 *
 ****************************************************************************/

static int perf_event_sched_out(FAR struct perf_event_s *event)
{
  FAR struct perf_event_context_s *ctx = event->ctx;
  FAR struct pmu_s *pmu = event->pmu;
  int ret;

#ifdef CONFIG_SMP
  if (event->oncpu != this_cpu())
    {
      return -EAGAIN;
    }
#endif

  if (event->state != PERF_EVENT_STATE_ACTIVE)
    {
      return -EPERM;
    }

  perf_pmu_disable(pmu);

  ret = pmu->ops->event_stop(event, PERF_EF_UPDATE);
  if (ret < 0)
    {
      serr("pmu event stop fail\n");
      goto out;
    }

  pmu->ops->event_del(event, 0);

#ifdef CONFIG_SMP
  event->oncpu = -1;
#endif
  event->state = PERF_EVENT_STATE_INACTIVE;
  ctx->active_num--;

out:
  perf_pmu_enable(pmu);
  return ret;
}

/****************************************************************************
 * Name: perf_group_sched_in
 *
 * Description:
 *   Perf group event sched in
 *
 * Input Parameters:
 *   group_leader  - Group leader event
 *   ctx           - Perf event context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_group_sched_in(FAR struct perf_event_s *group_leader,
                                FAR struct perf_event_context_s *ctx)
{
  FAR struct perf_event_s *sub_event;

  /* Sched in group leader first */

  perf_event_sched_in(group_leader);

  /* Sched out event in this group */

  list_for_every_entry(&group_leader->sibling_list, sub_event,
                       struct perf_event_s, group_node)
    {
      perf_event_sched_in(sub_event);
    }
}

/****************************************************************************
 * Name: perf_group_sched_out
 *
 * Description:
 *   Perf group event sched out
 *
 * Input Parameters:
 *   group_leader  - Group leader event
 *   ctx           - Perf event context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_group_sched_out(FAR struct perf_event_s *group_leader,
                                 FAR struct perf_event_context_s *ctx)
{
  FAR struct perf_event_s *sub_event;

  /* Sched out group leader first */

  perf_event_sched_out(group_leader);

  /* Sched out event in this group */

  list_for_every_entry(&group_leader->sibling_list, sub_event,
                       struct perf_event_s, group_node)
    {
      perf_event_sched_out(sub_event);
    }
}

/****************************************************************************
 * Name: perf_context_enable
 *
 * Description:
 *   Perf context enable
 *
 * Input Parameters:
 *   ctx  - Perf event context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_context_enable(FAR struct perf_event_context_s *ctx)
{
  FAR struct pmu_event_context_s *pmu_ctx;

  list_for_every_entry(&ctx->pmu_ctx_list, pmu_ctx,
                       struct pmu_event_context_s, pmu_ctx_node)
    {
      perf_pmu_enable(pmu_ctx->pmu);
    }
}

/****************************************************************************
 * Name: perf_context_disable
 *
 * Description:
 *   Perf context disable
 *
 * Input Parameters:
 *   ctx  - Perf event context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_context_disable(FAR struct perf_event_context_s *ctx)
{
  FAR struct pmu_event_context_s *pmu_ctx;

  list_for_every_entry(&ctx->pmu_ctx_list, pmu_ctx,
                       struct pmu_event_context_s, pmu_ctx_node)
    {
      perf_pmu_disable(pmu_ctx->pmu);
    }
}

/****************************************************************************
 * Name: perf_context_sched_in
 *
 * Description:
 *   Perf context event sched in
 *
 * Input Parameters:
 *   ctx  - Perf event context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_context_sched_in(FAR struct perf_event_context_s *ctx)
{
  FAR struct perf_event_s *group_leader;

  /* Sched all the perf event in the context */

  list_for_every_entry(&ctx->group_list, group_leader,
                       struct perf_event_s, group_node)
    {
      perf_group_sched_in(group_leader, ctx);
    }
}

/****************************************************************************
 * Name: perf_context_sched_out
 *
 * Description:
 *   Perf context event sched out
 *
 * Input Parameters:
 *   ctx  - Perf event context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_context_sched_out(FAR struct perf_event_context_s *ctx)
{
  FAR struct perf_event_s *group_leader;

  if (ctx->active_num == 0)
    {
      return;
    }

  /* Stop all the perf event in the context */

  list_for_every_entry(&ctx->group_list, group_leader,
                       struct perf_event_s, group_node)
    {
      perf_group_sched_out(group_leader, ctx);
    }
}

/****************************************************************************
 * Name: perf_event_do_enable
 *
 * Description:
 *   Perf event enable internal
 *
 * Input Parameters:
 *   arg - Perf event
 *
 * Returned Value:
 *   Enable result
 *
 ****************************************************************************/

static int perf_event_do_enable(FAR void *arg)
{
  FAR struct perf_event_s *event = arg;
  FAR struct perf_event_context_s *ctx = event->ctx;

  if (event->state >= PERF_EVENT_STATE_INACTIVE)
    {
      return OK;
    }

  perf_context_disable(ctx);

  if (ctx->active_num != 0)
    {
      perf_context_sched_out(ctx);
    }

  event->state = PERF_EVENT_STATE_INACTIVE;

  perf_context_sched_in(ctx);

  perf_context_enable(ctx);

  return OK;
}

/****************************************************************************
 * Name: perf_event_do_disable
 *
 * Description:
 *   Perf event disable internal
 *
 * Input Parameters:
 *   arg - Perf event
 *
 * Returned Value:
 *   Disable result
 *
 ****************************************************************************/

static int perf_event_do_disable(FAR void *arg)
{
  FAR struct perf_event_s *event = arg;
  FAR struct perf_event_context_s *ctx = event->ctx;

  if (event->state == PERF_EVENT_STATE_OFF)
    {
      return OK;
    }

  if (event->group_leader == event)
    {
      perf_group_sched_out(event, ctx);
    }
  else
    {
      perf_event_sched_out(event);
    }

  event->state = PERF_EVENT_STATE_OFF;

  return OK;
}

/****************************************************************************
 * Name: perf_task_function_call
 *
 * Description:
 *   Perf event call function at designated task
 *
 * Input Parameters:
 *   tcb   - Tcb for this event
 *   event - Perf event
 *   func  - Executive function
 *
 * Returned Value:
 *   Execution result
 *
 ****************************************************************************/

static int perf_task_function_call(FAR struct tcb_s *tcb,
                                   FAR struct perf_event_s *event,
                                   nxsched_smp_call_t func)
{
#ifdef CONFIG_SMP
  if (tcb->cpu != this_cpu())
    {
      return nxsched_smp_call_single(tcb->cpu, func, event, true);
    }
#endif

  /* If task running on the current CPU, run func directly */

  return func(event);
}

/****************************************************************************
 * Name: perf_cpu_function_call
 *
 * Description:
 *   Perf event call function at designated cpu
 *
 * Input Parameters:
 *   event - Perf event
 *   func  - Executive function
 *
 * Returned Value:
 *   Execution result
 *
 ****************************************************************************/

static int perf_cpu_function_call(int cpu, FAR struct perf_event_s *event,
                                  nxsched_smp_call_t func)
{
#ifdef CONFIG_SMP
  if (cpu != this_cpu())
    {
      return nxsched_smp_call_single(cpu, func, event, true);
    }
#endif

  /* If on the current CPU, run func directly */

  return func(event);
}

/****************************************************************************
 * Name: perf_function_call
 *
 * Description:
 *   Perf event call function
 *
 * Input Parameters:
 *   event - Perf event
 *   func  - Executive function
 *
 * Returned Value:
 *   Execution result
 *
 ****************************************************************************/

static int perf_function_call(FAR struct perf_event_s *event,
                              nxsched_smp_call_t func)
{
  FAR struct perf_event_context_s *ctx = event->ctx;

  /* Enable event immediately if event bound in cpu */

  if (ctx->tcb == NULL)
    {
      return perf_cpu_function_call(event->cpu, event, func);
    }
  else
    {
      return perf_task_function_call(ctx->tcb, event, func);
    }
}

/****************************************************************************
 * Name: perf_event_enable
 *
 * Description:
 *   Perf event enable
 *
 * Input Parameters:
 *   arg - Perf event
 *
 * Returned Value:
 *   Execution result
 *
 ****************************************************************************/

static int perf_event_enable(FAR void *arg)
{
  FAR struct perf_event_s *event = arg;

  if (event->state >= PERF_EVENT_STATE_INACTIVE)
    {
      serr("Perf event was already enable\n");
      return OK;
    }

  return perf_function_call(event, perf_event_do_enable);
}

/****************************************************************************
 * Name: perf_event_disable
 *
 * Description:
 *   Perf event disable
 *
 * Input Parameters:
 *   arg - Perf event
 *
 * Returned Value:
 *   Disable result
 *
 ****************************************************************************/

static int perf_event_disable(FAR void *arg)
{
  FAR struct perf_event_s *event = arg;

  if (event->state != PERF_EVENT_STATE_ACTIVE)
    {
      serr("Perf event not active\n");
      return OK;
    }

  return perf_function_call(event, perf_event_do_disable);
}

/****************************************************************************
 * Name: perf_event_reset
 *
 * Description:
 *   Perf event reset
 *
 * Input Parameters:
 *   arg - Perf event
 *
 * Returned Value:
 *   Reset result
 *
 ****************************************************************************/

static int perf_event_reset(FAR void *arg)
{
  FAR struct perf_event_s *event = arg;

  event->count = 0;
  return 0;
}

/****************************************************************************
 * Name: perf_event_for_child
 *
 * Description:
 *   Handle func for each event for the same group leader
 *
 * Input Parameters:
 *   event - Perf event
 *   func  - Executive function
 *
 * Returned Value:
 *   Execution result
 *
 ****************************************************************************/

static int perf_event_for_child(FAR struct perf_event_s *event,
                                nxsched_smp_call_t func)
{
  FAR struct perf_event_s *child_event;
  int ret;

  ret = func(event);
  if (ret < 0)
    {
      return ret;
    }

  nxmutex_lock(&event->child_mutex);

  list_for_every_entry(&event->child_list, child_event,
                       struct perf_event_s, child_list)
    {
      ret = func(child_event);
      if (ret < 0)
        {
          break;
        }
    }

  nxmutex_unlock(&event->child_mutex);
  return ret;
}

/****************************************************************************
 * Name: perf_event_for_group
 *
 * Description:
 *   Handle func for each event in the group
 *
 * Input Parameters:
 *   event - Perf event
 *   func  - Executive function
 *
 * Returned Value:
 *   Execution result
 *
 ****************************************************************************/

static int perf_event_for_group(FAR struct perf_event_s *event,
                                nxsched_smp_call_t func)
{
  FAR struct perf_event_s *sub_event;
  int ret;

  event = event->group_leader;
  ret = perf_event_for_child(event, func);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&event->sibling_list, sub_event,
                       struct perf_event_s, group_node)
    {
      ret = perf_event_for_child(sub_event, func);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: perf_get_event_count
 *
 * Description:
 *   Perf event read count from lowerhalf
 *
 * Input Parameters:
 *   arg - Perf event
 *
 * Returned Value:
 *   Count result
 *
 ****************************************************************************/

static int perf_get_event_count(FAR void *arg)
{
  FAR struct perf_event_s *event = arg;
  FAR struct perf_event_s *sub_event;
  int ret;

#ifdef CONFIG_SMP
  if (event->oncpu != this_cpu())
    {
      return -EAGAIN;
    }
#endif

  if (event->state != PERF_EVENT_STATE_ACTIVE)
    {
      return OK;
    }

  ret = event->pmu->ops->event_read(event);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&event->sibling_list, sub_event,
                       struct perf_event_s, group_node)
    {
      ret = sub_event->pmu->ops->event_read(sub_event);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: perf_read_event_count
 *
 * Description:
 *   Perf event read event count
 *
 * Input Parameters:
 *   event       - Perf event
 *
 * Returned Value:
 *   Count result
 *
 ****************************************************************************/

static uint64_t perf_read_event_count(FAR struct perf_event_s *event)
{
  FAR struct perf_event_s *child_event;
  uint64_t count;

  perf_function_call(event, perf_get_event_count);
  count = PERF_GET_COUNT(event);

  nxmutex_lock(&event->child_mutex);

  list_for_every_entry(&event->child_list, child_event,
                       struct perf_event_s, child_list)
    {
      perf_function_call(child_event, perf_get_event_count);
      count += child_event->count;
    }

  nxmutex_unlock(&event->child_mutex);
  return count;
}

/****************************************************************************
 * Name: perf_read_one
 *
 * Description:
 *   Perf event read result from one event
 *
 * Input Parameters:
 *   event       - Perf event
 *   buffer      - Read buffer
 *   read_format - Read format for result data
 *
 * Returned Value:
 *   Read result
 *
 ****************************************************************************/

static ssize_t perf_read_one(FAR struct perf_event_s *event,
                             FAR char *buffer, uint64_t read_format)
{
  FAR uint64_t *data = (FAR uint64_t *)buffer;
  ssize_t n = 0;

  data[n++] = perf_read_event_count(event);
  if (read_format & PERF_FORMAT_TOTAL_TIME_ENABLED)
    {
      /* TODO */

      data[n++] = 0;
    }

  if (read_format & PERF_FORMAT_TOTAL_TIME_RUNNING)
    {
      /* TODO */

      data[n++] = 0;
    }

  if (read_format & PERF_FORMAT_ID)
    {
      data[n++] = perf_get_eventid(event);
    }

  if (read_format & PERF_FORMAT_LOST)
    {
      /* TODO */

      data[n++] = 0;
    }

  return n * sizeof(uint64_t);
}

/****************************************************************************
 * Name: perf_read_group_one
 *
 * Description:
 *   Perf event read result from group
 *
 * Input Parameters:
 *   event       - Perf event
 *   buffer      - Read buffer
 *   read_format - Read format for result data
 *
 * Returned Value:
 *   Read result
 *
 ****************************************************************************/

static ssize_t perf_read_group_one(FAR struct perf_event_s *event,
                                   FAR uint64_t *data,
                                   uint64_t read_format)
{
  FAR struct perf_event_s *sub_event;
  ssize_t n = 0;

  if (read_format & PERF_FORMAT_TOTAL_TIME_ENABLED)
    {
      /* TODO */

      data[n++] += 0;
    }

  if (read_format & PERF_FORMAT_TOTAL_TIME_RUNNING)
    {
      /* TODO */

      data[n++] += 0;
    }

  data[n++] = perf_read_event_count(event);
  if (read_format & PERF_FORMAT_ID)
    {
      data[n++] = perf_get_eventid(event);
    }

  if (read_format & PERF_FORMAT_LOST)
    {
      /* TODO */

      data[n++] = 0;
    }

  list_for_every_entry(&event->sibling_list, sub_event,
                       struct perf_event_s, group_node)
    {
      data[n++] += perf_read_event_count(sub_event);
      if (read_format & PERF_FORMAT_ID)
        {
          data[n++] = perf_get_eventid(sub_event);
        }

      if (read_format & PERF_FORMAT_LOST)
        {
          /* TODO */

          data[n++] = 0;
        }
    }

  return n * sizeof(uint64_t);
}

/****************************************************************************
 * Name: perf_read_group
 *
 * Description:
 *   Perf event read result from group
 *
 * Input Parameters:
 *   event       - Perf event
 *   buffer      - Read buffer
 *   read_format - Read format for result data
 *
 * Returned Value:
 *   Read result
 *
 ****************************************************************************/

static ssize_t perf_read_group(FAR struct perf_event_s *event,
                               FAR char *buffer, uint64_t read_format)
{
  FAR struct perf_event_s *group_leader = event->group_leader;
  FAR struct perf_event_s *child_event;
  FAR uint64_t *data = (FAR uint64_t *)buffer;
  ssize_t ret;

  data[0] = 1 + group_leader->sibling_num;

  ret = perf_read_group_one(group_leader, data, read_format);

  nxmutex_lock(&group_leader->child_mutex);

  list_for_every_entry(&event->child_list, child_event,
                       struct perf_event_s, child_list)
    {
      ret = perf_read_group_one(child_event, data, read_format);
    }

  nxmutex_unlock(&group_leader->child_mutex);
  return ret;
}

/****************************************************************************
 * Name: perf_group_detach
 *
 * Description:
 *   Remove perf event from group
 *
 * Input Parameters:
 *   event  - Perf event
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_group_detach(FAR struct perf_event_s *event)
{
  FAR struct perf_event_s *sub_event;

  /* If is not the group leader, remove it */

  if (event->group_leader != event)
    {
      list_delete(&event->sibling_list);
      event->group_leader->sibling_num--;
      return;
    }

  /* If is group leader, upgrade the sibling node to singleton event */

  list_for_every_entry(&event->sibling_list, sub_event,
                       struct perf_event_s, group_node)
    {
      sub_event->group_leader = sub_event;
      list_delete(&sub_event->sibling_list);
    }

  list_delete(&event->group_node);
}

/****************************************************************************
 * Name: perf_child_detach
 *
 * Description:
 *   Remove child events
 *
 * Input Parameters:
 *   event  - Perf event
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_child_detach(FAR struct perf_event_s *event)
{
  FAR struct perf_event_s *parent_event = event->parent_event;
  uint64_t child_count;

  if (!(event->attach_state & PERF_ATTACH_CHILD))
    {
      return;
    }

  event->attach_state &= ~PERF_ATTACH_CHILD;

  nxmutex_lock(&parent_event->child_mutex);

  child_count = PERF_GET_COUNT(event);
  parent_event->child_count += child_count;
  parent_event->child_num--;

  nxmutex_unlock(&parent_event->child_mutex);

  list_delete(&event->child_list);
}

/****************************************************************************
 * Name: perf_context_detach_internal
 *
 * Description:
 *   Remove perf event from context
 *
 * Input Parameters:
 *   arg - Perf event
 *
 * Returned Value:
 *   Detach result
 *
 ****************************************************************************/

static int perf_context_detach_internal(FAR void *arg)
{
  FAR struct perf_event_s *event = arg;

  perf_event_sched_out(event);

  perf_group_detach(event);

  perf_child_detach(event);

  list_delete(&event->event_node);

  return OK;
}

/****************************************************************************
 * Name: perf_context_detach
 *
 * Description:
 *   Remove perf event from context
 *
 * Input Parameters:
 *   event  - Perf event
 *   ctx    - Perf event context
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_context_detach(FAR struct perf_event_s *event,
                                FAR struct perf_event_context_s *ctx)
{
  irqstate_t flags = spin_lock_irqsave(&ctx->lock);

  /* If no event active, call remove directly */

  if (ctx->active_num == 0)
    {
      perf_context_detach_internal(event);
      spin_unlock_irqrestore(&ctx->lock, flags);
      return;
    }

  spin_unlock_irqrestore(&ctx->lock, flags);

  perf_function_call(event, perf_context_detach_internal);
}

/****************************************************************************
 * Name: perf_setup_task_context
 *
 * Description:
 *   Perf event task context initialization
 *
 * Input Parameters:
 *   tcb   - Task for init.
 *
 * Returned Value:
 *   Initialization result
 *
 ****************************************************************************/

static int perf_setup_task_context(FAR struct tcb_s *tcb)
{
  FAR struct tcb_s *parent = this_task();
  FAR struct perf_event_context_s *parent_ctx = parent->perf_event_ctx;
  FAR struct perf_event_s *group_leader;
  irqstate_t flags;
  int ret;

  /* Inherit parent event if it has */

  if (parent_ctx == NULL)
    {
      return OK;
    }

  flags = spin_lock_irqsave(&parent_ctx->lock);

  list_for_every_entry(&parent_ctx->group_list, group_leader,
                       struct perf_event_s, group_node)
    {
      ret = perf_inherit_group_events(group_leader, parent, tcb);
      if (ret < 0)
        {
          break;
        }
    }

  spin_unlock_irqrestore(&parent_ctx->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: perf_close
 *
 * Description:
 *   Perf event close
 *
 * Input Parameters:
 *   filep - File point
 *
 * Returned Value:
 *   Close result
 *
 ****************************************************************************/

static int perf_close(FAR struct file *filep)
{
  FAR struct perf_event_s *event = filep->f_priv;
  FAR struct perf_event_context_s *ctx;

  ASSERT(event != NULL);

  ctx = event->ctx;
  if (ctx != NULL)
    {
      perf_context_detach(event, ctx);
    }

  perf_free_event(event);
  return OK;
}

/****************************************************************************
 * Name: perf_read
 *
 * Description:
 *   Perf event read
 *
 * Input Parameters:
 *   filep  - File point
 *   buffer - Read buffer
 *   len    - Read data len
 *
 * Returned Value:
 *   Read data len
 *
 ****************************************************************************/

static ssize_t perf_read(FAR struct file *filep, FAR char *buffer,
                         size_t len)
{
  FAR struct perf_event_s *event = filep->f_priv;
  uint64_t read_format;
  ssize_t ret;

  ASSERT(event != NULL);

  read_format = event->attr.read_format;

  if (read_format & PERF_FORMAT_GROUP)
    {
      ret = perf_read_group(event, buffer, read_format);
    }
  else
    {
      ret = perf_read_one(event, buffer, read_format);
    }

  return ret;
}

/****************************************************************************
 * Name: perf_ioctl
 *
 * Description:
 *   Perf event ioctl
 *
 * Input Parameters:
 *   filep  - File point
 *   cmd    - Command id
 *   arg    - Command arg
 *
 * Returned Value:
 *   Ioctl result
 *
 ****************************************************************************/

static int perf_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct perf_event_s *event = filep->f_priv;
  FAR struct perf_event_context_s *ctx;
  nxsched_smp_call_t func = NULL;
  irqstate_t flags;
  int ret = OK;

  ASSERT(event != NULL);

  switch (cmd)
    {
      case PERF_EVENT_IOC_ENABLE:
        func = perf_event_enable;
        break;
      case PERF_EVENT_IOC_DISABLE:
        func = perf_event_disable;
        break;
      case PERF_EVENT_IOC_RESET:
        func = perf_event_reset;
        break;
      default:
        return -EINVAL;
    }

  ctx = event->ctx;

  flags = spin_lock_irqsave(&ctx->lock);

  if (arg & PERF_IOC_FLAG_GROUP)
    {
      ret = perf_event_for_group(event, func);
    }
  else
    {
      ret = perf_event_for_child(event, func);
    }

  spin_unlock_irqrestore(&ctx->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: perf_poll
 *
 * Description:
 *   Perf event poll
 *
 * Input Parameters:
 *   filep  - File point
 *   fds    - Poll fds
 *   setup  - Setup or close
 *
 * Returned Value:
 *   Poll result
 *
 ****************************************************************************/

static int perf_poll(FAR struct file *filep, FAR struct pollfd *fds,
                     bool setup)
{
  return 0;
}

/****************************************************************************
 * Name: perf_cpuclock_event_init
 *
 * Description:
 *   CPU clock event init
 *
 * Input Parameters:
 *   event  - Perf event
 *
 * Returned Value:
 *   Result
 *
 ****************************************************************************/

static int perf_cpuclock_event_init(FAR struct perf_event_s *event)
{
  return -1;
}

/****************************************************************************
 * Name: perf_cpuclock_event_add
 *
 * Description:
 *   Add cpu clock event
 *
 * Input Parameters:
 *   event  - Perf event
 *
 * Returned Value:
 *   Result
 *
 ****************************************************************************/

static int perf_cpuclock_event_add(FAR struct perf_event_s *event,
                                   int flags)
{
  return 0;
}

/****************************************************************************
 * Name: perf_cpuclock_event_del
 *
 * Description:
 *   Del cpu clock event
 *
 * Input Parameters:
 *   event  - Perf event
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void perf_cpuclock_event_del(FAR struct perf_event_s *event,
                                    int flags)
{
}

/****************************************************************************
 * Name: perf_cpuclock_event_start
 *
 * Description:
 *   Start cpu clock event
 *
 * Input Parameters:
 *   event  - Perf event
 *   flags  - Perf event flags
 *
 * Returned Value:
 *   Result
 *
 ****************************************************************************/

static int perf_cpuclock_event_start(FAR struct perf_event_s *event,
                                     int flags)
{
  return 0;
}

/****************************************************************************
 * Name: perf_cpuclock_event_stop
 *
 * Description:
 *   Stop cpu clock event
 *
 * Input Parameters:
 *   event  - Perf event
 *   flags  - Perf event flags
 *
 * Returned Value:
 *   Result
 *
 ****************************************************************************/

static int perf_cpuclock_event_stop(FAR struct perf_event_s *event,
                                    int flags)
{
  return 0;
}

/****************************************************************************
 * Name: perf_cpuclock_event_read
 *
 * Description:
 *   Read cpu clock event
 *
 * Input Parameters:
 *   event  - Perf event
 *
 * Returned Value:
 *   Result
 *
 ****************************************************************************/

static int perf_cpuclock_event_read(FAR struct perf_event_s *event)
{
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int perf_event_overflow(FAR struct perf_event_s *event)
{
  return 0;
}

/****************************************************************************
 * Name: perf_event_init
 *
 * Description:
 *   Perf event initialization
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Init result
 *
 ****************************************************************************/

int perf_event_init(void)
{
  int i;

  /* Init perf event context */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      perf_init_context(&g_perf_cpu_ctx[i]);
    }

  /* Register clock event */

  perf_pmu_register(&g_perf_cpu_clock, "cpu_clock", -1);

  return OK;
}

/****************************************************************************
 * Name: perf_event_open
 *
 * Description:
 *   Create and open a perf event
 *
 * Input Parameters:
 *   attr     - Perf event attribute
 *   pid      - Task pid perf event bind
 *   cpu      - Cpu id perf event bind
 *   group_fd - Perf event fd for group leader
 *   flags    - Perf event flags
 *
 * Returned Value:
 *   Perf event fd
 *
 ****************************************************************************/

int perf_event_open(FAR struct perf_event_attr_s *attr, pid_t pid,
                    int cpu, int group_fd, unsigned long flags)
{
  FAR struct perf_event_s *group_leader = NULL;
  FAR struct perf_event_s *event;
  FAR struct tcb_s *tcb = NULL;
  FAR struct perf_event_context_s *ctx;
  FAR struct pmu_event_context_s *pmu_ctx;
  int event_fd = -1;
  int ret;

  if (pid == -1 && cpu == -1)
    {
      serr("perf event pid & cpu error\n");
      return -EINVAL;
    }

  if (cpu != -1 && cpu >= CONFIG_SMP_NCPUS)
    {
      serr("perf event cpu fail:%d\n", cpu);
      return -EINVAL;
    }

  if (group_fd >= 0)
    {
      FAR struct file *group_file;
      ret = fs_getfilep(group_fd, &group_file);
      if (ret < 0)
        {
          serr("perf event group_fd fail:%d\n", group_fd);
          return ret;
        }

      group_leader = group_file->f_priv;

      if (group_leader->cpu != cpu)
        {
          serr("Event cpu must be same as group leader\n");
          return -EINVAL;
        }
    }

  if (pid != -1)
    {
      if (pid == 0)
        {
          tcb = this_task();
        }
      else if (pid > 0)
        {
          tcb = nxsched_get_tcb(pid);
        }

      if (tcb == NULL)
        {
          serr("perf event pid fail:%d\n", pid);
          return -ESRCH;
        }
    }

  event = perf_event_alloc(attr, cpu, tcb, group_leader, NULL);
  if (event == NULL)
    {
      return -ENOMEM;
    }

  if (attr->disabled)
    {
      event->state = PERF_EVENT_STATE_OFF;
    }
  else
    {
      event->state = PERF_EVENT_STATE_INACTIVE;
    }

  ctx = perf_get_context(event, tcb);
  if (ctx == NULL)
    {
      ret = -ESRCH;
      goto err_with_event;
    }

  event->ctx = ctx;

  pmu_ctx = perf_get_pmu_context(event->pmu, event, ctx);
  if (pmu_ctx == NULL)
    {
      ret = -ESRCH;
      goto err_with_event;
    }

  event->pmuctx = pmu_ctx;

  event_fd = file_allocate(&g_perf_inode, O_RDONLY | flags,
                           0, event, 0, true);
  if (event_fd < 0)
    {
      ret = -EINVAL;
      goto err_with_event;
    }

  perf_add_event_to_count(event, ctx);
  return event_fd;

err_with_event:
  perf_free_event(event);
  return ret;
}

/****************************************************************************
 * Name: perf_pmu_register
 *
 * Description:
 *   Register perf pmu
 *
 * Input Parameters:
 *   pmu  - Pmu entry.
 *   name - Pmu name.
 *   type - Perf type id.
 *
 * Returned Value:
 *   Register result
 *
 ****************************************************************************/

int perf_pmu_register(FAR struct pmu_s *pmu, FAR const char *name, int type)
{
  int i;

  pmu->name = name;
  pmu->type = type;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      FAR struct pmu_cpu_context_s *cpu_pmu_ctx = &pmu->cpu_pmu_ctx[i];
      perf_init_pmu_context(&cpu_pmu_ctx->pmuctx, pmu);
    }

  nxmutex_lock(&g_perf_pmus_lock);
  list_add_tail(&g_perf_pmus, &pmu->node);
  nxmutex_unlock(&g_perf_pmus_lock);

  return OK;
}

/****************************************************************************
 * Name: perf_pmu_unregister
 *
 * Description:
 *   Unregister perf pmu
 *
 * Input Parameters:
 *   pmu  - Pmu entry.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void perf_pmu_unregister(FAR struct pmu_s *pmu)
{
  nxmutex_lock(&g_perf_pmus_lock);
  list_delete(&pmu->node);
  nxmutex_unlock(&g_perf_pmus_lock);
}

/****************************************************************************
 * Name: perf_event_task_init
 *
 * Description:
 *   Perf event task init, call during task initialization.
 *
 * Input Parameters:
 *   tcb   - Task for init.
 *
 * Returned Value:
 *   Initialization result
 *
 ****************************************************************************/

int perf_event_task_init(FAR struct tcb_s *tcb)
{
  int ret;

  tcb->perf_event_ctx = NULL;
  nxmutex_init(&tcb->perf_event_mutex);

  ret = perf_setup_task_context(tcb);
  if (ret < 0)
    {
      perf_event_task_exit(tcb);
    }

  return ret;
}

/****************************************************************************
 * Name: perf_event_task_exit
 *
 * Description:
 *   Perf event task free
 *
 * Input Parameters:
 *   tcb   - Task for init.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void perf_event_task_exit(FAR struct tcb_s *tcb)
{
  FAR struct perf_event_context_s *ctx = tcb->perf_event_ctx;

  if (ctx != NULL)
    {
      FAR struct perf_event_s *event;
      FAR struct perf_event_s *next;

      list_for_every_entry_safe(&ctx->event_list, event, next,
                                struct perf_event_s, event_node)
        {
          perf_context_detach(event, ctx);
          perf_free_event(event);
        }

      nxmutex_destroy(&tcb->perf_event_mutex);
    }
}

/****************************************************************************
 * Name: perf_event_task_sched_in
 *
 * Description:
 *   Record task sched in event
 *
 * Input Parameters:
 *   tcb - Refers to the head task which will be executed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void perf_event_task_sched_in(FAR struct tcb_s *tcb)
{
  FAR struct perf_event_context_s *ctx = tcb->perf_event_ctx;

  if (ctx != NULL)
    {
      irqstate_t flags = spin_lock_irqsave(&ctx->lock);

      perf_context_disable(ctx);

      perf_context_sched_in(ctx);

      perf_context_enable(ctx);

      spin_unlock_irqrestore(&ctx->lock, flags);
    }
}

/****************************************************************************
 * Name: perf_event_task_sched_out
 *
 * Description:
 *   Record task sched out event
 *
 * Input Parameters:
 *   tcb - Refers to the running task which will be blocked.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void perf_event_task_sched_out(FAR struct tcb_s *tcb)
{
  FAR struct perf_event_context_s *ctx = tcb->perf_event_ctx;

  if (ctx != NULL)
    {
      irqstate_t flags = spin_lock_irqsave(&ctx->lock);

      perf_context_disable(ctx);

      perf_context_sched_out(ctx);

      perf_context_enable(ctx);

      spin_unlock_irqrestore(&ctx->lock, flags);
    }
}
