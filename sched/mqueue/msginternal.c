/****************************************************************************
 * sched/mqueue/msginternal.c
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

#include <assert.h>
#include <stdint.h>

#include <nuttx/kmalloc.h>
#include "mqueue/msg.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MQ_PERBLOCK 10

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int                 g_nmsgq; /* The number of entries in g_msgqs */
static FAR struct msgq_s **g_msgqs; /* The queues; the index is the id - 1 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct list_node g_msgfreelist;
struct mqueue_cmn_s g_msgpoolwait;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmsg_alloc_internal
 ****************************************************************************/

static int nxmsg_alloc_internal(FAR struct msgq_s **pmsgq)
{
  FAR struct msgq_s *msgq;
  FAR struct msgq_s **tmp;
  int i;

  msgq = kmm_zalloc(sizeof(struct msgq_s));
  if (msgq == NULL)
    {
      return -ENOMEM;
    }

  for (i = 0; i < g_nmsgq; i++)
    {
      if (g_msgqs[i] == NULL)
        {
          g_msgqs[i] = msgq;
          msgq->id = i + 1;
          *pmsgq = msgq;
          return OK;
        }
    }

  /* The identifier must fit in the int16_t id of the queue */

  if (g_nmsgq > INT16_MAX - MQ_PERBLOCK)
    {
      kmm_free(msgq);
      return -ENOSPC;
    }

  tmp = kmm_realloc(g_msgqs, sizeof(FAR void *) *
                    (g_nmsgq + MQ_PERBLOCK));
  if (tmp == NULL)
    {
      kmm_free(msgq);
      return -ENOMEM;
    }

  g_msgqs = tmp;

  memset(&g_msgqs[g_nmsgq], 0, sizeof(FAR void *) * MQ_PERBLOCK);

  g_msgqs[g_nmsgq] = msgq;

  msgq->id = g_nmsgq + 1;

  g_nmsgq += MQ_PERBLOCK;

  *pmsgq = msgq;
  return OK;
}

/****************************************************************************
 * Name: nxmsg_wakeup
 *
 * Description:
 *   Wake up all tasks that wait on a removed queue.  Their msgrcv() or
 *   msgsnd() fails with EIDRM.
 *
 ****************************************************************************/

static void nxmsg_wakeup(FAR struct msgq_s *msgq)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct tcb_s *wtcb;
  bool need_switch = false;

  while ((wtcb = (FAR struct tcb_s *)
                 dq_remfirst(MQ_WNELIST(msgq->cmn))) != NULL ||
         (wtcb = (FAR struct tcb_s *)
                 dq_remfirst(MQ_WNFLIST(msgq->cmn))) != NULL)
    {
      wtcb->waitobj = NULL;
      wtcb->errcode = EIDRM;

      if (nxsched_add_readytorun(wtcb))
        {
          need_switch = true;
        }
    }

  if (need_switch)
    {
      up_switch_context(this_task(), rtcb);
    }
}

/****************************************************************************
 * Name: nxmsg_wakeup_list
 *
 * Description:
 *   Wake up all tasks in a wait list of a queue.  Each task checks again
 *   if it can go on.
 *
 ****************************************************************************/

static void nxmsg_wakeup_list(FAR dq_queue_t *list, FAR int16_t *nwait)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct tcb_s *btcb;
  bool need_switch = false;

  while (*nwait > 0)
    {
      btcb = (FAR struct tcb_s *)dq_remfirst(list);
      DEBUGASSERT(btcb != NULL);

      wd_cancel(&btcb->waitdog);
      (*nwait)--;
      btcb->waitobj = NULL;

      if (nxsched_add_readytorun(btcb))
        {
          need_switch = true;
        }
    }

  if (need_switch)
    {
      up_switch_context(this_task(), rtcb);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmsg_alloc
 ****************************************************************************/

int nxmsg_alloc(FAR struct msgq_s **pmsgq)
{
  FAR struct msgq_s *msgq;
  int ret;

  ret = nxmsg_alloc_internal(&msgq);
  if (ret < 0)
    {
      return ret;
    }

  /* Initialize the new named message queue */

  list_initialize(&msgq->msglist);

  msgq->qbytes     = MSG_DEF_QBYTES;
  msgq->maxmsgsize = MSG_MAX_BYTES;

  *pmsgq = msgq;
  return OK;
}

/****************************************************************************
 * Name: nxmsg_free
 ****************************************************************************/

void nxmsg_free(FAR struct msgq_s *msgq)
{
  FAR struct msgbuf_s *entry;
  FAR struct msgbuf_s *tmp;
  int index;

  if (msgq == NULL || msgq->id <= 0 || msgq->id > g_nmsgq)
    {
      return;
    }

  index = msgq->id - 1;

  /* Remove the queue from the table first, so that no task finds it after
   * the wake-up or while kmm_free() waits for the heap lock.
   */

  g_msgqs[index] = NULL;
  nxmsg_wakeup(msgq);

  list_for_every_entry_safe(&msgq->msglist, entry,
                            tmp, struct msgbuf_s, node)
    {
      list_delete(&entry->node);
      list_add_tail(&g_msgfreelist, &entry->node);
    }

  /* Also the senders of this queue that wait for a free message: they
   * get EIDRM.
   */

  nxmsg_wakeup_pool();
  kmm_free(msgq);
}

/****************************************************************************
 * Name: nxmsg_wakeup_senders
 ****************************************************************************/

void nxmsg_wakeup_senders(FAR struct msgq_s *msgq)
{
  nxmsg_wakeup_list(MQ_WNFLIST(msgq->cmn), &msgq->cmn.nwaitnotfull);
}

/****************************************************************************
 * Name: nxmsg_wakeup_receivers
 ****************************************************************************/

void nxmsg_wakeup_receivers(FAR struct msgq_s *msgq)
{
  nxmsg_wakeup_list(MQ_WNELIST(msgq->cmn), &msgq->cmn.nwaitnotempty);
}

/****************************************************************************
 * Name: nxmsg_wakeup_pool
 ****************************************************************************/

void nxmsg_wakeup_pool(void)
{
  nxmsg_wakeup_list(MQ_WNFLIST(g_msgpoolwait), &g_msgpoolwait.nwaitnotfull);
}

/****************************************************************************
 * Name: nxmsg_lookup
 ****************************************************************************/

FAR struct msgq_s *nxmsg_lookup(int msqid)
{
  if (msqid <= 0 || msqid > g_nmsgq)
    {
      return NULL;
    }

  return g_msgqs[msqid - 1];
}

/****************************************************************************
 * Name: nxmsg_findkey
 ****************************************************************************/

FAR struct msgq_s *nxmsg_findkey(key_t key)
{
  int i;

  for (i = 0; i < g_nmsgq; i++)
    {
      if (g_msgqs[i] != NULL && g_msgqs[i]->key == key)
        {
          return g_msgqs[i];
        }
    }

  return NULL;
}
