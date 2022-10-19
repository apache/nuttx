/****************************************************************************
 * sched/mqueue/msginternal.c
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

#include <nuttx/kmalloc.h>
#include "mqueue/msg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MQ_PERBLOCK 10

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t             g_nmsgq; /* The number of groups of msgs array */
static FAR struct msgq_s **g_msgqs; /* The pointer of two layer file descriptors array */

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct list_node g_msgfreelist = LIST_INITIAL_VALUE(g_msgfreelist);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmsg_alloc_internal
 ****************************************************************************/

static FAR struct msgq_s *nxmsg_alloc_internal(void)
{
  FAR struct msgq_s *msgq;
  FAR struct msgq_s **tmp;
  int i;

  msgq = kmm_zalloc(sizeof(struct msgq_s));
  if (msgq == NULL)
    {
      return NULL;
    }

  for (i = 0; i < g_nmsgq; i++)
    {
      if (g_msgqs[i] == NULL)
        {
          g_msgqs[i] = msgq;
          msgq->key = i + 1;
          return msgq;
        }
    }

  tmp = kmm_realloc(g_msgqs, sizeof(FAR void *) *
                    (g_nmsgq + MQ_PERBLOCK));
  if (tmp == NULL)
    {
      kmm_free(msgq);
      return NULL;
    }

  g_msgqs = tmp;

  memset(&g_msgqs[g_nmsgq], 0, sizeof(FAR void *) * MQ_PERBLOCK);

  g_msgqs[g_nmsgq] = msgq;

  msgq->key = g_nmsgq + 1;

  g_nmsgq += MQ_PERBLOCK;

  return msgq;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmsg_initialize
 ****************************************************************************/

void nxmsg_initialize(void)
{
  FAR struct msgbuf_s *msg;

  msg = (FAR struct msgbuf_s *)kmm_malloc(sizeof(*msg) *
                                          CONFIG_PREALLOC_MQ_MSGS);
  if (msg)
    {
      int i;
      for (i = 0; i < CONFIG_PREALLOC_MQ_MSGS; i++)
        {
          list_add_tail(&g_msgfreelist, &msg->node);
          msg++;
        }
    }
}

/****************************************************************************
 * Name: nxmsg_alloc
 ****************************************************************************/

int nxmsg_alloc(FAR struct msgq_s **pmsgq)
{
  FAR struct msgq_s *msgq;

  msgq = nxmsg_alloc_internal();
  if (msgq == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the new named message queue */

  list_initialize(&msgq->msglist);

  msgq->maxmsgs    = MSG_MAX_MSGS;
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

  if (msgq == NULL || msgq->key <= 0 || msgq->key > g_nmsgq)
    {
      return;
    }

  index = msgq->key - 1;

  list_for_every_entry_safe(&msgq->msglist, entry,
                            tmp, struct msgbuf_s, node)
    {
      list_delete(&entry->node);
      list_add_tail(&g_msgfreelist, &entry->node);
    }

  kmm_free(g_msgqs[index]);
  g_msgqs[index] = NULL;
}

/****************************************************************************
 * Name: nxmsg_lookup
 ****************************************************************************/

FAR struct msgq_s *nxmsg_lookup(key_t key)
{
  if (key <= 0 || key > g_nmsgq)
    {
      return NULL;
    }

  return g_msgqs[key - 1];
}
