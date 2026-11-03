/****************************************************************************
 * sched/mqueue/msg.h
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

#ifndef __SCHED_MQUEUE_MSG_H
#define __SCHED_MQUEUE_MSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <nuttx/irq.h>
#include <nuttx/mqueue.h>
#include <sys/msg.h>

#include <errno.h>

#if defined(CONFIG_MQ_MAXMSGSIZE) && CONFIG_MQ_MAXMSGSIZE > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MSG_MAX_BYTES   CONFIG_MQ_MAXMSGSIZE
#define MSG_DEF_QBYTES  (16 * MSG_MAX_BYTES) /* Default msg_qbytes */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct msgq_s
{
  struct mqueue_cmn_s  cmn;
  struct list_node     msglist;       /* Prioritized message list */
  key_t                key;           /* The msgget() key, or IPC_PRIVATE */
  time_t               stime;         /* Time of the last msgsnd() */
  time_t               rtime;         /* Time of the last msgrcv() */
  time_t               ctime;         /* Time of the last change */
  struct ipc_perm      perm;          /* Ownership and permissions */
  pid_t                lspid;         /* PID of the last msgsnd() */
  pid_t                lrpid;         /* PID of the last msgrcv() */
  msglen_t             cbytes;        /* Number of bytes in the queue */
  msglen_t             qbytes;        /* Maximum bytes in the queue */
  int16_t              id;            /* The message queue identifier */
  uint16_t             maxmsgsize;    /* Max size of message in message queue */
};

struct msgbuf_s
{
  struct list_node node;
  uint16_t         msize;                /* Message data length */
  long             mtype;                /* Message type, must be > 0 */
  char             mtext[MSG_MAX_BYTES]; /* Message data */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

extern struct list_node g_msgfreelist;

/* The senders that wait for a free message in g_msgfreelist */

extern struct mqueue_cmn_s g_msgpoolwait;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxmsg_alloc
 *
 * Description:
 *   Allocate a message queue instance
 *
 ****************************************************************************/

int nxmsg_alloc(FAR struct msgq_s **pmsgq);

/****************************************************************************
 * Name: nxmsg_free
 *
 * Description:
 *   Remove the message queue, wake up its waiters with EIDRM and free it
 *
 ****************************************************************************/

void nxmsg_free(FAR struct msgq_s *msgq);

/****************************************************************************
 * Name: nxmsg_wakeup_senders
 *
 * Description:
 *   Wake up all tasks that wait in msgsnd() for room in the queue
 *
 ****************************************************************************/

void nxmsg_wakeup_senders(FAR struct msgq_s *msgq);

/****************************************************************************
 * Name: nxmsg_wakeup_receivers
 *
 * Description:
 *   Wake up all tasks that wait in msgrcv() for a message in the queue
 *
 ****************************************************************************/

void nxmsg_wakeup_receivers(FAR struct msgq_s *msgq);

/****************************************************************************
 * Name: nxmsg_wakeup_pool
 *
 * Description:
 *   Wake up all tasks that wait in msgsnd() for a free message
 *
 ****************************************************************************/

void nxmsg_wakeup_pool(void);

/****************************************************************************
 * Name: nxmsg_lookup
 *
 * Description:
 *   Find the message queue with the identifier msqid
 *
 ****************************************************************************/

FAR struct msgq_s *nxmsg_lookup(int msqid);

/****************************************************************************
 * Name: nxmsg_findkey
 *
 * Description:
 *   Find the message queue of a key (not IPC_PRIVATE)
 *
 ****************************************************************************/

FAR struct msgq_s *nxmsg_findkey(key_t key);

#ifdef __cplusplus
}
#endif

#endif /* defined(CONFIG_MQ_MAXMSGSIZE) && CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* __SCHED_MQUEUE_MSG_H */
