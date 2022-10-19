/****************************************************************************
 * sched/mqueue/msg.h
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
#define MSG_MAX_MSGS    16

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct msgq_s
{
  struct mqueue_cmn_s  cmn;
  struct list_node     msglist;       /* Prioritized message list */
  key_t                key;
  int16_t              maxmsgs;       /* Maximum number of messages in the queue */
  int16_t              nmsgs;         /* Number of message in the queue */
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
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN struct list_node g_msgfreelist;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxmsg_initialize
 *
 * Description:
 *   Initialize the message queue
 *
 ****************************************************************************/

void nxmsg_initialize(void);

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
 *   Free the message queue instance
 *
 ****************************************************************************/

void nxmsg_free(FAR struct msgq_s *msgq);

/****************************************************************************
 * Name: nxmsg_lookup
 *
 * Description:
 *   Find the message queue in look-up table
 *
 ****************************************************************************/

FAR struct msgq_s *nxmsg_lookup(key_t key);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* defined(CONFIG_MQ_MAXMSGSIZE) && CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* __SCHED_MQUEUE_MSG_H */
