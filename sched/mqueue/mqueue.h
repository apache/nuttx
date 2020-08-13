/********************************************************************************
 *  sched/mqueue/mqueue.h
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
 ********************************************************************************/

#ifndef __SCHED_MQUEUE_MQUEUE_H
#define __SCHED_MQUEUE_MQUEUE_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <mqueue.h>
#include <sched.h>

#include <nuttx/mqueue.h>

#if CONFIG_MQ_MAXMSGSIZE > 0

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

#define MQ_MAX_BYTES   CONFIG_MQ_MAXMSGSIZE
#define MQ_MAX_MSGS    16
#define MQ_PRIO_MAX    _POSIX_MQ_PRIO_MAX

/* This defines the number of messages descriptors to allocate at each
 * "gulp."
 */

#define NUM_MSG_DESCRIPTORS  4

/* This defines the number of messages to set aside for exclusive use by
 * interrupt handlers
 */

#define NUM_INTERRUPT_MSGS   8

/********************************************************************************
 * Public Type Definitions
 ********************************************************************************/

enum mqalloc_e
{
  MQ_ALLOC_FIXED = 0,  /* Pre-allocated; never freed */
  MQ_ALLOC_DYN,        /* Dynamically allocated; free when unused */
  MQ_ALLOC_IRQ         /* Preallocated, reserved for interrupt handling */
};

/* This structure describes one buffered POSIX message. */

struct mqueue_msg_s
{
  FAR struct mqueue_msg_s *next;  /* Forward link to next message */
  uint8_t type;                   /* (Used to manage allocations) */
  uint8_t priority;               /* priority of message */
#if MQ_MAX_BYTES < 256
  uint8_t msglen;                 /* Message data length */
#else
  uint16_t msglen;                /* Message data length */
#endif
  char mail[MQ_MAX_BYTES];        /* Message data */
};

/********************************************************************************
 * Public Data
 ********************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* The g_msgfree is a list of messages that are available for general use.
 * The number of messages in this list is a system configuration item.
 */

EXTERN sq_queue_t  g_msgfree;

/* The g_msgfreeInt is a list of messages that are reserved for use by
 * interrupt handlers.
 */

EXTERN sq_queue_t  g_msgfreeirq;

/* The g_desfree data structure is a list of message descriptors available
 * to the operating system for general use. The number of messages in the
 * pool is a constant.
 */

EXTERN sq_queue_t  g_desfree;

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

struct tcb_s;        /* Forward reference */
struct task_group_s; /* Forward reference */

/* Functions defined in mq_initialize.c *****************************************/

void weak_function nxmq_initialize(void);
void nxmq_alloc_desblock(void);
void nxmq_free_msg(FAR struct mqueue_msg_s *mqmsg);

/* mq_waitirq.c *****************************************************************/

void nxmq_wait_irq(FAR struct tcb_s *wtcb, int errcode);

/* mq_rcvinternal.c *************************************************************/

int nxmq_verify_receive(mqd_t mqdes, FAR char *msg, size_t msglen);
int nxmq_wait_receive(mqd_t mqdes, FAR struct mqueue_msg_s **rcvmsg);
ssize_t nxmq_do_receive(mqd_t mqdes, FAR struct mqueue_msg_s *mqmsg,
                        FAR char *ubuffer, FAR unsigned int *prio);

/* mq_sndinternal.c *************************************************************/

int nxmq_verify_send(mqd_t mqdes, FAR const char *msg, size_t msglen,
                     unsigned int prio);
FAR struct mqueue_msg_s *nxmq_alloc_msg(void);
int nxmq_wait_send(mqd_t mqdes);
int nxmq_do_send(mqd_t mqdes, FAR struct mqueue_msg_s *mqmsg,
                 FAR const char *msg, size_t msglen, unsigned int prio);

/* mq_release.c *****************************************************************/

void nxmq_release(FAR struct task_group_s *group);

/* mq_recover.c *****************************************************************/

void nxmq_recover(FAR struct tcb_s *tcb);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* __SCHED_MQUEUE_MQUEUE_H */
