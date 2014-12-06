/****************************************************************************
 *  sched/mqueue/mqueue.h
 *
 *   Copyright (C) 2007, 2009, 2011, 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __SCHED_MQUEUE_MQUEUE_H
#define __SCHED_MQUEUE_MQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <mqueue.h>
#include <sched.h>
#include <signal.h>

#include <nuttx/mqueue.h>

#if CONFIG_MQ_MAXMSGSIZE > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MQ_MAX_BYTES   CONFIG_MQ_MAXMSGSIZE
#define MQ_MAX_MSGS    16
#define MQ_PRIO_MAX    _POSIX_MQ_PRIO_MAX

/* This defines the number of messages descriptors to allocate at each
 * "gulp."
 */

#define NUM_MSG_DESCRIPTORS 24

/* This defines the number of messages to set aside for exclusive use by
 * interrupt handlers
 */

#define NUM_INTERRUPT_MSGS   8

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

enum mqalloc_e
{
  MQ_ALLOC_FIXED = 0,  /* pre-allocated; never freed */
  MQ_ALLOC_DYN,        /* dynamically allocated; free when unused */
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

/****************************************************************************
 * Public Variables
 ****************************************************************************/

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Functions defined in mq_initialize.c ************************************/

void weak_function mq_initialize(void);
void mq_desblockalloc(void);

FAR struct mqueue_inode_s *mq_findnamed(FAR const char *mq_name);
void mq_msgfree(FAR struct mqueue_msg_s *mqmsg);

/* mq_waitirq.c ************************************************************/

void mq_waitirq(FAR struct tcb_s *wtcb, int errcode);

/* mq_rcvinternal.c ********************************************************/

int mq_verifyreceive(mqd_t mqdes, FAR char *msg, size_t msglen);
FAR struct mqueue_msg_s *mq_waitreceive(mqd_t mqdes);
ssize_t mq_doreceive(mqd_t mqdes, FAR struct mqueue_msg_s *mqmsg,
                     FAR char *ubuffer, FAR int *prio);

/* mq_sndinternal.c ********************************************************/

int mq_verifysend(mqd_t mqdes, FAR const char *msg, size_t msglen, int prio);
FAR struct mqueue_msg_s *mq_msgalloc(void);
int mq_waitsend(mqd_t mqdes);
int mq_dosend(mqd_t mqdes, FAR struct mqueue_msg_s *mqmsg,
              FAR const char *msg, size_t msglen, int prio);

/* mq_release.c ************************************************************/

struct task_group_s; /* Forward reference */
void mq_release(FAR struct task_group_s *group);

/* mq_recover.c ************************************************************/

void mq_recover(FAR struct tcb_s *tcb);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* __SCHED_MQUEUE_MQUEUE_H */

