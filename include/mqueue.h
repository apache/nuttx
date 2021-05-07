/********************************************************************************
 * include/mqueue.h
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

#ifndef __INCLUDE_MQUEUE_H
#define __INCLUDE_MQUEUE_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <sys/types.h>
#include <signal.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

#define MQ_NONBLOCK O_NONBLOCK

/********************************************************************************
 * Public Type Declarations
 ********************************************************************************/

/* Message queue attributes */

struct mq_attr
{
  size_t         mq_maxmsg;    /* Max number of messages in queue */
  size_t         mq_msgsize;   /* Max message size */
  unsigned       mq_flags;     /* Queue flags */
  size_t         mq_curmsgs;   /* Number of messages currently in queue */
};

/* Message queue descriptor */

typedef int mqd_t;

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

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

mqd_t   mq_open(FAR const char *mq_name, int oflags, ...);
int     mq_close(mqd_t mqdes);
int     mq_unlink(FAR const char *mq_name);
int     mq_send(mqd_t mqdes, FAR const char *msg, size_t msglen,
                unsigned int prio);
int     mq_timedsend(mqd_t mqdes, FAR const char *msg, size_t msglen,
                     unsigned int prio, FAR const struct timespec *abstime);
ssize_t mq_receive(mqd_t mqdes, FAR char *msg, size_t msglen,
                   FAR unsigned int *prio);
ssize_t mq_timedreceive(mqd_t mqdes, FAR char *msg, size_t msglen,
                        FAR unsigned int *prio,
                        FAR const struct timespec *abstime);
int     mq_notify(mqd_t mqdes, FAR const struct sigevent *notification);
int     mq_setattr(mqd_t mqdes, FAR const struct mq_attr *mq_stat,
                   FAR struct mq_attr *oldstat);
int     mq_getattr(mqd_t mqdes, FAR struct mq_attr *mq_stat);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_MQUEUE_H */
