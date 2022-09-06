/****************************************************************************
 * libs/libc/wqueue/wqueue.h
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

#ifndef __LIBS_LIBC_WQUEUE_WQUEUE_H
#define __LIBS_LIBC_WQUEUE_WQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#if defined(CONFIG_LIBC_USRWORK) && !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This structure defines the state of one user-modework queue. */

struct usr_wqueue_s
{
  struct dq_queue_s q;      /* The queue of pending work */
  mutex_t           lock;   /* exclusive access to user-mode work queue */
  sem_t             wake;   /* The wake-up semaphore of the  usrthread */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The state of the user mode work queue */

extern struct usr_wqueue_s g_usrwork;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_LIBC_USRWORK && !__KERNEL__*/
#endif /* __LIBS_LIBC_WQUEUE_WQUEUE_H */
