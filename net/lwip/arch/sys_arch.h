/****************************************************************************
 * net/lwip/arch/sys_arch.h
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

#ifndef __NET_LWIP_ARCH_SYS_ARCH_H
#define __NET_LWIP_ARCH_SYS_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYS_MBOX_NULL NULL
#define SYS_SEM_NULL  NULL

#ifndef SYS_MBOX_SIZE
#  define SYS_MBOX_SIZE 16
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef irqstate_t sys_prot_t;
typedef sem_t sys_sem_t;
typedef pthread_mutex_t sys_mutex_t;
typedef pthread_t sys_thread_t;

struct sys_mbox_s;
typedef struct sys_mbox_s *sys_mbox_t;

#endif /* __NET_LWIP_ARCH_SYS_ARCH_H */
