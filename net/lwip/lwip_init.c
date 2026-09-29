/****************************************************************************
 * net/lwip/lwip_init.c
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

#include <nuttx/config.h>
#ifdef CONFIG_NET_LWIP

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"

#include "lwip_port.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_mutex_t g_lwip_init_lock = PTHREAD_MUTEX_INITIALIZER;
static bool g_lwip_inited;

extern int board_lwip_netif_init(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void nuttx_lwip_init_done(void *arg)
{
  sem_post((sem_t *)arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nuttx_lwip_initialize(void)
{
  sem_t done;
  int ret;

  pthread_mutex_lock(&g_lwip_init_lock);

  if (g_lwip_inited)
    {
      pthread_mutex_unlock(&g_lwip_init_lock);
      return 0;
    }

  if (sem_init(&done, 0, 0) != 0)
    {
      pthread_mutex_unlock(&g_lwip_init_lock);
      return -errno;
    }

  netif_default = NULL;
  tcpip_init(nuttx_lwip_init_done, &done);

  while (sem_wait(&done) < 0 && errno == EINTR)
    {
    }

  sem_destroy(&done);

  ret = board_lwip_netif_init();
  if (ret != 0)
    {
      pthread_mutex_unlock(&g_lwip_init_lock);
      return ret;
    }

  g_lwip_inited = true;
  pthread_mutex_unlock(&g_lwip_init_lock);
  return 0;
}

#endif /* CONFIG_NET_LWIP */
