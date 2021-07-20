/****************************************************************************
 * sched/task/task_tls_alloc.c
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

#include <sys/types.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/tls.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_TLS_TASK_NELEM > 0

static int g_tlsset = 0;
static sem_t g_tlssem = SEM_INITIALIZER(1);
static tls_dtor_t g_tlsdtor[CONFIG_TLS_TASK_NELEM];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int task_tls_alloc(tls_dtor_t dtor)
{
  int ret;
  int candidate;

  ret = nxsem_wait(&g_tlssem);

  if (ret < 0)
    {
      return ret;
    }

  ret = -EAGAIN;

  for (candidate = 0; candidate < CONFIG_TLS_TASK_NELEM; candidate++)
    {
      int mask = 1 << candidate;
      if ((g_tlsset & mask) == 0)
        {
          g_tlsset |= mask;
          g_tlsdtor[candidate] = dtor;
          ret = candidate;
          break;
        }
    }

  nxsem_post(&g_tlssem);
  return ret;
}

/****************************************************************************
 * Name: task_tls_destruct
 *
 * Description:
 *   Destruct all TLS data element associated with allocated key
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *   A set of allocated TLS index
 *
 ****************************************************************************/

void task_tls_destruct(void)
{
  int candidate;
  uintptr_t elem;
  tls_dtor_t dtor;
  FAR struct task_info_s *info = task_get_info();

  for (candidate = 0; candidate < CONFIG_TLS_TASK_NELEM; candidate++)
    {
      int mask = 1 << candidate;
      if (g_tlsset & mask)
        {
          elem = info->ta_telem[candidate];
          dtor = g_tlsdtor[candidate];
          if (dtor)
            {
              dtor((void *)elem);
            }
        }
    }
}

#endif
