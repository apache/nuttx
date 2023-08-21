/****************************************************************************
 * libs/libc/tls/task_tls_destruct.c
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
#include <nuttx/mutex.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_TLS_TASK_NELEM) && CONFIG_TLS_TASK_NELEM > 0

static mutex_t g_tlslock = NXMUTEX_INITIALIZER;
static tls_dtor_t g_tlsdtor[CONFIG_TLS_TASK_NELEM];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_dtor
 ****************************************************************************/

static void tls_dtor(FAR void *arg)
{
  UNUSED(arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_tls_alloc
 *
 * Description:
 *   Allocate a global-unique task local storage data index
 *
 * Input Parameters:
 *   dtor     - The destructor of task local storage data element
 *
 * Returned Value:
 *   A TLS index that is unique.
 *
 ****************************************************************************/

int task_tls_alloc(tls_dtor_t dtor)
{
  int ret;
  int candidate;

  ret = nxmutex_lock(&g_tlslock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EUSERS;

  for (candidate = 0; candidate < CONFIG_TLS_TASK_NELEM; candidate++)
    {
      if (g_tlsdtor[candidate] == NULL)
        {
          if (dtor)
            {
              g_tlsdtor[candidate] = dtor;
            }
          else
            {
              g_tlsdtor[candidate] = tls_dtor;
            }

          ret = candidate;
          break;
        }
    }

  nxmutex_unlock(&g_tlslock);
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
 *    None
 *
 ****************************************************************************/

void task_tls_destruct(void)
{
  int candidate;
  uintptr_t elem;
  tls_dtor_t dtor;
  FAR struct task_info_s *info = task_get_info();

  for (candidate = CONFIG_TLS_TASK_NELEM - 1; candidate >= 0; candidate--)
    {
      elem = info->ta_telem[candidate];
      dtor = g_tlsdtor[candidate];
      if (dtor != NULL && elem != 0)
        {
          dtor((FAR void *)elem);
        }

      info->ta_telem[candidate] = 0;
    }
}

#endif
