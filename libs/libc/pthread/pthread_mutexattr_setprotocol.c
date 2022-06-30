/****************************************************************************
 * libs/libc/pthread/pthread_mutexattr_setprotocol.c
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

#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutexattr_setprotocol
 *
 * Description:
 *    Set mutex protocol attribute.
 *
 * Input Parameters:
 *    attr     - A pointer to the mutex attributes to be modified
 *    protocol - The new protocol to use
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code.
 *
 ****************************************************************************/

int pthread_mutexattr_setprotocol(FAR pthread_mutexattr_t *attr,
                                  int protocol)
{
  linfo("attr=%p protocol=%d\n", attr, protocol);
  DEBUGASSERT(attr != NULL);

  if (protocol >= PTHREAD_PRIO_NONE && protocol <= PTHREAD_PRIO_PROTECT)
    {
      switch (protocol)
        {
          case PTHREAD_PRIO_NONE:
#ifdef CONFIG_PRIORITY_INHERITANCE
          case PTHREAD_PRIO_INHERIT:
            attr->proto = protocol;
#endif /* CONFIG_PRIORITY_INHERITANCE */
            return OK;

          default:
            return ENOTSUP;
        }
    }

  return EINVAL;
}
