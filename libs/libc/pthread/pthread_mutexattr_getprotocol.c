/****************************************************************************
 * libs/libc/pthread/pthread_mutexattr_getprotocol.c
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
#include <debug.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutexattr_getprotocol
 *
 * Description:
 *    Return the value of the mutex protocol attribute.
 *
 * Input Parameters:
 *    attr     - A pointer to the mutex attributes to be queried.
 *    protocol - The user provided location in which to store the protocol
 *               value.
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code.
 *
 ****************************************************************************/

int pthread_mutexattr_getprotocol(FAR const pthread_mutexattr_t *attr,
                                  FAR int *protocol)
{
  DEBUGASSERT(attr != NULL && protocol != NULL);

#ifdef CONFIG_PRIORITY_INHERITANCE
  linfo("Returning %d\n", attr->proto);
  *protocol = attr->proto;
#else
  linfo("Returning %d\n", PTHREAD_PRIO_NONE);
  *protocol = PTHREAD_PRIO_NONE;
#endif /* CONFIG_PRIORITY_INHERITANCE */

  return 0;
}
