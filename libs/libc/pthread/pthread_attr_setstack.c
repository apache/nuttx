/****************************************************************************
 * libs/libc/pthread/pthread_attr_setstack.c
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
#include <string.h>
#include <debug.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_attr_setstack
 *
 * Description:
 *
 * Parameters:
 *   attr
 *   stackaddr
 *   stacksize
 *
 * Return Value:
 *   0 if successful.  Otherwise, an error code.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_attr_setstack(FAR pthread_attr_t *attr,
                          FAR void *stackaddr, long stacksize)
{
  int ret;

  linfo("attr=0x%p stackaddr=0x%p stacksize=%ld\n",
        attr, stackaddr, stacksize);

  if (!attr || !stackaddr || stacksize < PTHREAD_STACK_MIN)
    {
      ret = EINVAL;
    }
  else
    {
      attr->stackaddr = stackaddr;
      attr->stacksize = stacksize;
      ret = OK;
    }

  linfo("Returning %d\n", ret);
  return ret;
}
