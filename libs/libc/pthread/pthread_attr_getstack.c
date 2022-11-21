/****************************************************************************
 * libs/libc/pthread/pthread_attr_getstack.c
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
#include <pthread.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_attr_getstack
 *
 * Description:
 *
 * Parameters:
 *   attr
 *   stacksize
 *
 * Return Value:
 *   0 if successful.  Otherwise, an error code.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_attr_getstack(FAR pthread_attr_t *attr,
                          FAR void **stackaddr, FAR size_t *stacksize)
{
  int ret;

  linfo("attr=%p stackaddr=%p stacksize=%p\n",
        attr, stackaddr, stacksize);

  if (!attr || !stackaddr || !stacksize)
    {
      ret = EINVAL;
    }
  else
    {
      *stackaddr = attr->stackaddr;
      *stacksize = attr->stacksize;
      ret = OK;
    }

  linfo("Returning %d\n", ret);
  return ret;
}
