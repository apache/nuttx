/****************************************************************************
 * libs/libc/pthread/pthread_attr_setdetachstate.c
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
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_attr_setdetachstate
 *
 * Description:
 *   Set the detachstate attribute
 *
 * Returned Value:
 *   Upon successful completion, pthread_attr_setdetachstate() shall return
 *   a value of 0; otherwise, an error number shall be returned to indicate
 *   the error.
 *
 ****************************************************************************/

int pthread_attr_setdetachstate(FAR pthread_attr_t *attr,
                                int detachstate)
{
  if (!attr || (detachstate != PTHREAD_CREATE_DETACHED &&
      detachstate != PTHREAD_CREATE_JOINABLE))
    {
      return EINVAL;
    }

  attr->detachstate = detachstate;

  return OK;
}
