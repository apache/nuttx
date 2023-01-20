/********************************************************************************
 * libs/libc/pthread/pthread_condattr_setpshared.c
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>
#include <errno.h>
#include <debug.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Type Declarations
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Private Data
 ********************************************************************************/

/********************************************************************************
 * Private Function Prototypes
 ********************************************************************************/

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: pthread_condattr_setpshared
 *
 * Description:
 *   The process-shared attribute is set to PTHREAD_PROCESS_SHARED to permit
 *   a cond to be operated upon by any thread that has access to the
 *   memory where the cond is allocated. If the process-shared attribute
 *   is PTHREAD_PROCESS_PRIVATE, the cond can only be operated upon by
 *   threads created within the same process as the thread that initialized
 *   the cond.
 *   If threads of different processes attempt to operate on such a cond,
 *   the behavior is undefined. The default value of the attribute is
 *   PTHREAD_PROCESS_PRIVATE.
 *
 *   Both constants PTHREAD_PROCESS_SHARED and PTHREAD_PROCESS_PRIVATE are
 *   defined in pthread.h.
 *
 * Input Parameters:
 *   attr - cond attributes to be modified.
 *   pshared - the new value of the pshared attribute.
 *
 * Returned Value:
 *   0 (OK) on success or EINVAL if either attr is invalid or pshared is not
 *   one of PTHREAD_PROCESS_SHARED or PTHREAD_PROCESS_PRIVATE.
 *
 * Assumptions:
 *
 ********************************************************************************/

int pthread_condattr_setpshared(FAR pthread_condattr_t *attr, int pshared)
{
  int ret = OK;

  if (!attr || (pshared != PTHREAD_PROCESS_SHARED &&
       pshared != PTHREAD_PROCESS_PRIVATE))
    {
      ret = EINVAL;
    }
  else
    {
      attr->pshared = pshared;
    }

  return ret;
}
