/****************************************************************************
 * libs/libc/pthread/pthread_attr_getaffinity.c
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
#include <debug.h>
#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_attr_getaffinity
 *
 * Description:
 *
 * Input Parameters:
 *   attr
 *   cpuset
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_attr_getaffinity_np(FAR const pthread_attr_t *attr,
                                size_t cpusetsize, cpu_set_t *cpuset)
{
  linfo("attr=%p cpusetsize=%d cpuset=%p\n",
         attr, (int)cpusetsize, cpuset);

  DEBUGASSERT(attr != NULL && cpusetsize == sizeof(cpu_set_t) &&
                                            cpuset != NULL);

  *cpuset = attr->affinity;
  return OK;
}
