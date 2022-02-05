/****************************************************************************
 * libs/libc/pthread/pthread_setname_np.c
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

#include <sys/prctl.h>
#include <errno.h>
#include <pthread.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_setname_np
 *
 * Description:
 *   Set the name of a thread
 *
 * Parameters:
 *   thread - The thread whose name is to be changed
 *   name   - The new thread name
 *
 * Returned Value:
 *   On success, these functions return 0;
 *   on error, they return a nonzero error number.
 *
 ****************************************************************************/

int pthread_setname_np(pthread_t thread, FAR const char *name)
{
  return prctl(PR_SET_NAME_EXT, name, (int)thread) < 0 ? get_errno() : 0;
}
