/****************************************************************************
 * sched/pthread/pthread_sigmask.c
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

#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_sigmask
 *
 * Description:
 *   This function is a simple wrapper around nxsig_procmask().
 *   See the nxsig_procmask() function description for further
 *   information.
 *
 * Input Parameters:
 *   how - How the signal mask will be changed:
 *         SIG_BLOCK   - The resulting set is the union of
 *                       the current set and the signal set
 *                       pointed to by 'set'.
 *         SIG_UNBLOCK - The resulting set is the intersection
 *                       of the current set and the complement
 *                       of the signal set pointed to by 'set'.
 *         SIG_SETMASK - The resulting set is the signal set
 *                       pointed to by 'set'.
 *   set - Location of the new signal mask
 *   oset - Location to store the old signal mask
 *
 * Returned Value:
 *   On success, this function will return 0 (OK).  It will return EINVAL if
 *   how is invalid.
 *
 ****************************************************************************/

int pthread_sigmask(int how, FAR const sigset_t *set, FAR sigset_t *oset)
{
  int ret = nxsig_procmask(how, set, oset);
  return ret < 0 ? -ret : OK;
}
