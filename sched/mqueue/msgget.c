/****************************************************************************
 * sched/mqueue/msgget.c
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

#include "mqueue/msg.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: msgget
 *
 * Description:
 *   Get a System V message queue identifier
 *   The msgget() system call returns the System V message queue
 *   identifier associated with the value of the key argument.  It may
 *   be used either to obtain the identifier of a previously created
 *   message queue (when msgflg is zero and key does not have the
 *   value IPC_PRIVATE), or to create a new set.
 *
 * Input Parameters:
 *   key    - Key associated with the message queue
 *   msgflg - Operations and permissions flag
 *
 * Returned Value:
 *   On success, msgget() returns the message queue identifier (a
 *   nonnegative integer).  On failure, -1 is returned, and errno is
 *   set to indicate the error.
 *
 ****************************************************************************/

int msgget(key_t key, int msgflg)
{
  FAR struct msgq_s *msgq;
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();

  msgq = nxmsg_lookup(key);
  if (msgq)
    {
      if ((msgflg & IPC_CREAT) && (msgflg & IPC_EXCL))
        {
          ret = -EEXIST;
        }
    }
  else
    {
      ret = (key != IPC_PRIVATE && !(msgflg & IPC_CREAT)) ?
            -ENOENT : nxmsg_alloc(&msgq);
    }

  leave_critical_section(flags);

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return msgq->key;
}
