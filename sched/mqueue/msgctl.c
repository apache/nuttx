/****************************************************************************
 * sched/mqueue/msgctl.c
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
 * Name: msgctl
 *
 * Description:
 *   System V message control operations.
 *   msgctl() performs the control operation specified by cmd on the
 *   System V message queue with identifier msqid.
 *
 * Input Parameters:
 *   msqid    - System V message queue identifier
 *   cmd      - Command operations
 *   msqid_ds - Defines a message queue
 *
 * Returned Value:
 *   On success, IPC_STAT, IPC_SET, and IPC_RMID return 0.  A
 *   successful IPC_INFO or MSG_INFO operation returns the index of
 *   the highest used entry in the kernel's internal array recording
 *   information about all message queues.  (This information can be
 *   used with repeated MSG_STAT or MSG_STAT_ANY operations to obtain
 *   information about all queues on the system.)  A successful
 *   MSG_STAT or MSG_STAT_ANY operation returns the identifier of the
 *   queue whose index was given in msqid.
 *
 *   On failure, -1 is returned and errno is set to indicate the error.
 *
 ****************************************************************************/

int msgctl(int msqid, int cmd, FAR struct msqid_ds *buf)
{
  FAR struct msgq_s *msgq;
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();

  msgq = nxmsg_lookup(msqid);
  if (msgq == NULL)
    {
      ret = -EINVAL;
      goto errout_with_critical;
    }

  switch (cmd)
    {
      case IPC_RMID:
        {
          nxmsg_free(msgq);
          break;
        }

      case IPC_SET:
        {
          if (buf == NULL)
            {
              ret = -EFAULT;
              break;
            }

          msgq->maxmsgs = buf->msg_qbytes / CONFIG_MQ_MAXMSGSIZE;

          break;
        }

      case IPC_STAT:
        {
          if (buf == NULL)
            {
              ret = -EFAULT;
              break;
            }

          buf->msg_qnum   = list_length(&msgq->msglist);
          buf->msg_cbytes = buf->msg_qnum * CONFIG_MQ_MAXMSGSIZE;
          buf->msg_qbytes = msgq->maxmsgs * CONFIG_MQ_MAXMSGSIZE;
          break;
        }

      default:
        {
          ret = -EINVAL;
          break;
        }
    }

errout_with_critical:
  leave_critical_section(flags);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return ret;
}
