/****************************************************************************
 * net/utils/net_cmsg.c
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

#include <sys/socket.h>

#include "utils/utils.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmsg_append
 *
 * Description:
 *   Append specified data into the control message, msg_control and
 *   msg_controllen will be changed to the appropriate value when success
 *
 * Input Parameters:
 *   msg       - Buffer to receive the message.
 *   level     - The level of control message.
 *   type      - The type of control message.
 *   value     - If the value is not NULL, this interface copies the data
 *               to the appropriate location in msg_control, and modify
 *               msg_control and msg_controllen.
 *               If the value is NULL, just modify the corresponding value
 *               of msg.
 *   value_len - The value length of control message.
 *
 * Returned Value:
 *   On success, a pointer to the start address of control message data,
 *               the caller can copy the data in.
 *   On failure, return NULL, because of msg_controllen is not enough
 *
 ****************************************************************************/

FAR void *cmsg_append(FAR struct msghdr *msg, int level, int type,
                      FAR void *value, int value_len)
{
  FAR struct cmsghdr *cmsg;
  unsigned long cmsgspace = CMSG_SPACE(value_len);
  FAR void *cmsgdata;

  if (msg->msg_controllen < cmsgspace)
    {
      return NULL;
    }

  cmsg              = CMSG_FIRSTHDR(msg);
  cmsg->cmsg_level  = level;
  cmsg->cmsg_type   = type;
  cmsg->cmsg_len    = CMSG_LEN(value_len);
  cmsgdata          = CMSG_DATA(cmsg);
  if (value)
    {
      memcpy(cmsgdata, value, value_len);
    }

  msg->msg_control     = (char *)msg->msg_control + cmsgspace;
  msg->msg_controllen -= cmsgspace;

  return cmsgdata;
}
