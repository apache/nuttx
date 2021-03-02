/****************************************************************************
 * libc/net/lib_sendmsg.c
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

#if defined(CONFIG_NET) && !defined(CONFIG_NET_CMSG)

#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sendmsg
 *
 * Description:
 *   The sendmsg() call is identical to sendto() with a NULL from parameter.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Buffer to send data
 *   len      Length of buffer
 *   flags    Receive flags
 *
 * Returned Value:
 *  (see sendto)
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t sendmsg(int sockfd, FAR struct msghdr *msg, int flags)
{
  FAR void *buf           = msg->msg_iov->iov_base;
  FAR struct sockaddr *to = msg->msg_name;
  socklen_t tolen         = msg->msg_namelen;
  size_t len              = msg->msg_iov->iov_len;

  if (msg->msg_iovlen == 1)
    {
      return sendto(sockfd, buf, len, flags, to, tolen);
    }
  else
    {
      set_errno(ENOTSUP);
      return ERROR;
    }
}

#endif /* CONFIG_NET && !CONFIG_NET_CMSG */
