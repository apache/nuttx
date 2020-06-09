/****************************************************************************
 * libc/net/lib_sendmsg.c
 *
 *   Copyright (C) 2007, 2008, 2012, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "libc.h"

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
  FAR struct sockaddr *to = msg->msg_name;
  socklen_t tolen = msg->msg_namelen;
  FAR struct iovec *iov;
  FAR struct iovec *end;
  FAR void *buf;
  size_t len;
  int ret;

  if (msg->msg_iovlen == 1)
    {
      return sendto(sockfd, msg->msg_iov->iov_base,
                    msg->msg_iov->iov_len, flags, to, tolen);
    }

  end = &msg->msg_iov[msg->msg_iovlen];
  for (len = 0, iov = msg->msg_iov; iov != end; iov++)
    {
      len += iov->iov_len;
    }

  buf = lib_malloc(len);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  for (len = 0, iov = msg->msg_iov; iov != end; iov++)
    {
      memcpy(buf + len, iov->iov_base, iov->iov_len);
      len += iov->iov_len;
    }

  ret = sendto(sockfd, buf, len, flags, to, tolen);

  lib_free(buf);

  return ret;
}

#endif /* CONFIG_NET && !CONFIG_NET_CMSG */
