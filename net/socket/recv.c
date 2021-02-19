/****************************************************************************
 * net/socket/recv.c
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
#ifdef CONFIG_NET

#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: recv
 *
 * Description:
 *   The recv() call is identical to recvfrom() with a NULL from parameter.
 *
 * Input Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *
 * Returned Value:
 *  (see recvfrom)
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t recv(int sockfd, FAR void *buf, size_t len, int flags)
{
  /* recv is a cancellation point, but that can all be handled by recvfrom */

  return recvfrom(sockfd, buf, len, flags, NULL, 0);
}

#endif /* CONFIG_NET */
