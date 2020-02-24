/****************************************************************************
 * net/socket/net_clear_sinzero.c
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
#include <netinet/in.h>

#include <errno.h>
#include <debug.h>
#include <string.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_clear_sinzero()
 *
 * Description:
 *   In IPv4, sin_zero field exists in sockaddr_in for padding and should
 *   be set to zero.
 *
 * Input Parameters:
 *   from    - Address of source (may be NULL)
 *   fromlen - The length of the address structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void net_clear_sinzero(FAR struct sockaddr *from, socklen_t len)
{
  struct sockaddr_in *sin;

  /* Check if from->sa_family is AF_INET(IPv4) and len is enough */

  if (from && from->sa_family == AF_INET &&
      sizeof(struct sockaddr_in) <= len)
    {
      /* Clear the sin_zero field */

      sin = (struct sockaddr_in *)from;
      memset(sin->sin_zero, 0, sizeof(sin->sin_zero));
    }
}
