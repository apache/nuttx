/****************************************************************************
 * net/socket/net_checksd.c
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

#include <sched.h>
#include <errno.h>
#include <debug.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_checksd
 *
 * Description:
 *   Check if the socket descriptor is valid for the provided TCB and if it
 *   supports the requested access.  This trivial operation is part of the
 *   fdopen() operation when the fdopen() is performed on a socket
 *   descriptor.  It simply performs some sanity checking before permitting
 *   the socket descriptor to be wrapped as a C FILE stream.
 *
 ****************************************************************************/

int net_checksd(int sd, int oflags)
{
  FAR struct socket *psock = sockfd_socket(sd);

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      ninfo("No valid socket for sd: %d\n", sd);
      return -EBADF;
    }

  /* NOTE:  We permit the socket FD to be "wrapped" in a stream as
   * soon as the socket descriptor is created by socket().  Therefore
   * (1) we don't care if the socket is connected yet, and (2) there
   * are no access restrictions that can be enforced yet.
   */

  return OK;
}
