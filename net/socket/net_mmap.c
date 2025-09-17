/****************************************************************************
 * net/socket/net_mmap.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <assert.h>
#include <errno.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_mmap
 *
 * Description:
 *   Redirect the mmap operation on socket descriptors to the address family-
 *   specific mmap implementation. This function serves as a dispatcher that
 *   invokes the socket interface's mmap method if it exists.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   map   - Pointer to the memory mapping entry containing mapping
 *           details such as address, length, protection flags,
 *           and file offset.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int psock_mmap(FAR struct socket *psock, FAR struct mm_map_entry_s *map)
{
  DEBUGASSERT(psock != NULL && map != NULL);

  /* Let the address family's mmap() method handle the operation */

  DEBUGASSERT(psock->s_sockif != NULL);
  if (psock->s_sockif->si_mmap == NULL)
    {
      return -EOPNOTSUPP;
    }

  return psock->s_sockif->si_mmap(psock, map);
}
