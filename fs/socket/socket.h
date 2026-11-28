/****************************************************************************
 * fs/socket/socket.h
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

#ifndef __FS_SOCKET_SOCKET_H
#define __FS_SOCKET_SOCKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/net.h>

#include "fs_heap.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if CONFIG_FS_PREALLOC_SOCKETS > 0 || CONFIG_FS_ALLOC_SOCKETS > 0
FAR struct socket *sock_alloc(void);
void sock_free(FAR struct socket *psock);
#else
#  define sock_alloc() fs_heap_zalloc(sizeof(struct socket))
#  define sock_free(p) fs_heap_free(p)
#endif

#endif /* __FS_SOCKET_SOCKET_H */
