/****************************************************************************
 * include/sys/un.h
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

#ifndef __INCLUDE_SYS_UN_H
#define __INCLUDE_SYS_UN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The size of sun_path is not specified. Different implementations us
 * different sizes. BSD4.3 uses a size of 108; BSD4.4 uses a size of 104.
 * Most implementation use a size that ranges from 92 to 108. Applications
 * should not assume a particular length for sun_path.
 *
 * _POSIX_PATH_MAX would be a good choice too.
 */

#define UNIX_PATH_MAX  108

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* A UNIX domain socket address is represented in the following structure.
 * This structure must be cast compatible with struct sockaddr.
 */

struct sockaddr_un
{
  sa_family_t sun_family;        /* AF_UNIX */
  char sun_path[UNIX_PATH_MAX];  /* pathname */
};

/* There are three types of addresses:
 *
 * 1. pathname:  sun_path holds a null terminated string.  The allocated
 *    size may be variable:  sizeof(sa_family_t) + strlen(pathname) + 1
 * 2. unnamed:  A unix socket that is not bound to any name.  This case
 *    there is no path.  The allocated size may be sizeof(sa_family_t)
 * 3. abstract. The abstract path is distinguished because the pathname
 *    consists of only the NUL terminator.  The allocated size is then
 *    sizeof(s_family_t) + 1.
 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_SYS_UN_H */
