/****************************************************************************
 * include/sys/un.h
 *
 *   Copyright (C) 2015, 2020 Gregory Nutt. All rights reserved.
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
