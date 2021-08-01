/****************************************************************************
 * include/sys/utsname.h
 *
 *   Copyright (C) 2015 Stavros Polymenis. All rights reserved.
 *   Author: Stavros Polymenis <sp@orbitalfox.com>
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

#ifndef __INCLUDE_SYS_UTSNAME_H
#define __INCLUDE_SYS_UTSNAME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYS_NAMELEN      21
#define VERSION_NAMELEN  41

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These are the values returned by uname:
 *
 * FIELD       Default
 * sysname     NuttX
 * nodename    CONFIG_LIBC_HOSTNAME
 * release     From version.h
 * version     From version.h
 * machine     CONFIG_ARCH
 */

struct utsname
{
  char sysname[SYS_NAMELEN];      /* Name of OS */
  char nodename[HOST_NAME_MAX];   /* Name of this network node */
  char release[SYS_NAMELEN];      /* Release level */
  char version[VERSION_NAMELEN];  /* Version level */
  char machine[SYS_NAMELEN];      /* Machine hardware */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int uname(FAR struct utsname *name);

#endif /* __INCLUDE_SYS_UTSNAME_H */
