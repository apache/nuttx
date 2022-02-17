/****************************************************************************
 * libs/libc/unistd/lib_sethostname.c
 *
 *   Copyright (C) 2015 Stavros Polymenis. All rights reserved.
 *   Copyright (C) 2015, 2016 Gregory Nutt. All rights reserved.
 *   Author: Stavros Polymenis <sp@orbitalfox.com>
 *           Gregory Nutt <gnutt@nuttx.org>
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

#include <string.h>
#include <unistd.h>

#include <nuttx/irq.h>

/* Further, in the protected and kernel build modes where kernel and
 * application code are separated, the hostname is a common system property
 * and must reside only in the kernel.  In that case, this accessor
 * function only be called from user space is only via a kernel system call.
 */

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* This is the system hostname (defined in lib_gethostname). */

extern char g_hostname[HOST_NAME_MAX + 1];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sethostname
 *
 * Description:
 *
 *   sethostname() sets the hostname to the value given in the character
 *   array name. The len argument specifies the number of bytes in name.
 *   (Thus, name does not require a terminating null byte.)
 *
 * Conforming To
 *   SVr4, 4.4BSD (these interfaces first appeared in 4.2BSD). POSIX.1-2001
 *   specifies gethostname() but not sethostname().
 *
 * Input Parameters:
 *   name - The user buffer to providing the new host name.
 *   namelen - The size of the user buffer in bytes.
 *
 * Returned Value:
 *   Upon successful completion, 0 will be returned; otherwise, -1 will be
 *   returned.  No errors are defined;  errno variable is not set.
 *
 ****************************************************************************/

int sethostname(FAR const char *name, size_t namelen)
{
  irqstate_t flags;

  /* Save the new host name, truncating to HOST_NAME_MAX if necessary.  This
   * internal copy is always NUL terminated. The hostname is global resource.
   * There is a microscopic possibility that it could be accessed while we
   * are setting it.
   */

  flags = enter_critical_section();
  strlcpy(g_hostname, name, sizeof(g_hostname));
  leave_critical_section(flags);

  return 0;
}

#endif /* CONFIG_BUILD_FLAT || __KERNEL__ */
