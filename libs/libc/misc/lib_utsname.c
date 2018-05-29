/****************************************************************************
 * include/sys/utsname.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/utsname.h>
#include <string.h>

#include <nuttx/version.h>
#include <unistd.h>

/* In the protected and kernel build modes where kernel and application code
 * are separated, some of these common system property must reside only in
 * the kernel.  In that case, uname() can only be called from user space via
 * a kernel system call.
 */

#if (!defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)) || \
      defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uname
 *
 * Description:
 *   The uname() function will store information identifying the current
 *   system in the structure pointed to by name.
 *
 *   The uname() function uses the utsname structure defined in
 *   <sys/utsname.h>.
 *
 *   The uname() function will return a string naming the current system in
 *   the character array sysname. Similarly, nodename will contain the name
 *   of this node within an implementation-defined communications network.
 *   The arrays release and version will further identify the operating
 *   system. The array machine will contain a name that identifies the
 *   hardware that the system is running on.
 *
 *   The format of each member is implementation-defined.
 *
 * Input Parameters:
 *   name - The user-provided buffer to receive the system information.
 *
 * Returned Value:
 *   Upon successful completion, a non-negative value will be returned.
 *   Otherwise, -1 will be returned and errno set to indicate the error.
 *
 ****************************************************************************/

int uname(FAR struct utsname *name)
{
  int ret = 1;

  /* Copy the strings.  Assure that each is NUL terminated. */

  strncpy(name->sysname, "NuttX", SYS_NAMELEN);

#ifdef CONFIG_LIBC_NETDB
  /* Get the hostname */

  if (-1 == gethostname(name->nodename, HOST_NAME_MAX))
    {
      ret = -1;
    }

  name->nodename[HOST_NAME_MAX-1] = '\0';

#else
  strncpy(name->nodename, "", HOST_NAME_MAX);
#endif

  strncpy(name->release,  CONFIG_VERSION_STRING, SYS_NAMELEN);
  name->release[SYS_NAMELEN-1] = '\0';

  strncpy(name->version,  CONFIG_VERSION_BUILD, VERSION_NAMELEN);
  name->version[VERSION_NAMELEN-1] = '\0';

  strncpy(name->machine,  CONFIG_ARCH, SYS_NAMELEN);
  name->machine[SYS_NAMELEN-1] = '\0';

  return ret;
}

#endif /* (!CONFIG_BUILD_PROTECTED) && !CONFIG_BUILD_KERNEL) || __KERNEL__ */
