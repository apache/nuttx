/****************************************************************************
 * net/netdb/netdb_sethostname.c
 *
 *   Copyright (C) 2015 Stavros Polymenis. All rights reserved.
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/net/netdb.h>
#include <arch/irq.h>

#include "netdb/netdb.h"

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

/****************************************************************************
 * Name: netdb_sethostname
 *
 * Description:
 *
 *   netdb_sethostname() sets the hostname to the value given in the
 *   character array name. The len argument specifies the number of bytes in
 *   name.  (Thus, name does not require a terminating null byte.)
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

int netdb_sethostname(FAR const char *name, size_t size)
{
  irqstate_t flags;

  /* Save the new host name, truncating to HOST_NAME_MAX if necessary.  This
   * internal copy is always NUL terminated.  The hostname is global resource.
   * There is a microscopic possibility that it could be accessed while we
   * are setting it.
   */

  flags = irqsave();
  strncpy(g_hostname, name, MIN(HOST_NAME_MAX, size));
  g_hostname[HOST_NAME_MAX] = '\0';
  irqrestore(flags);

  return 0;
}
