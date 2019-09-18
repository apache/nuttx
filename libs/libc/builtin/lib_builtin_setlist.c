/****************************************************************************
 * libs/libc/builtin/libbuiltin_getname.c
 *
 * Originally by:
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With subsequent updates, modifications, and general maintenance by:
 *
 *   Copyright (C) 2012-2013, 2019 Gregory Nutt.  All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/lib/builtin.h>

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_FS_BINFS) && \
    defined(__KERNEL__)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR const struct builtin_s *g_builtins;
int g_builtin_count;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: builtin_setlist
 *
 * Description:
 *   Saves the user-space list of built-in applications for use by BINFS in
 *   protected mode.  Normally this is small set of globals provided by
 *   user-space logic.  It provides name-value pairs for associating
 *   built-in application names with user-space entry point addresses.
 *   These globals are only needed for use by BINFS which executes built-in
 *   applications from kernel-space in PROTECTED mode.  In the FLAT build,
 *   the user space globals are readily available.  (BINFS is not
 *   supportable in KERNEL mode since user-space address have no general
 *   meaning that configuration).
 *
 * Input Parameters:
 *   builtins - The list of built-in functions.  Each entry is a name-value
 *              pair that maps a built-in function name to its user-space
 *              entry point address.
 *   count    - The number of name-value pairs in the built-in list.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void builtin_setlist(FAR const struct builtin_s *builtins, int count)
{
  g_builtins      = builtins;
  g_builtin_count = count;
}

#endif /* CONFIG_BUILD_PROTECTED && CONFIG_FS_BINFS && __KERNEL__ */
