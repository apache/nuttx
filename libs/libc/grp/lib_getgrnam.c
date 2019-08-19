/****************************************************************************
 * libs/libc/grp/lib_getgrnam.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
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
#include <grp.h>

#include "grp/lib_grp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getgrnam
 *
 * Description:
 *   The getgrnam() function searches the group database for an entry with
 *   a matching name.
 *
 * Input Parameters:
 *   name - The group name to return a group structure for
 *
 * Returned Value:
 *   A pointer to a statically allocated group structure, or NULL if no
 *   matching entry is found or an error occurs.  Applications wishing to
 *   check for error situations should set errno to 0 before calling
 *   getgrnam().  If getgrnam() returns a null pointer and errno is set to
 *   non-zero, an error occurred.
 *
 ****************************************************************************/

FAR struct group *getgrnam(FAR const char *name)
{
#ifdef CONFIG_LIBC_GROUP_FILE
  int ret;

  ret = grp_findby_name(name, &g_group, g_group_buffer, GRPBUF_RESERVE_SIZE);
  if (ret != 1)
    {
      return NULL;
    }

  return &g_group;
#else
  if (strcmp(name, "root"))
    {
      return NULL;
    }

  return getgrbuf(ROOT_GID, ROOT_NAME, ROOT_PASSWD);
#endif
}
