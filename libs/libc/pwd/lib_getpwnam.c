/****************************************************************************
 * libs/libc/pwd/lib_getpwnam.c
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
#include <pwd.h>

#include "pwd/lib_pwd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getpwnam
 *
 * Description:
 *   The getpwnam() function searches the user database for an entry with a
 *   matching name.
 *
 * Input Parameters:
 *   name - The user name to return a passwd structure for.
 *
 * Returned Value:
 *   A pointer to a statically allocated passwd structure, or NULL if no
 *   matching entry is found or an error occurs.  Applications wishing to
 *   check for error situations should set errno to 0 before calling
 *   getpwnam().  If getpwnam() returns a null pointer and errno is set to
 *   non-zero, an error occurred.
 *
 ****************************************************************************/

FAR struct passwd *getpwnam(FAR const char *name)
{
#ifdef CONFIG_LIBC_PASSWD_FILE
  int ret;

  ret = pwd_findby_name(name, &g_passwd, g_passwd_buffer,
                        CONFIG_LIBC_PASSWD_LINESIZE);
  if (ret != 1)
    {
      return NULL;
    }

  return &g_passwd;
#else
  if (strcmp(name, ROOT_NAME))
    {
      return NULL;
    }

  return getpwbuf(ROOT_UID, ROOT_GID, ROOT_NAME, ROOT_DIR, ROOT_SHELL);
#endif
}
