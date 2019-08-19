/****************************************************************************
 * libs/libc/pwd/lib_getpwbufr.c
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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pwd.h>

#include "pwd/lib_pwd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getpwbuf_r
 *
 * Description:
 *   libc/grp internal helper function for getpwuid_r and getpwnam_r to setup
 *   the caller supplied 'pwd' and 'buf' buffers once a matching entry has
 *   been found.
 *
 * Input Parameters:
 *   uid    - Value to set the passwd structure's pw_uid field to.
 *   gid    - Value to set the passwd structure's pw_gid field to.
 *   name   - Value to set the passwd structure's pw_name field to.
 *   dir    - Value to set the passwd structure's pw_dir field to.
 *   shell  - Value to set the passwd structure's pw_shell field to.
 *   pwd    - Pointer to the space to store the retrieved passwd structure in.
 *   buf    - String fields pointed to by the passwd struct are stored here.
 *   buflen - The length of buf in bytes.
 *   result - Pointer to the resulting passwd struct, or NULL in case of fail.
 *
 * Returned Value:
 *   On success getpwgid_r returns 0 and sets *result to pwd.  In case of
 *   failure an error number is returned.
 *
 ****************************************************************************/

int getpwbuf_r(uid_t uid, gid_t gid, FAR const char *name,
               FAR const char *dir, FAR const char *shell,
               FAR struct passwd *pwd, FAR char *buf, size_t buflen,
               FAR struct passwd **result)
{
  size_t reqdlen;

  reqdlen = strlen(name) + 1 + strlen(dir) + 1 + strlen(shell) + 1;

  if (buflen < reqdlen)
    {
      /* Insufficient buffer space supplied. */

      *result = NULL;
      return ERANGE;
    }

  pwd->pw_name  = buf;
  pwd->pw_dir   = &buf[strlen(name) + 1];
  pwd->pw_shell = &buf[strlen(name) + 1 + strlen(dir) + 1];

  pwd->pw_uid = uid;
  pwd->pw_gid = gid;
  strcpy(pwd->pw_name, name);
  strcpy(pwd->pw_dir, dir);
  strcpy(pwd->pw_shell, shell);

  *result = pwd;
  return 0;
}
