/****************************************************************************
 * libs/libc/grp/lib_getgrbufr.c
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
#include <grp.h>

#include "grp/lib_grp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getgrbuf_r
 *
 * Description:
 *   libc/grp internal helper function for getgrgid_r and getgrnam_r to setup
 *   the caller supplied 'grp' and 'buf' buffers once a matching entry has
 *   been found.
 *
 * Input Parameters:
 *   gid    - Value to set grp->gr_gid to.
 *   name   - Value to set grp->gr_name to.
 *   passwd - Value to set grp->passwd to.
 *   grp    - Pointer to the space to store the retrieved group structure in.
 *   buf    - String fields pointed to by the group struct are stored here.
 *   buflen - The length of buf in bytes.
 *   result - Pointer to the resulting group struct, or NULL in case of fail.
 *
 * Returned Value:
 *   On success getgrgid_r returns 0 and sets *result to grp.  In case of
 *   failure an error number is returned.
 *
 ****************************************************************************/

int getgrbuf_r(gid_t gid, FAR const char *name, FAR const char *passwd,
               FAR struct group *grp, FAR char *buf, size_t buflen,
               FAR struct group **result)
{
  size_t reqdlen;
  size_t padlen;

  /* In 'buf' a NULL pointer value will be stored, which must be naturally
   * aligned, followed by the null terminated group name string and the null
   * terminated passwd string 'x' (indicating 'no password').  Make sure
   * sufficient buffer space was supplied by the caller.
   */

  padlen  = sizeof(FAR void *) - ((uintptr_t)buf % sizeof(FAR char *));
  reqdlen = sizeof(FAR void *) + strlen(name) + 1 + strlen(passwd) + 1;

  if (buflen < padlen + reqdlen)
    {
      /* Insufficient buffer space supplied. */

      *result = NULL;
      return ERANGE;
    }

  grp->gr_mem    = (FAR char **)&buf[padlen];
  grp->gr_name   = &buf[padlen + sizeof(FAR char *)];
  grp->gr_passwd = &buf[padlen + sizeof(FAR char *) + strlen(name) + 1];

  strcpy(grp->gr_name, name);
  strcpy(grp->gr_passwd, passwd);
  grp->gr_gid  = gid;
  *grp->gr_mem = NULL;

  *result = grp;
  return 0;
}
