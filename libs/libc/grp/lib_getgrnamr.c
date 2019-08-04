/****************************************************************************
 * libs/libc/grp/lib_getgrnamr.c
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
 * Name: getgrnam_r
 *
 * Description:
 *   The getgrnam_r() function searches the group database for an entry with
 *   a matching name and stores the retrieved group structure in the space
 *   pointed to by grp.
 *
 * Input Parameters:
 *   name - The name of the group to return a group structure for.
 *   grp - Pointer to the space to store the retrieved group structure in.
 *   buf - The string fields pointed to by the group struct are stored here.
 *   buflen - The length of buf in bytes.
 *   result - Pointer to the resulting group struct, or NULL in case of fail.
 *
 * Returned Value:
 *   On success getgrnam_r returns 0 and sets *result to grp.  If no match
 *   is found, 0 is returned and *result is set to NULL.  In case of failure
 *   an error number is returned.
 *
 ****************************************************************************/

int getgrnam_r(FAR const char *name, FAR struct group *grp, FAR char *buf,
               size_t buflen, FAR struct group **result)
{
#ifdef CONFIG_LIBC_GROUP_FILE
  int ret;

  ret = grp_findby_name(name, grp, buf, buflen);
  if (ret != 1)
    {
      *result = NULL;
      return ret < 0 ? -ret : 0;
    }

  *result = grp;
  return 0;
#else
  if (strcmp(name, ROOT_NAME))
    {
      /* The only known group is 'root', which has a gid of 0.  Thus, report
       * back that no match was found.
       */

      *result = NULL;
      return 0;
    }

  return getgrbuf_r(ROOT_GID, ROOT_NAME, ROOT_PASSWD, grp, buf, buflen,
                    result);
#endif
}
