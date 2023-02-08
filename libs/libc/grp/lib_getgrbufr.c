/****************************************************************************
 * libs/libc/grp/lib_getgrbufr.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
  size_t namesize;
  size_t passwdsize;

  /* In 'buf' a NULL pointer value will be stored, which must be naturally
   * aligned, followed by the null terminated group name string and the null
   * terminated passwd string 'x' (indicating 'no password').  Make sure
   * sufficient buffer space was supplied by the caller.
   */

  namesize = strlen(name) + 1;
  passwdsize = strlen(passwd) + 1;
  padlen  = sizeof(FAR void *) - ((uintptr_t)buf % sizeof(FAR char *));
  reqdlen = sizeof(FAR void *) + namesize + passwdsize;

  if (buflen < padlen + reqdlen)
    {
      /* Insufficient buffer space supplied. */

      *result = NULL;
      return ERANGE;
    }

  grp->gr_mem    = (FAR char **)&buf[padlen];
  grp->gr_name   = &buf[padlen + sizeof(FAR char *)];
  grp->gr_passwd = &buf[padlen + sizeof(FAR char *) + namesize];

  strlcpy(grp->gr_name, name, namesize);
  strlcpy(grp->gr_passwd, passwd, passwdsize);
  grp->gr_gid  = gid;
  *grp->gr_mem = NULL;

  *result = grp;
  return 0;
}
