/****************************************************************************
 * libs/libc/grp/lib_getgrbuf.c
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
#include "libc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR char *g_buf;
static FAR struct group *g_grp;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getgrbuf
 *
 * Description:
 *   libc/grp internal helper function for getgrgid and getgrnam to allocate
 *   and setup a group structure once a matching entry has been found.
 *
 * Input Parameters:
 *   gid    - Value to set the group structure's gr_gid field to.
 *   name   - Value to set the group structure's gr_name field to.
 *   passwd - Value to set the group structure's passwd field to.
 *
 * Returned Value:
 *   A pointer to a statically allocated group structure, or NULL if an
 *   error occurs, in which case errno is set appropriately.
 *
 ****************************************************************************/

FAR struct group *getgrbuf(gid_t gid, FAR const char *name,
                           FAR const char *passwd)
{
  FAR struct group *result;
  FAR char *newbuf;
  size_t buflen;
  int err;

  buflen = sizeof(FAR char **) + strlen(name) + 1 + strlen(passwd) + 1;

  newbuf = (FAR char *)lib_realloc(g_buf, buflen);

  if (!newbuf)
    {
      err = ENOMEM;
      goto error;
    }

  g_buf = newbuf;

  if (!g_grp)
    {
      g_grp = (FAR struct group *)lib_malloc(sizeof(struct group));
    }

  if (!g_grp)
    {
      err = ENOMEM;
      goto error;
    }

  err = getgrbuf_r(gid, name, passwd, g_grp, g_buf, buflen, &result);

  if (err)
    {
      goto error;
    }

  return result;

error:
  lib_free(g_grp);
  lib_free(g_buf);
  g_grp = NULL;
  g_buf = NULL;
  set_errno(err);

  return NULL;
}
