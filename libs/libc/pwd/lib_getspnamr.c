/****************************************************************************
 * libs/libc/pwd/lib_getspnamr.c
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

#include <stdio.h>
#include <string.h>
#include <shadow.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUFFER_EXTRA 100

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This implementation support Openwall-style TCB passwords in place of
 * traditional shadow, if the appropriate directories and files exist.
 * Thus, it is careful to avoid following symlinks or blocking on fifos
 * which a malicious user might create in place of his or her TCB shadow
 * file. It also avoids any allocation to prevent memory-exhaustion
 * attacks via huge TCB shadow files
 */

static long xatol(FAR char **s)
{
  if (**s == ':' || **s == '\n')
    {
      return -1;
    }

  return strtol(*s, &(*s), 10);
}

static int parsespent(FAR char *s, FAR struct spwd *sp)
{
  sp->sp_namp = s;
  if (!(s = strchr(s, ':')))
    {
      return -1;
    }

  *s = 0;
  sp->sp_pwdp = ++s;
  if (!(s = strchr(s, ':')))
    {
      return -1;
    }

  *s = 0;
  s++;
  sp->sp_lstchg = xatol(&s);
  if (*s != ':')
    {
      return -1;
    }

  s++;
  sp->sp_min = xatol(&s);
  if (*s != ':')
    {
      return -1;
    }

  s++;
  sp->sp_max = xatol(&s);
  if (*s != ':')
    {
      return -1;
    }

  s++;
  sp->sp_warn = xatol(&s);
  if (*s != ':')
    {
      return -1;
    }

  s++;
  sp->sp_inact = xatol(&s);
  if (*s != ':')
    {
      return -1;
    }

  s++;
  sp->sp_expire = xatol(&s);
  if (*s != ':')
    {
      return -1;
    }

  s++;
  sp->sp_flag = xatol(&s);
  if (*s != '\n')
    {
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int getspnam_r(FAR const char *name, FAR struct spwd *sp, FAR char *buf,
               size_t size, FAR struct spwd **res)
{
  size_t l = strlen(name);
  size_t k;
  FAR FILE *f;
  int skip = 0;

  *res = NULL;

  /* Disallow potentially-malicious user names */

  if (*name == '.' || strchr(name, '/') || l == 0)
    {
      set_errno(EINVAL);
      return -1;
    }

  /* Buffer size must at least be able to hold name, plus some.. */

  if (size < l + BUFFER_EXTRA)
    {
      set_errno(ERANGE);
      return -1;
    }

  f = fopen(CONFIG_LIBC_PASSWD_FILEPATH, "rbe");
  if (f == NULL)
    {
      return -1;
    }

  while (fgets(buf, size, f) && (k = strlen(buf)) > 0)
    {
      if (skip || strncmp(name, buf, l) || buf[l] != ':')
        {
          skip = buf[k - 1] != '\n';
          continue;
        }

      if (buf[k - 1] != '\n')
        {
          set_errno(ERANGE);
          fclose(f);
          return -1;
        }

      if (parsespent(buf, sp) < 0)
        {
          continue;
        }

      *res = sp;
      break;
    }

  fclose(f);
  return 0;
}
