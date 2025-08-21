/****************************************************************************
 * libs/libc/string/lib_strerror.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STRERROR_UNKNOWN "Unknown error"
#define STRERROR_BUFSIZE sizeof(STRERROR_UNKNOWN " 2000")

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct errno_strmap_s
{
  uint8_t   errnum;
  FAR char *str;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LIBC_STRERROR

/* This table maps all error numbers to descriptive strings.
 * The only assumption that the code makes with regard to this
 * this table is that it is ordered by error number.
 *
 * The size of this table is quite large.  Its size can be
 * reduced by eliminating some of the more obscure error
 * strings.
 */

#ifndef CONFIG_LIBC_STRERROR_SHORT

static const struct errno_strmap_s g_errnomap[] =
{
  { 0,                   "Success"           },
#define ERRNO_ITEM(name, id, str) \
  { name,                 str                },
#include <nuttx/errno_lookup.h>
#undef ERRNO_ITEM
};

#else /* CONFIG_LIBC_STRERROR_SHORT */

static const struct errno_strmap_s g_errnomap[] =
{
  { 0,                   "OK"                },
#define ERRNO_ITEM(name, id, str) \
  {name,                 #name               },
#include <nuttx/errno_lookup.h>
#undef ERRNO_ITEM
};

#endif /* CONFIG_LIBC_STRERROR_SHORT */

#define NERRNO_STRS (sizeof(g_errnomap) / sizeof(struct errno_strmap_s))

#endif /* CONFIG_LIBC_STRERROR */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strerror
 ****************************************************************************/

FAR char *strerror(int errnum)
{
#ifdef CONFIG_LIBC_STRERROR_ERRNUM
  static char s_err[STRERROR_BUFSIZE];
#endif
#ifdef CONFIG_LIBC_STRERROR
  int ndxlow = 0;
  int ndxhi  = NERRNO_STRS - 1;
  int ndxmid;

  do
    {
      ndxmid = (ndxlow + ndxhi) >> 1;
      if (errnum > g_errnomap[ndxmid].errnum)
        {
          ndxlow = ndxmid + 1;
        }
      else if (errnum < g_errnomap[ndxmid].errnum)
        {
          ndxhi = ndxmid - 1;
        }
      else
        {
          return g_errnomap[ndxmid].str;
        }
    }
  while (ndxlow <= ndxhi);
#endif
#ifdef CONFIG_LIBC_STRERROR_ERRNUM
  if (snprintf(s_err, sizeof(s_err), STRERROR_UNKNOWN " %d", errnum)
      < sizeof(s_err))
    {
      return s_err;
    }
#elif !defined(CONFIG_LIBC_STRERROR)
  UNUSED(errnum);
#endif

  return STRERROR_UNKNOWN;
}
