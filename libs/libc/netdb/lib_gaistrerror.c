/****************************************************************************
 * libs/libc/netdb/lib_gaistrerror.c
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
#include <netdb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STRERROR_UNKNOWN "Unknown error"
#define STRERROR_BUFSIZE sizeof(STRERROR_UNKNOWN " 10")

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct errno_strmap_s
{
  uint8_t         errnum;
  FAR const char *str;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LIBC_GAISTRERROR

/* This table maps all error numbers to descriptive strings.
 * The only assumption that the code makes with regard to this
 * this table is that it is ordered by error number.
 *
 * The size of this table is quite large.  Its size can be
 * reduced by eliminating some of the more obscure error
 * strings.
 *
 * Only short names are currently supported.
 */

static const struct errno_strmap_s g_gaierrnomap[] =
{
  { EAI_AGAIN,           "EAI_AGAIN"         },
  { EAI_BADFLAGS,        "EAI_BADFLAGS"      },
  { EAI_FAIL,            "EAI_FAIL"          },
  { EAI_FAMILY,          "EAI_FAMILY"        },
  { EAI_MEMORY,          "EAI_MEMORY"        },
  { EAI_NONAME,          "EAI_NONAME"        },
  { EAI_SERVICE,         "EAI_SERVICE"       },
  { EAI_SOCKTYPE,        "EAI_SOCKTYPE"      },
  { EAI_SYSTEM,          "EAI_SYSTEM"        },
  { EAI_OVERFLOW,        "EAI_OVERFLOW"      },
};

#define NERRNO_STRS (sizeof(g_gaierrnomap) / sizeof(struct errno_strmap_s))

#endif /* CONFIG_LIBC_GAISTRERROR */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gai_strerror
 ****************************************************************************/

FAR const char *gai_strerror(int errnum)
{
#ifdef CONFIG_LIBC_GAISTRERROR_ERRNUM
  static char s_err[STRERROR_BUFSIZE];
#endif
#ifdef CONFIG_LIBC_GAISTRERROR
  int ndxlow = 0;
  int ndxhi  = NERRNO_STRS - 1;
  int ndxmid;

  do
    {
      ndxmid = (ndxlow + ndxhi) >> 1;
      if (errnum > g_gaierrnomap[ndxmid].errnum)
        {
          ndxlow = ndxmid + 1;
        }
      else if (errnum < g_gaierrnomap[ndxmid].errnum)
        {
          ndxhi = ndxmid - 1;
        }
      else
        {
          return g_gaierrnomap[ndxmid].str;
        }
    }
  while (ndxlow <= ndxhi);
#endif
#ifdef CONFIG_LIBC_GAISTRERROR_ERRNUM
  if (snprintf(s_err, sizeof(s_err), STRERROR_UNKNOWN " %d", errnum)
      < sizeof(s_err))
    {
      return s_err;
    }
#elif !defined(CONFIG_LIBC_GAISTRERROR)
  UNUSED(errnum);
#endif

  return STRERROR_UNKNOWN;
}
