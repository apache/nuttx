/****************************************************************************
 * libs/libc/netdb/lib_gaistrerror.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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

#include <stdint.h>
#include <netdb.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct errno_strmap_s
{
  uint8_t     errnum;
  const char *str;
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
  return "Unknown error";
}

