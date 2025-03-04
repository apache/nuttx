/****************************************************************************
 * libs/libc/search/hcreate.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2001 Christopher G. Demetriou.All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * hcreate() / hsearch() / hdestroy()
 *
 * SysV/XPG4 hash table functions.
 *
 * Implementation done based on NetBSD manual page and Solaris manual page,
 * plus my own personal experience about how they're supposed to work.
 *
 * I tried to look at Knuth (as cited by the Solaris manual page), but
 * nobody had a copy in the office, so...
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <search.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct hsearch_data g_htab;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hcreate
 *
 * Description:
 *   The hcreate() function creates a new hashing table with nel elements.
 *   The hashing table will be used by subsequent calls to hsearch() with
 *   the same htab argument.  The hashing table is initialized with nel
 *   hashing buckets.
 *
 *   The hcreate_r() function is the reentrant version of hcreate().
 *
 * Returned Value:
 *   If successful, hcreate() and hcreate_r() return 1; otherwise, they
 *   return 0.
 *
 ****************************************************************************/

int hcreate(size_t nel)
{
  return hcreate_r(nel, &g_htab);
}

/****************************************************************************
 * Name: hdestroy
 *
 * Description:
 *   The hdestroy() function destroys the hashing table specified by htab.
 *   The hashing table is destroyed only if there are no entries in the
 *   table.  The hashing table cannot be used again until hcreate() or
 *   hcreate_r() is called.
 *
 *   The hdestroy_r() function is the reentrant version of hdestroy().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void hdestroy(void)
{
  hdestroy_r(&g_htab);
}

/****************************************************************************
 * Name: hsearch
 *
 * Description:
 *   The hsearch() function searches the hashing table specified by htab
 *   for an entry with a key matching that of item.  If such an entry is
 *   found, hsearch() returns a pointer to the entry's data object.  If
 *   such an entry is not found, hsearch() creates a new entry using the
 *   key and data objects specified by item and returns a pointer to the
 *   new entry's data object.
 *
 *   The hsearch_r() function is the reentrant version of hsearch().
 *
 * Returned Value:
 *   If successful, hsearch() and hsearch_r() return a pointer to the data
 *   object of the matching or newly created entry.  Otherwise, they return
 *   NULL.
 *
 ****************************************************************************/

FAR ENTRY *hsearch(ENTRY item, ACTION action)
{
  FAR ENTRY *retval = NULL;

  hsearch_r(item, action, &retval, &g_htab);

  return retval;
}
