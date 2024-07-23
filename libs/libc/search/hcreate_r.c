/****************************************************************************
 * libs/libc/search/hcreate_r.c
 *
 * $NetBSD: hcreate.c,v 1.2 2001/02/19 21:26:04 ross Exp $
 *
 * Copyright (c) 2001 Christopher G. Demetriou
 * All rights reserved.
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
 * <<Id: LICENSE_GC,v 1.1 2001/10/01 23:24:05 cgd Exp>>
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

#include <sys/types.h>
#include <sys/queue.h>

#include <errno.h>
#include <search.h>
#include <stdlib.h>
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIN_BUCKETS_LG2 4
#define MIN_BUCKETS     (1 << MIN_BUCKETS_LG2)
#define MAX_BUCKETS_LG2 (sizeof (size_t) * 8 - 1 - 5)
#define MAX_BUCKETS     ((size_t)1 << MAX_BUCKETS_LG2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct internal_entry
{
  SLIST_ENTRY(internal_entry) link;
  ENTRY ent;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

SLIST_HEAD(internal_head, internal_entry);
extern uint32_t (*g_default_hash)(FAR const void *, size_t);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hcreate_r
 *
 * Description:
 *   Create a new hash table.
 *
 * Input Parameters:
 *   nel - The number of elements in the hash table.
 *   htab - The location to return the hash table reference.
 *
 * Returned Value:
 *   1 on success; 0 on failure with errno set appropriately.
 *
 ****************************************************************************/

int hcreate_r(size_t nel, FAR struct hsearch_data *htab)
{
  size_t idx;
  unsigned int p2;

  /* Make sure this this isn't called when a table already exists. */

  if (htab->htable != NULL)
    {
      _NX_SETERRNO(-EINVAL);
      return 0;
    }

  /* If nel is too small, make it min sized. */

  if (nel < MIN_BUCKETS)
    {
      nel = MIN_BUCKETS;
    }

  /* If it's too large, cap it. */

  if (nel > MAX_BUCKETS)
    {
      nel = MAX_BUCKETS;
    }

  /* If it's is not a power of two in size, round up. */

  if ((nel & (nel - 1)) != 0)
    {
      for (p2 = 0; nel != 0; p2++)
        {
          nel >>= 1;
        }

      nel = 1 << p2;
    }

  /* Allocate the table. */

  htab->htablesize = nel;
  htab->htable = lib_malloc(htab->htablesize * sizeof htab->htable[0]);
  if (htab->htable == NULL)
    {
      _NX_SETERRNO(-ENOMEM);
      return 0;
    }

  /* Initialize it. */

  for (idx = 0; idx < htab->htablesize; idx++)
    {
      SLIST_INIT(&(htab->htable[idx]));
    }

  return 1;
}

/****************************************************************************
 * Name: hdestroy_r
 *
 * Description:
 *   Destroy a hash table.
 *
 * Input Parameters:
 *   htab - The hash table to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void hdestroy_r(FAR struct hsearch_data *htab)
{
  FAR struct internal_entry *ie;
  size_t idx;

  if (htab->htable == NULL)
    {
      return;
    }

  for (idx = 0; idx < htab->htablesize; idx++)
    {
      while (!SLIST_EMPTY(&(htab->htable[idx])))
        {
          ie = SLIST_FIRST(&(htab->htable[idx]));
          SLIST_REMOVE_HEAD(&(htab->htable[idx]), link);
          lib_free(ie->ent.key);
          lib_free(ie->ent.data);
          lib_free(ie);
        }
    }

  lib_free(htab->htable);
  htab->htable = NULL;
}

/****************************************************************************
 * Name: hsearch_r
 *
 * Description:
 *   Search for an entry in a hash table.
 *
 * Input Parameters:
 *   item - The search key
 *   action - The action to take if the item is not found
 *   retval - The location to return the search result
 *   htab - The hash table to be searched
 *
 * Returned Value:
 *   1 on success; 0 on failure with errno set appropriately.
 *
 ****************************************************************************/

int hsearch_r(ENTRY item, ACTION action, FAR ENTRY **retval,
              FAR struct hsearch_data *htab)
{
  FAR struct internal_head *head;
  FAR struct internal_entry *ie;
  uint32_t hashval;
  size_t len;

  len = strlen(item.key);
  hashval = (*g_default_hash)(item.key, len);

  head = &(htab->htable[hashval & (htab->htablesize - 1)]);
  ie = SLIST_FIRST(head);
  while (ie != NULL)
    {
      if (strcmp(ie->ent.key, item.key) == 0)
        {
          break;
        }

      ie = SLIST_NEXT(ie, link);
    }

  if (action == DELETE)
    {
      if (ie != NULL)
        {
          SLIST_REMOVE(head, ie, internal_entry, link);
          lib_free(ie->ent.key);
          lib_free(ie->ent.data);
          lib_free(ie);
          return 1;
        }

      return 0;
    }
  else if (ie != NULL)
    {
      *retval = &ie->ent;
      return 1;
    }
  else if (action == FIND)
    {
      *retval = NULL;
      return 0;
    }

  ie = lib_malloc(sizeof *ie);
  if (ie == NULL)
    {
      *retval = NULL;
      return 0;
    }

  ie->ent.key = item.key;
  ie->ent.data = item.data;

  SLIST_INSERT_HEAD(head, ie, link);
  *retval = &ie->ent;
  return 1;
}
