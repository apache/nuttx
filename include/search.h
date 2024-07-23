/****************************************************************************
 * include/search.h
 *
 * $NetBSD: search.h,v 1.12 1999/02/22 10:34:28 christos Exp $
 * $FreeBSD: src/include/search.h,v 1.4 2002/03/23 17:24:53 imp Exp $
 *
 * Written by J.T. Conklin <jtc@netbsd.org>
 * Public domain.
 *
 ****************************************************************************/

#ifndef __INCLUDE_SEARCH_H
#define __INCLUDE_SEARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef struct entry
{
  FAR char *key;
  FAR void *data;
} ENTRY;

typedef enum
{
  FIND,
  ENTER,
  DELETE
} ACTION;

struct hsearch_data
{
  FAR struct internal_head *htable;
  size_t htablesize;
};

/****************************************************************************
 * Public Function Prototypes
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

int hcreate(size_t);

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

void hdestroy(void);

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

FAR ENTRY *hsearch(ENTRY, ACTION);

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

int hcreate_r(size_t, FAR struct hsearch_data *);

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

void hdestroy_r(FAR struct hsearch_data *);

/****************************************************************************
 * Name: hsearch_r
 *
 * Description:
 *   Search a hash table.
 *
 * Input Parameters:
 *   item - The search key.
 *   action - The action to take.
 *   result - The location to return the search result.
 *   htab - The hash table to be searched.
 *
 * Returned Value:
 *   1 on success; 0 on failure with errno set appropriately.
 *
 ****************************************************************************/

int hsearch_r(ENTRY, ACTION, FAR ENTRY **, FAR struct hsearch_data *);

#endif /* __INCLUDE_SEARCH_H */
