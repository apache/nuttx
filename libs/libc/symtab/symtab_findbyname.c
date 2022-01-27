/****************************************************************************
 * libs/libc/symtab/symtab_findbyname.c
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

#include <string.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/symtab.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: symtab_findbyname
 *
 * Description:
 *   Find the symbol in the symbol table with the matching name.
 *   This version assumes that table is not ordered with respect to symbol
 *   name and, hence, access time will be linear with respect to nsyms.
 *
 * Returned Value:
 *   A reference to the symbol table entry if an entry with the matching
 *   name is found; NULL is returned if the entry is not found.
 *
 ****************************************************************************/

FAR const struct symtab_s *
symtab_findbyname(FAR const struct symtab_s *symtab,
                  FAR const char *name, int nsyms)
{
#ifdef CONFIG_SYMTAB_ORDEREDBYNAME
  int low  = 0;
  int high = nsyms - 1;
  int mid;
  int cmp;
#endif

  if (symtab == NULL)
    {
      DEBUGASSERT(nsyms == 0);
      return NULL;
    }

#ifdef CONFIG_SYMTAB_DECORATED
  if (name[0] == '_')
    {
      name++;
    }
#endif

  DEBUGASSERT(name != NULL);

#ifdef CONFIG_SYMTAB_ORDEREDBYNAME
  while (low < high)
    {
      /* Compare the name to the one in the middle.  (or just below
       * the middle in the case where one is even and one is odd).
       */

      mid = (low + high) >> 1;
      cmp = strcmp(name, symtab[mid].sym_name);
      if (cmp < 0)
        {
          /* name < symtab[mid].sym_name
           *
           * NOTE: Because of truncation in the calculation of 'mid'.
           * 'mid' could be equal to 'low'
           */

          high = mid > low ? mid - 1 : low;
        }
      else if (cmp > 0)
        {
          /* name > symtab[mid].sym_name */

          low = mid + 1;
        }
      else
        {
          /* symtab[mid].sym_name == name */

          return &symtab[mid];
        }
    }

  /* low == high... One final check.  We might not have actually tested
   * the final symtab[] name.
   *
   *   Example: Only the last pass through loop, suppose low = 1, high = 2,
   *   mid = 1, and symtab[high].sym_name == name.  Then we would get here
   *   with low = 2, high = 2, but symtab[2].sym_name was never tested.
   */

  return strcmp(name, symtab[low].sym_name) == 0 ? &symtab[low] : NULL;
#else
  for (; nsyms > 0; symtab++, nsyms--)
    {
      if (strcmp(name, symtab->sym_name) == 0)
        {
          return symtab;
        }
    }

  return NULL;
#endif
}
