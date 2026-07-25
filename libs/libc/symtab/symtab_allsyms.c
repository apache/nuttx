/****************************************************************************
 * libs/libc/symtab/symtab_allsyms.c
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

#include <nuttx/allsyms.h>
#include <nuttx/symtab.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

extern const struct symtab_s g_allsyms[];
extern const int             g_nallsyms;

/****************************************************************************
 * Name: allsyms_lookup
 *
 * Description:
 *   Find the symbol in the symbol table with the matching name.
 *
 * Returned Value:
 *   A reference to the symbol table entry if an entry with the matching
 *   name is found; NULL is returned if the entry is not found.
 *
 ****************************************************************************/

static FAR const struct symtab_s *
allsyms_lookup(FAR const char *name, FAR void *value,
               FAR size_t *size)
{
  FAR const struct symtab_s *symbol = NULL;

  if (name)
    {
      symbol = symtab_findbyname(g_allsyms, name, g_nallsyms);
    }
  else if (value)
    {
      symbol = symtab_findbyvalue(g_allsyms, value, g_nallsyms);

      /* g_allsyms[0] and g_allsyms[g_nallsyms - 1] are boundary sentinels,
       * not real symbols.  A match against either one means 'value' falls
       * outside the range covered by real symbols, so treat it as not
       * found instead of reporting a bogus name/offset.
       */

      if (symbol == &g_allsyms[0] ||
          symbol == &g_allsyms[g_nallsyms - 1])
        {
          symbol = NULL;
        }
    }

  if (symbol == NULL)
    {
      *size = 0;
      return NULL;
    }

#ifdef CONFIG_SYMTAB_ORDEREDBYNAME
  /* g_allsyms is sorted by name here, not by value, so the physically
   * next table entry is not necessarily the next symbol by address.
   * Scan the real symbols (index 1..g_nallsyms - 2; 0 and g_nallsyms - 1
   * are the boundary sentinels) for the smallest address greater than
   * this symbol's, falling back to the high sentinel's now-meaningful
   * value if none is closer.
   */

  {
    FAR const struct symtab_s *next = &g_allsyms[g_nallsyms - 1];
    int i;

    for (i = 1; i < g_nallsyms - 1; i++)
      {
        if (g_allsyms[i].sym_value > symbol->sym_value &&
            g_allsyms[i].sym_value < next->sym_value)
          {
            next = &g_allsyms[i];
          }
      }

    *size = next->sym_value - symbol->sym_value;
  }
#else
  if (symbol != &g_allsyms[g_nallsyms - 1])
    {
      *size = (symbol + 1)->sym_value - symbol->sym_value;
    }
  else
    {
      *size = 0;
    }
#endif

  return symbol;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: allsyms_findbyname
 *
 * Description:
 *   Find the symbol in the symbol table with the matching name.
 *
 * Returned Value:
 *   A reference to the symbol table entry if an entry with the matching
 *   name is found; NULL is returned if the entry is not found.
 *
 ****************************************************************************/

FAR const struct symtab_s *allsyms_findbyname(FAR const char *name,
                                              FAR size_t *size)
{
  return allsyms_lookup(name, NULL, size);
}

/****************************************************************************
 * Name: symtab_findbyvalue
 *
 * Description:
 *   Find the symbol in the symbol table whose value closest (but not greater
 *   than), the provided value. This version assumes that table is not
 *   ordered with respect to symbol value and, hence, access time will be
 *   linear with respect to nsyms.
 *
 * Returned Value:
 *   A reference to the symbol table entry if an entry with the matching
 *   name is found; NULL is returned if the entry is not found.
 *
 ****************************************************************************/

FAR const struct symtab_s *allsyms_findbyvalue(FAR void *value,
                                               FAR size_t *size)
{
  return allsyms_lookup(NULL, value, size);
}

#else

FAR const struct symtab_s *allsyms_findbyname(FAR const char *name,
                                              FAR size_t *size)
{
  *size = 0;
  return NULL;
}

FAR const struct symtab_s *allsyms_findbyvalue(FAR void *value,
                                               FAR size_t *size)
{
  *size = 0;
  return NULL;
}

#endif /* CONFIG_BUILD_FLAT || __KERNEL__ */
