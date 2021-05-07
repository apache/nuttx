/****************************************************************************
 * libs/libc/symtab/symtab_findbyvalue.c
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

#include <stddef.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/symtab.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   value is found; NULL is returned if the entry is not found.
 *
 ****************************************************************************/

FAR const struct symtab_s *
symtab_findbyvalue(FAR const struct symtab_s *symtab,
                   FAR void *value, int nsyms)
{
  FAR const struct symtab_s *retval = NULL;

  DEBUGASSERT(symtab != NULL);
  for (; nsyms > 0; symtab++, nsyms--)
    {
      /* Look for symbols of lesser or equal value (probably address) to
       * value
       */

      if (symtab->sym_value <= value)
        {
          /* Found one.  Is it the largest we have found so far? */

          if (!retval || symtab->sym_value > retval->sym_value)
            {
              /* Yes, then it is the new candidate for the symbol whose value
               * just below 'value'
               */

              retval = symtab;

              /* If it is exactly equal to the search 'value', then we might
               * as well terminate early because we can't do any better than
               * that.
               */

              if (retval->sym_value == value)
                {
                  break;
                }
            }
        }
    }

  return retval;
}
