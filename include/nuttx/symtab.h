/****************************************************************************
 * include/nuttx/symtab.h
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

#ifndef __INCLUDE_NUTTX_SYMTAB_H
#define __INCLUDE_NUTTX_SYMTAB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* struct symbtab_s describes one entry in the symbol table.  A symbol table
 * is a fixed size array of struct symtab_s. The information is intentionally
 * minimal and supports only:
 *
 * 1. Function pointers as sym_values.  Of other kinds of values need to be
 *    supported, then typing information would also need to be included in
 *    the structure.
 *
 * 2. Fixed size arrays.  There is no explicit provisional for dynamically
 *    adding or removing entries from the symbol table (realloc might be
 *    used for that purpose if needed).  The intention is to support only
 *    fixed size arrays completely defined at compilation or link time.
 */

struct symtab_s
{
  FAR const char *sym_name;          /* A pointer to the symbol name string */
  FAR const void *sym_value;         /* The value associated with the string */
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: symtab_findbyname
 *
 * Description:
 *   Find the symbol in the symbol table with the matching name.
 *   The implementation will be linear with respect to nsyms if
 *   CONFIG_SYMTAB_ORDEREDBYNAME is not selected, and logarithmic
 *   if it is.
 *
 * Returned Value:
 *   A reference to the symbol table entry if an entry with the matching
 *   name is found; NULL is returned if the entry is not found.
 *
 ****************************************************************************/

FAR const struct symtab_s *
symtab_findbyname(FAR const struct symtab_s *symtab,
                  FAR const char *name, int nsyms);

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

FAR const struct symtab_s *
symtab_findbyvalue(FAR const struct symtab_s *symtab,
                   FAR void *value, int nsyms);

/****************************************************************************
 * Name: symtab_sortbyname
 *
 * Description:
 *   Sort the symbol table by name.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void symtab_sortbyname(FAR struct symtab_s *symtab, int nsyms);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_SYMTAB_H */
