/****************************************************************************
 * libs/libc/symtab/symtab_sortbyname.c
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

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <nuttx/symtab.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int symtab_comparename(FAR const void *arg1, FAR const void *arg2)
{
  FAR const struct symtab_s *symtab1 = arg1;
  FAR const struct symtab_s *symtab2 = arg2;

  return strcmp(symtab1->sym_name, symtab2->sym_name);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

void symtab_sortbyname(FAR struct symtab_s *symtab, int nsyms)
{
  DEBUGASSERT(symtab != NULL && nsyms != 0);
  qsort(symtab, nsyms, sizeof(symtab[0]), symtab_comparename);
}
