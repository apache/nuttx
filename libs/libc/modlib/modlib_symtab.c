/****************************************************************************
 * libs/libc/modlib/modlib_symtab.c
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

#include <assert.h>

#include <nuttx/symtab.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MODLIB_HAVE_SYMTAB
  /* Symbol table used by dlsym */

#  ifndef CONFIG_MODLIB_SYMTAB_ARRAY
#    error "CONFIG_MODLIB_SYMTAB_ARRAY must be defined"
#  endif

  /* Number of Symbols in the Table */

#  ifndef CONFIG_MODLIB_NSYMBOLS_VAR
#    error "CONFIG_MODLIB_NSYMBOLS_VAR must be defined"
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_MODLIB_HAVE_SYMTAB
extern const struct symtab_s CONFIG_MODLIB_SYMTAB_ARRAY[];
extern int CONFIG_MODLIB_NSYMBOLS_VAR;
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const struct symtab_s *g_modlib_symtab;
static FAR int g_modlib_nsymbols;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_getsymtab
 *
 * Description:
 *   Get the current kernel symbol table selection as an atomic operation.
 *
 * Input Parameters:
 *   symtab - The location to store the symbol table.
 *   nsymbols - The location to store the number of symbols in the symbol
 *   table.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void modlib_getsymtab(FAR const struct symtab_s **symtab, FAR int *nsymbols)
{
  DEBUGASSERT(symtab != NULL && nsymbols != NULL);

  /* Borrow the registry lock to assure atomic access */

  modlib_registry_lock();
#ifdef CONFIG_MODLIB_HAVE_SYMTAB
  if (g_modlib_symtab == NULL)
    {
      g_modlib_symtab = CONFIG_MODLIB_SYMTAB_ARRAY;
      g_modlib_nsymbols = CONFIG_MODLIB_NSYMBOLS_VAR;
    }
#endif

  *symtab   = g_modlib_symtab;
  *nsymbols = g_modlib_nsymbols;
  modlib_registry_unlock();
}

/****************************************************************************
 * Name: modlib_setsymtab
 *
 * Description:
 *   Select a new kernel symbol table selection as an atomic operation.
 *
 * Input Parameters:
 *   symtab - The new symbol table.
 *   nsymbols - The number of symbols in the symbol table.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void modlib_setsymtab(FAR const struct symtab_s *symtab, int nsymbols)
{
  /* Borrow the registry lock to assure atomic access */

  modlib_registry_lock();
  g_modlib_symtab   = symtab;
  g_modlib_nsymbols = nsymbols;
  modlib_registry_unlock();
}
