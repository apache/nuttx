/****************************************************************************
 * binfmt/binfmt_execsymtab.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/binfmt/symtab.h>

#ifdef CONFIG_LIBC_EXECFUNCS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If CONFIG_LIBC_EXECFUNCS is defined in the configuration, then the
 * following must also be defined:
 */

#ifdef CONFIG_EXECFUNCS_HAVE_SYMTAB
  /* Symbol table used by exec[l|v] */

#  ifndef CONFIG_EXECFUNCS_SYMTAB_ARRAY
#    error "CONFIG_EXECFUNCS_SYMTAB_ARRAY must be defined"
#  endif

  /* Number of Symbols in the Table */

#  ifndef CONFIG_EXECFUNCS_NSYMBOLS_VAR
#    error "CONFIG_EXECFUNCS_NSYMBOLS_VAR must be defined"
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_EXECFUNCS_HAVE_SYMTAB
extern const struct symtab_s CONFIG_EXECFUNCS_SYMTAB_ARRAY[];
extern int CONFIG_EXECFUNCS_NSYMBOLS_VAR;
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const struct symtab_s *g_exec_symtab;
static int g_exec_nsymbols;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec_getsymtab
 *
 * Description:
 *   Get the current symbol table selection as an atomic operation.
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

void exec_getsymtab(FAR const struct symtab_s **symtab, FAR int *nsymbols)
{
  irqstate_t flags;

  DEBUGASSERT(symtab != NULL && nsymbols != NULL);

  /* Disable interrupts very briefly so that both the symbol table and its
   * size are returned as a single atomic operation.
   */

  flags = enter_critical_section();

#ifdef CONFIG_EXECFUNCS_HAVE_SYMTAB
  /* If a bring-up symbol table has been provided and if the exec symbol
   * table has not yet been initialized, then use the provided start-up
   * symbol table.
   */

  if (g_exec_symtab == NULL)
    {
      g_exec_symtab = CONFIG_EXECFUNCS_SYMTAB_ARRAY;
      g_exec_nsymbols = CONFIG_EXECFUNCS_NSYMBOLS_VAR;
    }
#endif

  /* Return the symbol table and its size */

  *symtab   = g_exec_symtab;
  *nsymbols = g_exec_nsymbols;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: exec_setsymtab
 *
 * Description:
 *   Select a new symbol table selection as an atomic operation.
 *
 * Input Parameters:
 *   symtab - The new symbol table.
 *   nsymbols - The number of symbols in the symbol table.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void exec_setsymtab(FAR const struct symtab_s *symtab, int nsymbols)
{
  irqstate_t flags;

  DEBUGASSERT(symtab != NULL);

  /* Disable interrupts very briefly so that both the symbol table and its
   * size are set as a single atomic operation.
   */

  flags           = enter_critical_section();
  g_exec_symtab   = symtab;
  g_exec_nsymbols = nsymbols;
  leave_critical_section(flags);
}

#endif /* CONFIG_LIBC_EXECFUNCS */
