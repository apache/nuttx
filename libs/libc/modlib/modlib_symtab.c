/****************************************************************************
 * libs/libc/modlib/modlib_symtab.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/symtab.h>
#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

FAR const struct symtab_s *g_modlib_symtab;
FAR int g_modlib_nsymbols;

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
 *   nsymbols - The location to store the number of symbols in the symbol table.
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
