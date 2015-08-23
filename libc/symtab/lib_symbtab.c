/****************************************************************************
 * libc/symtab/lib_symtab.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Canned symtab implemented by Pavel Pisa <ppisa@pikron.com>
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

#include <nuttx/config.h>

#ifdef CONFIG_LIBC_SYMTAB

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/binfmt/symtab.h>
#include <nuttx/binfmt/canned_symtab.h>

#include "canned_symtab.inc"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: canned_symtab_initialize
 *
 * Description:
 *   Setup system provided canned symbol table.  NOTE that this a user-space
 *   interface only.  It is not generally available to to kernel mode code
 *   in protected or kernel builds.  That is because exec_setsymtab() and
 *   g_symtab lie in different address spaces.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_FLAT) || !defined(__KERNEL__)
void canned_symtab_initialize(void)
{
#ifdef CONFIG_BUILD_FLAT
  /* In the FLAT build, exec_symtab() can be called directly from any logic.
   * Both the symbol table and the function exec_setsymtabe reside in the same
   * namespace and address space.
   */

  exec_setsymtab(g_symtab, NSYMBOLS);

#else
  /* In the user mode portion of a protected or kernel build, we must set
   * the symbol table indirectly through the boardctl() call gate that will
   * proxy the call to canned_symtab_select().  In this case
   *
   * - canned_symbtab_initialize() and g_symtab() lie in the user space.
   * - boardctl(), canned_symtabl_select(), and exec_setsymtab() reside in
   *   kernel space.
   *
   * Access to boardctl() is provided in user space throug a call gate.
   */

  struct symtab_desc_s symdesc;

  symdesc.symtab   = g_symtab;
  symdesc.nsymbols = NSYMBOLS;
  (void)boardctl(BOARDIOC_SYMTAB, (uinptr_t)&symdesc);
#endif
}
#endif

/****************************************************************************
 * Name: canned_symtab_select
 *
 * Description:
 *   Setup system provided canned symbol table.  This function only exists
 *   the kernel portion of a protected or kernel build.  It is called only
 *   by boardctl().  I this case:
 *
 *   - canned_symbtab_initialize() and g_symtab() lie in the user space.
 *   - boardctl(), canned_symtabl_select(), and exec_setsymtab() reside in
 *     kernel space.
 *
 *   Access to boardctl() is provided in user space through a call gate.
 *
 * Input Parameters:
 *   symtab - The symbol table to be used
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
      defined(__KERNEL__)
int canned_symtab_select(FAR const struct symtab_desc_s *symdesc)
{
  exec_setsymtab(symdesc->symtab, symdesc->nsymbols);
}
#endif

#endif /* CONFIG_LIBC_SYMTAB */
