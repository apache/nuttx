/****************************************************************************
 * include/nuttx/binfmt/canned_symtab.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_BINFMT_CANNED_SYMTAB_H
#define __INCLUDE_NUTTX_BINFMT_CANNED_SYMTAB_H

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
 * is a fixed size array of struct symtab_s.  The information is intentionally
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

/* In order to full describe a symbol table, a vector containing the address
 * of the symbol table and the number of elements in the symbol table is
 * required.
 */

struct symtab_s; /* Forward reference */
struct symtab_desc_s
{
  FAR struct symtab_s *symtab;
  int nsymbols;
}

/****************************************************************************
 * Public Functions
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
 * Name: canned_symtab_initialize
 *
 * Description:
 *   Setup system provided canned symbol table.  NOTE that this a user-space
 *   interface only.  It is not generally available to to kernel mode code
 *   in protected or kernel builds.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_FLAT) || !defined(__KERNEL__)
void canned_symtab_initialize(void);
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
int canned_symtab_select(FAR const struct symtab_desc_s *symdesc);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_CANNED_SYMTAB_H */

