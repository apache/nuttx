/****************************************************************************
 * libs/libc/dlfcn/lib_dlsymtab.c
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

#include <dlfcn.h>
#include <errno.h>

#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dlsymtab
 *
 * Description:
 *   dlsymtab() is a non-standard shared library interface.  It selects the
 *   symbol table to use when binding a shared library to the base firmware
 *   which may be in FLASH memory.
 *
 * Input Parameters:
 *   symtab   - The new symbol table.
 *   nsymbols - The number of symbols in the symbol table.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

int dlsymtab(FAR const struct symtab_s *symtab, int nsymbols)
{
#ifdef CONFIG_BUILD_KERNEL
  /* The KERNEL build is considerably more complex:  In order to be shared,
   * the .text portion of the module must be (1) build for PIC/PID operation
   * and (2) must like in a shared memory region accessible from all
   * processes.  The .data/.bss portion of the module must be allocated in
   * the user space of each process, but must lie at the same virtual address
   * so that it can be referenced from the one copy of the text in the shared
   * memory region.
   */

#warning Missing logic
  return -ENOSYS;

#else
  /* Set the symbol take information that will be used by this instance of
   * the module library.
   */

  modlib_setsymtab(symtab, nsymbols);
  return OK;
#endif
}
