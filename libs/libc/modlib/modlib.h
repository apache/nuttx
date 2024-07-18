/****************************************************************************
 * libs/libc/modlib/modlib.h
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

#ifndef __LIBS_LIBC_MODLIB_MODLIB_H
#define __LIBS_LIBC_MODLIB_MODLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/addrenv.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_verifyheader
 *
 * Description:
 *   Given the header from a possible ELF executable, verify that it is
 *   an ELF executable.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_verifyheader(FAR const Elf_Ehdr *header);

/****************************************************************************
 * Name: modlib_findsymtab
 *
 * Description:
 *   Find the symbol table section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_findsymtab(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_readsym
 *
 * Description:
 *   Read the ELF symbol structure at the specified index into memory.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   index    - Symbol table index
 *   sym      - Location to return the table entry
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_readsym(FAR struct mod_loadinfo_s *loadinfo, int index,
                   FAR Elf_Sym *sym, FAR Elf_Shdr *shdr);

/****************************************************************************
 * Name: modlib_symvalue
 *
 * Description:
 *   Get the value of a symbol.  The updated value of the symbol is returned
 *   in the st_value field of the symbol table entry.
 *
 * Input Parameters:
 *   modp      - Module state information
 *   loadinfo  - Load state information
 *   sym       - Symbol table entry (value might be undefined)
 *   sh_offset - Offset of strtab
 *   exports   - Pointer to the symbol table
 *   nexports  - Number of symbols in the symbol table*
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   EINVAL - There is something inconsistent in the symbol table (should
 *            only happen if the file is corrupted)
 *   ENOSYS - Symbol lies in common
 *   ESRCH  - Symbol has no name
 *   ENOENT - Symbol undefined and not provided via a symbol table
 *
 ****************************************************************************/

int modlib_symvalue(FAR struct module_s *modp,
                    FAR struct mod_loadinfo_s *loadinfo, FAR Elf_Sym *sym,
                    Elf_Off sh_offset,
                    FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: modlib_insertsymtab
 *
 * Description:
 *   Insert a symbol table for the current module.
 *
 * Input Parameters:
 *   modp     - Module state information
 *   loadinfo - Module load information
 *   shdr     - Symbol table section header
 *   sym      - Symbol table entry
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   ENOMEM - Symbol undefined and not provided via a symbol table
 *
 ****************************************************************************/

int modlib_insertsymtab(FAR struct module_s *modp,
                        FAR struct mod_loadinfo_s *loadinfo,
                        FAR Elf_Shdr *shdr,
                        FAR Elf_Sym *sym);

/****************************************************************************
 * Name: modlib_findglobal
 *
 * Description:
 *   Find a symbol in the global symbol table
 *
 * Input Parameters:
 *   modp     - Module state information
 *   loadinfo - Module load information
 *   shdr     - Symbol table section header
 *   sym      - Symbol table entry
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   ENOMEM - Symbol undefined and not provided via a symbol table
 *
 ****************************************************************************/

void *modlib_findglobal(FAR struct module_s *modp,
                        FAR struct mod_loadinfo_s *loadinfo,
                        FAR Elf_Shdr *shdr,
                        FAR Elf_Sym *sym);

/****************************************************************************
 * Name: modlib_loadhdrs
 *
 * Description:
 *   Loads program and section headers into memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_loadhdrs(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_sectname
 *
 * Description:
 *   Get the symbol name in loadinfo->iobuffer[].
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_sectname(FAR struct mod_loadinfo_s *loadinfo,
                    FAR const Elf_Shdr *shdr);

/****************************************************************************
 * Name: modlib_allocbuffer
 *
 * Description:
 *   Perform the initial allocation of the I/O buffer, if it has not already
 *   been allocated.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_allocbuffer(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_reallocbuffer
 *
 * Description:
 *   Increase the size of I/O buffer by the specified buffer increment.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_reallocbuffer(FAR struct mod_loadinfo_s *loadinfo,
                         size_t increment);

/****************************************************************************
 * Name: modlib_freebuffers
 *
 * Description:
 *  Release all working buffers.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_freebuffers(FAR struct mod_loadinfo_s *loadinfo);

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Name: modlib_addrenv_alloc
 *
 * Description:
 *   Allocate memory for the ELF image (textalloc and datastart). If
 *   CONFIG_ARCH_ADDRENV=n, textalloc will be allocated using kmm_zalloc()
 *   and datastart will be a offset from textalloc.  If
 *   CONFIG_ARCH_ADDRENV=y, then textalloc and datastart will be allocated
 *   using up_addrenv_create().  In either case, there will be a unique
 *   instance of textalloc and datastart (and stack) for each instance of a
 *   process.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   textsize - The size (in bytes) of the .text address environment needed
 *     for the ELF image (read/execute).
 *   datasize - The size (in bytes) of the .bss/.data address environment
 *     needed for the ELF image (read/write).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int modlib_addrenv_alloc(FAR struct mod_loadinfo_s *loadinfo,
                         size_t textsize, size_t datasize);

/****************************************************************************
 * Name: modlib_addrenv_select
 *
 * Description:
 *   Temporarily select the task's address environment.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int modlib_addrenv_select(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_addrenv_restore
 *
 * Description:
 *   Restore the address environment before modlib_addrenv_select() was
 *   called.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int modlib_addrenv_restore(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_addrenv_free
 *
 * Description:
 *   Release the address environment previously created by
 *   modlib_addrenv_alloc().  This function is called only under certain
 *   error conditions after the module has been loaded but not yet started.
 *   After the module has been started, the address environment will
 *   automatically be freed when the module exits.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void modlib_addrenv_free(FAR struct mod_loadinfo_s *loadinfo);

#endif /* CONFIG_ARCH_ADDRENV */
#endif /* __LIBS_LIBC_MODLIB_MODLIB_H */
