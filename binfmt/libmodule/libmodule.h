/****************************************************************************
 * binfmt/libmodule/libmodule.h
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

#ifndef __BINFMT_LIBELF_LIBELF_H
#define __BINFMT_LIBELF_LIBELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <elf32.h>

#include <nuttx/arch.h>
#include <nuttx/binfmt/module.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct provides a description of the currently loaded instantiation
 * of the kernel module.
 */

struct libmod_loadinfo_s
{
  /* elfalloc is the base address of the memory that is allocated to hold the
   * module image.
   *
   * The alloc[] array in struct module_s will hold memory that persists after
   * the module has been loaded.
   */

  uintptr_t         textalloc;   /* .text memory allocated when module was loaded */
  uintptr_t         datastart;   /* Start of.bss/.data memory in .text allocation */
  size_t            textsize;    /* Size of the module .text memory allocation */
  size_t            datasize;    /* Size of the module .bss/.data memory allocation */
  off_t             filelen;     /* Length of the entire module file */
  Elf32_Ehdr        ehdr;        /* Buffered module file header */
  FAR Elf32_Shdr   *shdr;        /* Buffered module section headers */
  uint8_t          *iobuffer;    /* File I/O buffer */

  uint16_t          symtabidx;   /* Symbol table section index */
  uint16_t          strtabidx;   /* String table section index */
  uint16_t          buflen;      /* size of iobuffer[] */
  int               filfd;       /* Descriptor for the file being loaded */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * These are APIs exported by libmodule and used by insmod
 ****************************************************************************/

/****************************************************************************
 * Name: libmod_initialize
 *
 * Description:
 *   This function is called to configure the library to process an kernel
 *   module.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libmod_initialize(FAR const char *filename,
                      FAR struct libmod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libmod_uninitialize
 *
 * Description:
 *   Releases any resources committed by mod_init().  This essentially
 *   undoes the actions of libmod_initialize.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libmod_uninitialize(FAR struct libmod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libmod_load
 *
 * Description:
 *   Loads the binary into memory, allocating memory, performing relocations
 *   and initializing the data and bss segments.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libmod_load(FAR struct libmod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libmod_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by 'symtab'.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

struct symtab_s;
int libmod_bind(FAR struct libmod_loadinfo_s *loadinfo,
                FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: libmod_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of mod_load.  It is called only under certain error
 *   conditions after the module has been loaded but not yet started.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libmod_unload(struct libmod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libmod_verifyheader
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

int libmod_verifyheader(FAR const Elf32_Ehdr *header);

/****************************************************************************
 * Name: libmod_read
 *
 * Description:
 *   Read 'readsize' bytes from the object file at 'offset'.  The data is
 *   read into 'buffer.'
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libmod_read(FAR struct libmod_loadinfo_s *loadinfo, FAR uint8_t *buffer,
                size_t readsize, off_t offset);

/****************************************************************************
 * Name: libmod_loadshdrs
 *
 * Description:
 *   Loads section headers into memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libmod_loadshdrs(FAR struct libmod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libmod_findsection
 *
 * Description:
 *   A section by its name.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   sectname - Name of the section to find
 *
 * Returned Value:
 *   On success, the index to the section is returned; A negated errno value
 *   is returned on failure.
 *
 ****************************************************************************/

int libmod_findsection(FAR struct libmod_loadinfo_s *loadinfo,
                       FAR const char *sectname);

/****************************************************************************
 * Name: libmod_findsymtab
 *
 * Description:
 *   Find the symbol table section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libmod_findsymtab(FAR struct libmod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libmod_readsym
 *
 * Description:
 *   Read the ELFT symbol structure at the specfied index into memory.
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

int libmod_readsym(FAR struct libmod_loadinfo_s *loadinfo, int index,
                   FAR Elf32_Sym *sym);

/****************************************************************************
 * Name: libmod_symvalue
 *
 * Description:
 *   Get the value of a symbol.  The updated value of the symbol is returned
 *   in the st_value field of the symbol table entry.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   sym      - Symbol table entry (value might be undefined)
 *   exports  - The symbol table to use for resolving undefined symbols.
 *   nexports - Number of symbols in the symbol table.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   EINVAL - There is something inconsistent in the symbol table (should only
 *            happen if the file is corrupted)
 *   ENOSYS - Symbol lies in common
 *   ESRCH  - Symbol has no name
 *   ENOENT - Symbol undefined and not provided via a symbol table
 *
 ****************************************************************************/

int libmod_symvalue(FAR struct libmod_loadinfo_s *loadinfo, FAR Elf32_Sym *sym,
                    FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: libmod_freebuffers
 *
 * Description:
 *  Release all working buffers.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libmod_freebuffers(FAR struct libmod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libmod_allocbuffer
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

int libmod_allocbuffer(FAR struct libmod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libmod_reallocbuffer
 *
 * Description:
 *   Increase the size of I/O buffer by the specified buffer increment.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libmod_reallocbuffer(FAR struct libmod_loadinfo_s *loadinfo, size_t increment);

#endif /* __BINFMT_LIBELF_LIBELF_H */
