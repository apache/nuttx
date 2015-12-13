/****************************************************************************
 * sched/module/module.h
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

#ifndef __SCHED_MODULE_MODULE_H
#define __SCHED_MODULE_MODULE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <elf32.h>

#include <nuttx/arch.h>
#include <nuttx/module.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the file to be loaded. */

struct symtab_s;
struct module_s
{
  FAR struct module_s *flink;          /* Supports a singly linked list */
  FAR char modulename[MODULENAME_MAX]; /* Module name */
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  mod_initializer_t initializer;       /* Module initializer function */
#endif
  mod_uninitializer_t uninitializer;   /* Module uninitializer function */
  FAR void *arg;                       /* Uninitializer argument */
  FAR void *alloc;                     /* Allocated kernel memory */
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  size_t textsize;                     /* Size of the kernel .text memory allocation */
  size_t datasize;                     /* Size of the kernel .bss/.data memory allocation */
#endif
};

/* This struct provides a description of the currently loaded instantiation
 * of the kernel module.
 */

struct mod_loadinfo_s
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
 * Public Data
 ****************************************************************************/

FAR const struct symtab_s *g_mod_symtab;
FAR int g_mod_nsymbols;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mod_initialize
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

int mod_initialize(FAR const char *filename,
                   FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_uninitialize
 *
 * Description:
 *   Releases any resources committed by mod_init().  This essentially
 *   undoes the actions of mod_initialize.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int mod_uninitialize(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_load
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

int mod_load(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by mod_setsymtab().
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int mod_bind(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_unload
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

int mod_unload(struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_verifyheader
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

int mod_verifyheader(FAR const Elf32_Ehdr *header);

/****************************************************************************
 * Name: mod_read
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

int mod_read(FAR struct mod_loadinfo_s *loadinfo, FAR uint8_t *buffer,
             size_t readsize, off_t offset);

/****************************************************************************
 * Name: mod_loadshdrs
 *
 * Description:
 *   Loads section headers into memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int mod_loadshdrs(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_findsection
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

int mod_findsection(FAR struct mod_loadinfo_s *loadinfo,
                    FAR const char *sectname);

/****************************************************************************
 * Name: mod_findsymtab
 *
 * Description:
 *   Find the symbol table section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int mod_findsymtab(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_readsym
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

int mod_readsym(FAR struct mod_loadinfo_s *loadinfo, int index,
                FAR Elf32_Sym *sym);

/****************************************************************************
 * Name: mod_symvalue
 *
 * Description:
 *   Get the value of a symbol.  The updated value of the symbol is returned
 *   in the st_value field of the symbol table entry.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   sym      - Symbol table entry (value might be undefined)
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

int mod_symvalue(FAR struct mod_loadinfo_s *loadinfo, FAR Elf32_Sym *sym);

/****************************************************************************
 * Name: mod_freebuffers
 *
 * Description:
 *  Release all working buffers.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int mod_freebuffers(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_allocbuffer
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

int mod_allocbuffer(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_reallocbuffer
 *
 * Description:
 *   Increase the size of I/O buffer by the specified buffer increment.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int mod_reallocbuffer(FAR struct mod_loadinfo_s *loadinfo, size_t increment);

/****************************************************************************
 * Name: mod_registry_lock
 *
 * Description:
 *   Get exclusive access to the module registry.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mod_registry_lock(void);

/****************************************************************************
 * Name: mod_registry_unlock
 *
 * Description:
 *   Relinquish the lock on the module registry
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mod_registry_unlock(void);

/****************************************************************************
 * Name: mod_registry_add
 *
 * Description:
 *   Add a new entry to the module registry.
 *
 * Input Parameters:
 *   modp - The module data structure to be registered.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

void mod_registry_add(FAR struct module_s *modp);

/****************************************************************************
 * Name: mod_registry_del
 *
 * Description:
 *   Remove a module entry from the registry
 *
 * Input Parameters:
 *   modp - The registry entry to be removed.
 *
 * Returned Value:
 *   Zero (OK) is returned if the registry entry was deleted.  Otherwise,
 *   a negated errno value is returned.
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

int mod_registry_del(FAR struct module_s *modp);

/****************************************************************************
 * Name: mod_registry_find
 *
 * Description:
 *   Find an entry in the module registry using the name of the module.
 *
 * Input Parameters:
 *   modulename - The name of the module to be found
 *
 * Returned Value:
 *   If the registry entry is found, a pointer to the module entry is
 *   returned.  NULL is returned if the they entry is not found.
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

FAR struct module_s *mod_registry_find(FAR const char *modulename);

#endif /* __SCHED_MODULE_MODULE_H */
