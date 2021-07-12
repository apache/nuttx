/****************************************************************************
 * include/nuttx/binfmt/nxflat.h
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

#ifndef __INCLUDE_NUTTX_BINFMT_NXFLAT_H
#define __INCLUDE_NUTTX_BINFMT_NXFLAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nxflat.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct provides a description of the currently loaded instantiation
 * of an nxflat binary.
 */

struct nxflat_loadinfo_s
{
  /* Instruction Space (ISpace):  This region contains the nxflat file header
   * plus everything from the text section.
   *
   * The ISpace region is allocated using mmap() and, thus, can be shared by
   * multiple tasks.  Ideally, will have only one mmap'ed text section
   * instance in the system for each module.
   */

  uintptr_t ispace;        /* Address where hdr/text is loaded */
  uint32_t entryoffs;      /* Offset from ispace to entry point */
  uint32_t isize;          /* Size of ispace. */

  /* Data Space (DSpace): This region contains all information that is
   * referenced as data (other than the stack which is separately allocated).
   *
   * If CONFIG_ARCH_ADDRENV=n, DSpace will be allocated using kmm_malloc()
   * (or kmm_zalloc()).
   * If CONFIG_ARCH_ADDRENV-y, then DSpace will be allocated using
   * up_addrenv_create().  In either case, there will be a unique instance
   * of DSpace (and stack) for each instance of a process.
   */

  struct dspace_s *dspace; /* Allocated D-Space (data/bss/etc) */
  uint32_t datasize;       /* Size of data segment in dspace */
  uint32_t bsssize;        /* Size of bss segment in dspace */
  uint32_t stacksize;      /* Size of stack (not allocated) */
  uint32_t dsize;          /* Size of dspace (may be large than parts) */

  /* This is temporary memory where relocation records will be loaded. */

  uint32_t relocstart;     /* Start of array of struct flat_reloc */
  uint16_t reloccount;     /* Number of elements in reloc array */

  /* Address environment.
   *
   * addrenv - This is the handle created by up_addrenv_create() that can be
   *   used to manage the tasks address space.
   * oldenv  - This is a value returned by up_addrenv_select() that must be
   *   used to restore the current address environment.
   */

#ifdef CONFIG_ARCH_ADDRENV
  group_addrenv_t addrenv; /* Task group address environment */
  save_addrenv_t oldenv;   /* Saved address environment */
#endif

  /* File descriptors */

  struct file file;        /* Descriptor for the file being loaded */

  /* This is a copy of the NXFLAT header (still in network order) */

  struct nxflat_hdr_s header;
};

/****************************************************************************
 * Public Functions Definitions
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
 * Name: nxflat_initialize
 *
 * Description:
 *   In order to use the NxFLAT binary format, this function must be called
 *   during system initialization to register the NXFLAT binary
 *   format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_initialize(void);

/****************************************************************************
 * Name: nxflat_uninitialize
 *
 * Description:
 *   Unregister the NXFLAT binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxflat_uninitialize(void);

/****************************************************************************
 * Name: nxflat_verifyheader
 *
 * Description:
 *   Given the header from a possible NXFLAT executable, verify that it is
 *   an NXFLAT executable.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_verifyheader(const struct nxflat_hdr_s *header);

/****************************************************************************
 * Name: nxflat_init
 *
 * Description:
 *   This function is called to configure the library to process an NXFLAT
 *   program binary.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_init(const char *filename, struct nxflat_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: nxflat_uninit
 *
 * Description:
 *   Releases any resources committed by nxflat_init().  This essentially
 *   undoes the actions of nxflat_init.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_uninit(struct nxflat_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: nxflat_load
 *
 * Description:
 *   Loads the binary specified by nxflat_init into memory, mapping
 *   the I-space executable regions, allocating the D-Space region,
 *   and initializing the data segment (relocation information is
 *   temporarily loaded into the BSS region.  BSS will be cleared
 *   by nxflat_bind() after the relocation data has been processed).
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_load(struct nxflat_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: nxflat_read
 *
 * Description:
 *   Read 'readsize' bytes from the object file at 'offset'
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_read(struct nxflat_loadinfo_s *loadinfo, char *buffer,
                int readsize, int offset);

/****************************************************************************
 * Name: nxflat_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by 'symtab'
 *   After binding the module, clear the BSS region (which held the
 *   relocation data) in preparation for execution.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

struct symtab_s;
int nxflat_bind(FAR struct nxflat_loadinfo_s *loadinfo,
                FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: nxflat_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of nxflat_load.  It is called only under certain error
 *   conditions after the module has been loaded but not yet started.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_unload(struct nxflat_loadinfo_s *loadinfo);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_NXFLAT_H */
