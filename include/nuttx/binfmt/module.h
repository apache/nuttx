/****************************************************************************
 * include/nuttx/binfmt/module.h
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

#ifndef __INCLUDE_NUTTX_BINFMT_MODULE_H
#define __INCLUDE_NUTTX_BINFMT_MODULE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <elf32.h>

#include <nuttx/binfmt/binfmt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_MODULE_ALIGN_LOG2
#  define CONFIG_MODULE_ALIGN_LOG2 2
#endif

#ifndef CONFIG_MODULE_STACKSIZE
#  define CONFIG_MODULE_STACKSIZE 2048
#endif

#ifndef CONFIG_MODULE_BUFFERSIZE
#  define CONFIG_MODULE_BUFFERSIZE 128
#endif

#ifndef CONFIG_MODULE_BUFFERINCR
#  define CONFIG_MODULE_BUFFERINCR 32
#endif

/* Allocation array size and indices */

#define LIBMODULE_MODULE_ALLOC     0
#ifdef CONFIG_BINFMT_CONSTRUCTORS
#  define LIBMODULE_CTORS_ALLOC    1
#  define LIBMODULE_CTPRS_ALLOC    2
#  define LIBMODULE_NALLOC         3
#else
#  define LIBMODULE_NALLOC         1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct provides a description of the currently loaded instantiation
 * of the kernel module.
 */

struct mod_loadinfo_s
{
  /* elfalloc is the base address of the memory that is allocated to hold the
   * module image.
   *
   * If CONFIG_ARCH_ADDRENV=n, elfalloc will be allocated using kmm_malloc() (or
   * kmm_zalloc()).  If CONFIG_ARCH_ADDRENV-y, then elfalloc will be allocated using
   * up_addrenv_create().  In either case, there will be a unique instance
   * of elfalloc (and stack) for each instance of a process.
   *
   * The alloc[] array in struct binary_s will hold memory that persists after
   * the module has been loaded.
   */

  uintptr_t         textalloc;   /* .text memory allocated when module was loaded */
  uintptr_t         dataalloc;   /* .bss/.data memory allocated when module was loaded */
  size_t            textsize;    /* Size of the module .text memory allocation */
  size_t            datasize;    /* Size of the module .bss/.data memory allocation */
  off_t             filelen;     /* Length of the entire module file */
  Elf32_Ehdr        ehdr;        /* Buffered module file header */
  FAR Elf32_Shdr    *shdr;       /* Buffered module section headers */
  uint8_t           *iobuffer;   /* File I/O buffer */

  /* Constructors and destructors */

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  FAR void          *ctoralloc;  /* Memory allocated for ctors */
  FAR void          *dtoralloc;  /* Memory allocated dtors */
  FAR binfmt_ctor_t *ctors;      /* Pointer to a list of constructors */
  FAR binfmt_dtor_t *dtors;      /* Pointer to a list of destructors */
  uint16_t           nctors;     /* Number of constructors */
  uint16_t           ndtors;     /* Number of destructors */
#endif

  /* Address environment.
   *
   * addrenv - This is the handle created by up_addrenv_create() that can be
   *   used to manage the tasks address space.
   * oldenv  - This is a value returned by up_addrenv_select() that must be
   *   used to restore the current address environment.
   */

#ifdef CONFIG_ARCH_ADDRENV
  group_addrenv_t    addrenv;    /* Task group address environment */
  save_addrenv_t     oldenv;     /* Saved address environment */
#endif

  uint16_t           symtabidx;  /* Symbol table section index */
  uint16_t           strtabidx;  /* String table section index */
  uint16_t           buflen;     /* size of iobuffer[] */
  int                filfd;      /* Descriptor for the file being loaded */
};

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
 * These are APIs exported by libelf (but are used only by the binfmt logic):
 ****************************************************************************/

/****************************************************************************
 * Name: mod_init
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

int mod_init(FAR const char *filename, FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: mod_uninit
 *
 * Description:
 *   Releases any resources committed by mod_init().  This essentially
 *   undoes the actions of mod_init.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int mod_uninit(FAR struct mod_loadinfo_s *loadinfo);

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
 *   'loadinfo' using the exported symbol values provided by 'symtab'.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

struct symtab_s;
int mod_bind(FAR struct mod_loadinfo_s *loadinfo,
             FAR const struct symtab_s *exports, int nexports);

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
 * These are APIs used outside of binfmt by NuttX:
 ****************************************************************************/
/****************************************************************************
 * Name: mod_initialize
 *
 * Description:
 *   Module support is built unconditionally.  However, in order to
 *   use this binary format, this function must be called during system
 *   initialization in order to register the module binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int mod_initialize(void);

/****************************************************************************
 * Name: mod_uninitialize
 *
 * Description:
 *   Unregister the module loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mod_uninitialize(void);

/****************************************************************************
 * These are APIs must be provided by architecture-specific logic.
 * (These really belong in include/nuttx/arch.h):
 ****************************************************************************/
/****************************************************************************
 * Name: up_checkarch
 *
 * Description:
 *   Given the ELF header in 'hdr', verify that the module is appropriate
 *   for the current, configured architecture.  Every architecture that uses
 *   the module loader must provide this function.
 *
 * Input Parameters:
 *   hdr - The ELF header read from the module file.
 *
 * Returned Value:
 *   True if the architecture supports this module file.
 *
 ****************************************************************************/

bool up_checkarch(FAR const Elf32_Ehdr *hdr);

/****************************************************************************
 * Name: up_relocate and up_relocateadd
 *
 * Description:
 *   Perform on architecture-specific ELF relocation.  Every architecture
 *   that uses the module loader must provide this function.
 *
 * Input Parameters:
 *   rel - The relocation type
 *   sym - The ELF symbol structure containing the fully resolved value.
 *         There are a few relocation types for a few architectures that do
 *         not require symbol information.  For those, this value will be
 *         NULL.  Implementations of these functions must be able to handle
 *         that case.
 *   addr - The address that requires the relocation.
 *
 * Returned Value:
 *   Zero (OK) if the relocation was successful.  Otherwise, a negated errno
 *   value indicating the cause of the relocation failure.
 *
 ****************************************************************************/

int up_relocate(FAR const Elf32_Rel *rel, FAR const Elf32_Sym *sym,
                uintptr_t addr);
int up_relocateadd(FAR const Elf32_Rela *rel,
                   FAR const Elf32_Sym *sym, uintptr_t addr);

#ifdef CONFIG_UCLIBCXX_EXCEPTION
/****************************************************************************
 * Name: up_init_exidx
 *
 * Description:
 *   Load the boundaries of the Exception Index ELF section in order to
 *   support exception handling for loaded modules.
 *
 * Input Parameters:
 *   address - The ELF section address for the Exception Index
 *   size    - The size of the ELF section.
 *
 * Returned Value:
 *   Always returns Zero (OK).
 *
 ****************************************************************************/
int up_init_exidx(Elf32_Addr address, Elf32_Word size);
#endif

/****************************************************************************
 * Name: up_coherent_dcache
 *
 * Description:
 *   Ensure that the I and D caches are coherent within specified region
 *   by cleaning the D cache (i.e., flushing the D cache contents to memory
 *   and invalidating the I cache. This is typically used when code has been
 *   written to a memory region, and will be executed.
 *
 * Input Parameters:
 *   addr - virtual start address of region
 *   len  - Size of the address region in bytes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_COHERENT_DCACHE
void up_coherent_dcache(uintptr_t addr, size_t len);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_MODULE_H */
