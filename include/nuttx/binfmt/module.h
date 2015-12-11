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

/* A NuttX module is expected to export a function called module_initialize()
 * that has the following function prototype.  This function should appear as
 * the entry point in the ELF module file and will be called bythe binfmt
 * logic after the module has been loaded into kernel memory.
 *
 * As an alternative using GCC, the module may mark a function with the
 * "constructor" attribute and the module initializer will be called along
 * with any other C++ constructors.  The "destructor" attribute may also
 * be used to mark an module uninitialization function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure to
 *   initialize the module.
 */

typedef CODE int (*mod_initializer_t)(void);

/* This describes the file to be loaded.
 *
 * NOTE 1: The 'filename' must be the full, absolute path to the file to be
 * executed unless CONFIG_BINFMT_EXEPATH is defined.  In that case,
 * 'filename' may be a relative path; a set of candidate absolute paths
 * will be generated using the PATH environment variable and load_module()
 * will attempt to load each file that is found at those absolute paths.
 */

struct symtab_s;
struct module_s
{
  /* Information provided to insmod by the caller */

  FAR const char *filename;            /* Full path to the binary to be loaded (See NOTE 1 above) */
  FAR const struct symtab_s *exports;  /* Table of exported symbols */
  int nexports;                        /* The number of symbols in exports[] */

  /* Information provided from insmod (if successful) describing the
   * resources used by the loaded module.
   */

  mod_initializer_t initializer;       /* Module initializer function */
  FAR void *alloc;                     /* Allocated kernel memory */
};

/****************************************************************************
 * Public Function Prototypes
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
 * Name: insmod
 *
 * Description:
 *   Verify that the file is an ELF module binary and, if so, load the
 *   module into kernel memory and initialize it for use.
 *
 ****************************************************************************/

int insmod(FAR struct module_s *modp);

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
