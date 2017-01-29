/****************************************************************************
 * include/nuttx/module.h
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

#ifndef __INCLUDE_NUTTX_MODULE_H
#define __INCLUDE_NUTTX_MODULE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <elf32.h>

#include <nuttx/arch.h>
#include <nuttx/symtab.h>
#include <nuttx/binfmt/binfmt.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the function that is called to uninitialize the
 * the loaded module.  This may mean, for example, un-registering a device
 * driver. If the module is successfully initialized, its memory will be
 * deallocated.
 *
 * Input Parameters:
 *   arg - An opaque argument that was previously returned by the initializer
 *         function.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure to
 *   initialize the module.  If zero is returned, then the module memory
 *   will be deallocated.  If the module is still in use (for example with
 *   open driver instances), the uninitialization function should fail with
 *   -EBUSY
 */

typedef CODE int (*mod_uninitializer_t)(FAR void *arg);

/* The contect of this structure is returned by module_initialize().
 *
 *   uninitializer - The pointer to the uninitialization function.  NULL may
 *                   be returned if no uninitialization is needed (i.e, the
 *                   the module memory can be deallocated at any time).
 *   arg           - An argument that will be passed to the uninitialization
                     function.
 *   exports       - A symbol table exported by the module
 *   nexports      - The number of symbols in the exported symbol table.
 */

struct mod_info_s
{
  mod_uninitializer_t uninitializer;   /* Module uninitializer */
  FAR void *arg;                       /* Uninitializer argument */
  FAR const struct symtab_s *exports;  /* Symbols exported by module */
  unsigned int nexports;               /* Number of symobols in exports list */
};

/* A NuttX module is expected to export a function called module_initialize()
 * that has the following function prototype.  This function should appear as
 * the entry point in the ELF module file and will be called by the binfmt
 * logic after the module has been loaded into kernel memory.
 *
 * Input Parameters:
 *   modinfo - Module information returned by mod_initialize().
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure to
 *   initialize the module.
 */

typedef CODE int (*mod_initializer_t)(FAR struct mod_info_s *modinfo);

#if defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT)
/* This is the type of the callback function used by mod_registry_foreach() */

struct module_s;
typedef CODE int (*mod_callback_t)(FAR struct module_s *modp, FAR void *arg);
#endif

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
 * Name: mod_getsymtab
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

#if defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT)
void mod_getsymtab(FAR const struct symtab_s **symtab, FAR int *nsymbols);
#endif

/****************************************************************************
 * Name: mod_setsymtab
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

#if defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT)
void mod_setsymtab(FAR const struct symtab_s *symtab, int nsymbols);
#endif

/****************************************************************************
 * Name: insmod
 *
 * Description:
 *   Verify that the file is an ELF module binary and, if so, load the
 *   module into kernel memory and initialize it for use.
 *
 *   NOTE: mod_setsymtab had to have been called in board-specific OS logic
 *   prior to calling this function from application logic (perhaps via
 *   boardctl(BOARDIOC_OS_SYMTAB).  Otherwise, insmod will be unable to
 *   resolve symbols in the OS module.
 *
 * Input Parameters:
 *
 *   filename   - Full path to the module binary to be loaded
 *   modulename - The name that can be used to refer to the module after
 *     it has been loaded.
 *
 * Returned Value:
 *   A non-NULL module handle that can be used on subsequent calls to other
 *   module interfaces is returned on success.  If insmod() was unable to
 *   load the module insmod() will return a NULL handle and the errno
 *   variable will be set appropriately.
 *
 ****************************************************************************/

FAR void *insmod(FAR const char *filename, FAR const char *modulename);

/****************************************************************************
 * Name: rmmod
 *
 * Description:
 *   Remove a previously installed module from memory.
 *
 * Input Parameters:
 *   handle - The module handler previously returned by insmod().
 *
 * Returned Value:
 *   Zero (OK) on success.  On any failure, -1 (ERROR) is returned the
 *   errno value is set appropriately.
 *
 ****************************************************************************/

int rmmod(FAR void *handle);

/****************************************************************************
 * Name: modsym
 *
 * Description:
 *   modsym() returns the address of a symbol defined within the object that
 *   was previously made accessible through a insmod() call.  handle is the
 *   value returned from a call to insmod() (and which has not since been
 *   released via a call to rmmod()), name is the symbol's name as a
 *   character string.
 *
 *   The returned symbol address will remain valid until rmmod() is called.
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to insmod().
 *   name   - A pointer to the symbol name string.
 *
 * Returned Value:
 *   The address associated with the symbol is returned on success.
 *   If handle does not refer to a valid module opened by insmod(), or if
 *   the named symbol cannot be found within any of the objects associated
 *   with handle, modsym() will return NULL and the errno variable will be
 *   set appropriately.
 *
 ****************************************************************************/

FAR const void *modsym(FAR void *handle, FAR const char *name);

/****************************************************************************
 * Name: modhandle
 *
 * Description:
 *   modhandle() returns the module handle for the installed module with the
 *   provided name.  A secondary use of this function is to determin if a
 *   module has been loaded or not.
 *
 * Input Parameters:
 *   name   - A pointer to the module name string.
 *
 * Returned Value:
 *   The non-NULL module handle previously returned by insmod() is returned
 *   on success.  If no module with that name is installed, modhandle() will
 *   return a NULL handle and the errno variable will be set appropriately.
 *
 ****************************************************************************/

FAR void *modhandle(FAR const char *name);

/****************************************************************************
 * Name: mod_registry_foreach
 *
 * Description:
 *   Visit each module in the registry.  This is an internal OS interface and
 *   not available for use by applications.
 *
 * Input Parameters:
 *   callback - This callback function was be called for each entry in the
 *     registry.
 *   arg - This opaque argument will be passed to the callback function.
 *
 * Returned Value:
 *   This function normally returns zero (OK).  If, however, any callback
 *   function returns a non-zero value, the traversal will be terminated and
 *   that non-zero value will be returned.
 *
 * Assumptions:
 *   The caller does NOT hold the lock on the module registry.
 *
 ****************************************************************************/

#if defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT)
int mod_registry_foreach(mod_callback_t callback, FAR void *arg);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_MODULE_H */
