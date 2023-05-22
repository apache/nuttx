/****************************************************************************
 * include/nuttx/lib/modlib.h
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

#ifndef __INCLUDE_NUTTX_LIB_MODLIB_H
#define __INCLUDE_NUTTX_LIB_MODLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <elf.h>

#include <nuttx/arch.h>
#include <nuttx/symtab.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_MODLIB_MAXDEPEND
#  define CONFIG_MODLIB_MAXDEPEND  0
#endif

#ifndef CONFIG_MODLIB_ALIGN_LOG2
#  define CONFIG_MODLIB_ALIGN_LOG2 2
#endif

#ifndef CONFIG_MODLIB_BUFFERSIZE
#  define CONFIG_MODLIB_BUFFERSIZE 32
#endif

#ifndef CONFIG_MODLIB_BUFFERINCR
#  define CONFIG_MODLIB_BUFFERINCR 32
#endif

/* CONFIG_DEBUG_INFO, and CONFIG_DEBUG_BINFMT have to be defined or
 * CONFIG_MODLIB_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_INFO) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_MODLIB_DUMPBUFFER
#endif

#ifdef CONFIG_MODLIB_DUMPBUFFER
#  define modlib_dumpbuffer(m,b,n) sinfodumpbuffer(m,b,n)
#else
#  define modlib_dumpbuffer(m,b,n)
#endif

/* Module names.  These are only used by the kernel module and will be
 * disabled in all other configurations.
 *
 * FLAT build:  There are only kernel modules and kernel modules
 *   masquerading as shared libraries.  Names are required.
 * PROTECTED and KERNEL builds:  Names are only needed in the kernel
 *   portion of the build
 */

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
#  define HAVE_MODLIB_NAMES
#  define MODLIB_NAMEMAX 16
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the function that is called to uninitialize the
 * the loaded module.  This may mean, for example, un-registering a device
 * driver. If the module is successfully uninitialized, its memory will be
 * deallocated.
 *
 * Input Parameters:
 *   arg - An opaque argument that was previously returned by the initializer
 *         function.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure to
 *   uninitialize the module.  If zero is returned, then the module memory
 *   will be deallocated.  If the module is still in use (for example with
 *   open driver instances), the uninitialization function should fail with
 *   -EBUSY
 */

typedef CODE int (*mod_uninitializer_t)(FAR void *arg);

/* The content of this structure is filled by module_initialize().
 *
 *   uninitializer - The pointer to the uninitialization function.  NULL may
 *                   be specified if no uninitialization is needed (i.e, the
 *                   the module memory can be deallocated at any time).
 *   arg           - An argument that will be passed to the uninitialization
 *                   function.
 *   exports       - A symbol table exported by the module
 *   nexports      - The number of symbols in the exported symbol table.
 */

struct mod_info_s
{
  mod_uninitializer_t uninitializer;   /* Module uninitializer */
  FAR void *arg;                       /* Uninitializer argument */
  FAR const struct symtab_s *exports;  /* Symbols exported by module */
  unsigned int nexports;               /* Number of symbols in exports list */
};

/* A NuttX module is expected to export a function called module_initialize()
 * that has the following function prototype.  This function should appear as
 * the entry point in the ELF module file and will be called by the loader
 * logic after the module has been loaded into memory.
 *
 * Input Parameters:
 *   modinfo - Module information to be filled by the initializer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure to
 *   initialize the module.
 */

typedef CODE int (*mod_initializer_t)(FAR struct mod_info_s *modinfo);

/* This is the type of the callback function used by
 * modlib_registry_foreach()
 */

struct module_s;
typedef CODE int (*mod_callback_t)(FAR struct module_s *modp, FAR void *arg);

/* This describes the file to be loaded. */

struct module_s
{
  FAR struct module_s *flink;          /* Supports a singly linked list */
#ifdef HAVE_MODLIB_NAMES
  FAR char modname[MODLIB_NAMEMAX];    /* Module name */
#endif
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  mod_initializer_t initializer;       /* Module initializer function */
#endif
  struct mod_info_s modinfo;           /* Module information */
  FAR void *textalloc;                 /* Allocated kernel text memory */
  FAR void *dataalloc;                 /* Allocated kernel memory */
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  size_t textsize;                     /* Size of the kernel .text memory allocation */
  size_t datasize;                     /* Size of the kernel .bss/.data memory allocation */
#endif

#if CONFIG_MODLIB_MAXDEPEND > 0
  uint8_t dependents;                  /* Number of modules that depend on this module */

  /* This is an upacked array of pointers to other modules that this module
   * depends upon.
   */

  FAR struct module_s *dependencies[CONFIG_MODLIB_MAXDEPEND];
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
   * The alloc[] array in struct module_s will hold memory that persists
   * after the module has been loaded.
   */

  uintptr_t         textalloc;   /* .text memory allocated when module was loaded */
  uintptr_t         datastart;   /* Start of.bss/.data memory in .text allocation */
  size_t            textsize;    /* Size of the module .text memory allocation */
  size_t            datasize;    /* Size of the module .bss/.data memory allocation */
  size_t            textalign;   /* Necessary alignment of .text */
  size_t            dataalign;   /* Necessary alignment of .bss/.text */
  off_t             filelen;     /* Length of the entire module file */
  Elf_Ehdr          ehdr;        /* Buffered module file header */
  FAR Elf_Shdr     *shdr;        /* Buffered module section headers */
  uint8_t          *iobuffer;    /* File I/O buffer */

  uint16_t          symtabidx;   /* Symbol table section index */
  uint16_t          strtabidx;   /* String table section index */
  uint16_t          buflen;      /* size of iobuffer[] */
  int               filfd;       /* Descriptor for the file being loaded */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_initialize
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

int modlib_initialize(FAR const char *filename,
                      FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_uninitialize
 *
 * Description:
 *   Releases any resources committed by modlib_initialize().  This
 *   essentially undoes the actions of modlib_initialize.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_uninitialize(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_getsymtab
 *
 * Description:
 *   Get the current symbol table selection as an atomic operation.
 *
 * Input Parameters:
 *   symtab - The location to store the symbol table.
 *   nsymbols - The location to store the number of symbols in the symbol
 *              table.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void modlib_getsymtab(FAR const struct symtab_s **symtab, FAR int *nsymbols);

/****************************************************************************
 * Name: modlib_setsymtab
 *
 * Description:
 *   Select a new symbol table selection as an atomic operation.
 *
 * Input Parameters:
 *   symtab - The new symbol table.
 *   nsymbols - The number of symbols in the symbol table.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void modlib_setsymtab(FAR const struct symtab_s *symtab, int nsymbols);

/****************************************************************************
 * Name: modlib_load
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

int modlib_load(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by
 *   modlib_setsymtab().
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_bind(FAR struct module_s *modp,
                FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of modlib_load.  It is called only under certain error
 *   conditions after the module has been loaded but not yet started.
 *
 * Input Parameters:
 *   modp     - Module state information
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_unload(struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: modlib_depend
 *
 * Description:
 *   Set up module dependencies between the exporter and the importer of a
 *   symbol.  The happens when the module is installed via insmod and a
 *   symbol is imported from another module.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 * Assumptions:
 *   Caller holds the registry lock.
 *
 ****************************************************************************/

#if CONFIG_MODLIB_MAXDEPEND > 0
int modlib_depend(FAR struct module_s *importer,
                  FAR struct module_s *exporter);
#endif

/****************************************************************************
 * Name: modlib_undepend
 *
 * Description:
 *   Tear down module dependencies between the exporters and the importer of
 *   symbols.  This happens when the module is removed via rmmod (and on
 *   error handling cases in insmod).
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 * Assumptions:
 *   Caller holds the registry lock.
 *
 ****************************************************************************/

#if CONFIG_MODLIB_MAXDEPEND > 0
int modlib_undepend(FAR struct module_s *importer);
#endif

/****************************************************************************
 * Name: modlib_read
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

int modlib_read(FAR struct mod_loadinfo_s *loadinfo, FAR uint8_t *buffer,
                size_t readsize, off_t offset);

/****************************************************************************
 * Name: modlib_registry_lock
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

void modlib_registry_lock(void);

/****************************************************************************
 * Name: modlib_registry_unlock
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

void modlib_registry_unlock(void);

/****************************************************************************
 * Name: modlib_registry_add
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

void modlib_registry_add(FAR struct module_s *modp);

/****************************************************************************
 * Name: modlib_registry_del
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

int modlib_registry_del(FAR struct module_s *modp);

/****************************************************************************
 * Name: modlib_registry_find
 *
 * Description:
 *   Find an entry in the module registry using the name of the module.
 *
 * Input Parameters:
 *   modname - The name of the module to be found
 *
 * Returned Value:
 *   If the registry entry is found, a pointer to the module entry is
 *   returned.  NULL is returned if the entry is not found.
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

#ifdef HAVE_MODLIB_NAMES
FAR struct module_s *modlib_registry_find(FAR const char *modname);
#endif

/****************************************************************************
 * Name: modlib_registry_verify
 *
 * Description:
 *   Verify that a module handle is valid by traversing the module list and
 *   assuring that the module still resides in the list.  If it does not,
 *   the handle is probably a stale pointer.
 *
 * Input Parameters:
 *   modp - The registry entry to be verified.
 *
 * Returned Value:
 *   Returns OK is the module is valid; -ENOENT otherwise.
 *
 * Assumptions:
 *   The caller holds the lock on the module registry.
 *
 ****************************************************************************/

int modlib_registry_verify(FAR struct module_s *modp);

/****************************************************************************
 * Name: modlib_registry_foreach
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

int modlib_registry_foreach(mod_callback_t callback, FAR void *arg);

#endif /* __INCLUDE_NUTTX_LIB_MODLIB_H */
