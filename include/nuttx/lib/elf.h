/****************************************************************************
 * include/nuttx/lib/elf.h
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

#ifndef __INCLUDE_NUTTX_LIB_LIBC_ELF_H
#define __INCLUDE_NUTTX_LIB_LIBC_ELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <elf.h>

#include <nuttx/addrenv.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_LIBC_ELF_MAXDEPEND
#  define CONFIG_LIBC_ELF_MAXDEPEND  0
#endif

#ifndef CONFIG_LIBC_ELF_ALIGN_LOG2
#  define CONFIG_LIBC_ELF_ALIGN_LOG2 2
#endif

#ifndef CONFIG_LIBC_ELF_BUFFERSIZE
#  define CONFIG_LIBC_ELF_BUFFERSIZE 32
#endif

#ifndef CONFIG_LIBC_ELF_BUFFERINCR
#  define CONFIG_LIBC_ELF_BUFFERINCR 32
#endif

/* CONFIG_DEBUG_INFO, and CONFIG_LIBC_ELF have to be defined or
 * CONFIG_LIBC_ELF_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_INFO) || !defined(CONFIG_LIBC_ELF)
#  undef CONFIG_LIBC_ELF_DUMPBUFFER
#endif

#ifdef CONFIG_LIBC_ELF_DUMPBUFFER
#  define libelf_dumpbuffer(m,b,n) sinfodumpbuffer(m,b,n)
#else
#  define libelf_dumpbuffer(m,b,n)
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
#  define HAVE_LIBC_ELF_NAMES
#  define LIBC_ELF_NAMEMAX NAME_MAX
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

struct symtab_s;
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
 * libelf_registry_foreach()
 */

struct module_s;
typedef CODE int (*mod_callback_t)(FAR struct module_s *modp, FAR void *arg);

/* This describes the file to be loaded. */

struct module_s
{
  FAR struct module_s *flink;          /* Supports a singly linked list */
#ifdef HAVE_LIBC_ELF_NAMES
  char modname[LIBC_ELF_NAMEMAX];        /* Module name */
#endif
  struct mod_info_s modinfo;           /* Module information */
  FAR void *textalloc;                 /* Allocated kernel text memory */
  FAR void *dataalloc;                 /* Allocated kernel memory */
  uintptr_t xipbase;                   /* if elf is position independent, and use
                                        * romfs/tmps, we can try get xipbase,
                                        * skip the copy.
                                        */
#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
  FAR void **sectalloc;                /* All sections memory allocated when ELF file was loaded */
  uint16_t nsect;                      /* Number of entries in sectalloc array */
#endif
  int dynamic;                         /* Module is a dynamic shared object */
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  size_t textsize;                     /* Size of the kernel .text memory allocation */
  size_t datasize;                     /* Size of the kernel .bss/.data memory allocation */
#endif

#if CONFIG_LIBC_ELF_MAXDEPEND > 0
  uint8_t dependents;                  /* Number of modules that depend on this module */

  /* This is an unpacked array of pointers to other modules that this module
   * depends upon.
   */

  FAR struct module_s *dependencies[CONFIG_LIBC_ELF_MAXDEPEND];
#endif
  uintptr_t initarr;                     /* .init_array */
  uint16_t  ninit;                       /* Number of entries in .init_array */
  uintptr_t finiarr;                     /* .fini_array */
  uint16_t  nfini;                       /* Number of entries in .fini_array */
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

#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
  FAR uintptr_t *sectalloc;  /* All sections memory allocated when ELF file was loaded */
#endif

  uintptr_t     textalloc;   /* .text memory allocated when module was loaded */
  uintptr_t     datastart;   /* Start of.bss/.data memory in .text allocation */
  size_t        textsize;    /* Size of the module .text memory allocation */
  size_t        datasize;    /* Size of the module .bss/.data memory allocation */
  size_t        textalign;   /* Necessary alignment of .text */
  size_t        dataalign;   /* Necessary alignment of .bss/.text */
  off_t         filelen;     /* Length of the entire module file */
  uid_t         fileuid;     /* Uid of the file system */
  gid_t         filegid;     /* Gid of the file system */
  int           filemode;    /* Mode of the file system */
  Elf_Ehdr      ehdr;        /* Buffered module file header */
  FAR Elf_Phdr *phdr;        /* Buffered module program headers */
  FAR Elf_Shdr *shdr;        /* Buffered module section headers */
  FAR void     *exported;    /* Module exports */
  FAR uint8_t  *iobuffer;    /* File I/O buffer */
  uintptr_t     datasec;     /* ET_DYN - data area start from Phdr */
  uintptr_t     segpad;      /* Padding between text and data */
  uintptr_t     initarr;     /* .init_array */
  uintptr_t     finiarr;     /* .fini_array */
  uintptr_t     preiarr;     /* .preinit_array */
  uint16_t      ninit;       /* Number of .init_array entries */
  uint16_t      nfini;       /* Number of .fini_array entries */
  uint16_t      nprei;       /* Number of .preinit_array entries */
  uint16_t      symtabidx;   /* Symbol table section index */
  uint16_t      strtabidx;   /* String table section index */
  uint16_t      dsymtabidx;  /* Dynamic symbol table section index */
  uint16_t      buflen;      /* size of iobuffer[] */
  int           filfd;       /* Descriptor for the file being loaded */
  int           nexports;    /* ET_DYN - Number of symbols exported */
  int           gotindex;    /* Index to the GOT section */
  uintptr_t     xipbase;     /* if elf is position independent, and use
                              * romfs/tmps, we can try get xipbase,
                              * skip the copy.
                              */

  /* Address environment.
   *
   * addrenv - This is the handle created by addrenv_allocate() that can be
   *   used to manage the tasks address space.
   */

#ifdef CONFIG_ARCH_ADDRENV
  FAR addrenv_t     *addrenv;    /* Address environment */
  FAR addrenv_t     *oldenv;     /* Saved address environment */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: libelf_initialize
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

int libelf_initialize(FAR const char *filename,
                      FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libelf_uninitialize
 *
 * Description:
 *   Releases any resources committed by libelf_initialize().  This
 *   essentially undoes the actions of libelf_initialize.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libelf_uninitialize(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libelf_getsymtab
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

void libelf_getsymtab(FAR const struct symtab_s **symtab, FAR int *nsymbols);

/****************************************************************************
 * Name: libelf_setsymtab
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

void libelf_setsymtab(FAR const struct symtab_s *symtab, int nsymbols);

/****************************************************************************
 * Name: libelf_load
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

int libelf_load(FAR struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libelf_load_with_addrenv
 *
 * Description:
 *   Loads the binary into memory, use the address environment to load the
 *   binary.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int libelf_load_with_addrenv(FAR struct mod_loadinfo_s *loadinfo);
#else
#  define libelf_load_with_addrenv(l) libelf_load(l)
#endif

/****************************************************************************
 * Name: libelf_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by
 *   libelf_setsymtab().
 *
 * Input Parameters:
 *   modp     - Module state information
 *   loadinfo - Load state information
 *   exports  - The table of exported symbols
 *   nexports - The number of symbols in the exports table
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int libelf_bind(FAR struct module_s *modp,
                FAR struct mod_loadinfo_s *loadinfo,
                FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: libelf_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of libelf_load.  It is called only under certain error
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

int libelf_unload(struct mod_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: libelf_depend
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

#if CONFIG_LIBC_ELF_MAXDEPEND > 0
int libelf_depend(FAR struct module_s *importer,
                  FAR struct module_s *exporter);
#endif

/****************************************************************************
 * Name: libelf_undepend
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

#if CONFIG_LIBC_ELF_MAXDEPEND > 0
int libelf_undepend(FAR struct module_s *importer);
#endif

/****************************************************************************
 * Name: libelf_read
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

int libelf_read(FAR struct mod_loadinfo_s *loadinfo, FAR uint8_t *buffer,
                size_t readsize, off_t offset);

/****************************************************************************
 * Name: libelf_findsection
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

int libelf_findsection(FAR struct mod_loadinfo_s *loadinfo,
                       FAR const char *sectname);

/****************************************************************************
 * Name: libelf_registry_lock
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

void libelf_registry_lock(void);

/****************************************************************************
 * Name: libelf_registry_unlock
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

void libelf_registry_unlock(void);

/****************************************************************************
 * Name: libelf_registry_add
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

void libelf_registry_add(FAR struct module_s *modp);

/****************************************************************************
 * Name: libelf_registry_del
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

int libelf_registry_del(FAR struct module_s *modp);

/****************************************************************************
 * Name: libelf_registry_find
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

#ifdef HAVE_LIBC_ELF_NAMES
FAR struct module_s *libelf_registry_find(FAR const char *modname);
#endif

/****************************************************************************
 * Name: libelf_registry_verify
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

int libelf_registry_verify(FAR struct module_s *modp);

/****************************************************************************
 * Name: libelf_registry_foreach
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

int libelf_registry_foreach(mod_callback_t callback, FAR void *arg);

/****************************************************************************
 * Name: libelf_freesymtab
 *
 * Description:
 *   Free a symbol table for the current module.
 *
 * Input Parameters:
 *   modp - Module state descriptor
 *
 ****************************************************************************/

void libelf_freesymtab(FAR struct module_s *modp);

/****************************************************************************
 * Name: libelf_dumploadinfo
 *
 * Description:
 *  Dump the load information to debug output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BINFMT_INFO
void libelf_dumploadinfo(FAR struct mod_loadinfo_s *loadinfo);
#else
#  define libelf_dumploadinfo(i)
#endif

/****************************************************************************
 * Name: libelf_dumpmodule
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BINFMT_INFO
void libelf_dumpmodule(FAR struct module_s *modp);
#else
#  define libelf_dumpmodule(m)
#endif

/****************************************************************************
 * Name: elf_dumpentrypt
 ****************************************************************************/

#ifdef CONFIG_LIBC_ELF_DUMPBUFFER
void libelf_dumpentrypt(FAR struct mod_loadinfo_s *loadinfo);
#else
#  define libelf_dumpentrypt(l)
#endif

/****************************************************************************
 * Name: libelf_insert
 *
 * Description:
 *   Verify that the file is an ELF module binary and, if so, load the
 *   module into kernel memory and initialize it for use.
 *
 *   NOTE: libelf_setsymtab() had to have been called in board-specific OS
 *   logic prior to calling this function from application logic (perhaps via
 *   boardctl(BOARDIOC_OS_SYMTAB).  Otherwise, insmod will be unable to
 *   resolve symbols in the OS module.
 *
 * Input Parameters:
 *
 *   filename - Full path to the module binary to be loaded
 *   modname  - The name that can be used to refer to the module after
 *     it has been loaded.
 *
 * Returned Value:
 *   A non-NULL module handle that can be used on subsequent calls to other
 *   module interfaces is returned on success.  If libelf_insert() was
 *   unable to load the module libelf_insert() will return a NULL handle
 *   and the errno variable will be set appropriately.
 *
 ****************************************************************************/

FAR void *libelf_insert(FAR const char *filename, FAR const char *modname);

/****************************************************************************
 * Name: libelf_getsymbol
 *
 * Description:
 *   libelf_getsymbol() returns the address of a symbol defined within the
 *   object that was previously made accessible through a libelf_getsymbol()
 *   call.  handle is the value returned from a call to libelf_insert() (and
 *   which has not since been released via a call to libelf_remove()),
 *   name is the symbol's name as a character string.
 *
 *   The returned symbol address will remain valid until libelf_remove() is
 *   called.
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to libelf_insert().
 *   name   - A pointer to the symbol name string.
 *
 * Returned Value:
 *   The address associated with the symbol is returned on success.
 *   If handle does not refer to a valid module opened by libelf_insert(),
 *   or if the named libelf_symbol cannot be found within any of the objects
 *   associated with handle, libelf_getsymbol() will return NULL and the
 *   errno variable will be set appropriately.
 *
 *   NOTE: This means that the address zero can never be a valid return
 *   value.
 *
 ****************************************************************************/

FAR const void *libelf_getsymbol(FAR void *handle, FAR const char *name);

/****************************************************************************
 * Name: libelf_uninit
 *
 * Description:
 *   Uninitialize module resources.
 *
 ****************************************************************************/

int libelf_uninit(FAR struct module_s *modp);

/****************************************************************************
 * Name: libelf_remove
 *
 * Description:
 *   Remove a previously installed module from memory.
 *
 * Input Parameters:
 *   handle - The module handler previously returned by libelf_insert().
 *
 * Returned Value:
 *   Zero (OK) on success.  On any failure, -1 (ERROR) is returned the
 *   errno value is set appropriately.
 *
 ****************************************************************************/

int libelf_remove(FAR void *handle);

/****************************************************************************
 * Name: libelf_modhandle
 *
 * Description:
 *   libelf_modhandle() returns the module handle for the installed
 *   module with the provided name.  A secondary use of this function is to
 *   determine if a module has been loaded or not.
 *
 * Input Parameters:
 *   name   - A pointer to the module name string.
 *
 * Returned Value:
 *   The non-NULL module handle previously returned by libelf_insert() is
 *   returned on success.  If no module with that name is installed,
 *   libelf_modhandle() will return a NULL handle and the errno variable
 *   will be set appropriately.
 *
 ****************************************************************************/

#ifdef HAVE_LIBC_ELF_NAMES
FAR void *libelf_gethandle(FAR const char *name);
#else
#  define libelf_gethandle(n) NULL
#endif

#endif /* __INCLUDE_NUTTX_LIB_LIBC_ELF_H */
