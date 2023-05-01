/****************************************************************************
 * libs/libc/dlfcn/lib_dlopen.c
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

#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <dlfcn.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/envpath.h>
#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dldump_loadinfo
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_BINFMT)
static void dldump_loadinfo(FAR struct mod_loadinfo_s *loadinfo)
{
  int i;

  binfo("LOAD_INFO:\n");
  binfo("  textalloc:    %08lx\n", (long)loadinfo->textalloc);
  binfo("  datastart:    %08lx\n", (long)loadinfo->datastart);
  binfo("  textsize:     %ld\n",   (long)loadinfo->textsize);
  binfo("  datasize:     %ld\n",   (long)loadinfo->datasize);
  binfo("  filelen:      %ld\n",   (long)loadinfo->filelen);
  binfo("  filfd:        %d\n",    loadinfo->filfd);
  binfo("  symtabidx:    %d\n",    loadinfo->symtabidx);
  binfo("  strtabidx:    %d\n",    loadinfo->strtabidx);

  binfo("ELF Header:\n");
  binfo("  e_ident:      %02x %02x %02x %02x\n",
    loadinfo->ehdr.e_ident[0], loadinfo->ehdr.e_ident[1],
    loadinfo->ehdr.e_ident[2], loadinfo->ehdr.e_ident[3]);
  binfo("  e_type:       %04x\n",  loadinfo->ehdr.e_type);
  binfo("  e_machine:    %04x\n",  loadinfo->ehdr.e_machine);
  binfo("  e_version:    %08x\n",  loadinfo->ehdr.e_version);
  binfo("  e_entry:      %08lx\n", (long)loadinfo->ehdr.e_entry);
  binfo("  e_phoff:      %d\n",    loadinfo->ehdr.e_phoff);
  binfo("  e_shoff:      %d\n",    loadinfo->ehdr.e_shoff);
  binfo("  e_flags:      %08x\n" , loadinfo->ehdr.e_flags);
  binfo("  e_ehsize:     %d\n",    loadinfo->ehdr.e_ehsize);
  binfo("  e_phentsize:  %d\n",    loadinfo->ehdr.e_phentsize);
  binfo("  e_phnum:      %d\n",    loadinfo->ehdr.e_phnum);
  binfo("  e_shentsize:  %d\n",    loadinfo->ehdr.e_shentsize);
  binfo("  e_shnum:      %d\n",    loadinfo->ehdr.e_shnum);
  binfo("  e_shstrndx:   %d\n",    loadinfo->ehdr.e_shstrndx);

  if (loadinfo->shdr && loadinfo->ehdr.e_shnum > 0)
    {
      for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
        {
          FAR Elf_Shdr *shdr = &loadinfo->shdr[i];
          binfo("Sections %d:\n", i);
          binfo("  sh_name:      %08x\n", shdr->sh_name);
          binfo("  sh_type:      %08x\n", shdr->sh_type);
          binfo("  sh_flags:     %08x\n", shdr->sh_flags);
          binfo("  sh_addr:      %08x\n", shdr->sh_addr);
          binfo("  sh_offset:    %d\n",   shdr->sh_offset);
          binfo("  sh_size:      %d\n",   shdr->sh_size);
          binfo("  sh_link:      %d\n",   shdr->sh_link);
          binfo("  sh_info:      %d\n",   shdr->sh_info);
          binfo("  sh_addralign: %d\n",   shdr->sh_addralign);
          binfo("  sh_entsize:   %d\n",   shdr->sh_entsize);
        }
    }
}
#else
# define dldump_loadinfo(i)
#endif
#endif

/****************************************************************************
 * Name: dldump_initializer
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
#ifdef CONFIG_MODLIB_DUMPBUFFER
static void dldump_initializer(mod_initializer_t initializer,
                               FAR struct mod_loadinfo_s *loadinfo)
{
  modlib_dumpbuffer("Initializer code", (FAR const uint8_t *)initializer,
                    MIN(loadinfo->textsize - loadinfo->ehdr.e_entry, 512));
}
#else
# define dldump_initializer(b,l)
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dlinsert
 *
 * Description:
 *   Verify that the file is an ELF module binary and, if so, load the
 *   shared library into user memory and initialize it for use.
 *
 *   NOTE: modlib_setsymtab() had to have been called by application logic
 *   logic prior to calling this.  Otherwise, dlinsert will be unable to
 *   resolve symbols in the OS module.
 *
 * Input Parameters:
 *   filename - Full path to the shared library file to be loaded
 *
 * Returned Value:
 *   A non-NULL module handle that can be used on subsequent calls to other
 *   shared library interfaces is returned on success.  If insmod() was
 *   unable to load the module insmod() will return a NULL handle and the
 *   errno variable will be set appropriately.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
/* The PROTECTED build is equivalent to the FLAT build EXCEPT that there
 * must be two copies of the module logic:  One residing in kernel
 * space and using the kernel symbol table and one residing in user space
 * using the user space symbol table.
 *
 * dlinsert() is essentially a clone of insmod().
 */

static inline FAR void *dlinsert(FAR const char *filename)
{
  struct mod_loadinfo_s loadinfo;
  FAR struct module_s *modp;
  mod_initializer_t initializer;
  int ret;

  binfo("Loading file: %s\n", filename);

  /* Get exclusive access to the module registry */

  modlib_registry_lock();

  /* Initialize the ELF library to load the program binary. */

  ret = modlib_initialize(filename, &loadinfo);
  dldump_loadinfo(&loadinfo);
  if (ret != 0)
    {
      serr("ERROR: Failed to initialize to load module: %d\n", ret);
      goto errout_with_lock;
    }

  /* Allocate a module registry entry to hold the module data */

  modp = (FAR struct module_s *)lib_zalloc(sizeof(struct module_s));
  if (ret != 0)
    {
      binfo("Failed to initialize for load of ELF program: %d\n", ret);
      goto errout_with_loadinfo;
    }

  /* Load the program binary */

  ret = modlib_load(&loadinfo);
  dldump_loadinfo(&loadinfo);
  if (ret != 0)
    {
      binfo("Failed to load ELF program binary: %d\n", ret);
      goto errout_with_registry_entry;
    }

  /* Bind the program to the kernel symbol table */

  ret = modlib_bind(modp, &loadinfo);
  if (ret != 0)
    {
      binfo("Failed to bind symbols program binary: %d\n", ret);
      goto errout_with_load;
    }

  /* Save the load information */

  modp->textalloc       = (FAR void *)loadinfo.textalloc;
  modp->dataalloc       = (FAR void *)loadinfo.datastart;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  modp->textsize    = loadinfo.textsize;
  modp->datasize    = loadinfo.datasize;
#endif

  /* Get the module initializer entry point */

  initializer = (mod_initializer_t)(loadinfo.textalloc +
                                    loadinfo.ehdr.e_entry);
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MODULE)
  modp->initializer = initializer;
#endif
  dldump_initializer(initializer, &loadinfo);

  /* Call the module initializer */

  ret = initializer(&modp->modinfo);
  if (ret < 0)
    {
      binfo("Failed to initialize the module: %d\n", ret);
      goto errout_with_load;
    }

  /* Add the new module entry to the registry */

  modlib_registry_add(modp);

  modlib_uninitialize(&loadinfo);
  modlib_registry_unlock();
  return (FAR void *)modp;

errout_with_load:
  modlib_unload(&loadinfo);
  modlib_undepend(modp);
errout_with_registry_entry:
  lib_free(modp);
errout_with_loadinfo:
  modlib_uninitialize(&loadinfo);
errout_with_lock:
  modlib_registry_unlock();
  set_errno(-ret);
  return NULL;
}
#elif defined(CONFIG_BUILD_FLAT)
/* In the FLAT build, a shared library is essentially the same as a kernel
 * module.
 *
 * REVISIT:  Missing functionality:
 * - No automatic binding of symbols
 * - No dependencies
 * - mode is ignored.
 */

static inline FAR void *dlinsert(FAR const char *filename)
{
  FAR void *handle;
  FAR char *name;

  DEBUGASSERT(filename != NULL);

  name = strdup(filename);
  if (name == NULL)
    {
      return NULL;
    }

  /* Then install the file using the basename of the file as the module
   * name.
   */

  handle = insmod(filename, basename(name));
  lib_free(name);
  return handle;
}
#else /* if defined(CONFIG_BUILD_KERNEL) */
/* The KERNEL build is considerably more complex:  In order to be shared,
 * the .text portion of the module must be (1) build for PIC/PID operation
 * and (2) must like in a shared memory region accessible from all
 * processes.  The .data/.bss portion of the module must be allocated in
 * the user space of each process, but must lie at the same virtual address
 * so that it can be referenced from the one copy of the text in the shared
 * memory region.
 */

static inline FAR void *dlinsert(FAR const char *filename)
{
  /* #warning Missing logic */

  return NULL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dlopen
 *
 * Description:
 *   dlopen() makes an executable object file specified by file available to
 *   the calling program.  The class of files eligible for this operation and
 *   the manner of their construction are specified by the implementation,
 *   though typically such files are executable objects such as shared
 *   libraries, relocatable files or programs. Note that some implementations
 *   permit the construction of dependencies between such objects that are
 *   embedded within files. In such cases, a dlopen() operation will load
 *   such dependencies in addition to the object referenced by file.
 *   Implementations may also impose specific constraints on the construction
 *   of programs that can employ dlopen() and its related services.
 *
 *   If a file is specified in multiple dlopen() invocations, mode is
 *   interpreted at each invocation. Note, however, that once RTLD_NOW has
 *   been specified all relocations will have been completed rendering
 *   further RTLD_NOW operations redundant and any further RTLD_LAZY
 *   operations irrelevant.  Similarly note that once RTLD_GLOBAL has been
 *   specified the object will maintain the RTLD_GLOBAL status regardless
 *   of any previous or future specification of RTLD_LOCAL, so long as the
 *   object remains in the address space (see dlclose()).
 *
 *   Symbols introduced into a program through calls to dlopen() may be
 *   used in relocation activities. Symbols so introduced may duplicate
 *   symbols already defined by the program or previous dlopen()
 *   operations. To resolve the ambiguities such a situation might
 *   present, the resolution of a symbol reference to symbol definition is
 *   based on a symbol resolution order. Two such resolution orders are
 *   defined: load or dependency ordering. Load order establishes an
 *   ordering among symbol definitions, such that the definition first
 *   loaded (including definitions from the image file and any dependent
 *   objects loaded with it) has priority over objects added later (via
 *   dlopen()). Load ordering is used in relocation processing. Dependency
 *   ordering uses a breadth-first order starting with a given object,
 *   then all of its dependencies, then any dependents of those, iterating
 *   until all dependencies are satisfied. With the exception of the global
 *   symbol object obtained via a dlopen() operation on a file of 0,
 *   dependency ordering is used by the dlsym() function. Load ordering is
 *   used in dlsym() operations upon the global symbol object.
 *
 *   When an object is first made accessible via dlopen() it and its
 *   dependent objects are added in dependency order. Once all the objects
 *   are added, relocations are performed using load order. Note that if an
 *   object or its dependencies had been previously loaded, the load and
 *   dependency orders may yield different resolutions.
 *
 *   The symbols introduced by dlopen() operations, and available through
 *   dlsym() are at a minimum those which are exported as symbols of global
 *   scope by the object. Typically such symbols will be those that were
 *   specified in (for example) C source code as having extern linkage. The
 *   precise manner in which an implementation constructs the set of
 *   exported symbols for a dlopen() object is specified by that
 *   implementation.
 *
 * Input Parameters:
 *   file - Used to construct a pathname to the object file. If file
 *          contains a slash character, the file argument is used as the
 *          pathname for the file. Otherwise, file is used in an
 *          implementation-dependent manner to yield a pathname.
 *
 *          If the value of file is 0, dlopen() provides a handle on a
 *          global symbol object. This object provides access to the symbols
 *          from an ordered set of objects consisting of the original
 *          program image file, together with any objects loaded at program
 *          startup as specified by that process image file (for example,
 *          shared libraries), and the set of objects loaded using a
 *          dlopen() operation together with the RTLD_GLOBAL flag. As the
 *          latter set of objects can change during execution, the set
 *          identified by handle can also change dynamically.
 *
 *          Only a single copy of an object file is brought into the address
 *          space, even if dlopen() is invoked multiple times in reference
 *          to the file, and even if different pathnames are used to
 *          reference the file.
 *  mode  - Describes how dlopen() will operate upon file with respect to
 *          the processing of relocations and the scope of visibility of the
 *          symbols provided within file. When an object is brought into the
 *          address space of a process, it may contain references to symbols
 *          whose addresses are not known until the object is loaded. These
 *          references must be relocated before the symbols can be accessed.
 *          The mode parameter governs when these relocations take place.
 *          See definitions above for values of the mode parameter:.
 *
 * Returned Value:
 *   A successful dlopen() returns a handle which the caller may use on
 *   subsequent calls to dlsym() and dlclose(). The value of this handle
 *   should not be interpreted in any way by the caller.
 *
 *   If file cannot be found, cannot be opened for reading, is not of an
 *   appropriate object format for processing by dlopen(), or if an error
 *   occurs during the process of loading file or relocating its symbolic
 *   references, dlopen() will return NULL. More detailed diagnostic
 *   information will be available through dlerror().
 *
 * Reference: OpenGroup.org
 *
 ****************************************************************************/

FAR void *dlopen(FAR const char *file, int mode)
{
  FAR void *handle = NULL;

#ifdef CONFIG_LIBC_ENVPATH
  if (file[0] != '/')
    {
      FAR const char *relpath;
      FAR char *fullpath;
      ENVPATH_HANDLE env;

      /* Set aside the relative path */

      relpath = file;

      /* Initialize to traverse the LD_LIBRARY_PATH variable */

      env = envpath_init("LD_LIBRARY_PATH");
      if (env)
        {
          /* Get the next absolute file path */

          while ((fullpath = envpath_next(env, relpath)) != NULL)
            {
              /* Try to load the file at this path */

              handle = dlinsert(fullpath);

              /* Free the allocated fullpath */

              lib_free(fullpath);

              /* Break out of the loop with handle != NULL on success */

              if (handle != NULL)
                {
                  break;
                }
            }

          /* Release the traversal handle */

          envpath_release(env);
        }
    }
  else
#endif
    {
      /* We already have the one and only absolute path to the file to
       * be loaded.
       */

       handle = dlinsert(file);
    }

  return handle;
}
