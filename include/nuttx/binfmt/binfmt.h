/****************************************************************************
 * include/nuttx/binfmt/binfmt.h
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

#ifndef __INCLUDE_NUTTX_BINFMT_BINFMT_H
#define __INCLUDE_NUTTX_BINFMT_BINFMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <spawn.h>

#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/streams.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BINFMT_NALLOC 4

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The type of one C++ constructor or destructor */

typedef FAR void (*binfmt_ctor_t)(void);
typedef FAR void (*binfmt_dtor_t)(void);

/* This describes the file to be loaded.
 *
 * NOTE 1: The 'filename' must be the full, absolute path to the file to be
 * executed unless CONFIG_LIBC_ENVPATH is defined.  In that case,
 * 'filename' may be a relative path; a set of candidate absolute paths
 * will be generated using the PATH environment variable and load_module()
 * will attempt to load each file that is found at those absolute paths.
 */

struct symtab_s;
struct binary_s
{
  /* Information provided from the loader (if successful) describing the
   * resources used by the loaded module.
   */

  main_t entrypt;                      /* Entry point into a program module */
  FAR void *mapped;                    /* Memory-mapped, address space */
  FAR void *alloc[BINFMT_NALLOC];      /* Allocated address spaces */

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  /* Constructors/destructors */

  FAR binfmt_ctor_t *ctors;            /* Pointer to a list of constructors */
  FAR binfmt_dtor_t *dtors;            /* Pointer to a list of destructors */
  uint16_t nctors;                     /* Number of constructors in the list */
  uint16_t ndtors;                     /* Number of destructors in the list */
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Address environment.
   *
   * addrenv - This is the handle created by up_addrenv_create() that can be
   *   used to manage the tasks address space.
   */

  group_addrenv_t addrenv;             /* Task group address environment */
#endif

  size_t mapsize;                      /* Size of the mapped address region (needed for munmap) */

  /* Start-up information that is provided by the loader, but may be modified
   * by the caller between load_module() and exec_module() calls.
   */

  uint8_t priority;                    /* Task execution priority */
  size_t stacksize;                    /* Size of the stack in bytes (unallocated) */

  /* Unload module callback */

  CODE int (*unload)(FAR struct binary_s *bin);
};

/* This describes binfmt coredump filed */

struct memory_region_s
{
  uintptr_t start;   /* Start address of this region */
  uintptr_t end;     /* End address of this region */
  uint32_t  flags;   /* Figure 5-3: Segment Flag Bits: PF_[X|W|R] */
};

/* This describes one binary format handler */

struct binfmt_s
{
  /* Supports a singly-linked list */

  FAR struct binfmt_s *next;

  /* Verify and load binary into memory */

  CODE int (*load)(FAR struct binary_s *bin,
                   FAR const char *filename,
                   FAR const struct symtab_s *exports,
                   int nexports);

  /* Unload module callback */

  CODE int (*unload)(FAR struct binary_s *bin);

  /* Unload module callback */

  CODE int (*coredump)(FAR struct memory_region_s *regions,
                       FAR struct lib_outstream_s *stream);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: binfmt_initialize
 *
 * Description:
 *   initialize binfmt subsystem
 *
 ****************************************************************************/

void binfmt_initialize(void);

/****************************************************************************
 * Name: register_binfmt
 *
 * Description:
 *   Register a loader for a binary format
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int register_binfmt(FAR struct binfmt_s *binfmt);

/****************************************************************************
 * Name: unregister_binfmt
 *
 * Description:
 *   Register a loader for a binary format
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int unregister_binfmt(FAR struct binfmt_s *binfmt);

/****************************************************************************
 * Name: core_dump
 *
 * Description:
 *   This function for generating core dump stream.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int core_dump(FAR struct memory_region_s *regions,
              FAR struct lib_outstream_s *stream);

/****************************************************************************
 * Name: load_module
 *
 * Description:
 *   Load a module into memory, bind it to an exported symbol take, and
 *   prep the module for execution.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int load_module(FAR struct binary_s *bin, FAR const char *filename,
                FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: unload_module
 *
 * Description:
 *   Unload a (non-executing) module from memory.  If the module has
 *   been started (via exec_module) and has not exited, calling this will
 *   be fatal.
 *
 *   However, this function must be called after the module exist.  How
 *   this is done is up to your logic.  Perhaps you register it to be
 *   called by on_exit()?
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int unload_module(FAR struct binary_s *bin);

/****************************************************************************
 * Name: exec_module
 *
 * Description:
 *   Execute a module that has been loaded into memory by load_module().
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int exec_module(FAR const struct binary_s *binp,
                FAR const char *filename, FAR char * const *argv,
                FAR char * const *envp);

/****************************************************************************
 * Name: exec
 *
 * Description:
 *   This is a convenience function that wraps load_ and exec_module into
 *   one call.  If CONFIG_BINFMT_LOADABLE is defined, this function will
 *   schedule to unload the module when task exits.
 *
 *   This non-standard, NuttX function is similar to execv() and
 *   posix_spawn() but differs in the following ways;
 *
 *   - Unlike execv() and posix_spawn() this function accepts symbol table
 *     information as input parameters. This means that the symbol table
 *     used to link the application prior to execution is provided by the
 *     caller, not by the system.
 *   - Unlike execv(), this function always returns.
 *
 *   This non-standard interface is included as a official NuttX API only
 *   because it is needed in certain build modes: exec() is probably the
 *   only want to load programs in the PROTECTED mode. Other file execution
 *   APIs rely on a symbol table provided by the OS. In the PROTECTED build
 *   mode, the OS cannot provide any meaningful symbolic information for
 *   execution of code in the user-space blob so that is the exec() function
 *   is really needed in that build case
 *
 *   The interface is available in the FLAT build mode although it is not
 *   really necessary in that case. It is currently used by some example
 *   code under the apps/ that that generate their own symbol tables for
 *   linking test programs. So although it is not necessary, it can still
 *   be useful.
 *
 *   The interface would be completely useless and will not be supported in
 *   in the KERNEL build mode where the contrary is true: An application
 *   process cannot provide any meaning symbolic information for use in
 *   linking a different process.
 *
 *   NOTE: This function is flawed and useless without CONFIG_BINFMT_LOADABLE
 *   because without that features there is then no mechanism to unload the
 *   module once it exits.
 *
 * Input Parameters:
 *   filename - The path to the program to be executed. If
 *              CONFIG_LIBC_ENVPATH is defined in the configuration, then
 *              this may be a relative path from the current working
 *              directory. Otherwise, path must be the absolute path to the
 *              program.
 *   argv     - A pointer to an array of string arguments. The end of the
 *              array is indicated with a NULL entry.
 *   envp     - An array of character pointers to null-terminated strings
 *              that provide the environment for the new process image.
 *              The environment array is terminated by a null pointer.
 *   exports  - The address of the start of the caller-provided symbol
 *              table. This symbol table contains the addresses of symbols
 *              exported by the caller and made available for linking the
 *              module into the system.
 *   nexports - The number of symbols in the exports table.
 *
 * Returned Value:
 *   It returns the PID of the exec'ed module.  On failure, it returns
 *   the negative errno value appropriately.
 *
 ****************************************************************************/

int exec(FAR const char *filename, FAR char * const *argv,
         FAR char * const *envp, FAR const struct symtab_s *exports,
         int nexports);

/****************************************************************************
 * Name: exec_spawn
 *
 * Description:
 *   exec() configurable version, delivery the spawn attribute if this
 *   process has special customization.
 *
 * Input Parameters:
 *   filename - The path to the program to be executed. If
 *              CONFIG_LIBC_ENVPATH is defined in the configuration, then
 *              this may be a relative path from the current working
 *              directory. Otherwise, path must be the absolute path to the
 *              program.
 *   argv     - A pointer to an array of string arguments. The end of the
 *              array is indicated with a NULL entry.
 *   envp     - A pointer to an array of environment strings. Terminated with
 *              a NULL entry.
 *   exports  - The address of the start of the caller-provided symbol
 *              table. This symbol table contains the addresses of symbols
 *              exported by the caller and made available for linking the
 *              module into the system.
 *   nexports - The number of symbols in the exports table.
 *   attr     - The spawn attributes.
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   It returns the PID of the exec'ed module.  On failure, it returns
 *   -1 (ERROR) and sets errno appropriately.
 *
 ****************************************************************************/

int exec_spawn(FAR const char *filename, FAR char * const *argv,
               FAR char * const *envp, FAR const struct symtab_s *exports,
               int nexports, FAR const posix_spawnattr_t *attr);

/****************************************************************************
 * Name: binfmt_exit
 *
 * Description:
 *   This function may be called when a tasked loaded into RAM exits.
 *   This function will unload the module when the task exits and reclaim
 *   all resources used by the module.
 *
 * Input Parameters:
 *   bin - This structure must have been allocated with kmm_malloc() and must
 *         persist until the task unloads
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BINFMT_LOADABLE
int binfmt_exit(FAR struct binary_s *bin);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_BINFMT_H */
