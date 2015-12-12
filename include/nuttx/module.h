/****************************************************************************
 * include/nuttx/module.h
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

/* A NuttX module is expected to export a function called module_initialize()
 * that has the following function prototype.  This function should appear as
 * the entry point in the ELF module file and will be called bythe binfmt
 * logic after the module has been loaded into kernel memory.
 *
 * Input Parameters:
 *   uninitializer - The pointer to the uninitialization function.  NULL may
 *     be returned if no uninitialization is needed (i.e, the the module
 *     memory can be deallocated at any time).
 *   arg - An argument that will be passed to the uninitialization function.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure to
 *   initialize the module.
 */

typedef CODE int (*mod_initializer_t)(mod_uninitializer_t *uninitializer,
                                      FAR void **arg);

/* This describes the file to be loaded. */

struct symtab_s;
struct module_s
{
  /* Information provided to insmod by the caller */

  FAR const char *filename;            /* Full path to the binary to be loaded */
  FAR const struct symtab_s *exports;  /* Table of exported symbols */
  int nexports;                        /* The number of symbols in exports[] */

  /* Information provided from insmod (if successful) describing the
   * resources used by the loaded module.
   */

  mod_uninitializer_t uninitializer;   /* Module uninitializer function */
  FAR void *arg;                       /* Uninitializer argument */
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
 * Name: rmmod
 *
 * Description:
 *   Remove a previously installed module from memory.
 *
 ****************************************************************************/

int rmmod(FAR struct module_s *modp);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_MODULE_H */
