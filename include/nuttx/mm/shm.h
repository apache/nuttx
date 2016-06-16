/****************************************************************************
 * include/nuttx/mm/shm.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_MM_SHM_H
#define __INCLUDE_NUTTX_MM_SHM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/mm/gran.h>

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_ADDRENV
#  error CONFIG_ARCH_ADDRENV must be selected with CONFIG_MM_SHM
#endif

#ifndef CONFIG_BUILD_KERNEL
#  error CONFIG_BUILD_KERNEL must be selected with CONFIG_MM_SHM
#endif

#ifndef CONFIG_GRAN
#  error CONFIG_GRAN must be selected with CONFIG_MM_SHM
#endif

#ifdef CONFIG_GRAN_SINGLE
#  error CONFIG_GRAN_SINGLE must NOT be selected with CONFIG_MM_SHM
#endif

#ifndef CONFIG_MM_PGALLOC
#  error CONFIG_MM_PGALLOC must be selected with CONFIG_MM_SHM
#endif

/* Debug */

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG_SHM
#    define shmerr(format, ...)       _err(format, ##__VA_ARGS__)
#    define shminfo(format, ...)      _info(format, ##__VA_ARGS__)
#  else
#    define shmerr(format, ...)       merr(format, ##__VA_ARGS__)
#    define shminfo(format, ...)      minfo(format, ##__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG_SHM
#    define shmerr                    _err
#    define shminfo                   _info
#  else
#    define shmerr                    (void)
#    define shminfo                   (void)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the virtual page allocator that is use to manage
 * the mapping of shared memory into the group/process address space.
 */

struct group_shm_s
{
  /* Handle returned by gran_initialize() when the virtual page allocator
   * was created.
   */

  GRAN_HANDLE gs_handle;

  /* This array is used to do a reverse lookup:  Give the virtual address
   * of a shared memory region, find the region index that performs that
   * mapping.
   */

  uintptr_t gs_vaddr[CONFIG_ARCH_SHM_MAXREGIONS];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: shm_initialize
 *
 * Description:
 *   Perform one time, start-up initialization of the shared memor logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void shm_initialize(void);

/****************************************************************************
 * Name: shm_group_initialize
 *
 * Description:
 *   Initialize the group shared memory data structures when a new task
 *   group is initialized.
 *
 * Input Parameters:
 *   group - A reference to the new group structure to be initialized.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct task_group_s; /* Forward reference */
int shm_group_initialize(FAR struct task_group_s *group);

/****************************************************************************
 * Name: shm_group_release
 *
 * Description:
 *   Release resources used by the group shared memory logic.  This function
 *   is called at the time at the group is destroyed.
 *
 * Input Parameters:
 *   group - A reference to the group structure to be un-initialized.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

struct task_group_s; /* Forward reference */
void shm_group_release(FAR struct task_group_s *group);

#endif /* CONFIG_MM_SHM */
#endif /* __INCLUDE_NUTTX_MM_SHM_H */
