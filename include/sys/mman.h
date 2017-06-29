/****************************************************************************
 * include/sys/mman.h
 *
 *   Copyright (C) 2008, 2009, 2011, 2014 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_MMAN_H
#define __INCLUDE_SYS_MMAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Protections are chosen from these bits, OR'd together.  NuttX does not
 * yet support any of these, but are provided for source level compatibility
 */

#define PROT_NONE       0x0             /* Page may not be accessed */
#define PROT_READ       0x1             /* Page may be read */
#define PROT_WRITE      0x2             /* Page may be written */
#define PROT_EXEC       0x4             /* Page may be executed */

/* Sharing types -- ignored by NuttX. */

#define MAP_SHARED      0x00001         /* Share this mapping */
#define MAP_PRIVATE     0x00002         /* Create a private copy-on-write mapping */
#define MAP_TYPE        0x0000f         /* Mask for type of mapping */
#define MAP_FIXED       0x00010         /* Map to specified address exactly */
#define MAP_FILE        0x00000         /* The mapping is backed by a file */
#define MAP_ANONYMOUS   0x00020         /* The mapping is not backed by any file */
#define MAP_ANON        MAP_ANONYMOUS

/* These are Linux-specific.  */

#define MAP_GROWSDOWN   0x00100         /* Used to stack allocations */
#define MAP_DENYWRITE   0x00800         /* Do not permit writes to file */
#define MAP_EXECUTABLE  0x01000         /* Mark it as an executable */
#define MAP_LOCKED      0x02000         /* Lock pages mapped into memory */
#define MAP_NORESERVE   0x04000         /* Do not reserve swap space for this mapping */
#define MAP_POPULATE    0x08000         /* Populate (prefault) pagetables */
#define MAP_NONBLOCK    0x10000         /* Do not block on IO */

/* Failure return */

#define MAP_FAILED      ((void*)-1)

/* The following flags are used with msync() */

#define MS_ASYNC        0x01            /* Perform asynchronous writes */
#define MS_SYNC         0x02            /* Perform synchronous writes */
#define MS_INVALIDATE   0x04            /* Invalidate mapping */

/* The following flags are used with mlockall() */

#define MCL_CURRENT     0x01            /* Lock currently mapped pages */
#define MCL_FUTURE      0x02            /* Lock pages that become mapped */

/* If the Advisory Information and either the Memory Mapped Files or Shared
 * Memory Objects options are supported, values for advice used by
 * posix_madvise() are defined as follows:
 *
 * POSIX_MADV_NORMAL
 *   The application has no advice to give on its behavior with respect to
 *   the specified range. It is the default characteristic if no advice is
 *   given for a range of memory.
 * POSIX_MADV_SEQUENTIAL
 *   The application expects to access the specified range sequentially from
 *   lower addresses to higher addresses.
 * POSIX_MADV_RANDOM
 *   The application expects to access the specified range in a random order.
 * POSIX_MADV_WILLNEED
 *   The application expects to access the specified range in the near
 *   future.
 * POSIX_MADV_DONTNEED
 *   The application expects that it will not access the specified range in
 *   the near future.
 */

#define POSIX_MADV_NORMAL     (0)
#define POSIX_MADV_SEQUENTIAL (1)
#define POSIX_MADV_RANDOM     (2)
#define POSIX_MADV_WILLNEED   (3)
#define POSIX_MADV_DONTNEED   (4)

/* The following flags are defined for posix_typed_mem_open():
 *
 * POSIX_TYPED_MEM_ALLOCATE
 *    Allocate on mmap().
 * POSIX_TYPED_MEM_ALLOCATE_CONTIG
 *    Allocate contiguously on mmap().
 * POSIX_TYPED_MEM_MAP_ALLOCATABLE
 *    Map on mmap(), without affecting allocatability.
 */

#define POSIX_TYPED_MEM_ALLOCATE         (0)
#define POSIX_TYPED_MEM_ALLOCATE_CONTIG  (1)
#define POSIX_TYPED_MEM_MAP_ALLOCATABLE  (2)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct posix_typed_mem_info
{
  size_t posix_tmi_length;  /* Maximum length which may be allocated from a
                             * typed memory object */
};

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

int mlock(FAR const void *addr, size_t len);
int mlockall(int flags);
FAR void *mmap(FAR void *start, size_t length, int prot, int flags, int fd,
               off_t offset);
int mprotect(FAR void *addr, size_t len, int prot);
int msync(FAR void *addr, size_t len, int flags);
int munlock(FAR const void *addr, size_t len);
int munlockall(void);

#ifdef CONFIG_FS_RAMMAP
int munmap(FAR void *start, size_t length);
#else
#  define munmap(start, length)
#endif

int posix_madvise(FAR void *addr, size_t len, int advice);
int posix_mem_offset(FAR const void *addr, size_t len, FAR off_t *off,
                     FAR size_t *contig_len, FAR int *fildes);
int posix_typed_mem_get_info(int fildes, FAR struct posix_typed_mem_info *info);
int posix_typed_mem_open(FAR const char *name, int oflag, int tflag);
int shm_open(FAR const char *name, int oflag, mode_t mode);
int shm_unlink(FAR const char *name);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_MMAN_H */
