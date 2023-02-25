/****************************************************************************
 * include/sys/mman.h
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

#define PROT_NONE       0x0             /*        Page may not be accessed */
#define PROT_READ       (1 << 0)        /* Bit 0: Page may be read */
#define PROT_WRITE      (1 << 1)        /* Bit 1: Page may be written */
#define PROT_EXEC       (1 << 2)        /* Bit 2: Page may be executed */

/* mmpa() flags bits.  Only MAP_SHARED, MAP_PRIVATE, and MAP_FIXED are
 * specified by OpenGroup.org.  The rest are unique to NuttX or are
 * definitions used by other systems.
 */

/* Sharing types (most are not implemented). */

#define MAP_SHARED      (1 << 0)        /* Bit 0:  Share this mapping */
#define MAP_PRIVATE     (1 << 1)        /* Bit 1:  Create a private copy-on-write mapping */
#define MAP_TYPE        (3 << 0)        /*         Mask for type of mapping */
#define MAP_FIXED       (1 << 2)        /* Bit 2:  Map to specified address exactly */
#define MAP_FILE        (1 << 3)        /* Bit 3:  The mapping is backed by a file */
#define MAP_ANONYMOUS   (1 << 4)        /* Bit 4:  The mapping is not backed by any file */
#define MAP_ANON        MAP_ANONYMOUS   /*         Alias */

/* These are Linux-specific (none are implemented).  */

#define MAP_GROWSDOWN   (1 << 5)        /* Bit 5:  Used to stack allocations */
#define MAP_DENYWRITE   (1 << 6)        /* Bit 6:  Do not permit writes to file */
#define MAP_EXECUTABLE  (1 << 7)        /* Bit 7:  Mark it as an executable */
#define MAP_LOCKED      (1 << 8)        /* Bit 8:  Lock pages mapped into memory */
#define MAP_NORESERVE   (1 << 9)        /* Bit 9:  Do not reserve swap space for this mapping */
#define MAP_POPULATE    (1 << 10)       /* Bit 10: populate (prefault) page tables */
#define MAP_NONBLOCK    (1 << 11)       /* Bit 11: Do not block on IO */

/* Failure return */

#define MAP_FAILED      ((FAR void*)-1)

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

#define MADV_NORMAL           POSIX_MADV_NORMAL
#define MADV_SEQUENTIAL       POSIX_MADV_SEQUENTIAL
#define MADV_RANDOM           POSIX_MADV_RANDOM
#define MADV_WILLNEED         POSIX_MADV_WILLNEED
#define MADV_DONTNEED         POSIX_MADV_DONTNEED

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

/* Flags for memfd_create(). */

#define MFD_CLOEXEC       O_CLOEXEC
#define MFD_ALLOW_SEALING (1 << 16)
#define MFD_HUGETLB       (1 << 17)

#define MFD_HUGE_SHIFT    26
#define MFD_HUGE_MASK     0x3f
#define MFD_HUGE_64KB     (16 << MFD_HUGE_SHIFT)
#define MFD_HUGE_512KB    (19 << MFD_HUGE_SHIFT)
#define MFD_HUGE_1MB      (20 << MFD_HUGE_SHIFT)
#define MFD_HUGE_2MB      (21 << MFD_HUGE_SHIFT)
#define MFD_HUGE_8MB      (23 << MFD_HUGE_SHIFT)
#define MFD_HUGE_16MB     (24 << MFD_HUGE_SHIFT)
#define MFD_HUGE_32MB     (25 << MFD_HUGE_SHIFT)
#define MFD_HUGE_256MB    (28 << MFD_HUGE_SHIFT)
#define MFD_HUGE_512MB    (29 << MFD_HUGE_SHIFT)
#define MFD_HUGE_1GB      (30 << MFD_HUGE_SHIFT)
#define MFD_HUGE_2GB      (31 << MFD_HUGE_SHIFT)
#define MFD_HUGE_16GB     (34 << MFD_HUGE_SHIFT)

#if defined(CONFIG_FS_LARGEFILE)
#  define mmap64 mmap
#endif

#define madvise posix_madvise

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

int munmap(FAR void *start, size_t length);

int posix_madvise(FAR void *addr, size_t len, int advice);
int posix_mem_offset(FAR const void *addr, size_t len, FAR off_t *off,
                     FAR size_t *contig_len, FAR int *fildes);
int posix_typed_mem_get_info(int fildes,
                             FAR struct posix_typed_mem_info *info);
int posix_typed_mem_open(FAR const char *name, int oflag, int tflag);
int shm_open(FAR const char *name, int oflag, mode_t mode);
int shm_unlink(FAR const char *name);
int memfd_create(FAR const char *name, unsigned int flags);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_MMAN_H */
