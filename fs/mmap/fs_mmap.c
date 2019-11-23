/****************************************************************************
 * fs/mmap/fs_mmap.c
 *
 *   Copyright (C) 2008-2009, 2011, 2019 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "inode/inode.h"
#include "fs_rammap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmap
 *
 * Description:
 *   NuttX operates in a flat open address space.  Therefore, it generally
 *   does not require mmap() functionality.  There are two exceptions:
 *
 *   1. mmap() is the API that is used to support direct access to random
 *     access media under the following very restrictive conditions:
 *
 *     a. The filesystem supports the FIOC_MMAP ioctl command.  Any file
 *        system that maps files contiguously on the media should support
 *        this ioctl. (vs. file system that scatter files over the media
 *        in non-contiguous sectors).  As of this writing, ROMFS is the
 *        only file system that meets this requirement.
 *     b. The underlying block driver supports the BIOC_XIPBASE ioctl
 *        command that maps the underlying media to a randomly accessible
 *        address. At  present, only the RAM/ROM disk driver does this.
 *
 *   2. If CONFIG_FS_RAMMAP is defined in the configuration, then mmap() will
 *      support simulation of memory mapped files by copying files whole
 *      into RAM.
 *
 * Input Parameters:
 *   start   A hint at where to map the memory -- ignored.  The address
 *           of the underlying media is fixed and cannot be re-mapped without
 *           MMU support.
 *   length  The length of the mapping.  For exception #1 above, this length
 *           ignored:  The entire underlying media is always accessible.
 *   prot    See the PROT_* definitions in sys/mman.h.
 *           PROT_NONE      - Will cause an error
 *           PROT_READ      - PROT_WRITE and PROT_EXEC also assumed
 *           PROT_WRITE     - PROT_READ and PROT_EXEC also assumed
 *           PROT_EXEC      - PROT_READ and PROT_WRITE also assumed
 *   flags   See the MAP_* definitions in sys/mman.h.
 *           MAP_SHARED     - MAP_PRIVATE or MAP_SHARED required
 *           MAP_PRIVATE    - MAP_PRIVATE or MAP_SHARED required
 *           MAP_FIXED      - Will cause an error
 *           MAP_FILE       - Ignored
 *           MAP_ANONYMOUS  - Optional
 *           MAP_ANON       - Will cause an error
 *           MAP_GROWSDOWN  - Ignored
 *           MAP_DENYWRITE  - Will cause an error
 *           MAP_EXECUTABLE - Ignored
 *           MAP_LOCKED     - Ignored
 *           MAP_NORESERVE  - Ignored
 *           MAP_POPULATE   - Ignored
 *           MAP_NONBLOCK   - Ignored
 *   fd      file descriptor of the backing file -- required.
 *   offset  The offset into the file to map
 *
 * Returned Value:
 *   On success, mmap() returns a pointer to the mapped area. On error, the
 *   value MAP_FAILED is returned, and errno is set appropriately.
 *
 *     ENOSYS
 *       Returned if any of the unsupported mmap() features are attempted
 *     EBADF
 *      'fd' is not a valid file descriptor.
 *     EINVAL
 *      Length is 0. flags contained neither MAP_PRIVATE or MAP_SHARED, or
 *      contained both of these values.
 *     ENODEV
 *      The underlying filesystem of the specified file does not support
 *      memory mapping.
 *
 ****************************************************************************/

FAR void *mmap(FAR void *start, size_t length, int prot, int flags,
               int fd, off_t offset)
{
  FAR void *addr;
  int errcode;
  int ret = -1;

  /* Since only a tiny subset of mmap() functionality, we have to verify many
   * things.
   */

#ifdef CONFIG_DEBUG_FEATURES
  /* Fixed mappings and protections are not currently supported.  These
   * options could be supported in the KERNEL build with an MMU, but that
   * logic is not in place.
   */

  if (prot == PROT_NONE ||
      (flags & (MAP_FIXED | MAP_DENYWRITE)) != 0)
    {
      ferr("ERROR: Unsupported options, prot=%x flags=%04x\n", prot, flags);
      errcode = ENOSYS;
      goto errout;
    }

  /* A length of 0 is invalid. */

  if (length == 0)
    {
      ferr("ERROR: Invalid length, length=%d\n", length);
      errcode = EINVAL;
      goto errout;
    }
#endif

  /* Check if we are just be asked to allocate memory, i.e., MAP_ANONYMOUS
   * set meaning that the memory is not backed up from a file.  The file
   * descriptor should be -1 (or refer to opened /dev/zero) in this case.
   * The file descriptor is ignored in either case.
   */

  if ((flags & MAP_ANONYMOUS) != 0)
    {
      FAR void *alloc;

      /* REVISIT:  Should reside outside of the heap.  That is really the
       * only purpose of MAP_ANONYMOUS:  To get non-heap memory.  In KERNEL
       * build, this could be accomplished using pgalloc(), provided that
       * you had logic in place to assign a virtual address to the mapping.
       */

      alloc = kumm_zalloc(length);
      if (alloc == NULL)
        {
          ferr("ERROR: kumm_alloc() failed: %d\n", ret);
          errcode = ENOMEM;
          goto errout;
        }

      return alloc;
    }

  /* Perform the ioctl to get the base address of the file in 'mapped'
   * in memory. (casting to uintptr_t first eliminates complaints on some
   * architectures where the sizeof long is different from the size of
   * a pointer).
   */

  if ((flags & MAP_PRIVATE) == 0)
    {
      ret = ioctl(fd, FIOC_MMAP, (unsigned long)((uintptr_t)&addr));
    }

  if (ret < 0)
    {
      /* Not directly mappable, probably because the underlying media does
       * not support random access.
       */

#ifdef CONFIG_FS_RAMMAP
      /* Allocate memory and copy the file into memory.  We would, of course,
       * do much better in the KERNEL build using the MMU.
       */

      return rammap(fd, length, offset);
#else
      /* Error out.  The errno value was already set by ioctl() */

      ferr("ERROR: ioctl(FIOC_MMAP) failed: %d\n", get_errno());
      errcode = ENOSYS;
      goto errout;
#endif
    }

  /* Return the offset address */

  return (FAR void *)(((FAR uint8_t *)addr) + offset);

errout:
  set_errno(errcode);
  return MAP_FAILED;
}
