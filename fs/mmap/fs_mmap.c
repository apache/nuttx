/****************************************************************************
 * fs/mmap/fs_mmap.c
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

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "inode/inode.h"
#include "fs_rammap.h"
#include "fs_anonmap.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_mmap_
 ****************************************************************************/

static int file_mmap_(FAR struct file *filep, FAR void *start,
                      size_t length, int prot, int flags,
                      off_t offset, bool kernel, FAR void **mapped)
{
  int ret;

  /* Pass the information about the mapping in mm_map_entry_s structure.
   * The driver may alter the structure, and if it supports unmap, it
   * will also add it to the kernel maintained list of mappings.
   */

  struct mm_map_entry_s entry =
    {
     NULL,     /* sq_entry_t */
     start,
     length,
     offset,
     prot,
     flags,
     { NULL }, /* priv.p */
     NULL      /* munmap */
    };

  /* Since only a tiny subset of mmap() functionality, we have to verify many
   * things.
   */

#ifdef CONFIG_DEBUG_FEATURES
  /* Fixed mappings and protections are not currently supported.  These
   * options could be supported in the KERNEL build with an MMU, but that
   * logic is not in place.
   */

  if ((flags & (MAP_FIXED | MAP_DENYWRITE)) != 0)
    {
      ferr("ERROR: Unsupported options, prot=%x flags=%04x\n", prot, flags);
      return -ENOSYS;
    }

  /* A length of 0 is invalid. */

  if (length == 0)
    {
      ferr("ERROR: Invalid length, length=%zu\n", length);
      return -EINVAL;
    }
#endif /* CONFIG_DEBUG_FEATURES */

  /* Check if we are just be asked to allocate memory, i.e., MAP_ANONYMOUS
   * set meaning that the memory is not backed up from a file.  The file
   * descriptor should be -1 (or refer to opened /dev/zero) in this case.
   * The file descriptor is ignored in either case.
   */

  if ((flags & MAP_ANONYMOUS) != 0)
    {
      ret = map_anonymous(&entry, kernel);
      goto out;
    }

  if (filep == NULL)
    {
      return -EBADF;
    }

  if ((filep->f_oflags & O_WROK) == 0 && prot == PROT_WRITE)
    {
      ferr("ERROR: Unsupported options for read-only file descriptor,"
           "prot=%x flags=%04x\n", prot, flags);
      return -EACCES;
    }

  if ((filep->f_oflags & O_RDOK) == 0)
    {
      ferr("ERROR: File descriptor does not have read permission\n");
      return -EACCES;
    }

  /* Call driver's mmap to get the base address of the file in 'mapped'
   * in memory.
   */

  if (filep->f_inode &&
      filep->f_inode->u.i_ops->mmap != NULL)
    {
      ret = filep->f_inode->u.i_ops->mmap(filep, &entry);
    }
  else
    {
      /* Caller request the private mapping. Or not directly mappable,
       * probably because the underlying media doesn't support random access.
       */

      /* Allocate memory and copy the file into memory.  We would, of course,
       * do much better in the KERNEL build using the MMU.
       */

      ret = rammap(filep, &entry, kernel);
    }

  /* Return */

out:
  if (ret >= OK)
    {
      *mapped = entry.vaddr;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_mmap
 *
 * Description:
 *   Equivalent to the standard mmap() function except that is accepts
 *   a struct file instance instead of a file descriptor and it does not set
 *   the errno variable.
 *
 ****************************************************************************/

int file_mmap(FAR struct file *filep, FAR void *start, size_t length,
              int prot, int flags, off_t offset, FAR void **mapped)
{
  return file_mmap_(filep, start, length,
                    prot, flags, offset, true, mapped);
}

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
 *     a. The filesystem implements the mmap file operation.  Any file
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
 *   fd      file descriptor of the backing file.
 *   offset  The offset into the file to map
 *
 * Returned Value:
 *   On success, mmap() returns a pointer to the mapped area. On error, the
 *   value MAP_FAILED is returned, and errno is set appropriately.
 *
 *     EACCES
 *       The fd argument is not open for read, regardless of the
 *       protection specified, or fd is not open for write and PROT_WRITE
 *       was specified for a MAP_SHARED type mapping.
 *     ENOSYS
 *       Returned if any of the unsupported mmap() features are attempted
 *     EBADF
 *       'fd' is not a valid file descriptor.
 *     EINVAL
 *       Length is 0. flags contained neither MAP_PRIVATE or MAP_SHARED, or
 *       contained both of these values.
 *     ENODEV
 *       The underlying filesystem of the specified file does not support
 *       memory mapping.
 *     ENOMEM
 *       Insufficient memory is available to map the file.
 *
 ****************************************************************************/

FAR void *mmap(FAR void *start, size_t length, int prot, int flags,
               int fd, off_t offset)
{
  FAR struct file *filep = NULL;
  FAR void *mapped = NULL;
  int ret;

  if (fd != -1 && fs_getfilep(fd, &filep) < 0)
    {
      ferr("ERROR: Invalid file descriptor, fd=%d\n", fd);
      ret = -EBADF;
      goto errout;
    }

  ret = file_mmap_(filep, start, length,
                   prot, flags, offset, false, &mapped);
  if (ret < 0)
    {
      goto errout;
    }

  return mapped;

errout:
  set_errno(-ret);
  return MAP_FAILED;
}
