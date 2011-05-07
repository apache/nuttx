/****************************************************************************
 * fs/mmap/fs_munmap.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include "fs_internal.h"

#ifdef CONFIG_FS_RAMMAP

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: munmap
 *
 * Description:
 *
 *   munmap() system call deletes mappings for the specified address range.
 *   The region is also automatically unmapped when the process is terminated.
 *   On the other hand, closing the file descriptor does not unmap the region.
 *
 *   NuttX operates in a flat open address space.  Therefore, it generally
 *   does not require mmap() and, hence, munmap functionality.  There are
 *   two exceptions where mmap() is available:
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
 *     munmap() is still not required in this first case.  In this first
 *     The mapped address is a static address in the MCUs address space
 *     does not need to be munmapped.  Support for munmap() in this case
 *     provided by the simple definition in sys/mman.h:
 *
 *        #define munmap(start, length)
 *
 *   2. If CONFIG_FS_RAMMAP is defined in the configuration, then mmap() will
 *      support simulation of memory mapped files by copying files whole
 *      into RAM.  munmap() is required in this case to free the allocated
 *      memory holding the shared copy of the file.
 *
 * Parameters:
 *   start   The start address of the mapping to delete.  For this
 *           simplified munmap() implementation, the *must* be the start
 *           address of the memory region (the same address returned by
 *           mmap()).
 *   length  The length region to be umapped.  Ignored.  The entire underlying
 *           media is always freed.
 *
 * Returned Value:
 *   On success, munmap() returns 0, on failure -1, and errno is set
 *   (probably to EINVAL).
 *
 ****************************************************************************/

int munmap(FART void *start, size_t length)
{
#warning "Missing logic"
  return MAP_FAILED;
}

#endif /* CONFIG_FS_RAMMAP */
