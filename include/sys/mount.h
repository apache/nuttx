/****************************************************************************
 * include/sys/mount.h
 *
 *   Copyright (C) 2007-2009, 2015 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_MOUNT_H
#define __INCLUDE_SYS_MOUNT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mount flags */

#define MS_RDONLY 1 /* Mount file system read-only */

/* Un-mount flags
 *
 * These flags control the behavior of umount2() when there are open file
 * references through the mountpoint.  umount2() with no flag bits set is
 * equivalent to umount(), i.e., the umount() will fail with EBUSY if there
 * are open references on the file.
 *
 *   MNT_FORCE - Force unmount even if busy. This can cause data loss.
 *   MNT_DETACH - Perform a lazy unmount: make the mount point unavailable
 *     for new accesses, and actually perform the unmount when the mount
 *     point ceases to be busy.
 *   MNT_EXPIRE - Mark the mount point as expired. If a mount point is not
 *     currently in use, then an initial call to umount2() with this flag
 *     fails with the error EAGAIN, but marks the mount point as expired. The
 *     mount point remains expired as long as it isn't accessed by any
 *     process. A second umount2() call specifying MNT_EXPIRE unmounts an
 *     expired mount point. This flag cannot be specified with either
 *     MNT_FORCE or MNT_DETACH.
 *  UMOUNT_NOFOLLOW (- Don't dereference target if it is a symbolic link.
 *     For Linux, this flag allows security problems to be avoided in
 *     set-user-ID-root programs that allow unprivileged users to unmount
 *     file systems.  For NuttX, it is provided only for compatibility
 *
 * Not all options are supported on all file systems.
 */

#define MNT_FORCE       (1 << 0)
#define MNT_DETACH      (1 << 1)
#define MNT_EXPIRE      (1 << 2)
#define UMOUNT_NOFOLLOW (0)

/* mount() is equivalent to umount2() with flags = 0 */

#define umount(t)       umount2(t,0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

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

int mount(FAR const char *source, FAR const char *target,
          FAR const char *filesystemtype, unsigned long mountflags,
          FAR const void *data);
int umount2(FAR const char *target, unsigned int flags);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_MOUNT_H */
