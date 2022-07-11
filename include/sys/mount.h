/****************************************************************************
 * include/sys/mount.h
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

#ifndef __INCLUDE_SYS_MOUNT_H
#define __INCLUDE_SYS_MOUNT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLKSSZGET BIOC_BLKSSZGET

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

/* umount() is equivalent to umount2() with flags = 0 */

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
