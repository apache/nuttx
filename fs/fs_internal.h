/****************************************************************************
 * fs_internal.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#ifndef __FS_INTERNAL_H
#define __FS_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs.h>
#include <dirent.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define FSNODEFLAG_TYPE_MASK      0x00000003
#define   FSNODEFLAG_TYPE_DRIVER  0x00000000
#define   FSNODEFLAG_TYPE_BLOCK   0x00000001
#define   FSNODEFLAG_TYPE_MOUNTPT 0x00000002
#define FSNODEFLAG_DELETED        0x00000004

#define INODE_IS_DRIVER(i) \
  (((i)->i_flags & FSNODEFLAG_TYPE_MASK) == FSNODEFLAG_TYPE_DRIVER)
#define INODE_IS_BLOCK(i) \
  (((i)->i_flags & FSNODEFLAG_TYPE_BLOCK) == FSNODEFLAG_TYPE_BLOCK)
#define INODE_IS_MOUNTPT(i) \
  (((i)->i_flags & FSNODEFLAG_TYPE_MOUNTPT) == FSNODEFLAG_TYPE_MOUNTPT)

#define INODE_SET_DRIVER(i) \
  ((i)->i_flags &= ~FSNODEFLAG_TYPE_MASK)
#define INODE_SET_BLOCK(i) \
  ((i)->i_flags = (((i)->i_flags & ~FSNODEFLAG_TYPE_MASK) | FSNODEFLAG_TYPE_BLOCK))
#define INODE_SET_MOUNTPT(i) \
  ((i)->i_flags = (((i)->i_flags & ~FSNODEFLAG_TYPE_MASK) | FSNODEFLAG_TYPE_MOUNTPT))

/* Mountpoint fd_flags values */

#define DIRENTFLAGS_PSUEDONODE 1

#define DIRENT_SETPSUEDONODE(f) do (f) |= DIRENTFLAGS_PSUEDONODE; while (0)
#define DIRENT_ISPSUEDONODE(f) (((f) & DIRENTFLAGS_PSUEDONODE) != 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The internal representation of type DIR is just a container for an inode
 * reference, a position, a dirent structure, and file-system-specific
 * information.
 *
 * For the root psuedo-file system, we need retain only the 'next' inode
 * need for the next readdir() operation.  We hold a reference on this
 * inode so we know that it will persist until closedir is called.
 */

struct fs_psuedodir_s
{
  struct inode *fd_next;             /* The inode for the next call to readdir() */
};

#if defined(CONFIG_FS_FAT) && !defined(CONFIG_DISABLE_MOUNTPOINT)
/* For fat, we need to retun the start cluster, current cluster, current
 * sector and current directory index.
 */

struct fs_fatdir_s
{
  uint32       fd_startcluster;        /* Start cluster number of the directory*/
  uint32       fd_currcluster;         /* Current cluster number being read*/
  size_t       fd_currsector;          /* Current sector being read*/
  unsigned int fd_index;               /* Current index of the directory entry to read */
};
#endif

struct internal_dir_s
{
  /* This is the node that was opened by opendir.  The type of the inode
   * determines the way that the readdir() operations are performed. For the
   * psuedo root psuedo-file system, it is also used to support rewind.
   *
   * We hold a reference on this inode so we know that it will persist until
   * closedir() is called (although inodes linked to this inode may change).
   */

  struct inode *fd_root;

  /* At present, only mountpoints require special handling flags */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  unsigned int fd_flags;
#endif

  /* This keeps track of the current directory position for telldir */

  off_t fd_position;

  /* Retained control information depends on the type of file system that
   * provides is provides the mountpoint.  Ideally this information should
   * be hidden behind an opaque, file-system-dependent void *, but we put
   * the private definitions in line here for now to reduce allocations.
   */

  union
    {
      /* Private data used by the built-in psuedo-file system */

      struct fs_psuedodir_s psuedo;

      /* Private data used by other file systems */

#ifndef CONFIG_DISABLE_MOUNTPOINT
#ifdef CONFIG_FS_FAT
      struct fs_fatdir_s    fat;
#endif
#endif
   } u;

  /* In any event, this the actual struct dirent that is returned by readdir */

  struct dirent fd_dir;              /* Populated when readdir is called */
};

/****************************************************************************
 * Global Variables
 ****************************************************************************/

extern FAR struct inode *root_inode;

/****************************************************************************
 * Pulblic Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* fs_inode.c ***************************************************************/

EXTERN void inode_semtake(void);
EXTERN void inode_semgive(void);
EXTERN FAR struct inode *inode_search(const char **path,
                                      FAR struct inode **peer,
                                      FAR struct inode **parent,
                                      const char **relpath);
EXTERN void inode_free(FAR struct inode *node);
EXTERN const char *inode_nextname(const char *name);

/* fs_inodereserver.c ********************************************************/

EXTERN FAR struct inode *inode_reserve(const char *path);

/* fs_inoderemove.c **********************************************************/

EXTERN STATUS inode_remove(const char *path);

/* fs_inodefind.c ************************************************************/

EXTERN FAR struct inode *inode_find(const char *path, const char **relpath);

/* fs_inodeaddref.c **********************************************************/

EXTERN void inode_addref(FAR struct inode *inode);

/* fs_inoderelease.c *********************************************************/

EXTERN void inode_release(FAR struct inode *inode);

/* fs_files.c ***************************************************************/

#if CONFIG_NFILE_DESCRIPTORS >0
EXTERN void weak_function files_initialize(void);
EXTERN int  files_allocate(FAR struct inode *inode, int oflags, off_t pos);
EXTERN void files_release(int filedes);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_INTERNAL_H */
