/****************************************************************************
 * include/nuttx/fs/dirent.h
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

#ifndef __INCLUDE_NUTTX_FS_DIRENT_H
#define __INCLUDE_NUTTX_FS_DIRENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <dirent.h>

#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NFS
#  define DIRENT_NFS_MAXHANDLE 64        /* Maximum length of an NFSv3 file handle */
#  define DIRENT_NFS_VERFLEN    8        /* Length of the copy verifier */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The internal representation of type DIR is just a container for an inode
 * reference, a position, a dirent structure, and file-system-specific
 * information.
 *
 * For the root pseudo-file system, we need retain only the 'next' inode
 * need for the next readdir() operation.  We hold a reference on this
 * inode so we know that it will persist until closedir is called.
 */

struct fs_pseudodir_s
{
  struct inode *fd_next;             /* The inode for the next call to readdir() */
};

#ifndef CONFIG_DISABLE_MOUNTPOINT
#ifdef CONFIG_FS_FAT
/* For fat, we need to return the start cluster, current cluster, current
 * sector and current directory index.
 */

struct fs_fatdir_s
{
  off_t        fd_startcluster;        /* Start cluster number of the directory */
  off_t        fd_currcluster;         /* Current cluster number being read */
  off_t        fd_currsector;          /* Current sector being read */
  unsigned int fd_index;               /* Current index of the directory entry to read */
};
#endif /* CONFIG_FS_FAT */

#ifdef CONFIG_FS_ROMFS
#ifdef CONFIG_FS_ROMFS_CACHE_NODE

/* This structure represents one entry node in the romfs file system */

struct romfs_nodeinfo_s
{
  FAR struct romfs_nodeinfo_s **rn_child;  /* The node array for link to lower level */
  uint16_t rn_count;                       /* The count of node in rn_child level */
  uint32_t rn_offset;                      /* The offset to real file header of the current entry */
  uint32_t rn_next;                        /* The offset of the next file header+flags */
  uint32_t rn_size;                        /* The size to the entry (if file) */
  uint8_t  rn_namesize;                    /* The length of name of the entry */
  char rn_name[1];                         /* The name to the entry */
};

/* For ROMFS, we need to return the node to the current and start node
 * of the directory entry being read
 */

struct fs_romfsdir_s
{
  FAR struct romfs_nodeinfo_s **fr_firstnode; /* The address of first node in the directory */
  FAR struct romfs_nodeinfo_s **fr_currnode;  /* The address of current node into the directory */
};
#else

/* For ROMFS, we need to return the offset to the current and start positions
 * of the directory entry being read
 */

struct fs_romfsdir_s
{
  off_t        fr_firstoffset;         /* Offset to the first entry in the directory */
  off_t        fr_curroffset;          /* Current offset into the directory contents */
};
#endif
#endif /* CONFIG_FS_ROMFS */

#ifdef CONFIG_FS_CROMFS
/* For CROMFS, we need to return the next compressed node to be examined. */

struct fs_cromfsdir_s
{
  uint32_t     cr_firstoffset;         /* Offset to the first entry in the directory */
  uint32_t     cr_curroffset;          /* Current offset into the directory contents */
};
#endif /* CONFIG_FS_CROMFS */

#ifdef CONFIG_FS_TMPFS
/* For TMPFS, we need the directory object and an index into the directory
 * entries.
 */

struct tmpfs_directory_s;               /* Forward reference */
struct fs_tmpfsdir_s
{
  FAR struct tmpfs_directory_s *tf_tdo; /* Directory being enumerated */
  unsigned int tf_index;                /* Directory index */
};
#endif /* CONFIG_FS_TMPFS */

#ifdef CONFIG_FS_BINFS
/* The apps/ pseudo bin/ directory.  The state value is simply an index */

struct fs_binfsdir_s
{
  unsigned int fb_index;               /* Index to the next named entry point */
};
#endif

#ifdef CONFIG_FS_NXFFS
/* NXFFS is the tiny NuttX wear-leveling FLASH file system.
 * The state value is the offset in FLASH memory to the next inode entry.
 */

struct fs_nxffsdir_s
{
  off_t nx_offset;                     /* Offset to the next inode */
};
#endif

#ifdef CONFIG_NFS
/* The NFS client file system */

struct nfsdir_s
{
  uint8_t  nfs_fhsize;                        /* Length of the file handle */
  uint8_t  nfs_fhandle[DIRENT_NFS_MAXHANDLE]; /* File handle (max size allocated) */
  uint8_t  nfs_verifier[DIRENT_NFS_VERFLEN];  /* Cookie verifier */
  uint32_t nfs_cookie[2];                     /* Cookie */
};
#endif

#ifdef CONFIG_FS_SMARTFS
/* SMARTFS is the Sector Mapped Allocation for Really Tiny FLASH filesystem.
 * it is designed to use small sectors on small serial FLASH devices, using
 * minimal RAM footprint.
 */

struct fs_smartfsdir_s
{
  uint16_t fs_firstsector;                    /* First sector of directory list */
  uint16_t fs_currsector;                     /* Current sector of directory list */
  uint16_t fs_curroffset;                     /* Current offset within current sector */
};
#endif

#ifdef CONFIG_FS_SPIFFS

/* SPIFFS is an SPI-oriented FLASH file system
 * originally by Peter Andersson
 */

struct fs_spiffsdir_s
{
  int16_t block;                               /* Current block */
  int entry;                                   /* Current entry */
};
#endif

#ifdef CONFIG_FS_UNIONFS
/* The Union File System can be used to merge to different mountpoints so
 * that they appear as a single merged directory.
 */

struct fs_dirent_s;                           /* Forward reference */
struct fs_unionfsdir_s
{
  uint8_t fu_ndx;                             /* Index of file system being enumerated */
  bool fu_eod;                                /* True: At end of directory */
  bool fu_prefix[2];                          /* True: Fake directory in prefix */
  FAR char *fu_relpath;                       /* Path being enumerated */
  FAR struct fs_dirent_s *fu_lower[2];        /* dirent struct used by contained file system */
};
#endif

#ifdef CONFIG_FS_USERFS
/* The UserFS uses an opaque representation since the actual userspace
 * representation of the directory state structure is unknowable.
 */

struct fs_userfsdir_s
{
  FAR void *fs_dir;                           /* Opaque pointer to UserFS DIR */
};
#endif

#ifdef CONFIG_FS_HOSTFS
/* HOSTFS provides mapping to directories on the host machine in the
 * sim environment.
 */

struct fs_hostfsdir_s
{
  FAR void *fs_dir;                           /* Opaque pointer to host DIR */
};
#endif

#ifdef CONFIG_FS_RPMSGFS
/* RPMSGFS provides mapping to directories on the host machine in the
 * sim environment.
 */

struct fs_rpmsgfsdir_s
{
  FAR void *fs_dir;                           /* Opaque pointer to remote DIR */
};
#endif

#endif /* CONFIG_DISABLE_MOUNTPOINT */

struct fs_dirent_s
{
  /* This is the node that was opened by opendir.  The type of the inode
   * determines the way that the readdir() operations are performed. For the
   * pseudo root pseudo-file system, it is also used to support rewind.
   *
   * We hold a reference on this inode so we know that it will persist until
   * closedir() is called (although inodes linked to this inode may change).
   */

  struct inode *fd_root;

  /* Retained control information depends on the type of file system that
   * provides the mountpoint.  Ideally this information should
   * be hidden behind an opaque, file-system-dependent void *, but we put
   * the private definitions in line here for now to reduce allocations.
   */

  union
    {
      /* Private data used by the built-in pseudo-file system */

      struct fs_pseudodir_s pseudo;

      /* Private data used by other file systems */

#ifndef CONFIG_DISABLE_MOUNTPOINT
#ifdef CONFIG_FS_FAT
      struct fs_fatdir_s     fat;
#endif
#ifdef CONFIG_FS_ROMFS
      struct fs_romfsdir_s   romfs;
#endif
#ifdef CONFIG_FS_CROMFS
      struct fs_cromfsdir_s  cromfs;
#endif
#ifdef CONFIG_FS_TMPFS
      struct fs_tmpfsdir_s   tmpfs;
#endif
#ifdef CONFIG_FS_BINFS
      struct fs_binfsdir_s   binfs;
#endif
#ifdef CONFIG_FS_PROCFS
      FAR void              *procfs;
#endif
#ifdef CONFIG_FS_NXFFS
      struct fs_nxffsdir_s   nxffs;
#endif
#ifdef CONFIG_NFS
      struct nfsdir_s        nfs;
#endif
#ifdef CONFIG_FS_SMARTFS
      struct fs_smartfsdir_s smartfs;
#endif
#ifdef CONFIG_FS_SPIFFS
      struct fs_spiffsdir_s  spiffs;
#endif
#ifdef CONFIG_FS_LITTLEFS
      FAR void              *littlefs;
#endif
#ifdef CONFIG_FS_UNIONFS
      struct fs_unionfsdir_s unionfs;
#endif
#ifdef CONFIG_FS_USERFS
      struct fs_userfsdir_s  userfs;
#endif
#ifdef CONFIG_FS_HOSTFS
      struct fs_hostfsdir_s  hostfs;
#endif
#ifdef CONFIG_FS_RPMSGFS
      struct fs_rpmsgfsdir_s rpmsgfs;
#endif
#endif /* !CONFIG_DISABLE_MOUNTPOINT */
  } u;
};

/****************************************************************************
 * Public Data
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_FS_DIRENT_H */
