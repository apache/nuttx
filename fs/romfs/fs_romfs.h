/****************************************************************************
 * fs/romfs/fs_romfs.h
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

#ifndef __FS_ROMFS_FS_ROMFS_H
#define __FS_ROMFS_FS_ROMFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Volume header (multi-byte values are big-endian) */

#define ROMFS_VHDR_ROM1FS   0  /*  0-7:  "-rom1fs-" */
#define ROMFS_VHDR_SIZE     8  /*  8-11: Number of accessible bytes in this fs. */
#define ROMFS_VHDR_CHKSUM  12  /* 12-15: Checksum of the first 512 bytes. */
#define ROMFS_VHDR_VOLNAME 16  /* 16-..: Zero terminated volume name, padded to
                                *        16 byte boundary. */

#define ROMFS_VHDR_MAGIC   "-rom1fs-"

/* File header offset (multi-byte values are big-endian) */

#define ROMFS_FHDR_NEXT     0  /*  0-3:  Offset of the next file header
                                *        (zero if no more files) */
#define ROMFS_FHDR_INFO     4  /*  4-7:  Info for directories/hard links/
                                *        devices */
#define ROMFS_FHDR_SIZE     8  /*  8-11: Size of this file in bytes */
#define ROMFS_FHDR_CHKSUM  12  /* 12-15: Checksum covering the meta data,
                                *        including the file name, and
                                *        padding. */
#define ROMFS_FHDR_NAME    16  /* 16-..: Zero terminated volume name, padded
                                *        to 16 byte boundary. */

/* Bits 0-3 of the rf_next offset provide mode information.  These are the
 * values specified in
 */

#define RFNEXT_MODEMASK    7    /* Bits 0-2: Mode; bit 3: Executable */
#define RFNEXT_ALLMODEMASK 15   /* Bits 0-3: All mode bits */
#define RFNEXT_OFFSETMASK (~15) /* Bits n-3: Offset to next entry */

#define RFNEXT_HARDLINK    0    /* rf_info = Link destination file header */
#define RFNEXT_DIRECTORY   1    /* rf_info = First file's header */
#define RFNEXT_FILE        2    /* rf_info = Unused, must be zero */
#define RFNEXT_SOFTLINK    3    /* rf_info = Unused, must be zero */
#define RFNEXT_BLOCKDEV    4    /* rf_info = 16/16 bits major/minor number */
#define RFNEXT_CHARDEV     5    /* rf_info = 16/16 bits major/minor number */
#define RFNEXT_SOCKET      6    /* rf_info = Unused, must be zero */
#define RFNEXT_FIFO        7    /* rf_info = Unused, must be zero */
#define RFNEXT_EXEC        8    /* Modifier of RFNEXT_DIRECTORY and RFNEXT_FILE */

#define IS_MODE(rfn,mode)  ((((uint32_t)(rfn))&RFNEXT_MODEMASK)==(mode))
#define IS_HARDLINK(rfn)   IS_MODE(rfn,RFNEXT_HARDLINK)
#define IS_DIRECTORY(rfn)  IS_MODE(rfn,RFNEXT_DIRECTORY)
#define IS_FILE(rfn)       IS_MODE(rfn,RFNEXT_FILE)
#define IS_SOFTLINK(rfn)   IS_MODE(rfn,RFNEXT_SOFTLINK)
#define IS_BLOCKDEV(rfn)   IS_MODE(rfn,RFNEXT_BLOCKDEV)
#define IS_CHARDEV(rfn)    IS_MODE(rfn,RFNEXT_CHARDEV)
#define IS_SOCKET(rfn)     IS_MODE(rfn,RFNEXT_SOCKET)
#define IS_FIFO(rfn)       IS_MODE(rfn,RFNEXT_FIFO)
#define IS_EXECUTABLE(rfn) (((rfn) & RFNEXT_EXEC) != 0)

/* RFNEXT_SOFTLINK, RFNEXT_BLOCKDEV, RFNEXT_CHARDEV, RFNEXT_SOCKET, and
 * RFNEXT_FIFO are not presently supported in NuttX.
 */

/* Alignment macros */

#define ROMFS_ALIGNMENT       16
#define ROMFS_MAXPADDING      (ROMFS_ALIGNMENT-1)
#define ROMFS_ALIGNMASK       (~ROMFS_MAXPADDING)
#define ROMFS_ALIGNUP(addr)   ((((uint32_t)(addr))+ROMFS_MAXPADDING)&ROMFS_ALIGNMASK)
#define ROMFS_ALIGNDOWN(addr) (((uint32_t)(addr))&ROMFS_ALIGNMASK)

/* Offset and sector conversions */

#define SEC_NDXMASK(r)       ((r)->rm_hwsectorsize - 1)
#define SEC_NSECTORS(r,o)    ((o) / (r)->rm_hwsectorsize)
#define SEC_ALIGN(r,o)       ((o) & ~SEC_NDXMASK(r))

/* Maximum numbr of links that will be followed before we decide that there
 * is a problem.
 */

#define ROMF_MAX_LINKS 64

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the overall mountpoint state.  An instance of
 * this structure is retained as inode private data on each mountpoint that
 * is mounted with a romfs filesystem.
 */

struct romfs_file_s;
struct romfs_mountpt_s
{
  FAR struct inode *rm_blkdriver; /* The block driver inode that hosts the romfs */
#ifdef CONFIG_FS_ROMFS_CACHE_NODE
  FAR struct romfs_nodeinfo_s *rm_root; /* The node for root node */
#else
  uint32_t rm_rootoffset;         /* Saved offset to the first root directory entry */
#endif
  bool     rm_mounted;            /* true: The file system is ready */
  uint16_t rm_hwsectorsize;       /* HW: Sector size reported by block driver */
  rmutex_t rm_lock;               /* Used to assume thread-safe access */
  uint32_t rm_refs;               /* The references for all files opened on this mountpoint */
  uint32_t rm_hwnsectors;         /* HW: The number of sectors reported by the hardware */
  uint32_t rm_volsize;            /* Size of the ROMFS volume */
  uint32_t rm_cachesector;        /* Current sector in the rm_buffer */
  FAR uint8_t *rm_xipbase;        /* Base address of directly accessible media */
  FAR uint8_t *rm_buffer;         /* Device sector buffer, allocated if rm_xipbase==0 */
};

/* This structure represents on open file under the mountpoint.  An instance
 * of this structure is retained as struct file specific information on each
 * opened file.
 */

struct romfs_file_s
{
  uint32_t rf_startoffset;        /* Offset to the start of the file data */
  uint32_t rf_endsector;          /* Last sector of the file data */
  uint32_t rf_size;               /* Size of the file in bytes */
  uint32_t rf_cachesector;        /* First sector in the rf_buffer */
  uint32_t rf_ncachesector;       /* Number of sectors in the rf_buffer */
  FAR uint8_t *rf_buffer;         /* File sector buffer, allocated if rm_xipbase==0 */
  uint8_t rf_type;                /* File type (for fstat()) */
  char rf_path[1];                /* Path of open file */
};

struct romfs_nodeinfo_s
{
#ifdef CONFIG_FS_ROMFS_CACHE_NODE
  FAR struct romfs_nodeinfo_s **rn_child;  /* The node array for link to lower level */
  uint16_t rn_count;                       /* The count of node in rn_child level */
#endif
  uint32_t rn_offset;                      /* Offset of real file header */
  uint32_t rn_next;                        /* Offset of the next file header+flags */
  uint32_t rn_size;                        /* Size (if file) */
#ifdef CONFIG_FS_ROMFS_CACHE_NODE
  uint8_t  rn_namesize;                    /* The length of name of the entry */
  char     rn_name[1];                     /* The name to the entry */
#endif
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

int  romfs_hwread(FAR struct romfs_mountpt_s *rm, FAR uint8_t *buffer,
                  uint32_t sector, unsigned int nsectors);
int  romfs_filecacheread(FAR struct romfs_mountpt_s *rm,
                         FAR struct romfs_file_s *rf, uint32_t sector);
int  romfs_hwconfigure(FAR struct romfs_mountpt_s *rm);
int  romfs_fsconfigure(FAR struct romfs_mountpt_s *rm);
int  romfs_fileconfigure(FAR struct romfs_mountpt_s *rm,
                         FAR struct romfs_file_s *rf);
int  romfs_checkmount(FAR struct romfs_mountpt_s *rm);
int  romfs_finddirentry(FAR struct romfs_mountpt_s *rm,
                        FAR struct romfs_nodeinfo_s *nodeinfo,
                        FAR const char *path);
int  romfs_parsedirentry(FAR struct romfs_mountpt_s *rm, uint32_t offset,
                         FAR uint32_t *poffset, FAR uint32_t *pnext,
                         FAR uint32_t *pinfo, FAR uint32_t *psize);
int  romfs_parsefilename(FAR struct romfs_mountpt_s *rm, uint32_t offset,
                         FAR char *pname);
int  romfs_datastart(FAR struct romfs_mountpt_s *rm,
                     FAR struct romfs_nodeinfo_s *nodeinfo,
                     FAR uint32_t *start);
#ifdef CONFIG_FS_ROMFS_CACHE_NODE
void romfs_freenode(FAR struct romfs_nodeinfo_s *node);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_ROMFS_FS_ROMFS_H */
