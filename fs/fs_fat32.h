/****************************************************************************
 * fs_fat.h
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

#ifndef __FS_FAT_H
#define __FS_FAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <semaphore.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * These offsets describes the master boot record.
 *
 * The folowing fields are common to FAT12/16/32 (but all value descriptions
 * refer to the interpretation under FAT32.
 */

#define BS_JUMP             0 /*  3@0: Jump instruction to boot code (ignored) */
#define BS_OEMNAME          3 /*  8@3: Usually "MSWIN4.1" */
#define BS_BYTESPERSEC     11 /*  2@11: Bytes per sector: 512, 1024, 2048, 4096  */
#define BS_SECPERCLUS      13 /*  1@13: Sectors per allocation unit: 2**n, n=0..7 */
#define BS_RESVDSECCOUNT   14 /*  2@14: Reserved sector count: Usually 32 */
#define BS_NUMFATS         16 /*  1@16: Number of FAT data structures: always 2 */
#define BS_ROOTENTCNT      17 /*  2@17: FAT12/16: Must be 0 for FAT32 */
#define BS_TOTSEC16        19 /*  2@19: FAT12/16: Must be 0, see BS32_totsec32 */
#define BS_MEDIA           21 /*  1@21: Media code: f0, f8, f9-fa, fc-ff */ 
#define BS_FATSZ16         22 /*  2@22: FAT12/16: Must be 0, see BS32_fatsz32 */
#define BS_SECPERTRK       24 /*  2@24: Sectors per track geometry value */
#define BS_NUMHEADS        26 /*  2@26: Number of heads geometry value */
#define BS_HIDSEC          28 /*  4@28: Count of hidden sectors preceding FAT */
#define BS_TOTSEC32        32 /*  4@32: Total count of sectors on the volume */

/* The following fields are only valid for FAT12/16 */

#define BS16_DRVNUM        36 /*  1@36: Drive number for MSDOS bootstrap */
                              /*  1@37: Reserverd (zero) */
#define BS16_BOOTSIG       38 /*  1@38: Extended boot signature: 0x29 if following valid */
#define BS16_VOLID         39 /*  4@39: Volume serial number */
#define BS16_VOLLAB        43 /* 11@43: Volume label */
#define BS16_FILESYSTYPE   54 /*  8@54: "FAT12  ", "FAT16  ", or "FAT    " */

/* The following fields are only valid for FAT32 */

#define BS32_FATSZ32       36 /*  4@36: Count of sectors occupied by one FAT */
#define BS32_EXTFLAGS      40 /*  2@40: 0-3:Active FAT, 7=0 both FATS, 7=1 one FAT */ 
#define BS32_FSVER         42 /*  2@42: MSB:Major LSB:Minor revision number (0.0) */
#define BS32_ROOTCLUS      44 /*  4@44: Cluster no. of 1st cluster of root dir */
#define BS32_FSINFO        48 /*  2@48: Sector number of fsinfo structure. Usually 1. */
#define BS32_BKBOOTSEC     50 /*  2@50: Sector number of boot record. Usually 6  */
                              /* 12@52: Reserved (zero) */
#define BS32_DRVNUM        64 /*  1@64: Drive number for MSDOS bootstrap */
                              /*  1@65: Reserverd (zero) */
#define BS32_BOOTSIG       66 /*  1@66: Extended boot signature: 0x29 if following valid */
#define BS32_VOLID         67 /*  4@67: Volume serial number */
#define BS32_VOLLAB        71 /* 11@71: Volume label */
#define BS32_FILESYSTYPE   82 /*  8@82: "FAT12  ", "FAT16  ", or "FAT    " */

/* If the sector is not an MBR, then it could have a partition table at
 * this offset.
 */

#define MBR_TABLE         446

/* The magic bytes at the end of the MBR are common to FAT12/16/32 */

#define BS_SIGNATURE      510 /*  2@510: Valid MBRs have 0x55aa here */

/* File system types */

#define FSTYPE_FAT12        0
#define FSTYPE_FAT16        1
#define FSTYPE_FAT32        2

/****************************************************************************
 * These offset describe the FSINFO sector
 */

#define FSI_LEADSIG         0 /*   4@0:   0x41615252 */
                              /* 480@4:   Reserved (zero) */
#define FSI_STRUCTSIG     484 /*   4@484: 0x61417272 */
#define FSI_FREECOUNT     488 /*   4@488: Last free cluster count on volume */
#define FSI_NXTFREE       492 /*   4@492: Cluster number of 1st free cluster */
                              /*  12@496: Reserved (zero) */
#define FSI_TRAILSIG      508 /*   4@508: 0xaa550000 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct fat_direntry_s
{
    ubyte fde_name[8];         /* name */
    ubyte fde_ext[3];          /* name and extension */
    ubyte fde_attr;            /* attribute bits */
    ubyte fde_lcase;           /* Case for base and extension */
    ubyte fde_ctimecs;         /* Creation time, centiseconds (0-199) */
    ubyte fde_ctime[2];        /* Creation time */
    ubyte fde_cdate[2];        /* Creation date */
    ubyte fde_adate[2];        /* Last access date */
    ubyte fde_starthi[2];      /* High 16 bits of cluster in FAT32 */
    ubyte fde_time[2];         /* Date */
    ubyte fde_date[2];         /* Time */
    ubyte fde_start[2];        /* First cluster */
    ubyte fde_size[4];         /* file size (in bytes) */
};

struct fat_dirslot_s
{
    ubyte fds_id;             /* sequence number for slot */
    ubyte fds_name0_4[10];    /* first 5 characters in name */
    ubyte fds_attr;           /* attribute byte */
    ubyte fds_reserved;       /* always 0 */
    ubyte fds_alias_checksum; /* checksum for 8.3 alias */
    ubyte fds_name5_10[12];   /* 6 more characters in name */
    ubyte fds_start[2];       /* starting cluster number, 0 in long slots */
    ubyte fds_name11_12[4];   /* last 2 characters in name */
};

/* This structure represents the overall mountpoint state.  An instance of this
 * structure is retained as inode private data on each mountpoint that is
 * mounted with a fat32 filesystem.
 */

struct fat_file_s;
struct fat_mountpt_s
{
  struct inode      *fs_blkdriver; /* The block driver inode that hosts the FAT32 fs */
  struct fat_file_s *fs_head;      /* A list to all files opened on this mountpoint */

  boolean  fs_mounted;             /* TRUE: The file system is ready */
  sem_t    fs_sem;                 /* Used to assume thread-safe access */
  size_t   fs_hwsectorsize;        /* HW: Sector size reported by block driver*/
  size_t   fs_hwnsectors;          /* HW: The number of sectors reported by the hardware */
  size_t   fs_fatbase;             /* Logical block of start of filesystem (past resd sectors) */
  size_t   fs_rootbase;            /* MBR: Cluster no. of 1st cluster of root dir */
  size_t   fs_database;            /* Logical block of start data sectors */
  size_t   fs_fsinfo;              /* MBR: Sector number of FSINFO sector */
  uint32   fs_nclusters;           /* Maximum number of data clusters */
  uint32   fs_fatsize;             /* MBR: Count of sectors occupied by one fat */
  uint32   fs_fattotsec;           /* MBR: Total count of sectors on the volume */
  uint32   fs_fsifreecount;        /* FSI: Last free cluster count on volume */
  uint32   fs_fsinextfree;         /* FSI: Cluster number of 1st free cluster */
  uint16   fs_fatresvdseccount;    /* MBR: The total number of reserved sectors */
  uint16   fs_rootentcnt;          /* MBR: Count of 32-bit root directory entries */
  ubyte    fs_type;                /* FSTYPE_FAT12, FSTYPE_FAT16, or FSTYPE_FAT32 */
  ubyte    fs_fatnumfats;          /* MBR: Number of FATs (probably 2) */
  ubyte    fs_fatsecperclus;       /* MBR: Sectors per allocation unit: 2**n, n=0..7 */
  ubyte   *fs_buffer;              /* This is an allocated buffer to hold one sector
                                    * from the device */
};

/* This structure represents on open file under the mountpoint.  An instance of this
 * structure is retained as struct file specific information on each opened file.
 */

struct fat_file_s
{
  struct fat_file_s *ff_next;     /* File structures are retained in a singly linked list */
  struct fat_mountpt_s *ff_parent;
  boolean  ff_open;               /* TRUE: The file is (still) open */
};

/****************************************************************************
 * Global Variables
 ****************************************************************************/

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_FAT_H */
