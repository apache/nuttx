/****************************************************************************
 * include/nuttx/fs/smart.h
 * Sector Mapped Allocation for Really Tiny (SMART) FLASH interface
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#ifndef __INCLUDE_NUTTX_SMART_H
#define __INCLUDE_NUTTX_SMART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros to hide implementation */

#define SMART_FMT_ISFORMATTED   0x01
#define SMART_FMT_HASBYTEWRITE  0x02

/* The logical sector number of the root directory. */

#define SMARTFS_ROOT_DIR_SECTOR   3

/* Defines the sector types */

#define SMARTFS_SECTOR_TYPE_DIR   1
#define SMARTFS_SECTOR_TYPE_FILE  2

#ifdef CONFIG_SMART_DEV_LOOP
/* Loop device IOCTL commands */

/* Command:      SMART_LOOPIOC_SETUP
 * Description:  Setup the loop device
 * Argument:     A pointer to a read-only instance of struct losetup_s.
 * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
 */

/* Command:      SMART_LOOPIOC_TEARDOWN
 * Description:  Teardown a loop device previously setup vis LOOPIOC_SETUP
 * Argument:     A read-able pointer to the path of the device to be
 *               torn down
 * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
 */

#define SMART_LOOPIOC_SETUP     _LOOPIOC(0x0001)
#define SMART_LOOPIOC_TEARDOWN  _LOOPIOC(0x0002)

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The following defines the format information for the device.  This
 * information is retrieved via the BIOC_GETFORMAT ioctl.
 */

struct smart_format_s
{
  uint16_t sectorsize;      /* Size of one read/write sector */
  uint16_t availbytes;      /* Number of bytes available in each sector */
  uint16_t nsectors;        /* Total number of sectors on device */
  uint16_t nfreesectors;    /* Number of free sectors on device */
  uint8_t  flags;           /* Format flags (see above) */
  uint8_t  namesize;        /* Size of filenames on this volume */
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  uint8_t  nrootdirentries; /* Number of root directories on this device */
  uint8_t  rootdirnum;      /* Root directory number for this dev entry */
#endif
};

/* The following defines the information for writing a logical sector
 * to the device.
 */

struct smart_read_write_s
{
  uint16_t logsector;     /* The logical sector number */
  uint16_t offset;        /* Offset within the sector to write to */
  uint16_t count;         /* Number of bytes to write */
  const uint8_t *buffer;  /* Pointer to the data to write */
};

/* The following defines the procfs data exchange interface between the
 * SMART MTD and FS layers.
 */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
struct smart_procfs_data_s
{
#ifdef CONFIG_MTD_SMART_ERASE_DEBUG
  const uint16_t  *erasecounts;   /* Pointer to the erase counts array */
  uint16_t        erasesize;      /* Number of entries in the erase counts array */
#endif
};
#endif

#ifdef CONFIG_SMART_DEV_LOOP
/* This is the structure referred to in the argument to the LOOPIOC_SETUP
 * IOCTL command.
 */

struct smart_losetup_s
{
  FAR const char *filename;     /* The file or character device to use */
  int             minor;        /* The minor number of thedevice */
  int             erasesize;    /* The erase size to use on the file */
  int             sectsize;     /* The sector / page size of the file */
  off_t           offset;       /* An offset that may be applied to the device */
  bool            readonly;     /* True: Read access will be supported only */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_SMART_H */
