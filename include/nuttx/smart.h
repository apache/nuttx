/****************************************************************************
 * include/nuttx/smart.h
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

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Macros to hide implementation */

#define SMART_FMT_ISFORMATTED   0x01
#define SMART_FMT_HASBYTEWRITE  0x02

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
  const uint8_t *buffer;        /* Pointer to the data to write */
};

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
