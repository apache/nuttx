/****************************************************************************
 * fs/fat/fs_initmbr.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <errno.h>

#include <nuttx/fs.h>
#include <nuttx/fat.h>
#include <nuttx/mkfatfs.h>

#include "fs_internal.h"
#include "fs_fat32.h"
#include "fs_mkfatfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfatfs_initmbr
 *
 * Description:
 *   Initialize the sector image of a masterbood record
 *
 * Input:
 *    fmt  - User specified format parameters
 *    var  - Other format parameters that are not user specifiable
 *    sect - Allocated memory to hold the MBR
 *
 * Return:
 *    None; caller is responsible for providing valid parameters.
 *
 ****************************************************************************/
void mkfatfs_initmbr(FAR struct fat_format_s *fmt, FAR struct fat_var_s *var, ubyte *sect)
{
  memset(sect, 0, var->fv_sectorsize);

  /*  3@0:  Jump instruction to boot code */

  memcpy(&sect[BS_JUMP], var->fv_jump, 3);

  /*  8@3:  Usually "MSWIN4.1" */

  strcpy(&sect[BS_OEMNAME], "NUTTX   ");

  /*  2@11: Bytes per sector: 512, 1024, 2048, 4096  */

  MBR_PUTBYTESPERSEC(sect, var->fv_sectorsize);

  /*  1@13: Sectors per allocation unit: 2**n, n=0..7 */

  MBR_PUTSECPERCLUS(sect, fmt->ff_clustsize);

  /*  2@14: Reserved sector count: Usually 32 */

  MBR_PUTRESVDSECCOUNT(sect, fmt->ff_rsvdseccount);

  /*  1@16: Number of FAT data structures: always 2 */

  MBR_PUTNUMFATS(sect, fmt->ff_nfats);

  /*  2@17: FAT12/16: Must be 0 for FAT32 */

  MBR_PUTROOTENTCNT(sect, fmt->ff_rootdirentries);

  /*  2@19: FAT12/16: Must be 0, see BS_TOTSEC32.  Handled with 4@32: Total count of sectors on the volume */

  if (var->fv_nsectors >= 65536)
    {
      MBR_PUTTOTSEC32(sect, var->fv_nsectors);
    }
  else
    {
      MBR_PUTTOTSEC16(sect, (uint16)var->fv_nsectors);
    }

  /*  1@21: Media code: f0, f8, f9-fa, fc-ff */ 

  MBR_PUTMEDIA(sect, FAT_DEFAULT_MEDIA_TYPE);   /* Only "hard drive" supported */
  
  /*  2@22: FAT12/16: Must be 0, see BS32_FATSZ32  -- handled in FAT specific logic */
 
  /*  2@24: Sectors per track geometry value and  2@26: Number of heads geometry value */

  MBR_PUTSECPERTRK(sect, FAT_DEFAULT_SECPERTRK);
  MBR_PUTNUMHEADS(sect, FAT_DEFAULT_NUMHEADS);

  /*  4@28: Count of hidden sectors preceding FAT */

  MBR_PUTHIDSEC(sect, fmt->ff_hidsec);

  /*  4@32: Total count of sectors on the volume -- handled above */

  /* Most of the rest of the sector depends on the FAT size */

  if (fmt->ff_fatsize != 32)
    {
      /*  2@22: FAT12/16: Must be 0, see BS32_FATSZ32 */

      MBR_PUTFATSZ16(sect, (uint16)var->fv_fatlen);

      /* The following fields are only valid for FAT12/16 */
      /*  1@36: Drive number for MSDOS bootstrap -- left zero */
      /*  1@37: Reserved (zero) */
      /*  1@38: Extended boot signature: 0x29 if following valid */

      MBR_PUTBOOTSIG16(sect, EXTBOOT_SIGNATURE);

      /*  4@39: Volume serial number */

      MBR_PUTVOLID16(sect, fmt->ff_volumeid);

      /* 11@43: Volume label */

      memcpy(&sect[BS16_VOLLAB], fmt->ff_volumelabel, 11);

      /*  8@54: "FAT12  ", "FAT16  ", or "FAT    " */
      /* Boot code may be placed in the remainder of the sector */

      memcpy(&sect[BS16_BOOTCODE], var->fv_bootcode, var->fv_bootcodesize);
    }
  else
    {
      /* The following fields are only valid for FAT32 */
      /*  4@36: Count of sectors occupied by one FAT */

      MBR_PUTFATSZ32(sect, var->fv_fatlen);

      /*  2@40: 0-3:Active FAT, 7=0 both FATS, 7=1 one FAT -- left zero*/ 
      /*  2@42: MSB:Major LSB:Minor revision number (0.0) -- left zero */
      /*  4@44: Cluster no. of 1st cluster of root dir */

      MBR_PUTROOTCLUS(sect, FAT32_DEFAULT_ROOT_CLUSTER);

      /*  2@48: Sector number of fsinfo structure. Usually 1. */

      MBR_PUTFSINFO(sect, FAT_DEFAULT_FSINFO_SECTOR);

      /*  2@50: Sector number of boot record. Usually 6  */

      MBR_PUTBKBOOTSEC(sect, fmt->ff_backupboot);

      /* 12@52: Reserved (zero) */
      /*  1@64: Drive number for MSDOS bootstrap -- left zero */
      /*  1@65: Reserved (zero) */
      /*  1@66: Extended boot signature: 0x29 if following valid */
 
      MBR_PUTBOOTSIG32(sect, EXTBOOT_SIGNATURE);

      /*  4@67: Volume serial number */

      MBR_PUTVOLID32(sect, fmt->ff_volumeid);

      /* 11@71: Volume label */

      memcpy(&sect[BS32_VOLLAB], fmt->ff_volumelabel, 11);

      /*  8@82: "FAT12  ", "FAT16  ", or "FAT    " */
      /* Boot code may be placed in the remainder of the sector */

      memcpy(&sect[BS16_BOOTCODE], var->fv_bootcode, var->fv_bootcodesize);
    }

  /* The magic bytes at the end of the MBR are common to FAT12/16/32 */
  /*  2@510: Valid MBRs have 0x55aa here */

  MBR_PUTSIGNATURE(sect, BOOT_SIGNATURE16);
}
