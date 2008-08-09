/****************************************************************************
 * fs/fat/fs_configfat.c
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
 * Name:  mkfatfs_nfatsect12
 *
 * Description:
 *   Calculate the number of sectors need for the fat in a FAT12 file system.
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *   navailsects - The number of sectors available for both FAT and data.
 *     This is a precalculated value equal to the total number of sectors
 *     minus the number of root directory sectors and minus the number of
 *     reserved sectors.
 *
 * Return:
 *   0: That calculation would have overflowed
 *  >0: The total size of the FAT in sectors.
 *
 ****************************************************************************/
static inline uint32 mkfatfs_nfatsect12(FAR struct fat_format_s *fmt,
                                        FAR struct fat_var_s *var,
                                        uint32 navailsects)
{
   uint32 denom ;
   uint32 numer;

  /* For FAT12, the cluster number is held in a 12-bit number or 1.5 bytes per
   * cluster reference.  So each FAT sector will hold sectorsize/1.5 cluster
   * references (except for the first sector of each FAT which has two reserved
   * 12-bit values).  And the total number of FAT sectors needed is:
   *
   *   nfatsects = nfats * (1.5 * (ndataclust + 2) / sectorsize)
   *
   * where:
   *
   *   ndataclust = ndatasect / clustsize
   *   nvailsects = nfatsects + ndatasect
   *
   * The solution to this set of linear equations is:
   *
   *   nfatsects  = nfats * (3 * navailsects + 6 * clustersize) / 
   *                        (3 * nfats + 2 * sectorsize * clustersize)
   *
   * The numerator would overflow uint32 if:
   *
   *   3 * navailsects + 6 * clustersize > 0xffffffff
   *
   * Or 
   *
   *   navailsects > 0x55555555 - 2 * clustersize
   */

  if (navailsects <= (0x55555555 - (1 << (fmt->ff_clustshift + 1))))
    {
      denom = (fmt->ff_nfats << 1) + fmt->ff_nfats
            + (var->fv_sectorsize << (fmt->ff_clustshift + 1));
      numer = (navailsects << 1) + navailsects
            + (1 << (fmt->ff_clustshift + 2)) + (1 << (fmt->ff_clustshift + 1));
      return fmt->ff_nfats * (numer + denom - 1) / denom;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name:  mkfatfs_nfatsect16
 *
 * Description:
 *   Calculate the number of sectors need for the fat in a FAT16 file system.
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *   navailsects - The number of sectors available for both FAT and data.
 *     This is a precalculated value equal to the total number of sectors
 *     minus the number of root directory sectors and minus the number of
 *     reserved sectors.
 *
 * Return:
 *    The total size of the FAT in sectors.
 *
 ****************************************************************************/
static inline uint32 mkfatfs_nfatsect16(FAR struct fat_format_s *fmt,
                                        FAR struct fat_var_s *var,
                                        uint32 navailsects)
{
   uint32 denom;
   uint32 numer;

  /* For FAT16, the cluster number is held in a 16-bit number or 2 bytes per
   * cluster reference.  So each FAT sector will hold sectorsize/2 cluster
   * references (except for the first sector of each FAT which has two reserved
   * 16-bit values).  And the total number of FAT sectors needed is:
   *
   *   nfatsects = nfats * (2 * (ndataclust + 2) / sectorsize)
   *
   * where:
   *
   *   ndataclust = ndatasect / clustsize
   *   nvailsects = nfatsects + ndatasect
   *
   * The solution to this set of linear equations is:
   *
   *   nfatsects  = nfats * (navailsects + 2 * clustersize) / 
   *                        (nfats + sectorsize * clustersize / 2)
   *
   * Overflow in the calculation of the numerator could occur if:
   *
   *   navailsects > 0xffffffff - 2 * clustersize
   */

  if (fmt->ff_clustshift == 0)
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize >> 1);
      numer = navailsects + 2;
    }
  else
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize << (fmt->ff_clustshift - 1));
      numer = navailsects + (1 << (fmt->ff_clustshift + 1));
    }
  return fmt->ff_nfats * (numer + denom - 1) / denom;
}

/****************************************************************************
 * Name:  mkfatfs_nfatsect32
 *
 * Description:
 *   Calculate the number of sectors need for the fat in a FAT32 file system.
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *   navailsects - The number of sectors available for both FAT and data.
 *     This is a precalculated value equal to the total number of sectors
 *     minus the number of root directory sectors and minus the number of
 *     reserved sectors.
 *
 * Return:
 *   The total size of the FAT in sectors.
 *
 ****************************************************************************/
static inline uint32 mkfatfs_nfatsect32(FAR struct fat_format_s *fmt,
                                        FAR struct fat_var_s *var,
                                        uint32 navailsects)
{
   uint32 denom;
   uint32 numer;

  /* For FAT32, the cluster number is held in a 32-bit number or 4 bytes per
   * cluster reference.  So each FAT sector will hold sectorsize/4 cluster
   * references (except for the first sector of each FAT which has three reserved
   * 32-bit values).  And the total number of FAT sectors needed is:
   *
   *   nfatsects = nfats * (4 * (ndataclust + 3) / sectorsize)
   *
   * where:
   *
   *   ndataclust = ndatasect / clustsize
   *   nvailsects = nfatsects + ndatasect
   *
   * The solution to this set of linear equations is:
   *
   *   nfatsects  = nfats * (navailsects + 3 * clustersize) / 
   *                        (nfats + sectorsize * clustersize / 4)
   *
   * Overflow in the calculation of the numerator could occur if:
   *
   *   navailsects > 0xffffffff - 3 * clustersize
   */

  if (fmt->ff_clustshift == 0)
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize >> 2);
      numer = navailsects + 3;
    }
  else if (fmt->ff_clustshift == 1)
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize >> 1);
      numer = navailsects + 6;
    }
  else
    {
      denom = fmt->ff_nfats + (var->fv_sectorsize << (fmt->ff_clustshift - 2));
      numer = navailsects + (1 << (fmt->ff_clustshift + 1)) + (1 << fmt->ff_clustshift);
    }
   return fmt->ff_nfats * (numer + denom - 1) / denom;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfatfs_configfatfs
 *
 * Description:
 *   Based on the geometry of the block device and upon the caller-selected
 *   values, configure the FAT filesystem for the device.
 *
 * Input:
 *    fmt  - Caller specified format parameters
 *    var  - Holds disk geomtry data.  Also, the location to return FAT
 *           configuration data
 *
 * Return:
 *    Zero on success; negated errno on failure
 *
 ****************************************************************************/
int mkfatfs_configfatfs(FAR struct fat_format_s *fmt,
                        FAR struct fat_var_s *var)
{
  return OK;
}

