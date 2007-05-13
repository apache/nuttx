/****************************************************************************
 * fs_fat32.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdlib.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs.h>

#include "fs_internal.h"
#include "fs_fat32.h"

#if CONFIG_FS_FAT

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Access to data in raw sector data */

#define UBYTE_VAL(p,o)            (((ubyte*)(p))[o])
#define UBYTE_PTR(p,o)            &UBYTE_VAL(p,o)
#define UBYTE_PUT(p,o,v)          (UBYTE_VAL(p,o)=(ubyte)(v))

#define UINT16_PTR(p,o)           ((uint16*)UBYTE_PTR(p,o))
#define UINT16_VAL(p,o)           (*UINT16_PTR(p,o))
#define UINT16_PUT(p,o,v)         (UINT16_VAL(p,o)=(uint16)(v))

#define UINT32_PTR(p,o)           ((uint32*)UBYTE_PTR(p,o))
#define UINT32_VAL(p,o)           (*UINT32_PTR(p,o))
#define UINT32_PUT(p,o,v)         (UINT32_VAL(p,o)=(uint32)(v))

/* Regardless of the endian-ness of the target or alignment of the data, no
 * special operations are required for byte, string or byte array accesses.
 * The FAT data stream is little endian so multiple byte values must be
 * accessed byte-by-byte for big-endian targets.
 */

#define MBR_GETSECPERCLUS(p)      UBYTE_VAL(p,BS_SECPERCLUS)
#define MBR_GETNUMFATS(p)         UBYTE_VAL(p,BS_NUMFATS)
#define MBR_GETMEDIA(p)           UBYTE_VAL(p,BS_MEDIA)
#define MBR_GETDRVNUM(p)          UBYTE_VAL(p,BS32_DRVNUM)
#define MBR_GETBOOTSIG(p)         UBYTE_VAL(p,BS32_BOOTSIG)

#define MBR_PUTSECPERCLUS(p,v)    UBYTE_PUT(p,BS_SECPERCLUS),v)
#define MBR_PUTNUMFATS(p,v)       UBYTE_PUT(p,BS_NUMFATS,v)
#define MBR_PUTMEDIA(p,v)         UBYTE_PUT(p,BS_MEDIA,v)
#define MBR_PUTDRVNUM(p,v)        UBYTE_PUT(p,BS32_DRVNUM,v)
#define MBR_PUTBOOTSIG(p,v)       UBYTE_PUT(p,BS32_BOOTSIG,v)

/* For the all targets, unaligned values need to be accessed byte-by-byte.
 * Some architectures may handle unaligned accesses with special interrupt
 * handlers.  But even in that case, it is more efficient to avoid the traps.
 */

/* Unaligned multi-byte access macros */

#define MBR_GETBYTESPERSEC(p)      fat_getuint16(UBYTE_PTR(p,BS_BYTESPERSEC))
#define MBR_GETROOTENTCNT(p)       fat_getuint16(UBYTE_PTR(p,BS_ROOTENTCNT))
#define MBR_GETTOTSEC16(p)         fat_getuint16(UBYTE_PTR(p,BS_TOTSEC16))
#define MBR_GETVOLID(p)            fat_getuint32(UBYTE_PTR(p,BS32_VOLID))

#define MBR_PUTBYTESPERSEC(p,v)    fat_putuint16(UBYTE_PTR(p,BS_BYTESPERSEC),v)
#define MBR_PUTROOTENTCNT(p,v)     fat_putuint16(UBYTE_PTR(p,BS_ROOTENTCNT),v)
#define MBR_PUTTOTSEC16(p,v)       fat_putuint16(UBYTE_PTR(p,BS_TOTSEC16),v)
#define MBR_PUTVOLID(p,v)          fat_putuint32(UBYTE_PTR(p,BS32_VOLID),v)

/* But for multi-byte values, the endian-ness of the target vs. the little
 * endian order of the byte stream or alignment of the data within the byte
 * stream can force special, byte-by-byte accesses.
 */

#ifdef CONFIG_ARCH_BIGENDIAN
/* If the target is big-endian, then even aligned multi-byte values must be
 * accessed byte-by-byte.
 */

# define MBR_GETRESVDSECCOUNT(p)   fat_getuint16(UBYTE_PTR(p,BS_RESVDSECCOUNT))
# define MBR_GETFATSZ16(p)         fat_getuint16(UBYTE_PTR(p,BS_FATSZ16))
# define MBR_GETSECPERTRK(p)       fat_getuint16(UBYTE_PTR(p,BS_SECPERTRK))
# define MBR_GETNUMHEADS(p)        fat_getuint16(UBYTE_PTR(p,BS_NUMHEADS))
# define MBR_GETHIDSEC(p)          fat_getuint32(UBYTE_PTR(p,BS_HIDSEC))
# define MBR_GETTOTSEC32(p)        fat_getuint32(UBYTE_PTR(p,BS_TOTSEC32))
# define MBR_GETFATSZ32(p)         fat_getuint32(UBYTE_PTR(p,BS32_FATSZ32))
# define MBR_GETEXTFLAGS(p)        fat_getuint16(UBYTE_PTR(p,BS32_EXTFLAGS))
# define MBR_GETFSVER(p)           fat_getuint16(UBYTE_PTR(p,BS32_FSVER))
# define MBR_GETROOTCLUS(p)        fat_getuint32(UBYTE_PTR(p,BS32_ROOTCLUS))
# define MBR_GETFSINFO(p)          fat_getuint16(UBYTE_PTR(p,BS32_FSINFO))
# define MBR_GETBKBOOTSEC(p)       fat_getuint16(UBYTE_PTR(p,BS32_BKBOOTSEC))
# define MBR_GETSIGNATURE(p)       fat_getuint16(UBYTE_PTR(p,BS_SIGNATURE))

# define MBR_GETPARTSECTOR(s)      fat_getuint32(s);

# define FSI_GETLEADSIG(p)         fat_getuint32(UBYTE_PTR(p,FSI_LEADSIG))
# define FSI_GETSTRUCTSIG(p)       fat_getuint32(UBYTE_PTR(p,FSI_STRUCTSIG))
# define FSI_GETFREECOUNT(p)       fat_getuint32(UBYTE_PTR(p,FSI_FREECOUNT))
# define FSI_GETNXTFREE(p)         fat_getuint32(UBYTE_PTR(p,FSI_NXTFREE))
# define FSI_GETTRAILSIG(p)        fat_getuint32(UBYTE_PTR(p,FSI_TRAILSIG))

# define MBR_PUTRESVDSECCOUNT(p,v) fat_putuint16(UBYTE_PTR(p,BS_RESVDSECCOUNT,v))
# define MBR_PUTFATSZ16(p,v)       fat_putuint16(UBYTE_PTR(p,BS_FATSZ16,v))
# define MBR_PUTSECPERTRK(p,v)     fat_putuint16(UBYTE_PTR(p,BS_SECPERTRK,v))
# define MBR_PUTNUMHEADS(p,v)      fat_putuint16(UBYTE_PTR(p,BS_NUMHEADS,v))
# define MBR_PUTHIDSEC(p,v)        fat_putuint32(UBYTE_PTR(p,BS_HIDSEC,v))
# define MBR_PUTTOTSEC32(p,v)      fat_putuint32(UBYTE_PTR(p,BS_TOTSEC32,v))
# define MBR_PUTFATSZ32(p,v)       fat_putuint32(UBYTE_PTR(p,BS32_FATSZ32,v))
# define MBR_PUTEXTFLAGS(p,v)      fat_putuint16(UBYTE_PTR(p,BS32_EXTFLAGS,v))
# define MBR_PUTFSVER(p,v)         fat_putuint16(UBYTE_PTR(p,BS32_FSVER,v))
# define MBR_PUTROOTCLUS(p,v)      fat_putuint32(UBYTE_PTR(p,BS32_ROOTCLUS,v))
# define MBR_PUTFSINFO(p,v)        fat_putuint16(UBYTE_PTR(p,BS32_FSINFO,v))
# define MBR_PUTBKBOOTSEC(p,v)     fat_putuint16(UBYTE_PTR(p,BS32_BKBOOTSEC,v))
# define MBR_PUTSIGNATURE(p,v)     fat_getuint16(UBYTE_PTR(p,BS_SIGNATURE),v)

# define FSI_PUTLEADSIG(p,v)       fat_putuint32(UBYTE_PTR(p,FSI_LEADSIG),v)
# define FSI_PUTSTRUCTSIG(p,v)     fat_putuint32(UBYTE_PTR(p,FSI_STRUCTSIG),v)
# define FSI_PUTFREECOUNT(p,v)     fat_putuint32(UBYTE_PTR(p,FSI_FREECOUNT),v)
# define FSI_PUTNXTFREE(p,v)       fat_putuint32(UBYTE_PTR(p,FSI_NXTFREE),v)
# define FSI_PUTTRAILSIG(p,v)      fat_putuint32(UBYTE_PTR(p,FSI_TRAILSIG),v)

#else

/* But nothing special has to be done for the little endian-case for access
 * to aligned mulitbyte values.
 */

# define MBR_GETRESVDSECCOUNT(p)   UINT16_VAL(p,BS_RESVDSECCOUNT)
# define MBR_GETFATSZ16(p)         UINT16_VAL(p,BS_FATSZ16)
# define MBR_GETSECPERTRK(p)       UINT16_VAL(p,BS_SECPERTRK)
# define MBR_GETNUMHEADS(p)        UINT16_VAL(p,BS_NUMHEADS)
# define MBR_GETHIDSEC(p)          UINT32_VAL(p,BS_HIDSEC)
# define MBR_GETTOTSEC32(p)        UINT32_VAL(p,BS_TOTSEC32)
# define MBR_GETFATSZ32(p)         UINT32_VAL(p,BS32_FATSZ32)
# define MBR_GETEXTFLAGS(p)        UINT16_VAL(p,BS32_EXTFLAGS)
# define MBR_GETFSVER(p)           UINT16_VAL(p,BS32_FSVER)
# define MBR_GETROOTCLUS(p)        UINT32_VAL(p,BS32_ROOTCLUS)
# define MBR_GETFSINFO(p)          UINT16_VAL(p,BS32_FSINFO)
# define MBR_GETBKBOOTSEC(p)       UINT16_VAL(p,BS32_BKBOOTSEC)
# define MBR_GETSIGNATURE(p)       UINT16_VAL(p,BS_SIGNATURE)

# define MBR_GETPARTSECTOR(s)      (*((uint32*)(s)))

# define FSI_GETLEADSIG(p)         UINT32_VAL(p,FSI_LEADSIG)
# define FSI_GETSTRUCTSIG(p)       UINT32_VAL(p,FSI_STRUCTSIG)
# define FSI_GETFREECOUNT(p)       UINT32_VAL(p,FSI_FREECOUNT)
# define FSI_GETNXTFREE(p)         UINT32_VAL(p,FSI_NXTFREE)
# define FSI_GETTRAILSIG(p)        UINT32_VAL(p,FSI_TRAILSIG)

# define MBR_PUTRESVDSECCOUNT(p,v) UINT16_PUT(p,BS_RESVDSECCOUNT,v)
# define MBR_PUTFATSZ16(p,v)       UINT16_PUT(p,BS_FATSZ16,v)
# define MBR_PUTSECPERTRK(p,v)     UINT16_PUT(p,BS_SECPERTRK,v)
# define MBR_PUTNUMHEADS(p,v)      UINT16_PUT(p,BS_NUMHEADS,v)
# define MBR_PUTHIDSEC(p,v)        UINT32_PUT(p,BS_HIDSEC,v)
# define MBR_PUTTOTSEC32(p,v)      UINT32_PUT(p,BS_TOTSEC32,v)
# define MBR_PUTFATSZ32(p,v)       UINT32_PUT(p,BS32_FATSZ32,v)
# define MBR_PUTEXTFLAGS(p,v)      UINT16_PUT(p,BS32_EXTFLAGS,v)
# define MBR_PUTFSVER(p,v)         UINT16_PUT(p,BS32_FSVER,v)
# define MBR_PUTROOTCLUS(p,v)      UINT32_PUT(p,BS32_ROOTCLUS,v)
# define MBR_PUTFSINFO(p,v)        UINT16_PUT(p,BS32_FSINFO,v)
# define MBR_PUTBKBOOTSEC(p,v)     UINT16_PUT(p,BS32_BKBOOTSEC,v)
# define MBR_PUTSIGNATURE(p,v)     UINT16_PUT(p,BS_SIGNATURE,v)

# define FSI_PUTLEADSIG(p)         UINT32_PUT(p,FSI_LEADSIG)
# define FSI_PUTSTRUCTSIG(p)       UINT32_PUT(p,FSI_STRUCTSIG)
# define FSI_PUTFREECOUNT(p)       UINT32_PUT(p,FSI_FREECOUNT)
# define FSI_PUTNXTFREE(p)         UINT32_PUT(p,FSI_NXTFREE)
# define FSI_PUTTRAILSIG(p)        UINT32_PUT(p,FSI_TRAILSIG)

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     fat_open(FAR struct file *filp, const char *rel_path,
                        int oflags, mode_t mode);
static int     fat_close(FAR struct file *filp);
static ssize_t fat_read(FAR struct file *filp, char *buffer, size_t buflen);
static ssize_t fat_write(FAR struct file *filp, const char *buffer,
                         size_t buflen);
static off_t   fat_seek(FAR struct file *filp, off_t offset, int whence);
static int     fat_ioctl(FAR struct file *filp, int cmd, unsigned long arg);
static int     fat_bind(FAR struct inode *blkdriver, const void *data,
                        void **handle);
static int     fat_unbind(void *handle);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations fat_operations =
{
  fat_open,
  fat_close,
  fat_read,
  fat_write,
  fat_seek,
  fat_ioctl,
  fat_bind,
  fat_unbind
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fat_getuint16
 ****************************************************************************/

static uint16 fat_getuint16(ubyte *ptr)
{
#ifdef CONFIG_ARCH_BIGENDIAN
  /* The bytes always have to be swapped if the target is big-endian */

  return ((uint16)ptr[0] << 8) | ptr[1];
#else
  /* Byte-by-byte transfer is still necessary if the address is un-aligned */

  return ((uint16)ptr[1] << 8) | ptr[0];
#endif
}

/****************************************************************************
 * Name: fat_getuint32
 ****************************************************************************/

static uint32 fat_getuint32(ubyte *ptr)
{
#ifdef CONFIG_ARCH_BIGENDIAN
  /* The bytes always have to be swapped if the target is big-endian */

  return ((uint32)fat_getuint16(&ptr[0]) << 16) | fat_getuint16(&ptr[2]);
#else
  /* Byte-by-byte transfer is still necessary if the address is un-aligned */

  return ((uint32)fat_getuint16(&ptr[2]) << 16) | fat_getuint16(&ptr[0]);
#endif
}

/****************************************************************************
 * Name: fat_putuint16
 ****************************************************************************/

static void fat_putuint16(ubyte *ptr, uint16 value16)
{
  ubyte *val = (ubyte*)&value16;
#ifdef CONFIG_ARCH_BIGENDIAN
  /* The bytes always have to be swapped if the target is big-endian */

  ptr[0] = val[1];
  ptr[1] = val[0];
#else
  /* Byte-by-byte transfer is still necessary if the address is un-aligned */

  ptr[0] = val[0];
  ptr[1] = val[1];
#endif
}

/****************************************************************************
 * Name: fat_putuint32
 ****************************************************************************/

static void fat_putuint32(ubyte *ptr, uint32 value32)
{
  uint16 *val = (uint16*)&value32;
#ifdef CONFIG_ARCH_BIGENDIAN
  /* The bytes always have to be swapped if the target is big-endian */

  fat_putuint16(&ptr[0], val[2]);
  fat_putuint16(&ptr[2], val[0]);
#else
  /* Byte-by-byte transfer is still necessary if the address is un-aligned */

  fat_putuint16(&ptr[0], val[0]);
  fat_putuint16(&ptr[2], val[2]);
#endif
}

/****************************************************************************
 * Name: fat_semtake
 ****************************************************************************/

static void fat_semtake(struct fat_mountpt_s *fs)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&fs->fs_sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

/****************************************************************************
 * Name: fat_semgive
 ****************************************************************************/

static inline void fat_semgive(struct fat_mountpt_s *fs)
{
   sem_post(&fs->fs_sem);
}

/****************************************************************************
 * Name: fat_checkmount
 *
 * Desciption: Check if the mountpoint is still valid.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

static int fat_checkmount(struct fat_mountpt_s *fs)
{
  /* If the fs_mounted flag is FALSE, then we have already handled the loss
   * of the mount.
   */

  if (fs->fs_mounted)
    {
      struct fat_file_s *file;

      /* We still think the mount is healthy.  Check an see if this is
       * still the case
       */

      if (fs->fs_blkdriver)
        {
          struct inode *inode = fs->fs_blkdriver;
          if (inode && inode->u.i_bops && inode->u.i_bops->geometry)
            {
              struct geometry geo;
              int errcode = inode->u.i_bops->geometry(inode, &geo);
              if (errcode == OK && geo.geo_available && !geo.geo_mediachanged)
                {
                  return OK;
                }
            }
        }

      /* If we get here, the mount is NOT healthy */

      fs->fs_mounted = FALSE;

      /* Make sure that this is flagged in every opened file */

      for (file = fs->fs_head; file; file = file->ff_next)
      {
          file->ff_open = FALSE;
      }
    }
  return -ENODEV;
}

/****************************************************************************
 * Name: fat_bread
 *
 * Desciption: Read the specified sector
 *
 ****************************************************************************/

static int fat_bread(struct fat_mountpt_s *fs, size_t sector)
{
  int ret = -ENODEV;
  if (fs && fs->fs_blkdriver )
    {
      struct inode *inode = fs->fs_blkdriver;
      if (inode && inode->u.i_bops && inode->u.i_bops->read)
        {
          ssize_t nSectorsRead = inode->u.i_bops->read(inode, fs->fs_buffer,
                                                       sector, 1);
          if (nSectorsRead == 1)
            {
              ret = OK;
            }
          else if (nSectorsRead < 0)
            {
              ret = nSectorsRead;
            }
        }
    }
  return ret;
}

/****************************************************************************
 * Name: fat_readfsinfo
 *
 * Desciption: Read the FAT32 FSINFO sector
 *
 ****************************************************************************/

static int fat_readfsinfo(struct fat_mountpt_s *fs)
{
  /* Verify that this is, indeed, an FSINFO sector */

  if (FSI_GETLEADSIG(fs->fs_buffer) == 0x41615252  &&
      FSI_GETSTRUCTSIG(fs->fs_buffer) == 0x61417272 &&
      FSI_GETTRAILSIG(fs->fs_buffer) == 0xaa550000)
    {
      fs->fs_fsinextfree  = FSI_GETFREECOUNT(fs->fs_buffer);
      fs->fs_fsifreecount = FSI_GETNXTFREE(fs->fs_buffer);
      return OK;
    }
  return -ENODEV;
}

/****************************************************************************
 * Name: fat_checkbootrecord
 *
 * Desciption: Read a sector and verify that it is a a FAT boot record.
 *
 ****************************************************************************/

static int fat_checkbootrecord(struct fat_mountpt_s *fs)
{
  uint32 ndatasectors;
  uint32 fatsize;

  /* Verify the MBR signature at offset 510 in the sector (true even
   * if the sector size is greater than 512.  All FAT file systems have
   * this signature. On a FAT32 volume, the RootEntCount , FatSz16, and
   * FatSz32 values should always be zero.  The FAT sector size should
   * match the reported hardware sector size.
   */

  if (MBR_GETSIGNATURE(fs->fs_buffer) != 0xaa55 ||
      MBR_GETROOTENTCNT(fs->fs_buffer) != 0 ||
      MBR_GETFATSZ16(fs->fs_buffer) != 0 ||
      MBR_GETTOTSEC16(fs->fs_buffer) != 0 ||
      MBR_GETBYTESPERSEC(fs->fs_buffer) != fs->fs_hwsectorsize)
    {
      return -ENODEV;
    }

  /* Verify the FAT32 file system type. The determination of the file
   * system type is based on the number of clusters on the volume:  FAT12
   * volume has < 4085 cluseter, a FAT16 volume has fewer than 65,525
   * clusters, and any larger is FAT32.
   *
   * Determine the number of sectors in a FAT.
   */

  fs->fs_fatsize = MBR_GETFATSZ32(fs->fs_buffer);
  if (fs->fs_fatsize >= fs->fs_hwnsectors)
    {
      return -ENODEV;
    }

  /* Get the total number of sectors on the volume. */

  fs->fs_fattotsec  = MBR_GETTOTSEC32(fs->fs_buffer);
  if (fs->fs_fattotsec > fs->fs_hwnsectors)
    {
      return -ENODEV;
    }

  /* Get the total number of reserved sectors */

  fs->fs_fatresvdseccount = MBR_GETRESVDSECCOUNT(fs->fs_buffer);
  if (fs->fs_fatresvdseccount > fs->fs_hwnsectors)
    {
      return -ENODEV;
    }

  /* Get the number of FATs. This is probably two but could have other values */

  fs->fs_fatnumfats = MBR_GETNUMFATS(fs->fs_buffer);
  fatsize = fs->fs_fatnumfats * fs->fs_fatsize;

  /* Get the total number of data sectors */

  ndatasectors = fs->fs_fattotsec - fs->fs_fatresvdseccount - fatsize;
  if (ndatasectors > fs->fs_hwnsectors)
    {
      return -ENODEV;
    }

  /* Get the sectors per cluster */

  fs->fs_fatsecperclus = MBR_GETSECPERCLUS(fs->fs_buffer);

  /* Calculate the number of clusters */

  fs->fs_nclusters = ndatasectors / fs->fs_fatsecperclus;

  /* Finally, the test: */

  if (fs->fs_nclusters < 65525)
    {
      return -ENODEV;
    }

  /* We have what appears to be a valid FAT filesystem! Save a few more things
   * from the boot record that we will need later.
   */

  fs->fs_fsinfo       = fs->fs_fatbase + MBR_GETFSINFO(fs->fs_buffer);
  fs->fs_fatbase     += fs->fs_fatresvdseccount;
  fs->fs_database     = fs->fs_fatbase + fatsize; 
  fs->fs_rootclus     = MBR_GETROOTCLUS(fs->fs_buffer);
  fs->fs_fsifreecount = 0xffffffff;

  return OK;
}

/****************************************************************************
 * Name: fat_mount
 *
 * Desciption: This function is called only when the mountpoint is first
 *   established.  It initializes the mountpoint structure and verifies
 *   that a valid FAT32 filesystem is provided by the block driver.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

static int fat_mount(struct fat_mountpt_s *fs, boolean writeable)
{
  FAR struct inode *inode;
  struct geometry geo;
  int ret;

  /* Assume that the mount is successful */

  fs->fs_mounted = TRUE;

  /* Check if there is media available */

  inode = fs->fs_blkdriver;
  if (!inode || !inode->u.i_bops || !inode->u.i_bops->geometry ||
      inode->u.i_bops->geometry(inode, &geo) != OK || !geo.geo_available)
    {
      ret = -ENODEV;
      goto errout;
    }

  /* Make sure that that the media is write-able (if write access is needed) */

  if (writeable && !geo.geo_writeenabled)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Save the hardware geometry */

  fs->fs_hwsectorsize = geo.geo_sectorsize;
  fs->fs_hwnsectors   = geo.geo_nsectors;

  /* Allocate a buffer to hold one hardware sector */

  fs->fs_buffer = (ubyte*)malloc(fs->fs_hwsectorsize);
  if (!fs->fs_buffer)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Search FAT boot record on the drive.  First check at sector zero.  This
   * could be either the boot record or a partition that refers to the boot
   * record.
   *
   * First read sector zero.  This will be the first access to the drive and a
   * likely failure point.
   */

  fs->fs_fatbase = 0;
  ret = fat_bread(fs, 0);
  if (ret < 0)
    {
      goto errout_with_buffer;
    }

  if (fat_checkbootrecord(fs) != OK)
    {
      /* The contents of sector 0 is not a boot record.  It could be a
       * partition, however.  Assume it is a partition and get the offset
       * into the partition table.  This table is at offset MBR_TABLE and is
       * indexed by 16x the partition number.  Here we support only
       * parition 0.
       */

      ubyte *partition = &fs->fs_buffer[MBR_TABLE + 0];

      /* Check if the partition exists and, if so, get the bootsector for that
       * partition and see if we can find the boot record there.
       */
 
      if (partition[4])
        {
          /* There appears to be a partition, get the sector number of the
           * partition (LBA)
           */

          fs->fs_fatbase = MBR_GETPARTSECTOR(&partition[8]);

          /* Read the new candidate boot sector */

          ret = fat_bread(fs, 0);
          if (ret < 0)
          {
              goto errout_with_buffer;
          }

          /* Check if this is a boot record */

          if (fat_checkbootrecord(fs) != OK)
            {
              goto errout_with_buffer;
            }
        }
    }

  /* We have what appears to be a valid FAT filesystem! Now read the
   * FSINFO sector.
   */

  ret = fat_readfsinfo(fs);
  if (ret != OK)
  {
    goto errout_with_buffer;
  }

  /* We did it! */

  return OK;

 errout_with_buffer:
  free(fs->fs_buffer);
  fs->fs_buffer = 0;
 errout:
  fs->fs_mounted = FALSE;
  return ret;
}

/****************************************************************************
 * Name: fat_open
 ****************************************************************************/

static int fat_open(FAR struct file *filp, const char *rel_path,
                    int oflags, mode_t mode)
{
  struct fat_mountpt_s *fs = filp->f_priv;
  int ret;
 
  /* Make sure that the mount is still healthy */

 ret = fat_checkmount(fs);
  if (ret != OK)
    {
      return ret;
    }
  return -ENOSYS;
}

/****************************************************************************
 * Name: fat_close
 ****************************************************************************/

static int fat_close(FAR struct file *filp)
{
  struct fat_mountpt_s *fs = filp->f_priv;

  /* Do not check if the mount is healthy.  We must support closing of
   * the file even when there is healthy mount.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: fat_read
 ****************************************************************************/

static ssize_t fat_read(FAR struct file *filp, char *buffer, size_t buflen)
{
  struct fat_mountpt_s *fs = filp->f_priv;
  int ret;

  /* Make sure that the mount is still healthy */

  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      return ret;
    }
  return -ENOSYS;
}

/****************************************************************************
 * Name: fat_write
 ****************************************************************************/

static ssize_t fat_write(FAR struct file *filp, const char *buffer,
                         size_t buflen)
{
  struct fat_mountpt_s *fs = filp->f_priv;
  int ret;

  /* Make sure that the mount is still healthy */

  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      return ret;
    }
  return -ENOSYS;
}

/****************************************************************************
 * Name: fat_seek
 ****************************************************************************/

static off_t fat_seek(FAR struct file *filp, off_t offset, int whence)
{
  struct fat_mountpt_s *fs = filp->f_priv;
  int ret;

  /* Make sure that the mount is still healthy */

  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      return ret;
    }
  return -ENOSYS;
}

/****************************************************************************
 * Name: fat_ioctl
 ****************************************************************************/

static int fat_ioctl(FAR struct file *filp, int cmd, unsigned long arg)
{
  struct fat_mountpt_s *fs = filp->f_priv;
  int ret;

  /* Make sure that the mount is still healthy */

  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      return ret;
    }

  /* ioctl calls are just passed through to the contained block driver */
  return -ENOSYS;
}

/****************************************************************************
 * Name: fat_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int fat_bind(FAR struct inode *blkdriver, const void *data,
                    void **handle)
{
  struct fat_mountpt_s *fs;
  int ret;

  /* Create an instance of the mountpt state structure */
  fs = (struct fat_mountpt_s *)zalloc(sizeof(struct fat_mountpt_s));
  if ( !fs )
    {
      return -ENOMEM;
    }

  /* Initialize the allocated mountpt state structure */

  fs->fs_blkdriver = blkdriver;
  sem_init(&fs->fs_sem, 0, 0);

  /* Then get information about the FAT32 filesystem on the devices managed
   * by this block driver.
   */

  ret = fat_mount(fs, TRUE);
  if ( ret != 0 )
    {
      sem_destroy(&fs->fs_sem);
      free(fs);
      return ret;
    }

  *handle = (void*)fs;
  fat_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: fat_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int fat_unbind(void *handle)
{
  struct fat_mountpt_s *fs = (struct fat_mountpt_s*)handle;
  int ret;

  if ( !fs )
    {
      return -EINVAL;
    }

  /* Check if there are sill any files opened on the filesystem. */

  ret = OK; /* Assume success */
  fat_semtake(fs);
  if (fs->fs_head)
    {
      /* We cannot unmount now.. there are open files */

      ret = -EBUSY;
    }
  else
    {
       /* Unmount ... close the block driver */

      if (fs->fs_blkdriver)
        {
          struct inode *inode = fs->fs_blkdriver;
          if (inode && inode->u.i_bops && inode->u.i_bops->close)
            {
              (void)inode->u.i_bops->close(inode);
            }
        }

      /* Release the mountpoint private data */

      if (fs->fs_buffer)
        {
          free(fs->fs_buffer);
        }
      free(fs);
    }

  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_FS_FAT */
