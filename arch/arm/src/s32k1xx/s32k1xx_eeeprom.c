/******************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_eeeprom.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include "arm_arch.h"

#include "hardware/s32k1xx_ftfc.h"
#include "hardware/s32k1xx_sim.h"

#include "s32k1xx_config.h"
#include "s32k1xx_eeeprom.h"

#include "arm_internal.h"

#include <arch/board/board.h> /* Include last:  has dependencies */

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

/******************************************************************************
 * Private Types
 ******************************************************************************/

struct eeed_struct_s
{
  uint32_t eeed_nsectors;         /* Number of sectors on device */
  uint16_t eeed_sectsize;         /* The size of one sector */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  uint8_t eeed_crefs;             /* Open reference count */
#endif
  FAR uint8_t *eeed_buffer;       /* FlexRAM memory */
};

/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     eeed_open(FAR struct inode *inode);
static int     eeed_close(FAR struct inode *inode);
#endif

static ssize_t eeed_read(FAR struct inode *inode, FAR unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
static ssize_t eeed_write(FAR struct inode *inode,
                 FAR const unsigned char *buffer, size_t start_sector,
                 unsigned int nsectors);

static int     eeed_geometry(FAR struct inode *inode,
                 FAR struct geometry *geometry);
static int     eeed_ioctl(FAR struct inode *inode, int cmd,
                 unsigned long arg);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     eeed_unlink(FAR struct inode *inode);
#endif

/******************************************************************************
 * Private Data
 ******************************************************************************/

static const struct block_operations g_bops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  eeed_open,     /* open     */
  eeed_close,    /* close    */
#else
  0,           /* open     */
  0,           /* close    */
#endif
  eeed_read,     /* read     */
  eeed_write,    /* write    */
  eeed_geometry, /* geometry */
  eeed_ioctl,    /* ioctl    */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  eeed_unlink    /* unlink   */
#endif
};

/******************************************************************************
 * Private Functions
 ******************************************************************************/

static inline void wait_ftfc_ready()
{
  while ((getreg8(S32K1XX_FTFC_FSTAT) & FTTC_FSTAT_CCIF) == 0)
    {
      /* Busy */
    }
}

static uint32_t execute_ftfc_command()
{
  uint8_t regval;
  uint32_t retval = 0;

  /* Clear CCIF to launch command */

  regval = getreg8(S32K1XX_FTFC_FSTAT);
  regval |= FTTC_FSTAT_CCIF;
  putreg8(regval, S32K1XX_FTFC_FSTAT);

  wait_ftfc_ready();

  retval = getreg8(S32K1XX_FTFC_FSTAT);

  if (retval & (FTTC_FSTAT_MGSTAT0 | FTTC_FSTAT_FPVIOL |
                FTTC_FSTAT_ACCERR | FTTC_FSTAT_RDCOLERR))
    {
      return retval; /* Error has occured */
    }

  return retval;
}

/******************************************************************************
 * Name: eeed_open
 *
 * Description: Open the block device
 *
 ******************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int eeed_open(FAR struct inode *inode)
{
  FAR struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct eeed_struct_s *)inode->i_private;

  /* Increment the open reference count */

  dev->eeed_crefs++;
  DEBUGASSERT(dev->eeed_crefs > 0);

  finfo("eeed_crefs: %d\n", dev->eeed_crefs);
  return OK;
}
#endif

/******************************************************************************
 * Name: eeed_close
 *
 * Description: close the block device
 *
 ******************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int eeed_close(FAR struct inode *inode)
{
  FAR struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct eeed_struct_s *)inode->i_private;

  /* Increment the open reference count */

  DEBUGASSERT(dev->eeed_crefs > 0);
  dev->eeed_crefs--;
  finfo("eeed_crefs: %d\n", dev->eeed_crefs);

  return OK;
}
#endif

/******************************************************************************
 * Name: eeed_read
 *
 * Description:  Read the specified number of sectors
 *
 ******************************************************************************/

static ssize_t eeed_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors)
{
  FAR struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct eeed_struct_s *)inode->i_private;

  finfo("sector: %d nsectors: %d sectorsize: %d\n",
        start_sector, dev->eeed_sectsize, nsectors);

  if (start_sector < dev->eeed_nsectors &&
      start_sector + nsectors <= dev->eeed_nsectors)
    {
       finfo("Transfer %d bytes from %p\n",
             nsectors * dev->eeed_sectsize,
             &dev->eeed_buffer[start_sector * dev->eeed_sectsize]);

       wait_ftfc_ready();

       memcpy(buffer,
             &dev->eeed_buffer[start_sector * dev->eeed_sectsize],
             nsectors * dev->eeed_sectsize);
      return nsectors;
    }

  return -EINVAL;
}

/******************************************************************************
 * Name: eeed_write
 *
 * Description: Write the specified number of sectors
 *
 ******************************************************************************/

static ssize_t eeed_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct eeed_struct_s *)inode->i_private;

  finfo("sector: %d nsectors: %d sectorsize: %d\n",
        start_sector, dev->eeed_sectsize, nsectors);

  if (start_sector < dev->eeed_nsectors &&
           start_sector + nsectors <= dev->eeed_nsectors)
    {
      finfo("Transfer %d bytes to %p\n",
             nsectors * dev->eeed_sectsize,
             &dev->eeed_buffer[start_sector * dev->eeed_sectsize]);

      FAR uint32_t *dest = (FAR uint32_t *)&dev->eeed_buffer
                           [start_sector * dev->eeed_sectsize];

      uint32_t *src = (uint32_t *)buffer;

      for (int i = 0; i < nsectors; i++)
        {
          wait_ftfc_ready();
          *dest = *src;
          dest++;
          src++;
        }

      return nsectors;
    }

  return -EFBIG;
}

/******************************************************************************
 * Name: eeed_geometry
 *
 * Description: Return device geometry
 *
 ******************************************************************************/

static int eeed_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  struct eeed_struct_s *dev;

  finfo("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (struct eeed_struct_s *)inode->i_private;
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
      geometry->geo_writeenabled  = true;
      geometry->geo_nsectors      = dev->eeed_nsectors;
      geometry->geo_sectorsize    = dev->eeed_sectsize;

      finfo("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");
      finfo("nsectors: %d sectorsize: %d\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);

      return OK;
    }

  return -EINVAL;
}

/******************************************************************************
 * Name: eeed_ioctl
 *
 * Description:
 *   Return device geometry
 *
 ******************************************************************************/

static int eeed_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR struct eeed_struct_s *dev;
  FAR void **ppv = (void**)((uintptr_t)arg);

  finfo("Entry\n");

  /* Only one ioctl command is supported */

  DEBUGASSERT(inode && inode->i_private);
  if (cmd == BIOC_XIPBASE && ppv)
    {
      dev  = (FAR struct eeed_struct_s *)inode->i_private;
      *ppv = (FAR void *)dev->eeed_buffer;

      finfo("ppv: %p\n", *ppv);
      return OK;
    }

  return -ENOTTY;
}

/******************************************************************************
 * Name: eeed_unlink
 *
 * Description:
 *   The block driver has been unlinked.
 *
 ******************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int eeed_unlink(FAR struct inode *inode)
{
  FAR struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct eeed_struct_s *)inode->i_private;

  /* And free the block driver itself */

  kmm_free(dev);

  return OK;
}
#endif

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: s32k1xx_eeeprom_register
 *
 * Description:
 *   Non-standard function to register a eeeprom
 *
 * Input Parameters:
 *   minor:         Selects suffix of device named /dev/eeepromN, N={1,2,3...}
 *   size:          The size of eeprom in bytes
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ******************************************************************************/

int s32k1xx_eeeprom_register(int minor, uint32_t size)
{
  struct eeed_struct_s *dev;
  char devname[16];
  int ret = -ENOMEM;

  /* Allocate a eeeprom device structure */

  dev = (struct eeed_struct_s *)kmm_zalloc(sizeof(struct eeed_struct_s));
  if (dev)
    {
      /* Initialize the eeeprom device structure */

      dev->eeed_nsectors     = size / 4;     /* Number of sectors on device */
      dev->eeed_sectsize     = 4;            /* The size of one sector */
      dev->eeed_buffer       = (FAR uint8_t *)S32K1XX_FTFC_EEEPROM_BASE;

      /* Create a eeeprom device name */

      snprintf(devname, 16, "/dev/eeeprom%d", minor);

      /* Inode private data is a reference to the eeeprom device structure */

      ret = register_blockdriver(devname, &g_bops, 0, dev);
      if (ret < 0)
        {
          ferr("register_blockdriver failed: %d\n", -ret);
          kmm_free(dev);
        }
    }

  return ret;
}

/******************************************************************************
 * Name: s32k1xx_eeeprom_init
 *
 * Description:
 *   Init FTFC flash controller to run in Enhanced EEPROM mode
 *
 *
 ******************************************************************************/

void s32k1xx_eeeprom_init()
{
  uint32_t regval;

  regval = getreg32(S32K1XX_SIM_FCFG1);

  /* If the FlexNVM memory has not been partitioned */

  if (((regval & SIM_FCFG1_DEPART_MASK) >> SIM_FCFG1_DEPART_SHIFT) == 0xf)
    {
      /* Setup D-flash partitioning for use with EEEPROM */

      putreg8(S32K1XX_FTFC_PROGRAM_PARTITION, S32K1XX_FTFC_FCCOB0); /* Command */

      putreg8(0x0, S32K1XX_FTFC_FCCOB1); /* CSEc key size */
      putreg8(0x0, S32K1XX_FTFC_FCCOB2); /* uSFE */
      putreg8(0x0, S32K1XX_FTFC_FCCOB3); /* Load FlexRAM EEE */
      putreg8(0x2, S32K1XX_FTFC_FCCOB4); /* EEE Partition code (4KB) */
      putreg8(0x8, S32K1XX_FTFC_FCCOB5); /* DE  Partition code (64KB FlexNVM as
                                          * EEEPROM backup
                                          */

      execute_ftfc_command();
    }

  /* Enable FlexRAM for use with EEPROM */

  putreg8(S32K1XX_FTFC_SET_FLEXRAM_FUNCTION, S32K1XX_FTFC_FCCOB0); /* Command */

  putreg8(0x0, S32K1XX_FTFC_FCCOB1); /* FlexRAM used for EEEPROM */

  execute_ftfc_command();

  /* Wait for Emulated EEPROM to be ready */

  while ((getreg8(S32K1XX_FTFC_FCNFG) & FTTC_FCNFG_EEERDY) == 0)
    {
      /* Busy */
    }
}
