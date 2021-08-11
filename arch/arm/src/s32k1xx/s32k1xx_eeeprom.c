/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_eeeprom.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "hardware/s32k1xx_ftfc.h"
#include "hardware/s32k1xx_sim.h"

#include "s32k1xx_config.h"
#include "s32k1xx_eeeprom.h"
#include "arm_internal.h"

#include <arch/board/board.h> /* Include last:  has dependencies */

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct eeed_struct_s
{
  uint32_t eeed_nsectors;         /* Number of sectors on device */
  uint16_t eeed_sectsize;         /* The size of one sector */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  uint8_t eeed_crefs;             /* Open reference count */
#endif
  uint8_t *eeed_buffer;       /* FlexRAM memory */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void wait_ftfc_ready(void);
static uint32_t execute_ftfc_command(void);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     eeed_open(struct inode *inode);
static int     eeed_close(struct inode *inode);
#endif

static ssize_t eeed_read(struct inode *inode, unsigned char *buffer,
                 blkcnt_t start_sector, unsigned int nsectors);
static ssize_t eeed_write(struct inode *inode,
                 const unsigned char *buffer, blkcnt_t start_sector,
                 unsigned int nsectors);

static int     eeed_geometry(struct inode *inode,
                 struct geometry *geometry);
static int     eeed_ioctl(struct inode *inode, int cmd,
                 unsigned long arg);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     eeed_unlink(struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

/****************************************************************************
 * Name: eeed_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int eeed_open(struct inode *inode)
{
  struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct eeed_struct_s *)inode->i_private;

  /* Increment the open reference count */

  dev->eeed_crefs++;
  DEBUGASSERT(dev->eeed_crefs > 0);

  finfo("eeed_crefs: %d\n", dev->eeed_crefs);
  return OK;
}
#endif

/****************************************************************************
 * Name: eeed_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int eeed_close(struct inode *inode)
{
  struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct eeed_struct_s *)inode->i_private;

  /* Increment the open reference count */

  DEBUGASSERT(dev->eeed_crefs > 0);
  dev->eeed_crefs--;
  finfo("eeed_crefs: %d\n", dev->eeed_crefs);

  return OK;
}
#endif

/****************************************************************************
 * Name: eeed_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t eeed_read(struct inode *inode, unsigned char *buffer,
                         blkcnt_t start_sector, unsigned int nsectors)
{
  struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct eeed_struct_s *)inode->i_private;

  finfo("sector: %" PRIu64 " nsectors: %u sectorsize: %d\n",
        start_sector, nsectors, dev->eeed_sectsize);

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

/****************************************************************************
 * Name: eeed_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

static ssize_t eeed_write(struct inode *inode,
                          const unsigned char *buffer,
                          blkcnt_t start_sector, unsigned int nsectors)
{
  struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct eeed_struct_s *)inode->i_private;

  finfo("sector: %" PRIu64 " nsectors: %u sectorsize: %d\n",
        start_sector, nsectors, dev->eeed_sectsize);

  if (start_sector < dev->eeed_nsectors &&
           start_sector + nsectors <= dev->eeed_nsectors)
    {
      finfo("Transfer %d bytes to %p\n",
             nsectors * dev->eeed_sectsize,
             &dev->eeed_buffer[start_sector * dev->eeed_sectsize]);

      uint32_t *dest = (uint32_t *)&dev->eeed_buffer
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

/****************************************************************************
 * Name: eeed_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int eeed_geometry(struct inode *inode, struct geometry *geometry)
{
  struct eeed_struct_s *dev;

  finfo("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (struct eeed_struct_s *)inode->i_private;

      memset(geometry, 0, sizeof(*geometry));

      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
      geometry->geo_writeenabled  = true;
      geometry->geo_nsectors      = dev->eeed_nsectors;
      geometry->geo_sectorsize    = dev->eeed_sectsize;

      finfo("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");
      finfo("nsectors: %" PRIuOFF " sectorsize: %" PRIu16 "\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: eeed_ioctl
 *
 * Description:
 *   Return device geometry
 *
 ****************************************************************************/

static int eeed_ioctl(struct inode *inode, int cmd, unsigned long arg)
{
  struct eeed_struct_s *dev;
  void **ppv = (void**)((uintptr_t)arg);

  finfo("Entry\n");

  /* Only one ioctl command is supported */

  DEBUGASSERT(inode && inode->i_private);
  if (cmd == BIOC_XIPBASE && ppv)
    {
      dev  = (struct eeed_struct_s *)inode->i_private;
      *ppv = (void *)dev->eeed_buffer;

      finfo("ppv: %p\n", *ppv);
      return OK;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Name: eeed_unlink
 *
 * Description:
 *   The block driver has been unlinked.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int eeed_unlink(struct inode *inode)
{
  struct eeed_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct eeed_struct_s *)inode->i_private;

  /* And free the block driver itself */

  kmm_free(dev);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_eeeprom_register
 *
 * Description:
 *   Non-standard function to register a eeeprom
 *
 * Input Parameters:
 *   minor:     Selects suffix of device named /dev/eeepromN, N={1,2,3...}
 *   size:      The size of eeprom in bytes
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

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
      dev->eeed_buffer       = (uint8_t *)S32K1XX_FTFC_EEEPROM_BASE;

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

/****************************************************************************
 * Name: s32k1xx_eeeprom_init
 *
 * Description:
 *   Init FTFC flash controller to run in Enhanced EEPROM mode
 *
 *
 ****************************************************************************/

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
