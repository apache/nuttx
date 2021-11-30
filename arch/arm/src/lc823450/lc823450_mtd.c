/****************************************************************************
 * arch/arm/src/lc823450/lc823450_mtd.c
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

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/semaphore.h>
#include <arch/board/board.h>

#include "lc823450_mtd.h"
#include "lc823450_mmcl.h"
#include "lc823450_sdc.h"
#include "lc823450_sddrv_if.h"
#include "lc823450_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_MTD_DEV_MAX > 2
# error "MTD: Too many MTD device"
#endif

#if CONFIG_MTD_DEV_MAX == 2
#  if (CONFIG_MTD_DEVNO_EMMC == CONFIG_MTD_DEVNO_SDC)
#     error "MTD: Invalid devno specified"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct skel_dev_s.
 */

struct lc823450_mtd_dev_s
{
  struct mtd_dev_s mtd;

  /* Other implementation specific data may follow here */

  sem_t sem;            /* Assures mutually exclusive access to the slot */
  uint32_t nblocks;     /* Number of blocks */
  uint32_t blocksize;   /* Size of one read/write blocks */
  uint32_t channel;     /* 0: eMMC, 1: SDC */
};

struct lc823450_partinfo_s
{
  off_t startblock;
  off_t nblocks;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_sem = SEM_INITIALIZER(1);
static FAR struct mtd_dev_s *g_mtdpart[LC823450_NPARTS];
static FAR struct mtd_dev_s *g_mtdmaster[CONFIG_MTD_DEV_MAX];   /* 0: eMMC, 1: SDC */

static const char g_mtdname[2][4] =
{
  "sd",
  "mmc"
};

static struct lc823450_partinfo_s partinfo[LC823450_NPARTS] =
{
  { LC823450_PART1_START,  LC823450_PART1_NBLOCKS, },
  { LC823450_PART2_START,  LC823450_PART2_NBLOCKS, },
  { LC823450_PART3_START,  LC823450_PART3_NBLOCKS, },
  { LC823450_PART4_START,  LC823450_PART4_NBLOCKS, },
  { LC823450_PART5_START,  LC823450_PART5_NBLOCKS, },
  { LC823450_PART6_START,  LC823450_PART6_NBLOCKS, },
  { LC823450_PART7_START,  LC823450_PART7_NBLOCKS, },
  { LC823450_PART8_START,  LC823450_PART8_NBLOCKS, },
  { LC823450_PART9_START,  LC823450_PART9_NBLOCKS, },
  { LC823450_PART10_START, LC823450_PART10_NBLOCKS, },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtd_semtake
 ****************************************************************************/

static int mtd_semtake(FAR sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: mtd_semgive
 ****************************************************************************/

static void mtd_semgive(FAR sem_t *sem)
{
  nxsem_post(sem);
}

/****************************************************************************
 * Name: lc823450_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported.
 *
 ****************************************************************************/

static int lc823450_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks)
{
  finfo("dev=%p startblock=%jd nblocks=%zd\n",
        dev, (intmax_t)startblock, nblocks);
  return OK;
}

/****************************************************************************
 * Name: lc823450_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t lc823450_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR uint8_t *buf)
{
  int ret;
  FAR struct lc823450_mtd_dev_s *priv = (FAR struct lc823450_mtd_dev_s *)dev;

  unsigned long type;

  if (!((uint32_t)buf & 0x3))
    {
      type = SDDR_RW_INC_WORD;
    }
  else if (!((uint32_t)buf & 0x1))
    {
      type = SDDR_RW_INC_HWORD;
    }
  else
    {
      type = SDDR_RW_INC_BYTE;
    }

  finfo("startblockr=%jd, nblocks=%zd buf=%p type=%lx\n",
        (intmax_t)startblock, nblocks, buf, type);

  DEBUGASSERT(dev && buf);

  if (startblock >= priv->nblocks)
    {
      return -EINVAL;
    }

  ret = mtd_semtake(&priv->sem);
  if (ret < 0)
    {
      return ret;
    }

  if (!g_mtdmaster[priv->channel])
    {
      finfo("device removed\n");
      mtd_semgive(&priv->sem);
      return -ENODEV;
    }

  if (startblock + nblocks > priv->nblocks)
    {
      nblocks = priv->nblocks - startblock;
    }

  ret = lc823450_sdc_readsector(priv->channel, (unsigned long)(startblock),
                                (unsigned short)nblocks, buf, type);

  mtd_semgive(&priv->sem);

  if (ret != OK)
    {
      finfo("ERROR: Failed to read sector, ret=%d startblock=%jd "
            "nblocks=%zd\n",
            ret, (intmax_t)startblock, nblocks);
      return ret;
    }

  return (ssize_t)nblocks;
}

/****************************************************************************
 * Name: lc823450_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t lc823450_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, FAR const uint8_t *buf)
{
  int ret;
  FAR struct lc823450_mtd_dev_s *priv = (FAR struct lc823450_mtd_dev_s *)dev;

  unsigned long type;

  if (!((uint32_t)buf & 0x3))
    {
      type = SDDR_RW_INC_WORD;
    }
  else if (!((uint32_t)buf & 0x1))
    {
      type = SDDR_RW_INC_HWORD;
    }
  else
    {
      type = SDDR_RW_INC_BYTE;
    }

  finfo("startblockr=%jd, nblocks=%zd buf=%p type=%lx\n",
        (intmax_t)startblock, nblocks, buf, type);

  DEBUGASSERT(dev && buf);

  if (startblock >= priv->nblocks)
    {
      return -EINVAL;
    }

  ret = mtd_semtake(&priv->sem);
  if (ret < 0)
    {
      return ret;
    }

  if (!g_mtdmaster[priv->channel])
    {
      finfo("device removed\n");
      mtd_semgive(&priv->sem);
      return -ENODEV;
    }

  if (startblock + nblocks > priv->nblocks)
    {
      nblocks = priv->nblocks - startblock;
    }

  ret = lc823450_sdc_writesector(priv->channel, (unsigned long)(startblock),
                                 (unsigned short)nblocks, (void *)buf, type);

  mtd_semgive(&priv->sem);

  if (ret != OK)
    {
      finfo("ERROR: Failed to write sector, ret=%d startblock=%jd "
            "nblocks=%zd\n",
            ret, (intmax_t)startblock, nblocks);
      return ret;
    }

  return (ssize_t)nblocks;
}

/****************************************************************************
 * Name: lc823450_ioctl
 ****************************************************************************/

static int lc823450_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                          unsigned long arg)
{
  int ret;
  FAR struct lc823450_mtd_dev_s *priv = (FAR struct lc823450_mtd_dev_s *)dev;
  FAR struct mtd_geometry_s *geo;
  FAR void **ppv;

  finfo("cmd=%xh, arg=%lxh\n", cmd, arg);

  ret = mtd_semtake(&priv->sem);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;

  if (!g_mtdmaster[priv->channel])
    {
      finfo("device removed\n");
      mtd_semgive(&priv->sem);
      return -ENODEV;
    }

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        finfo("MTDIOC_GEOMETRY\n");
        geo = (FAR struct mtd_geometry_s *)arg;
        if (geo)
          {
            /* Populate the geometry structure with information needed to
             * know the capacity and how to access the device.
             */

            geo->blocksize = priv->blocksize;
            geo->erasesize = priv->blocksize;
            geo->neraseblocks = priv->nblocks;
            ret = OK;
          }

        finfo("blocksize=%" PRId32 " erasesize=%" PRId32
              " neraseblocks=%" PRId32 "\n", geo->blocksize,
              geo->erasesize, geo->neraseblocks);
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = priv->nblocks;
              info->sectorsize  = priv->blocksize;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case BIOC_XIPBASE:
        finfo("BIOC_XIPBASE\n");
        ppv = (FAR void**)arg;
        if (ppv)
          {
            /* If media is directly acccesible, return (void*) base address
             * of device memory.  NULL otherwise.  It is acceptable to omit
             * this case altogether and simply return -ENOTTY.
             */

            *ppv = NULL;
            ret  = OK;
          }
        break;

      case MTDIOC_BULKERASE:
        finfo("MTDIOC_BULKERASE\n");

        /* Erase the entire device */

        ret = OK;
        break;

#ifdef TODO
      case MTDIOC_CIDSTR:
        ret = lc823450_sdc_getcid(priv->channel, (FAR char *)arg, 33);
        break;
#endif

      default:
        finfo("Command not found\n");
        ret = -ENOTTY; /* Bad command */
        break;
    }

  mtd_semgive(&priv->sem);
  return ret;
}

/****************************************************************************
 * Name: mtd_mediainitialize
 *
 * Description:
 *   Detect media and initialize.
 *
 * Precondition:
 *   Semaphore has been taken.
 ****************************************************************************/

static int mtd_mediainitialize(FAR struct lc823450_mtd_dev_s *dev)
{
  int ret;
  unsigned long nblocks;
  unsigned long blocksize;
  uint32_t sysclk = lc823450_get_ahb();

  finfo("enter\n");

  ret = mtd_semtake(&dev->sem);
  if (ret < 0)
    {
      return ret;
    }

  ret = lc823450_sdc_initialize(dev->channel);
  DEBUGASSERT(ret == OK);

  ret = lc823450_sdc_setclock(dev->channel, 20000000, sysclk);

  if (ret != OK)
    {
      finfo("ERROR: Failed to set clock: ret=%d\n", ret);
      goto exit_with_error;
    }

  ret = lc823450_sdc_identifycard(dev->channel);
  if (ret != OK)
    {
      finfo("ERROR: Failed to identify card: channel=%" PRId32 " ret=%d)\n",
            dev->channel, ret);
      goto exit_with_error;
    }

  if (0 == dev->channel)
    {
      /* Try to change to High Speed DDR mode */

      ret = lc823450_sdc_changespeedmode(dev->channel, 4);
      finfo("ch=%" PRId32 " DDR mode ret=%d \n", dev->channel, ret);
    }
  else
    {
#ifdef CONFIG_LC823450_SDC_UHS1
      /* Try to change to DDR50  mode */

      ret = lc823450_sdc_changespeedmode(dev->channel, 4);

      if (0 == ret)
        {
          lldbg("ch=%d DDR50 mode ret=%d \n", dev->channel, ret);
          goto get_card_size;
        }
#endif

      /* Try to change to High Speed mode */

      ret = lc823450_sdc_changespeedmode(dev->channel, 1);

      if (0 == ret)
        {
          ret = lc823450_sdc_setclock(dev->channel, 40000000, sysclk);
          finfo("ch=%" PRId32 " HS mode ret=%d \n", dev->channel, ret);
        }
    }

#ifdef CONFIG_LC823450_SDC_UHS1
get_card_size:
#endif

  ret = lc823450_sdc_getcardsize(dev->channel, &nblocks, &blocksize);
  if (ret != 0)
    {
      finfo("ERROR: No media found\n");
      goto exit_with_error;
    }

  finfo("blocksize=%ld nblocks=%ld\n", blocksize, nblocks);

  dev->nblocks = nblocks;
  dev->blocksize = blocksize;

  /* check if the media type is eMMC:1  */

  if (1 == lc823450_sdc_refmediatype(dev->channel))
    {
      /* cache on */

      lc823450_sdc_cachectl(dev->channel, 1);
    }

  finfo("ch=%" PRId32 " size=%" PRId64 " \n",
        dev->channel, (uint64_t)blocksize * (uint64_t)nblocks);

exit_with_error:
  mtd_semgive(&dev->sem);
  return ret;
}

/****************************************************************************
 * Name: lc823450_mtd_allocdev
 *
 * Description:
 *   Allocate MTD device and initialize media.
 *
 * Precondition:
 *   Semaphore has been taken.
 ****************************************************************************/

static FAR struct mtd_dev_s *lc823450_mtd_allocdev(uint32_t channel)
{
  int ret;
  int mtype = lc823450_sdc_refmediatype(channel);
  FAR struct lc823450_mtd_dev_s *priv;

  /* Create an instance of the LC823450 MTD device state structure */

  priv = (FAR struct lc823450_mtd_dev_s *)
    kmm_zalloc(sizeof(struct lc823450_mtd_dev_s));
  if (!priv)
    {
      finfo("Failed to allocate the LC823450 MTD devicestructure\n");
      return NULL;
    }

  nxsem_init(&priv->sem, 0, 1);

  priv->mtd.erase  = lc823450_erase;
  priv->mtd.bread  = lc823450_bread;
  priv->mtd.bwrite = lc823450_bwrite;
#ifdef CONFIG_MTD_BYTE_READ
  priv->mtd.read   = NULL;
#endif
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = NULL;
#endif
  priv->mtd.ioctl  = lc823450_ioctl;
  priv->mtd.name   = g_mtdname[mtype];

  priv->channel = channel;

  ret = mtd_mediainitialize(priv);
  if (ret != OK)
    {
      finfo("ERROR: Failed to initialize media\n");
      nxsem_destroy(&priv->sem);
      kmm_free(priv);
      return NULL;
    }

  /* Return the implementation-specific state structure as the MTD device */

  return &priv->mtd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_mtd_initialize
 *
 * Description:
 *   Create and initialize an MTD device instance.  MTD devices are not
 *   registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

int lc823450_mtd_initialize(uint32_t devno)
{
  int i;
  int partno;
  int ret;
  FAR struct lc823450_mtd_dev_s *priv;
  off_t maxblock;
  uint32_t ch = (devno == CONFIG_MTD_DEVNO_EMMC)? 0 : 1;

#if CONFIG_MTD_DEV_MAX == 1
  DEBUGASSERT(devno == CONFIG_MTD_DEVNO_EMMC);
#else
  DEBUGASSERT(devno == CONFIG_MTD_DEVNO_EMMC ||
              devno == CONFIG_MTD_DEVNO_SDC);
#endif

  /* Following block devices are created.
   *
   *  /dev/mtdblock0   : Master partition
   *  /dev/mtdblock0p1 : 1st child partition
   *  /dev/mtdblock0p2 : 2nd child partition
   *  ...
   *  /dev/mtdblock0pN : Nth child partition
   */

  ret = mtd_semtake(&g_sem);
  if (ret < 0)
    {
      return ret;
    }

  if (g_mtdmaster[ch])
    {
      finfo("Device already registered\n");
      mtd_semgive(&g_sem);
      return -EBUSY;
    }

  /* Create master partition */

  g_mtdmaster[ch] = lc823450_mtd_allocdev(ch);
  if (!g_mtdmaster[ch])
    {
      finfo("Failed to create master partition: ch=%" PRId32 "\n", ch);
      mtd_semgive(&g_sem);
      return -ENODEV;
    }

  ret = mmcl_initialize(devno, g_mtdmaster[ch]);
  if (ret != OK)
    {
      finfo("Failed to create block device on master partition: "
            "ch=%" PRId32 "\n",
            ch);
      kmm_free(g_mtdmaster[ch]);
      g_mtdmaster[ch] = NULL;
      mtd_semgive(&g_sem);
      return -ENODEV;
    }

#ifdef CONFIG_DEBUG
  finfo("/dev/mtdblock%d created\n", devno);
#endif

  priv = (FAR struct lc823450_mtd_dev_s *)g_mtdmaster[ch];

  /* If SDC, create no child partition */

#if CONFIG_MTD_DEV_MAX > 1
  if (devno == CONFIG_MTD_DEVNO_SDC)
    {
      finfo("SDC has no child partitions.\n");
      mtd_semgive(&g_sem);
      return OK;
    }
#endif

  maxblock = priv->nblocks;

  /* Check partition table */

#ifdef CONFIG_DEBUG
  for (i = 0; i < LC823450_NPARTS - 1; i++)
    {
      DEBUGASSERT(partinfo[i].startblock < partinfo[i + 1].startblock);
      DEBUGASSERT(partinfo[i].startblock + partinfo[i].nblocks <= maxblock);
      DEBUGASSERT(partinfo[i + 1].startblock + partinfo[i + 1].nblocks <=
                  maxblock);
    }
#endif

  /* Create child partitions */

  for (i = 0, partno = 1; partno <= LC823450_NPARTS; i++, partno++)
    {
      if (partno == LC823450_NPARTS)
        {
          if (partinfo[i].nblocks == 0)
            {
              partinfo[i].nblocks = (i == 0) ?
                maxblock - partinfo[i].startblock :
                maxblock - (partinfo[i - 1].startblock +
                partinfo[i - 1].nblocks);
            }
        }

      g_mtdpart[i] = mtd_partition(g_mtdmaster[ch], partinfo[i].startblock,
                                   partinfo[i].nblocks);
      if (!g_mtdpart[i])
        {
          finfo("%s(): mtd_partition failed. startblock=%"
                PRIuOFF " nblocks=%" PRIuOFF "\n", __func__,
                partinfo[i].startblock, partinfo[i].nblocks);
          mtd_semgive(&g_sem);
          DEBUGASSERT(0);
          return -EIO;
        }

      ret = mmcl_createpartition(devno, partno, g_mtdpart[i]);
      if (ret < 0)
        {
          finfo("%s(): mmcl_initialize part%d failed: %d\n",
                __func__, partno, ret);
          mtd_semgive(&g_sem);
          DEBUGASSERT(0);
          return ret;
        }
    }

  mtd_semgive(&g_sem);
  return OK;
}

#if CONFIG_MTD_DEV_MAX > 1

/****************************************************************************
 * Name: lc823450_mtd_reinitialize_card
 *
 * Description:
 *   Called in resume sequence, if a card exists
 *
 ****************************************************************************/

int lc823450_mtd_reinitialize_card(void)
{
  const uint32_t ch = 1;   /* SDC */
  uint32_t sysclk = lc823450_get_ahb();

  int ret = lc823450_sdc_clearcardinfo(ch);

  if (ret != OK)
    {
      finfo("ERROR: Failed to set clock: ret=%d\n", ret);
      goto exit_with_error;
    }

  ret = lc823450_sdc_setclock(ch, 20000000, sysclk);

  if (ret != OK)
    {
      finfo("ERROR: Failed to set clock: ret=%d\n", ret);
      goto exit_with_error;
    }

  ret = lc823450_sdc_identifycard(ch);

  if (ret != OK)
    {
      finfo("ERROR: Failed to identify card: ret=%d)\n", ret);
      goto exit_with_error;
    }

#ifdef CONFIG_LC823450_SDC_UHS1
  /* Try to change to DDR50  mode */

  ret = lc823450_sdc_changespeedmode(ch, 4);

  if (0 == ret)
    {
      goto exit_with_error;
    }
#endif

  /* Try to change to High Speed mode */

  ret = lc823450_sdc_changespeedmode(ch, 1);

  if (0 == ret)
    {
      ret = lc823450_sdc_setclock(ch, 40000000, sysclk);
    }

exit_with_error:
  return ret;
}

/****************************************************************************
 * Name: lc823450_mtd_uninitialize
 ****************************************************************************/

int lc823450_mtd_uninitialize(uint32_t devno)
{
  int ret;
  char devname[16];
  FAR struct lc823450_mtd_dev_s *priv;
  const uint32_t ch = 1;   /* SDC */
  finfo("devno=%" PRId32 "\n", devno);

  DEBUGASSERT(devno == CONFIG_MTD_DEVNO_SDC);

  ret = mtd_semtake(&g_sem);
  if (ret < 0)
    {
      return ret;
    }

  priv = (FAR struct lc823450_mtd_dev_s *)g_mtdmaster[ch];
  if (!priv)
    {
      finfo("SD card is not identified yet\n");
      mtd_semgive(&g_sem);
      return -ENODEV;
    }

  snprintf(devname, 16, "/dev/mtdblock%" PRId32, devno);

#ifdef CONFIG_MTD_REGISTRATION
  mtd_unregister(g_mtdmaster[ch]);
#endif

  ret = mtd_semtake(&priv->sem);
  if (ret < 0)
    {
      mtd_semgive(&g_sem);
      return ret;
    }

  ret = lc823450_sdc_clearcardinfo(ch);
  DEBUGASSERT(ret == OK);

  mtd_semgive(&priv->sem);

  ret = mmcl_uninitialize(devname);
  if (ret != OK)
    {
      finfo("mmcl_uninitialize failed: %d", ret);
    }

  ret = lc823450_sdc_finalize(ch);
  DEBUGASSERT(ret == OK);

  nxsem_destroy(&priv->sem);

  kmm_free(g_mtdmaster[ch]);

  g_mtdmaster[ch] = NULL;

  mtd_semgive(&g_sem);

#ifdef CONFIG_DEBUG
  finfo("/dev/mtdblock%d deleted\n", devno);
#endif
  return OK;
}
#endif
