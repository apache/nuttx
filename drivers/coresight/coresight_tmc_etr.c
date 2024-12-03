/****************************************************************************
 * drivers/coresight/coresight_tmc_etr.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>
#include <debug.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/cache.h>
#include <nuttx/irq.h>

#include <nuttx/coresight/coresight_tmc.h>

#include "coresight_common.h"
#include "coresight_tmc_core.h"

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int tmc_etr_enable(FAR struct coresight_dev_s *csdev);
static void tmc_etr_disable(FAR struct coresight_dev_s *csdev);

static int tmc_etr_open(FAR struct file *filep);
static int tmc_etr_close(FAR struct file *filep);
static ssize_t tmc_etr_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct coresight_sink_ops_s g_tmc_etr_sink_ops =
{
  .enable  = tmc_etr_enable,
  .disable = tmc_etr_disable,
};

static const struct coresight_ops_s g_tmc_sink_ops =
{
  .sink_ops = &g_tmc_etr_sink_ops,
};

static const struct file_operations g_tmc_fops =
{
  tmc_etr_open,  /* open */
  tmc_etr_close, /* close */
  tmc_etr_read,  /* read */
  NULL,          /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmc_etr_hw_enable
 ****************************************************************************/

static int tmc_etr_hw_enable(FAR struct coresight_tmc_dev_s *tmcdev)
{
  uint32_t axictl;

  coresight_unlock(tmcdev->csdev.addr);

  /* Wait for TMCSReady bit to be set. */

  if (coresight_timeout(TMC_STS_TMCREADY, TMC_STS_TMCREADY,
                        tmcdev->csdev.addr + TMC_STS) < 0)
    {
      cserr("tmc device is not ready\n");
      coresight_lock(tmcdev->csdev.addr);
      return -EAGAIN;
    }

  coresight_put32(tmcdev->size / 4, tmcdev->csdev.addr + TMC_RSZ);
  coresight_put32(TMC_MODE_CIRCULAR_BUFFER, tmcdev->csdev.addr + TMC_MODE);

  /* Set AXICTL. */

  axictl = coresight_get32(tmcdev->csdev.addr + TMC_AXICTL);
  axictl &= ~TMC_AXICTL_CLEAR_MASK;
  axictl |= TMC_AXICTL_PROT_CTL_B1;
  axictl |= TMC_AXICTL_WR_BURST(tmcdev->burst_size);
  axictl |= TMC_AXICTL_AXCACHE_OS;
  if (tmcdev->caps & TMC_ETR_AXI_ARCACHE)
    {
      axictl &= ~TMC_AXICTL_ARCACHE_MASK;
      axictl |= TMC_AXICTL_ARCACHE_OS;
    }

  if (tmcdev->mode == TMC_ETR_MODE_ETR_SG)
    {
      axictl |= TMC_AXICTL_SCT_GAT_MODE;
    }

  coresight_put32(axictl, tmcdev->csdev.addr + TMC_AXICTL);
  coresight_put32((uintptr_t)tmcdev->buf, tmcdev->csdev.addr + TMC_DBALO);
  coresight_put32(((uint64_t)(uintptr_t)tmcdev->buf) >> 32,
                  tmcdev->csdev.addr + TMC_DBAHI);

  if (tmcdev->caps & TMC_ETR_SAVE_RESTORE)
    {
      coresight_put32((uintptr_t)tmcdev->buf, tmcdev->csdev.addr + TMC_RRP);
      coresight_put32(((uint64_t)(uintptr_t)tmcdev->buf >> 32),
                      tmcdev->csdev.addr + TMC_RRPHI);

      coresight_put32((uintptr_t)tmcdev->buf, tmcdev->csdev.addr + TMC_RWP);
      coresight_put32(((uint64_t)(uintptr_t)tmcdev->buf >> 32),
                      tmcdev->csdev.addr + TMC_RWPHI);

      coresight_modify32(0x0, TMC_STS_FULL, tmcdev->csdev.addr + TMC_STS);
    }

  coresight_put32(TMC_FFCR_EN_FMT | TMC_FFCR_EN_TI | TMC_FFCR_FON_FLIN |
                  TMC_FFCR_FON_TRIG_EVT | TMC_FFCR_TRIGON_TRIGIN,
                  tmcdev->csdev.addr + TMC_FFCR);
  coresight_put32(tmcdev->trigger_cntr, tmcdev->csdev.addr + TMC_TRG);

  /* Enable capture. */

  coresight_put32(TMC_CTL_CAPT_EN, tmcdev->csdev.addr + TMC_CTL);

  coresight_lock(tmcdev->csdev.addr);
  return 0;
}

/****************************************************************************
 * Name: tmc_flush_and_stop
 ****************************************************************************/

static void tmc_flush_and_stop(FAR struct coresight_tmc_dev_s *tmcdev)
{
  coresight_modify32(TMC_FFCR_STOP_ON_FLUSH, TMC_FFCR_STOP_ON_FLUSH,
                     tmcdev->csdev.addr + TMC_FFCR);
  coresight_modify32(TMC_FFCR_FON_MAN, TMC_FFCR_FON_MAN,
                     tmcdev->csdev.addr + TMC_FFCR);
  if (coresight_timeout(0x0, TMC_FFCR_FON_MAN,
                        tmcdev->csdev.addr + TMC_FFCR) < 0)
    {
      cserr("timeout while waiting for completion of Manual Flush\n");
    }

  if (coresight_timeout(TMC_STS_TMCREADY, TMC_STS_TMCREADY,
                        tmcdev->csdev.addr + TMC_STS) < 0)
    {
      cserr("timeout while waiting for TMC to be Ready\n");
    }
}

/****************************************************************************
 * Name: tmc_etr_hw_read
 ****************************************************************************/

static void tmc_etr_hw_read(FAR struct coresight_tmc_dev_s *tmcdev)
{
  uintptr_t rrp;
  uintptr_t rwp;
  uint32_t status;
  bool lost = false;

  rrp = (uint64_t)coresight_get32(tmcdev->csdev.addr + TMC_RRPHI) << 32 |
        coresight_get32(tmcdev->csdev.addr + TMC_RRP);
  rwp = (uint64_t)coresight_get32(tmcdev->csdev.addr + TMC_RWPHI) << 32 |
        coresight_get32(tmcdev->csdev.addr + TMC_RWP);
  status = coresight_get32(tmcdev->csdev.addr + TMC_STS);

  /* If there were memory errors in the session, truncate the buffer. */

  if (status & TMC_STS_MEMERR)
    {
      cserr("tmc memory error detected, truncating buffer\n");
      tmcdev->len = 0;
      return;
    }

  if ((status & TMC_STS_FULL) == 1)
    {
      lost = true;
    }

  tmcdev->offset = rrp - (uintptr_t)tmcdev->buf;
  if (lost == true)
    {
      tmcdev->len = tmcdev->size;
    }
  else
    {
      tmcdev->len = (uint32_t)(rwp - rrp);
    }

  if (tmcdev->offset + tmcdev->len > tmcdev->size)
    {
      up_invalidate_dcache((uintptr_t)tmcdev->buf, tmcdev->size);
    }
  else
    {
      up_invalidate_dcache((uintptr_t)((FAR char *)tmcdev->buf +
                                       tmcdev->offset), tmcdev->len);
    }

  if (lost == true)
    {
      coresight_insert_barrier_packet((FAR char *)tmcdev->buf +
                                      tmcdev->offset);
    }
}

/****************************************************************************
 * Name: tmc_etr_hw_disable_and_read
 ****************************************************************************/

static void
tmc_etr_hw_disable_and_read(FAR struct coresight_tmc_dev_s *tmcdev)
{
  coresight_unlock(tmcdev->csdev.addr);
  tmc_flush_and_stop(tmcdev);
  tmc_etr_hw_read(tmcdev);

  /* Disable capture enable bit. */

  coresight_put32(0x0, tmcdev->csdev.addr + TMC_CTL);
  coresight_lock(tmcdev->csdev.addr);
}

/****************************************************************************
 * Name: tmc_etr_hw_disable
 ****************************************************************************/

static void tmc_etr_hw_disable(FAR struct coresight_tmc_dev_s *tmcdev)
{
  coresight_unlock(tmcdev->csdev.addr);
  tmc_flush_and_stop(tmcdev);

  /* Disable capture enable bit. */

  coresight_put32(0x0, tmcdev->csdev.addr + TMC_CTL);
  coresight_lock(tmcdev->csdev.addr);
}

/****************************************************************************
 * Name: tmc_etr_enable
 ****************************************************************************/

static int tmc_etr_enable(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_tmc_dev_s *tmcdev =
    (FAR struct coresight_tmc_dev_s *)csdev;
  int ret;

  ret = coresight_claim_device(tmcdev->csdev.addr);
  if (ret < 0)
    {
      cserr("%s claimed failed\n", csdev->name);
      return ret;
    }

  ret = tmc_etr_hw_enable(tmcdev);
  if (ret < 0)
    {
      coresight_disclaim_device(tmcdev->csdev.addr);
    }

  return ret;
}

/****************************************************************************
 * Name: tmc_etr_disable
 ****************************************************************************/

static void tmc_etr_disable(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_tmc_dev_s *tmcdev =
    (FAR struct coresight_tmc_dev_s *)csdev;

  tmc_etr_hw_disable(tmcdev);
  coresight_disclaim_device(tmcdev->csdev.addr);
}

/****************************************************************************
 * Name: tmc_etr_open
 *
 * Description:
 *   TMC etr devices write data directly to system memory, it can not enable
 *   after disable like tmc_etf_open does, which may cause data confusion.
 *
 ****************************************************************************/

static int tmc_etr_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct coresight_tmc_dev_s *tmcdev;
  int ret;

  DEBUGASSERT(inode->i_private);
  tmcdev = (FAR struct coresight_tmc_dev_s *)inode->i_private;

  ret = nxmutex_lock(&tmcdev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (tmcdev->opencnt++ == 0)
    {
      irqstate_t flags;

      flags = enter_critical_section();
      if (tmcdev->csdev.refcnt > 0)
        {
          tmc_etr_hw_disable_and_read(tmcdev);
        }
      else
        {
          /* ETR devices can not read buffer directly when it is not
           * enabled like etb device does. Its buffer needs to be
           * captured in stopped state which is transferred after it
           * hase been enabled.
           */

          tmcdev->opencnt--;
          ret = -EACCES;
        }

      leave_critical_section(flags);
    }

  nxmutex_unlock(&tmcdev->lock);
  return ret;
}

/****************************************************************************
 * Name: tmc_etr_close
 ****************************************************************************/

static int tmc_etr_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct coresight_tmc_dev_s *tmcdev;
  int ret;

  DEBUGASSERT(inode->i_private);
  tmcdev = (FAR struct coresight_tmc_dev_s *)inode->i_private;

  ret = nxmutex_lock(&tmcdev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (--tmcdev->opencnt == 0)
    {
      irqstate_t flags;

      flags = enter_critical_section();
      if (tmcdev->csdev.refcnt > 0)
        {
          if (tmc_etr_hw_enable(tmcdev) < 0)
            {
              cserr("%s enabled failed after read\n", tmcdev->csdev.name);
            }
        }

      leave_critical_section(flags);
    }

  nxmutex_unlock(&tmcdev->lock);
  return ret;
}

/****************************************************************************
 * Name: tmc_etr_read
 ****************************************************************************/

static ssize_t tmc_etr_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct coresight_tmc_dev_s *tmcdev;
  off_t off;

  DEBUGASSERT(inode->i_private);
  tmcdev = (FAR struct coresight_tmc_dev_s *)inode->i_private;

  if (filep->f_pos > tmcdev->len)
    {
      return 0;
    }

  if (filep->f_pos + buflen > tmcdev->len)
    {
      buflen = tmcdev->len - filep->f_pos;
    }

  /* Compute the offset from which we read the data. */

  off = tmcdev->offset + filep->f_pos;
  if (off >= tmcdev->size)
    {
      off -= tmcdev->size;
    }

  /* Adjust the length to limit this transaction to end of buffer. */

  if (buflen > tmcdev->size - off)
    {
      buflen = tmcdev->size - off;
    }

  memcpy(buffer, (FAR char *)tmcdev->buf + off, buflen);
  filep->f_pos += buflen;

  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmc_etr_register
 ****************************************************************************/

int tmc_etr_register(FAR struct coresight_tmc_dev_s *tmcdev,
                     FAR const struct coresight_desc_s *desc)
{
  char pathname[TMC_MAX_NAME_LEN];
  int ret;

  /* Check for AXI access. */

  if ((coresight_get32(desc->addr + TMC_AUTHSTATUS) &
       TMC_AUTH_NSID_MASK) != TMC_NSID_EN)
    {
      return -EACCES;
    }

  if (desc->subtype.sink_subtype != CORESIGHT_DEV_SUBTYPE_SINK_TMC_SYSMEM)
    {
      cserr("unsupported tmc device type\n");
      return -EPERM;
    }

  tmcdev->mode = TMC_ETR_MODE_FLAT;
  tmcdev->buf = kmm_zalloc(tmcdev->size);
  if (tmcdev->buf == NULL)
    {
      return -ENOMEM;
    }

  tmcdev->csdev.ops = &g_tmc_sink_ops;
  ret = coresight_register(&tmcdev->csdev, desc);
  if (ret < 0)
    {
      cserr("%s:coresight register failed\n", desc->name);
      goto cs_err;
    }

  snprintf(pathname, sizeof(pathname), "/dev/%s", desc->name);
  ret = register_driver(pathname, &g_tmc_fops, 0444, tmcdev);
  if (ret < 0)
    {
      cserr("%s:driver register failed\n", desc->name);
      goto drv_err;
    }

  return ret;

drv_err:
  coresight_unregister(&tmcdev->csdev);
cs_err:
  kmm_free(tmcdev->buf);
  return ret;
}

/****************************************************************************
 * Name: tmc_etr_unregister
 ****************************************************************************/

void tmc_etr_unregister(FAR struct coresight_tmc_dev_s * tmcdev)
{
  char pathname[TMC_MAX_NAME_LEN];

  snprintf(pathname, sizeof(pathname), "/dev/%s", tmcdev->csdev.name);
  unregister_driver(pathname);
  coresight_unregister(&tmcdev->csdev);
  kmm_free(tmcdev->buf);
}
