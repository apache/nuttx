/****************************************************************************
 * drivers/coresight/coresight_tmc_etf.c
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
#include <stdio.h>
#include <string.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>

#include <nuttx/coresight/coresight_tmc.h>

#include "coresight_common.h"
#include "coresight_tmc_core.h"

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int tmc_etf_sink_enable(FAR struct coresight_dev_s *csdev);
static void tmc_etf_sink_disable(FAR struct coresight_dev_s *csdev);
static int tmc_etf_link_enable(FAR struct coresight_dev_s *csdev,
                               int iport, int oport);
static void tmc_etf_link_disable(FAR struct coresight_dev_s *csdev,
                                 int iport, int oport);

static int tmc_etf_open(FAR struct file *filep);
static int tmc_etf_close(FAR struct file *filep);
static ssize_t tmc_etf_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* TMC-ETB and TMC-ETF sink device operations. */

static const struct coresight_sink_ops_s g_tmc_etf_sink_ops =
{
  .enable  = tmc_etf_sink_enable,
  .disable = tmc_etf_sink_disable,
};

static const struct coresight_ops_s g_tmc_sink_ops =
{
  .sink_ops = &g_tmc_etf_sink_ops,
};

/* TMC-ETF link device operations. */

static const struct coresight_link_ops_s g_tmc_etf_link_ops =
{
  .enable  = tmc_etf_link_enable,
  .disable = tmc_etf_link_disable,
};

static const struct coresight_ops_s g_tmc_link_ops =
{
  .link_ops = &g_tmc_etf_link_ops,
};

static const struct file_operations g_tmc_fops =
{
  tmc_etf_open,  /* open */
  tmc_etf_close, /* close */
  tmc_etf_read,  /* read */
  NULL,          /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmc_etf_sink_hw_enable
 ****************************************************************************/

static int tmc_etf_sink_hw_enable(FAR struct coresight_tmc_dev_s *tmcdev)
{
  coresight_unlock(tmcdev->csdev.addr);

  /* Wait for TMCSReady bit to be set. */

  if (coresight_timeout(TMC_STS_TMCREADY, TMC_STS_TMCREADY,
                        tmcdev->csdev.addr + TMC_STS) < 0)
    {
      cserr("tmc device is not ready\n");
      coresight_lock(tmcdev->csdev.addr);
      return -EAGAIN;
    }

  /* TMC-ETB and TMC-ETF sink device use cirular buffer mode. */

  coresight_put32(TMC_MODE_CIRCULAR_BUFFER, tmcdev->csdev.addr + TMC_MODE);
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
 * Name: tmc_etf_link_hw_enable
 ****************************************************************************/

static int tmc_etf_link_hw_enable(FAR struct coresight_tmc_dev_s *tmcdev)
{
  coresight_unlock(tmcdev->csdev.addr);

  /* Wait for TMCSReady bit to be set. */

  if (coresight_timeout(TMC_STS_TMCREADY, TMC_STS_TMCREADY,
                        tmcdev->csdev.addr + TMC_STS) < 0)
    {
      cserr("tmc device is not ready\n");
      coresight_lock(tmcdev->csdev.addr);
      return -EAGAIN;
    }

  /* TMC-ETF link device use Hardware FIFO buffer mode. */

  coresight_put32(TMC_MODE_HARDWARE_FIFO, tmcdev->csdev.addr + TMC_MODE);
  coresight_put32(TMC_FFCR_EN_FMT | TMC_FFCR_EN_TI,
                  tmcdev->csdev.addr + TMC_FFCR);
  coresight_put32(0x0, tmcdev->csdev.addr + TMC_BUFWM);

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
 * Name: tmc_etf_hw_read
 *
 * Description:
 *   Dump ETB RAM buffer to device's buffer for usrspace's read. It just need
 *   to performing successive reads to the RRD Register, until the value
 *   0xFFFFFFFF is returned whick is kind different from coresight ETB
 *   device's reading process. refers to TRM.
 *
 ****************************************************************************/

static void tmc_etf_hw_read(FAR struct coresight_tmc_dev_s *tmcdev)
{
  FAR uint32_t *bufptr;
  uint32_t read_data;

  bufptr = tmcdev->buf;
  tmcdev->len = 0;
  for (; ; )
    {
      read_data = coresight_get32(tmcdev->csdev.addr + TMC_RRD);
      if (read_data == 0xffffffff)
        {
          break;
        }

      memcpy(bufptr, &read_data, 4);
      bufptr += 1;
      tmcdev->len += 4;
    }

  if ((coresight_get32(tmcdev->csdev.addr + TMC_STS) & TMC_STS_FULL) == 1)
    {
      coresight_insert_barrier_packet(tmcdev->buf);
    }
}

/****************************************************************************
 * Name: tmc_etf_hw_disable_and_read
 *
 * Description:
 *   Used for ETF sink devices to dump trace buffer. Do not dump trace buffer
 *   in tmc_etf_hw_disable to avoid trace buffer's data confusion when a
 *   process is reading trace buffer and anther process calles tmc_disable.
 *
 ****************************************************************************/

static void
tmc_etf_hw_disable_and_read(FAR struct coresight_tmc_dev_s *tmcdev)
{
  coresight_unlock(tmcdev->csdev.addr);
  tmc_flush_and_stop(tmcdev);

  /* This is kind different from ETB coresight device, it should read data
   * from ram buffer in stopped state rather then disabled state.
   */

  tmc_etf_hw_read(tmcdev);

  /* Disable capture enable bit. */

  coresight_put32(0x0, tmcdev->csdev.addr + TMC_CTL);
  coresight_lock(tmcdev->csdev.addr);
}

/****************************************************************************
 * Name: tmc_etf_hw_disable
 *
 * Description:
 *   ETF link devices and ETF sink devices share same disable flow.
 *
 ****************************************************************************/

static void tmc_etf_hw_disable(FAR struct coresight_tmc_dev_s *tmcdev)
{
  coresight_unlock(tmcdev->csdev.addr);
  tmc_flush_and_stop(tmcdev);

  /* Disable capture enable bit. */

  coresight_put32(0x0, tmcdev->csdev.addr + TMC_CTL);
  coresight_lock(tmcdev->csdev.addr);
}

/****************************************************************************
 * Name: tmc_etf_sink_enable
 ****************************************************************************/

static int tmc_etf_sink_enable(FAR struct coresight_dev_s *csdev)
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

  ret = tmc_etf_sink_hw_enable(tmcdev);
  if (ret < 0)
    {
      coresight_disclaim_device(tmcdev->csdev.addr);
    }

  return ret;
}

/****************************************************************************
 * Name: tmc_etf_sink_disable
 ****************************************************************************/

static void tmc_etf_sink_disable(FAR struct coresight_dev_s *csdev)
{
  FAR struct coresight_tmc_dev_s *tmcdev =
    (FAR struct coresight_tmc_dev_s *)csdev;

  tmc_etf_hw_disable(tmcdev);
  coresight_disclaim_device(tmcdev->csdev.addr);
}

/****************************************************************************
 * Name: tmc_etf_link_enable
 ****************************************************************************/

static int tmc_etf_link_enable(FAR struct coresight_dev_s *csdev,
                               int iport, int oport)
{
  FAR struct coresight_tmc_dev_s *tmcdev =
    (FAR struct coresight_tmc_dev_s *)csdev;
  int ret;

  if (csdev->refcnt != 1)
    {
      return 0;
    }

  ret = coresight_claim_device(tmcdev->csdev.addr);
  if (ret < 0)
    {
      cserr("%s claimed failed\n", csdev->name);
      return ret;
    }

  ret = tmc_etf_link_hw_enable(tmcdev);
  if (ret < 0)
    {
      coresight_disclaim_device(tmcdev->csdev.addr);
    }

  return ret;
}

/****************************************************************************
 * Name: tmc_etf_link_disable
 ****************************************************************************/

static void tmc_etf_link_disable(FAR struct coresight_dev_s *csdev,
                                 int iport, int oport)
{
  FAR struct coresight_tmc_dev_s *tmcdev =
    (FAR struct coresight_tmc_dev_s *)csdev;

  if (csdev->refcnt != 1)
    {
      return;
    }

  tmc_etf_hw_disable(tmcdev);
  coresight_disclaim_device(tmcdev->csdev.addr);
}

/****************************************************************************
 * Name: tmc_etf_open
 ****************************************************************************/

static int tmc_etf_open(FAR struct file *filep)
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
      tmcdev->buf = kmm_zalloc(tmcdev->size);
      if (tmcdev->buf == NULL)
        {
          cserr("malloc buffer failed\n");
          tmcdev->opencnt--;
          ret = -ENOMEM;
        }
      else
        {
          irqstate_t flags;

          flags = enter_critical_section();
          if (tmcdev->csdev.refcnt > 0)
            {
              tmc_etf_hw_disable_and_read(tmcdev);
              tmc_etf_sink_hw_enable(tmcdev);
            }
          else
            {
              /* ETF devices can not read buffer directly when it is not
               * enabled like etb device does. Its buffer needs to be
               * captured in stopped state which is transferred after it
               * hase been enabled.
               */

              kmm_free(tmcdev->buf);
              tmcdev->opencnt--;
              ret = -EACCES;
            }

          leave_critical_section(flags);
        }
    }

  nxmutex_unlock(&tmcdev->lock);
  return ret;
}

/****************************************************************************
 * Name: tmc_etf_close
 ****************************************************************************/

static int tmc_etf_close(FAR struct file *filep)
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
      kmm_free(tmcdev->buf);
    }

  nxmutex_unlock(&tmcdev->lock);
  return ret;
}

/****************************************************************************
 * Name: tmc_etf_read
 ****************************************************************************/

static ssize_t tmc_etf_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct coresight_tmc_dev_s *tmcdev;

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

  memcpy(buffer, (FAR char *)tmcdev->buf + filep->f_pos, buflen);
  filep->f_pos += buflen;

  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmc_etf_register
 ****************************************************************************/

int tmc_etf_register(FAR struct coresight_tmc_dev_s * tmcdev,
                     FAR const struct coresight_desc_s *desc)
{
  int ret;

  if (desc->type == CORESIGHT_DEV_TYPE_SINK)
    {
      enum coresight_dev_subtype_sink_e subtype = desc->subtype.sink_subtype;
      char pathname[TMC_MAX_NAME_LEN];

      if ((subtype != CORESIGHT_DEV_SUBTYPE_SINK_TMC_BUFFER) &&
          (subtype != CORESIGHT_DEV_SUBTYPE_SINK_TMC_ETF))
        {
          cserr("unsupported tmc device type\n");
          return -EPERM;
        }

      tmcdev->csdev.ops = &g_tmc_sink_ops;
      ret = coresight_register(&tmcdev->csdev, desc);
      if (ret < 0)
        {
          cserr("%s:coresight register failed\n", desc->name);
          return ret;
        }

      snprintf(pathname, sizeof(pathname), "/dev/%s", desc->name);
      ret = register_driver(pathname, &g_tmc_fops, 0444, tmcdev);
      if (ret < 0)
        {
          cserr("%s:driver register failed\n", desc->name);
          coresight_unregister(&tmcdev->csdev);
          return ret;
        }
    }
  else if (desc->type == CORESIGHT_DEV_TYPE_LINK)
    {
      if (desc->subtype.link_subtype != CORESIGHT_DEV_SUBTYPE_LINK_FIFO)
        {
          cserr("unsupported tmc link device type\n");
          return -EPERM;
        }

      tmcdev->csdev.ops = &g_tmc_link_ops;
      ret = coresight_register(&tmcdev->csdev, desc);
      if (ret < 0)
        {
          cserr("%s:coresight register failed\n", desc->name);
          return ret;
        }
    }
  else
    {
      cserr("unsupported tmc device type\n");
      return -EPERM;
    }

  return 0;
}

/****************************************************************************
 * Name: tmc_etf_unregister
 ****************************************************************************/

void tmc_etf_unregister(FAR struct coresight_tmc_dev_s * tmcdev)
{
  if (tmcdev->csdev.type == CORESIGHT_DEV_TYPE_SINK)
    {
      char pathname[TMC_MAX_NAME_LEN];

      snprintf(pathname, sizeof(pathname), "/dev/%s", tmcdev->csdev.name);
      unregister_driver(pathname);
    }
  else if (tmcdev->csdev.refcnt > 0)
    {
      irqstate_t flags;

      /* Link device should disable hw here, and sink device will disable
       * hw in coresight_core.c
       */

      flags = enter_critical_section();
      tmc_etf_hw_disable(tmcdev);
      coresight_disclaim_device(tmcdev->csdev.addr);
      leave_critical_section(flags);
    }

  coresight_unregister(&tmcdev->csdev);
}
