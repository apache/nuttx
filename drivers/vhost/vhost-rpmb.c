/****************************************************************************
 * drivers/vhost/vhost-rpmb.c
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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mmcsd.h>
#include <nuttx/semaphore.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio-rpmb.h>

#include "vhost-rpmb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MMC_* command opcodes */

#define VHOST_RPMB_MMC_SEND_EXT_CSD         8
#define VHOST_RPMB_MMC_READ_MULTIPLE_BLOCK  18
#define VHOST_RPMB_MMC_WRITE_MULTIPLE_BLOCK 25

/* MMC command flags */

#define VHOST_RPMB_MMC_RSP_PRESENT          (1 << 0)
#define VHOST_RPMB_MMC_RSP_CRC              (1 << 2)
#define VHOST_RPMB_MMC_RSP_OPCODE           (1 << 4)
#define VHOST_RPMB_MMC_CMD_ADTC             (1 << 5)
#define VHOST_RPMB_MMC_RSP_R1               (VHOST_RPMB_MMC_RSP_PRESENT | \
                                             VHOST_RPMB_MMC_RSP_CRC | \
                                             VHOST_RPMB_MMC_RSP_OPCODE)

/* CMD23 reliable write flag */

#define VHOST_RPMB_MMC_CMD23_ARG_REL_WR     (1 << 31)

/* Maximum number of commands in a multi-command request */

#define VHOST_RPMB_MMC_IOC_MAX_CMDS         3

/* RPMB frame size */

#define VHOST_RPMB_FRAME_SIZE               512

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_rpmb_priv_s
{
  FAR struct vhost_device *hdev;   /* Virtio device */
  sem_t                    semrx;  /* Wake up the worker thread */
  pid_t                    pid;    /* Worker thread PID */
  struct file              bdev;   /* eMMC RPMB partition */
  char                     path[VIRTIO_RPMB_PATH_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Vhost driver callbacks */

static int  vhost_rpmb_probe(FAR struct vhost_device *hdev);
static void vhost_rpmb_remove(FAR struct vhost_device *hdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vhost_driver g_vhost_rpmb_driver =
{
  LIST_INITIAL_VALUE(g_vhost_rpmb_driver.node), /* node */
  VIRTIO_ID_RPMB,                               /* device id */
  vhost_rpmb_probe,                             /* probe */
  vhost_rpmb_remove,                            /* remove */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_rpmb_set_mmc_io_cmd
 *
 * Description:
 *   Set the per-request fields of one command.
 *
 ****************************************************************************/

static void vhost_rpmb_set_mmc_ioc_cmd(FAR struct mmc_ioc_cmd *cmd,
                                       uint32_t opcode, int write_flag,
                                       FAR void *data, unsigned int len)
{
  cmd->blksz      = VHOST_RPMB_FRAME_SIZE;
  cmd->blocks     = len / VHOST_RPMB_FRAME_SIZE;
  cmd->flags      = VHOST_RPMB_MMC_RSP_R1 | VHOST_RPMB_MMC_CMD_ADTC;
  cmd->opcode     = opcode;
  cmd->write_flag = write_flag;
  mmc_ioc_cmd_set_data(*cmd, data);
}

/****************************************************************************
 * Name: vhost_rpmb_update_config
 *
 * Description:
 *   Read EXT_CSD from the eMMC and publish capacity/max_wr_cnt in
 *   the virtio config space, so the TEE can query them without relaying a
 *   raw eMMC command through the virtqueue.
 *
 ****************************************************************************/

static int vhost_rpmb_update_config(FAR struct vhost_device *hdev,
                                    FAR struct file *bdev)
{
  struct virtio_rpmb_config_s cfg;
  struct mmc_ioc_cmd cmd;
  uint8_t ext_csd[VHOST_RPMB_FRAME_SIZE];
  int ret;

  memset(&cmd, 0, sizeof(cmd));
  memset(ext_csd, 0, sizeof(ext_csd));
  vhost_rpmb_set_mmc_ioc_cmd(&cmd, VHOST_RPMB_MMC_SEND_EXT_CSD, 0,
                             ext_csd, sizeof(ext_csd));

  ret = file_ioctl(bdev, MMC_IOC_CMD, (unsigned long)&cmd);
  if (ret < 0)
    {
      vhosterr("MMC_SEND_EXT_CSD failed, ret=%d\n", ret);
      return ret;
    }

  cfg.capacity   = ext_csd[VIRTIO_RPMB_EXT_CSD_RPMB_SIZE];
  cfg.max_wr_cnt = ext_csd[VIRTIO_RPMB_EXT_CSD_REL_WR_SEC_C];
  cfg.max_rd_cnt = 0;

  vhost_write_config(hdev, 0, &cfg, sizeof(cfg));
  return OK;
}

/****************************************************************************
 * Name: vhost_rpmb_handler
 *
 * Description:
 *   Process a single RPMB request from the virtqueue. Build an
 *   mmc_ioc_multi_cmd from the descriptor chain buffers and forward it to
 *   the eMMC RPMB device. The SDHCI driver writes response data directly
 *   into the in buffers (shared memory).
 *
 *   Returns the number of bytes written into the response (in) buffers,
 *   or a negative errno on failure.
 *
 ****************************************************************************/

static ssize_t vhost_rpmb_handler(FAR struct vhost_rpmb_priv_s *priv,
                                  FAR struct virtqueue_buf *vb,
                                  size_t vbcnt)
{
  uint8_t mcmd_buf[sizeof(struct mmc_ioc_multi_cmd) +
                   VHOST_RPMB_MMC_IOC_MAX_CMDS * sizeof(struct mmc_ioc_cmd)];
  FAR struct mmc_ioc_multi_cmd *mcmd;
  size_t out_cnt;
  ssize_t ret;
  size_t i;

  /* vb[] is in descriptor chain order: request frames first, response
   * frame last. A chain without both is malformed.
   */

  if (vbcnt < 2 || vbcnt > VHOST_RPMB_MMC_IOC_MAX_CMDS)
    {
      vhosterr("Malformed request, vbcnt=%zu\n", vbcnt);
      return -EINVAL;
    }

  out_cnt = vbcnt - 1;

  memset(mcmd_buf, 0, sizeof(mcmd_buf));
  mcmd = (FAR struct mmc_ioc_multi_cmd *)mcmd_buf;
  mcmd->num_of_cmds = vbcnt;

  /* Each out buffer -> CMD25 (write to eMMC RPMB partition) */

  for (i = 0; i < out_cnt; i++)
    {
      FAR struct mmc_ioc_cmd *cmd = &mcmd->cmds[i];
      FAR struct virtio_rpmb_frame *frm = vb[i].buf;
      uint16_t msg_type;
      int rel_wr = 0;

      /* Check if this is a reliable write (DATA_WRITE or PROGRAM_KEY) */

      if (vb[i].len >= sizeof(struct virtio_rpmb_frame))
        {
          msg_type = betoh16(frm->req_resp);
          if (msg_type == VIRTIO_RPMB_REQ_DATA_WRITE ||
              msg_type == VIRTIO_RPMB_REQ_PROGRAM_KEY)
            {
              rel_wr = VHOST_RPMB_MMC_CMD23_ARG_REL_WR;
            }
        }

      vhost_rpmb_set_mmc_ioc_cmd(cmd, VHOST_RPMB_MMC_WRITE_MULTIPLE_BLOCK,
                                 1 | rel_wr, vb[i].buf, vb[i].len);
    }

  /* Each in buffer -> CMD18 (read from eMMC RPMB partition) */

  for (; i < vbcnt; i++)
    {
      FAR struct mmc_ioc_cmd *cmd = &mcmd->cmds[i];

      vhost_rpmb_set_mmc_ioc_cmd(cmd, VHOST_RPMB_MMC_READ_MULTIPLE_BLOCK,
                                 0, vb[i].buf, vb[i].len);
    }

  ret = file_ioctl(&priv->bdev, MMC_IOC_MULTI_CMD, (unsigned long)mcmd);
  if (ret < 0)
    {
      vhosterr("MMC_IOC_MULTI_CMD failed, ret=%zd\n", ret);
      return ret;
    }

  /* Return the total length of the response (in) buffers written back */

  for (i = out_cnt; i < vbcnt; i++)
    {
      ret += vb[i].len;
    }

  return ret;
}

/****************************************************************************
 * Name: vhost_rpmb_work
 ****************************************************************************/

static void vhost_rpmb_work(FAR struct vhost_rpmb_priv_s *priv)
{
  FAR struct virtqueue *vq;
  struct virtqueue_buf vb[VHOST_RPMB_MMC_IOC_MAX_CMDS];
  size_t vbcnt;
  ssize_t ret;
  int head;

  vq = priv->hdev->vrings_info[0].vq;
  for (; ; )
    {
      head = vhost_get_vq_buffers(vq, vb, VHOST_RPMB_MMC_IOC_MAX_CMDS,
                                  &vbcnt);
      if (head < 0)
        {
          break;
        }

      ret = vhost_rpmb_handler(priv, vb, vbcnt);
      if (ret < 0)
        {
          vhosterr("handler failed, ret=%zd\n", ret);
          ret = VHOST_RPMB_FRAME_SIZE;
        }

      virtqueue_add_consumed_buffer(vq, head, ret);
    }

  virtqueue_kick(vq);
}

/****************************************************************************
 * Name: vhost_rpmb_thread
 ****************************************************************************/

static int vhost_rpmb_thread(int argc, FAR char *argv[])
{
  FAR struct vhost_rpmb_priv_s *priv =
    (FAR struct vhost_rpmb_priv_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  for (; ; )
    {
      nxsem_wait_uninterruptible(&priv->semrx);
      vhost_rpmb_work(priv);
    }

  return 0;
}

/****************************************************************************
 * Name: vhost_rpmb_callback
 ****************************************************************************/

static void vhost_rpmb_callback(FAR struct virtqueue *vq)
{
  FAR struct vhost_rpmb_priv_s *priv = vq->vq_dev->priv;
  int semcount;

  nxsem_get_value(&priv->semrx, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->semrx);
    }
}

/****************************************************************************
 * Name: vhost_rpmb_probe
 ****************************************************************************/

static int vhost_rpmb_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_rpmb_priv_s *priv;
  FAR const char *vqname[1];
  vq_callback callback[1];
  FAR char *argv[2];
  char arg0[32];
  int ret;

  /* Alloc the vhost rpmb driver private data */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vhosterr("No enough memory\n");
      return -ENOMEM;
    }

  /* Init the vhost rpmb driver */

  priv->hdev = hdev;
  hdev->priv = priv;
  nxsem_init(&priv->semrx, 0, 0);

  /* Read the device path from virtio config space */

  vhost_read_config_member(hdev, struct vhost_rpmb_config_s,
                           path, &priv->path);
  ret = file_open(&priv->bdev, priv->path, O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      vhosterr("ERROR: Failed to open %s, ret=%d\n", priv->path, ret);
      goto err_init;
    }

  /* Read EXT_CSD and update virtio config space with accurate values */

  ret = vhost_rpmb_update_config(hdev, &priv->bdev);
  if (ret < 0)
    {
      vhosterr("ERROR: Failed to update RPMB config, ret=%d\n", ret);
      goto err_bdev;
    }

  /* Create the virtqueue */

  vqname[0]   = "vhost_rpmb";
  callback[0] = vhost_rpmb_callback;
  ret = vhost_create_virtqueues(hdev, 0, 1, vqname, callback, NULL);
  if (ret < 0)
    {
      vhosterr("vhost_create_virtqueues failed, ret=%d\n", ret);
      goto err_vq;
    }

  /* Start the worker thread */

  snprintf(arg0, sizeof(arg0), "%p", priv);
  argv[0] = arg0;
  argv[1] = NULL;

  priv->pid = kthread_create("vhost-rpmb",
                             CONFIG_DRIVERS_VHOST_RPMB_PRIORITY,
                             CONFIG_DRIVERS_VHOST_RPMB_STACKSIZE,
                             vhost_rpmb_thread, argv);
  if (priv->pid < 0)
    {
      vhosterr("vhost rpmb thread create failed %d\n", priv->pid);
      ret = priv->pid;
      goto err_thread;
    }

  virtqueue_enable_cb(hdev->vrings_info[0].vq);
  return OK;

err_thread:
  vhost_delete_virtqueues(hdev);
err_vq:
err_bdev:
  file_close(&priv->bdev);
  nxsem_destroy(&priv->semrx);
err_init:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: vhost_rpmb_remove
 ****************************************************************************/

static void vhost_rpmb_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_rpmb_priv_s *priv = hdev->priv;

  virtqueue_disable_cb(hdev->vrings_info[0].vq);
  kthread_delete(priv->pid);
  vhost_delete_virtqueues(hdev);
  file_close(&priv->bdev);
  nxsem_destroy(&priv->semrx);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_rpmb_driver
 ****************************************************************************/

int vhost_register_rpmb_driver(void)
{
  return vhost_register_driver(&g_vhost_rpmb_driver);
}
