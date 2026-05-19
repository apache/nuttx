/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_lp_mailbox.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <nuttx/debug.h>

#include "lp_core_mailbox.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_lp_mailbox_priv_s
{
  const struct file_operations *ops; /* Standard file operations */
  lp_mailbox_t mailbox;              /* LP Mailbox handler struct */
  lp_mailbox_config_t config;        /* LP Mailbox configuration */
  xcpt_t handler;                    /* Async I/O callaback handler */
  bool async_op;                     /* Flag to check active transaction mode */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_lp_mailbox_read(struct file *filep,
                               char *buffer,
                               size_t buflen);
static int esp_lp_mailbox_write(struct file *filep,
                                const char *buffer,
                                size_t buflen);
static int esp_lp_mailbox_ioctl(struct file *filep,
                                int cmd,
                                unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_esp_lp_mailbox_fops =
{
  .open = NULL,                       /* open */
  .close = NULL,                      /* close */
  .read = esp_lp_mailbox_read,        /* read */
  .write = esp_lp_mailbox_write,      /* write */
  .ioctl = esp_lp_mailbox_ioctl       /* ioctl */
};

struct esp_lp_mailbox_priv_s esp_lp_mailbox_priv =
{
  .ops = &g_esp_lp_mailbox_fops,
  .mailbox = NULL,
  .config =
  {
    0
  },
  .handler = NULL,
  .async_op = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_lp_mailbox_rcv_callback
 *
 * Description:
 *   Handler for the LP Mailbox controller interrupt.
 *
 * Input Parameters:
 *   msg - Message content
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_lp_mailbox_rcv_callback(lp_message_t msg)
{
  if (esp_lp_mailbox_priv.handler != NULL)
    {
      esp_lp_mailbox_priv.handler(0, NULL, (void *)msg);
    }
}

/****************************************************************************
 * Name: esp_lp_mailbox_read
 *
 * Description:
 *   Read LP Mailbox data.
 *
 * Input Parameters:
 *   filep  - The pointer of file, represents each user using the driver
 *   buffer - Buffer to save data
 *   buflen - Length of the buffer
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_lp_mailbox_read(struct file *filep,
                               char *buffer,
                               size_t buflen)
{
  struct inode *inode = filep->f_inode;
  struct esp_lp_mailbox_priv_s *priv = inode->i_private;
  lp_message_t msg;
  int i = 0;
  int ret = OK;
  lp_message_t recv;

  DEBUGASSERT(priv);

  while (i < buflen)
    {
      ret = lp_core_mailbox_receive(priv->mailbox, &recv, UINT32_MAX);
      if (ret != OK)
        {
          ferr("Failed to receive %dth byte\n", i + 1);
          return i;
        }

      buffer[i] = recv;
      i++;
    }

  return buflen;
}

/****************************************************************************
 * Name: esp_lp_mailbox_write
 *
 * Description:
 *   Write data to LP Mailbox.
 *
 * Input Parameters:
 *   filep  - The pointer of file, represents each user using the driver
 *   buffer - Buffer to write data
 *   buflen - Length of the buffer
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_lp_mailbox_write(struct file *filep,
                                const char *buffer,
                                size_t buflen)
{
  struct inode *inode = filep->f_inode;
  struct esp_lp_mailbox_priv_s *priv = inode->i_private;
  lp_message_t msg;
  int i = 0;
  int ret = OK;
  bool async_send = false;

  DEBUGASSERT(priv);

  if (priv->async_op == true && priv->handler == NULL)
    {
      async_send = true;
    }

  while (i < buflen)
    {
      if (async_send == true)
        {
          ret = lp_core_mailbox_send_async(priv->mailbox, buffer[i]);
        }
      else
        {
          ret = lp_core_mailbox_send(priv->mailbox, buffer[i], UINT32_MAX);
        }

      if (ret != OK)
        {
          ferr("Failed to send %dth byte\n", i + 1);
          return i;
        }

      i++;
    }

  return buflen;
}

/****************************************************************************
 * Name: esp_lp_mailbox_ioctl
 *
 * Description:
 *   Write data to LP Mailbox.
 *
 * Input Parameters:
 *   filep  - The pointer of file, represents each user using the driver
 *   buffer - Buffer to write data
 *   buflen - Length of the buffer
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_lp_mailbox_ioctl(struct file *filep,
                                int cmd,
                                unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct esp_lp_mailbox_priv_s *priv = inode->i_private;
  int ret = OK;
  xcpt_t handler;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
      case FIOC_NOTIFY:
        {
          /* Set event callback */

          handler = (xcpt_t)arg;
          if (handler == NULL)
            {
              if (priv->async_op == true)
                {
                  ret = lp_core_mailbox_receive_async_cancel(priv->mailbox,
                                                             NULL);
                  priv->async_op = false;
                }
              else
                {
                  priv->async_op = true;
                }
            }
          else
            {
              ret = lp_core_mailbox_receive_async(priv->mailbox, 1,
                                                esp_lp_mailbox_rcv_callback);
              priv->async_op = true;
            }

            if (ret != OK)
              {
                priv->async_op = false;
                ferr("Could not register callback-%lx to lp-mailbox!\n",
                      (uint32_t)handler);
                return ERROR;
              }

            priv->handler = handler;
          break;
        }

      default:
        {
          ferr("Unrecognized IOCTL command: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_lp_mailbox_init
 *
 * Description:
 *   Initialize and register LP mailbox driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_lp_mailbox_init(void)
{
  int ret = lp_core_mailbox_init(&esp_lp_mailbox_priv.mailbox,
                                 &esp_lp_mailbox_priv.config);

  if (ret != OK)
    {
      ferr("Failed to initialize LP Mailbox driver: %d\n", ret);
      return ret;
    }

  register_driver("/dev/lp_mailbox", esp_lp_mailbox_priv.ops,
                  0666, (void *)&esp_lp_mailbox_priv);
  return OK;
}
