/****************************************************************************
 * drivers/lcd/lcd_dev.c
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
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>

#include <nuttx/lcd/lcd_dev.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of the lcd_dev driver */

struct lcddev_dev_s
{
  FAR struct lcd_dev_s *lcd_ptr;
  struct lcd_planeinfo_s planeinfo;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int lcddev_ioctl(FAR struct file *filep, int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations lcddev_fops =
{
  NULL,         /* open */
  NULL,         /* close */
  NULL,         /* read */
  NULL,         /* write */
  NULL,         /* seek */
  lcddev_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lcddev_ioctl
 ****************************************************************************/

static int lcddev_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct lcddev_dev_s *priv;
  int ret = OK;

  priv = (FAR struct lcddev_dev_s *)filep->f_inode->i_private;

  switch (cmd)
    {
    case LCDDEVIO_GETRUN:
      {
        FAR struct lcddev_run_s *lcd_run =
            (FAR struct lcddev_run_s *)arg;

        ret = priv->planeinfo.getrun(priv->lcd_ptr, lcd_run->row,
                                     lcd_run->col, lcd_run->data,
                                     lcd_run->npixels);
      }
      break;
    case LCDDEVIO_PUTRUN:
      {
        FAR const struct lcddev_run_s *lcd_run =
            (FAR const struct lcddev_run_s *)arg;

        ret = priv->planeinfo.putrun(priv->lcd_ptr,
                                     lcd_run->row, lcd_run->col,
                                     lcd_run->data, lcd_run->npixels);
      }
      break;
    case LCDDEVIO_GETAREA:
      {
        FAR struct lcddev_area_s *lcd_area =
            (FAR struct lcddev_area_s *)arg;

        if (priv->planeinfo.getarea)
          {
            ret = priv->planeinfo.getarea(priv->lcd_ptr,
                                          lcd_area->row_start,
                                          lcd_area->row_end,
                                          lcd_area->col_start,
                                          lcd_area->col_end,
                                          lcd_area->data);
          }
        else
          {
            /* Emulate getarea() using getrun() */

            uint8_t *buf = lcd_area->data;
            size_t npixels = (lcd_area->col_end - lcd_area->col_start + 1);
            int row;

            for (row = lcd_area->row_start; row <= lcd_area->row_end; row++)
              {
                ret = priv->planeinfo.getrun(priv->lcd_ptr, row,
                                             lcd_area->col_start, buf,
                                             npixels);
                if (ret < 0)
                  {
                    break;
                  }

                buf += npixels * (priv->planeinfo.bpp >> 3);
              }
          }
      }
      break;
    case LCDDEVIO_PUTAREA:
      {
        FAR const struct lcddev_area_s *lcd_area =
            (FAR const struct lcddev_area_s *)arg;

        if (priv->planeinfo.putarea)
          {
            ret = priv->planeinfo.putarea(priv->lcd_ptr,
                                          lcd_area->row_start,
                                          lcd_area->row_end,
                                          lcd_area->col_start,
                                          lcd_area->col_end,
                                          lcd_area->data);
          }
        else
          {
            /* Emulate putarea() using putrun() */

            uint8_t *buf = lcd_area->data;
            size_t npixels = (lcd_area->col_end - lcd_area->col_start + 1);
            int row;

            for (row = lcd_area->row_start; row <= lcd_area->row_end; row++)
              {
                ret = priv->planeinfo.putrun(priv->lcd_ptr, row,
                                             lcd_area->col_start, buf,
                                             npixels);
                if (ret < 0)
                  {
                    break;
                  }

                buf += npixels * (priv->planeinfo.bpp >> 3);
              }
          }
      }
      break;
    case LCDDEVIO_GETPOWER:
      {
        *((FAR int *)arg) = priv->lcd_ptr->getpower(priv->lcd_ptr);
      }
      break;
    case LCDDEVIO_SETPOWER:
      {
        ret = priv->lcd_ptr->setpower(priv->lcd_ptr, (int)arg);
      }
      break;
    case LCDDEVIO_GETCONTRAST:
      {
        *((FAR int *)arg) = priv->lcd_ptr->getcontrast(priv->lcd_ptr);
      }
      break;
    case LCDDEVIO_SETCONTRAST:
      {
        ret = priv->lcd_ptr->setcontrast(priv->lcd_ptr, (unsigned int)arg);
      }
      break;
    case LCDDEVIO_GETPLANEINFO:
      {
        *((FAR struct lcd_planeinfo_s *)arg) = priv->planeinfo;
      }
      break;
    case LCDDEVIO_GETVIDEOINFO:
      {
        ret = priv->lcd_ptr->getvideoinfo(priv->lcd_ptr,
                                          (FAR struct fb_videoinfo_s *)arg);
      }
      break;
    case LCDDEVIO_SETPLANENO:
      {
        ret = priv->lcd_ptr->getplaneinfo(priv->lcd_ptr, (int)arg,
                                          &priv->planeinfo);
      }
      break;
#ifdef CONFIG_FB_CMAP
    case LCDDEVIO_GETCMAP:
      {
        FAR struct fb_cmap_s *cmap = (FAR struct fb_cmap_s *)arg;

        ret = priv->lcd_ptr->getcmap(priv->lcd_ptr, cmap);
      }
      break;
    case LCDDEVIO_PUTCMAP:
      {
        FAR const struct fb_cmap_s *cmap = (FAR const struct fb_cmap_s *)arg;

        ret = priv->lcd_ptr->putcmap(priv->lcd_ptr, cmap);
      }
      break;
#endif
#ifdef CONFIG_FB_HWCURSOR
    case LCDDEVIO_GETCURSOR:
      {
        FAR struct fb_cursorattrib_s *attrib =
            (FAR struct fb_cursorattrib_s *)arg;

        ret = priv->lcd_ptr->getcursor(priv->lcd_ptr, attrib);
      }
      break;
    case LCDDEVIO_SETCURSOR:
      {
        FAR struct fb_setcursor_s *settings =
            (FAR struct fb_setcursor_s *)arg;

        ret = priv->lcd_ptr->setcursor(priv->lcd_ptr, settings);
      }
      break;
#endif
    case LCDDEVIO_SETFRAMERATE:
      {
        ret = priv->lcd_ptr->setframerate(priv->lcd_ptr, (int)arg);
      }
      break;
    case LCDDEVIO_GETFRAMERATE:
      {
        *((FAR int *)arg) = priv->lcd_ptr->getframerate(priv->lcd_ptr);
      }
      break;
    default:
      ret = -EINVAL;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lcddev_register
 *
 * Description:
 *   Register the LCD character driver as /dev/lcdN.
 *
 * Input Parameters:
 *   devno - The LCD device number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int lcddev_register(int devno)
{
  FAR struct lcddev_dev_s *priv;
  int ret = OK;
  char devname[16];

  /* Allocate a new lcd_dev driver instance */

  priv = (FAR struct lcddev_dev_s *)kmm_zalloc(sizeof(struct lcddev_dev_s));

  if (!priv)
    {
      return -ENOMEM;
    }

  priv->lcd_ptr = board_lcd_getdev(devno);
  ret = priv->lcd_ptr->getplaneinfo(priv->lcd_ptr, 0, &priv->planeinfo);
  if (ret < 0)
    {
      goto err;
    }

  snprintf(devname, sizeof(devname), "/dev/lcd%i", devno);
  ret = register_driver(devname, &lcddev_fops, 0666, priv);
  if (ret < 0)
    {
      goto err;
    }

  return ret;
err:
  kmm_free(priv);
  return ret;
}
