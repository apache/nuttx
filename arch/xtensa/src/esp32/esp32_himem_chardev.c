/****************************************************************************
 * arch/xtensa/src/esp32/esp32_himem_chardev.c
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

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <debug.h>
#include <sys/types.h>
#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/himem/himem.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <arch/esp32/esp32_himem_chardev.h>
#include "esp32_himem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HIMEM_UNMAPPED (-1)

struct himem_chardev_s
{
  struct list_node node;
  size_t size;
  esp_himem_handle_t mem_handle;
  char name[32];
};

struct himem_chardev_priv_s
{
  off_t offset; /* himem offset[byte]. update by seek(), read(), write(). */
};

static struct list_node g_himem_chardev_list
                          = LIST_INITIAL_VALUE(g_himem_chardev_list);
static esp_himem_rangehandle_t g_range_handle;
static mutex_t lock;
static size_t g_ram_offset; /* used by himem map/unmap */

static void *g_himem_ptr;  /* mapped himem pointer */

static struct file *g_mapped_filep; /* for multi device */

/****************************************************************************
 * Priavte Functions
 ****************************************************************************/

static int himem_chardev_open(struct file *filep)
{
  struct himem_chardev_priv_s *priv;

  priv = kmm_malloc(sizeof(struct himem_chardev_priv_s));
  if (priv == NULL)
    {
       merr("Failed to malloc.\n");
       return -1;
    }
  priv->offset = 0;
  filep->f_priv = priv;

  return 0;
}

static int himem_chardev_close(struct file *filep)
{
  struct himem_chardev_priv_s *priv = filep->f_priv;
  kmm_free(priv);

  return 0;
}

static int himem_chardev_read_write(int is_write, struct file *filep,
                                    char *buffer, size_t length)
{
  struct himem_chardev_s *dev = filep->f_inode->i_private;
  struct himem_chardev_priv_s *priv = filep->f_priv;
  size_t mmap_offset = 0;
  int ret = 0;
  size_t i = 0;
  size_t copy_size = 0;
  void *himem_ptr;
  size_t copy_range = 0;

  ret = nxmutex_lock(&lock);
  if (ret != 0)
    {
      merr("Failed to lock semaphore.\n");
      return -1;
    }

  while (i < length)
    {
      mmap_offset = priv->offset / ESP_HIMEM_BLKSZ;
      if (mmap_offset > (dev->size / ESP_HIMEM_BLKSZ))
        {
          merr("copy size(%d) over himem phy mem size(%d).\n",
                length, dev->size);
          goto err;
        }
      if ((mmap_offset != g_ram_offset) || (g_mapped_filep != filep))
        {
          if (g_ram_offset != HIMEM_UNMAPPED)
            {
              /* already mapped another page */

              ret = esp_himem_unmap(g_range_handle,
                                    g_himem_ptr, ESP_HIMEM_BLKSZ);
              if (ret != 0)
                {
                  merr("Failed to unmap himem.\n");
                  goto err;
                }
              g_ram_offset = HIMEM_UNMAPPED;
              g_mapped_filep = NULL;
            }
          ret = esp_himem_map(dev->mem_handle, g_range_handle,
                              mmap_offset * ESP_HIMEM_BLKSZ,
                              0, ESP_HIMEM_BLKSZ, 0, &g_himem_ptr);
          if (ret != 0)
            {
              merr("Failed to map himem.\n");
              goto err;
            }
          g_ram_offset = mmap_offset;
          g_mapped_filep = filep;
        }
      himem_ptr = g_himem_ptr + priv->offset % ESP_HIMEM_BLKSZ;
      copy_range = ESP_HIMEM_BLKSZ - priv->offset % ESP_HIMEM_BLKSZ;

      /* The case of crossing a phy mem page boundary */

      if (copy_range < (length - i))
        {
          if (is_write)
            {
              memcpy(himem_ptr, buffer + i, copy_range);
            }
          else
            {
              memcpy(buffer + i, himem_ptr, copy_range);
            }
          priv->offset += copy_range;
          copy_size += copy_range;
          i += copy_range;
        }
      else
        {
          if (is_write)
            {
              memcpy(himem_ptr, buffer + i, length - i);
            }
          else
            {
              memcpy(buffer + i, himem_ptr, length - i);
            }
          priv->offset += length - i;
          copy_size += length - i;
          i += length - i;
        }
    }
  nxmutex_unlock(&lock);
  return (int)(copy_size & INT_MAX);
err:
  nxmutex_unlock(&lock);
  return copy_size > 0 ? (int)(copy_size & INT_MAX) : -1;
}

static int himem_chardev_read(struct file *filep,
                              char *dst, size_t length)
{
  return himem_chardev_read_write(0, filep, dst, length);
}

static int himem_chardev_write(struct file *filep,
                               const char *src, size_t length)
{
  return himem_chardev_read_write(1, filep, (char *)src, length);
}

static off_t himem_chardev_seek(struct file *filep,
                                off_t offset, int whence)
{
  struct himem_chardev_priv_s *priv = filep->f_priv;
  struct himem_chardev_s *dev = filep->f_inode->i_private;
  nxmutex_lock(&lock);
  switch (whence)
    {
    case SEEK_SET:
      priv->offset = offset;
      break;
    case SEEK_CUR:
      priv->offset += offset;
      break;
    case SEEK_END:
      priv->offset = dev->size + offset;
      break;
    default:
      merr("invalid parameter: whence:%d\n", whence);
      nxmutex_unlock(&lock);
      return -1;
  }
  filep->f_pos = priv->offset;
  nxmutex_unlock(&lock);
  return priv->offset;
}

static int himem_chardev_ioctl(struct file *filep,
                               int cmd, unsigned long arg)
{
  return 0;
}

static const struct file_operations fops =
{
  .open = himem_chardev_open,
  .close = himem_chardev_close,
  .read = himem_chardev_read,
  .write = himem_chardev_write,
  .seek = himem_chardev_seek,
  .ioctl = himem_chardev_ioctl,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int himem_chardev_init(void)
{
  int ret = 0;
  ret = esp_himem_init();
  if (ret != 0)
    {
      merr("Failed to himem init.\n");
      return ret;
    }

  ret = esp_himem_alloc_map_range(ESP_HIMEM_BLKSZ, &g_range_handle); /* 32KB */
  if (ret != 0)
    {
      merr("Failed to alloc himem map.\n");
      return ret;
    }

  ret = nxmutex_init(&lock);
  if (ret != 0)
    {
      merr("Failed to init semaphore.\n");
      esp_himem_free_map_range(g_range_handle);
      return ret;
    }

  g_ram_offset = HIMEM_UNMAPPED;
  g_mapped_filep = NULL;
  return ret;
}

int himem_chardev_exit(void)
{
  int ret = 0;
  ret = nxmutex_destroy(&lock);
  if (ret != 0)
    {
      merr("Failed to destroy semaphore.\n");
      esp_himem_free_map_range(g_range_handle);
      return ret;
    }

  ret = esp_himem_free_map_range(g_range_handle);

  return ret;
}

int himem_chardev_register(char *name, size_t size)
{
  int ret = 0;
  struct himem_chardev_s *dev;
  dev = (struct himem_chardev_s *)kmm_malloc(sizeof(struct himem_chardev_s));
  if (dev == NULL)
    {
      merr("Failed to malloc.\n");
      return -1;
    }

  /* 32KB Alignment */

  size_t mod = size % ESP_HIMEM_BLKSZ;
  if (mod != 0)
    {
      size += (ESP_HIMEM_BLKSZ - mod);
    }

  dev->size = size;

  strncpy(dev->name, name, 32);
  ret = esp_himem_alloc(dev->size, &dev->mem_handle);
  if (ret != 0)
    {
      merr("Failed to alloc himem. size=%d\n", dev->size);
      kmm_free(dev);
      return ret;
    }

  ret = register_driver(dev->name, &fops, 0666, dev);
  if (ret != 0)
    {
      merr("Failed to register driver. dev=%s\n", dev->name);
      esp_himem_free(dev->mem_handle);
      kmm_free(dev);
      return ret;
    }

  ret = nxmutex_lock(&lock);
  if (ret != 0)
    {
      merr("Failed to lock semaphore.\n");
      unregister_driver(dev->name);
      esp_himem_free(dev->mem_handle);
      kmm_free(dev);
      return ret;
    }

  list_add_tail(&g_himem_chardev_list, &dev->node);
  ret = nxmutex_unlock(&lock);
  if (ret != 0)
    {
      merr("Failed to unlock semaphore.\n");
      list_delete(&dev->node);
      unregister_driver(dev->name);
      esp_himem_free(dev->mem_handle);
      kmm_free(dev);
      return ret;
    }

  return 0;
}

int himem_chardev_unregister(char *name)
{
  int ret = 0;
  int successed = 0;
  struct himem_chardev_s *dev;
  struct himem_chardev_s *tmp;
  nxmutex_lock(&lock);
  list_for_every_entry_safe(&g_himem_chardev_list, dev,
                            tmp, struct himem_chardev_s, node)
    {
      if (strcmp(name, dev->name) == 0)
        {
          if (g_ram_offset != HIMEM_UNMAPPED)
            {
              /* driver is mapping some page */

              ret = esp_himem_unmap(g_range_handle,
                                    g_himem_ptr, ESP_HIMEM_BLKSZ);
              if (ret != 0)
                {
                  merr("Failed to unmap himem.\n");
                  successed = -1;
                }

              g_ram_offset = HIMEM_UNMAPPED;
              g_mapped_filep = NULL;
            }

          ret = esp_himem_free(dev->mem_handle);
          if (ret != 0)
            {
              merr("Failed to free himem.\n");
              successed = -1;
            }

          ret = unregister_driver(dev->name);
          if (ret != 0)
            {
              merr("Failed to unregister driver. dev=%s\n", dev->name);
              successed = -1;
            }

          list_delete(&dev->node);
          kmm_free(dev);
          nxmutex_unlock(&lock);
          return successed;
        }
    }

  nxmutex_unlock(&lock);
  merr("dev=%s is not registerd.\n", name);

  return -1;
}
