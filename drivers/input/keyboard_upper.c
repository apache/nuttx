/****************************************************************************
 * drivers/input/keyboard_upper.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <poll.h>

#include <nuttx/input/keyboard.h>
#include <nuttx/input/kbd_codec.h>
#include <nuttx/input/virtio-input-event-codes.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/circbuf.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct keyboard_opriv_s
{
  sem_t              waitsem;
  mutex_t            lock;
  struct circbuf_s   circ;
  struct list_node   node;
  FAR struct pollfd *fds;
};

/* This structure is for keyboard upper half driver */

struct keyboard_upperhalf_s
{
  mutex_t          lock;     /* Manages exclusive access to this structure */
  struct list_node head;     /* Head of list */
  FAR struct keyboard_lowerhalf_s
                  *lower;    /* A pointer of lower half instance */
  uint8_t          nums;     /* Number of buffer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     keyboard_open(FAR struct file *filep);
static int     keyboard_close(FAR struct file *filep);
static ssize_t keyboard_read(FAR struct file *filep, FAR char *buffer,
                             size_t len);
static ssize_t keyboard_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len);
static int     keyboard_poll(FAR struct file *filep, FAR struct pollfd *fds,
                             bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_keyboard_fops =
{
  keyboard_open,  /* open */
  keyboard_close, /* close */
  keyboard_read,  /* read */
  keyboard_write, /* write */
  NULL,           /* seek */
  NULL,           /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  keyboard_poll   /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: keyboard_open
 ****************************************************************************/

static int keyboard_open(FAR struct file *filep)
{
  FAR struct keyboard_opriv_s     *opriv = NULL;
  FAR struct inode                *inode = filep->f_inode;
  FAR struct keyboard_upperhalf_s *upper = inode->i_private;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  opriv = kmm_zalloc(sizeof(FAR struct keyboard_opriv_s));
  if (opriv == NULL)
    {
      nxmutex_unlock(&upper->lock);
      return -ENOMEM;
    }

  /* Initializes the buffer for each open file */

  ret = circbuf_init(&opriv->circ, NULL,
                     upper->nums * sizeof(struct keyboard_event_s));
  if (ret < 0)
    {
      kmm_free(opriv);
      nxmutex_unlock(&upper->lock);
      return ret;
    }

  /* Perform the operations required by the lower level */

  if (upper->lower->open)
    {
      ret = upper->lower->open(upper->lower);
      if (ret < 0)
        {
          circbuf_uninit(&opriv->circ);
          kmm_free(opriv);
          nxmutex_unlock(&upper->lock);
          return ret;
        }
    }

  nxsem_init(&opriv->waitsem, 0, 0);
  nxmutex_init(&opriv->lock);
  list_add_tail(&upper->head, &opriv->node);
  filep->f_priv = opriv;

  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Name: keyboard_close
 ****************************************************************************/

static int keyboard_close(FAR struct file *filep)
{
  FAR struct keyboard_opriv_s     *opriv = filep->f_priv;
  FAR struct inode                *inode = filep->f_inode;
  FAR struct keyboard_upperhalf_s *upper = inode->i_private;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Perform the operations required by the lower level */

  if (upper->lower->close)
    {
      ret = upper->lower->close(upper->lower);
      if (ret < 0)
        {
          goto out;
        }
    }

  list_delete(&opriv->node);
  circbuf_uninit(&opriv->circ);
  nxsem_destroy(&opriv->waitsem);
  nxmutex_destroy(&opriv->lock);
  kmm_free(opriv);

out:
  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Name: keyboard_read
 ****************************************************************************/

static ssize_t keyboard_read(FAR struct file *filep,
                             FAR char *buff, size_t len)
{
  FAR struct keyboard_opriv_s *opriv = filep->f_priv;
  int ret;

  /* Make sure that we have exclusive access to the private data structure */

  ret = nxmutex_lock(&opriv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Is there keyboard data now? */

  while (circbuf_is_empty(&opriv->circ))
    {
      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          ret = -EAGAIN;
          goto out;
        }
      else
        {
          nxmutex_unlock(&opriv->lock);
          ret = nxsem_wait_uninterruptible(&opriv->waitsem);
          if (ret < 0)
            {
              return ret;
            }

          ret = nxmutex_lock(&opriv->lock);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  ret = circbuf_read(&opriv->circ, buff, len);

out:
  nxmutex_unlock(&opriv->lock);
  return ret;
}

/****************************************************************************
 * Name: keyboard_poll
 ****************************************************************************/

static int keyboard_poll(FAR struct file *filep,
                         FAR struct pollfd *fds, bool setup)
{
  FAR struct keyboard_opriv_s *opriv = filep->f_priv;
  int ret;

  ret = nxmutex_lock(&opriv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      if (opriv->fds == NULL)
        {
          opriv->fds = fds;
          fds->priv  = &opriv->fds;
        }
      else
        {
          ret = -EBUSY;
          goto errout;
        }

      if (!circbuf_is_empty(&opriv->circ))
        {
          poll_notify(&fds, 1, POLLIN);
        }
    }
  else
    {
      opriv->fds = NULL;
      fds->priv  = NULL;
    }

errout:
  nxmutex_unlock(&opriv->lock);
  return ret;
}

/****************************************************************************
 * Name: keyboard_write
 ****************************************************************************/

static ssize_t keyboard_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct keyboard_upperhalf_s *upper = inode->i_private;
  FAR struct keyboard_lowerhalf_s *lower = upper->lower;

  if (lower->write != NULL)
    {
      return lower->write(lower, buffer, buflen);
    }

  return -ENOSYS;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: keyboard_register
 ****************************************************************************/

int keyboard_register(FAR struct keyboard_lowerhalf_s *lower,
                      FAR const char *path, uint8_t nums)
{
  FAR struct keyboard_upperhalf_s *upper;
  int ret;

  iinfo("Registering %s\n", path);
  if (lower == NULL)
    {
      ierr("ERROR: invalid touchscreen device\n");
      return -EINVAL;
    }

  upper = kmm_zalloc(sizeof(struct keyboard_upperhalf_s));
  if (upper == NULL)
    {
      ierr("ERROR: Failed to mem alloc!\n");
      return -ENOMEM;
    }

  upper->lower = lower;
  upper->nums  = nums;
  lower->priv  = upper;
  list_initialize(&upper->head);
  nxmutex_init(&upper->lock);

  ret = register_driver(path, &g_keyboard_fops, 0666, upper);
  if (ret < 0)
    {
      nxmutex_destroy(&upper->lock);
      kmm_free(upper);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: keyboard_unregister
 ****************************************************************************/

int keyboard_unregister(FAR struct keyboard_lowerhalf_s *lower,
                        FAR const char *path)
{
  FAR struct keyboard_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(lower->priv != NULL);

  upper = lower->priv;

  iinfo("UnRegistering %s\n", path);
  ret = unregister_driver(path);
  if (ret < 0)
    {
      return ret;
    }

  nxmutex_destroy(&upper->lock);
  kmm_free(upper);
  return 0;
}

/****************************************************************************
 * keyboard_event
 ****************************************************************************/

void keyboard_event(FAR struct keyboard_lowerhalf_s *lower, uint32_t keycode,
                    uint32_t type)
{
  FAR struct keyboard_upperhalf_s *upper = lower->priv;
  FAR struct keyboard_opriv_s     *opriv;
  struct keyboard_event_s          key;
  int semcount;

  if (nxmutex_lock(&upper->lock) < 0)
    {
      return;
    }

  key.code = keycode;
  key.type = type;
  list_for_every_entry(&upper->head, opriv, struct keyboard_opriv_s, node)
    {
      if (nxmutex_lock(&opriv->lock) == 0)
        {
          circbuf_overwrite(&opriv->circ, &key,
                            sizeof(struct keyboard_event_s));
          nxsem_get_value(&opriv->waitsem, &semcount);
          if (semcount < 1)
            {
              nxsem_post(&opriv->waitsem);
            }

          poll_notify(&opriv->fds, 1, POLLIN);
          nxmutex_unlock(&opriv->lock);
        }
    }

  nxmutex_unlock(&upper->lock);
}

/****************************************************************************
 * keyboard_translate_virtio_code
 ****************************************************************************/

uint32_t keyboard_translate_virtio_code(uint16_t keycode)
{
  switch (keycode)
    {
      case KEY_DELETE:
        return KEYCODE_FWDDEL;
      case KEY_BACKSPACE:
        return KEYCODE_BACKDEL;
      case KEY_HOME:
        return KEYCODE_HOME;
      case KEY_END:
        return KEYCODE_END;
      case KEY_LEFT:
        return KEYCODE_LEFT;
      case KEY_RIGHT:
        return KEYCODE_RIGHT;
      case KEY_UP:
        return KEYCODE_UP;
      case KEY_DOWN:
        return KEYCODE_DOWN;
      case KEY_PAGEUP:
        return KEYCODE_PAGEUP;
      case KEY_PAGEDOWN:
        return KEYCODE_PAGEDOWN;
      case KEY_ENTER:
        return KEYCODE_ENTER;
      case KEY_CAPSLOCK:
        return KEYCODE_CAPSLOCK;
      case KEY_SCROLLLOCK:
        return KEYCODE_SCROLLLOCK;
      case KEY_NUMLOCK:
        return KEYCODE_NUMLOCK;
      case KEY_SYSRQ:
        return KEYCODE_PRTSCRN;
      case KEY_F1:
        return KEYCODE_F1;
      case KEY_F2:
        return KEYCODE_F2;
      case KEY_F3:
        return KEYCODE_F3;
      case KEY_F4:
        return KEYCODE_F4;
      case KEY_F5:
        return KEYCODE_F5;
      case KEY_F6:
        return KEYCODE_F6;
      case KEY_F7:
        return KEYCODE_F7;
      case KEY_F8:
        return KEYCODE_F8;
      case KEY_F9:
        return KEYCODE_F9;
      case KEY_F10:
        return KEYCODE_F10;
      case KEY_F11:
        return KEYCODE_F11;
      case KEY_F12:
        return KEYCODE_F12;
      case KEY_F13:
        return KEYCODE_F13;
      case KEY_F14:
        return KEYCODE_F14;
      case KEY_F15:
        return KEYCODE_F15;
      case KEY_F16:
        return KEYCODE_F16;
      case KEY_F17:
        return KEYCODE_F17;
      case KEY_F18:
        return KEYCODE_F18;
      case KEY_F19:
        return KEYCODE_F19;
      case KEY_F20:
        return KEYCODE_F20;
      case KEY_F21:
        return KEYCODE_F21;
      case KEY_F22:
        return KEYCODE_F22;
      case KEY_F23:
        return KEYCODE_F23;
      case KEY_F24:
        return KEYCODE_F24;
      default:
        return keycode;
    }
}
