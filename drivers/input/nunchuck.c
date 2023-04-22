/****************************************************************************
 * drivers/input/nunchuck.c
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

/* This file provides a driver for a Nintendo Wii Nunchuck joystick device.
 * The nunchuck joystick provides X/Y positional data as integer values.
 * The analog positional data may also be accompanied by discrete button
 * data.
 *
 * The nunchuck joystick driver exports a standard character driver
 * interface. By convention, the nunchuck joystick is registered as an input
 * device at /dev/nunchuckN where N uniquely identifies the driver instance.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/nunchuck.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one nunchuck joystick driver */

struct nunchuck_dev_s
{
  FAR struct i2c_master_s *i2c_dev; /* I2C interface connected to Nunchuck */
  nunchuck_buttonset_t nck_sample;  /* Last sampled button states */
  mutex_t nck_lock;                 /* Supports exclusive access to the device */

  /* The following is a singly linked list of open references to the
   * joystick device.
   */

  FAR struct nunchuck_open_s *nck_open;
};

/* This structure describes the state of one open joystick driver instance */

struct nunchuck_open_s
{
  /* Supports a singly linked list */

  FAR struct nunchuck_open_s *nck_flink;

  /* The following will be true if we are closing */

  volatile bool nck_closing;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int     nunchuck_open(FAR struct file *filep);
static int     nunchuck_close(FAR struct file *filep);
static ssize_t nunchuck_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static int     nunchuck_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);

/* I2C Helpers */

static int     nunchuck_i2c_read(FAR struct nunchuck_dev_s *priv,
                                 FAR uint8_t *regval, int len);
static int     nunchuck_i2c_write(FAR struct nunchuck_dev_s *priv,
                                  uint8_t const *data, int len);
static int     nunchuck_sample(FAR struct nunchuck_dev_s *priv,
                               FAR struct nunchuck_sample_s *buffer);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_nunchuck_fops =
{
  nunchuck_open,  /* open */
  nunchuck_close, /* close */
  nunchuck_read,  /* read */
  NULL,           /* write */
  NULL,           /* seek */
  nunchuck_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nunchuck_i2c_read
 ****************************************************************************/

static int nunchuck_i2c_read(FAR struct nunchuck_dev_s *priv,
                             FAR uint8_t *regval, int len)
{
  struct i2c_config_s config;
  int ret = -1;

  /* Set up the I2C configuration */

  config.frequency = NUNCHUCK_I2C_FREQ;
  config.address   = NUNCHUCK_ADDR;
  config.addrlen   = 7;

  /* Read "len" bytes from regaddr */

  ret = i2c_read(priv->i2c_dev, &config, regval, len);
  if (ret < 0)
    {
      ierr ("i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: nunchuck_i2c_write
 ****************************************************************************/

static int nunchuck_i2c_write(FAR struct nunchuck_dev_s *priv,
                              uint8_t const *data, int len)
{
  struct i2c_config_s config;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = NUNCHUCK_I2C_FREQ;
  config.address   = NUNCHUCK_ADDR;
  config.addrlen   = 7;

  /* Write the data */

  ret = i2c_write(priv->i2c_dev, &config, data, len);
  if (ret < 0)
    {
      ierr("ERROR: i2c_write failed: %d\n", ret);
    }

  return ret;
}

static int nunchuck_sample(FAR struct nunchuck_dev_s *priv,
                           FAR struct nunchuck_sample_s *buffer)
{
  uint8_t cmd[2];
  uint8_t data[6];
  static bool initialized;

  if (!initialized)
    {
      /* Start device */

      cmd[0] = 0x40;
      cmd[1] = 0x00;
      nunchuck_i2c_write(priv, cmd, 2);

      /* Delay 20ms */

      nxsig_usleep(20 * 1000);

      initialized = true;
    }

  /* Prepare to read */

  cmd[0] = 0x00;
  nunchuck_i2c_write(priv, cmd, 1);

  /* Wait */

  nxsig_usleep(1000);

  /* Read data */

  nunchuck_i2c_read(priv, &data[0], 1);

  /* Wait */

  nxsig_usleep(1000);

  /* Wait */

  nxsig_usleep(1000);

  nunchuck_i2c_read(priv, &data[1], 1);

  /* Wait */

  nxsig_usleep(1000);

  nunchuck_i2c_read(priv, &data[2], 1);

  /* Wait */

  nxsig_usleep(1000);

  nunchuck_i2c_read(priv, &data[3], 1);

  /* Wait */

  nxsig_usleep(1000);

  nunchuck_i2c_read(priv, &data[4], 1);

  /* Wait */

  nxsig_usleep(1000);

  nunchuck_i2c_read(priv, &data[5], 1);

  /* Save the sample */

  buffer->js_x        = (uint16_t) data[0];
  buffer->js_y        = (uint16_t) data[1];
  buffer->acc_x       = (uint16_t) data[2];
  buffer->acc_y       = (uint16_t) data[3];
  buffer->acc_z       = (uint16_t) data[4];
  buffer->nck_buttons = (uint8_t) ((data[5] + 1) & 0x03);

  iinfo("X: %03d | Y: %03d | AX: %03d AY: %03d AZ: %03d | B: %d\n",
        data[0], data[1], data[2], data[3],
        data[4], ((data[5] + 1) & 0x03));

  return OK;
}

/****************************************************************************
 * Name: nunchuck_open
 ****************************************************************************/

static int nunchuck_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct nunchuck_dev_s *priv;
  FAR struct nunchuck_open_s *opriv;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = (FAR struct nunchuck_dev_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Allocate a new open structure */

  opriv = (FAR struct nunchuck_open_s *)
               kmm_zalloc(sizeof(struct nunchuck_open_s));
  if (!opriv)
    {
      ierr("ERROR: Failed to allocate open structure\n");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Attach the open structure to the device */

  opriv->nck_flink = priv->nck_open;
  priv->nck_open = opriv;

  /* Attach the open structure to the file structure */

  filep->f_priv = (FAR void *)opriv;
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: nunchuck_close
 ****************************************************************************/

static int nunchuck_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct nunchuck_dev_s *priv;
  FAR struct nunchuck_open_s *opriv;
  FAR struct nunchuck_open_s *curr;
  FAR struct nunchuck_open_s *prev;
  irqstate_t flags;
  bool closing;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct nunchuck_dev_s *)inode->i_private;

  /* Handle an improbable race conditions with the following atomic test
   * and set.
   *
   * This is actually a pretty feeble attempt to handle this.  The
   * improbable race condition occurs if two different threads try to
   * close the joystick driver at the same time.  The rule:  don't do
   * that!  It is feeble because we do not really enforce stale pointer
   * detection anyway.
   */

  flags = enter_critical_section();
  closing = opriv->nck_closing;
  opriv->nck_closing = true;
  leave_critical_section(flags);

  if (closing)
    {
      /* Another thread is doing the close */

      return OK;
    }

  /* Get exclusive access to the driver structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Find the open structure in the list of open structures for the device */

  for (prev = NULL, curr = priv->nck_open;
       curr && curr != opriv;
       prev = curr, curr = curr->nck_flink);

  DEBUGASSERT(curr);
  if (!curr)
    {
      ierr("ERROR: Failed to find open entry\n");
      ret = -ENOENT;
      goto errout_with_lock;
    }

  /* Remove the structure from the device */

  if (prev)
    {
      prev->nck_flink = opriv->nck_flink;
    }
  else
    {
      priv->nck_open = opriv->nck_flink;
    }

  /* And free the open structure */

  kmm_free(opriv);

  ret = OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: nunchuck_read
 ****************************************************************************/

static ssize_t nunchuck_read(FAR struct file *filep, FAR char *buffer,
                         size_t len)
{
  FAR struct inode *inode;
  FAR struct nunchuck_dev_s *priv;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct nunchuck_dev_s *)inode->i_private;

  /* Make sure that the buffer is sufficiently large to hold at least one
   * complete sample.
   *
   * REVISIT:  Should also check buffer alignment.
   */

  if (len < sizeof(struct nunchuck_sample_s))
    {
      ierr("ERROR: buffer too small: %lu\n", (unsigned long)len);
      return -EINVAL;
    }

  /* Get exclusive access to the driver structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Read and return the current state of the joystick buttons */

  ret = nunchuck_sample(priv, (FAR struct nunchuck_sample_s *)buffer);
  if (ret >= 0)
    {
      ret = sizeof(struct nunchuck_sample_s);
    }

  nxmutex_unlock(&priv->lock);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: nunchuck_ioctl
 ****************************************************************************/

static int nunchuck_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct nunchuck_dev_s *priv;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct nunchuck_dev_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Handle the ioctl command */

  ret = -EINVAL;
  switch (cmd)
    {
    /* Command:     NUNCHUCKIOC_SUPPORTED
     * Description: Report the set of button events supported by the
     *              hardware;
     * Argument:    A pointer to writeable integer value in which to
     *              return the set of supported buttons.
     * Return:      Zero (OK) on success.  Minus one will be returned
     *              on failure with the errno value set appropriately.
     */

    case NUNCHUCKIOC_SUPPORTED:
      {
        FAR int *supported = (FAR int *)((uintptr_t)arg);

        if (supported)
          {
            *supported = (NUNCHUCK_BUTTON_Z_BIT | NUNCHUCK_BUTTON_C_BIT);
            ret = OK;
          }
      }
      break;

    default:
      ierr("ERROR: Unrecognized command: %ld\n", cmd);
      ret = -ENOTTY;
      break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nunchuck_register
 *
 * Description:
 *   Register the Nunchuck character driver as the specified device.
 *
 * Input Parameters:
 *   devname - The name of the Nunchuck joystick device to be registered.
 *     This should be a string of the form "/dev/nunchuckN" where N is the
 *     minor device number.
 *   i2c - An instance of the platform-specific I2C connected to Nunchuck.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nunchuck_register(FAR const char *devname, FAR struct i2c_master_s *i2c)
{
  FAR struct nunchuck_dev_s *priv;
  int ret;

  iinfo("Enter\n");

  DEBUGASSERT(devname && i2c);

  /* Allocate a new nunchuck driver instance */

  priv = (FAR struct nunchuck_dev_s *)
    kmm_zalloc(sizeof(struct nunchuck_dev_s));

  if (!priv)
    {
      ierr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Save the i2c device */

  priv->i2c_dev = i2c;

  /* Initialize the new nunchuck driver instance */

  nxmutex_init(&priv->nck_lock);

  /* And register the nunchuck driver */

  ret = register_driver(devname, &g_nunchuck_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver failed: %d\n", ret);
      goto errout_with_priv;
    }

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->nck_lock);
  kmm_free(priv);
  return ret;
}
