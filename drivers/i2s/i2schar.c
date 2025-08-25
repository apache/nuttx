/****************************************************************************
 * drivers/i2s/i2schar.c
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
 *
 * This is a simple character driver for testing I2C.  It is not an audio
 * driver but does conform to some of the buffer management heuristics of an
 * audio driver.  It is not suitable for use in any real driver application
 * in its current form.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <semaphore.h>

#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_AUDIO_I2SCHAR_RXTIMEOUT
#  define CONFIG_AUDIO_I2SCHAR_RXTIMEOUT 0
#endif

#ifndef CONFIG_AUDIO_I2SCHAR_TXTIMEOUT
#  define CONFIG_AUDIO_I2SCHAR_TXTIMEOUT 0
#endif

/* Device naming ************************************************************/
#define DEVNAME_FMT    "/dev/i2schar%d"
#define DEVNAME_FMTLEN (12 + 3 + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct i2schar_dev_s
{
  FAR struct i2s_dev_s *i2s;  /* The lower half i2s driver */
  mutex_t rx_lock;            /* Assures mutually exclusive access to RX operations */
  mutex_t tx_lock;            /* Assures mutually exclusive access to TX operations */
  sem_t rx_sem;               /* Semaphore for blocking read operations */
  sem_t tx_sem;               /* Semaphore for blocking write operations */
  int rx_result;              /* Result of the last read operation */
  int tx_result;              /* Result of the last write operation */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2S callback function */

static void i2schar_rxcallback(FAR struct i2s_dev_s *dev,
                               FAR struct ap_buffer_s *apb,
                               FAR void *arg,
                               int result);
static void i2schar_txcallback(FAR struct i2s_dev_s *dev,
                               FAR struct ap_buffer_s *apb,
                               FAR void *arg,
                               int result);

/* Character driver methods */

static ssize_t i2schar_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t i2schar_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int i2schar_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_i2schar_fops =
{
  NULL,                 /* open  */
  NULL,                 /* close */
  i2schar_read,         /* read  */
  i2schar_write,        /* write */
  NULL,                 /* seek  */
  i2schar_ioctl,        /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2schar_rxcallback
 *
 * Description:
 *   Callback function invoked upon completion of an I2S RX (receive)
 *   transfer. This function is called by the lower-half I2S driver when
 *   the reception of an audio buffer is complete.
 *
 *   In this test driver implementation, the received buffer is simply
 *   freed. This is acceptable if this driver has the sole reference to the
 *   buffer; in that case, the buffer will be freed. Otherwise, this may
 *   result in a memory leak. A more efficient design would recycle the
 *   audio buffers for future use.
 *
 *   In a real application, the received data would typically be returned
 *   to the caller or passed to another subsystem via IPC or a queue.
 *
 * Input Parameters:
 *   dev    - Pointer to the I2S device structure
 *   apb    - Pointer to the audio buffer structure that was received
 *   arg    - Pointer to the private data structure (i2schar_dev_s)
 *   result - The result of the transfer (OK on success, negated errno on
 *            failure)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2schar_rxcallback(FAR struct i2s_dev_s *dev,
                               FAR struct ap_buffer_s *apb,
                               FAR void *arg, int result)
{
  FAR struct i2schar_dev_s *priv = (FAR struct i2schar_dev_s *)arg;

  DEBUGASSERT(priv != NULL && apb != NULL);

  i2sinfo("apb=%p nbytes=%d result=%d\n", apb, apb->nbytes, result);

  /* Store the result and signal completion */

  priv->rx_result = result;

  /* Signal that the read operation has completed */

  nxsem_post(&priv->rx_sem);

  /* REVISIT: If you want this to actually do something other than
   * test I2S data transfer, then this is the point where you would
   * want to pass the received I2S to some application.
   */

  /* Release our reference to the audio buffer. Hopefully it will be freed
   * now.
   */

  i2sinfo("Freeing apb=%p crefs=%d\n", apb, apb->crefs);
  apb_free(apb);
}

/****************************************************************************
 * Name: i2schar_txcallback
 *
 * Description:
 *   Callback function invoked upon completion of an I2S TX (transmit)
 *   transfer. This function is called by the lower-half I2S driver when
 *   the transmission of an audio buffer is complete.
 *
 *   In this test driver implementation, the transmitted buffer is simply
 *   freed. This is acceptable if this driver has the sole reference to the
 *   buffer; in that case, the buffer will be freed. Otherwise, this may
 *   result in a memory leak. A more efficient design would recycle the
 *   audio buffers for future use.
 *
 * Input Parameters:
 *   dev    - Pointer to the I2S device structure
 *   apb    - Pointer to the audio buffer structure that was transmitted
 *   arg    - Pointer to the private data structure (i2schar_dev_s)
 *   result - The result of the transfer (OK on success, negated errno on
 *            failure)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2schar_txcallback(FAR struct i2s_dev_s *dev,
                               FAR struct ap_buffer_s *apb,
                               FAR void *arg, int result)
{
  FAR struct i2schar_dev_s *priv = (FAR struct i2schar_dev_s *)arg;

  DEBUGASSERT(priv != NULL && apb != NULL);

  i2sinfo("apb=%p nbytes=%d result=%d\n", apb, apb->nbytes, result);

  /* Store the result and signal completion */

  priv->tx_result = result;

  /* Signal that the write operation has completed */

  nxsem_post(&priv->tx_sem);

  /* REVISIT: If you want this to actually do something other than
   * test I2S data transfer, then this is the point where you would
   * want to let some application know that the transfer has completed.
   */

  /* Release our reference to the audio buffer.  Hopefully it will be freed
   * now.
   */

  i2sinfo("Freeing apb=%p crefs=%d\n", apb, apb->crefs);
  apb_free(apb);
}

/****************************************************************************
 * Name: i2schar_read
 *
 * Description:
 *   Standard character driver read method. This function reads audio data
 *   from the I2S character device. The data is expected to be provided in
 *   the form of a single, correctly sized audio buffer (struct ap_buffer_s
 *   followed by audio data). The function adds a reference to the buffer,
 *   passes it to the lower-half I2S driver for reception, and waits for the
 *   transfer to complete via a callback. Upon completion, the function
 *   returns the number of bytes read or a negated errno value on failure.
 *
 * Input Parameters:
 *   filep  - Pointer to the file structure instance
 *   buffer - Pointer to the buffer where the received audio data will be
 *            stored. This must point to a struct ap_buffer_s followed by
 *            sufficient space for the audio data.
 *   buflen - The length of the buffer in bytes. Must be at least the size of
 *            struct ap_buffer_s plus the number of audio data bytes.
 *
 * Returned Value:
 *   On success, returns the number of bytes read (including the
 *   struct ap_buffer_s header and the audio data). On failure, returns
 *   a negated errno value indicating the error.
 *
 ****************************************************************************/

static ssize_t i2schar_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode;
  FAR struct i2schar_dev_s *priv;
  FAR struct ap_buffer_s *apb;
  size_t nbytes;
  int ret;

  i2sinfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Get our private data structure */

  DEBUGASSERT(buffer != NULL);

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Verify that the buffer refers to one, correctly sized audio buffer */

  DEBUGASSERT(buflen >= sizeof(struct ap_buffer_s));

  apb    = (FAR struct ap_buffer_s *)buffer;
  nbytes = apb->nmaxbytes;
  DEBUGASSERT(buflen >= (sizeof(struct ap_buffer_s) + nbytes));

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Get exclusive access to i2c character driver */

  ret = nxmutex_lock(&priv->rx_lock);
  if (ret < 0)
    {
      i2serr("ERROR: nxsem_wait returned: %d\n", ret);
      goto errout_with_reference;
    }

  /* Give the buffer to the I2S driver */

  ret = I2S_RECEIVE(priv->i2s, apb, i2schar_rxcallback, priv,
                    CONFIG_AUDIO_I2SCHAR_RXTIMEOUT);
  if (ret < 0)
    {
      i2serr("ERROR: I2S_RECEIVE returned: %d\n", ret);
      goto errout_with_reference;
    }

  /* Wait for the RX callback to signal completion */

  ret = nxsem_wait(&priv->rx_sem);
  if (ret < 0)
    {
      i2serr("ERROR: nxsem_wait returned: %d\n", ret);
      goto errout_with_reference;
    }

  /* Get the result from the private data */

  ret = priv->rx_result;

  /* If the operation was successful, return the number of bytes received */

  if (ret >= 0)
    {
      ret = sizeof(struct ap_buffer_s) + apb->nbytes;
    }

  /* Release our reference to the audio buffer */

  nxmutex_unlock(&priv->rx_lock);
  return ret;

errout_with_reference:
  apb_free(apb);
  nxmutex_unlock(&priv->rx_lock);
  return ret;
}

/****************************************************************************
 * Name: i2schar_write
 *
 * Description:
 *   Standard character driver write method. This function writes audio data
 *   to the I2S character device. The data must be provided in the form of a
 *   single, correctly sized audio buffer (struct ap_buffer_s followed by
 *   audio data). The function adds a reference to the buffer, sends it to
 *   the lower-half I2S driver for transmission, and waits for the transfer
 *   to complete via a callback. Upon completion, the function returns the
 *   number of bytes written or a negated errno value on failure.
 *
 * Input Parameters:
 *   filep  - Pointer to the file structure instance
 *   buffer - Pointer to the buffer containing the audio data to be written.
 *            This must point to a struct ap_buffer_s followed by the audio
 *            data.
 *   buflen - The length of the buffer in bytes. Must be at least the size of
 *            struct ap_buffer_s plus the number of audio data bytes.
 *
 * Returned Value:
 *   On success, returns the number of bytes written (including the
 *   struct ap_buffer_s header and the audio data). On failure, returns
 *   a negated errno value indicating the error.
 *
 ****************************************************************************/

static ssize_t i2schar_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode;
  FAR struct i2schar_dev_s *priv;
  FAR struct ap_buffer_s *apb;
  size_t nbytes;
  int ret;

  i2sinfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Get our private data structure */

  DEBUGASSERT(buffer);

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  /* Verify that the buffer refers to one, correctly sized audio buffer */

  DEBUGASSERT(buflen >= sizeof(struct ap_buffer_s));

  apb    = (FAR struct ap_buffer_s *)buffer;
  nbytes = apb->nmaxbytes;
  DEBUGASSERT(buflen >= (sizeof(struct ap_buffer_s) + nbytes));

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Get exclusive access to i2c character driver */

  ret = nxmutex_lock(&priv->tx_lock);
  if (ret < 0)
    {
      i2serr("ERROR: nxsem_wait returned: %d\n", ret);
      goto errout_with_reference;
    }

  /* Give the audio buffer to the I2S driver */

  ret = I2S_SEND(priv->i2s, apb, i2schar_txcallback, priv,
                 CONFIG_AUDIO_I2SCHAR_TXTIMEOUT);
  if (ret < 0)
    {
      i2serr("ERROR: I2S_SEND returned: %d\n", ret);
      goto errout_with_reference;
    }

  /* Wait for the TX callback to signal completion */

  ret = nxsem_wait(&priv->tx_sem);
  if (ret < 0)
    {
      i2serr("ERROR: nxsem_wait returned: %d\n", ret);
      goto errout_with_reference;
    }

  /* Get the result from the private data */

  ret = priv->tx_result;

  /* If the operation was successful, return the number of bytes sent */

  if (ret >= 0)
    {
      ret = sizeof(struct ap_buffer_s) + apb->nbytes;
    }

  /* Release our reference to the audio buffer */

  nxmutex_unlock(&priv->tx_lock);
  return ret;

errout_with_reference:
  apb_free(apb);
  nxmutex_unlock(&priv->tx_lock);
  return ret;
}

/****************************************************************************
 * Name: i2schar_ioctl
 *
 * Description:
 *   Perform device-specific operations on the I2S character device via ioctl
 *   commands. This function handles a set of standard I2S ioctl commands for
 *   getting and setting data width, channel count, and sample rate for both
 *   RX and TX directions. If the command is not recognized, it is forwarded
 *   to the lower-half I2S driver's i2s_ioctl method if available.
 *
 * Input Parameters:
 *   filep - Pointer to the file structure instance
 *   cmd   - The ioctl command to be performed
 *   arg   - The argument accompanying the ioctl command (may be a pointer or
 *           a value, depending on the command)
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value on failure.
 *   If the command is not supported, returns -ENOTTY.
 *
 ****************************************************************************/

static int i2schar_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct i2schar_dev_s *priv;
  int ret = OK;

  /* Get our private data structure */

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv != NULL && priv->i2s && priv->i2s->ops);

  switch (cmd)
    {
      case I2SIOC_GRXDATAWIDTH:
        {
          *(FAR uint32_t *)arg = I2S_RXDATAWIDTH(priv->i2s, 0);
          break;
        }

      case I2SIOC_GTXDATAWIDTH:
        {
          *(FAR uint32_t *)arg = I2S_TXDATAWIDTH(priv->i2s, 0);
        }
        break;

      case I2SIOC_GRXCHANNELS:
        {
          *(FAR int *)arg = I2S_RXCHANNELS(priv->i2s, 0);
        }
        break;

      case I2SIOC_GTXCHANNELS:
        {
          *(FAR int *)arg = I2S_TXCHANNELS(priv->i2s, 0);
        }
        break;

      case I2SIOC_GRXSAMPLERATE:
        {
          *(FAR uint32_t *)arg = I2S_RXSAMPLERATE(priv->i2s, 0);
        }
        break;

      case I2SIOC_GTXSAMPLERATE:
        {
          *(FAR uint32_t *)arg = I2S_TXSAMPLERATE(priv->i2s, 0);
        }
        break;

      case I2SIOC_SRXDATAWIDTH:
        {
          *(FAR uint32_t *)arg = I2S_RXDATAWIDTH(priv->i2s, arg);
          break;
        }

      case I2SIOC_STXDATAWIDTH:
        {
          *(FAR uint32_t *)arg = I2S_TXDATAWIDTH(priv->i2s, arg);
        }
        break;

      case I2SIOC_SRXCHANNELS:
        {
          *(FAR int *)arg = I2S_RXCHANNELS(priv->i2s, arg);
        }
        break;

      case I2SIOC_STXCHANNELS:
        {
          *(FAR int *)arg = I2S_TXCHANNELS(priv->i2s, arg);
        }
        break;

      case I2SIOC_SRXSAMPLERATE:
        {
          *(FAR uint32_t *)arg = I2S_RXSAMPLERATE(priv->i2s, arg);
        }
        break;

      case I2SIOC_STXSAMPLERATE:
        {
          *(FAR uint32_t *)arg = I2S_TXSAMPLERATE(priv->i2s, arg);
        }
        break;

      default:
        {
          if (priv->i2s->ops->i2s_ioctl)
            {
              ret = priv->i2s->ops->i2s_ioctl(priv->i2s, cmd, arg);
            }
          else
            {
              ret = -ENOTTY;
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2schar_register
 *
 * Description:
 *   Create and register the I2S character driver.
 *
 *   The I2S character driver is a simple character driver that supports I2S
 *   transfers via a read() and write().  The intent of this driver is to
 *   support I2S testing.  It is not an audio driver but does conform to some
 *   of the buffer management heuristics of an audio driver.  It is not
 *   suitable for use in any real driver application in its current form.
 *
 * Input Parameters:
 *   i2s - An instance of the lower half I2S driver
 *   minor - The device minor number.  The I2S character device will be
 *     registers as /dev/i2scharN where N is the minor number
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int i2schar_register(FAR struct i2s_dev_s *i2s, int minor)
{
  FAR struct i2schar_dev_s *priv;
  char devname[DEVNAME_FMTLEN];
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2s != NULL && (unsigned)minor < 1000);

  /* Allocate a I2S character device structure */

  size_t dev_size = sizeof(struct i2schar_dev_s);
  priv = kmm_zalloc(dev_size);
  if (priv)
    {
      /* Initialize the I2S character device structure */

      priv->i2s = i2s;
      nxmutex_init(&priv->rx_lock);
      nxmutex_init(&priv->tx_lock);
      nxsem_init(&priv->rx_sem, 0, 0);
      nxsem_init(&priv->tx_sem, 0, 0);
      priv->rx_result = 0;
      priv->tx_result = 0;

      /* Create the character device name */

      snprintf(devname, sizeof(devname), DEVNAME_FMT, minor);
      ret = register_driver(devname, &g_i2schar_fops, 0666, priv);
      if (ret < 0)
        {
          /* Free the device structure if we failed to create the character
           * device.
           */

          nxsem_destroy(&priv->tx_sem);
          nxsem_destroy(&priv->rx_sem);
          nxmutex_destroy(&priv->rx_lock);
          nxmutex_destroy(&priv->tx_lock);
          kmm_free(priv);
          return ret;
        }

      /* Return the result of the registration */

      return OK;
    }

  return -ENOMEM;
}
