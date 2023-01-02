/****************************************************************************
 * drivers/spi/spi_slave_driver.c
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

#include <semaphore.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/slave.h>

#ifdef CONFIG_SPI_SLAVE_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME_FMT    "/dev/spislv%d"
#define DEVNAME_FMTLEN (11 + 3 + 1)

#ifndef MIN
#  define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define WORDS2BYTES(_wn)   ((_wn) * (CONFIG_SPI_SLAVE_DRIVER_WIDTH / 8))
#define BYTES2WORDS(_bn)   ((_bn) / (CONFIG_SPI_SLAVE_DRIVER_WIDTH / 8))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spi_slave_driver_s
{
  /* Externally visible part of the SPI Slave device interface */

  struct spi_slave_dev_s dev;

  /* Reference to SPI Slave controller interface */

  struct spi_slave_ctrlr_s *ctrlr;

  /* Receive buffer */

  uint8_t rx_buffer[CONFIG_SPI_SLAVE_DRIVER_BUFFER_SIZE];
  uint32_t rx_length;         /* Location of next RX value */

  /* Transmit buffer */

  uint8_t tx_buffer[CONFIG_SPI_SLAVE_DRIVER_BUFFER_SIZE];
  uint32_t tx_length;         /* Location of next TX value */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  mutex_t lock;               /* Mutual exclusion */
  int16_t crefs;              /* Number of open references */
  bool unlinked;              /* Indicates if the driver has been unlinked */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int     spi_slave_open(FAR struct file *filep);
static int     spi_slave_close(FAR struct file *filep);
static ssize_t spi_slave_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t spi_slave_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int     spi_slave_unlink(FAR struct inode *inode);

/* SPI Slave driver methods */

static void    spi_slave_select(FAR struct spi_slave_dev_s *sdev,
                                bool selected);
static void    spi_slave_cmddata(FAR struct spi_slave_dev_s *sdev,
                                 bool data);
static size_t  spi_slave_getdata(FAR struct spi_slave_dev_s *sdev,
                                 FAR const void **data);
static size_t  spi_slave_receive(FAR struct spi_slave_dev_s *sdev,
                                 FAR const void *data, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_spislavefops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  spi_slave_open,               /* open */
  spi_slave_close,              /* close */
#else
  NULL,                         /* open */
  NULL,                         /* close */
#endif
  spi_slave_read,               /* read */
  spi_slave_write,              /* write */
  NULL,                         /* seek */
  NULL,                         /* ioctl */
  NULL,                         /* mmap */
  NULL,                         /* truncate */
  NULL                          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , spi_slave_unlink            /* unlink */
#endif
};

static const struct spi_slave_devops_s g_spisdev_ops =
{
  spi_slave_select,             /* select */
  spi_slave_cmddata,            /* cmddata */
  spi_slave_getdata,            /* getdata */
  spi_slave_receive,            /* receive */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_slave_open
 *
 * Description:
 *   This function is called whenever the SPI Slave device is opened.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int spi_slave_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct spi_slave_driver_s *priv;
  int ret;

  DEBUGASSERT(filep != NULL);
  DEBUGASSERT(filep->f_inode != NULL);
  DEBUGASSERT(filep->f_inode->i_private != NULL);

  spiinfo("filep: %p\n", filep);

  /* Get our private data structure */

  inode = filep->f_inode;
  priv = (FAR struct spi_slave_driver_s *)inode->i_private;

  /* Get exclusive access to the SPI Slave driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      spierr("Failed to get exclusive access to the driver: %d\n", ret);
      return ret;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: spi_slave_close
 *
 * Description:
 *   This routine is called when the SPI Slave device is closed.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int spi_slave_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct spi_slave_driver_s *priv;
  int ret;

  DEBUGASSERT(filep != NULL);
  DEBUGASSERT(filep->f_inode != NULL);
  DEBUGASSERT(filep->f_inode->i_private != NULL);

  spiinfo("filep: %p\n", filep);

  /* Get our private data structure */

  inode = filep->f_inode;
  priv = (FAR struct spi_slave_driver_s *)inode->i_private;

  /* Get exclusive access to the SPI Slave driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      spierr("Failed to get exclusive access to the driver: %d\n", ret);
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been already
   * unlinked, then dispose of the driver resources.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      inode->i_private = NULL;
      return OK;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: spi_slave_read
 *
 * Description:
 *   This routine is called when the application requires to read the data
 *   received by the SPI Slave device.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   buffer - User-provided to save the data
 *   buflen - The maximum size of the user-provided buffer
 *
 * Returned Value:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

static ssize_t spi_slave_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct spi_slave_driver_s *priv;
  size_t read_bytes;
  size_t remaining_words;

  spiinfo("filep=%p buffer=%p buflen=%zu\n", filep, buffer, buflen);

  /* Get our private data structure */

  inode = filep->f_inode;
  priv  = (FAR struct spi_slave_driver_s *)inode->i_private;

  if (buffer == NULL)
    {
      spierr("ERROR: Buffer is NULL\n");
      return -ENOBUFS;
    }

  remaining_words = SPIS_CTRLR_QPOLL(priv->ctrlr);
  if (remaining_words == 0)
    {
      spiinfo("All words retrieved!\n");
    }
  else
    {
      spiinfo("%zu words left in the buffer\n", remaining_words);
    }

  read_bytes = MIN(buflen, priv->rx_length);

  memcpy(buffer, priv->rx_buffer, read_bytes);

  return (ssize_t)read_bytes;
}

/****************************************************************************
 * Name: spi_slave_write
 *
 * Description:
 *   This routine is called when the application needs to enqueue data to be
 *   transferred at the next leading clock edge of the SPI Slave controller.
 *
 * Input Parameters:
 *   filep  - Instance of struct file to use with the write
 *   buffer - Data to write
 *   buflen - Length of data to write in bytes
 *
 * Returned Value:
 *   On success, the number of bytes written are returned (zero indicates
 *   nothing was written). On any failure, a negated errno value is returned.
 *
 ****************************************************************************/

static ssize_t spi_slave_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  FAR struct inode *inode;
  FAR struct spi_slave_driver_s *priv;
  size_t num_words;
  size_t enqueued_bytes;

  spiinfo("filep=%p buffer=%p buflen=%zu\n", filep, buffer, buflen);

  /* Get our private data structure */

  inode = filep->f_inode;
  priv = (FAR struct spi_slave_driver_s *)inode->i_private;

  memcpy(priv->tx_buffer, buffer, buflen);
  priv->tx_length = buflen;
  num_words = BYTES2WORDS(priv->tx_length);

  enqueued_bytes = WORDS2BYTES(SPIS_CTRLR_ENQUEUE(priv->ctrlr,
                                                  priv->tx_buffer,
                                                  num_words));

  spiinfo("%zu bytes enqueued\n", enqueued_bytes);

  return (ssize_t)enqueued_bytes;
}

/****************************************************************************
 * Name: spi_slave_unlink
 *
 * Description:
 *   This routine is called when the SPI Slave device is unlinked.
 *
 * Input Parameters:
 *   inode  - The inode associated with the SPI Slave device
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int spi_slave_unlink(FAR struct inode *inode)
{
  FAR struct spi_slave_driver_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL);
  DEBUGASSERT(inode->i_private != NULL);

  /* Get our private data structure */

  priv = (FAR struct spi_slave_driver_s *)inode->i_private;

  /* Get exclusive access to the SPI Slave driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      spierr("Failed to get exclusive access to the driver: %d\n", ret);
      return ret;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      inode->i_private = NULL;
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resources when the
   * last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: spi_slave_select
 *
 * Description:
 *   This is a SPI device callback that is used when the SPI controller
 *   driver detects any change in the chip select pin.
 *
 * Input Parameters:
 *   dev      - SPI Slave device interface instance
 *   selected - Indicates whether the chip select is in active state
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   May be called from an interrupt handler. Processing should be as
 *   brief as possible.
 *
 ****************************************************************************/

static void spi_slave_select(FAR struct spi_slave_dev_s *dev, bool selected)
{
  spiinfo("sdev: %p CS: %s\n", dev, selected ? "select" : "free");
}

/****************************************************************************
 * Name: spi_slave_cmddata
 *
 * Description:
 *   This is a SPI device callback that is used when the SPI controller
 *   driver detects any change in the command/data condition.
 *
 *   Normally only LCD devices distinguish command and data. For devices
 *   that do not distinguish between command and data, this method may be
 *   a stub. For devices that do make that distinction, they should treat
 *   all subsequent calls to getdata() or receive() appropriately for the
 *   current command/data selection.
 *
 * Input Parameters:
 *   dev  - SPI Slave device interface instance
 *   data - True: Data is selected
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   May be called from an interrupt handler. Processing should be as
 *   brief as possible.
 *
 ****************************************************************************/

static void spi_slave_cmddata(FAR struct spi_slave_dev_s *dev, bool data)
{
  spiinfo("sdev: %p CMD: %s\n", dev, data ? "data" : "command");
}

/****************************************************************************
 * Name: spi_slave_getdata
 *
 * Description:
 *   This is a SPI device callback that is used when the SPI controller
 *   requires data be shifted out at the next leading clock edge. This
 *   is necessary to "prime the pump" so that the SPI controller driver
 *   can keep pace with the shifted-in data.
 *
 *   The SPI controller driver will prime for both command and data
 *   transfers as determined by a preceding call to the device cmddata()
 *   method. Normally only LCD devices distinguish command and data.
 *
 * Input Parameters:
 *   dev  - SPI Slave device interface instance
 *   data - Pointer to the data buffer pointer to be shifed out.
 *          The device will set the data buffer pointer to the actual data
 *
 * Returned Value:
 *   The number of data units to be shifted out from the data buffer.
 *
 * Assumptions:
 *   May be called from an interrupt handler and the response is usually
 *   time-critical.
 *
 ****************************************************************************/

static size_t spi_slave_getdata(FAR struct spi_slave_dev_s *dev,
                                FAR const void **data)
{
  FAR struct spi_slave_driver_s *priv = (FAR struct spi_slave_driver_s *)dev;

  *data = priv->tx_buffer;

  return BYTES2WORDS(priv->tx_length);
}

/****************************************************************************
 * Name: spi_slave_receive
 *
 * Description:
 *   This is a SPI device callback that is used when the SPI controller
 *   receives a new value shifted in. Notice that these values may be out of
 *   synchronization by several words.
 *
 * Input Parameters:
 *   dev  - SPI Slave device interface instance
 *   data - Pointer to the new data that has been shifted in
 *   len  - Length of the new data in units of nbits wide,
 *          nbits being the data width previously provided to the bind()
 *          method.
 *
 * Returned Value:
 *   Number of units accepted by the device. In other words,
 *   number of units to be removed from controller's receive queue.
 *
 * Assumptions:
 *   May be called from an interrupt handler and in time-critical
 *   circumstances. A good implementation might just add the newly
 *   received word to a queue, post a processing task, and return as
 *   quickly as possible to avoid any data overrun problems.
 *
 ****************************************************************************/

static size_t spi_slave_receive(FAR struct spi_slave_dev_s *dev,
                                FAR const void *data, size_t len)
{
  FAR struct spi_slave_driver_s *priv = (FAR struct spi_slave_driver_s *)dev;
  size_t recv_bytes = MIN(len, sizeof(priv->rx_buffer));

  memcpy(priv->rx_buffer, data, recv_bytes);

  priv->rx_length = recv_bytes;

  return BYTES2WORDS(recv_bytes);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_slave_register
 *
 * Description:
 *   Register the SPI Slave character device driver as 'devpath'.
 *
 * Input Parameters:
 *   ctrlr - An instance of the SPI Slave interface to use to communicate
 *           with the SPI Slave device
 *   bus   - The SPI Slave bus number. This will be used as the SPI device
 *           minor number. The SPI Slave character device will be
 *           registered as /dev/spislvN where N is the minor number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int spi_slave_register(FAR struct spi_slave_ctrlr_s *ctrlr, int bus)
{
  FAR struct spi_slave_driver_s *priv;
  char devname[DEVNAME_FMTLEN];
  int ret;

  /* Sanity check */

  DEBUGASSERT(ctrlr != NULL && (unsigned int)bus < 1000);

  /* Initialize the SPI Slave device structure */

  priv = (FAR struct spi_slave_driver_s *)
    kmm_zalloc(sizeof(struct spi_slave_driver_s));
  if (!priv)
    {
      spierr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->dev.ops = &g_spisdev_ops;
  priv->ctrlr = ctrlr;

#ifdef CONFIG_SPI_SLAVE_DRIVER_COLORIZE_TX_BUFFER
  memset(priv->tx_buffer,
         CONFIG_SPI_SLAVE_DRIVER_COLORIZE_PATTERN,
         CONFIG_SPI_SLAVE_DRIVER_COLORIZE_NUM_BYTES);
  priv->tx_length = CONFIG_SPI_SLAVE_DRIVER_COLORIZE_NUM_BYTES;
#endif

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  nxmutex_init(&priv->lock);
#endif

  /* Create the character device name */

  snprintf(devname, sizeof(devname), DEVNAME_FMT, bus);

  /* Register the character driver */

  ret = register_driver(devname, &g_spislavefops, 0666, priv);
  if (ret < 0)
    {
      spierr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }

  SPIS_CTRLR_BIND(priv->ctrlr, (FAR struct spi_slave_dev_s *)priv,
                  CONFIG_SPI_SLAVE_DRIVER_MODE,
                  CONFIG_SPI_SLAVE_DRIVER_WIDTH);

  spiinfo("SPI Slave driver loaded successfully!\n");

  return ret;
}

#endif /* CONFIG_SPI_SLAVE_DRIVER */
