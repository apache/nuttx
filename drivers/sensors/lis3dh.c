/****************************************************************************
 * drivers/sensors/lis3dh.c
 *
 *   Copyright (C) 2018 Extent3D. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/lis3dh.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_SPI) && defined(CONFIG_LIS3DH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LIS3DH_QUEUE_MAX 32
#define LIS3DH_FIFOBUF_SIZE ((32 * 6) + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lis3dh_dev_s
{
  FAR struct lis3dh_config_s *config;   /* Driver configuration */
  FAR struct spi_dev_s *spi;            /* Pointer to the SPI instance */
  struct work_s work;                   /* Work Queue */
  uint8_t power_mode;                   /* The power mode used to determine mg/digit */
  uint8_t odr;                          /* The current output data rate */
  sem_t readsem;                        /* Read notification semaphore */
  uint8_t fifobuf[LIS3DH_FIFOBUF_SIZE]; /* Raw FIFO buffer */
  struct lis3dh_sensor_data_s queue[LIS3DH_QUEUE_MAX];
  mutex_t queuelock;                    /* Queue exclusive lock */
  uint8_t queue_rpos;                   /* Queue read position */
  uint8_t queue_wpos;                   /* Queue write position */
  uint8_t queue_count;                  /* Number of elements in the queue */
};

struct lis3dh_sample_s
{
  int16_t x;
  int16_t y;
  int16_t z;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void lis3dh_read_register(FAR struct lis3dh_dev_s *dev,
                                 uint8_t const reg_addr, uint8_t *reg_data);
static void lis3dh_write_register(FAR struct lis3dh_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t const reg_data);
static void lis3dh_reset(FAR struct lis3dh_dev_s *dev);
static int lis3dh_ident(FAR struct lis3dh_dev_s *dev);
static int lis3dh_read_fifo(FAR struct lis3dh_dev_s *dev);
static int lis3dh_interrupt_handler(int irq, FAR void *context,
                                    FAR void *arg);
static void lis3dh_worker(FAR void *arg);
static int lis3dh_irq_enable(FAR struct lis3dh_dev_s *dev, bool enable);
static int lis3dh_fifo_enable(FAR struct lis3dh_dev_s *dev);

static int lis3dh_open(FAR struct file *filep);
static int lis3dh_close(FAR struct file *filep);
static ssize_t lis3dh_read(FAR struct file *, FAR char *buffer,
                           size_t buflen);
static ssize_t lis3dh_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int lis3dh_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lis3dh_fops =
{
  lis3dh_open,     /* open */
  lis3dh_close,    /* close */
  lis3dh_read,     /* read */
  lis3dh_write,    /* write */
  NULL,            /* seek */
  lis3dh_ioctl,    /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3dh_read_register
 *
 * Description:
 *   Read a single register from the LIS3DH sensor.
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *   reg_addr - LIS3DH register address
 *   reg_data - Pointer to uint8_t where read value will be stored
 *
 ****************************************************************************/

static void lis3dh_read_register(FAR struct lis3dh_dev_s *dev,
                                  uint8_t const reg_addr, uint8_t * reg_data)
{
  uint8_t buffer[2];

  /* Lock the SPI bus so that only one device can access it at the same
   * time.
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3DH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  buffer[0] = reg_addr | 0x80;
  buffer[1] = 0;
  SPI_EXCHANGE(dev->spi, buffer, buffer, 2);
  *reg_data = buffer[1];

  /* Set CS to high which deselects the LIS3DH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3dh_write_register
 *
 * Description:
 *   Write a single register to the LIS3DH sensor.
 *
 * Input Parameters:
 *   dev      - Pointer to device driver instance
 *   reg_addr - LIS3DH register address
 *   reg_data - Value to write into the specified register address
 *
 ****************************************************************************/

static void lis3dh_write_register(FAR struct lis3dh_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t const reg_data)
{
  uint8_t buffer[2];

  /* Lock the SPI bus so that only one device can access it at the same
   * time.
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3DH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  buffer[0] = reg_addr;
  buffer[1] = reg_data;
  SPI_EXCHANGE(dev->spi, buffer, buffer, 2);

  /* Set CS to high which deselects the LIS3DH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3dh_reset
 *
 * Description:
 *   Perform a software reset of the LIS3DH sensor.
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *
 ****************************************************************************/

static void lis3dh_reset(FAR struct lis3dh_dev_s *dev)
{
  lis3dh_write_register(dev, LIS3DH_CTRL_REG5, LIS3DH_CTRL_REG5_BOOT);
  up_mdelay(100);
}

/****************************************************************************
 * Name: lis3dh_ident
 *
 * Description:
 *   Identify an LIS3DH sensor on the SPI bus using the WHO_AM_I register.
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *
 * Returned Value:
 *   OK if the device responded with the correct ID
 *   -ENODEV if the device did not respond with the correct ID
 *
 ****************************************************************************/

static int lis3dh_ident(FAR struct lis3dh_dev_s *dev)
{
  uint8_t reg;
  lis3dh_read_register(dev, LIS3DH_WHO_AM_I, &reg);

  if (reg == LIS3DH_DEVICE_ID)
    {
      return OK;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: lis3dh_queue_push
 *
 * Description:
 *   Push a sensor measurement into the queue
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *
 ****************************************************************************/

static int lis3dh_queue_push(FAR struct lis3dh_dev_s *dev,
                             struct lis3dh_sensor_data_s *data)
{
  nxmutex_lock(&dev->queuelock);
  if (dev->queue_count >= LIS3DH_QUEUE_MAX)
    {
      nxmutex_unlock(&dev->queuelock);
      return -ENOMEM;
    }

  dev->queue_wpos++;
  dev->queue[dev->queue_wpos % LIS3DH_QUEUE_MAX] = *data;

  dev->queue_count++;
  nxmutex_unlock(&dev->queuelock);

  return OK;
}

/****************************************************************************
 * Name: lis3dh_queue_pop
 *
 * Description:
 *   Pops a sensor measurement from the queue
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *
 * Returned Value:
 *   OK if a measurement was successfully dequeued
 *   -EAGAIN if the queue is empty
 *
 ****************************************************************************/

static int lis3dh_queue_pop(FAR struct lis3dh_dev_s *dev,
                            struct lis3dh_sensor_data_s *data)
{
  nxmutex_lock(&dev->queuelock);
  if (dev->queue_count == 0)
    {
      nxmutex_unlock(&dev->queuelock);
      return -EAGAIN;
    }

  dev->queue_rpos++;
  *data = dev->queue[dev->queue_rpos % LIS3DH_QUEUE_MAX];

  dev->queue_count--;

  nxmutex_unlock(&dev->queuelock);
  return OK;
}

/****************************************************************************
 * Name: lis3dh_read_fifo
 *
 * Description:
 *   Reads the FIFO from the LIS3DH sensor, and pushes to the internal queue.
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *
 * Returned Value:
 *
 ****************************************************************************/

static int lis3dh_read_fifo(FAR struct lis3dh_dev_s *dev)
{
  uint8_t fifosrc;
  uint8_t count;
  int i;

  /* Lock the SPI bus */

  SPI_LOCK(dev->spi, true);

  /* Read the FIFO source register */

  dev->fifobuf[0] = LIS3DH_FIFO_SRC_REG | 0x80;
  dev->fifobuf[1] = 0;

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);
  SPI_EXCHANGE(dev->spi, dev->fifobuf, dev->fifobuf, 2);
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  fifosrc = dev->fifobuf[1];

  count = fifosrc & 0x1f;

  if (fifosrc & LIS3DH_FIFO_SRC_REG_OVRN_FIFO)
    {
      snerr("FIFO overrun\n");
    }

  memset(dev->fifobuf, 0, LIS3DH_FIFOBUF_SIZE);
  dev->fifobuf[0] = LIS3DH_OUT_X_L | 0xc0;

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);
  SPI_EXCHANGE(dev->spi, dev->fifobuf, dev->fifobuf, 1 + count * 6);
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  for (i = 0; i < count; i++)
    {
      struct lis3dh_sensor_data_s data;
      uint16_t x_raw;
      uint16_t y_raw;
      uint16_t z_raw;

      int16_t x_acc;
      int16_t y_acc;
      int16_t z_acc;

      x_raw = (uint16_t)dev->fifobuf[(i * 6) + 1] |
              (uint16_t)dev->fifobuf[(i * 6) + 2] << 8;

      y_raw = (uint16_t)dev->fifobuf[(i * 6) + 3] |
              (uint16_t)dev->fifobuf[(i * 6) + 4] << 8;

      z_raw = (uint16_t)dev->fifobuf[(i * 6) + 5] |
              (uint16_t)dev->fifobuf[(i * 6) + 6] << 8;

      /* The sensor left justifies the data in the register, so we must
       * shift it to the right depending on the selected power mode
       */

      switch (dev->power_mode)
        {
          case LIS3DH_POWER_LOW:     /* 8 bit measurements */
            x_acc = (int16_t)x_raw >> 8;
            y_acc = (int16_t)y_raw >> 8;
            z_acc = (int16_t)z_raw >> 8;

            data.x_acc = (float)x_acc * 0.016;
            data.y_acc = (float)y_acc * 0.016;
            data.z_acc = (float)z_acc * 0.016;
            break;

          case LIS3DH_POWER_NORMAL:  /* 10 bit measurements */
            x_acc = (int16_t)x_raw >> 6;
            y_acc = (int16_t)y_raw >> 6;
            z_acc = (int16_t)z_raw >> 6;

            data.x_acc = (float)x_acc * 0.004;
            data.y_acc = (float)y_acc * 0.004;
            data.z_acc = (float)z_acc * 0.004;
            break;

          case LIS3DH_POWER_HIGH:    /* 12 bit measurements */
            x_acc = (int16_t)x_raw >> 4;
            y_acc = (int16_t)y_raw >> 4;
            z_acc = (int16_t)z_raw >> 4;

            data.x_acc = (float)x_acc * 0.001;
            data.y_acc = (float)y_acc * 0.001;
            data.z_acc = (float)z_acc * 0.001;
            break;

          default:
            snerr("Unknown power mode\n");
            return -EINVAL;
        }

      if (lis3dh_queue_push(dev, &data) == OK)
        {
          nxsem_post(&dev->readsem);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: lis3dh_interrupt_handler
 *
 * Description:
 *   Interrupt service routine, queues work on high priority work queue.
 *
 * Input Parameters:
 *   irq - Interrupt request number
 *   context - Context pointer
 *   arg - Pointer to device driver instance
 *
 * Returned Value:
 *   Returns OK to indicate the interrupt has been serviced.
 *
 ****************************************************************************/

static int lis3dh_interrupt_handler(int irq, FAR void *context,
                                    FAR void *arg)
{
  /* The interrupt handler is called when the FIFO watermark is reached */

  FAR struct lis3dh_dev_s *priv = (FAR struct lis3dh_dev_s *)arg;
  int ret;

  DEBUGASSERT(priv != NULL);

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, lis3dh_worker, priv, 0);
      if (ret < 0)
        {
          snerr("ERROR: Failed to queue work: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: lis3dh_worker
 *
 * Description:
 *   Worker callback executed from high priority work queue.
 *   Performs reading the FIFO from the sensor and pushing samples to
 *   the internal device driver queue.
 *
 * Input Parameters:
 *   arg - Pointer to device driver instance
 *
 ****************************************************************************/

static void lis3dh_worker(FAR void *arg)
{
  FAR struct lis3dh_dev_s *priv = (FAR struct lis3dh_dev_s *)(arg);

  DEBUGASSERT(priv != NULL);

  /* Read the FIFO and fill the queue */

  lis3dh_read_fifo(priv);
}

/****************************************************************************
 * Name: lis3dh_set_power_mode
 *
 * Description:
 *   Sets the power mode of the sensor.
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *   power_mode - LIS3DH_POWER_*
 *
 * Returned Value:
 *   OK - Power mode was set successfully
 *   -EINVAL - Invalid power mode argument
 *
 ****************************************************************************/

static int lis3dh_set_power_mode(FAR struct lis3dh_dev_s *dev,
                                 uint8_t power_mode)
{
  uint8_t ctrl1;
  uint8_t ctrl4;

  switch (power_mode)
    {
      case LIS3DH_POWER_LOW:
        lis3dh_read_register(dev, LIS3DH_CTRL_REG1, &ctrl1);
        lis3dh_read_register(dev, LIS3DH_CTRL_REG4, &ctrl4);

        ctrl1 |= LIS3DH_CTRL_REG1_LPEN;
        ctrl4 &= ~LIS3DH_CTRL_REG4_HR;

        lis3dh_write_register(dev, LIS3DH_CTRL_REG1, ctrl1);
        lis3dh_write_register(dev, LIS3DH_CTRL_REG4, ctrl4);
        break;

      case LIS3DH_POWER_NORMAL:
        lis3dh_read_register(dev, LIS3DH_CTRL_REG1, &ctrl1);
        lis3dh_read_register(dev, LIS3DH_CTRL_REG4, &ctrl4);

        ctrl1 &= ~LIS3DH_CTRL_REG1_LPEN;
        ctrl4 &= ~LIS3DH_CTRL_REG4_HR;

        lis3dh_write_register(dev, LIS3DH_CTRL_REG1, ctrl1);
        lis3dh_write_register(dev, LIS3DH_CTRL_REG4, ctrl4);
        break;

      case LIS3DH_POWER_HIGH:
        lis3dh_read_register(dev, LIS3DH_CTRL_REG1, &ctrl1);
        lis3dh_read_register(dev, LIS3DH_CTRL_REG4, &ctrl4);

        ctrl1 &= ~LIS3DH_CTRL_REG1_LPEN;
        ctrl4 |= LIS3DH_CTRL_REG4_HR;

        lis3dh_write_register(dev, LIS3DH_CTRL_REG1, ctrl1);
        lis3dh_write_register(dev, LIS3DH_CTRL_REG4, ctrl4);
        break;

      default:
        return -EINVAL;
        break;
    }

  dev->power_mode = power_mode;
  sninfo("Power mode set to %d\n", power_mode);

  return OK;
}

/****************************************************************************
 * Name: lis3dh_set_odr
 *
 * Description:
 *   Sets the output data rate of the sensor.
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *   odr - LIS3DH_ODR_*
 *
 * Returned Value:
 *   OK - Power mode was set successfully
 *   -EINVAL - Invalid odr argument
 *
 ****************************************************************************/

static int lis3dh_set_odr(FAR struct lis3dh_dev_s *dev, uint8_t odr)
{
  uint8_t ctrl1;

  if (odr > LIS3DH_CTRL_REG1_ODR_LP_5376HZ)
    {
      return -EINVAL;
    }

  lis3dh_read_register(dev, LIS3DH_CTRL_REG1, &ctrl1);
  ctrl1 |= LIS3DH_CTRL_REG1_ODR(odr) & LIS3DH_CTRL_REG1_ODR_MASK;
  lis3dh_write_register(dev, LIS3DH_CTRL_REG1, ctrl1);

  /* Cache the current ODR in the device structure */

  dev->odr = odr;
  sninfo("Output data rate set to %d\n", odr);

  return OK;
}

/****************************************************************************
 * Name: lis3dh_irq_enable
 *
 * Description:
 *   Enable or disable sensor interrupt generation on FIFO watermark.
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *   enable - Boolean to enable or disable the interrupt generation
 *
 * Returned Value:
 *   OK - Interrupt generation register written.
 *
 ****************************************************************************/

static int lis3dh_irq_enable(FAR struct lis3dh_dev_s *dev, bool enable)
{
  uint8_t reg;

  reg = 0;

  if (enable == true)
    {
      reg = LIS3DH_CTRL_REG3_I1_WTM;
    }

  lis3dh_write_register(dev, LIS3DH_CTRL_REG3, reg);
  return OK;
}

/****************************************************************************
 * Name: lis3dh_irq_enable
 *
 * Description:
 *   Enable or disable sensor interrupt generation on FIFO watermark.
 *
 * Input Parameters:
 *   dev - Pointer to device driver instance
 *   enable - Boolean to enable or disable the interrupt generation
 *
 * Returned Value:
 *   OK - Interrupt generation register written.
 *
 ****************************************************************************/

static int lis3dh_fifo_enable(FAR struct lis3dh_dev_s *dev)
{
  uint8_t reg;
  lis3dh_write_register(dev, LIS3DH_FIFO_CTRL_REG,
                        LIS3DH_FIFO_CTRL_REG_MODE_STREAM | 28);

  lis3dh_read_register(dev, LIS3DH_CTRL_REG5, &reg);
  reg |= LIS3DH_CTRL_REG5_FIFO_EN;
  lis3dh_write_register(dev, LIS3DH_CTRL_REG5, reg);

  return OK;
}

/****************************************************************************
 * Name: lis3dh_enable
 *
 * Description:
 *   Enable sensor measurements into the on-chip FIFO.
 *
 * Input Parameters:
 *   priv - Pointer to device driver instance
 *
 * Returned Value:
 *   OK - Sensor measurements enabled.
 *
 ****************************************************************************/

static int lis3dh_enable(FAR struct lis3dh_dev_s *priv)
{
  uint8_t reg;

  /* Select the default power mode */

  lis3dh_set_power_mode(priv, LIS3DH_POWER_NORMAL);

  /* Select the default output data rate */

  lis3dh_set_odr(priv, LIS3DH_ODR_1344HZ);

  reg = LIS3DH_CTRL_REG4_BDU | LIS3DH_CTRL_REG4_FS_2G;
  lis3dh_write_register(priv, LIS3DH_CTRL_REG4, reg);

  lis3dh_fifo_enable(priv);

  /* Enable X, Y, and Z axes */

  lis3dh_read_register(priv, LIS3DH_CTRL_REG1, &reg);
  reg |= LIS3DH_CTRL_REG1_ZEN | LIS3DH_CTRL_REG1_YEN | LIS3DH_CTRL_REG1_XEN;
  lis3dh_write_register(priv, LIS3DH_CTRL_REG1, reg);

  return OK;
}

/****************************************************************************
 * Name: lis3dh_open
 *
 * Description:
 *   Character device open call.
 *
 * Input Parameters:
 *   filep - Pointer to struct file
 *
 * Returned Value:
 *   -ENODEV - Device was not identified on the SPI bus.
 *   OK      - Sensor device was opened successfully.
 *
 ****************************************************************************/

static int lis3dh_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3dh_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  lis3dh_reset(priv);
  if (lis3dh_ident(priv) < 0)
    {
      snerr("ERROR: Failed to identify LIS3DH on SPI bus\n");
      return -ENODEV;
    }

  /* Attach the interrupt line */

  (priv->config->irq_attach)(priv->config, lis3dh_interrupt_handler, priv);

  /* Enable interrupt generation on the sensor */

  lis3dh_irq_enable(priv, true);

  /* Enable measurements on the sensor */

  if (lis3dh_enable(priv) < 0)
    {
      snerr("ERROR: Failed to enable LIS3DH\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lis3dh_close
 *
 * Description:
 *   Character device close call.
 *
 * Input Parameters:
 *   filep - Pointer to struct file
 *
 * Returned Value:
 *   OK - Sensor device was closed successfully.
 *
 ****************************************************************************/

static int lis3dh_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3dh_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  lis3dh_reset(priv);

  /* Detach the interrupt line */

  (priv->config->irq_detach)(priv->config);

  return OK;
}

/****************************************************************************
 * Name: lis3dh_read
 *
 * Description:
 *   Character device read call. Blocks until requested buffer size is full.
 *
 * Input Parameters:
 *   filep - Pointer to struct file
 *   buffer - Pointer to user buffer
 *   buflen - Size of user buffer in bytes
 *
 * Returned Value:
 *   Returns the number of bytes written to the buffer.
 *   -EINVAL - Supplied buffer length invalid
 *
 ****************************************************************************/

static ssize_t lis3dh_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3dh_dev_s *priv = inode->i_private;
  FAR struct lis3dh_sensor_data_s *data;
  int count;
  int remain;

  DEBUGASSERT(priv != NULL);

  if ((buflen % sizeof(FAR struct lis3dh_sensor_data_s)) != 0)
    {
      snerr("ERROR: Provided buffer must be multiple of sensor data size\n");
      return -EINVAL;
    }

  /* Get the number of samples the user has requested */

  count = buflen / sizeof(struct lis3dh_sensor_data_s);

  /* Cast a pointer into the user buffer */

  data = (FAR struct lis3dh_sensor_data_s *)buffer;

  for (remain = count; remain > 0; remain--)
    {
      /* Wait for data to be available in the queue */

      nxsem_wait(&priv->readsem);

      /* Pop a sample off of the queue */

      lis3dh_queue_pop(priv, data);

      data++;
    }

  return count * sizeof(struct lis3dh_sensor_data_s);
}

/****************************************************************************
 * Name: lis3dh_write
 *
 * Description:
 *   Character device write call. Not supported.
 *
 * Input Parameters:
 *   filep  - Pointer to struct file
 *   buffer - Pointer to user buffer
 *   buflen - Size of user buffer in bytes
 *
 * Returned Value:
 *   -ENOSYS
 *
 ****************************************************************************/

static ssize_t lis3dh_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lis3dh_ioctl
 *
 * Description:
 *   Character device ioctl call. Sets device parameters.
 *
 * Input Parameters:
 *   filep - Pointer to struct file
 *   cmd   - SNIOC_*
 *   arg   - ioctl specific argument
 *
 * Returned Value:
 *   OK - The command was executed successfully.
 *   -ENOTTY - The request command is not applicable to the driver.
 *
 ****************************************************************************/

static int lis3dh_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3dh_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case SNIOC_SET_POWER_MODE:
        ret = lis3dh_set_power_mode(priv, arg);
        break;

      case SNIOC_SET_DATA_RATE:
        ret = lis3dh_set_odr(priv, arg);
        break;

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3dh_register
 *
 * Description:
 *   Register the LIS3DH character device at the specified device path
 *
 * Input Parameters:
 *   devpath - Full path of device node to register ie "/dev/accel0"
 *   spi     - SPI bus device instance
 *   config  - Driver instance configuration structure
 *
 * Returned Value:
 *   OK on success or a negative errno value on failure.
 *
 ****************************************************************************/

int lis3dh_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
  FAR struct lis3dh_config_s *config)
{
  FAR struct lis3dh_dev_s *priv;
  int ret;

  DEBUGASSERT(spi != NULL);

  /* Initialize the LIS3DH device structure */

  priv = (FAR struct lis3dh_dev_s *)kmm_malloc(sizeof(struct lis3dh_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;
  priv->spi = spi;
  priv->work.worker = NULL;
  priv->queue_count = 0;

  /* Initialize queue mutex */

  nxmutex_init(&priv->queuelock);

  /* Initialize read notification semaphore */

  nxsem_init(&priv->readsem, 0, 0);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(spi, LIS3DH_SPI_FREQUENCY);
  SPI_SETMODE(spi, LIS3DH_SPI_MODE);

  /* Register the character driver */

  ret = register_driver(devpath, &g_lis3dh_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      nxmutex_destroy(&priv->queuelock);
      nxsem_destroy(&priv->readsem);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_LIS3DH */
