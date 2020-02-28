/****************************************************************************
 * drivers/sensors/lis3mdl.c
 * Character driver for the LIS3MDL 3-Axis magnetometer.
 *
 *   Copyright (C) 2016 DS-Automotion GmbH. All rights reserved.
 *   Author: Alexander Entinger <a.entinger@ds-automotion.com>
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

#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/lis3mdl.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_LIS3MDL)

/****************************************************************************
 * Private
 ****************************************************************************/

struct lis3mdl_sensor_data_s
{
  int16_t x_mag;              /* Measurement result for x axis */
  int16_t y_mag;              /* Measurement result for y axis */
  int16_t z_mag;              /* Measurement result for z axis */
  int16_t temperature;        /* Measurement result for temperature sensor */
};

struct lis3mdl_dev_s
{
  FAR struct lis3mdl_dev_s *flink;     /* Supports a singly linked list of
                                        * drivers */
  FAR struct spi_dev_s *spi;           /* Pointer to the SPI instance */
  FAR struct lis3mdl_config_s *config; /* Pointer to the configuration
                                        * of the LIS3MDL sensor */
  sem_t datasem;                       /* Manages exclusive access to this
                                        * structure */
  struct lis3mdl_sensor_data_s data;   /* The data as measured by the sensor */
  struct work_s work;                  /* The work queue is responsible for
                                        * retrieving the data from the
                                        * sensor after the arrival of new
                                        * data was signalled in an interrupt */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void lis3mdl_read_register(FAR struct lis3mdl_dev_s *dev,
                                  uint8_t const reg_addr, uint8_t * reg_data);
static void lis3mdl_write_register(FAR struct lis3mdl_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint8_t const reg_data);
static void lis3mdl_reset(FAR struct lis3mdl_dev_s *dev);
static void lis3mdl_read_measurement_data(FAR struct lis3mdl_dev_s *dev);
static void lis3mdl_read_magnetic_data(FAR struct lis3mdl_dev_s *dev,
                                       uint16_t * x_mag, uint16_t * y_mag,
                                       uint16_t * z_mag);
static void lis3mdl_read_temperature(FAR struct lis3mdl_dev_s *dev,
                                     uint16_t * temperature);
static int lis3mdl_interrupt_handler(int irq, FAR void *context);
static void lis3mdl_worker(FAR void *arg);

static int lis3mdl_open(FAR struct file *filep);
static int lis3mdl_close(FAR struct file *filep);
static ssize_t lis3mdl_read(FAR struct file *, FAR char *, size_t);
static ssize_t lis3mdl_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int lis3mdl_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lis3mdl_fops =
{
  lis3mdl_open,
  lis3mdl_close,
  lis3mdl_read,
  lis3mdl_write,
  NULL,
  lis3mdl_ioctl,
  NULL
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/* Single linked list to store instances of drivers */

static struct lis3mdl_dev_s *g_lis3mdl_list = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3mdl_read_register
 ****************************************************************************/

static void lis3mdl_read_register(FAR struct lis3mdl_dev_s *dev,
                                  uint8_t const reg_addr, uint8_t * reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read - the MSB needs
   * to be set to indicate the read indication.
   */

  SPI_SEND(dev->spi, reg_addr | 0x80);

  /* Write an idle byte while receiving the required data */

  *reg_data = (uint8_t) (SPI_SEND(dev->spi, 0));

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_write_register
 ****************************************************************************/

static void lis3mdl_write_register(FAR struct lis3mdl_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint8_t const reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read */

  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written in the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_reset
 ****************************************************************************/

static void lis3mdl_reset(FAR struct lis3mdl_dev_s *dev)
{
  lis3mdl_write_register(dev,
                         LIS3MDL_CTRL_REG_2, LIS3MDL_CTRL_REG_2_SOFT_RST_bm);

  up_mdelay(100);
}

/****************************************************************************
 * Name: lis3mdl_interrupt_handler
 ****************************************************************************/

static void lis3mdl_read_measurement_data(FAR struct lis3mdl_dev_s *dev)
{
  /* Magnetic data */

  uint16_t x_mag = 0;
  uint16_t y_mag = 0;
  uint16_t z_mag = 0;

  lis3mdl_read_magnetic_data(dev, &x_mag, &y_mag, &z_mag);

  /* Temperature */

  uint16_t temperature = 0;

  lis3mdl_read_temperature(dev, &temperature);

  /* Acquire the semaphore before the data is copied */

  int ret = nxsem_wait(&dev->datasem);
  if (ret != OK)
    {
      snerr("ERROR: Could not acquire dev->datasem: %d\n", ret);
      return;
    }

  /* Copy retrieve data to internal data structure */

  dev->data.x_mag = (int16_t) (x_mag);
  dev->data.y_mag = (int16_t) (y_mag);
  dev->data.z_mag = (int16_t) (z_mag);
  dev->data.temperature = (int16_t) (temperature);

  /* Give back the semaphore */

  nxsem_post(&dev->datasem);

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((x_mag << 16) ^ (y_mag << 10) ^ (z_mag << 2) ^
                        temperature);
}

/****************************************************************************
 * Name: lis3mdl_read_magnetic_data
 ****************************************************************************/

static void lis3mdl_read_magnetic_data(FAR struct lis3mdl_dev_s *dev,
                                       uint16_t * x_mag, uint16_t * y_mag,
                                       uint16_t * z_mag)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading 0x80 ->
   * MSB is set -> Read Indication 0x40 -> MSB-1 (MS-Bit) is set -> auto
   * increment of address when reading multiple bytes.
   */

  SPI_SEND(dev->spi, (LIS3MDL_OUT_X_L_REG | 0x80 | 0x40)); /* RX */
  *x_mag  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);     /* LSB */
  *x_mag |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);     /* MSB */

  *y_mag  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);     /* LSB */
  *y_mag |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);     /* MSB */

  *z_mag  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);     /* LSB */
  *z_mag |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);     /* MSB */

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_read_temperature
 ****************************************************************************/

static void lis3mdl_read_temperature(FAR struct lis3mdl_dev_s *dev,
                                     uint16_t * temperature)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading 0x80 ->
   * MSB is set -> Read Indication 0x40 -> MSB-1 (MS-Bit) is set -> auto
   * increment of address when reading multiple bytes.
   */

  SPI_SEND(dev->spi, (LIS3MDL_TEMP_OUT_L_REG | 0x80 | 0x40));

  /* RX */

  *temperature  = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);    /* LSB */
  *temperature |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);    /* MSB */

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_interrupt_handler
 ****************************************************************************/

static int lis3mdl_interrupt_handler(int irq, FAR void *context)
{
  /* This function should be called upon a rising edge on the LIS3MDL DRDY pin
   * since it signals that new data has been measured.
   */

  FAR struct lis3mdl_dev_s *priv = 0;
  int ret;

  /* Find out which LIS3MDL device caused the interrupt */

  for (priv = g_lis3mdl_list; priv && priv->config->irq != irq;
       priv = priv->flink);
  DEBUGASSERT(priv != NULL);

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long. Also we cannot lock the
   * SPI bus from within an interrupt.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, lis3mdl_worker, priv, 0);
  if (ret < 0)
    {
      snerr("ERROR: Failed to queue work: %d\n", ret);
      return ret;
    }
  else
    {
      return OK;
    }
}

/****************************************************************************
 * Name: lis3mdl_worker
 ****************************************************************************/

static void lis3mdl_worker(FAR void *arg)
{
  FAR struct lis3mdl_dev_s *priv = (FAR struct lis3mdl_dev_s *)(arg);
  DEBUGASSERT(priv != NULL);

  /* Read out the latest sensor data */

  lis3mdl_read_measurement_data(priv);
}

/****************************************************************************
 * Name: lis3mdl_open
 ****************************************************************************/

static int lis3mdl_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3mdl_dev_s *priv = inode->i_private;
  uint8_t reg_content;
  uint8_t reg_addr;

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  lis3mdl_reset(priv);

  /* Enable * - the maximum full scale mode. * Full scale = +/- 1.6 mT (16
   * Gauss).
   */

  lis3mdl_write_register(priv,
                         LIS3MDL_CTRL_REG_2,
                         LIS3MDL_CTRL_REG_2_FS_1_bm |
                         LIS3MDL_CTRL_REG_2_FS_0_bm);

  /* Enable - temperature sensor - ultra high performance mode (UMP) for X and
   * Y - fast output data rates This results in a output data rate of 155 Hz
   * for X and Y.
   */

  lis3mdl_write_register(priv,
                         LIS3MDL_CTRL_REG_1,
                         LIS3MDL_CTRL_REG_1_TEMP_EN_bm |
                         LIS3MDL_CTRL_REG_1_OM_1_bm | LIS3MDL_CTRL_REG_1_OM_0_bm
                         | LIS3MDL_CTRL_REG_1_FAST_ODR_bm);

  /* Enable * - ultra high performance mode (UMP) for Z * This should result to
   * the same output data rate as for X and Y.
   */

  lis3mdl_write_register(priv,
                         LIS3MDL_CTRL_REG_4,
                         LIS3MDL_CTRL_REG_4_OMZ_1_bm |
                         LIS3MDL_CTRL_REG_4_OMZ_0_bm);

  /* Enable * - block data update for magnetic sensor data * This should
   * prevent race conditions when reading sensor data.
   */

  lis3mdl_write_register(priv, LIS3MDL_CTRL_REG_5, LIS3MDL_CTRL_REG_5_BDU_bm);

  /* Enable continuous conversion mode - the device starts measuring now. */

  lis3mdl_write_register(priv, LIS3MDL_CTRL_REG_3, 0);

  /* Read measurement data to ensure DRDY is low */

  lis3mdl_read_measurement_data(priv);

  /* Read back the content of all control registers for debug purposes */

  reg_content = 0;
  for (reg_addr = LIS3MDL_CTRL_REG_1;
       reg_addr <= LIS3MDL_CTRL_REG_5;
       reg_addr++)
    {
      lis3mdl_read_register(priv, reg_addr, &reg_content);
      sninfo("R#%04x = %04x\n", reg_addr, reg_content);
    }

  lis3mdl_read_register(priv, LIS3MDL_STATUS_REG, &reg_content);
  sninfo("STATUS_REG = %04x\n", reg_content);

  return OK;
}

/****************************************************************************
 * Name: lis3mdl_close
 ****************************************************************************/

static int lis3mdl_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3mdl_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  lis3mdl_reset(priv);

  return OK;
}

/****************************************************************************
 * Name: lis3mdl_read
 ****************************************************************************/

static ssize_t lis3mdl_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3mdl_dev_s *priv = inode->i_private;
  FAR struct lis3mdl_sensor_data_s *data;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Check if enough memory was provided for the read call */

  if (buflen < sizeof(FAR struct lis3mdl_sensor_data_s))
    {
      snerr("ERROR: Not enough memory for reading out a sensor data sample\n");
      return -ENOSYS;
    }

  /* Acquire the semaphore before the data is copied */

  ret = nxsem_wait(&priv->datasem);
  if (ret < 0)
    {
      snerr("ERROR: Could not acquire priv->datasem: %d\n", ret);
      return ret;
    }

  /* Copy the sensor data into the buffer */

  data = (FAR struct lis3mdl_sensor_data_s *)buffer;
  memset(data, 0, sizeof(FAR struct lis3mdl_sensor_data_s));

  data->x_mag = priv->data.x_mag;
  data->y_mag = priv->data.y_mag;
  data->z_mag = priv->data.z_mag;
  data->temperature = priv->data.temperature;

  /* Give back the semaphore */

  nxsem_post(&priv->datasem);

  return sizeof(FAR struct lis3mdl_sensor_data_s);
}

/****************************************************************************
 * Name: lis3mdl_write
 ****************************************************************************/

static ssize_t lis3mdl_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lis3mdl_ioctl
 ****************************************************************************/

static int lis3mdl_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* Command was not recognized */

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
 * Name: lis3mdl_register
 *
 * Description:
 *   Register the LIS3MDL character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             LIS3MDL
 *   config  - configuration for the LIS3MDL driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis3mdl_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     FAR struct lis3mdl_config_s const *config)
{
  FAR struct lis3mdl_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the LIS3MDL device structure */

  priv = (FAR struct lis3mdl_dev_s *)kmm_malloc(sizeof(struct lis3mdl_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi = spi;
  priv->config = config;

  priv->work.worker = NULL;

  nxsem_init(&priv->datasem, 0, 1);     /* Initialize sensor data access
                                         * semaphore */

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(spi, LIS3MDL_SPI_FREQUENCY);
  SPI_SETMODE(spi, LIS3MDL_SPI_MODE);

  /* Attach the interrupt handler */

  ret = priv->config->attach(priv->config, &lis3mdl_interrupt_handler);
  if (ret < 0)
    {
      snerr("ERROR: Failed to attach interrupt\n");
      return -ENODEV;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_lis3mdl_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      nxsem_destroy(&priv->datasem);
      return -ENODEV;
    }

  /* Since we support multiple LIS3MDL devices are supported, we will need to
   * add this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the received IRQ number.
   */

  priv->flink    = g_lis3mdl_list;
  g_lis3mdl_list = priv;

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_LIS3MDL */
