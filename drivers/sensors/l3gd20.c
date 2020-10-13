/****************************************************************************
 * drivers/sensors/l3gd20.c
 * Character driver for the ST L3GD20 3-Axis gyroscope.
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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>

#include <nuttx/fs/fs.h>

#include <nuttx/sensors/l3gd20.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_L3GD20)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SCHED_HPWORK)
#  error Hi-priority work queue support is required (CONFIG_SCHED_HPWORK)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct l3gd20_dev_s
{
  FAR struct l3gd20_dev_s *flink;     /* Supports a singly linked list of
                                       * drivers */
  FAR struct spi_dev_s *spi;          /* Pointer to the SPI instance */
  FAR struct l3gd20_config_s *config; /* Pointer to the configuration of the
                                       * L3GD20 sensor */
  sem_t datasem;                      /* Manages exclusive access to this
                                       * structure */
  struct l3gd20_sensor_data_s data;   /* The data as measured by the sensor */
  struct work_s work;                 /* The work queue is responsible for
                                       * retrieving the data from the sensor
                                       * after the arrival of new data was
                                       * signalled in an interrupt */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SENSORS_INFO
static void l3gd20_read_register(FAR struct l3gd20_dev_s *dev,
                                 uint8_t const reg_addr, uint8_t *reg_data);
#endif
static void l3gd20_write_register(FAR struct l3gd20_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t const reg_data);
static void l3gd20_reset(FAR struct l3gd20_dev_s *dev);
static void l3gd20_read_measurement_data(FAR struct l3gd20_dev_s *dev);
static void l3gd20_read_gyroscope_data(FAR struct l3gd20_dev_s *dev,
                                       uint16_t *x_gyr, uint16_t *y_gyr,
                                       uint16_t *z_gyr);
static void l3gd20_read_temperature(FAR struct l3gd20_dev_s *dev,
                                    uint8_t * temperature);
static int l3gd20_interrupt_handler(int irq, FAR void *context,
                                    FAR void *arg);
static void l3gd20_worker(FAR void *arg);

static int l3gd20_open(FAR struct file *filep);
static int l3gd20_close(FAR struct file *filep);
static ssize_t l3gd20_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t l3gd20_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int l3gd20_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_l3gd20_fops =
{
  l3gd20_open,
  l3gd20_close,
  l3gd20_read,
  l3gd20_write,
  NULL,
  l3gd20_ioctl,
  NULL
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/* Single linked list to store instances of drivers */

static struct l3gd20_dev_s *g_l3gd20_list = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SENSORS_INFO
/****************************************************************************
 * Name: l3gd20_read_register
 ****************************************************************************/

static void l3gd20_read_register(FAR struct l3gd20_dev_s *dev,
                                 uint8_t const reg_addr, uint8_t *reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the
   * same time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the L3GD20 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read - the MSB needs
   * to be set to indicate the read indication.
   */

  SPI_SEND(dev->spi, reg_addr | 0x80);

  /* Write an idle byte while receiving the required data */

  *reg_data = (uint8_t) (SPI_SEND(dev->spi, 0));

  /* Set CS to high which deselects the L3GD20 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}
#endif

/****************************************************************************
 * Name: l3gd20_write_register
 ****************************************************************************/

static void l3gd20_write_register(FAR struct l3gd20_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t const reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the L3GD20 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read */

  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written in the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high which deselects the L3GD20 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: l3gd20_reset
 ****************************************************************************/

static void l3gd20_reset(FAR struct l3gd20_dev_s *dev)
{
  /* Reboot memory content */

  l3gd20_write_register(dev, L3GD20_CTRL_REG_5, L3GD20_CTRL_REG_5_BOOT_BM);

  up_mdelay(100);
}

/****************************************************************************
 * Name: l3gd20_read_measurement_data
 ****************************************************************************/

static void l3gd20_read_measurement_data(FAR struct l3gd20_dev_s *dev)
{
  uint16_t x_gyr = 0;
  uint16_t y_gyr = 0;
  uint16_t z_gyr = 0;
  uint8_t temperature = 0;
  int ret;

  /* Read Gyroscope */

  l3gd20_read_gyroscope_data(dev, &x_gyr, &y_gyr, &z_gyr);

  /* Read Temperature */

  l3gd20_read_temperature(dev, &temperature);

  /* Acquire the semaphore before the data is copied */

  ret = nxsem_wait(&dev->datasem);
  if (ret < 0)
    {
      snerr("ERROR: Could not acquire dev->datasem: %d\n", ret);
      return;
    }

  /* Copy retrieve data to internal data structure */

  dev->data.x_gyr = (int16_t) (x_gyr);
  dev->data.y_gyr = (int16_t) (y_gyr);
  dev->data.z_gyr = (int16_t) (z_gyr);

  /* Give back the semaphore */

  nxsem_post(&dev->datasem);

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((x_gyr << 16) ^ (y_gyr << 8) ^ (z_gyr << 0));
}

/****************************************************************************
 * Name: l3gd20_read_gyroscope_data
 ****************************************************************************/

static void l3gd20_read_gyroscope_data(FAR struct l3gd20_dev_s *dev,
                                       uint16_t * x_gyr, uint16_t * y_gyr,
                                       uint16_t * z_gyr)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the L3GD20 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set -> Read Indication
   * 0x40 -> MSB-1 (MS-Bit) is set -> auto increment of address when reading
   * multiple bytes.
   */

  SPI_SEND(dev->spi, (L3GD20_OUT_X_L_REG | 0x80 | 0x40)); /* RX */

  *x_gyr = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);   /* LSB */
  *x_gyr |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *y_gyr = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);   /* LSB */
  *y_gyr |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *z_gyr = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);   /* LSB */
  *z_gyr |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  /* Set CS to high which deselects the L3GD20 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: l3gd20_read_temperature
 ****************************************************************************/

static void l3gd20_read_temperature(FAR struct l3gd20_dev_s *dev,
                                    FAR uint8_t *temperature)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the L3GD20 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80  MSB is set -> Read Indication
   */

  SPI_SEND(dev->spi, (L3GD20_OUT_TEMP_REG | 0x80));

  /* RX */

  *temperature  = (SPI_SEND(dev->spi, 0));

  /* Set CS to high which deselects the L3GD20 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: l3gd20_interrupt_handler
 ****************************************************************************/

static int l3gd20_interrupt_handler(int irq, FAR void *context,
                                    FAR void *arg)
{
  /* This function should be called upon a rising edge on the L3GD20 new data
   * interrupt pin since it signals that new data has been measured.
   */

  FAR struct l3gd20_dev_s *priv = 0;
  int ret;

  /* Find out which L3GD20 device caused the interrupt */

  for (priv = g_l3gd20_list;
       priv && priv->config->irq != irq;
       priv = priv->flink)
    {
      DEBUGASSERT(priv != NULL);
    }

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long. Also we cannot lock
   * the SPI bus from within an interrupt.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, l3gd20_worker, priv, 0);
  if (ret < 0)
    {
      snerr("ERROR: Failed to queue work: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: l3gd20_worker
 ****************************************************************************/

static void l3gd20_worker(FAR void *arg)
{
  FAR struct l3gd20_dev_s *priv = (FAR struct l3gd20_dev_s *)(arg);
  DEBUGASSERT(priv != NULL);

  /* Read out the latest sensor data */

  l3gd20_read_measurement_data(priv);
}

/****************************************************************************
 * Name: l3gd20_open
 ****************************************************************************/

static int l3gd20_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct l3gd20_dev_s *priv = inode->i_private;
#ifdef CONFIG_DEBUG_SENSORS_INFO
  uint8_t reg_content;
  uint8_t reg_addr;
#endif

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  l3gd20_reset(priv);

  /* Enable DRDY signal on INT 2 */

  l3gd20_write_register(priv,
                        L3GD20_CTRL_REG_3,
                        L3GD20_CTRL_REG_3_I2_DRDY_BM);

  /* Enable the maximum full scale mode.
   * Enable block data update for gyro sensor data.
   * This should prevent race conditions when reading sensor data.
   */

  l3gd20_write_register(priv,
                        L3GD20_CTRL_REG_4,
                        L3GD20_CTRL_REG_4_BDU_BM |
                        L3GD20_CTRL_REG_4_FS_1_BM |
                        L3GD20_CTRL_REG_4_FS_0_BM);

  /* Enable X,Y,Z axis
   * DR=00 -> Output data rate = 95 Hz, Cut-off = 12.5
   */

  l3gd20_write_register(priv,
                        L3GD20_CTRL_REG_1,
                        L3GD20_CTRL_REG_1_POWERDOWN_BM |
                        L3GD20_CTRL_REG_1_X_EN_BM |
                        L3GD20_CTRL_REG_1_Y_EN_BM |
                        L3GD20_CTRL_REG_1_Z_EN_BM);

  /* Read measurement data to ensure DRDY is low */

  l3gd20_read_measurement_data(priv);

  /* Read back the content of all control registers for debug purposes */

#ifdef CONFIG_DEBUG_SENSORS_INFO
  reg_content = 0;

  l3gd20_read_register(priv, L3GD20_WHO_AM_I, &reg_content);
  sninfo("WHO_AM_I_REG = %04x\n", reg_content);

  for (reg_addr = L3GD20_CTRL_REG_1;
       reg_addr <= L3GD20_CTRL_REG_5;
       reg_addr++)
    {
      l3gd20_read_register(priv, reg_addr, &reg_content);
      sninfo("R#%04x = %04x\n", reg_addr, reg_content);
    }

  l3gd20_read_register(priv, L3GD20_STATUS_REG, &reg_content);
  sninfo("STATUS_REG = %04x\n", reg_content);
#endif

  return OK;
}

/****************************************************************************
 * Name: l3gd20_close
 ****************************************************************************/

static int l3gd20_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct l3gd20_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  l3gd20_reset(priv);
  return OK;
}

/****************************************************************************
 * Name: l3gd20_read
 ****************************************************************************/

static ssize_t l3gd20_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct l3gd20_dev_s *priv = inode->i_private;
  FAR struct l3gd20_sensor_data_s *data;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Check if enough memory was provided for the read call */

  if (buflen < sizeof(FAR struct l3gd20_sensor_data_s))
    {
      snerr("ERROR: Not enough memory for reading out a sensor data"
            " sample\n");
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

  data = (FAR struct l3gd20_sensor_data_s *)buffer;
  memset(data, 0, sizeof(FAR struct l3gd20_sensor_data_s));

  data->x_gyr = priv->data.x_gyr;
  data->y_gyr = priv->data.y_gyr;
  data->z_gyr = priv->data.z_gyr;
  data->temperature = priv->data.temperature;

  /* Give back the semaphore */

  nxsem_post(&priv->datasem);

  return sizeof(FAR struct l3gd20_sensor_data_s);
}

/****************************************************************************
 * Name: l3gd20_write
 ****************************************************************************/

static ssize_t l3gd20_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: l3gd20_ioctl
 ****************************************************************************/

static int l3gd20_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* @TODO */

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
 * Name: l3gd20_register
 *
 * Description:
 *   Register the L3DF20 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/gyr0".
 *   spi     - An SPI driver instance.
 *   config  - configuration for the L3GD20 driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int l3gd20_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                    FAR struct l3gd20_config_s *config)
{
  FAR struct l3gd20_dev_s *priv;
  int ret = OK;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the L3GD20 device structure */

  priv = (FAR struct l3gd20_dev_s *)kmm_malloc(sizeof(struct l3gd20_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      ret = -ENOMEM;
      goto errout;
    }

  priv->spi              = spi;
  priv->config           = config;
  priv->work.worker      = NULL;

  priv->data.x_gyr       = 0;
  priv->data.y_gyr       = 0;
  priv->data.z_gyr       = 0;
  priv->data.temperature = 0;

  /* Initialize sensor data access semaphore */

  nxsem_init(&priv->datasem, 0, 1);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(spi, L3GD20_SPI_FREQUENCY);
  SPI_SETMODE(spi, L3GD20_SPI_MODE);

  /* Attach the interrupt handler */

  ret = priv->config->attach(priv->config, &l3gd20_interrupt_handler);
  if (ret < 0)
    {
      snerr("ERROR: Failed to attach interrupt\n");
      goto errout;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_l3gd20_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      nxsem_destroy(&priv->datasem);
      goto errout;
    }

  /* Since we support multiple L3GD20 devices, we will need to add this new
   * instance to a list of device instances so that it can be found by the
   * interrupt handler based on the received IRQ number.
   */

  priv->flink   = g_l3gd20_list;
  g_l3gd20_list = priv;

errout:
  return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_L3GD20 */
