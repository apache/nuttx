/****************************************************************************
 * drivers/sensors/mlx90393.c
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
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/mlx90393.h>
#include <nuttx/random.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MLX90393)

/****************************************************************************
 * Private
 ****************************************************************************/

struct mlx90393_sensor_data_s
{
  int16_t x_mag;              /* Measurement result for x axis */
  int16_t y_mag;              /* Measurement result for y axis */
  int16_t z_mag;              /* Measurement result for z axis */
  uint16_t temperature;       /* Measurement result for temperature sensor */
};

struct mlx90393_dev_s
{
  FAR struct mlx90393_dev_s *flink;     /* Supports a singly linked list
                                         * of drivers */
  FAR struct spi_dev_s *spi;            /* Pointer to the SPI instance */
  FAR struct mlx90393_config_s *config; /* Pointer to the configuration of
                                         * the MLX90393 sensor */

  mutex_t datalock;                     /* Manages exclusive access to
                                         * this structure */
  struct mlx90393_sensor_data_s data;   /* The data as measured by the
                                         * sensor */
  struct work_s work;                   /* The work queue is responsible
                                         * for retrieving the data from
                                         * the sensor after the arrival of
                                         * new data was signalled in an
                                         * interrupt */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mlx90393_start_burst_mode(FAR struct mlx90393_dev_s *dev);
static void mlx90393_read_measurement_data(FAR struct mlx90393_dev_s *dev);
static void mlx90393_read_register(FAR struct mlx90393_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint16_t *reg_data);
static void mlx90393_write_register(FAR struct mlx90393_dev_s *dev,
                                    uint8_t const reg_addr,
                                    uint16_t const reg_data);
static void mlx90393_reset(FAR struct mlx90393_dev_s *dev);
static int mlx90393_interrupt_handler(int irq, FAR void *context);
static void mlx90393_worker(FAR void *arg);

static int mlx90393_open(FAR struct file *filep);
static int mlx90393_close(FAR struct file *filep);
static ssize_t mlx90393_read(FAR struct file *, FAR char *, size_t);
static ssize_t mlx90393_write(FAR struct file *filep,
                              FAR const char *buffer,
                              size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mlx90393_fops =
{
  mlx90393_open,   /* open */
  mlx90393_close,  /* close */
  mlx90393_read,   /* read */
  mlx90393_write,  /* write */
};

/* Single linked list to store instances of drivers */

static struct mlx90393_dev_s *g_mlx90393_list = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mlx90393_start_burst_mode
 ****************************************************************************/

static void mlx90393_start_burst_mode(FAR struct mlx90393_dev_s *dev)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Start Burst Mode (Continuous Measurement on all channels) */

  SPI_SEND(dev->spi, MLX90393_SB | MLX90393_ZYXT_BM);

  /* Write an idle byte to retrieve the status byte */

  SPI_SEND(dev->spi, 0);

  /* Set CS to high which deselects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: mlx90393_read_measurement_data
 ****************************************************************************/

static void mlx90393_read_measurement_data(FAR struct mlx90393_dev_s *dev)
{
  int ret;

  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Issue command to read measurement data on all channels */

  SPI_SEND(dev->spi, MLX90393_RM | MLX90393_ZYXT_BM);

  /* Write an idle byte to retrieve the status byte */

  SPI_SEND(dev->spi, 0);

  /* The data is output in the following order: T (MSB), T (LSB), X (MSB), X
   * (LSB), Y (MSB), Y (LSB), Z (MSB), Z (LSB)
   */

  uint16_t temperature = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8); /* MSB */
  temperature |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);         /* LSB */

  uint16_t x_mag = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);       /* MSB */
  x_mag |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);               /* LSB */

  uint16_t y_mag = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);       /* MSB */
  y_mag |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);               /* LSB */

  uint16_t z_mag = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);       /* MSB */
  z_mag |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);               /* LSB */

  /* Set CS to high which deselects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  /* Acquire the mutex before the data is copied */

  ret = nxmutex_lock(&dev->datalock);
  if (ret != OK)
    {
      snerr("ERROR: Could not acquire dev->datalock: %d\n", ret);
      return;
    }

  /* Copy retrieve data to internal data structure */

  dev->data.temperature = temperature;
  dev->data.x_mag = (int16_t) (x_mag);
  dev->data.y_mag = (int16_t) (y_mag);
  dev->data.z_mag = (int16_t) (z_mag);

  /* Give back the mutex */

  nxmutex_unlock(&dev->datalock);

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((x_mag << 17) ^ (y_mag << 9) ^ (z_mag << 1) ^
                        temperature);
}

/****************************************************************************
 * Name: mlx90393_start_burst_mode
 ****************************************************************************/

static void mlx90393_reset(FAR struct mlx90393_dev_s *dev)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Issue reset command */

  SPI_SEND(dev->spi, MLX90393_RT);

  /* Write an idle byte to retrieve the status byte */

  SPI_SEND(dev->spi, 0);

  /* Set CS to high which deselects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  /* Wait a little so the device has time to perform a proper reset */

  up_mdelay(100);
}

/****************************************************************************
 * Name: mlx90393_read_register
 ****************************************************************************/

static void mlx90393_read_register(FAR struct mlx90393_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint16_t *reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Issue a read register command */

  SPI_SEND(dev->spi, MLX90393_RR);

  /* Send the register address which needs to be left shifted by 2 */

  SPI_SEND(dev->spi, (reg_addr << 2));

  /* Write an idle byte to retrieve the status byte */

  SPI_SEND(dev->spi, 0);

  *reg_data = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);        /* MSB */
  *reg_data |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);       /* LSB */

  /* Set CS to high which deselects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: mlx90393_write_register
 ****************************************************************************/

static void mlx90393_write_register(FAR struct mlx90393_dev_s *dev,
                                    uint8_t const reg_addr,
                                    uint16_t const reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Issue a write register command */

  SPI_SEND(dev->spi, MLX90393_WR);

  /* Send the data high byte of the register */

  SPI_SEND(dev->spi, (uint8_t) ((reg_data & 0xff00) >> 8));

  /* Send the data low byte of the register */

  SPI_SEND(dev->spi, (uint8_t) (reg_data & 0x00ff));

  /* Send the register address which needs to be left shifted by 2 */

  SPI_SEND(dev->spi, (uint8_t) (reg_addr << 2));

  /* Write an idle byte to retrieve the status byte */

  SPI_SEND(dev->spi, 0);

  /* Set CS to high which deselects the MLX90393 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: mlx90393_interrupt_handler
 ****************************************************************************/

static int mlx90393_interrupt_handler(int irq, FAR void *context)
{
  /* This function should be called upon a rising edge on the MLX90393 INT
   * pin since it signals that new data has been measured. (INT = DRDY).
   */

  FAR struct mlx90393_dev_s *priv = 0;
  int ret;

  /* Find out which MLX90396 device caused the interrupt */

  for (priv = g_mlx90393_list; priv && priv->config->irq != irq;
       priv = priv->flink);
  DEBUGASSERT(priv != NULL);

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long. Also we cannot lock
   * the SPI bus from within an interrupt.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, mlx90393_worker, priv, 0);
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
 * Name: mlx90393_worker
 ****************************************************************************/

static void mlx90393_worker(FAR void *arg)
{
  FAR struct mlx90393_dev_s *priv = (FAR struct mlx90393_dev_s *)(arg);
  DEBUGASSERT(priv != NULL);

  /* Read out the latest sensor data */

  mlx90393_read_measurement_data(priv);
}

/****************************************************************************
 * Name: mlx90393_open
 ****************************************************************************/

static int mlx90393_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mlx90393_dev_s *priv = inode->i_private;
  static int const NUM_REGS = 10;
  int reg_addr;

  DEBUGASSERT(priv != NULL);

  /* Reset the device */

  mlx90393_reset(priv);

#ifdef CONFIG_DEBUG_SENSORS_INFO
  /* Read the content of ALL registers for debug purposes */

  for (reg_addr = 0; reg_addr < NUM_REGS; reg_addr++)
    {
      uint16_t reg_content = 0;
      mlx90393_read_register(priv, reg_addr, &reg_content);
      sninfo("R%d = %x\n", reg_addr, reg_content);
    }
#endif

  /* Start the burst mode */

  mlx90393_start_burst_mode(priv);

  return OK;
}

/****************************************************************************
 * Name: mlx90393_close
 ****************************************************************************/

static int mlx90393_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mlx90393_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Reset the device */

  mlx90393_reset(priv);

  return OK;
}

/****************************************************************************
 * Name: mlx90393_read
 ****************************************************************************/

static ssize_t mlx90393_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mlx90393_dev_s *priv = inode->i_private;
  FAR struct mlx90393_sensor_data_s *data;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Check if enough memory was provided for the read call */

  if (buflen < sizeof(struct mlx90393_sensor_data_s))
    {
      snerr("ERROR: "
            "Not enough memory for reading out a sensor data sample\n");
      return -ENOSYS;
    }

  /* Copy the sensor data into the buffer */

  /* Acquire the mutex before the data is copied */

  ret = nxmutex_lock(&priv->datalock);
  if (ret < 0)
    {
      snerr("ERROR: Could not acquire priv->datalock: %d\n", ret);
      return ret;
    }

  data = (FAR struct mlx90393_sensor_data_s *)buffer;
  memset(data, 0, sizeof(struct mlx90393_sensor_data_s));

  data->x_mag = priv->data.x_mag;
  data->y_mag = priv->data.y_mag;
  data->z_mag = priv->data.z_mag;
  data->temperature = priv->data.temperature;

  /* Give back the mutex */

  nxmutex_unlock(&priv->datalock);

  return sizeof(struct mlx90393_sensor_data_s);
}

/****************************************************************************
 * Name: mlx90393_write
 ****************************************************************************/

static ssize_t mlx90393_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mlx90393_register
 *
 * Description:
 *   Register the MLX90393 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MLX90393
 *   config  - Describes the configuration of the MLX90393 part.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mlx90393_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                      FAR struct mlx90393_config_s *config)
{
  FAR struct mlx90393_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the MLX90393 device structure */

  priv = (FAR struct mlx90393_dev_s *)
                      kmm_malloc(sizeof(struct mlx90393_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi = spi;
  priv->config = config;

  priv->work.worker = NULL;

  nxmutex_init(&priv->datalock); /* Initialize sensor data access mutex */

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(spi, MLX90393_SPI_FREQUENCY);
  SPI_SETMODE(spi, MLX90393_SPI_MODE);

  /* Attach the interrupt handler */

  ret = priv->config->attach(priv->config, &mlx90393_interrupt_handler);
  if (ret < 0)
    {
      snerr("ERROR: Failed to attach interrupt\n");
      nxmutex_destroy(&priv->datalock);
      kmm_free(priv);
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_mlx90393_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->datalock);
      kmm_free(priv);
      return ret;
    }

  /* Since we support multiple MLX90393 devices are supported, we will need
   * to add this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the received IRQ number.
   */

  priv->flink = g_mlx90393_list;
  g_mlx90393_list = priv;

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_MLX90393 */
