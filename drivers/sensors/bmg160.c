/****************************************************************************
 * drivers/sensors/bmg160.c
 * Character driver for the BMG160 3-Axis gyroscope.
 *
 *   Copyright (C) 2016 DS-Automotion GmbH. All rights reserved.
 *   Author: Alexander Entinger <a.entinger@ds-automotion.com>
 *           Thomas Ilk
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
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/bmg160.h>
#include <nuttx/random.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_BMG160)

/****************************************************************************
 * Private
 ****************************************************************************/

struct bmg160_sensor_data_s
{
  int16_t x_gyr;              /* Measurement result for x axis */
  int16_t y_gyr;              /* Measurement result for y axis */
  int16_t z_gyr;              /* Measurement result for z axis */
};

struct bmg160_dev_s
{
  FAR struct bmg160_dev_s *flink;     /* Supports a singly linked list of
                                       * drivers */
  FAR struct spi_dev_s *spi;          /* Pointer to the SPI instance */
  FAR struct bmg160_config_s *config; /* Pointer to the configuration of the
                                       * BMG160 sensor */
  sem_t datasem;                      /* Manages exclusive access to this
                                       * structure */
  struct bmg160_sensor_data_s data;   /* The data as measured by the sensor */
  struct work_s work;                 /* The work queue is responsible for
                                       * retrieving the data from the sensor
                                       * after the arrival of new data was
                                       * signalled in an interrupt */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void bmg160_read_register(FAR struct bmg160_dev_s *dev,
                                 uint8_t const reg_addr, uint8_t * reg_data);
static void bmg160_write_register(FAR struct bmg160_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t const reg_data);
static void bmg160_reset(FAR struct bmg160_dev_s *dev);
static void bmg160_read_measurement_data(FAR struct bmg160_dev_s *dev);
static void bmg160_read_gyroscope_data(FAR struct bmg160_dev_s *dev,
                                       uint16_t * x_gyr, uint16_t * y_gyr,
                                       uint16_t * z_gyr);
static int bmg160_interrupt_handler(int irq, FAR void *context);
static void bmg160_worker(FAR void *arg);

static int bmg160_open(FAR struct file *filep);
static int bmg160_close(FAR struct file *filep);
static ssize_t bmg160_read(FAR struct file *, FAR char *, size_t);
static ssize_t bmg160_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int bmg160_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bmg160_fops =
{
  bmg160_open,
  bmg160_close,
  bmg160_read,
  bmg160_write,
  NULL,
  bmg160_ioctl,
  NULL
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/* Single linked list to store instances of drivers */

static struct bmg160_dev_s *g_bmg160_list = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmg160_read_register
 ****************************************************************************/

static void bmg160_read_register(FAR struct bmg160_dev_s *dev,
                                 uint8_t const reg_addr, uint8_t * reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the BMG160 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read - the MSB needs
   * to be set to indicate the read indication.
   */

  SPI_SEND(dev->spi, reg_addr | 0x80);

  /* Write an idle byte while receiving the required data */

  *reg_data = (uint8_t) (SPI_SEND(dev->spi, 0));

  /* Set CS to high which deselects the BMG160 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: bmg160_write_register
 ****************************************************************************/

static void bmg160_write_register(FAR struct bmg160_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t const reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the BMG160 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read */

  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written in the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high which deselects the BMG160 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: bmg160_reset
 ****************************************************************************/

static void bmg160_reset(FAR struct bmg160_dev_s *dev)
{
  bmg160_write_register(dev, BMG160_BGW_SOFTRESET_REG, 0xb6);

  up_mdelay(100);
}

/****************************************************************************
 * Name: bmg160_read_measurement_data
 ****************************************************************************/

static void bmg160_read_measurement_data(FAR struct bmg160_dev_s *dev)
{
  int ret;

  /* Read Gyroscope */

  uint16_t x_gyr = 0;
  uint16_t y_gyr = 0;
  uint16_t z_gyr = 0;

  bmg160_read_gyroscope_data(dev, &x_gyr, &y_gyr, &z_gyr);

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

  add_sensor_randomness((x_gyr << 16) ^ (y_gyr << 8) ^ z_gyr);
}

/****************************************************************************
 * Name: bmg160_read_gyroscope_data
 ****************************************************************************/

static void bmg160_read_gyroscope_data(FAR struct bmg160_dev_s *dev,
                                       uint16_t * x_gyr, uint16_t * y_gyr,
                                       uint16_t * z_gyr)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the BMG160 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading. 0x80
   * -> MSB is set -> Read Indication.
   */

  SPI_SEND(dev->spi, (BMG160_RATE_X_LSB_REG | 0x80));

  /* RX */

  *x_gyr = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);   /* LSB */
  *x_gyr |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *y_gyr = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);   /* LSB */
  *y_gyr |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *z_gyr = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);   /* LSB */
  *z_gyr |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  /* Set CS to high which deselects the BMG160 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: bmg160_interrupt_handler
 ****************************************************************************/

static int bmg160_interrupt_handler(int irq, FAR void *context)
{
  /* This function should be called upon a rising edge on the BMG160 new data
   * interrupt pin since it signals that new data has been measured.
   */

  FAR struct bmg160_dev_s *priv = 0;
  int ret;

  /* Find out which BMG160 device caused the interrupt */

  for (priv = g_bmg160_list; priv && priv->config->irq != irq;
       priv = priv->flink);
  DEBUGASSERT(priv != NULL);

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long. Also we cannot lock the
   * SPI bus from within an interrupt.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, bmg160_worker, priv, 0);
  if (ret < 0)
    {
      snerr("ERROR: Failed to queue work: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bmg160_worker
 ****************************************************************************/

static void bmg160_worker(FAR void *arg)
{
  FAR struct bmg160_dev_s *priv = (FAR struct bmg160_dev_s *)(arg);
  DEBUGASSERT(priv != NULL);

  /* Read out the latest sensor data */

  bmg160_read_measurement_data(priv);
}

/****************************************************************************
 * Name: bmg160_open
 ****************************************************************************/

static int bmg160_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bmg160_dev_s *priv = inode->i_private;
#ifdef CONFIG_DEBUG_SENSORS_INFO
  uint8_t reg_content;
#endif

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  bmg160_reset(priv);

  /* Configure the sensor for our needs */

  /* Enable - the full scale range FS = +/- 250 °/s */

  bmg160_write_register(priv,
                        BMG160_RANGE_REG,
                        BMG160_RANGE_REG_FIX_VAL_bm | BMG160_RANGE_REG_FSR_1_bm |
                        BMG160_RANGE_REG_FSR_0_bm);

  /* Enable - the fastest data output rate ODR = 2000 Hz -> BW = 230 Hz */

  bmg160_write_register(priv, BMG160_BW_REG, BMG160_BW_REG_ODR_0_bm);

  /* Enable - new data interrupt 1 */

  bmg160_write_register(priv,
                        BMG160_INT_EN_0_REG, BMG160_INT_EN_0_REG_DATA_EN_bm);

  /* Enable - active high level interrupt 1 - push-pull interrupt */

  bmg160_write_register(priv,
                        BMG160_INT_EN_1_REG, BMG160_INT_EN_1_REG_INT1_LVL_bm);

  /* Enable - map new data interrupt to INT1 */

  bmg160_write_register(priv,
                        BMG160_INT_MAP_1_REG,
                        BMG160_INT_MAP_1_REG_INT1_DATA_bm);

  /* Read measurement data to ensure DRDY is low */

  bmg160_read_measurement_data(priv);

#ifdef CONFIG_DEBUG_SENSORS_INFO
  /* Read back the content of all control registers for debug purposes */

  reg_content = 0;
  bmg160_read_register(priv, BMG160_RANGE_REG, &reg_content);
  sninfo("BMG160_RANGE_REG = %04x\n", reg_content);

  bmg160_read_register(priv, BMG160_BW_REG, &reg_content);
  sninfo("BMG160_BW_REG = %04x\n", reg_content);

  bmg160_read_register(priv, BMG160_INT_EN_0_REG, &reg_content);
  sninfo("BMG160_INT_EN_0_REG = %04x\n", reg_content);

  bmg160_read_register(priv, BMG160_INT_EN_1_REG, &reg_content);
  sninfo("BMG160_INT_EN_1_REG = %04x\n", reg_content);

  bmg160_read_register(priv, BMG160_INT_MAP_1_REG, &reg_content);
  sninfo("BMG160_INT_MAP_1_REG = %04x\n", reg_content);
#endif

  return OK;
}

/****************************************************************************
 * Name: bmg160_close
 ****************************************************************************/

static int bmg160_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bmg160_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  bmg160_reset(priv);

  return OK;
}

/****************************************************************************
 * Name: bmg160_read
 ****************************************************************************/

static ssize_t bmg160_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bmg160_dev_s *priv = inode->i_private;
  FAR struct bmg160_sensor_data_s *data;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Check if enough memory was provided for the read call */

  if (buflen < sizeof(FAR struct bmg160_sensor_data_s))
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

  data = (FAR struct bmg160_sensor_data_s *)buffer;
  memset(data, 0, sizeof(FAR struct bmg160_sensor_data_s));

  data->x_gyr = priv->data.x_gyr;
  data->y_gyr = priv->data.y_gyr;
  data->z_gyr = priv->data.z_gyr;

  /* Give back the semaphore */

  nxsem_post(&priv->datasem);

  return sizeof(FAR struct bmg160_sensor_data_s);
}

/****************************************************************************
 * Name: bmg160_write
 ****************************************************************************/

static ssize_t bmg160_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bmg160_ioctl
 ****************************************************************************/

static int bmg160_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
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
 * Name: bmg160_register
 *
 * Description:
 *   Register the BMG160 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gyr0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             BMG160
 *   config  - configuration for the BMG160 driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmg160_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                    FAR struct bmg160_config_s *config)
{
  FAR struct bmg160_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the BMG160 device structure */

  priv = (FAR struct bmg160_dev_s *)kmm_malloc(sizeof(struct bmg160_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi         = spi;
  priv->config      = config;
  priv->work.worker = NULL;

  /* Initialize sensor data access semaphore */

  nxsem_init(&priv->datasem, 0, 1);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(spi, BMG160_SPI_FREQUENCY);
  SPI_SETMODE(spi, BMG160_SPI_MODE);

  /* Attach the interrupt handler */

  ret = priv->config->attach(priv->config, &bmg160_interrupt_handler);
  if (ret < 0)
    {
      snerr("ERROR: Failed to attach interrupt\n");
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_bmg160_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      nxsem_destroy(&priv->datasem);
      return ret;
    }

  /* Since we support multiple BMG160 devices, we will need to add this new
   * instance to a list of device instances so that it can be found by the
   * interrupt handler based on the received IRQ number.
   */

  priv->flink = g_bmg160_list;
  g_bmg160_list = priv;

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_BMG160 */
