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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>

#include <nuttx/fs/fs.h>

#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/l3gd20.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_L3GD20)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
  uint64_t timestamp;                 /* Units is microseconds */
  struct sensor_lowerhalf_s lower;    /* The struct of lower half driver */
#if CONFIG_SENSORS_L3GD20_BUFFER_SIZE > 0
  struct work_s work;                 /* The work queue is responsible for
                                       * retrieving the data from the sensor
                                       * after the arrival of new data was
                                       * signalled in an interrupt */
#endif
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
static void l3gd20_read_measurement_data(FAR struct l3gd20_dev_s *dev,
                                         FAR struct sensor_gyro *data);
static void l3gd20_read_gyroscope_data(FAR struct l3gd20_dev_s *dev,
                                       uint16_t *x_gyr, uint16_t *y_gyr,
                                       uint16_t *z_gyr);
static void l3gd20_read_temperature(FAR struct l3gd20_dev_s *dev,
                                    uint8_t * temperature);
static int l3gd20_interrupt_handler(int irq, FAR void *context,
                                    FAR void *arg);
static int l3gd20_activate(FAR struct sensor_lowerhalf_s *lower,
                           bool enable);
#if CONFIG_SENSORS_L3GD20_BUFFER_SIZE > 0
static void l3gd20_worker(FAR void *arg);
#else
static int l3gd20_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lower half sensor driver operations for sensor register */

static const struct sensor_ops_s g_l2gd20_ops =
{
  .activate = l3gd20_activate,
  .set_interval = NULL,
  .batch = NULL,
#if CONFIG_SENSORS_L3GD20_BUFFER_SIZE > 0
  .fetch = NULL,
#else
  .fetch = l3gd20_fetch,
#endif
  .control = NULL
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

static void l3gd20_read_measurement_data(FAR struct l3gd20_dev_s *dev,
                                         FAR struct sensor_gyro *data)
{
  uint16_t x_gyr = 0;
  uint16_t y_gyr = 0;
  uint16_t z_gyr = 0;
  uint8_t temperature = 0;

  /* Read Gyroscope */

  l3gd20_read_gyroscope_data(dev, &x_gyr, &y_gyr, &z_gyr);

  /* Read Temperature */

  l3gd20_read_temperature(dev, &temperature);

  data->x = ((int16_t)x_gyr / 180.0f) * (float)M_PI;
  data->y = ((int16_t)y_gyr / 180.0f) * (float)M_PI;
  data->z = ((int16_t)z_gyr / 180.0f) * (float)M_PI;
  data->temperature = temperature;
  data->timestamp   = dev->timestamp;

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

  /* Get the timestamp */

  priv->timestamp = sensor_get_timestamp();

#if CONFIG_SENSORS_L3GD20_BUFFER_SIZE > 0
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
#else

  /* notify event to upper half driver */

  priv->lower.notify_event(priv->lower.priv);

#endif
  return OK;
}

#if CONFIG_SENSORS_L3GD20_BUFFER_SIZE > 0
/****************************************************************************
 * Name: l3gd20_worker
 ****************************************************************************/

static void l3gd20_worker(FAR void *arg)
{
  struct sensor_gyro temp;

  FAR struct l3gd20_dev_s *priv = (FAR struct l3gd20_dev_s *)(arg);
  DEBUGASSERT(priv != NULL);

  /* Read out the latest sensor data */

  l3gd20_read_measurement_data(priv, &temp);

  /* push data to upper half driver */

  priv->lower.push_event(priv->lower.priv, &temp,
                         sizeof(struct sensor_gyro));
}

#else

/****************************************************************************
 * Name: l3gd20_fetch
 ****************************************************************************/

static int l3gd20_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR char *buffer, size_t buflen)
{
  FAR struct l3gd20_dev_s *priv = container_of(lower,
                                               FAR struct l3gd20_dev_s,
                                               lower);

  if (buflen != sizeof(struct sensor_gyro))
      return 0;

  DEBUGASSERT(priv != NULL);

  /* Read out the latest sensor data */

  l3gd20_read_measurement_data(priv, (FAR struct sensor_gyro *)buffer);

  return sizeof(struct sensor_gyro);
}
#endif

/****************************************************************************
 * Name: l3gd20_activate
 ****************************************************************************/

static int l3gd20_activate(FAR struct sensor_lowerhalf_s *lower,
                           bool enable)
{
  FAR struct l3gd20_dev_s *priv = container_of(lower,
                                               FAR struct l3gd20_dev_s,
                                               lower);
  struct sensor_gyro temp;

#ifdef CONFIG_DEBUG_SENSORS_INFO
  uint8_t reg_content;
  uint8_t reg_addr;
#endif

  DEBUGASSERT(priv != NULL);

  if (enable == true)
    {
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

      l3gd20_read_measurement_data(priv, &temp);

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
    }
  else
    {
      /* Perform a reset */

      l3gd20_reset(priv);
    }

  return 0;
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
 *   devno   - The device number, used to build the device path
 *             as /dev/sensor/gyro_uncalN
 *   spi     - An SPI driver instance.
 *   config  - configuration for the L3GD20 driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int l3gd20_register(int devno, FAR struct spi_dev_s *spi,
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
#if CONFIG_SENSORS_L3GD20_BUFFER_SIZE > 0
  priv->work.worker      = NULL;
#endif
  priv->timestamp        = 0;

  priv->lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->lower.nbuffer = CONFIG_SENSORS_L3GD20_BUFFER_SIZE;
  priv->lower.ops = &g_l2gd20_ops;
  priv->lower.uncalibrated = true;

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

  /* Register the sensor driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
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
