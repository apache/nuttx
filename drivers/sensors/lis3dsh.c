/****************************************************************************
 * drivers/sensors/lis3dsh.c
 * Character driver for the LIS3DSH 3-Axis acclerometer.
 *
 *   Copyright (C) 2016 DS-Automotion GmbH. All rights reserved.
 *   Author:  Alexander Entinger <a.entinger@ds-automotion.com>
 *            Thomas Ilk
 *            Florian Olbrich <flox@posteo.de>
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
#include <nuttx/sensors/lis3dsh.h>

#if defined(CONFIG_SPI) && defined(CONFIG_LIS3DSH)

/****************************************************************************
 * Private
 ****************************************************************************/

struct lis3dsh_sensor_data_s
{
  int16_t x_acc;                       /* Measurement result for x axis */
  int16_t y_acc;                       /* Measurement result for y axis */
  int16_t z_acc;                       /* Measurement result for z axis */
};

struct lis3dsh_dev_s
{
  FAR struct lis3dsh_dev_s *flink;     /* Supports a singly linked list of
                                        * drivers */
  FAR struct spi_dev_s *spi;           /* Pointer to the SPI instance */
  FAR struct lis3dsh_config_s *config; /* Pointer to the configuration
                                        * of the LIS3DSH sensor */
  sem_t datasem;                       /* Manages exclusive access to this
                                        * structure */
  struct lis3dsh_sensor_data_s data;   /* The data as measured by the sensor */
  struct work_s work;                  /* The work queue is responsible for
                                        * retrieving the data from the
                                        * sensor after the arrival of new
                                        * data was signalled in an interrupt */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SENSORS_INFO
static void lis3dsh_read_register(FAR struct lis3dsh_dev_s *dev,
                                  uint8_t const reg_addr, uint8_t *reg_data);
#endif

static void lis3dsh_write_register(FAR struct lis3dsh_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint8_t const reg_data);
static void lis3dsh_reset(FAR struct lis3dsh_dev_s *dev);
static void lis3dsh_read_measurement_data(FAR struct lis3dsh_dev_s *dev);
static void lis3dsh_read_acclerometer_data(FAR struct lis3dsh_dev_s *dev,
                                           uint16_t *x_acc, uint16_t *y_acc,
                                           uint16_t *z_acc);
static int lis3dsh_interrupt_handler(int irq, FAR void *context,
                                     FAR void *arg);
static void lis3dsh_worker(FAR void *arg);

static int lis3dsh_open(FAR struct file *filep);
static int lis3dsh_close(FAR struct file *filep);
static ssize_t lis3dsh_read(FAR struct file *, FAR char *buffer,
                            size_t buflen);
static ssize_t lis3dsh_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int lis3dsh_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lis3dsh_fops =
{
  lis3dsh_open,
  lis3dsh_close,
  lis3dsh_read,
  lis3dsh_write,
  NULL,
  lis3dsh_ioctl,
  NULL
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/* Single linked list to store instances of drivers */

static struct lis3dsh_dev_s *g_lis3dsh_list = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SENSORS_INFO
/****************************************************************************
 * Name: lis3dsh_read_register
 ****************************************************************************/

static void lis3dsh_read_register(FAR struct lis3dsh_dev_s *dev,
                                  uint8_t const reg_addr, uint8_t * reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3DSH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read - the MSB needs
   * to be set to indicate the read indication.
   */

  SPI_SEND(dev->spi, reg_addr | 0x80);

  /* Write an idle byte while receiving the required data */

  *reg_data = (uint8_t) (SPI_SEND(dev->spi, 0));

  /* Set CS to high which deselects the LIS3DSH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}
#endif

/****************************************************************************
 * Name: lis3dsh_write_register
 ****************************************************************************/

static void lis3dsh_write_register(FAR struct lis3dsh_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint8_t const reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3DSH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read */

  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written in the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high which deselects the LIS3DSH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3dsh_reset
 ****************************************************************************/

static void lis3dsh_reset(FAR struct lis3dsh_dev_s *dev)
{
  lis3dsh_write_register(dev, LIS3DSH_CTRL_REG_6,
                         LIS3DSH_CTRL_REG_6_BOOT_BM);

  up_mdelay(100);
}

/****************************************************************************
 * Name: lis3dsh_read_measurement_data
 ****************************************************************************/

static void lis3dsh_read_measurement_data(FAR struct lis3dsh_dev_s *dev)
{
  uint16_t x_acc = 0;
  uint16_t y_acc = 0;
  uint16_t z_acc = 0;
  int ret;

  /* Read acclerometer data */

  lis3dsh_read_acclerometer_data(dev, &x_acc, &y_acc, &z_acc);

  /* Acquire the semaphore before the data is copied */

  ret = nxsem_wait(&dev->datasem);
  if (ret < 0)
    {
      snerr("ERROR: Could not acquire dev->datasem: %d\n", ret);
      return;
    }

  /* Copy retrieve data to internal data structure */

  dev->data.x_acc = (int16_t)x_acc;
  dev->data.y_acc = (int16_t)y_acc;
  dev->data.z_acc = (int16_t)z_acc;

  /* Give back the semaphore */

  nxsem_post(&dev->datasem);

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((x_acc << 16) ^ (y_acc << 8) ^ (z_acc << 0));
}

/****************************************************************************
 * Name: lis3dsh_read_acclerometer_data
 ****************************************************************************/

static void lis3dsh_read_acclerometer_data(FAR struct lis3dsh_dev_s *dev,
                                           uint16_t * x_acc,
                                           uint16_t * y_acc,
                                           uint16_t * z_acc)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3DSH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set -> Read Indication.
   */

  SPI_SEND(dev->spi, (LIS3DSH_OUT_X_L_REG | 0x80));

  /* RX */

  *x_acc = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);   /* LSB */
  *x_acc |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *y_acc = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);   /* LSB */
  *y_acc |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  *z_acc = ((uint16_t) (SPI_SEND(dev->spi, 0)) << 0);   /* LSB */
  *z_acc |= ((uint16_t) (SPI_SEND(dev->spi, 0)) << 8);  /* MSB */

  /* Set CS to high which deselects the LIS3DSH */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3dsh_interrupt_handler
 ****************************************************************************/

static int lis3dsh_interrupt_handler(int irq, FAR void *context,
                                     FAR void *arg)
{
  /* This function should be called upon a rising edge on the LIS3DSH new
   * data interrupt pin since it signals that new data has been measured.
   */

  FAR struct lis3dsh_dev_s *priv = NULL;
  int ret;

  /* Find out which device caused the interrupt */

  for (priv = g_lis3dsh_list;
       priv && priv->config->irq != irq;
       priv = priv->flink)
    {
    }

  DEBUGASSERT(priv != NULL);

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long. Also we cannot lock
   * the SPI bus from within an interrupt.
   */

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, lis3dsh_worker, priv, 0);
      if (ret < 0)
        {
          snerr("ERROR: Failed to queue work: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: lis3dsh_worker
 ****************************************************************************/

static void lis3dsh_worker(FAR void *arg)
{
  FAR struct lis3dsh_dev_s *priv = (FAR struct lis3dsh_dev_s *)(arg);
  DEBUGASSERT(priv != NULL);

  /* Read out the latest sensor data */

  lis3dsh_read_measurement_data(priv);
}

/****************************************************************************
 * Name: lis3dsh_open
 ****************************************************************************/

static int lis3dsh_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3dsh_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  lis3dsh_reset(priv);

  /* Enable - the full scale range (FS = +/- 16 g) */

  lis3dsh_write_register(priv,
                         LIS3DSH_CTRL_REG_5, LIS3DSH_CTRL_REG_5_FSCALE_2_BM);

  /* Enable - Auto increment of address when reading multiple bytes */

  lis3dsh_write_register(priv,
                         LIS3DSH_CTRL_REG_6, LIS3DSH_CTRL_REG_6_ADD_INC_BM);

  /* Enable - Measurement of X-, Y-, and Z-axis - Block data update for
   * accelerating data This should prevent race conditions when reading
   * sensor data - fastest output data rate (ODR = 1600 Hz).
   */

  lis3dsh_write_register(priv,
                         LIS3DSH_CTRL_REG_4,
                         LIS3DSH_CTRL_REG_4_XEN_BM |
                         LIS3DSH_CTRL_REG_4_YEN_BM |
                         LIS3DSH_CTRL_REG_4_ZEN_BM |
                         LIS3DSH_CTRL_REG_4_BDU_BM |
                         LIS3DSH_CTRL_REG_4_ODR_3_BM |
                         LIS3DSH_CTRL_REG_4_ODR_0_BM);

  /* Enable - DRDY signal enable to INT 1 */

  lis3dsh_write_register(priv,
                         LIS3DSH_CTRL_REG_3,
                         LIS3DSH_CTRL_REG_3_DR_EN_BM |
                         LIS3DSH_CTRL_REG_3_IEA_BM |
                         LIS3DSH_CTRL_REG_3_IEL_BM |
                         LIS3DSH_CTRL_REG_3_INT1_EN_BM);

  /* Read back the content of all control registers for debug purposes */

#ifdef CONFIG_DEBUG_SENSORS_INFO
    {
      uint8_t reg_content = 0;

      lis3dsh_read_register(priv, LIS3DSH_CTRL_REG_1, &reg_content);
      sninfo("LIS3DSH_CTRL_REG_1 = %04x\n", reg_content);
      lis3dsh_read_register(priv, LIS3DSH_CTRL_REG_2, &reg_content);
      sninfo("LIS3DSH_CTRL_REG_2 = %04x\n", reg_content);
      lis3dsh_read_register(priv, LIS3DSH_CTRL_REG_3, &reg_content);
      sninfo("LIS3DSH_CTRL_REG_3 = %04x\n", reg_content);
      lis3dsh_read_register(priv, LIS3DSH_CTRL_REG_4, &reg_content);
      sninfo("LIS3DSH_CTRL_REG_4 = %04x\n", reg_content);
      lis3dsh_read_register(priv, LIS3DSH_CTRL_REG_5, &reg_content);
      sninfo("LIS3DSH_CTRL_REG_5 = %04x\n", reg_content);
      lis3dsh_read_register(priv, LIS3DSH_CTRL_REG_6, &reg_content);
      sninfo("LIS3DSH_CTRL_REG_6 = %04x\n", reg_content);
      lis3dsh_read_register(priv, LIS3DSH_STATUS_REG, &reg_content);
      sninfo("STATUS_REG = %04x\n", reg_content);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: lis3dsh_close
 ****************************************************************************/

static int lis3dsh_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3dsh_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Perform a reset */

  lis3dsh_reset(priv);

  return OK;
}

/****************************************************************************
 * Name: lis3dsh_read
 ****************************************************************************/

static ssize_t lis3dsh_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lis3dsh_dev_s *priv = inode->i_private;
  FAR struct lis3dsh_sensor_data_s *data;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Check if enough memory was provided for the read call */

  if (buflen < sizeof(FAR struct lis3dsh_sensor_data_s))
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

  data = (FAR struct lis3dsh_sensor_data_s *)buffer;
  memset(data, 0, sizeof(FAR struct lis3dsh_sensor_data_s));

  data->x_acc = priv->data.x_acc;
  data->y_acc = priv->data.y_acc;
  data->z_acc = priv->data.z_acc;

  /* Give back the semaphore */

  nxsem_post(&priv->datasem);

  return sizeof(FAR struct lis3dsh_sensor_data_s);
}

/****************************************************************************
 * Name: lis3dsh_write
 ****************************************************************************/

static ssize_t lis3dsh_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lis3dsh_ioctl
 ****************************************************************************/

static int lis3dsh_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
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
 * Name: lis3dsh_register
 *
 * Description:
 *   Register the LIS3DSH character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/acc0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             LIS3DSH
 *   config  - configuration for the LIS3DSH driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis3dsh_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     FAR struct lis3dsh_config_s *config)
{
  FAR struct lis3dsh_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the LIS3DSH device structure */

  priv =
      (FAR struct lis3dsh_dev_s *)kmm_malloc(sizeof(struct lis3dsh_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi         = spi;
  priv->config      = config;
  priv->work.worker = NULL;

  nxsem_init(&priv->datasem, 0, 1);     /* Initialize sensor data access
                                         * semaphore */

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(spi, LIS3DSH_SPI_FREQUENCY);
  SPI_SETMODE(spi, LIS3DSH_SPI_MODE);

  /* Register the character driver */

  ret = register_driver(devpath, &g_lis3dsh_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      nxsem_destroy(&priv->datasem);
      return ret;
    }

  /* Since we support multiple LIS3DSH devices, we will need to add this new
   * instance to a list of device instances so that it can be found by the
   * interrupt handler based on the received IRQ number.
   */

  priv->flink = g_lis3dsh_list;
  g_lis3dsh_list = priv;

  /* Attach the interrupt handler */

  ret = priv->config->attach(priv->config, &lis3dsh_interrupt_handler);
  if (ret < 0)
    {
      snerr("ERROR: Failed to attach interrupt: %d\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_LIS3DSH */
