/****************************************************************************
 * drivers/sensors/lsm330_spi.c
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

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_LSM330SPI) \
    && defined(CONFIG_SPI_EXCHANGE)

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/lsm330.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSM330_INITIAL_ACLCR_SIZE  7
#define LSM330_INITIAL_GYROCR_SIZE 5

/****************************************************************************
 * Private structure definitions
 ****************************************************************************/

struct sensor_data_s
{
  int16_t x_gyr;              /* Measurement result for x axis */
  int16_t y_gyr;              /* Measurement result for y axis */
  int16_t z_gyr;              /* Measurement result for z axis */
};

struct lsm330_dev_s
{
  FAR struct lsm330_dev_s *flink;     /* Supports a singly linked list of
                                       * drivers */
  FAR struct spi_dev_s *spi;          /* Pointer to the SPI instance */
  FAR struct lsm330_config_s *config; /* Pointer to the configuration of the
                                       * LSM330 sensor */
  mutex_t devicelock;                 /* Manages exclusive access to this
                                       * device */
  struct sensor_data_s data;          /* The data as measured by the sensor */
  uint8_t seek_address;               /* Current device address. */
  uint8_t readonly;                   /* 0 = writing to the device in enabled */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t  lsm330_read_register(FAR struct lsm330_dev_s *dev,
                                     uint8_t reg_addr);
static void     lsm330_read_acl_registerblk(FAR struct lsm330_dev_s *dev,
                                            uint8_t reg_addr,
                                            FAR uint8_t *reg_data,
                                            uint8_t xfercnt);
static void     lsm330_read_gyro_registerblk(FAR struct lsm330_dev_s *dev,
                                             uint8_t reg_addr,
                                             FAR uint8_t *reg_data,
                                             uint8_t xfercnt);
static void     lsm330_write_register(FAR struct lsm330_dev_s *dev,
                                      uint8_t reg_addr,
                                      uint8_t reg_data);
static void     lsm330_write_acl_registerblk(FAR struct lsm330_dev_s *dev,
                                             uint8_t reg_addr,
                                             FAR uint8_t *reg_data,
                                             uint8_t xfercnt);
static void     lsm330_write_gyro_registerblk(FAR struct lsm330_dev_s *dev,
                                              uint8_t reg_addr,
                                              FAR uint8_t *reg_data,
                                              uint8_t xfercnt);
static void     lsm330acl_reset(FAR struct lsm330_dev_s *dev);
static void     lsm330gyro_reset(FAR struct lsm330_dev_s *dev);
static int      lsm330acl_open(FAR struct file *filep);
static int      lsm330gyro_open(FAR struct file *filep);
static int      lsm330acl_close(FAR struct file *filep);
static int      lsm330gyro_close(FAR struct file *filep);
static ssize_t  lsm330acl_read(FAR struct file *, FAR char *, size_t);
static ssize_t  lsm330gyro_read(FAR struct file *, FAR char *, size_t);
static ssize_t  lsm330acl_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static ssize_t  lsm330gyro_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen);
static off_t    lsm330acl_seek(FAR struct file *filep, off_t offset,
                               int whence);
static off_t    lsm330gyro_seek(FAR struct file *filep, off_t offset,
                                int whence);
static int      lsm330_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

static int      lsm330acl_dvr_open(FAR void *instance_handle, int32_t arg);
static int      lsm330acl_dvr_close(FAR void *instance_handle, int32_t arg);
static int      lsm330gyro_dvr_open(FAR void *instance_handle, int32_t arg);
static int      lsm330gyro_dvr_close(FAR void *instance_handle,
                                     int32_t arg);
static ssize_t  lsm330acl_dvr_read(FAR void *instance_handle,
                                   FAR char *buffer, size_t buflen);
static ssize_t  lsm330gyro_dvr_read(FAR void *instance_handle,
                                    FAR char *buffer, size_t buflen);
static ssize_t  lsm330acl_dvr_write(FAR void *instance_handle,
                                    FAR const char *buffer, size_t buflen);
static ssize_t  lsm330gyro_dvr_write(FAR void *instance_handle,
                                     FAR const char *buffer, size_t buflen);
static off_t    lsm330acl_dvr_seek(FAR void *instance_handle, off_t offset,
                                   int whence);
static off_t    lsm330gyro_dvr_seek(FAR void *instance_handle, off_t offset,
                                    int whence);
static int      lsm330_dvr_ioctl(FAR void *instance_handle, int cmd,
                                 unsigned long arg);
static void     lsm330_dvr_exchange(FAR void *instance_handle,
                                    FAR const void *txbuffer,
                                    FAR void *rxbuffer, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lsm330a_fops =
{
  lsm330acl_open,      /* open */
  lsm330acl_close,     /* close */
  lsm330acl_read,      /* read */
  lsm330acl_write,     /* write */
  lsm330acl_seek,      /* seek */
  lsm330_ioctl,        /* ioctl */
  NULL                 /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL               /* unlink */
#endif
};

static const struct file_operations g_lsm330g_fops =
{
  lsm330gyro_open,     /* open */
  lsm330gyro_close,    /* close */
  lsm330gyro_read,     /* read */
  lsm330gyro_write,    /* write */
  lsm330gyro_seek,     /* seek */
  lsm330_ioctl,        /* ioctl */
  NULL                 /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL               /* unlink */
#endif
};

static const struct lsm330spi_dvr_entry_vector_s g_lsm330acl_dops =
{
  { /* Standard sensor cluster driver entry-vector */
    .driver_open    = lsm330acl_dvr_open,
    .driver_close   = lsm330acl_dvr_close,
    .driver_read    = lsm330acl_dvr_read,
    .driver_write   = lsm330acl_dvr_write,
    .driver_seek    = lsm330acl_dvr_seek,
    .driver_ioctl   = lsm330_dvr_ioctl,
    .driver_suspend = 0,
    .driver_resume  = 0,
  },

  /* lsm330 extensions follow */

  .driver_spiexc = lsm330_dvr_exchange,
};

static const struct lsm330spi_dvr_entry_vector_s g_lsm330gyro_dops =
{
  { /* Standard sensor cluster driver entry-vector */
    .driver_open    = lsm330gyro_dvr_open,
    .driver_close   = lsm330gyro_dvr_close,
    .driver_read    = lsm330gyro_dvr_read,
    .driver_write   = lsm330gyro_dvr_write,
    .driver_seek    = lsm330gyro_dvr_seek,
    .driver_ioctl   = lsm330_dvr_ioctl,
    .driver_suspend = 0,
    .driver_resume  = 0,
  },

  /* lsm330 extensions follow */

  .driver_spiexc = lsm330_dvr_exchange,
};

/* Single linked list to store instances of drivers */

static struct lsm330_dev_s *g_lsm330a_list = NULL;
static struct lsm330_dev_s *g_lsm330g_list = NULL;

/* Default accelerometer initialization sequence:
 *
 * Configure LSM330 to measure mode.
 *
 * 1. CR5: ODR=0, power-off
 * 2. CR6: Bandwidth=800Hz. +/-16g range. 4-wire SPI.
 * 3. CR7: Make sure auto-increment is turned on.
 * 4. CR2: Zero state machine 1.
 * 5. CR3: Zero state machine 2.
 * 6. CR4: DataReady interrupt disabled.
 * 7. CR5: Power up. 1600Hz sample rate. X,Y, & Z enabled.
 */

static struct lsm330_reg_pair_s g_default_lsm330_aclcr_values[] =
{
  /* CR5 ODR[3:0]        BDU ZEN YEN XEN
   *     0000=Off         0   0   0   0=all disabled
   */

  {
    .addr  = LSM330_ACL_CTRL_REG5,
    .value = 0x00
  },

  /* CR6 BW[2:1]   FSCALE[2:0] - -  SIM
   *     00=800Hz   10 0=16g   0 0  0=4-wire
   */

  {
    .addr  = LSM330_ACL_CTRL_REG6,
    .value = 0x20
  },

  /* CR7 BOOT FIFO_EN WTM_EN ADD_INC P1_MTY P1_WTM P1_OVR WTM_EN
   *      0      0      0      1       0      0      0       0
   */

  {
    .addr  = LSM330_ACL_CTRL_REG7,
    .value = 0x10
  },

  /* CR2 HYST1 -  SM1_PIN - - SM1_EN
   *     000   0     0    0 0    0
   */

  {
    .addr  = LSM330_ACL_CTRL_REG2,
    .value = 0x00
  },

  /* CR3 HYST2 -  SM2_PIN - - SM2_EN
   *     000   0     0    0 0    0
   */

  {
    .addr  = LSM330_ACL_CTRL_REG3,
    .value = 0x00
  },

  /* CR4 DR_EN IEA IEL INT2_EN INT1_EN  VFILT STRT
   *       1    1   0     0       0       0    0
   */

  {
    .addr  = LSM330_ACL_CTRL_REG4,
    .value = 0xc0
  },

  /* CR5 ODR[3:0]        BDU ZEN YEN XEN
   *     1001=1600Hz      1   1   1   1=all enabled
  */

  {
    .addr  = LSM330_ACL_CTRL_REG5,
    .value = 0x9f
  }
};

/* Default gyroscope initialization sequence
 *
 * Configure LSM330 Gyroscope to measure mode.
 *
 * 1. CR1: Power up. 760Hz sample rate. Bandwidth=100Hz. X,Y, & Z enabled.
 * 2. CR2: Normal measurement mode. High pass filter OFF.
 * 3. CR3: All interrupts disabled.
 * 4. CR4: +/-500dps range. Block update. 4-wire SPI.
 * 5. CR5: Select Low Pass Filter 1 outputs. LPF2 bypassed.
 */

static struct lsm330_reg_pair_s g_default_lsm330_gyrocr_values[] =
{
  /* CR1  DR[1:0]   BW[1:0]    PD Zen Xen Yen
   *      1 1=760Hz 1 1=100Hz  1   1   1   1
   */

  {
    .addr =  LSM330_GYRO_CTRL_REG1,
    .value = 0xff
  },

  /* CR2 EXTRen LVLen HPM[1:0]  HPCF[3:0]
   *      0      0    00=Normal xxxx  Default
   */

  {
    .addr  = LSM330_GYRO_CTRL_REG2,
    .value = 0x00
  },

  /* CR3 I1_Int1 I1_Boot H_Lactive PP_OD I2_DRDY I2_WTM I2_ORun I2_Empty
   *       0        0       0        0      0      0      0        0
   */

  {
    .addr  = LSM330_GYRO_CTRL_REG3,
    .value = 0x00
  },

  /* CR4 BDU BLE FS[1:0]   0 0 0 SIM
   *      1   0  01=500dps 0 0 0 0=4-wire
   */

  {
    .addr  = LSM330_GYRO_CTRL_REG4,
    .value = 0x90
  },

  /* CR5 BOOT FIFO_EN  - HPen INT1_Sel[1:0] Out_Sel[1:0]
   *      0     0      0  0    00=LPF1        00=LPF1
   */

  {
    .addr  = LSM330_GYRO_CTRL_REG5,
    .value = 0x0a
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm330_read_register
 ****************************************************************************/

static uint8_t lsm330_read_register(FAR struct lsm330_dev_s *dev,
                                    uint8_t reg_addr)
{
  uint8_t reg_data;

  /* Lock the SPI bus so that only one device can access it
   * at the same time
   */

  SPI_LOCK(dev->spi, true);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(dev->spi, LSM330_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, LSM330_SPI_MODE);

  /* Set CS to low to select the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read. */

  SPI_SEND(dev->spi, reg_addr | LSM330_READ);

  /* Write an idle byte while receiving the requested data */

  reg_data = (uint8_t) (SPI_SEND(dev->spi, 0xff));

  /* Set CS to high to deselect the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
  return reg_data;
}

/****************************************************************************
 * Name: lsm330_read_acl_registerblk
 ****************************************************************************/

static void lsm330_read_acl_registerblk(FAR struct lsm330_dev_s *dev,
                                        uint8_t reg_addr,
                                        FAR uint8_t *reg_data,
                                        uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it
   * at the same time
   */

  SPI_LOCK(dev->spi, true);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(dev->spi, LSM330_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, LSM330_SPI_MODE);

  /* Set CS to low to select the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading */

  SPI_SEND(dev->spi, reg_addr | LSM330_READ);

  /* Write idle bytes while receiving the requested data */

  while (0 != xfercnt--)
    {
      *reg_data++ = (uint8_t)SPI_SEND(dev->spi, 0xff);
    }

  /* Set CS to high to deselect the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lsm330_read_gyro_registerblk
 ****************************************************************************/

static void lsm330_read_gyro_registerblk(FAR struct lsm330_dev_s *dev,
                                         uint8_t reg_addr,
                                         FAR uint8_t *reg_data,
                                         uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it
   * at the same time
   */

  SPI_LOCK(dev->spi, true);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(dev->spi, LSM330_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, LSM330_SPI_MODE);

  /* Set CS to low to select the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading */

  SPI_SEND(dev->spi, reg_addr | LSM330_READ |
           (xfercnt > 1 ? LSM330_GYRO_AUTO : 0));

  /* Write idle bytes while receiving the requested data */

  while (0 != xfercnt--)
    {
      *reg_data++ = (uint8_t)SPI_SEND(dev->spi, 0xff);
    }

  /* Set CS to high to deselect the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lsm330_write_register
 ****************************************************************************/

static void lsm330_write_register(FAR struct lsm330_dev_s *dev,
                                  uint8_t reg_addr,
                                  uint8_t reg_data)
{
  /* Lock the SPI bus so that only one device can access it
   * at the same time
   */

  SPI_LOCK(dev->spi, true);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(dev->spi, LSM330_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, LSM330_SPI_MODE);

  /* Set CS to low to select the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address to where we want to write */

  SPI_SEND(dev->spi, reg_addr | LSM330_WRITE);

  /* Transmit the content which should be written into the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high to deselect the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lsm330_write_acl_registerblk
 ****************************************************************************/

static void lsm330_write_acl_registerblk(FAR struct lsm330_dev_s *dev,
                                         uint8_t reg_addr,
                                         FAR uint8_t *reg_data,
                                         uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it
   * at the same time
   */

  SPI_LOCK(dev->spi, true);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(dev->spi, LSM330_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, LSM330_SPI_MODE);

  /* Set CS to low which selects the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address to where we want to start writing */

  SPI_SEND(dev->spi, reg_addr | LSM330_WRITE);

  /* Transmit the content which should be written in the register block */

  while (0 != xfercnt--)
    {
      SPI_SEND(dev->spi, *reg_data++);
    }

  /* Set CS to high to deselect the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lsm330_write_gyro_registerblk
 ****************************************************************************/

static void lsm330_write_gyro_registerblk(FAR struct lsm330_dev_s *dev,
                                          uint8_t reg_addr,
                                          FAR uint8_t *reg_data,
                                          uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it
   * at the same time
   */

  SPI_LOCK(dev->spi, true);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(dev->spi, LSM330_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, LSM330_SPI_MODE);

  /* Set CS to low which selects the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address to where we want to start writing */

  SPI_SEND(dev->spi, reg_addr | LSM330_WRITE |
           (xfercnt > 1 ? LSM330_GYRO_AUTO : 0));

  /* Transmit the content which should be written in the register block */

  while (0 != xfercnt--)
    {
      SPI_SEND(dev->spi, *reg_data++);
    }

  /* Set CS to high to deselect the LSM330 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lsm330acl_reset
 ****************************************************************************/

static void lsm330acl_reset(FAR struct lsm330_dev_s *dev)
{
  /* Reset LSM330 Accelerometer. Write only. Begin a boot.
   * Note that the LSM330 ACL does not set the BOOT bit for read, so we
   * can't loop on it.
   */

  lsm330_write_register(dev, LSM330_ACL_CTRL_REG7, LSM330_ACR7_BOOT);

  /* Wait for boot to finish */

  up_mdelay(20);

  /* Set auto-increment.
   *
   * CR7 BOOT FIFO_EN WTM_EN ADD_INC P1_MTY P1_WTM P1_OVR WTM_EN
   *      0      0      0      1       0      0      0       0
   */

  lsm330_write_register(dev, LSM330_ACL_CTRL_REG7, 0x10);
}

/****************************************************************************
 * Name: lsm330gyro_reset
 ****************************************************************************/

static void lsm330gyro_reset(FAR struct lsm330_dev_s *dev)
{
  /* Reset LSM330 Gyroscope. Write only. Begin a boot.
   * Note that the LSM330 ACL does not set the BOOT bit for read, so we
   * can't loop on it.
   */

  lsm330_write_register(dev, LSM330_GYRO_CTRL_REG5, LSM_GYRO_BOOT_MASK);

  /* Wait for boot to finish */

  up_mdelay(20);
}

/****************************************************************************
 * Name: lsm330acl_dvr_open
 ****************************************************************************/

static int lsm330acl_dvr_open(FAR void *instance_handle, int32_t arg)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;
  FAR struct lsm330_reg_pair_s *initp;
  uint8_t reg_content;
  int ret;
  int sz;
  int i;

  sninfo("lsm330acl_open: entered...\n");

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  ret = nxmutex_trylock(&priv->devicelock);
  if (ret < 0)
    {
      sninfo("INFO: LSM330 Accelerometer is already open.\n");

      return -EBUSY;
    }

  /* Read the ID Register */

  priv->readonly = false;
  reg_content    = lsm330_read_register(priv, LSM330_ACL_IDREG);

  sninfo("LSM330_ACL_IDREG = 0x%02x\n", reg_content);

  if (reg_content != LSM330_ACL_IDREG_VALUE)
    {
      /* Made info log level to permit open being used as a device probe. */

      snwarn("INFO: Device ID (0x%02X) "
             "does not match expected LSM330 Acl ID (0x%02).\n",
             reg_content, LSM330_ACL_IDREG_VALUE);

      priv->readonly = true;
    }
  else  /* ID matches */
    {
      lsm330acl_reset(priv);  /* Perform a sensor reset */

      /* Choose the initialization sequence */

      if (priv->config->initial_cr_values_size == 0 ||
          priv->config->initial_cr_values == NULL)
        {
          initp = g_default_lsm330_aclcr_values;      /* Default values */
          sz = LSM330_INITIAL_ACLCR_SIZE;

          sninfo("Using default CRs\n");
        }
      else
        {
          initp = priv->config->initial_cr_values;    /* User supplied values */
          sz = priv->config->initial_cr_values_size;

          sninfo("Using provided CRs\n");
        }

      /* Apply the initialization sequence */

      for (i = 0; i < sz; i++)
        {
          lsm330_write_register(priv, initp[i].addr, initp[i].value);
        }

#ifdef CONFIG_DEBUG_SENSORS_INFO
      /* Read back the content of all control registers for debug purposes */

      reg_content = lsm330_read_register(priv, LSM330_ACL_CTRL_REG5);
      sninfo("LSM330_ACL_CTRL_REG5 = 0x%02x\n", reg_content);

      reg_content = lsm330_read_register(priv, LSM330_ACL_CTRL_REG7);
      sninfo("LSM330_ACL_CTRL_REG7 = 0x%02x\n", reg_content);

      reg_content = lsm330_read_register(priv, LSM330_ACL_CTRL_REG6);
      sninfo("LSM330_ACL_CTRL_REG6 = 0x%02x\n", reg_content);

      reg_content = lsm330_read_register(priv, LSM330_ACL_CTRL_REG4);
      sninfo("LSM330_ACL_CTRL_REG4 = 0x%02x\n", reg_content);
#endif
    }

  priv->seek_address = (uint8_t) LSM330_ACL_OUT_X_L;
  return OK;
}

/****************************************************************************
 * Name: lsm330gyro_dvr_open
 ****************************************************************************/

static int lsm330gyro_dvr_open(FAR void *instance_handle, int32_t arg)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;
  FAR struct lsm330_reg_pair_s *initp;
  uint8_t reg_content;
  int ret;
  int sz;
  int i;

  sninfo("lsm330gyro_open: entered...\n");

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  ret = nxmutex_trylock(&priv->devicelock);
  if (ret < 0)
    {
      sninfo("INFO: LSM330 Gyroscope is already open.\n");
      return -EBUSY;
    }

  /* Read the ID Register */

  priv->readonly = false;
  reg_content    = lsm330_read_register(priv, LSM330_GYRO_IDREG);

  sninfo("LSM330_GYRO_IDREG = 0x%02x\n", reg_content);

  if (reg_content != LSM330_GYRO_IDREG_VALUE)
    {
      /* Made warning log level to permit open being used as
       * a device probe.
       */

      snwarn("INFO: Device ID (0x%02X) "
             "does not match expected LSM330 Gyro ID (0x%02).\n",
             reg_content, LSM330_GYRO_IDREG_VALUE);

      priv->readonly = true;
    }
  else /* ID matches */
    {
      lsm330gyro_reset(priv);     /* Perform a sensor reset */

      /* Choose the initialization sequence */

      if (priv->config->initial_cr_values_size == 0 ||
          priv->config->initial_cr_values == NULL)
        {
          initp = g_default_lsm330_gyrocr_values;     /* Default values */
          sz = LSM330_INITIAL_GYROCR_SIZE;
        }
      else
        {
          initp = priv->config->initial_cr_values;    /* User supplied values */
          sz = priv->config->initial_cr_values_size;
        }

      /* Apply the initialization sequence */

      for (i = 0; i < sz; i++)
        {
          lsm330_write_register(priv, initp[i].addr, initp[i].value);
        }

#ifdef CONFIG_DEBUG_SENSORS_INFO
      /* Read back the content of all control registers for debug purposes */

      reg_content = lsm330_read_register(priv, LSM330_GYRO_CTRL_REG1);
      sninfo("LSM330_GYRO_CTRL_REG1 = 0x%02x\n", reg_content);

      reg_content = lsm330_read_register(priv, LSM330_GYRO_CTRL_REG2);
      sninfo("LSM330_GYRO_CTRL_REG2 = 0x%02x\n", reg_content);

      reg_content = lsm330_read_register(priv, LSM330_GYRO_CTRL_REG3);
      sninfo("LSM330_GYRO_CTRL_REG3 = 0x%02x\n", reg_content);

      reg_content = lsm330_read_register(priv, LSM330_GYRO_CTRL_REG4);
      sninfo("LSM330_GYRO_CTRL_REG4 = 0x%02x\n", reg_content);

      reg_content = lsm330_read_register(priv, LSM330_GYRO_CTRL_REG5);
      sninfo("LSM330_GYRO_CTRL_REG5 = 0x%02x\n", reg_content);
#endif
    }

  priv->seek_address = (uint8_t) LSM330_GYRO_OUT_X_L;
  return OK;
}

/****************************************************************************
 * Name: lsm330acl_dvr_close
 ****************************************************************************/

static int lsm330acl_dvr_close(FAR void *instance_handle, int32_t arg)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  /* Perform a reset to put the sensor in standby mode. */

  lsm330acl_reset(priv);

  /* Release the sensor */

  nxmutex_unlock(&priv->devicelock);
  return OK;
}

/****************************************************************************
 * Name: lsm330gyro_dvr_close
 ****************************************************************************/

static int lsm330gyro_dvr_close(FAR void *instance_handle, int32_t arg)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;

  sninfo("lsm330gyro_close: entered...\n");

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  /* Perform a reset to put the sensor in standby mode. */

  lsm330gyro_reset(priv);

  /* Release the sensor */

  nxmutex_unlock(&priv->devicelock);
  return OK;
}

/****************************************************************************
 * Name: lsm330acl_dvr_read
 ****************************************************************************/

static ssize_t lsm330acl_dvr_read(FAR void *instance_handle,
                                  FAR char *buffer, size_t buflen)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  lsm330_read_acl_registerblk(priv, priv->seek_address, (uint8_t *)buffer,
                              buflen);
  return buflen;
}

/****************************************************************************
 * Name: lsm330gyro_dvr_read
 ****************************************************************************/

static ssize_t lsm330gyro_dvr_read(FAR void *instance_handle,
                                   FAR char *buffer, size_t buflen)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  lsm330_read_gyro_registerblk(priv, priv->seek_address, (uint8_t *)buffer,
                               buflen);
  return buflen;
}

/****************************************************************************
 * Name: lsm330acl_dvr_write
 ****************************************************************************/

static ssize_t lsm330acl_dvr_write(FAR void *instance_handle,
                                   FAR const char *buffer, size_t buflen)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  if (priv->readonly)
    {
      return -EROFS;
    }

  lsm330_write_acl_registerblk(priv, priv->seek_address, (uint8_t *)buffer,
                               buflen);
  return buflen;
}

/****************************************************************************
 * Name: lsm330gyro_dvr_write
 ****************************************************************************/

static ssize_t lsm330gyro_dvr_write(FAR void *instance_handle,
                                    FAR const char *buffer, size_t buflen)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  if (priv->readonly)
    {
      return -EROFS;
    }

  lsm330_write_gyro_registerblk(priv, priv->seek_address,
                                (FAR uint8_t *)buffer, buflen);
  return buflen;
}

/****************************************************************************
 * Name: lsm330acl_dvr_seek
 ****************************************************************************/

static off_t lsm330acl_dvr_seek(FAR void *instance_handle,
                                off_t offset, int whence)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;
  off_t reg;

  DEBUGASSERT(priv != NULL);

  switch (whence)
    {
      case SEEK_CUR:  /* incremental seek */
        reg = priv->seek_address + offset;
        if (0 > reg || reg > LSM330_ACL_LAST)
          {
            return -EINVAL;
          }

        priv->seek_address = reg;
        break;

      case SEEK_END:  /* seek to the 1st X-data register */
        priv->seek_address = LSM330_ACL_OUT_X_L;
        break;

      case SEEK_SET:  /* seek to designated address */
        if (0 > offset || offset > LSM330_ACL_LAST)
          {
            return -EINVAL;
          }

        priv->seek_address = offset;
        break;

    default:  /* Invalid whence */
        return -EINVAL;
    }

  return priv->seek_address;
}

/****************************************************************************
 * Name: lsm330gyro_dvr_seek
 ****************************************************************************/

static off_t lsm330gyro_dvr_seek(FAR void *instance_handle, off_t offset,
                                 int whence)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;
  off_t reg;

  DEBUGASSERT(priv != NULL);

  switch (whence)
    {
      case SEEK_CUR:  /* incremental seek */
        reg = priv->seek_address + offset;
        if (0 > reg || reg > LSM330_GYRO_LAST)
          {
            return -EINVAL;
          }

        priv->seek_address = reg;
        break;

      case SEEK_END:  /* seek to the 1st data register */
        priv->seek_address = LSM330_GYRO_OUT_X_L;
        break;

      case SEEK_SET:  /* seek to designated address */
        if (0 > offset || offset > LSM330_GYRO_LAST)
          {
            return -EINVAL;
          }

        priv->seek_address = offset;
        break;

      default:  /* Invalid whence */
        return -EINVAL;
    }

  return priv->seek_address;
}

/****************************************************************************
 * Name: lsm330_dvr_ioctl
 ****************************************************************************/

static int lsm330_dvr_ioctl(FAR void *instance_handle, int cmd,
                            unsigned long arg)
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
 * Name: lsm330_dvr_exchange (with SPI DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   instance_handle - Pointer to struct lsm330_dev_s.
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lsm330_dvr_exchange(FAR void *instance_handle,
                                FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords)
{
  FAR struct lsm330_dev_s *priv = (FAR struct lsm330_dev_s *)instance_handle;
  FAR struct spi_dev_s *spi = priv->spi;

  sninfo("In lsm330_dvr_exchange: Handle=0x%08X\n", instance_handle);

  /* Lock the SPI bus so that only one device can access it
   * at the same time
   */

  SPI_LOCK(spi, true);

  SPI_SETFREQUENCY(spi, LSM330_SPI_FREQUENCY);
  SPI_SETMODE(spi, LSM330_SPI_MODE);

  sninfo("Calling SPI_EXCHANGE: devid=0x%08X\n", priv->config->spi_devid);

  /* Set CS to low which selects the LSM330 */

  SPI_SELECT(spi, priv->config->spi_devid, true);

  /* Perform an SPI exchange block operation. */

  SPI_EXCHANGE(spi, txbuffer, rxbuffer, nwords);

  /* Set CS to high to deselect the LSM330 */

  SPI_SELECT(spi, priv->config->spi_devid, false);

  sninfo("Returned from : SPI_EXCHANGE\n");

  /* Unlock the SPI bus */

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: lsm330acl_open
 ****************************************************************************/

static int lsm330acl_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;
  int ret;

  ret = lsm330acl_dvr_open((FAR void *)priv, 0);
  return ret;
}

/****************************************************************************
 * Name: lsm330gyro_open
 ****************************************************************************/

static int lsm330gyro_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;
  int ret;

  ret = lsm330gyro_dvr_open((FAR void *)priv, 0);
  return ret;
}

/****************************************************************************
 * Name: lsm330acl_close
 ****************************************************************************/

static int lsm330acl_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;
  int ret;

  ret = lsm330acl_dvr_close((FAR void *)priv, 0);
  return ret;
}

/****************************************************************************
 * Name: lsm330gyro_close
 ****************************************************************************/

static int lsm330gyro_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;
  int ret;

  ret = lsm330gyro_dvr_close((FAR void *)priv, 0);
  return ret;
}

/****************************************************************************
 * Name: lsm330acl_read
 ****************************************************************************/

static ssize_t lsm330acl_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;

  return lsm330acl_dvr_read(priv, buffer, buflen);
}

/****************************************************************************
 * Name: lsm330gyro_read
 ****************************************************************************/

static ssize_t lsm330gyro_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;

  return lsm330gyro_dvr_read(priv, buffer, buflen);
}

/****************************************************************************
 * Name: lsm330acl_write
 ****************************************************************************/

static ssize_t lsm330acl_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;

  return lsm330acl_dvr_write(priv, buffer, buflen);
}

/****************************************************************************
 * Name: lsm330gyro_write
 ****************************************************************************/

static ssize_t lsm330gyro_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;

  return lsm330gyro_dvr_write(priv, buffer, buflen);
}

/****************************************************************************
 * Name: lsm330acl_seek
 ****************************************************************************/

static off_t lsm330acl_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;

  return lsm330acl_dvr_seek(priv, offset, whence);
}

/****************************************************************************
 * Name: lsm330gyro_seek
 ****************************************************************************/

static off_t lsm330gyro_seek(FAR struct file *filep,
                             off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;

  return lsm330gyro_dvr_seek(priv, offset, whence);
}

/****************************************************************************
 * Name: lsm330_ioctl
 ****************************************************************************/

static int lsm330_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm330_dev_s *priv = inode->i_private;

  return lsm330_dvr_ioctl(priv, cmd, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm330_register
 *
 * Description:
 *   Register the LSM330 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath_acl  - The full path to the driver to register. E.g.,
 *                  "/dev/acl0"
 *   devpath_gyro - The full path to the driver to register. E.g.,
 *                  "/dev/gyr0"
 *   spi          - An instance of the SPI interface to use to communicate
 *                  with LSM330
 *   config_acl   - configuration for the LSM330 accelerometer driver.
 *                  For details see description above.
 *   config_gyro  - configuration for the LSM330 gyroscope driver.
 *                  For details see description above.
 *   caller_is_driver - 0 (false) Driver user is a user application using
 *                        the fops interface.
 *                      1 (true)  "Driver to Driver" interface will be used
 *                        in addition to the fops interface.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm330_register(FAR const char *devpath_acl,
                    FAR const char *devpath_gyro,
                    FAR struct spi_dev_s *spi,
                    FAR struct lsm330_config_s *config_acl,
                    FAR struct lsm330_config_s *config_gyro)
{
  FAR struct lsm330_dev_s *priva;
  FAR struct lsm330_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config_acl != NULL);
  DEBUGASSERT(config_gyro != NULL);

  config_acl->leaf_handle  = NULL;
  config_gyro->leaf_handle = NULL;
  config_acl->sc_ops       = NULL;
  config_gyro->sc_ops      = NULL;

  /* Initialize the LSM330 accelerometer device structure. */

  priv = (FAR struct lsm330_dev_s *)kmm_malloc(sizeof(struct lsm330_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate accelerometer instance\n");
      return -ENOMEM;
    }

  priv->spi    = spi;
  priv->config = config_acl;

  /* Initialize sensor and sensor data access mutex */

  nxmutex_init(&priv->devicelock);

  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(spi, LSM330_SPI_FREQUENCY);
  SPI_SETMODE(spi, LSM330_SPI_MODE);

  /* Register the character driver */

  ret = register_driver(devpath_acl, &g_lsm330a_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register accelerometer driver: %d\n", ret);

      nxmutex_destroy(&priv->devicelock);
      kmm_free(priv);
      return ret;
    }

  /* Since we support multiple LSM330 devices, we will need to add this new
   * instance to a list of device instances so that it can be found by the
   * interrupt handler based on the received IRQ number.
   */

  priv->flink = g_lsm330a_list;
  g_lsm330a_list = priv;
  priva = priv;
  config_acl->leaf_handle = (void *)priv;

  /* Initialize the LSM330 gyroscope device structure. */

  priv = (FAR struct lsm330_dev_s *)kmm_malloc(sizeof(struct lsm330_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate gyroscope instance\n");
      ret = -ENOMEM;
      goto err_exit;
    }

  priv->spi    = spi;
  priv->config = config_gyro;

  /* Initialize sensor and sensor data access mutex */

  nxmutex_init(&priv->devicelock);

  /* Register the character driver */

  ret = register_driver(devpath_gyro, &g_lsm330g_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register gyroscope driver: %d\n", ret);

      nxmutex_destroy(&priv->devicelock);
      kmm_free(priv);
      goto err_exit;
    }

  /* Since we support multiple LSM330 devices, we will need to add this new
   * instance to a list of device instances so that it can be found by the
   * interrupt handler based on the received IRQ number.
   */

  priv->flink = g_lsm330g_list;
  g_lsm330g_list = priv;
  config_gyro->leaf_handle = (void *)priv;

  config_acl->sc_ops  = &g_lsm330acl_dops;
  config_gyro->sc_ops = &g_lsm330gyro_dops;

  /* If this is part of a kernel controlled sensor cluster driver,
   * then return a handle to the caller
   */

  return OK;

err_exit:
  /* Registration the of the gyroscope failed, so we need to destroy the
   * accelerometer instance.
   */

  nxmutex_destroy(&priva->devicelock);
  kmm_free(priva);
  return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_LSM330SPI && CONFIG_SPI_EXCHANGE */
