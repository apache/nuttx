/****************************************************************************
 * drivers/sensors/mpu60x0.c
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
 * TODO: Theory of Operation
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <limits.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#ifdef CONFIG_MPU60X0_SPI
#include <nuttx/spi/spi.h>
#else
#include <nuttx/i2c/i2c_master.h>
#endif
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/mpu60x0.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sets bit @n */

#define BIT(n) (1 << (n))

/* Creates a mask of @m bits, i.e. MASK(2) -> 00000011 */

#define MASK(m) (BIT((m) + 1) - 1)

/* Masks and shifts @v into bit field @m */

#define TO_BITFIELD(m,v) (((v) & MASK(m ##__WIDTH)) << (m ##__SHIFT))

/* Un-masks and un-shifts bit field @m from @v */

#define FROM_BITFIELD(m,v) (((v) >> (m ##__SHIFT)) & MASK(m ##__WIDTH))

/* SPI read/write codes */

#define MPU_REG_READ 0x80
#define MPU_REG_WRITE 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum mpu_regaddr_e
{
  SELF_TEST_X = 0x0d,
  SELF_TEST_Y = 0x0e,
  SELF_TEST_Z = 0x0f,
  SELF_TEST_A = 0x10,
  SMPLRT_DIV = 0x19,

  /* __SHIFT : number of empty bits to the right of the field
   * __WIDTH : width of the field, in bits
   *
   * single-bit fields don't have __SHIFT or __mask
   */

  CONFIG = 0x1a,
  CONFIG__EXT_SYNC_SET__SHIFT = 3,
  CONFIG__EXT_SYNC_SET__WIDTH = 2,
  CONFIG__DLPF_CFG__SHIFT = 0,
  CONFIG__DLPF_CFG__WIDTH = 2,

  GYRO_CONFIG = 0x1b,
  GYRO_CONFIG__XG_ST = BIT(7),
  GYRO_CONFIG__YG_ST = BIT(6),
  GYRO_CONFIG__ZG_ST = BIT(5),
  GYRO_CONFIG__FS_SEL__SHIFT = 3,
  GYRO_CONFIG__FS_SEL__WIDTH = 2,

  ACCEL_CONFIG = 0x1c,
  ACCEL_CONFIG__XA_ST = BIT(7),
  ACCEL_CONFIG__YA_ST = BIT(6),
  ACCEL_CONFIG__ZA_ST = BIT(5),
  ACCEL_CONFIG__AFS_SEL__SHIFT = 3,
  ACCEL_CONFIG__AFS_SEL__WIDTH = 2,

  MOT_THR = 0x1f,
  FIFO_EN = 0x23,
  I2C_MST_CTRL = 0x24,
  I2C_SLV0_ADDR = 0x25,
  I2C_SLV0_REG = 0x26,
  I2C_SLV0_CTRL = 0x27,
  I2C_SLV1_ADDR = 0x28,
  I2C_SLV1_REG = 0x29,
  I2C_SLV1_CTRL = 0x2a,
  I2C_SLV2_ADDR = 0x2b,
  I2C_SLV2_REG = 0x2c,
  I2C_SLV2_CTRL = 0x2d,
  I2C_SLV3_ADDR = 0x2e,
  I2C_SLV3_REG = 0x2f,
  I2C_SLV3_CTRL = 0x30,
  I2C_SLV4_ADDR = 0x31,
  I2C_SLV4_REG = 0x32,
  I2C_SLV4_DO = 0x33,
  I2C_SLV4_CTRL = 0x34,
  I2C_SLV4_DI = 0x35,         /* RO */
  I2C_MST_STATUS = 0x36,      /* RO */

  INT_PIN_CFG = 0x37,
  INT_PIN_CFG__INT_LEVEL = BIT(7),
  INT_PIN_CFG__INT_OPEN = BIT(6),
  INT_PIN_CFG__LATCH_INT_EN = BIT(5),
  INT_PIN_CFG__INT_RD_CLEAR = BIT(4),
  INT_PIN_CFG__FSYNC_INT_LEVEL = BIT(3),
  INT_PIN_CFG__FSYNC_INT_EN = BIT(2),
  INT_PIN_CFG__I2C_BYPASS_EN = BIT(1),

  INT_ENABLE = 0x38,
  INT_STATUS = 0x3a,          /* RO */

  ACCEL_XOUT_H = 0x3b,        /* RO */
  ACCEL_XOUT_L = 0x3c,        /* RO */
  ACCEL_YOUT_H = 0x3d,        /* RO */
  ACCEL_YOUT_L = 0x3e,        /* RO */
  ACCEL_ZOUT_H = 0x3f,        /* RO */
  ACCEL_ZOUT_L = 0x40,        /* RO */
  TEMP_OUT_H = 0x41,          /* RO */
  TEMP_OUT_L = 0x42,          /* RO */
  GYRO_XOUT_H = 0x43,         /* RO */
  GYRO_XOUT_L = 0x44,         /* RO */
  GYRO_YOUT_H = 0x45,         /* RO */
  GYRO_YOUT_L = 0x46,         /* RO */
  GYRO_ZOUT_H = 0x47,         /* RO */
  GYRO_ZOUT_L = 0x48,         /* RO */

  EXT_SENS_DATA_00 = 0x49,    /* RO */
  EXT_SENS_DATA_01 = 0x4a,    /* RO */
  EXT_SENS_DATA_02 = 0x4b,    /* RO */
  EXT_SENS_DATA_03 = 0x4c,    /* RO */
  EXT_SENS_DATA_04 = 0x4d,    /* RO */
  EXT_SENS_DATA_05 = 0x4e,    /* RO */
  EXT_SENS_DATA_06 = 0x4f,    /* RO */
  EXT_SENS_DATA_07 = 0x50,    /* RO */
  EXT_SENS_DATA_08 = 0x51,    /* RO */
  EXT_SENS_DATA_09 = 0x52,    /* RO */
  EXT_SENS_DATA_10 = 0x53,    /* RO */
  EXT_SENS_DATA_11 = 0x54,    /* RO */
  EXT_SENS_DATA_12 = 0x55,    /* RO */
  EXT_SENS_DATA_13 = 0x56,    /* RO */
  EXT_SENS_DATA_14 = 0x57,    /* RO */
  EXT_SENS_DATA_15 = 0x58,    /* RO */
  EXT_SENS_DATA_16 = 0x59,    /* RO */
  EXT_SENS_DATA_17 = 0x5a,    /* RO */
  EXT_SENS_DATA_18 = 0x5b,    /* RO */
  EXT_SENS_DATA_19 = 0x5c,    /* RO */
  EXT_SENS_DATA_20 = 0x5d,    /* RO */
  EXT_SENS_DATA_21 = 0x5e,    /* RO */
  EXT_SENS_DATA_22 = 0x5f,    /* RO */
  EXT_SENS_DATA_23 = 0x60,    /* RO */

  I2C_SLV0_DO = 0x63,
  I2C_SLV1_DO = 0x64,
  I2C_SLV2_DO = 0x65,
  I2C_SLV3_DO = 0x66,
  I2C_MST_DELAY_CTRL = 0x67,

  SIGNAL_PATH_RESET = 0x68,
  SIGNAL_PATH_RESET__GYRO_RESET = BIT(2),
  SIGNAL_PATH_RESET__ACCEL_RESET = BIT(1),
  SIGNAL_PATH_RESET__TEMP_RESET = BIT(0),
  SIGNAL_PATH_RESET__ALL_RESET = BIT(3) - 1,

  MOT_DETECT_CTRL = 0x69,

  USER_CTRL = 0x6a,
  USER_CTRL__FIFO_EN = BIT(6),
  USER_CTRL__I2C_MST_EN = BIT(5),
  USER_CTRL__I2C_IF_DIS = BIT(4),
  USER_CTRL__FIFO_RESET = BIT(2),
  USER_CTRL__I2C_MST_RESET = BIT(1),
  USER_CTRL__SIG_COND_RESET = BIT(0),

  PWR_MGMT_1 = 0x6b,          /* Reset: 0x40 */
  PWR_MGMT_1__DEVICE_RESET = BIT(7),
  PWR_MGMT_1__SLEEP = BIT(6),
  PWR_MGMT_1__CYCLE = BIT(5),
  PWR_MGMT_1__TEMP_DIS = BIT(3),
  PWR_MGMT_1__CLK_SEL__SHIFT = 0,
  PWR_MGMT_1__CLK_SEL__WIDTH = 3,

  PWR_MGMT_2 = 0x6c,
  FIFO_COUNTH = 0x72,
  FIFO_COUNTL = 0x73,
  FIFO_R_W = 0x74,
  WHO_AM_I = 0x75,            /* RO reset: 0x68 */
};

/* Describes the mpu60x0 sensor register file. This structure reflects
 * the underlying hardware, so don't change it!
 */

begin_packed_struct struct sensor_data_s
{
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t temp;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
} end_packed_struct;

/* Used by the driver to manage the device */

struct mpu_dev_s
{
  mutex_t lock;               /* mutex for this structure */
  struct mpu_config_s config; /* board-specific information */

  struct sensor_data_s buf;   /* temporary buffer (for read(), etc.) */
  size_t bufpos;              /* cursor into @buf, in bytes (!) */
};

/****************************************************************************
 * Private Function Function Prototypes
 ****************************************************************************/

static int mpu_open(FAR struct file *filep);
static int mpu_close(FAR struct file *filep);
static ssize_t mpu_read(FAR struct file *filep, FAR char *buf, size_t len);
static ssize_t mpu_write(FAR struct file *filep, FAR const char *buf,
                         size_t len);
static off_t mpu_seek(FAR struct file *filep, off_t offset, int whence);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mpu_fops =
{
  mpu_open,        /* open */
  mpu_close,       /* close */
  mpu_read,        /* read */
  mpu_write,       /* write */
  mpu_seek,        /* seek */
  NULL,            /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NOTE :
 *
 * In all of the following code, functions named with a double leading
 * underscore '__' must be invoked ONLY if the mpu_dev_s lock is
 * already held. Failure to do this might cause the transaction to get
 * interrupted, which will likely confuse the data you get back.
 *
 * The mpu_dev_s lock is NOT the same thing as, i.e. the SPI master
 * interface lock: the latter protects the bus interface hardware
 * (which may have other SPI devices attached), the former protects
 * the chip and its associated data.
 */

#ifdef CONFIG_MPU60X0_SPI
/* __mpu_read_reg(), but for spi-connected devices. See that function
 * for documentation.
 */

static int __mpu_read_reg_spi(FAR struct mpu_dev_s *dev,
                              enum mpu_regaddr_e reg_addr,
                              FAR uint8_t *buf, uint8_t len)
{
  int ret;
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* We'll probably return the number of bytes asked for. */

  ret = len;

  /* Grab and configure the SPI master device: always mode 0, 20MHz if it's a
   * data register, 1MHz otherwise (per datasheet).
   */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);

  if ((reg_addr >= ACCEL_XOUT_H) && ((reg_addr + len) <= I2C_SLV0_DO))
    {
      SPI_SETFREQUENCY(spi, 20000000);
    }
  else
    {
      SPI_SETFREQUENCY(spi, 1000000);
    }

  /* Select the chip. */

  SPI_SELECT(spi, id, true);

  /* Send the read request. */

  SPI_SEND(spi, reg_addr | MPU_REG_READ);

  /* Clock in the data. */

  while (0 != len--)
    {
      *buf++ = (uint8_t) (SPI_SEND(spi, 0xff));
    }

  /* Deselect the chip, release the SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return ret;
}

/* __mpu_write_reg(), but for SPI connections. */

static int __mpu_write_reg_spi(FAR struct mpu_dev_s *dev,
                               enum mpu_regaddr_e reg_addr,
                               FAR const uint8_t * buf, uint8_t len)
{
  int ret;
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* Hopefully, we'll return all the bytes they're asking for. */

  ret = len;

  /* Grab and configure the SPI master device. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETFREQUENCY(spi, 1000000);

  /* Select the chip. */

  SPI_SELECT(spi, id, true);

  /* Send the write request. */

  SPI_SEND(spi, reg_addr | MPU_REG_WRITE);

  /* Send the data. */

  while (0 != len--)
    {
      SPI_SEND(spi, *buf++);
    }

  /* Release the chip and SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return ret;
}

#else

/* __mpu_read_reg(), but for i2c-connected devices. */

static int __mpu_read_reg_i2c(FAR struct mpu_dev_s *dev,
                              enum mpu_regaddr_e reg_addr,
                              FAR uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_MPU60X0_I2C_FREQ;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_MPU60X0_I2C_FREQ;
  msg[1].addr      = dev->config.addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = len;

  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(read) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

static int __mpu_write_reg_i2c(FAR struct mpu_dev_s *dev,
                               enum mpu_regaddr_e reg_addr,
                               FAR const uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_MPU60X0_I2C_FREQ;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;
  msg[1].frequency = CONFIG_MPU60X0_I2C_FREQ;
  msg[1].addr      = dev->config.addr;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = (FAR uint8_t *)buf;
  msg[1].length    = len;
  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(write) failed: %d\n", ret);
      return ret;
    }

  return OK;
}
#endif /* CONFIG_MPU60X0_SPI */

/* __mpu_read_reg()
 *
 * Reads a block of @len byte-wide registers, starting at @reg_addr,
 * from the device connected to @dev. Bytes are returned in @buf,
 * which must have a capacity of at least @len bytes.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes read, or a negative errno.
 */

static inline int __mpu_read_reg(FAR struct mpu_dev_s *dev,
                                 enum mpu_regaddr_e reg_addr,
                                 FAR uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_MPU60X0_SPI
  /* If we're wired to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return __mpu_read_reg_spi(dev, reg_addr, buf, len);
    }
#else
  /* If we're wired to I2C, use that function. */

  if (dev->config.i2c != NULL)
    {
      return __mpu_read_reg_i2c(dev, reg_addr, buf, len);
    }
#endif

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* __mpu_write_reg()
 *
 * Writes a block of @len byte-wide registers, starting at @reg_addr,
 * using the values in @buf to the device connected to @dev. Register
 * values are taken in numerical order from @buf, i.e.:
 *
 *   buf[0] -> register[@reg_addr]
 *   buf[1] -> register[@reg_addr + 1]
 *   ...
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes written, or a negative errno.
 */

static inline int __mpu_write_reg(FAR struct mpu_dev_s *dev,
                                  enum mpu_regaddr_e reg_addr,
                                  FAR const uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_MPU60X0_SPI
  /* If we're connected to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return __mpu_write_reg_spi(dev, reg_addr, buf, len);
    }
#else
  if (dev->config.i2c != NULL)
    {
      return __mpu_write_reg_i2c(dev, reg_addr, buf, len);
    }
#endif

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* __mpu_read_imu()
 *
 * Reads the whole IMU data file from @dev in one uninterrupted pass,
 * placing the sampled values into @buf. This function is the only way
 * to guarantee that the measured values are sampled as closely-spaced
 * in time as the hardware permits, which is almost always what you
 * want.
 */

static inline int __mpu_read_imu(FAR struct mpu_dev_s *dev,
                                 FAR struct sensor_data_s *buf)
{
  return __mpu_read_reg(dev, ACCEL_XOUT_H, (uint8_t *) buf, sizeof(*buf));
}

/* __mpu_read_pwr_mgmt_1()
 *
 * Returns the value of the PWR_MGMT_1 register from @dev.
 */

static inline uint8_t __mpu_read_pwr_mgmt_1(FAR struct mpu_dev_s *dev)
{
  uint8_t buf = 0xff;
  __mpu_read_reg(dev, PWR_MGMT_1, &buf, sizeof(buf));
  return buf;
}

static inline int __mpu_write_signal_path_reset(FAR struct mpu_dev_s *dev,
                                                uint8_t val)
{
  return __mpu_write_reg(dev, SIGNAL_PATH_RESET, &val, sizeof(val));
}

static inline int __mpu_write_int_pin_cfg(FAR struct mpu_dev_s *dev,
                                          uint8_t val)
{
  return __mpu_write_reg(dev, INT_PIN_CFG, &val, sizeof(val));
}

static inline int __mpu_write_pwr_mgmt_1(FAR struct mpu_dev_s *dev,
                                         uint8_t val)
{
  return __mpu_write_reg(dev, PWR_MGMT_1, &val, sizeof(val));
}

static inline int __mpu_write_pwr_mgmt_2(FAR struct mpu_dev_s *dev,
                                         uint8_t val)
{
  return __mpu_write_reg(dev, PWR_MGMT_2, &val, sizeof(val));
}

#ifdef CONFIG_MPU60X0_SPI
static inline int __mpu_write_user_ctrl(FAR struct mpu_dev_s *dev,
                                        uint8_t val)
{
  return __mpu_write_reg(dev, USER_CTRL, &val, sizeof(val));
}
#endif

/* __mpu_write_gyro_config() :
 *
 * Sets the @fs_sel bit in GYRO_CONFIG to the value provided. Per the
 * datasheet, the meaning of @fs_sel is as follows:
 *
 * GYRO_CONFIG(0x1b) :   XG_ST YG_ST ZG_ST FS_SEL1 FS_SEL0 x  x  x
 *
 *    XG_ST, YG_ST, ZG_ST  :  self-test (unsupported in this driver)
 *         1 -> activate self-test on X, Y, and/or Z gyros
 *
 *    FS_SEL[10] : full-scale range select
 *         0 -> ±  250 deg/sec
 *         1 -> ±  500 deg/sec
 *         2 -> ± 1000 deg/sec
 *         3 -> ± 2000 deg/sec
 */

static inline int __mpu_write_gyro_config(FAR struct mpu_dev_s *dev,
                                          uint8_t fs_sel)
{
  uint8_t val = TO_BITFIELD(GYRO_CONFIG__FS_SEL, fs_sel);
  return __mpu_write_reg(dev, GYRO_CONFIG, &val, sizeof(val));
}

/* __mpu_write_accel_config() :
 *
 * Sets the @afs_sel bit in ACCEL_CONFIG to the value provided. Per
 * the datasheet, the meaning of @afs_sel is as follows:
 *
 * ACCEL_CONFIG(0x1c) :   XA_ST YA_ST ZA_ST AFS_SEL1 AFS_SEL0 x  x  x
 *
 *    XA_ST, YA_ST, ZA_ST  :  self-test (unsupported in this driver)
 *         1 -> activate self-test on X, Y, and/or Z accelerometers
 *
 *    AFS_SEL[10] : full-scale range select
 *         0 -> ±  2 g
 *         1 -> ±  4 g
 *         2 -> ±  8 g
 *         3 -> ± 16 g
 */

static inline int __mpu_write_accel_config(FAR struct mpu_dev_s *dev,
                                           uint8_t afs_sel)
{
  uint8_t val = TO_BITFIELD(ACCEL_CONFIG__AFS_SEL, afs_sel);
  return __mpu_write_reg(dev, ACCEL_CONFIG, &val, sizeof(val));
}

/* CONFIG (0x1a) :   x   x   EXT_SYNC_SET[2..0] DLPF_CFG[2..0]
 *
 *    EXT_SYNC_SET  : frame sync bit position
 *    DLPF_CFG      : digital low-pass filter bandwidth
 * (see datasheet, it's ... complicated)
 */

static inline int __mpu_write_config(FAR struct mpu_dev_s *dev,
                                     uint8_t ext_sync_set, uint8_t dlpf_cfg)
{
  uint8_t val = TO_BITFIELD(CONFIG__EXT_SYNC_SET, ext_sync_set) |
                TO_BITFIELD(CONFIG__DLPF_CFG, dlpf_cfg);
  return __mpu_write_reg(dev, CONFIG, &val, sizeof(val));
}

/* Resets the mpu60x0, sets it to a default configuration. */

static int mpu_reset(FAR struct mpu_dev_s *dev)
{
  int ret;
#ifdef CONFIG_MPU60X0_SPI
  if (dev->config.spi == NULL)
    {
      return -EINVAL;
    }
#else
  if (dev->config.i2c == NULL)
    {
      return -EINVAL;
    }
#endif

  nxmutex_lock(&dev->lock);

  /* Awaken chip, issue hardware reset */

  ret = __mpu_write_pwr_mgmt_1(dev, PWR_MGMT_1__DEVICE_RESET);
  if (ret != OK)
    {
      nxmutex_unlock(&dev->lock);
      snerr("Could not find mpu60x0!\n");
      return ret;
    }

  /* Wait for reset cycle to finish (note: per the datasheet, we don't need
   * to hold NSS for this)
   */

  do
    {
      nxsig_usleep(50000);            /* usecs (arbitrary) */
    }
  while (__mpu_read_pwr_mgmt_1(dev) & PWR_MGMT_1__DEVICE_RESET);

  /* Reset signal paths */

  __mpu_write_signal_path_reset(dev, SIGNAL_PATH_RESET__ALL_RESET);
  nxsig_usleep(2000);

  /* Disable SLEEP, use PLL with z-axis clock source */

  __mpu_write_pwr_mgmt_1(dev, 3);
  nxsig_usleep(2000);

  /* Disable i2c if we're on spi. */

#ifdef CONFIG_MPU60X0_SPI
  if (dev->config.spi)
    {
      __mpu_write_user_ctrl(dev, USER_CTRL__I2C_IF_DIS);
    }
#endif

  /* Disable low-power mode, enable all gyros and accelerometers */

  __mpu_write_pwr_mgmt_2(dev, 0);

  /* default No FSYNC, set accel LPF at 184 Hz, gyro LPF at 188 Hz in
   * menuconfig
   */

  __mpu_write_config(dev, CONFIG_MPU60X0_EXT_SYNC_SET,
                     CONFIG_MPU60X0_DLPF_CFG);

  /* default ± 1000 deg/sec in menuconfig */

  __mpu_write_gyro_config(dev, CONFIG_MPU60X0_GYRO_FS_SEL);

  /* default ± 8g in menuconfig */

  __mpu_write_accel_config(dev, CONFIG_MPU60X0_ACCEL_AFS_SEL);

  /* clear INT on any read (we aren't using that pin right now) */

  __mpu_write_int_pin_cfg(dev, INT_PIN_CFG__INT_RD_CLEAR);

  nxmutex_unlock(&dev->lock);
  return 0;
}

/****************************************************************************
 * Name: mpu_open
 *
 * Note: we don't deal with multiple users trying to access this interface at
 * the same time. Until further notice, don't do that.
 *
 * And no, it's not as simple as just prohibiting concurrent opens or
 * reads with a mutex: there are legit reasons for truy concurrent
 * access, but they must be treated carefully in this interface lest a
 * partial reader end up with a mixture of old and new samples. This
 * will make some users unhappy.
 *
 ****************************************************************************/

static int mpu_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  /* Reset the register cache */

  nxmutex_lock(&dev->lock);
  dev->bufpos = 0;
  nxmutex_unlock(&dev->lock);

  return 0;
}

/****************************************************************************
 * Name: mpu_close
 ****************************************************************************/

static int mpu_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  /* Reset (clear) the register cache. */

  nxmutex_lock(&dev->lock);
  dev->bufpos = 0;
  nxmutex_unlock(&dev->lock);

  return 0;
}

/****************************************************************************
 * Name: mpu_read
 *
 * Returns a snapshot of the accelerometer, temperature, and gyro registers.
 *
 * Note: the chip uses traditional, twos-complement notation, i.e. "0"
 * is encoded as 0, and full-scale-negative is 0x8000, and
 * full-scale-positive is 0x7fff. If we read the registers
 * sequentially and directly into memory (as we do), the measurements
 * from each sensor are captured as big endian words.
 *
 * In contrast, ASN.1 maps "0" to 0x8000, full-scale-negative to 0,
 * and full-scale-positive to 0xffff. So if we want to send in a
 * format that an ASN.1 PER-decoder would recognize, must:
 *
 *   1. Treat the register data/measurements as unsigned,
 *   2. Add 0x8000 to each measurement, and then,
 *   3. Send each word in big-endian order.
 *
 * The result of the above will be something you could neatly describe
 * like this (confirmed with asn1scc):
 *
 *    Sint16  ::= INTEGER(-32768..32767)
 *
 *    Mpu60x0Sample ::= SEQUENCE
 *    {
 *      accel-X  Sint16,
 *      accel-Y  Sint16,
 *      accel-Z  Sint16,
 *      temp     Sint16,
 *      gyro-X   Sint16,
 *      gyro-Y   Sint16,
 *      gyro-Z   Sint16
 *    }
 *
 ****************************************************************************/

static ssize_t mpu_read(FAR struct file *filep, FAR char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;
  size_t send_len = 0;

  nxmutex_lock(&dev->lock);

  /* Populate the register cache if it seems empty. */

  if (!dev->bufpos)
    {
      __mpu_read_imu(dev, &dev->buf);
    }

  /* Send the lesser of: available bytes, or amount requested. */

  send_len = sizeof(dev->buf) - dev->bufpos;
  if (send_len > len)
    {
      send_len = len;
    }

  if (send_len)
    {
      memcpy(buf, ((uint8_t *)&dev->buf) + dev->bufpos, send_len);
    }

  /* Move the cursor, to mark them as sent. */

  dev->bufpos += send_len;

  /* If we've sent the last byte, reset the buffer. */

  if (dev->bufpos >= sizeof(dev->buf))
    {
      dev->bufpos = 0;
    }

  nxmutex_unlock(&dev->lock);
  return send_len;
}

/****************************************************************************
 * Name: mpu_write
 ****************************************************************************/

static ssize_t mpu_write(FAR struct file *filep, FAR const char *buf,
                         size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  UNUSED(inode);
  UNUSED(dev);
  snerr("ERROR: %p %p %d\n", inode, dev, len);

  return len;
}

/****************************************************************************
 * Name: mpu60x0_seek
 ****************************************************************************/

static off_t mpu_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  UNUSED(inode);
  UNUSED(dev);

  snerr("ERROR: %p %p\n", inode, dev);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu60x0_register
 *
 * Description:
 *   Registers the mpu60x0 interface as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the interface to register. E.g., "/dev/imu0"
 *   spi      - SPI interface for chip communications
 *   config   - Configuration information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mpu60x0_register(FAR const char *path, FAR struct mpu_config_s *config)
{
  FAR struct mpu_dev_s *priv;
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the device structure. */

  priv = (FAR struct mpu_dev_s *)kmm_malloc(sizeof(struct mpu_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate mpu60x0 device instance\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(*priv));
  nxmutex_init(&priv->lock);

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  priv->config = *config;

  /* Reset the chip, to give it an initial configuration. */

  ret = mpu_reset(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure mpu60x0: %d\n", ret);

      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return ret;
    }

  /* Register the device node. */

  ret = register_driver(path, &g_mpu_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register mpu60x0 interface: %d\n", ret);

      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return ret;
    }

  return OK;
}
