/****************************************************************************
 * drivers/sensors/mpu9250.c
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
#include <math.h>
#include <stdio.h>
#include <debug.h>
#include <string.h>
#include <limits.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>

#ifdef CONFIG_MPU9250_SPI
#include <nuttx/spi/spi.h>
#else
#include <nuttx/i2c/i2c_master.h>
#endif
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/mpu9250.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  MPU9250_AKM_DEV_ID                 0x48  /* Magnetometer device ID */
#define  MIN(x, y)         (x) > (y) ? (y) : (x)

/* 16bit mode: 0.15uTesla/LSB, 100 uTesla == 1 Gauss */

#define MAG_RAW_TO_GAUSS    (0.15f / 100.0f)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum mpu9250_idx_e
{
  MPU9250_ACCEL_IDX = 0,
  MPU9250_GYRO_IDX,
  MPU9250_MAG_IDX,
  MPU9250_MAX_IDX
};

enum mpu9250_regaddr_e
{
  SELF_TEST_G_X = 0x00,
  SELF_TEST_G_Y = 0x01,
  SELF_TEST_G_Z = 0x02,

  SELF_TEST_A_X = 0x0d,
  SELF_TEST_A_Y = 0x0e,
  SELF_TEST_A_Z = 0x0f,

  XG_OFFSETH = 0x13,
  XG_OFFSETL = 0x14,
  YG_OFFSETH = 0x15,
  YG_OFFSETL = 0x16,
  ZG_OFFSETH = 0x17,
  ZG_OFFSETL = 0x18,

  SMPLRT_DIV = 0x19,

  /* _SHIFT : number of empty bits to the right of the field
   * _WIDTH : width of the field, in bits
   *
   * single-bit fields don't have _SHIFT or _mask
   */

  MPU9250_CONFIG = 0x1a,
  CONFIG_EXT_SYNC_SET_SHIFT = 3,
  CONFIG_EXT_SYNC_SET_WIDTH = 3,
  CONFIG_DLPF_CFG_SHIFT = 0,
  CONFIG_DLPF_CFG_WIDTH = 3,

  GYRO_CONFIG = 0x1b,
  GYRO_CONFIG_XG_ST = BIT(7),
  GYRO_CONFIG_YG_ST = BIT(6),
  GYRO_CONFIG_ZG_ST = BIT(5),
  GYRO_CONFIG_FS_SEL_SHIFT = 3,
  GYRO_CONFIG_FS_SEL_WIDTH = 2,

  ACCEL_CONFIG = 0x1c,
  ACCEL_CONFIG_XA_ST = BIT(7),
  ACCEL_CONFIG_YA_ST = BIT(6),
  ACCEL_CONFIG_ZA_ST = BIT(5),
  ACCEL_CONFIG_AFS_SEL_SHIFT = 3,
  ACCEL_CONFIG_AFS_SEL_WIDTH = 2,

  ACCEL_CONFIG2 = 0x1d,
  ACCEL_CONFIG2_FCHOICE_B = BIT(3),
  CONFIG2_A_DLPF_CFG_SHIFT = 0,
  CONFIG2_A_DLPF_CFG_WIDTH = 3,
  LPACCEL_ODR = 0x1e,
  WOM_THR = 0x1f,

  FIFO_EN = 0x23,
  BITS_FIFO_ENABLE_TEMP_OUT = BIT(7),
  BITS_FIFO_ENABLE_GYRO_XOUT = BIT(6),
  BITS_FIFO_ENABLE_GYRO_YOUT = BIT(5),
  BITS_FIFO_ENABLE_GYRO_ZOUT = BIT(4),
  BITS_FIFO_ENABLE_ACCEL = BIT(3),
  BITS_FIFO_ENABLE_SLV2 = BIT(2),
  BITS_FIFO_ENABLE_SLV1 = BIT(1),
  BITS_FIFO_ENABLE_SLV0 = BIT(0),

  I2C_MST_CTRL = 0x24,
  I2C_SLV0_ADDR = 0x25,
  I2C_SLV0_REG = 0x26,
  I2C_SLV0_CTRL = 0x27,
  BITS_I2C_SLV0_EN = BIT(7),
  BITS_I2C_SLV0_READ_8BYTES = BIT(3),
  I2C_SLV1_ADDR = 0x28,
  I2C_SLV1_REG = 0x29,
  I2C_SLV1_CTRL = 0x2a,
  BITS_I2C_SLV1_EN = BIT(7),
  I2C_SLV2_ADDR = 0x2b,
  I2C_SLV2_REG = 0x2c,
  I2C_SLV2_CTRL = 0x2d,
  BITS_I2C_SLV2_EN = BIT(7),
  I2C_SLV3_ADDR = 0x2e,
  I2C_SLV3_REG = 0x2f,
  I2C_SLV3_CTRL = 0x30,
  I2C_SLV4_ADDR = 0x31,
  I2C_SLV4_REG = 0x32,
  I2C_SLV4_DO = 0x33,
  I2C_SLV4_CTRL = 0x34,
  BITS_I2C_SLV4_EN = BIT(7),
  BITS_I2C_SLV4_DONE = BIT(6),
  I2C_SLV4_DI = 0x35,         /* RO */
  I2C_MST_STATUS = 0x36,      /* RO */

  INT_PIN_CFG = 0x37,
  INT_PIN_CFG_INT_ACTL = BIT(7),
  INT_PIN_CFG_INT_OPEN = BIT(6),
  INT_PIN_CFG_LATCH_INT_EN = BIT(5),
  INT_PIN_CFG_INT_RD_CLEAR = BIT(4),
  INT_PIN_CFG_FSYNC_INT_LEVEL = BIT(3),
  INT_PIN_CFG_FSYNC_INT_EN = BIT(2),
  INT_PIN_CFG_I2C_BYPASS_EN = BIT(1),

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
  BITS_SLV4_DLY_EN = BIT(4),
  BITS_SLV3_DLY_EN = BIT(3),
  BITS_SLV2_DLY_EN = BIT(2),
  BITS_SLV1_DLY_EN = BIT(1),
  BITS_SLV0_DLY_EN = BIT(0),

  SIGNAL_PATH_RESET = 0x68,
  SIGNAL_PATH_RESET_GYRO_RESET = BIT(2),
  SIGNAL_PATH_RESET_ACCEL_RESET = BIT(1),
  SIGNAL_PATH_RESET_TEMP_RESET = BIT(0),
  SIGNAL_PATH_RESET_ALL_RESET = BIT(3) - 1,

  MOT_DETECT_CTRL = 0x69,

  USER_CTRL = 0x6a,
  USER_CTRL_FIFO_EN = BIT(6),
  USER_CTRL_I2C_MST_EN = BIT(5),
  USER_CTRL_I2C_IF_DIS = BIT(4),
  USER_CTRL_FIFO_RST = BIT(2),
  USER_CTRL_I2C_MST_RST = BIT(1),
  USER_CTRL_SIG_COND_RST = BIT(0),

  PWR_MGMT_1 = 0x6b,          /* Reset: 0x40 */
  PWR_MGMT_1_DEVICE_RESET = BIT(7),
  PWR_MGMT_1_SLEEP = BIT(6),
  PWR_MGMT_1_CYCLE = BIT(5),
  PWR_MGMT_1_GYRO_STANDBY = BIT(4),
  PWR_MGMT_1_PD_PTAT = BIT(3),
  PWR_MGMT_1_CLK_SEL_SHIFT = 0,
  PWR_MGMT_1_CLK_SEL_WIDTH = 3,

  PWR_MGMT_2 = 0x6c,
  FIFO_COUNTH = 0x72,
  FIFO_COUNTL = 0x73,
  FIFO_R_W = 0x74,
  WHO_AM_I = 0x75,            /* RO reset: 0x68 */

  XA_OFFSETH = 0x77,
  XA_OFFSETL = 0x78,
  YA_OFFSETH = 0x7a,
  YA_OFFSETL = 0x7b,
  ZA_OFFSETH = 0x7d,
  ZA_OFFSETL = 0x7e,

  /* Magnetometer device address */

  MPU9250_AK8963_I2C_ADDR = 0x0c,
  MPU9250_AK8963_I2C_READ = 0x80,
  MPU9250_AK8963_I2C_WRITE = 0x00,

  /* AK8963 Magnetometer Register Addresses */

  MPU9250_MAG_REG_WIA = 0x00,
  MPU9250_MAG_REG_ST1 = 0x02,
  MPU9250_MAG_REG_DATA = 0x03,
  MPU9250_MAG_REG_HXL = 0x03,
  MPU9250_MAG_REG_ST2 = 0x09,
  MPU9250_MAG_REG_CNTL1 = 0x0a,
  MPU9250_MAG_REG_CNTL2 = 0x0b,
  MPU9250_MAG_REG_ASAX = 0x10,
  MPU9250_MAG_REG_ASAY = 0x11,
  MPU9250_MAG_REG_ASAZ = 0x12,

  /* Bit definitions for the magnetometer registers */

  BIT_MAG_CNTL1_MODE_POWER_DOWN = 0x0,
  BIT_MAG_CNTL1_MODE_SINGLE_MEASURE_MODE = 0x1,
  BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_1 = 0x2,
  BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_2 = 0x6,
  BIT_MAG_CNTL1_FUSE_ROM_ACCESS_MODE = 0xf,
  BIT_MAG_CNTL1_16_BITS = 0x10,
  BIT_MAG_HOFL = 0x08,

  BIT_MAG_CNTL2_SOFT_RESET = 0x01,
};

/* Describes the mpu9250 sensor register file. This structure reflects
 * the underlying hardware, so don't change it!
 */

#pragma pack(push, 1)
struct sensor_data_s
{
  uint16_t x_accel;
  uint16_t y_accel;
  uint16_t z_accel;
  uint16_t temp;
  uint16_t x_gyro;
  uint16_t y_gyro;
  uint16_t z_gyro;
  char mag_st1;  /* 14 mag ST1 (1B) */
  int16_t x_mag; /* 15-16 (2B) */
  int16_t y_mag; /* 17-18 (2B) */
  int16_t z_mag; /* 19-20 (2B) */
  char mag_st2;  /* 21 mag ST2 (1B) */
};
#pragma pack(pop)

struct mpu9250_sensor_s
{
  struct sensor_lowerhalf_s lower;
  uint64_t                  last_update;
  bool                      enabled;
  float                     scale;
  float                     adj[3];
  unsigned long             interval;
  FAR void  *dev; /* The pointer to common device data of mpu9250 */
};

/* Used by the driver to manage the device */

struct mpu9250_dev_s
{
  struct mpu9250_sensor_s priv[MPU9250_MAX_IDX];
  struct mpu9250_config_s config; /* board-specific information */
  sem_t                   run;    /* Locks sensor thread */
  mutex_t                 lock;   /* mutex for this structure */
};

/****************************************************************************
 * Private Function Function Prototypes
 ****************************************************************************/

/* Sensor methods */

static int mpu9250_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us);
static int mpu9250_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
static int mpu9250_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg);

static inline int mpu9250_write_gyro_range(FAR struct mpu9250_dev_s *dev,
                                           uint8_t fs_sel);
static void mpu9250_gyro_scale(FAR struct mpu9250_sensor_s *priv,
                               enum gyro_config_bit scale);
static inline int mpu9250_write_accel_range(FAR struct mpu9250_dev_s *dev,
                                            uint8_t afs_sel);
static void mpu9250_accel_scale(FAR struct mpu9250_sensor_s *priv,
                                enum accel_config_bit scale);
static int ak8963_initialize(FAR struct mpu9250_dev_s *dev,
                             int measure_freq);
static int read_ak8963_reg(FAR struct mpu9250_dev_s *dev,
                           enum mpu9250_regaddr_e reg, uint8_t *val);
static int write_ak8963_reg(FAR struct mpu9250_dev_s *dev,
                            enum mpu9250_regaddr_e  reg, uint8_t val);
static int get_mag_adjustment(FAR struct mpu9250_dev_s *dev);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_mpu9250_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  mpu9250_activate,     /* activate */
  mpu9250_set_interval, /* set_interval */
  NULL,                 /* batch */
  NULL,                 /* fetch */
  NULL,                 /* selftest */
  NULL,                 /* set_calibvalue */
  NULL,                 /* calibrate */
  mpu9250_control       /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu9250_activate
 *
 * Description: Activate the sensor.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int mpu9250_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct mpu9250_sensor_s *priv = (FAR struct mpu9250_sensor_s *)lower;
  FAR struct mpu9250_dev_s *dev = (FAR struct mpu9250_dev_s *)(priv->dev);
  bool start_thread = false;

  if (enable)
    {
      if (!priv->enabled)
        {
          start_thread = true;
          priv->last_update = sensor_get_timestamp();
        }
    }

  priv->enabled = enable;

  if (start_thread)
    {
      /* Wake up the thread */

      nxsem_post(&dev->run);
    }

  return OK;
}

/****************************************************************************
 * Name: mpu9250_set_interval
 *
 * Description: Set data output interval of sensor.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int mpu9250_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *interval)
{
  FAR struct mpu9250_sensor_s *priv = (FAR struct mpu9250_sensor_s *)lower;

  priv->interval = *interval;

  return 0;
}

/****************************************************************************
 * Name: mpu9250_control
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int mpu9250_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg)
{
  FAR struct mpu9250_sensor_s *priv = (FAR struct mpu9250_sensor_s *)lower;
  FAR struct mpu9250_dev_s *dev = (FAR struct mpu9250_dev_s *)(priv->dev);
  int ret;

  switch (cmd)
    {
      /* Set full scale command */

      case SNIOC_SET_SCALE_XL:
        {
          switch (priv->lower.type)
            {
              /* Set gyroscope full scale */

              case SENSOR_TYPE_GYROSCOPE:
                {
                  ret = mpu9250_write_gyro_range(dev, (uint8_t)arg);
                  mpu9250_gyro_scale(priv, (enum gyro_config_bit)arg);
                }
                break;

              /* Set accelerometer full scale */

              case SENSOR_TYPE_ACCELEROMETER:
                {
                  ret = mpu9250_write_accel_range(dev, (uint8_t)arg);
                  mpu9250_accel_scale(priv, (enum accel_config_bit)arg);
                }
                break;

              default:
                snerr("ERROR: Unrecognized type: %d\n", priv->lower.type);
                ret = -ENOTTY;
                break;
            }
        }
        break;

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: mpu9250_accel_scale
 *
 * Description: Set scale of accelerometer.
 *
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 16384 LSB/mg
 * 1       | +/- 4g           | 8192 LSB/mg
 * 2       | +/- 8g           | 4096 LSB/mg
 * 3       | +/- 16g          | 2048 LSB/mg
 ****************************************************************************/

static void mpu9250_accel_scale(FAR struct mpu9250_sensor_s *priv,
                                enum accel_config_bit scale)
{
  switch (scale)
    {
      case ACCEL_FS_SEL_2G:
        priv->scale = CONSTANTS_ONE_G / 16384.f;
        break;

      case ACCEL_FS_SEL_4G:
        priv->scale = CONSTANTS_ONE_G / 8192.f;
        break;

      case ACCEL_FS_SEL_8G:
        priv->scale = CONSTANTS_ONE_G / 4096.f;
        break;

      case ACCEL_FS_SEL_16G:
        priv->scale = CONSTANTS_ONE_G / 2048.f;
        break;
      default:
        break;
    }
}

/****************************************************************************
 * Name: mpu9250_gyro_scale
 *
 * Description: Set scale of accelerometer.
 *
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 ****************************************************************************/

static void mpu9250_gyro_scale(FAR struct mpu9250_sensor_s *priv,
                               enum gyro_config_bit scale)
{
  switch (scale)
    {
      case GYRO_FS_SEL_250_DPS:
        priv->scale = (M_PI / 180.0f) * 250.f / 32768.f;
        break;

      case GYRO_FS_SEL_500_DPS:
        priv->scale = (M_PI / 180.0f) * 500.f / 32768.f;
        break;

      case GYRO_FS_SEL_1000_DPS:
        priv->scale = (M_PI / 180.0f) * 1000.f / 32768.f;
        break;

      case GYRO_FS_SEL_2000_DPS:
        priv->scale = (M_PI / 180.0f) * 2000.f / 32768.f;
        break;
      default:
        break;
    }
}

/* NOTE :
 *
 * In all of the following code, functions named with a double leading
 * underscore '_' must be invoked ONLY if the mpu9250_dev_s lock is
 * already held. Failure to do this might cause the transaction to get
 * interrupted, which will likely confuse the data you get back.
 *
 * The mpu9250_dev_s lock is NOT the same thing as, i.e. the SPI master
 * interface lock: the latter protects the bus interface hardware
 * (which may have other SPI devices attached), the former protects
 * the chip and its associated data.
 */

#ifdef CONFIG_MPU9250_SPI
/* mpu9250_read_reg(), but for spi-connected devices. See that function
 * for documentation.
 */

static int mpu9250_read_reg_spi(FAR struct mpu9250_dev_s *dev,
                                enum mpu9250_regaddr_e reg_addr,
                                FAR uint8_t *buf, uint8_t len)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;
  int ret;

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

/* mpu9250_write_reg(), but for SPI connections. */

static int mpu9250_write_reg_spi(FAR struct mpu9250_dev_s *dev,
                                 enum mpu9250_regaddr_e reg_addr,
                                 FAR const uint8_t * buf, uint8_t len)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;
  int ret;

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

/* mpu9250_read_reg(), but for i2c-connected devices. */

static int mpu9250_read_reg_i2c(FAR struct mpu9250_dev_s *dev,
                                uint8_t reg_addr,
                                FAR uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_MPU9250_I2C_FREQ;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_MPU9250_I2C_FREQ;
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

/* mpu9250_write_reg(), but for I2C connections. */

static int mpu9250_write_reg_i2c(FAR struct mpu9250_dev_s *dev,
                                 uint8_t reg_addr,
                                 FAR const uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_MPU9250_I2C_FREQ;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;
  msg[1].frequency = CONFIG_MPU9250_I2C_FREQ;
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
#endif /* CONFIG_MPU9250_SPI */

/* mpu9250_read_reg()
 *
 * Reads a block of @len byte-wide registers, starting at @reg_addr,
 * from the device connected to @dev. Bytes are returned in @buf,
 * which must have a capacity of at least @len bytes.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes read, or a negative errno.
 */

static inline int mpu9250_read_reg(FAR struct mpu9250_dev_s *dev,
                                   enum mpu9250_regaddr_e reg_addr,
                                   FAR uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_MPU9250_SPI
  /* If we're wired to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return mpu9250_read_reg_spi(dev, reg_addr, buf, len);
    }
#else
  /* If we're wired to I2C, use that function. */

  if (dev->config.i2c != NULL)
    {
      return mpu9250_read_reg_i2c(dev, reg_addr, buf, len);
    }
#endif

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* mpu9250_write_reg()
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

static inline int mpu9250_write_reg(FAR struct mpu9250_dev_s *dev,
                                    enum mpu9250_regaddr_e reg_addr,
                                    FAR const uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_MPU9250_SPI
  /* If we're connected to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return mpu9250_write_reg_spi(dev, reg_addr, buf, len);
    }
#else
  if (dev->config.i2c != NULL)
    {
      return mpu9250_write_reg_i2c(dev, reg_addr, buf, len);
    }
#endif

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/****************************************************************************
 * mpu9250_write_reg_verify
 *
 * Description:
 *   write a 8-bit register value by address
 *   read back a 8-bit register value by address to verify
 *
 * Returns number of bytes read, or a negative errno.
 ****************************************************************************/

static int mpu9250_write_reg_verify(FAR struct mpu9250_dev_s *dev,
                                    enum mpu9250_regaddr_e reg_addr,
                                    uint8_t val, uint8_t mask)
{
  int ret;
  uint8_t b;
  int retry = 5;
  bool err;

  while (retry)
    {
      err = false;
      --retry;
      ret = mpu9250_write_reg(dev, reg_addr, &val, sizeof(val));
      if (ret < 0)
        {
          err = true;
          continue;
        }

      ret = mpu9250_read_reg(dev, reg_addr, &b, sizeof(b));
      if (ret < 0)
        {
          err = true;
          continue;
        }

      if ((b & mask) != val)
        {
          continue;
        }
      else
        {
          return 0;
        }
    }

  if (err)
    {
      snerr("write_reg_verify failed at reg %d. Error %d.", reg_addr, ret);
    }
  else
    {
      snerr("write_reg_verify failed at reg %d. %d!=%d", reg_addr, val, b);
    }

  return ret;
}

/****************************************************************************
 * mpu9250_modify_reg()
 *
 * Description:
 *   Modify a 8-bit register value by address
 *   mpu9250_modify_reg(d,v,m,a) defined as:
 *   mpu9250_write_reg(d,(mpu9250_read_reg(d,a) & ~(m)) | ((v) & (m)), (a))
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes written, or a negative errno.
 ****************************************************************************/

static inline int mpu9250_modify_reg(FAR struct mpu9250_dev_s *dev,
                                    enum mpu9250_regaddr_e reg_addr,
                                    uint8_t clearbits, uint8_t setbits)
{
  uint8_t buf = 0xff;

  mpu9250_read_reg(dev, reg_addr, &buf, sizeof(buf));

  buf = (buf & ~clearbits) | (setbits & clearbits);

  return mpu9250_write_reg(dev, reg_addr, &buf, sizeof(buf));
}

/****************************************************************************
 * mpu9250_read_imu()
 *
 * Reads the whole IMU data file from @dev in one uninterrupted pass,
 * placing the sampled values into @buf. This function is the only way
 * to guarantee that the measured values are sampled as closely-spaced
 * in time as the hardware permits, which is almost always what you
 * want.
 ****************************************************************************/

static inline int mpu9250_read_imu(FAR struct mpu9250_dev_s *dev,
                                   FAR struct sensor_data_s *buf)
{
  return mpu9250_read_reg(dev, ACCEL_XOUT_H, (uint8_t *) buf, sizeof(*buf));
}

/* mpu9250_read_pwr_mgmt_1()
 *
 * Returns the value of the PWR_MGMT_1 register from @dev.
 */

static inline uint8_t mpu9250_read_pwr_mgmt_1(FAR struct mpu9250_dev_s *dev)
{
  uint8_t buf = 0xff;

  mpu9250_read_reg(dev, PWR_MGMT_1, &buf, sizeof(buf));
  return buf;
}

static inline int mpu9250_write_signal_reset(FAR struct mpu9250_dev_s *dev,
                                                  uint8_t val)
{
  return mpu9250_write_reg(dev, SIGNAL_PATH_RESET, &val, sizeof(val));
}

static inline int mpu9250_write_int_pin_cfg(FAR struct mpu9250_dev_s *dev,
                                            uint8_t val)
{
  return mpu9250_write_reg(dev, INT_PIN_CFG, &val, sizeof(val));
}

static inline int mpu9250_write_pwr_mgmt_1(FAR struct mpu9250_dev_s *dev,
                                           uint8_t val)
{
  return mpu9250_write_reg(dev, PWR_MGMT_1, &val, sizeof(val));
}

static inline int mpu9250_write_pwr_mgmt_2(FAR struct mpu9250_dev_s *dev,
                                           uint8_t val)
{
  return mpu9250_write_reg(dev, PWR_MGMT_2, &val, sizeof(val));
}

static inline int mpu9250_write_user_ctrl(FAR struct mpu9250_dev_s *dev,
                                          uint8_t val)
{
  return mpu9250_write_reg(dev, USER_CTRL, &val, sizeof(val));
}

static inline int mpu9250_write_fifo_en(FAR struct mpu9250_dev_s *dev,
                                        uint8_t val)
{
  return mpu9250_write_reg(dev, FIFO_EN, &val, sizeof(val));
}

/****************************************************************************
 * mpu9250_write_gyro_range() :
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
 ****************************************************************************/

static inline int mpu9250_write_gyro_range(FAR struct mpu9250_dev_s *dev,
                                           uint8_t fs_sel)
{
  uint8_t val = TO_BITFIELD(GYRO_CONFIG_FS_SEL, fs_sel);
  return mpu9250_write_reg(dev, GYRO_CONFIG, &val, sizeof(val));
}

/****************************************************************************
 * mpu9250_write_accel_range() :
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
 ****************************************************************************/

static inline int mpu9250_write_accel_range(FAR struct mpu9250_dev_s *dev,
                                            uint8_t afs_sel)
{
  uint8_t val = TO_BITFIELD(ACCEL_CONFIG_AFS_SEL, afs_sel);
  return mpu9250_write_reg(dev, ACCEL_CONFIG, &val, sizeof(val));
}

/****************************************************************************
 * CONFIG (0x1a) :   x   x   EXT_SYNC_SET[2..0] DLPF_CFG[2..0]
 *
 *    EXT_SYNC_SET  : frame sync bit position
 *    DLPF_CFG      : digital low-pass filter bandwidth
 ****************************************************************************/

static inline int mpu9250_write_config(FAR struct mpu9250_dev_s *dev,
                                     uint8_t ext_sync_set, uint8_t dlpf_cfg)
{
  uint8_t val = TO_BITFIELD(CONFIG_EXT_SYNC_SET, ext_sync_set) |
                TO_BITFIELD(CONFIG_DLPF_CFG, dlpf_cfg);
  return mpu9250_write_reg(dev, MPU9250_CONFIG, &val, sizeof(val));
}

/****************************************************************************
 * CONFIG2 (0x1d) :   x   x   accel_fchoice_b  A_DLPF_CFG[2..0]
 *
 *    accel_fchoice_b  : he inverted version of accel_fchoice
 *    A_DLPF_CFG      : Accelerometer low pass filter setting
 ****************************************************************************/

static inline int mpu9250_write_config2(FAR struct mpu9250_dev_s *dev,
                                        uint8_t acce_fchoice_b,
                                        uint8_t a_dlpf_cfg)
{
  uint8_t val = acce_fchoice_b | TO_BITFIELD(CONFIG2_A_DLPF_CFG, a_dlpf_cfg);
  return mpu9250_write_reg(dev, ACCEL_CONFIG2, &val, sizeof(val));
}

/****************************************************************************
 * Name: mpu9250_initialize
 *
 * Description: Initialize IMU and AK8963 magnetometer inside the MPU9250
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int mpu9250_initialize(FAR struct mpu9250_dev_s *dev)
{
  int ret = OK;

#ifdef CONFIG_MPU9250_SPI
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

  ret = mpu9250_write_pwr_mgmt_1(dev, PWR_MGMT_1_DEVICE_RESET);
  if (ret < 0)
    {
      snerr("mpu9250 write_pwr_mgmt_1 error!\n");
      goto errout;
    }

  /* Wait for reset cycle to finish (note: per the datasheet, we don't need
   * to hold NSS for this)
   */

  do
    {
      nxsig_usleep(50000);            /* usecs (arbitrary) */
    }
  while (mpu9250_read_pwr_mgmt_1(dev) & PWR_MGMT_1_DEVICE_RESET);

  /* Reset signal paths */

  ret = mpu9250_write_signal_reset(dev, SIGNAL_PATH_RESET_ALL_RESET);
  if (ret < 0)
    {
      snerr("mpu9250 write_signal_path_reset error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Disable SLEEP, use PLL with z-axis clock source */

  ret = mpu9250_write_pwr_mgmt_1(dev, 3);
  if (ret < 0)
    {
      snerr("mpu9250 write_pwr_mgmt_1 error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Disable low-power mode, enable all gyros and accelerometers */

  ret = mpu9250_write_pwr_mgmt_2(dev, 0);
  if (ret < 0)
    {
      snerr("mpu9250 write_pwr_mgmt_2 error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* clear first, and separate *_RST from *_EN */

  ret = mpu9250_write_user_ctrl(dev, 0);
  if (ret < 0)
    {
      snerr("mpu9250 write_user_ctrl error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  ret = mpu9250_write_fifo_en(dev, 0);
  if (ret < 0)
    {
      snerr("mpu9250 write_fifo_en error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Reset I2C Master module. */

  ret = mpu9250_write_user_ctrl(dev, USER_CTRL_FIFO_RST |
                                USER_CTRL_I2C_MST_RST |
                                USER_CTRL_SIG_COND_RST);
  if (ret < 0)
    {
      snerr("mpu9250 write_user_ctrl error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Disable i2c if we're on spi. */

#ifdef CONFIG_MPU9250_SPI
  if (dev->config.spi)
    {
      ret = mpu9250_write_user_ctrl(dev, USER_CTRL_I2C_MST_EN |
                                    USER_CTRL_I2C_IF_DIS);
    }
#else
  if (dev->config.i2c)
    {
      ret = mpu9250_write_user_ctrl(dev, USER_CTRL_I2C_MST_EN);
    }

#endif
  if (ret < 0)
    {
      snerr("mpu9250 write_user_ctrl error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* default No FSYNC, set accel LPF at 184 Hz, gyro LPF at 188 Hz in
   * menuconfig
   */

  ret = mpu9250_write_config(dev, CONFIG_MPU9250_EXT_SYNC_SET,
                             CONFIG_MPU9250_DLPF_CFG);
  if (ret < 0)
    {
      snerr("mpu9250 write_config error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* default ± 1000 deg/sec in menuconfig */

  ret = mpu9250_write_gyro_range(dev, CONFIG_MPU9250_GYRO_FS_SEL);
  if (ret < 0)
    {
      snerr("mpu9250 write_gyro_range error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* default ± 8g in menuconfig */

  ret = mpu9250_write_accel_range(dev, CONFIG_MPU9250_ACCEL_AFS_SEL);
  if (ret < 0)
    {
      snerr("mpu9250 write_accel_range error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Accelerometer low pass filter setting */

  ret = mpu9250_write_config2(dev, CONFIG_MPU9250_ACCEL_FCHOICE_B,
                              CONFIG_MPU9250_A_DLPF_CFG);
  if (ret < 0)
    {
      snerr("mpu9250 write_config2 error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* clear INT on any read (we aren't using that pin right now) */

  ret = mpu9250_write_int_pin_cfg(dev, INT_PIN_CFG_INT_RD_CLEAR);
  if (ret < 0)
    {
      snerr("mpu9250 write int pin cfg error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Initialize ak8963 magnetometer inside the IMU */

  ret = ak8963_initialize(dev, CONFIG_MPU9250_MEASURE_FREQ);
  if (ret < 0)
    {
      snerr("ak8963 initialize error!\n");
      goto errout;
    }

errout:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: ak8963_initialize
 *
 * Description: Initialize AK8963 magnetometer inside the IMU
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *   measure_freq - Data output_rate_in_hz
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ak8963_initialize(FAR struct mpu9250_dev_s *dev,
                             int measure_freq)
{
  uint8_t val = 0;
  int ret = 0;

  /* Enable the IMU extended I2C master. */

  ret = mpu9250_modify_reg(dev, USER_CTRL, USER_CTRL_I2C_MST_EN,
                           USER_CTRL_I2C_MST_EN);
  if (ret < 0)
    {
      snerr("MPU9250 I2C master bus enable failed: %d\n", ret);
      return ret;
    }

  /* Configure the IMU as an I2C master at 400 KHz. */

  val = 0x0d;
  ret = mpu9250_write_reg(dev, I2C_MST_CTRL, &val, sizeof(val));
  if (ret < 0)
    {
      snerr("MPU9250 I2C master bus clock set failed: %d\n", ret);
      return ret;
    }

  nxsig_usleep(1000);

  /* First set power-down mode */

  val = BIT_MAG_CNTL2_SOFT_RESET;
  ret = mpu9250_write_reg(dev, MPU9250_MAG_REG_CNTL2, &val, sizeof(val));
  if (ret < 0)
    {
      snerr("MPU9250 soft reset failed: %d\n", ret);

      /* Reset i2c master */

      mpu9250_modify_reg(dev, USER_CTRL, USER_CTRL_I2C_MST_RST,
                         USER_CTRL_I2C_MST_RST);
      return ret;
    }

  nxsig_usleep(1000);

  /* get mag version ID */

  ret = read_ak8963_reg(dev, MPU9250_MAG_REG_WIA, &val);
  if (ret < 0)
    {
      snerr("error reading mag whoami reg: %d", ret);
      return ret;
    }

  /* Detect mag presence by reading whoami register */

  if (val != MPU9250_AKM_DEV_ID)
    {
      snerr("wrong mag ID %u (expected %u)", val, MPU9250_AKM_DEV_ID);
      return -1;
    }

  /* Get mag calibraion data from Fuse ROM */

  ret = get_mag_adjustment(dev);
  if (ret < 0)
    {
      snerr("Unable to read mag sensitivity adjustment");
      return ret;
    }

  /* Power on and configure the mag in 100Hz measurement mode */

  ret = write_ak8963_reg(dev, MPU9250_MAG_REG_CNTL1, BIT_MAG_CNTL1_16_BITS |
                         BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_2);
  if (ret < 0)
    {
      snerr("Unable to configure the magnetometer mode.");
      return ret;
    }

  nxsig_usleep(1000);

  /* Slave 0 provides ST1, mag data, and ST2 data in a bulk transfer of
   * 8 bytes of data.  Use the address of ST1 in SLV0_REG as the beginning
   * register of the 8 byte bulk transfer.
   */

  val = MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_I2C_READ;
  ret = mpu9250_write_reg(dev, I2C_SLV0_ADDR, &val, sizeof(val));
  if (ret < 0)
    {
      snerr("MPU9250 I2C slave 0 address configuration failed.");
      return ret;
    }

  val = MPU9250_MAG_REG_ST1;
  ret = mpu9250_write_reg(dev, I2C_SLV0_REG, &val, sizeof(val));
  if (ret < 0)
    {
      snerr("MPU9250 I2C slave 0 register configuration failed.");
      return ret;
    }

  val = BITS_I2C_SLV0_EN | BITS_I2C_SLV0_READ_8BYTES;
  ret = mpu9250_write_reg(dev, I2C_SLV0_CTRL, &val, sizeof(val));
  if (ret < 0)
    {
      snerr("MPU9250 I2C slave 0 control configuration failed.");
      return ret;
    }

  nxsig_usleep(1000);

  /* Enable reading of the mag every n samples, dividing down from the
   * output data rate provided by the caller.
   */

  val = measure_freq / 100;
  ret = mpu9250_write_reg(dev, I2C_SLV4_CTRL, &val, sizeof(val));
  if (ret < 0)
    {
      snerr("Unable to configure I2C delay from given output data rate.");
      return ret;
    }

  nxsig_usleep(1000);

  /* Enable delayed I2C transfers for the mag on Slave 0 registers. */

  val = BITS_SLV0_DLY_EN;
  ret = mpu9250_write_reg(dev, I2C_MST_DELAY_CTRL, &val, sizeof(val));
  if (ret < 0)
    {
      snerr("Unable to enable the I2C delay on slave 0.");
      return ret;
    }

  nxsig_usleep(1000);

  return 0;
}

/****************************************************************************
 * Name: get_mag_adjustment
 *
 * Description: Get mag calibraion data from Fuse ROM
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int get_mag_adjustment(FAR struct mpu9250_dev_s *dev)
{
  FAR struct mpu9250_sensor_s *priv = &dev->priv[MPU9250_MAG_IDX];
  int ret = 0;
  uint8_t asa;
  int i = 0;

  /* First set power-down mode */

  ret = write_ak8963_reg(dev, MPU9250_MAG_REG_CNTL1,
                         BIT_MAG_CNTL1_MODE_POWER_DOWN);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(10000);

  /* Enable FUSE ROM, since the sensitivity adjustment data is stored in
   * compass registers 0x10, 0x11 and 0x12 which is only accessible in Fuse
   * access mode.
   */

  ret = write_ak8963_reg(dev, MPU9250_MAG_REG_CNTL1, BIT_MAG_CNTL1_16_BITS |
                         BIT_MAG_CNTL1_FUSE_ROM_ACCESS_MODE);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(10000);

  /* Get compass calibration register 0x10, 0x11, 0x12 store into context */

  for (i = 0; i < 3; ++i)
    {
      ret = read_ak8963_reg(dev, MPU9250_MAG_REG_ASAX + i, &asa);
      if (ret < 0)
        {
          return ret;
        }

      /* H_adj = H * ((ASA-128)*0.5/128 + 1)
       *       = H * ((ASA-128) / 256 + 1)
       * H is the raw compass reading.
       */

      priv->adj[i] = (((float)asa - 128.0f) / 256.0f) + 1.0f;
    }

  /* Leave in a power-down mode */

  ret = write_ak8963_reg(dev, MPU9250_MAG_REG_CNTL1,
                         BIT_MAG_CNTL1_MODE_POWER_DOWN);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(10000);

  return 0;
}

/****************************************************************************
 * Name: read_ak8963_reg
 *
 * Description: Read register of AK8963 by extended I2C bus
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *   reg  - register address of AK8963
 *   val  - Point to the data read from AK8963
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int read_ak8963_reg(FAR struct mpu9250_dev_s *dev,
                           enum mpu9250_regaddr_e reg, FAR uint8_t *val)
{
  int ret = 0;
  uint8_t b = 0;
  int loop_ctrl = 1000; /* wait up to 1000 * 1ms for completion */

  /* Read operation on the mag using the slave 4 registers */

  ret = mpu9250_write_reg_verify(dev, I2C_SLV4_ADDR, MPU9250_AK8963_I2C_ADDR
                                 | MPU9250_AK8963_I2C_READ, 0xff);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the mag register to read from */

  ret = mpu9250_write_reg_verify(dev, I2C_SLV4_REG, reg, 0xff);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the existing value of the SLV4 control register */

  ret = mpu9250_read_reg(dev, I2C_SLV4_CTRL, &b, sizeof(b));
  if (ret < 0)
    {
      return ret;
    }

  /* Set the I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting
   * other bits. Enable data transfer, a read transfer as configured above.
   */

  b |= BITS_I2C_SLV4_EN;

  /* Trigger the data transfer */

  ret = mpu9250_write_reg(dev, I2C_SLV4_CTRL, &b, sizeof(b));
  if (ret < 0)
    {
      return ret;
    }

  /* Continuously check I2C_MST_STATUS register value for the completion
   * of I2C transfer until timeout
   */

  do
    {
      nxsig_usleep(1000);
      ret = mpu9250_read_reg(dev, I2C_MST_STATUS,  &b, sizeof(b));
      if (ret < 0)
        {
          return ret;
        }
    }
  while (((b & BITS_I2C_SLV4_DONE) == 0x00) && (--loop_ctrl));

  if (loop_ctrl == 0)
    {
      snerr("I2C transfer timed out");
      return -1;
    }

  /* Read the value received from the mag */

  ret = mpu9250_read_reg(dev, I2C_SLV4_DI, val, sizeof(*val));
  if (ret < 0)
    {
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: write_ak8963_reg
 *
 * Description: Write register of AK8963 by extended I2C bus
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *   reg  - register address of AK8963
 *   val  - 8bit data will be write into AK8963 register
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int write_ak8963_reg(FAR struct mpu9250_dev_s *dev,
                            enum mpu9250_regaddr_e  reg, uint8_t val)
{
  int ret = 0;
  uint8_t b = 0;
  int loop_ctrl = 1000; /* wait up to 1000 * 1ms for completion */

  /* Configure a write operation to the mag using Slave 4 */

  ret = mpu9250_write_reg_verify(dev, I2C_SLV4_ADDR,
                                 MPU9250_AK8963_I2C_ADDR, 0xff);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the mag register address to write to using Slave 4 */

  ret = mpu9250_write_reg_verify(dev, I2C_SLV4_REG, reg, 0xff);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the value to write in the I2C_SLV4_DO register */

  ret = mpu9250_write_reg_verify(dev, I2C_SLV4_DO, val, 0xff);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the current value of the Slave 4 control register */

  ret = mpu9250_read_reg(dev, I2C_SLV4_CTRL, &b, sizeof(b));
  if (ret < 0)
    {
      return ret;
    }

  /* Set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
   * bits.
   */

  b |= BITS_I2C_SLV4_EN;

  /* Trigger the data transfer from the byte now stored in the SLV4_DO
   * register.
   */

  ret = mpu9250_write_reg(dev, I2C_SLV4_CTRL, &b, sizeof(b));
  if (ret < 0)
    {
      return ret;
    }

  /* Continuously check I2C_MST_STATUS regsiter value for the completion
   * of I2C transfer until timeout.
   */

  do
    {
      nxsig_usleep(1000);
      ret = mpu9250_read_reg(dev, I2C_MST_STATUS, &b, sizeof(b));
      if (ret < 0)
        {
          return ret;
        }
    }
  while (((b & BITS_I2C_SLV4_DONE) == 0x00) && (--loop_ctrl));

  if (loop_ctrl == 0)
    {
      snerr("I2C transfer to mag timed out");
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: swap16
 *
 * Description: swap H and L byte of a 16bit Data
 *
 * Parameter:
 *   val  - Big endian Data
 *
 * Return:
 *   do nothing if Big endian, swap H and L byte if little endian
 ****************************************************************************/

static uint16_t swap16(uint16_t val)
{
#ifdef CONFIG_ENDIAN_BIG
  return val;
#else
  return (val >> 8) | (val << 8);
#endif
}

/****************************************************************************
 * Name: mpu9250_accel_data
 *
 * Description: get and push accel data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to struct sensor_data_s
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static void mpu9250_accel_data(FAR struct mpu9250_sensor_s *priv,
                               FAR struct sensor_data_s *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_accel accel;

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  accel.timestamp = now;
  accel.x = (int16_t)swap16(buf->x_accel) * priv->scale;
  accel.y = (int16_t)swap16(buf->y_accel) * priv->scale;
  accel.z = (int16_t)swap16(buf->z_accel) * priv->scale;
  accel.temperature = swap16(buf->temp) / 333.87f + 21.0f;

  lower->push_event(lower->priv, &accel, sizeof(accel));
  sninfo("Accel: %.3fm/s^2 %.3fm/s^2 %.3fm/s^2, t:%.1f\n",
         accel.x, accel.y, accel.z, accel.temperature);
}

/****************************************************************************
 * Name: mpu9250_gyro_data
 *
 * Description: get and push gyro data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to struct sensor_data_s
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static void mpu9250_gyro_data(FAR struct mpu9250_sensor_s *priv,
                              FAR struct sensor_data_s *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_gyro gyro;

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  gyro.timestamp = now;
  gyro.x = (int16_t)swap16(buf->x_gyro) * priv->scale;
  gyro.y = (int16_t)swap16(buf->y_gyro) * priv->scale;
  gyro.z = (int16_t)swap16(buf->z_gyro) * priv->scale;
  gyro.temperature = swap16(buf->temp) / 333.87f + 21.0f;

  lower->push_event(lower->priv, &gyro, sizeof(gyro));
  sninfo("Gyro: %.3frad/s %.3frad/s %.3frad/s, t:%.1f\n",
          gyro.x, gyro.y, gyro.z, gyro.temperature);
}

/****************************************************************************
 * Name: mpu9250_mag_data
 *
 * Description: get and push magnetometer data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to struct sensor_data_s
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static void mpu9250_mag_data(FAR struct mpu9250_sensor_s *priv,
                             FAR struct sensor_data_s *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_mag mag;
  float temp_mag_x;

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  mag.timestamp = now;

  mag.x = (int16_t)buf->x_mag * priv->adj[0] * priv->scale;
  mag.y = (int16_t)buf->y_mag * priv->adj[1] * priv->scale;
  mag.z = (int16_t)buf->z_mag * priv->adj[2] * priv->scale;

  /* Swap magnetometer x and y axis, and invert z because internal mag in
   * MPU9250 has a different orientation.
   * Magnetometer X axis = Gyro and Accel Y axis
   * Magnetometer Y axis = Gyro and Accel X axis
   * Magnetometer Z axis = -Gyro and Accel Z axis
   */

  temp_mag_x = mag.x;
  mag.x = mag.y;
  mag.y = temp_mag_x;
  mag.z = -mag.z;

  mag.temperature = swap16(buf->temp) / 333.87f + 21.0f;

  lower->push_event(lower->priv, &mag, sizeof(mag));
  sninfo("Mag: %.3fuT %.3fuT %.3fuT, t:%.1f\n",
         mag.x, mag.y, mag.z, mag.temperature);
}

/****************************************************************************
 * Name: mpu9250_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 ****************************************************************************/

static int mpu9250_thread(int argc, FAR char **argv)
{
  FAR struct mpu9250_dev_s *dev = (FAR struct mpu9250_dev_s *)
                                  ((uintptr_t)strtoul(argv[1], NULL, 0));
  FAR struct mpu9250_sensor_s *accel = &dev->priv[MPU9250_ACCEL_IDX];
  FAR struct mpu9250_sensor_s *gyro = &dev->priv[MPU9250_GYRO_IDX];
  FAR struct mpu9250_sensor_s *mag = &dev->priv[MPU9250_MAG_IDX];
  struct sensor_data_s buf;   /* temporary buffer (for read(), etc.) */
  unsigned long  min_interval;
  int ret;

  while (true)
    {
      if ((!accel->enabled) && (!gyro->enabled) && (!mag->enabled))
        {
          /* Waiting to be woken up */

          ret = nxsem_wait(&dev->run);
          if (ret < 0)
            {
              continue;
            }
        }

      /* Returns a snapshot of the accelerometer, temperature, and gyro
       * registers.
       *
       * Note: the chip uses traditional, twos-complement notation, i.e. "0"
       * is encoded as 0, and full-scale-negative is 0x8000, and
       * full-scale-positive is 0x7fff. If we read the registers
       * sequentially and directly into memory (as we do), the measurements
       * from each sensor are captured as big endian words.
       */

      ret = mpu9250_read_imu(dev, &buf);
      if (ret < 0)
        {
          continue;
        }

      mpu9250_accel_data(accel, &buf);

      mpu9250_gyro_data(gyro, &buf);

      mpu9250_mag_data(mag, &buf);

      /* Sleeping thread before fetching the next sensor data */

      min_interval = MIN(accel->interval, gyro->interval);
      min_interval = MIN(min_interval, mag->interval);
      nxsig_usleep(min_interval);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu9250_register
 *
 * Description:
 *   Registers the mpu9250 character device
 *
 * Input Parameters:
 *   devno   - Instance number for driver
 *   config  - Configuration information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mpu9250_register(int devno, FAR struct mpu9250_config_s *config)
{
  FAR struct mpu9250_dev_s *dev;
  FAR struct mpu9250_sensor_s *tmp;
  FAR char *argv[2];
  char arg1[32];
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the device structure. */

  dev = (FAR struct mpu9250_dev_s *)kmm_malloc(sizeof(struct mpu9250_dev_s));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate mpu9250 device instance\n");
      return -ENOMEM;
    }

  memset(dev, 0, sizeof(*dev));
  nxmutex_init(&dev->lock);

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  dev->config = *config;

  /* Accelerometer register */

  tmp = &dev->priv[MPU9250_ACCEL_IDX];
  tmp->lower.ops = &g_mpu9250_ops;
  tmp->lower.type = SENSOR_TYPE_ACCELEROMETER;
  tmp->lower.nbuffer = 1;
  tmp->dev = dev;
  tmp->interval = 1000000 / CONFIG_MPU9250_MEASURE_FREQ;
  tmp->enabled = true;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto accel_err;
    }

  mpu9250_accel_scale(tmp, CONFIG_MPU9250_ACCEL_AFS_SEL);

  /* Gyroscope register */

  tmp = &dev->priv[MPU9250_GYRO_IDX];
  tmp->lower.ops = &g_mpu9250_ops;
  tmp->lower.type = SENSOR_TYPE_GYROSCOPE;
  tmp->lower.nbuffer = 1;
  tmp->dev = dev;
  tmp->interval = 1000000 / CONFIG_MPU9250_MEASURE_FREQ;
  tmp->enabled = false;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto gyro_err;
    }

  mpu9250_gyro_scale(tmp, CONFIG_MPU9250_GYRO_FS_SEL);

  /* Magnetic register */

  tmp = &dev->priv[MPU9250_MAG_IDX];
  tmp->lower.ops = &g_mpu9250_ops;
  tmp->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  tmp->lower.nbuffer = 1;
  tmp->dev = dev;
  tmp->interval = 1000000 / CONFIG_MPU9250_MEASURE_FREQ;
  ret = sensor_register(&tmp->lower, devno);
  tmp->enabled = false;
  if (ret < 0)
    {
      goto mag_err;
    }

  tmp->scale = MAG_RAW_TO_GAUSS;

  /* Reset the chip, to give it an initial configuration. */

  ret = mpu9250_initialize(dev);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure mpu9250: %d\n", ret);

      nxmutex_destroy(&dev->lock);
      kmm_free(dev);
      return ret;
    }

  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "0x%" PRIxPTR, (uintptr_t)dev);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("mpu9250_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_MPU9250_THREAD_STACKSIZE,
                       mpu9250_thread, argv);
  if (ret < 0)
    {
      goto thr_err;
    }

  return ret;

thr_err:
  sensor_unregister(&dev->priv[MPU9250_MAG_IDX].lower, devno);
mag_err:
  sensor_unregister(&dev->priv[MPU9250_GYRO_IDX].lower, devno);
gyro_err:
  sensor_unregister(&dev->priv[MPU9250_ACCEL_IDX].lower, devno);
accel_err:
  kmm_free(dev);
  return ret;
}
