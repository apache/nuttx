/****************************************************************************
 * drivers/sensors/bmi160.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi160.h>

#if defined(CONFIG_SENSORS_BMI160)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVID               0xd1

/* I2C  Address
 *
 * NOTE: If SDO pin is pulled to VDDIO, use 0x69
 */

#ifdef CONFIG_BMI160_I2C_ADDR_68
#define BMI160_I2C_ADDR     0x68
#else
#define BMI160_I2C_ADDR     0x69
#endif

#define BMI160_I2C_FREQ     400000

#define BMI160_CHIP_ID          (0x00) /* Chip ID */
#define BMI160_ERROR            (0x02) /* Error register */
#define BMI160_PMU_STAT         (0x03) /* Current power mode */
#define BMI160_DATA_0           (0x04) /* MAG X  7:0 (LSB) */
#define BMI160_DATA_1           (0x05) /* MAG X 15:8 (MSB) */
#define BMI160_DATA_2           (0x06) /* MAG Y  7:0 (LSB) */
#define BMI160_DATA_3           (0x07) /* MAG Y 15:8 (MSB) */
#define BMI160_DATA_4           (0x08) /* MAG Z  7:0 (LSB) */
#define BMI160_DATA_5           (0x09) /* MAG Z 15:8 (MSB) */
#define BMI160_DATA_6           (0x0A) /* RHALL  7:0 (LSB) */
#define BMI160_DATA_7           (0x0B) /* RHALL 15:8 (MSB) */
#define BMI160_DATA_8           (0x0C) /* GYR X  7:0 (LSB) */
#define BMI160_DATA_9           (0x0D) /* GYR X 15:8 (MSB) */
#define BMI160_DATA_10          (0x0E) /* GYR Y  7:0 (LSB) */
#define BMI160_DATA_11          (0x0F) /* GYR Y 15:8 (MSB) */
#define BMI160_DATA_12          (0x10) /* GYR Z  7:0 (LSB) */
#define BMI160_DATA_13          (0x11) /* GYR Z 15:8 (MSB) */
#define BMI160_DATA_14          (0x12) /* ACC X  7:0 (LSB) */
#define BMI160_DATA_15          (0x13) /* ACC X 15:8 (MSB) */
#define BMI160_DATA_16          (0x14) /* ACC Y  7:0 (LSB) */
#define BMI160_DATA_17          (0x15) /* ACC Y 15:8 (MSB) */
#define BMI160_DATA_18          (0x16) /* ACC Z  7:0 (LSB) */
#define BMI160_DATA_19          (0x17) /* ACC Z 15:8 (MSB) */
#define BMI160_SENSORTIME_0     (0x18) /* Sensor time 0 */
#define BMI160_SENSORTIME_1     (0x19) /* Sensor time 1 */
#define BMI160_SENSORTIME_2     (0x1A) /* Sensor time 2 */
#define BMI160_STAT             (0x1B) /* Status register */
#define BMI160_INTR_STAT_0      (0x1C) /* Interrupt status */
#define BMI160_INTR_STAT_1      (0x1D)
#define BMI160_INTR_STAT_2      (0x1E)
#define BMI160_INTR_STAT_3      (0x1F)
#define BMI160_TEMPERATURE_0    (0x20) /* Temperature */
#define BMI160_TEMPERATURE_1    (0x21)
#define BMI160_FIFO_LENGTH_0    (0x22) /* FIFO length */
#define BMI160_FIFO_LENGTH_1    (0x23)
#define BMI160_FIFO_DATA        (0x24)
#define BMI160_ACCEL_CONFIG     (0x40) /* ACCEL config for ODR, bandwidth and undersampling */
#define BMI160_ACCEL_RANGE      (0x41) /* ACCEL range */
#define BMI160_GYRO_CONFIG      (0x42) /* GYRO config for ODR and bandwidth */
#define BMI160_GYRO_RANGE       (0x43) /* GYRO range */
#define BMI160_MAG_CONFIG       (0x44) /* MAG config for ODR */
#define BMI160_FIFO_DOWN        (0x45) /* GYRO and ACCEL downsampling rates for FIFO */
#define BMI160_FIFO_CONFIG_0    (0x46) /* FIFO config */
#define BMI160_FIFO_CONFIG_1    (0x47)
#define BMI160_MAG_IF_0         (0x4B) /* MAG interface */
#define BMI160_MAG_IF_1         (0x4C)
#define BMI160_MAG_IF_2         (0x4D)
#define BMI160_MAG_IF_3         (0x4E)
#define BMI160_MAG_IF_4         (0x4F)
#define BMI160_INTR_ENABLE_0    (0x50) /* Interrupt enable */
#define BMI160_INTR_ENABLE_1    (0x51)
#define BMI160_INTR_ENABLE_2    (0x52)
#define BMI160_INTR_OUT_CTRL    (0x53)
#define BMI160_INTR_LATCH       (0x54) /* Latch duration */
#define BMI160_INTR_MAP_0       (0x55) /* Map interrupt */
#define BMI160_INTR_MAP_1       (0x56)
#define BMI160_INTR_MAP_2       (0x57)
#define BMI160_INTR_DATA_0      (0x58) /* Data source */
#define BMI160_INTR_DATA_1      (0x59)
#define BMI160_INTR_LOWHIGH_0   (0x5A) /* Threshold interrupt */
#define BMI160_INTR_LOWHIGH_1   (0x5B)
#define BMI160_INTR_LOWHIGH_2   (0x5C)
#define BMI160_INTR_LOWHIGH_3   (0x5D)
#define BMI160_INTR_LOWHIGH_4   (0x5E)
#define BMI160_INTR_MOTION_0    (0x5F)
#define BMI160_INTR_MOTION_1    (0x60)
#define BMI160_INTR_MOTION_2    (0x61)
#define BMI160_INTR_MOTION_3    (0x62)
#define BMI160_INTR_TAP_0       (0x63)
#define BMI160_INTR_TAP_1       (0x64)
#define BMI160_INTR_ORIENT_0    (0x65)
#define BMI160_INTR_ORIENT_1    (0x66)
#define BMI160_INTR_FLAT_0      (0x67)
#define BMI160_INTR_FLAT_1      (0x68)
#define BMI160_FOC_CONFIG       (0x69) /* Fast offset configuration */
#define BMI160_CONFIG           (0x6A) /* Miscellaneous configuration */
#define BMI160_IF_CONFIG        (0x6B) /* Serial interface configuration */
#define BMI160_PMU_TRIGGER      (0x6C) /* GYRO power mode trigger */
#define BMI160_SELF_TEST        (0x6D) /* Self test */
#define BMI160_NV_CONFIG        (0x70) /* SPI/I2C selection */
#define BMI160_OFFSET_0         (0x71) /* ACCEL and GYRO offset */
#define BMI160_OFFSET_1         (0x72)
#define BMI160_OFFSET_2         (0x73)
#define BMI160_OFFSET_3         (0x74)
#define BMI160_OFFSET_4         (0x75)
#define BMI160_OFFSET_5         (0x76)
#define BMI160_OFFSET_6         (0x77)
#define BMI160_STEP_COUNT_0     (0x78) /* Step counter interrupt */
#define BMI160_STEP_COUNT_1     (0x79)
#define BMI160_STEP_CONFIG_0    (0x7A) /* Step counter configuration */
#define BMI160_STEP_CONFIG_1    (0x7B)
#define BMI160_CMD              (0x7e) /* Command register */

/* Register 0x40 - ACCEL_CONFIG accel bandwidth */

#define ACCEL_OSR4_AVG1   (0 << 4)
#define ACCEL_OSR2_AVG2   (1 << 4)
#define ACCEL_NORMAL_AVG4 (2 << 4)
#define ACCEL_CIC_AVG8    (3 << 4)
#define ACCEL_RES_AVG2    (4 << 4)
#define ACCEL_RES_AVG4    (5 << 4)
#define ACCEL_RES_AVG8    (6 << 4)
#define ACCEL_RES_AVG16   (7 << 4)
#define ACCEL_RES_AVG32   (8 << 4)
#define ACCEL_RES_AVG64   (9 << 4)
#define ACCEL_RES_AVG128  (10 << 4)

#define ACCEL_ODR_0_78HZ      (0x01)
#define ACCEL_ODR_1_56HZ      (0x02)
#define ACCEL_ODR_3_12HZ      (0x03)
#define ACCEL_ODR_6_25HZ      (0x04)
#define ACCEL_ODR_12_5HZ      (0x05)
#define ACCEL_ODR_25HZ        (0x06)
#define ACCEL_ODR_50HZ        (0x07)
#define ACCEL_ODR_100HZ       (0x08)
#define ACCEL_ODR_200HZ       (0x09)
#define ACCEL_ODR_400HZ       (0x0A)
#define ACCEL_ODR_800HZ       (0x0B)
#define ACCEL_ODR_1600HZ      (0x0C)

/* Register 0x42 - GYRO_CONFIG accel bandwidth */

#define GYRO_OSR4_MODE   (0x00 << 4)
#define GYRO_OSR2_MODE   (0x01 << 4)
#define GYRO_NORMAL_MODE (0x02 << 4)
#define GYRO_CIC_MODE    (0x03 << 4)

#define GYRO_ODR_25HZ         (0x06)
#define GYRO_ODR_50HZ         (0x07)
#define GYRO_ODR_100HZ        (0x08)
#define GYRO_ODR_200HZ        (0x09)
#define GYRO_ODR_400HZ        (0x0A)
#define GYRO_ODR_800HZ        (0x0B)
#define GYRO_ODR_1600HZ       (0x0C)
#define GYRO_ODR_3200HZ       (0x0D)

/* Register 0x7b STEP_CONFIG_1 */

#define STEP_CNT_EN           (1 << 3)

/* Register 0x7e - CMD */

#define	ACCEL_PM_SUSPEND      (0X10)
#define ACCEL_PM_NORMAL       (0x11)
#define	ACCEL_PM_LOWPOWER     (0X12)
#define GYRO_PM_SUSPEND       (0x14)
#define GYRO_PM_NORMAL        (0x15)
#define GYRO_PM_FASTSTARTUP   (0x17)
#define MAG_PM_SUSPEND        (0x18)
#define MAG_PM_NORMAL         (0x19)
#define MAG_PM_LOWPOWER       (0x1A)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bmi160_dev_s
{
#ifdef CONFIG_SENSORS_BMI160_I2C
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  int freq;                     /* Frequency <= 3.4MHz */

#else /* CONFIG_SENSORS_BMI160_SPI */
  FAR struct spi_dev_s *spi;    /* SPI interface */

#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t bmi160_getreg8(FAR struct bmi160_dev_s *priv,
                              uint8_t regaddr);
static void bmi160_putreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);
static uint16_t bmi160_getreg16(FAR struct bmi160_dev_s *priv,
                                uint8_t regaddr);
static void bmi160_getregs(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                           uint8_t *regval, int len);

/* Character driver methods */

static int     bmi160_open(FAR struct file *filep);
static int     bmi160_close(FAR struct file *filep);
static ssize_t bmi160_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
static int     bmi160_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

static int bmi160_checkid(FAR struct bmi160_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bmi160fops =
{
  bmi160_open,    /* open */
  bmi160_close,   /* close */
  bmi160_read,    /* read */
  0,               /* write */
  0,               /* seek */
  bmi160_ioctl,    /* ioctl */
};

/****************************************************************************
 * Name: bmi160_configspi
 *
 * Description:
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SPI
static inline void bmi160_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the BMI160 */

  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, BMI160_SPI_MAXFREQUENCY);
}
#endif

/****************************************************************************
 * Name: bmi160_getreg8
 *
 * Description:
 *   Read from an 8-bit BMI160 register
 *
 ****************************************************************************/

static uint8_t bmi160_getreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval = 0;

#ifdef CONFIG_SENSORS_BMI160_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI160_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi160_configspi(priv->spi);

  /* Select the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
#endif

  return regval;
}

/****************************************************************************
 * Name: bmi160_putreg8
 *
 * Description:
 *   Write a value to an 8-bit BMI160 register
 *
 ****************************************************************************/

static void bmi160_putreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
#ifdef CONFIG_SENSORS_BMI160_I2C
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI160_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi160_configspi(priv->spi);

  /* Select the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);

  /* Deselect the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#endif
}

/****************************************************************************
 * Name: bmi160_getreg16
 *
 * Description:
 *   Read 16-bits of data from an BMI160 register
 *
 ****************************************************************************/

static uint16_t bmi160_getreg16(FAR struct bmi160_dev_s *priv,
                                uint8_t regaddr)
{
  uint16_t regval = 0;

#ifdef CONFIG_SENSORS_BMI160_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = (uint8_t *)&regval;
  msg[1].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI160_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi160_configspi(priv->spi);

  /* Select the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, &regval, 2);

  /* Deselect the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
#endif

  return regval;
}

/****************************************************************************
 * Name: bmi160_getregs
 *
 * Description:
 *   Read cnt bytes from specified dev_addr and reg_addr
 *
 ****************************************************************************/

static void bmi160_getregs(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                           uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BMI160_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI160_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi160_configspi(priv->spi);

  /* Select the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, regval, len);

  /* Deselect the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#endif
}

/****************************************************************************
 * Name: bmi160_set_normal_imu
 *
 * Description:
 *   set bmi160 to normal IMU mode.
 *
 ****************************************************************************/

static void bmi160_set_normal_imu(FAR struct bmi160_dev_s *priv)
{
  /* Set accel & gyro as normal mode. */

  bmi160_putreg8(priv, BMI160_CMD, ACCEL_PM_NORMAL);
  up_mdelay(30);
  bmi160_putreg8(priv, BMI160_CMD, GYRO_PM_NORMAL);
  up_mdelay(30);

  /* Set accel & gyro output data rate. */

  bmi160_putreg8(priv, BMI160_ACCEL_CONFIG,
                 ACCEL_NORMAL_AVG4 | ACCEL_ODR_100HZ);
  bmi160_putreg8(priv, BMI160_GYRO_CONFIG, GYRO_NORMAL_MODE | GYRO_ODR_100HZ);
}

/****************************************************************************
 * Name: bmi160_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int bmi160_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;

  bmi160_set_normal_imu(priv);

  return OK;
}

/****************************************************************************
 * Name: bmi160_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int bmi160_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;

  /* Set suspend mode to each sensors. */

  bmi160_putreg8(priv, BMI160_CMD, ACCEL_PM_SUSPEND);
  up_mdelay(30);

  bmi160_putreg8(priv, BMI160_CMD, GYRO_PM_SUSPEND);
  up_mdelay(30);

  return OK;
}

/****************************************************************************
 * Name: bmi160_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t bmi160_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;
  FAR struct accel_gyro_st_s *p = (FAR struct accel_gyro_st_s *)buffer;

  if (len < sizeof(struct accel_gyro_st_s))
    {
      snerr("Expected buffer size is %d\n", sizeof(struct accel_gyro_st_s));
      return 0;
    }

  bmi160_getregs(priv, BMI160_DATA_8, (FAR uint8_t *)buffer, 15);

  /* Adjust sensing time into 24 bit */

  p->sensor_time >>= 8;

  return len;
}

static void bmi160_enable_stepcounter(FAR struct bmi160_dev_s *priv,
                                      int enable)
{
  uint8_t val;

  val = bmi160_getreg8(priv, BMI160_STEP_CONFIG_1);
  if (enable)
    {
      val |= STEP_CNT_EN;
    }
  else
    {
      val &= ~STEP_CNT_EN;
    }

  bmi160_putreg8(priv, BMI160_STEP_CONFIG_1, val);

  sninfo("Step counter %sabled.\n", val & STEP_CNT_EN ? "en" : "dis");
}

/****************************************************************************
 * Name: bmi160_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int bmi160_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Enable bmi160 step counter. Arg: int value */

      case SNIOC_ENABLESC:
        {
          bmi160_enable_stepcounter(priv, (int)arg);
        }
        break;

      /* Read bmi160 step count. Arg:  int16_t* pointer */

      case SNIOC_READSC:
        {
          int16_t *ptr = (FAR int16_t *)((uintptr_t)arg);

          DEBUGASSERT(ptr != NULL);

          *ptr = bmi160_getreg16(priv, BMI160_STEP_COUNT_0);
        }
        break;

      default:
        snerr("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: bmi160_checkid
 *
 * Description:
 *   Read and verify the BMI160 chip ID
 *
 ****************************************************************************/

static int bmi160_checkid(FAR struct bmi160_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID  */

  devid = bmi160_getreg8(priv, BMI160_CHIP_ID);
  sninfo("devid: %04x\n", devid);

  if (devid != (uint16_t) DEVID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bmi160_register
 *
 * Description:
 *   Register the BMI160 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_I2C
int bmi160_register(FAR const char *devpath, FAR struct i2c_master_s *dev)
#else /* CONFIG_SENSORS_BMI160_SPI */
int bmi160_register(FAR const char *devpath, FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi160_dev_s *priv;
  int ret;

  priv = (FAR struct bmi160_dev_s *)kmm_malloc(sizeof(struct bmi160_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

#ifdef CONFIG_SENSORS_BMI160_I2C
  priv->i2c = dev;
  priv->addr = BMI160_I2C_ADDR;
  priv->freq = BMI160_I2C_FREQ;

#else /* CONFIG_SENSORS_BMI160_SPI */
  priv->spi = dev;

  /* BMI160 detects communication bus is SPI by rising edge of CS. */

  bmi160_getreg8(priv, 0x7f);
  bmi160_getreg8(priv, 0x7f); /* workaround: fail to switch SPI, run twice */
  up_udelay(200);

#endif

  ret = bmi160_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* To avoid gyro wakeup it is required to write 0x00 to 0x6C */

  bmi160_putreg8(priv, BMI160_PMU_TRIGGER, 0);

  ret = register_driver(devpath, &g_bmi160fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMI160 driver loaded successfully!\n");
  return OK;
}

#endif /* CONFIG_SENSORS_BMI160 */
