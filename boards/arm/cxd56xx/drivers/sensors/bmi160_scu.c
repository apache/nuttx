/****************************************************************************
 * boards/arm/cxd56xx/drivers/sensors/bmi160_scu.c
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

#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/bmi160.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_SENSORS_BMI160_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SCU_DECI_GYRO
#  define GYRO_SEQ_TYPE SEQ_TYPE_DECI
#else
#  define GYRO_SEQ_TYPE SEQ_TYPE_NORMAL
#endif

#ifdef CONFIG_SENSORS_BMI160_SCU_DECI_ACCEL
#  define ACCEL_SEQ_TYPE SEQ_TYPE_DECI
#else
#  define ACCEL_SEQ_TYPE SEQ_TYPE_NORMAL
#endif

#define DEVID               0xd1
#define BMI160_I2C_ADDR     0x68 /* If SDO pin is pulled to VDDIO, use 0x69 */
#define BMI160_I2C_FREQ     400000

/* BMI160 have accel and gyro, XYZ axis respectively in 16 bits. */

#define BMI160_BYTESPERSAMPLE 6
#define BMI160_ELEMENTSIZE    2

/* Use reading sensor data via oneshot, for debug use only */

/* #define USE_ONESHOTREAD */

/* BMI160 Registers *********************************************************/

/* Register Addresses */

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

/* Register 0x40 - ACCEL_CONFIG accel low power mode averaging */

#define ACCEL_LP_AVG1   (0 << 4)
#define ACCEL_LP_AVG2   (1 << 4)
#define ACCEL_LP_AVG4   (2 << 4)
#define ACCEL_LP_AVG8   (3 << 4)
#define ACCEL_LP_AVG16  (4 << 4)
#define ACCEL_LP_AVG32  (5 << 4)
#define ACCEL_LP_AVG64  (6 << 4)
#define ACCEL_LP_AVG128 (7 << 4)

/* Register 0x40 - ACCEL_CONFIG accel under sampling */

#define ACCEL_US_DISABLE (0 << 7)
#define ACCEL_US_ENABLE  (1 << 7)

/* Register 0x42 - GYRO_CONFIG accel bandwidth */

#define GYRO_OSR4_MODE   (0x00 << 4)
#define GYRO_OSR2_MODE   (0x01 << 4)
#define GYRO_NORMAL_MODE (0x02 << 4)
#define GYRO_CIC_MODE    (0x03 << 4)

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

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Save BMI160 power status */

uint32_t g_pmu_stat;

struct bmi160_dev_s
{
#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* BMP280 I2C address */
  int freq;                     /* BMP280 Frequency <= 3.4MHz */
  int port;                     /* I2C port */
  FAR struct seq_s *seq;        /* Sequencer */
  int fifoid;                   /* Sequencer id */

#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  FAR struct spi_dev_s *spi;    /* SPI interface */
  FAR struct seq_s *seq;        /* Sequencer */
  int fifoid;                   /* Sequencer id */

#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t bmi160_getreg8(FAR struct bmi160_dev_s *priv,
                              uint8_t regaddr);
static void bmi160_putreg8(FAR struct bmi160_dev_s *priv,
                           uint8_t regaddr, uint8_t regval);

/* Character driver methods */

static int     bmi160_open_gyro(FAR struct file *filep);
static int     bmi160_open_accel(FAR struct file *filep);
static int     bmi160_close_gyro(FAR struct file *filep);
static int     bmi160_close_accel(FAR struct file *filep);
static ssize_t bmi160_read(FAR struct file *filep, FAR char *buffer,
                           size_t len);
static int     bmi160_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

static int     bmi160_checkid(FAR struct bmi160_dev_s *priv);

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
static int bmi160_devregister(FAR const char *devpath,
                              FAR struct i2c_master_s *dev,
                              int minor,
                              const struct file_operations *fops,
                              int port);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
static int bmi160_devregister(FAR const char *devpath,
                              FAR struct spi_dev_s *dev,
                              int minor,
                              const struct file_operations *fops);
#endif

static int     bmi160_set_accel_pm(FAR struct bmi160_dev_s *priv, int pm);
static int     bmi160_set_accel_odr(FAR struct bmi160_dev_s *priv, int odr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bmi160gyrofops =
{
  bmi160_open_gyro,    /* open */
  bmi160_close_gyro,   /* close */
  bmi160_read,         /* read */
  NULL,                /* write */
  NULL,                /* seek */
  bmi160_ioctl,        /* ioctl */
  NULL                 /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL               /* unlink */
#endif
};

static const struct file_operations g_bmi160accelfops =
{
  bmi160_open_accel,    /* open */
  bmi160_close_accel,   /* close */
  bmi160_read,          /* read */
  NULL,                 /* write */
  NULL,                 /* seek */
  bmi160_ioctl,         /* ioctl */
  NULL                  /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                /* unlink */
#endif
};

/* SCU instructions for pick gyro sensing data. */

static const uint16_t g_bmi160gyroinst[] =
{
  SCU_INST_SEND(BMI160_DATA_8 | 0x80),
  SCU_INST_RECV(BMI160_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* SCU instructions for pick accel sensing data. */

static const uint16_t g_bmi160accelinst[] =
{
  SCU_INST_SEND(BMI160_DATA_14 | 0x80),
  SCU_INST_RECV(BMI160_BYTESPERSAMPLE) | SCU_INST_LAST,
};

/* Sequencer instance */

static FAR struct seq_s *g_seq_gyro = NULL;
static FAR struct seq_s *g_seq_accel = NULL;

static int g_refcnt_gyro = 0;
static int g_refcnt_accel = 0;

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
  uint16_t inst[2];

  /* Send register to read and get the next byte */

  inst[0] = SCU_INST_SEND(regaddr | 0x80);
  inst[1] = SCU_INST_RECV(1) | SCU_INST_LAST;

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  scu_i2ctransfer(priv->port, priv->addr, inst, 2, &regval, 1);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  scu_spitransfer(0, inst, 2, &regval, 1);
#endif

  return regval;
}

#ifdef USE_ONESHOTREAD
/****************************************************************************
 * Name: bmi160_getregs
 *
 * Description:
 *   Burst read from an BMI160 register
 *
 ****************************************************************************/

static uint8_t bmi160_getregs(FAR struct bmi160_dev_s *priv,
                              uint8_t regaddr, void *buffer, int len)
{
  uint16_t inst[3];
  int ilen;

  /* Send register to read and get the next byte */

  inst[0] = SCU_INST_SEND(regaddr | 0x80);
  if (len > 8)
    {
      inst[1] = SCU_INST_RECV(8);
      inst[2] = SCU_INST_RECV(len - 8) | SCU_INST_LAST;
      ilen = 3;
    }
  else
    {
      inst[1] = SCU_INST_RECV(len) | SCU_INST_LAST;
      ilen = 2;
    }

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  scu_i2ctransfer(priv->port, priv->addr, inst, ilen, buffer, len);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  scu_spitransfer(0, inst, ilen, buffer, len);
#endif
  return OK;
}
#endif

/****************************************************************************
 * Name: bmi160_putreg8
 *
 * Description:
 *   Write a value to an 8-bit BMI160 register
 *
 ****************************************************************************/

static void bmi160_putreg8(FAR struct bmi160_dev_s *priv,
                           uint8_t regaddr, uint8_t regval)
{
  uint16_t inst[2];

  /* Send register address and set the value */

  inst[0] = SCU_INST_SEND(regaddr);
  inst[1] = SCU_INST_SEND(regval) | SCU_INST_LAST;

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  scu_i2ctransfer(priv->port, priv->addr, inst, 2, NULL, 0);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  scu_spitransfer(0, inst, 2, NULL, 0);
#endif
}

/****************************************************************************
 * Name: bmi160_setcommand
 *
 * Description:
 *   Write a value to an 8-bit BMI160 register
 *
 ****************************************************************************/

static void bmi160_setcommand(FAR struct bmi160_dev_s *priv, uint8_t command)
{
  /* Write command register */

  bmi160_putreg8(priv, BMI160_CMD, command);

  /* Interface idle time delay */

  up_mdelay(1);

  /* Save power mode status of Accel and gyro */

  g_pmu_stat = bmi160_getreg8(priv, BMI160_PMU_STAT) & 0x3c;
}

static int bmi160_seqinit_gyro(FAR struct bmi160_dev_s *priv)
{
  DEBUGASSERT(!g_seq_gyro);

  /* Open sequencer */

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  g_seq_gyro = seq_open(GYRO_SEQ_TYPE,
                        (priv->port == 0) ? SCU_BUS_I2C0 : SCU_BUS_I2C1);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  g_seq_gyro = seq_open(GYRO_SEQ_TYPE, SCU_BUS_SPI);
#endif
  if (!g_seq_gyro)
    {
      return -ENOENT;
    }

  priv->seq = g_seq_gyro;

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  seq_setaddress(priv->seq, priv->addr);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  seq_setaddress(priv->seq, 0);
#endif

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq,
                     g_bmi160gyroinst,
                     itemsof(g_bmi160gyroinst));
  seq_setsample(priv->seq,
                BMI160_BYTESPERSAMPLE,
                0,
                BMI160_ELEMENTSIZE,
                false);

  return OK;
}

static int bmi160_seqinit_accel(FAR struct bmi160_dev_s *priv)
{
  DEBUGASSERT(!g_seq_accel);

  /* Open sequencer */

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  g_seq_accel = seq_open(ACCEL_SEQ_TYPE,
                         (priv->port == 0) ? SCU_BUS_I2C0 : SCU_BUS_I2C1);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  g_seq_accel = seq_open(ACCEL_SEQ_TYPE, SCU_BUS_SPI);
#endif
  if (!g_seq_accel)
    {
      return -ENOENT;
    }

  priv->seq = g_seq_accel;

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  seq_setaddress(priv->seq, priv->addr);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  seq_setaddress(priv->seq, 0);
#endif

  /* Set instruction and sample data information to sequencer */

  seq_setinstruction(priv->seq,
                     g_bmi160accelinst,
                     itemsof(g_bmi160accelinst));
  seq_setsample(priv->seq,
                BMI160_BYTESPERSAMPLE,
                0,
                BMI160_ELEMENTSIZE,
                false);

  return OK;
}

/****************************************************************************
 * Name: bmi160_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int bmi160_open_gyro(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;
  int ret;

  if (g_refcnt_gyro == 0)
    {
      /* Open and set sequencer */

      ret = bmi160_seqinit_gyro(priv);
      if (ret)
        {
          return ret;
        }

      /* Change gyroscope to normal mode */

      bmi160_setcommand(priv, GYRO_PM_NORMAL);
      up_mdelay(30);

      /* Set gyro to normal bandwidth and output data rate 100Hz
       * Hz = 100/2^(8-n)
       */

      bmi160_putreg8(priv, BMI160_GYRO_CONFIG, GYRO_NORMAL_MODE | 8);
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_seq_gyro;
    }

  g_refcnt_gyro++;

  return OK;
}

static int bmi160_open_accel(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;
  int ret;

  if (g_refcnt_accel == 0)
    {
      /* Open and set sequencer */

      ret = bmi160_seqinit_accel(priv);
      if (ret)
        {
          return ret;
        }

      /* Change accelerometer to normal mode */

      bmi160_setcommand(priv, ACCEL_PM_NORMAL);
      up_mdelay(30);

      /* Set accel to normal bandwidth and output data rate 100Hz
       * Hz = 100/2^(8-n)
       */

      bmi160_putreg8(priv, BMI160_ACCEL_CONFIG, ACCEL_OSR4_AVG1 | 8);
    }
  else
    {
      /* Set existing sequencer */

      priv->seq = g_seq_accel;
    }

  g_refcnt_accel++;

  return OK;
}

/****************************************************************************
 * Name: bmi160_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int bmi160_close_gyro(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;

  g_refcnt_gyro--;
  if (g_refcnt_gyro == 0)
    {
      DEBUGASSERT(g_seq_gyro);

      /* Change gyroscope to suspend */

      bmi160_setcommand(priv, GYRO_PM_SUSPEND);
      up_mdelay(30);

      seq_close(g_seq_gyro);
      g_seq_gyro = NULL;
    }
  else
    {
      seq_ioctl(priv->seq, priv->fifoid, SCUIOC_FREEFIFO, 0);
    }

  return OK;
}

static int bmi160_close_accel(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;

  g_refcnt_accel--;
  if (g_refcnt_accel == 0)
    {
      DEBUGASSERT(g_seq_accel);

      /* Change accelerometer to suspend */

      bmi160_setcommand(priv, ACCEL_PM_SUSPEND);
      up_mdelay(30);

      /* Close sequencer */

      seq_close(g_seq_accel);
      g_seq_accel = NULL;
    }
  else
    {
      seq_ioctl(priv->seq, priv->fifoid, SCUIOC_FREEFIFO, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: bmi160_read_gyro
 *
 * Description:
 *   Standard character driver read method for accel/gyro.
 *
 ****************************************************************************/

static ssize_t bmi160_read(FAR struct file *filep,
                           FAR char *buffer,
                           size_t len)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmi160_dev_s *priv  = inode->i_private;

#ifdef USE_ONESHOTREAD
  bmi160_getregs(priv, BMI160_DATA_14, buffer, 6);
  len = 6;
#else
  len = seq_read(priv->seq, priv->fifoid, buffer, len);
#endif

  return len;
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
      case SNIOC_SETACCPM:
        ret = bmi160_set_accel_pm(priv, arg);
        break;

      case SNIOC_SETACCODR:
        ret = bmi160_set_accel_odr(priv, arg);
        break;

      default:
        {
          if (_SCUIOCVALID(cmd))
            {
              /* Redirect SCU commands */

              ret = seq_ioctl(priv->seq, priv->fifoid, cmd, arg);
            }
          else
            {
              snerr("Unrecognized cmd: %d\n", cmd);
              ret = -ENOTTY;
            }
        }
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
 * Name: bmi160_seqregister
 *
 * Description:
 *   Register the BMI160 character device with sequencer
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/accel"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI160
 *   id      - FIFO ID
 *   fops    - File operations
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
static int bmi160_devregister(FAR const char *devpath,
                              FAR struct i2c_master_s *dev,
                              int minor,
                              const struct file_operations *fops,
                              int port)
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
static int bmi160_devregister(FAR const char *devpath,
                              FAR struct spi_dev_s *dev,
                              int minor,
                              const struct file_operations *fops)
#endif
{
  FAR struct bmi160_dev_s *priv;
  char path[12];
  int ret;

  priv = (FAR struct bmi160_dev_s *)kmm_malloc(sizeof(struct bmi160_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  priv->i2c = dev;
  priv->seq = NULL;
  priv->fifoid = minor;
  priv->addr = BMI160_I2C_ADDR;
  priv->freq = BMI160_I2C_FREQ;
  priv->port = port;

#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  priv->spi = dev;
  priv->seq = NULL;
  priv->fifoid = minor;

#endif
  snprintf(path, sizeof(path), "%s%d", devpath, minor);
  ret = register_driver(path, fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: bmi160_set_accel_pm
 *
 * Description:
 *   Set the accelerometer's power mode
 *
 * Input Parameters:
 *   pm - Power mode to be set.
 *        Modes are suspend(0) or normal(1) or low power(2).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bmi160_set_accel_pm(FAR struct bmi160_dev_s *priv, int pm)
{
  uint8_t value;

  switch (pm)
    {
      case BMI160_PM_SUSPEND:

        /* Set suspend mode */

        bmi160_setcommand(priv, ACCEL_PM_SUSPEND);
        up_mdelay(30);

        break;

      case BMI160_PM_NORMAL:

        /* Keep output data rate settings */

        value = bmi160_getreg8(priv, BMI160_ACCEL_CONFIG) & 0x0f;

        /* Output data rate less than 12.5Hz needs undersampling */

        if (value < BMI160_ACCEL_ODR_12_5HZ)
          {
            value |= ACCEL_US_ENABLE;
          }

        /* Reset undersampling and bandwidth setting */

        bmi160_putreg8(priv, BMI160_ACCEL_CONFIG, ACCEL_OSR4_AVG1 | value);
        up_mdelay(1);

        /* Set normal mode */

        bmi160_setcommand(priv, ACCEL_PM_NORMAL);
        up_mdelay(30);

        break;

      case BMI160_PM_LOWPOWER:

        /* Keep output data rate setting */

        value = bmi160_getreg8(priv, BMI160_ACCEL_CONFIG) & 0x0f;

        /* Set undersampling and averaging cycle */

        bmi160_putreg8(priv,
                       BMI160_ACCEL_CONFIG,
                       ACCEL_US_ENABLE | ACCEL_LP_AVG32 | value);
        up_mdelay(1);

        /* Set low power mode */

        bmi160_setcommand(priv, ACCEL_PM_LOWPOWER);
        up_mdelay(30);

        break;

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: bmi160_set_accel_odr
 *
 * Description:
 *   Set the accelerometer's output data rate
 *
 * Input Parameters:
 *   odr - Output data rate parameter to be set.
 *        The result rate is 100/2^(8-odr) Hz.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bmi160_set_accel_odr(FAR struct bmi160_dev_s *priv, int odr)
{
  uint8_t value;

  if (odr < BMI160_ACCEL_ODR_0_78HZ || BMI160_ACCEL_ODR_1600HZ < odr)
    {
      return -EINVAL;
    }

  /* Keep undersampling and bandwidth settings */

  value = bmi160_getreg8(priv, BMI160_ACCEL_CONFIG) & 0xf0;

  /* In normal mode, odr < 12.5Hz needs undersampling */

  if (((g_pmu_stat & 0x30) >> 4) == BMI160_PM_NORMAL)
    {
      if (odr < BMI160_ACCEL_ODR_12_5HZ)
        {
          value |= ACCEL_US_ENABLE;
        }
      else
        {
          value &= ~ACCEL_US_ENABLE;
        }
    }

  /* Set output data rate parameter */

  bmi160_putreg8(priv, BMI160_ACCEL_CONFIG, value | odr);
  up_mdelay(1);

  return OK;
}

/****************************************************************************
 * Name: bmi160_init
 *
 * Description:
 *   Register the BMI160 character device as 'devpath'
 *
 * Input Parameters:
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
int bmi160_init(FAR struct i2c_master_s *dev, int port)
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
int bmi160_init(FAR struct spi_dev_s *dev)
#endif
{
  struct bmi160_dev_s tmp;
  struct bmi160_dev_s *priv = &tmp;
  int ret;

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  /* Setup temporary device structure for initialization */

  priv->i2c = dev;
  priv->addr = BMI160_I2C_ADDR;
  priv->freq = BMI160_I2C_FREQ;
  priv->port = port;

#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  /* Configure SPI for the BMI160 */

  SPI_SETMODE(dev, SPIDEV_MODE3);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, BMI160_SPI_MAXFREQUENCY);

  /* BMI160 detects communication bus is SPI by rising edge of CS. */

  bmi160_getreg8(priv, 0x7f);
  bmi160_getreg8(priv, 0x7f); /* workaround: fail to switch SPI, run twice */

#endif

  ret = bmi160_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      return ret;
    }

  /* To avoid gyro wakeup it is required to write 0x00 to 0x6C */

  bmi160_putreg8(priv, BMI160_PMU_TRIGGER, 0);
  up_mdelay(1);

  return OK;
}

/****************************************************************************
 * Name: bmi160gyro_register
 *
 * Description:
 *   Register the BMI160 gyro sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/gyro"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
int bmi160gyro_register(FAR const char *devpath, int minor,
                        FAR struct i2c_master_s *dev, int port)
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
int bmi160gyro_register(FAR const char *devpath, int minor,
                        FAR struct spi_dev_s *dev)
#endif
{
  int ret;

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  ret = bmi160_devregister(devpath, dev, minor, &g_bmi160gyrofops, port);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  ret = bmi160_devregister(devpath, dev, minor, &g_bmi160gyrofops);
#endif
  if (ret < 0)
    {
      snerr("Gyroscope register failed. %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bmi160accel_register
 *
 * Description:
 *   Register the BMI160 accelerometer character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/accel"
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
int bmi160accel_register(FAR const char *devpath, int minor,
                         FAR struct i2c_master_s *dev, int port)
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
int bmi160accel_register(FAR const char *devpath, int minor,
                         FAR struct spi_dev_s *dev)
#endif
{
  int ret;

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
  ret = bmi160_devregister(devpath, dev, minor, &g_bmi160accelfops, port);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
  ret = bmi160_devregister(devpath, dev, minor, &g_bmi160accelfops);
#endif
  if (ret < 0)
    {
      snerr("Accelerometer register failed. %d\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_BMI160 */
