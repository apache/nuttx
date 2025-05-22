/****************************************************************************
 * drivers/sensors/lsm6dso32_uorb.c
 *
 * Contributed by Carleton University InSpace
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
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
#include <nuttx/nuttx.h>

#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/lsm6dso32.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>
#include <stdio.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The value that should be in the WHO_AM_I register. */

#define WHO_AM_I_VAL 0x6c

/* Convert milli Gs to m/s^2 */

#define MILLIG_TO_MS2 (0.0098067f)

/* Convert milli-dps to rad/s */

#define MDPS_TO_RADS (3.141592653f / (180.0f * 1000.0f))

/* Number of measurement rounds for gyro self test */

#define GYRO_SELFTEST_ROUNDS 5

/* Minimum and maximum values for self-test at +/-2000dps, converted from
 * 1dps/LSB to 70mdps/LSB
 */

#define GYRO_ST_MIN ((150 * 1000) / 70)
#define GYRO_ST_MAX ((700 * 1000) / 70)

#define XL_SELFTEST_ROUNDS 5

/* Minimum and maximum values for self-test at +/-4g in 1mg/LSB converted to
 * 0.122mg/LSB
 */

#define XL_ST_MIN (50 / 0.122f)
#define XL_ST_MAX (1700 / 0.122f)

/* Min & max representable accel offset in g using 2^{-10}g/LSB */

#define XL_10_W_MAX (127.0f / (1 << 10))
#define XL_10_W_MIN (-127.0f / (1 << 10))

/* Min & max representable accel offset in g using 2^{-6}g/LSB */

#define XL_6_W_MAX (127.0f / (1 << 6))
#define XL_6_W_MIN (-127.0f / (1 << 6))

/* Registers */

#define WHO_AM_I 0x0f   /* Hard-coded address on I2C bus. */
#define TIMESTAMP0 0x40 /* First timestamp register (32 bits) */
#define STATUS_REG 0x1e /* The status register */
#define CTRL1_XL 0x10   /* Accel control reg 1 */
#define CTRL2_G 0x11    /* Gyro control reg 2 */
#define CTRL3_C 0x12    /* Control reg 3 */
#define CTRL4_C 0x13    /* Control reg 4 */
#define CTRL5_C 0x14    /* Control reg 5 */
#define CTRL6_C 0x15    /* Control reg 6 */
#define CTRL7_G 0x16    /* Control reg 7 */
#define CTRL8_XL 0x17   /* Control reg 8 */
#define CTRL9_XL 0x18   /* Control reg 9 */
#define CTRL10_C 0x19   /* Control reg 10 */
#define FIFO_CTRL4 0x0a /* The fourth FIFO control reg  */
#define INT1_CTRL 0x0d  /* INT1 pin control */
#define INT2_CTRL 0x0e  /* INT2 pin control */
#define OUT_TEMP_L 0x20 /* Temp output low byte. */
#define OUT_TEMP_H 0x21 /* Temp output high byte. */
#define OUTX_L_G 0x22   /* Gyro pitch axis (X) low byte. */
#define OUTX_H_G 0x23   /* Gyro pitch axis (X) high byte. */
#define OUTY_L_G 0x24   /* Gyro roll axis (Y) low byte. */
#define OUTY_H_G 0x25   /* Gyro roll axis (Y) high byte. */
#define OUTZ_L_G 0x26   /* Gyro yaw axis (Z) low byte. */
#define OUTZ_H_G 0x27   /* Gyro yaw axis (Z) high byte. */
#define OUTX_L_A 0x28   /* Accel (X) low byte. */
#define OUTX_H_A 0x29   /* Accel (X) high byte. */
#define OUTY_L_A 0x2a   /* Accel (Y) low byte. */
#define OUTY_H_A 0x2b   /* Accel (Y) high byte. */
#define OUTZ_L_A 0x2c   /* Accel (Z) low byte. */
#define OUTZ_H_A 0x2d   /* Accel (Z) high byte. */
#define X_OFS_USR 0x73  /* X offset correction accel */
#define Y_OFS_USR 0x74  /* Y offset correction accel */
#define Z_OFS_USR 0x75  /* Z offset correction accel */

/* Bits */

#define BIT_STATUS_XLDA (1 << 0)    /* Accel data ready */
#define BIT_STATUS_GDA (1 << 1)     /* Gyro data ready */
#define BIT_STATUS_TDA (1 << 2)     /* Temp data ready */
#define BIT_G_ST_POS (1 << 2)       /* Enable gyro positive self-test */
#define BIT_XL_ST_POS (1 << 0)      /* Enable accel positive self-test */
#define BIT_USR_OFF_W (1 << 3)      /* User offset weight */
#define BIT_USR_OFF_ON_OUT (1 << 1) /* User offset enable */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ODRs common to the accelerometer and the gyroscope. NOTE: this driver does
 * implement the 1.6Hz low power ODR for the accelerometer.
 */

enum lsm6dso32_odr_e
{
  ODR_OFF = 0x0,    /* Sensor Deactivated */
  ODR_12_5HZ = 0x1, /* 12.5Hz Rate. */
  ODR_26HZ = 0x2,   /* 26Hz Rate. */
  ODR_52HZ = 0x3,   /* 52Hz Rate. */
  ODR_104HZ = 0x4,  /* 104Hz Rate */
  ODR_208HZ = 0x5,  /* 208Hz Rate */
  ODR_416HZ = 0x6,  /* 416Hz Rate */
  ODR_833HZ = 0x7,  /* 833Hz Rate */
  ODR_1660HZ = 0x8, /* 1.66kHz Rate */
  ODR_3330HZ = 0x9, /* 3.33kHz Rate */
  ODR_6660HZ = 0xa, /* 6.66kHz Rate */
  ODR_1_6HZ = 0xb,  /* 1.6Hz Rate */
};

/* Gyroscope FSRs */

enum lsm6dso32_fsr_gyro_e
{
  LSM6DSO32_FSR_GY_250DPS = 0x0,  /* +-250dps */
  LSM6DSO32_FSR_GY_125DPS = 0x1,  /* +-125dps */
  LSM6DSO32_FSR_GY_500DPS = 0x2,  /* +-500dps */
  LSM6DSO32_FSR_GY_1000DPS = 0x4, /* +-1000dps */
  LSM6DSO32_FSR_GY_2000DPS = 0x6, /* +-2000dps */
};

/* Accelerometer FSRs */

enum lsm6dso32_fsr_xl_e
{
  LSM6DSO32_FSR_XL_4G = 0x0,  /* +-4g */
  LSM6DSO32_FSR_XL_32G = 0x1, /* +-32g */
  LSM6DSO32_FSR_XL_8G = 0x2,  /* +-8g */
  LSM6DSO32_FSR_XL_16G = 0x3, /* +-16g */
};

/* Represents a lower half sensor driver of the LSM6DSO32 */

struct lsm6dso32_sens_s
{
  FAR struct sensor_lowerhalf_s lower; /* Lower-half sensor driver */
  FAR struct lsm6dso32_dev_s *dev;     /* Reference to parent device */
  bool enabled;                        /* If this sensor is enabled */
  enum lsm6dso32_odr_e odr;            /* Measurement interval of this
                                        * sensor */
  int fsr;                             /* Full scale range of this sensor.
                                        * Can be from either gyro or accel
                                        * FSR enum. */
  sem_t run;                           /* Polling cycle lock */
  enum lsm6dso32_int_e intpin;         /* The interrupt pin for this device */
  bool interrupts;                     /* Whether or not interrupts are
                                        * enabled. */
  struct work_s work;                  /* Interrupt work queue
                                        * structure */
};

/* Represents the LSM6DSO23 IMU device */

struct lsm6dso32_dev_s
{
  struct lsm6dso32_sens_s gyro;  /* Gyroscope */
  struct lsm6dso32_sens_s accel; /* Accelerometer lower half */
  FAR struct i2c_master_s *i2c;  /* I2C interface. */
  uint8_t addr;                  /* I2C address. */
  float gy_off[3];               /* Offsets for gyroscope measurements */
  mutex_t devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lsm6dso32_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, int cmd,
                             unsigned long arg);
static int lsm6dso32_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable);
static int lsm6dso32_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR uint32_t *period_us);
static int lsm6dso32_selftest(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, unsigned long arg);
static int lsm6dso32_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    unsigned long arg);
static int lsm6dso32_get_info(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR struct sensor_device_info_s *info);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ODR frequencies to measurement intervals in microseconds */

static const uint32_t ODR_INTERVAL[] =
{
  0,      /* ODR_OFF */
  80000,  /* ODR_12_5HZ */
  38462,  /* ODR_26HZ */
  19230,  /* ODR_52HZ */
  9615,   /* ODR_104HZ */
  4807,   /* ODR_208HZ */
  2403,   /* ODR_416HZ */
  1200,   /* ODR_833Hz */
  602,    /* ODR_1660HZ */
  300,    /* ODR_3330HZ */
  150,    /* ODR_6660HZ */
  625000, /* ODR_1_6Z */
};

/* Accelerometer FSR sensitivities in m/s^2 per LSB */

static const float FSR_XL_SENS[] =
{
  0.122f * MILLIG_TO_MS2, /* 4g */
  0.976f * MILLIG_TO_MS2, /* 32g */
  0.244f * MILLIG_TO_MS2, /* 8g */
  0.488f * MILLIG_TO_MS2, /* 16g */
};

/* Gyro FSR sensitivities in rad/s per LSB */

static const float FSR_GYRO_SENS[] =
{
  8.75f * MDPS_TO_RADS,  /* LSM6DSO32_FSR_GY_250DPS */
  4.375f * MDPS_TO_RADS, /* LSM6DSO32_FSR_GY_125DPS */
  17.50f * MDPS_TO_RADS, /* LSM6DSO32_FSR_GY_500DPS */
  0.0f,                  /* No such setting (3) */
  35.0f * MDPS_TO_RADS,  /* LSM6DSO32_FSR_GY_1000DPS */
  0.0f,                  /* No such setting (5) */
  70.0f * MDPS_TO_RADS,  /* LSM6DSO32_FSR_GY_2000DPS */
};

/* Interrupt control registers */

static const uint8_t INT_CTRL[] =
{
  INT1_CTRL, /* INT1 */
  INT2_CTRL, /* INT2 */
};

/* Sensor operations */

static const struct sensor_ops_s g_sensor_ops =
{
  .fetch = NULL,
  .activate = lsm6dso32_activate,
  .control = lsm6dso32_control,
  .set_interval = lsm6dso32_set_interval,
  .selftest = lsm6dso32_selftest,
  .set_calibvalue = lsm6dso32_set_calibvalue,
  .calibrate = NULL,
  .get_info = lsm6dso32_get_info,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dso32_write_bytes
 *
 * Description:
 *   Write bytes to the LSM6DSO32 sensor. Providing more than one byte will
 *   write to sequential registers starting at the provided address.
 *
 * Input Parameters:
 *   priv    - The instance of the LSM6DSO32 sensor.
 *   addr    - The register address to write to.
 *   buf     - The buffer of data to write.
 *   nbytes  - The number of bytes in the buffer to write.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm6dso32_write_bytes(FAR struct lsm6dso32_dev_s *priv,
                                 uint8_t addr, void *buf, size_t nbytes)
{
  struct i2c_msg_s cmd[2];

  /* Register addressing part of command. */

  cmd[0].frequency = CONFIG_SENSORS_LSM6DSO32_I2C_FREQUENCY;
  cmd[0].addr = priv->addr;
  cmd[0].flags = I2C_M_NOSTOP;
  cmd[0].buffer = &addr;
  cmd[0].length = sizeof(addr);

  /* Data to write. */

  cmd[1].frequency = CONFIG_SENSORS_LSM6DSO32_I2C_FREQUENCY;
  cmd[1].addr = priv->addr;
  cmd[1].flags = I2C_M_NOSTART;
  cmd[1].buffer = buf;
  cmd[1].length = nbytes;

  /* Send command over the wire */

  return I2C_TRANSFER(priv->i2c, cmd, 2);
}

/****************************************************************************
 * Name: lsm6dso32_read_bytes
 *
 * Description:
 *   Read bytes from the LSM6DSO32 sensor. Reading more than one byte will
 *   read from sequential registers starting at the provided address.
 *
 * Input Parameters:
 *   priv    - The instance of the LSM6DSO32 sensor.
 *   addr    - The register address to read from.
 *   buf     - The buffer of data to read into.
 *   nbytes  - The number of bytes to read into the buffer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm6dso32_read_bytes(FAR struct lsm6dso32_dev_s *priv,
                                uint8_t addr, void *buf, size_t nbytes)
{
  struct i2c_msg_s cmd[2];

  /* Register addressing part of command. */

  cmd[0].frequency = CONFIG_SENSORS_LSM6DSO32_I2C_FREQUENCY;
  cmd[0].addr = priv->addr;
  cmd[0].flags = I2C_M_NOSTOP;
  cmd[0].buffer = &addr;
  cmd[0].length = sizeof(addr);

  /* Read data into buffer. */

  cmd[1].frequency = CONFIG_SENSORS_LSM6DSO32_I2C_FREQUENCY;
  cmd[1].addr = priv->addr;
  cmd[1].flags = I2C_M_READ;
  cmd[1].buffer = buf;
  cmd[1].length = nbytes;

  /* Send command over the wire */

  return I2C_TRANSFER(priv->i2c, cmd, 2);
}

/****************************************************************************
 * Name: lsm6dso32_set_bits
 *
 * Description:
 *   Read current value of desired register and change specified bits
 *   while preserving previous ones.
 *   NOTE: Clear operation performed before set operation.
 *
 * Input Parameters:
 *   priv    - The instance of the LSM6DSO32 sensor.
 *   addr    - The register address being changed.
 *   set_bits - A mask of the bits to be set to 1.
 *   clear_bits  - A mask of the bits to be set to 0.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm6dso32_set_bits(FAR struct lsm6dso32_dev_s *priv, uint8_t addr,
                              uint8_t set_bits, uint8_t clear_bits)
{
  int err;
  uint8_t reg;

  err = lsm6dso32_read_bytes(priv, addr, &reg, sizeof(reg));

  if (err < 0)
    {
      return err;
    }

  reg = (reg & ~clear_bits) | set_bits;
  return lsm6dso32_write_bytes(priv, addr, &reg, sizeof(reg));
}

/****************************************************************************
 * Name: accel_set_odr
 *
 * Description:
 *      Sets the accelerometer ODR.
 *
 ****************************************************************************/

static int accel_set_odr(FAR struct lsm6dso32_dev_s *dev,
                         enum lsm6dso32_odr_e odr)
{
  int err;

  err = lsm6dso32_set_bits(dev, CTRL1_XL, (odr & 0xf) << 4, 0xf0);

  if (err < 0)
    {
      return err;
    }

  dev->accel.odr = odr;
  return err;
}

/****************************************************************************
 * Name: gyro_set_odr
 *
 * Description:
 *      Sets the gyroscope ODR.
 *
 ****************************************************************************/

static int gyro_set_odr(FAR struct lsm6dso32_dev_s *dev,
                        enum lsm6dso32_odr_e odr)
{
  int err;

  DEBUGASSERT(odr != ODR_1_6HZ); /* Invalid setting for gyroscope */

  err = lsm6dso32_set_bits(dev, CTRL2_G, (odr & 0x0f) << 4, 0xf0);

  if (err < 0)
    {
      return err;
    }

  dev->gyro.odr = odr;

  return err;
}

/****************************************************************************
 * Name: accel_set_fsr
 *
 * Description:
 *      Sets the accelerometer FSR.
 *
 ****************************************************************************/

static int accel_set_fsr(FAR struct lsm6dso32_dev_s *dev,
                         enum lsm6dso32_fsr_xl_e fsr)
{
  int err;

  err = lsm6dso32_set_bits(dev, CTRL1_XL, (fsr & 0x3) << 2, 0x0c);

  if (err < 0)
    {
      return err;
    }

  dev->accel.fsr = fsr;
  return err;
}

/****************************************************************************
 * Name: gyro_set_fsr
 *
 * Description:
 *      Sets the gyroscope FSR.
 *
 ****************************************************************************/

static int gyro_set_fsr(FAR struct lsm6dso32_dev_s *dev,
                        enum lsm6dso32_fsr_gyro_e fsr)
{
  int err;
  err = lsm6dso32_set_bits(dev, CTRL2_G, (fsr & 0x7) << 1, 0x0c);

  if (err < 0)
    {
      return err;
    }

  dev->gyro.fsr = fsr;
  return err;
}

/****************************************************************************
 * Name: gyro_int_enable
 *
 * Description:
 *      Enables/disables the gyroscope interrupt.
 *
 ****************************************************************************/

static int gyro_int_enable(FAR struct lsm6dso32_dev_s *dev, bool enable)
{
  int err;
  uint8_t enable_bits = enable ? 0x2 : 0x0;
  uint8_t disable_bits = enable ? 0x0 : 0x2;

  err = lsm6dso32_set_bits(dev, INT_CTRL[dev->gyro.intpin], enable_bits,
                           disable_bits);
  if (err < 0)
    {
      return err;
    }

  dev->gyro.interrupts = enable; /* We succeeded, update state */
  return err;
}

/****************************************************************************
 * Name: accel_int_enable
 *
 * Description:
 *      Enables/disables the accelerometer interrupt.
 *
 ****************************************************************************/

static int accel_int_enable(FAR struct lsm6dso32_dev_s *dev, bool enable)
{
  int err;
  uint8_t enable_bits = enable ? 0x1 : 0x0;
  uint8_t disable_bits = enable ? 0x0 : 0x1;

  err = lsm6dso32_set_bits(dev, INT_CTRL[dev->accel.intpin], enable_bits,
                           disable_bits);
  if (err < 0)
    {
      return err;
    }

  dev->accel.interrupts = enable; /* We succeeded, update state */
  return err;
}

/****************************************************************************
 * Name: lsm6dso32_convert_temp
 *
 * Description:
 *   Converts raw temperature reading into units of degrees Celsius.
 *
 ****************************************************************************/

static float lsm6dso32_convert_temp(int16_t temp)
{
  return (float)((temp / 256) + 25);
}

/****************************************************************************
 * Name: lsm6dso32_read_gyro
 *
 * Description:
 *   Reads gyroscope data into UORB structure.
 *
 ****************************************************************************/

static int lsm6dso32_read_gyro(FAR struct lsm6dso32_dev_s *dev,
                               FAR struct sensor_gyro *data)
{
  int16_t raw_data[4]; /* Holds 1 temp, 3 gyro (xyz) */
  int err;

  err = lsm6dso32_read_bytes(dev, OUT_TEMP_L, raw_data, sizeof(raw_data));
  if (err < 0)
    {
      return err;
    }

  /* Convert data into the format required */

  data->timestamp = sensor_get_timestamp();
  data->temperature = lsm6dso32_convert_temp(raw_data[0]);
  data->x =
      (float)(raw_data[1]) * FSR_GYRO_SENS[dev->gyro.fsr] - dev->gy_off[0];
  data->y =
      (float)(raw_data[2]) * FSR_GYRO_SENS[dev->gyro.fsr] - dev->gy_off[1];
  data->z =
      (float)(raw_data[3]) * FSR_GYRO_SENS[dev->gyro.fsr] - dev->gy_off[2];

  return err;
}

/****************************************************************************
 * Name: lsm6dso32_read_accel
 *
 * Description:
 *   Reads accelerometer data into UORB structure.
 *
 ****************************************************************************/

static int lsm6dso32_read_accel(FAR struct lsm6dso32_dev_s *dev,
                                FAR struct sensor_accel *data)
{
  int16_t raw_data[3]; /* 3 accel (xyz) */
  int16_t raw_temp;    /* Temperature */
  int err;

  /* Get accelerometer data */

  err = lsm6dso32_read_bytes(dev, OUTX_L_A, raw_data, sizeof(raw_data));
  if (err < 0)
    {
      return err;
    }

  /* Get temperature data TODO can I bundle this with gyro by decoupling? */

  err = lsm6dso32_read_bytes(dev, OUT_TEMP_L, &raw_temp, sizeof(raw_temp));
  if (err < 0)
    {
      return err;
    }

  /* Convert data into the required format */

  data->timestamp = sensor_get_timestamp();
  data->temperature = lsm6dso32_convert_temp(raw_temp);
  data->x = (float)(raw_data[0]) * FSR_XL_SENS[dev->accel.fsr];
  data->y = (float)(raw_data[1]) * FSR_XL_SENS[dev->accel.fsr];
  data->z = (float)(raw_data[2]) * FSR_XL_SENS[dev->accel.fsr];

  return err;
}

/****************************************************************************
 * Name: push_gyro
 *
 * Description:
 *   Push gyro data to the UORB upper half.
 *
 ****************************************************************************/

static int push_gyro(FAR struct lsm6dso32_dev_s *dev)
{
  int err;
  struct sensor_gyro data;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  err = lsm6dso32_read_gyro(dev, &data);
  if (err < 0)
    {
      goto early_ret;
    }

  dev->gyro.lower.push_event(dev->gyro.lower.priv, &data, sizeof(data));

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: push_accel
 *
 * Description:
 *   Push accelerometer data to the UORB upper half.
 *
 ****************************************************************************/

static int push_accel(FAR struct lsm6dso32_dev_s *dev)
{
  int err;
  struct sensor_accel data;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  err = lsm6dso32_read_accel(dev, &data);
  if (err < 0)
    {
      goto early_ret;
    }

  dev->accel.lower.push_event(dev->accel.lower.priv, &data, sizeof(data));

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: gyro_worker
 *
 * Description:
 *   Worker thread called by gyroscope interrupt handler.
 *
 ****************************************************************************/

static void gyro_worker(FAR void *arg)
{
  push_gyro(arg);
}

/****************************************************************************
 * Name: accel_worker
 *
 * Description:
 *   Worker thread called by accelerometer interrupt handler.
 *
 ****************************************************************************/

static void accel_worker(FAR void *arg)
{
  push_accel(arg);
}

/****************************************************************************
 * Name: gyro_int_handler
 *
 * Description:
 *   Interrupt handler for gyroscope interrupts.
 *
 ****************************************************************************/

static int gyro_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lsm6dso32_dev_s *dev = (FAR struct lsm6dso32_dev_s *)(arg);
  int err;
  (void)(context);

  DEBUGASSERT(arg != NULL);

  /* Start high priority worker thread */

  err = work_queue(HPWORK, &dev->gyro.work, &gyro_worker, dev, 0);

  if (err < 0)
    {
      snerr("Could not queue LSM6DSO32 gyro work queue: %d\n", err);
    }

  return err;
}

/****************************************************************************
 * Name: accel_int_handler
 *
 * Description:
 *   Interrupt handler for accelerometer interrupts.
 *
 ****************************************************************************/

static int accel_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lsm6dso32_dev_s *dev = (FAR struct lsm6dso32_dev_s *)(arg);
  int err;
  (void)(context);

  DEBUGASSERT(arg != NULL);

  /* Start high priority worker thread */

  err = work_queue(HPWORK, &dev->accel.work, &accel_worker, dev, 0);

  if (err < 0)
    {
      snerr("Could not queue LSM6DSO32 accel work queue: %d\n", err);
    }

  return err;
}

/****************************************************************************
 * Name: gyro_thread
 *
 * Description:
 *   Polling thread for gyroscope measurements
 *
 ****************************************************************************/

static int gyro_thread(int argc, char **argv)
{
  FAR struct lsm6dso32_dev_s *dev =
      (FAR struct lsm6dso32_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int err = 0;

  while (true)
    {
      /* If the sensor is disabled we wait indefinitely */

      if (!dev->gyro.enabled)
        {
          err = nxsem_wait(&dev->gyro.run);
          if (err < 0)
            {
              continue;
            }
        }

      /* If the sensor is enabled, grab some data */

      err = push_gyro(dev);
      if (err < 0)
        {
          continue;
        }

      /* Wait for next measurement cycle */

      nxsig_usleep(ODR_INTERVAL[dev->gyro.odr]);
    }

  return err;
}

/****************************************************************************
 * Name: accel_thread
 *
 * Description:
 *   Polling thread for accelerometer measurements.
 *
 ****************************************************************************/

static int accel_thread(int argc, char **argv)
{
  FAR struct lsm6dso32_dev_s *dev =
      (FAR struct lsm6dso32_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int err = 0;

  while (true)
    {
      /* If the sensor is disabled we wait indefinitely */

      if (!dev->accel.enabled)
        {
          err = nxsem_wait(&dev->accel.run);
          if (err < 0)
            {
              continue;
            }
        }

      /* If the sensor is enabled, grab some data */

      err = push_accel(dev);
      if (err < 0)
        {
          continue;
        }

      /* Wait for next measurement cycle */

      nxsig_usleep(ODR_INTERVAL[dev->accel.odr]);
    }

  return err;
}

/****************************************************************************
 * Name: lsm6dso32_activate
 ****************************************************************************/

static int lsm6dso32_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable)
{
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);
  FAR struct lsm6dso32_dev_s *dev = sens->dev;
  bool start_thread = false;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
      snerr("Failed to deactivate/activate LSM6DSO32");
    }

  /* Start collecting data continuously and enable thread if not already
   * enabled
   */

  if (enable && !sens->enabled)
    {
      start_thread = true;

      /* Set to a relatively low sampling rate to start up */

      if (lower->type == SENSOR_TYPE_GYROSCOPE)
        {
          err = gyro_set_odr(dev, ODR_12_5HZ);
        }
      else
        {
          err = accel_set_odr(dev, ODR_12_5HZ);
        }

      if (err < 0)
        {
          goto early_ret;
        }
    }

  /* Turn off sensor if we're disabling */

  if (!enable && sens->enabled)
    {
      if (lower->type == SENSOR_TYPE_GYROSCOPE)
        {
          err = gyro_set_odr(dev, ODR_OFF);
        }
      else
        {
          err = accel_set_odr(dev, ODR_OFF);
        }

      if (err < 0)
        {
          goto early_ret;
        }
    }

  /* If we got here, there was no error, we can record the activation state */

  sens->enabled = enable;

  /* Wake up polling thread if required */

  if (start_thread)
    {
      sninfo("Waking up LSM6DSO32 polling thread");
      err = nxsem_post(&sens->run);
    }

  sninfo("LSM6DSO32 activated");
early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: gyro_selftest
 *
 * Description:
 *   Performs the gyroscope self test as per LSM6DSO32 AN5473.
 *
 ****************************************************************************/

static int gyro_selftest(FAR struct lsm6dso32_dev_s *dev)
{
  int err;
  int restore_err;

  uint8_t prev_control_regs[10];
  uint8_t test_control_regs[10] =
  {
    0x0, 0x5c, 0x44, 0x0, 0x0,
    0x0, 0x0,  0x0,  0x0, 0x0
  };

  uint8_t status;
  int16_t gyro_tmp[3];
  int16_t gyro_nost[3];
  int16_t gyro_st[3];

  memset(gyro_nost, 0, sizeof(gyro_nost));
  memset(gyro_st, 0, sizeof(gyro_st));

  /* Exclusive access */

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Store previous register states to restore sensor state after test. */

  err = lsm6dso32_read_bytes(dev, CTRL1_XL, prev_control_regs,
                             sizeof(prev_control_regs));
  if (err < 0)
    {
      goto early_ret; /* No sensor state to restore */
    }

  /* Put sensor in test state */

  err = lsm6dso32_write_bytes(dev, CTRL1_XL, test_control_regs,
                              sizeof(test_control_regs));
  if (err < 0)
    {
      /* Could have partially failed only, attempt to restore */

      goto early_restore;
    }

  /* Wait for 100ms for stable output */

  nxsig_usleep(100000);

  /* Discard first measurement */

  err = lsm6dso32_read_bytes(dev, OUTX_L_G, gyro_tmp, sizeof(gyro_tmp));
  if (err < 0)
    {
      goto early_restore;
    }

  /* Get "no self-test" samples */

  for (int8_t i = 0; i < GYRO_SELFTEST_ROUNDS; i++)
    {
      /* Check if data is available, if not just repeat without incrementing
       * count.
       */

      err = lsm6dso32_read_bytes(dev, STATUS_REG, &status, sizeof(status));
      if (err < 0)
        {
          goto early_restore;
        }

      if (!(status & BIT_STATUS_GDA))
        {
          i--;
          continue;
        }

      /* Get measurement */

      err = lsm6dso32_read_bytes(dev, OUTX_L_G, gyro_tmp, sizeof(gyro_tmp));
      if (err < 0)
        {
          goto early_restore;
        }

      gyro_nost[0] += gyro_tmp[0];
      gyro_nost[1] += gyro_tmp[1];
      gyro_nost[2] += gyro_tmp[2];
    }

  /* Enable self-test */

  err = lsm6dso32_set_bits(dev, CTRL5_C, BIT_G_ST_POS, BIT_G_ST_POS);
  if (err < 0)
    {
      goto early_restore;
    }

  /* Wait 100ms for stable output */

  nxsig_usleep(100000);

  /* Discard first measurement */

  err = lsm6dso32_read_bytes(dev, OUTX_L_G, gyro_tmp, sizeof(gyro_tmp));
  if (err < 0)
    {
      goto early_restore;
    }

  /* Get self-test samples */

  for (int8_t i = 0; i < GYRO_SELFTEST_ROUNDS; i++)
    {
      /* Check if data is available, if not just repeat without incrementing
       * count
       */

      err = lsm6dso32_read_bytes(dev, STATUS_REG, &status, sizeof(status));
      if (err < 0)
        {
          goto early_restore;
        }

      if (!(status & BIT_STATUS_GDA))
        {
          i--;
          continue;
        }

      /* Get measurement */

      err = lsm6dso32_read_bytes(dev, OUTX_L_G, gyro_tmp, sizeof(gyro_tmp));
      if (err < 0)
        {
          goto early_restore;
        }

      gyro_st[0] += gyro_tmp[0];
      gyro_st[1] += gyro_tmp[1];
      gyro_st[2] += gyro_tmp[2];
    }

  /* Average data */

  gyro_st[0] /= GYRO_SELFTEST_ROUNDS;
  gyro_st[1] /= GYRO_SELFTEST_ROUNDS;
  gyro_st[2] /= GYRO_SELFTEST_ROUNDS;
  gyro_nost[0] /= GYRO_SELFTEST_ROUNDS;
  gyro_nost[1] /= GYRO_SELFTEST_ROUNDS;
  gyro_nost[2] /= GYRO_SELFTEST_ROUNDS;

  /* Get absolute difference */

  gyro_st[0] = abs(gyro_st[0] - gyro_nost[0]);
  gyro_st[1] = abs(gyro_st[1] - gyro_nost[1]);
  gyro_st[2] = abs(gyro_st[2] - gyro_nost[2]);

  /* Check for any results outside of bounds */

  if (!(GYRO_ST_MIN <= gyro_st[0] && gyro_st[0] <= GYRO_ST_MAX &&
        GYRO_ST_MIN <= gyro_st[1] && gyro_st[1] <= GYRO_ST_MAX &&
        GYRO_ST_MIN <= gyro_st[2] && gyro_st[2] <= GYRO_ST_MAX))
    {
      snerr("LSM6DSO32 failed gyro self-test (%d X, %d Y, %d Z).",
            gyro_st[0], gyro_st[1], gyro_st[2]);
      err = -EAGAIN; /* Failed self-test */
    }

  /* Turn off self-test and wait for stability */

  err = lsm6dso32_set_bits(dev, CTRL5_C, 0, BIT_G_ST_POS);
  if (err < 0)
    {
      snerr("Couldn't turn off gyro self test: %d", err);
      goto early_restore;
    }

  nxsig_usleep(100000);

early_restore:

  /* Restore previous registers */

  restore_err = lsm6dso32_write_bytes(dev, CTRL1_XL, prev_control_regs,
                                      sizeof(prev_control_regs));
  if (!err && restore_err < 0)
    {
      /* If we didn't fail earlier in the self-test, we want to report any
       * restoration errors
       */

      err = restore_err;
      snerr("Failed to restore LSM6DSO32 state after gyro selftest: %d\n",
            err);
    }

  nxsig_usleep(100000);

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
};

/****************************************************************************
 * Name: accel_selftest
 *
 * Description:
 *   Performs the accelerometer self test as per LSM6DSO32 AN5473.
 *
 ****************************************************************************/

static int accel_selftest(FAR struct lsm6dso32_dev_s *dev)
{
  int err;
  int restore_err;

  uint8_t prev_control_regs[10];
  uint8_t test_control_regs[10] =
  {
    0x30, 0x00, 0x44, 0x0, 0x0,
    0x0,  0x0,  0x0,  0x0, 0x0
  };

  uint8_t status;
  int16_t accel_tmp[3];
  int16_t accel_nost[3];
  int16_t accel_st[3];

  memset(accel_nost, 0, sizeof(accel_nost));
  memset(accel_st, 0, sizeof(accel_st));

  /* Exclusive access */

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Store previous register states to restore sensor state after test. */

  err = lsm6dso32_read_bytes(dev, CTRL1_XL, prev_control_regs,
                             sizeof(prev_control_regs));
  if (err < 0)
    {
      goto early_ret; /* No sensor state to restore */
    }

  /* Put sensor in test state */

  err = lsm6dso32_write_bytes(dev, CTRL1_XL, test_control_regs,
                              sizeof(test_control_regs));
  if (err < 0)
    {
      /* Could have partially failed only, attempt to restore */

      goto early_restore;
    }

  /* Wait for 100ms for stable output */

  nxsig_usleep(100000);

  /* Discard first measurement */

  err = lsm6dso32_read_bytes(dev, OUTX_L_A, accel_tmp, sizeof(accel_tmp));
  if (err < 0)
    {
      goto early_restore;
    }

  /* Get "no self-test" samples */

  for (int8_t i = 0; i < XL_SELFTEST_ROUNDS; i++)
    {
      /* Check if data is available, if not just repeat without incrementing
       * count.
       */

      err = lsm6dso32_read_bytes(dev, STATUS_REG, &status, sizeof(status));
      if (err < 0)
        {
          goto early_restore;
        }

      if (!(status & BIT_STATUS_XLDA))
        {
          i--;
          continue;
        }

      /* Get measurement */

      err = lsm6dso32_read_bytes(dev, OUTX_L_A, accel_tmp,
                                 sizeof(accel_tmp));
      if (err < 0)
        {
          goto early_restore;
        }

      accel_nost[0] += accel_tmp[0];
      accel_nost[1] += accel_tmp[1];
      accel_nost[2] += accel_tmp[2];
    }

  /* Enable self-test */

  err = lsm6dso32_set_bits(dev, CTRL5_C, BIT_XL_ST_POS, BIT_XL_ST_POS);
  if (err < 0)
    {
      goto early_restore;
    }

  /* Wait 100ms for stable output */

  nxsig_usleep(100000);

  /* Discard first measurement */

  err = lsm6dso32_read_bytes(dev, OUTX_L_A, accel_tmp, sizeof(accel_tmp));
  if (err < 0)
    {
      goto early_restore;
    }

  /* Get self-test samples */

  for (int8_t i = 0; i < XL_SELFTEST_ROUNDS; i++)
    {
      /* Check if data is available, if not just repeat without incrementing
       * count.
       */

      err = lsm6dso32_read_bytes(dev, STATUS_REG, &status, sizeof(status));
      if (err < 0)
        {
          goto early_restore;
        }

      if (!(status & BIT_STATUS_XLDA))
        {
          i--;
          continue;
        }

      /* Get measurement */

      err = lsm6dso32_read_bytes(dev, OUTX_L_A, accel_tmp,
                                 sizeof(accel_tmp));
      if (err < 0)
        {
          goto early_restore;
        }

      accel_st[0] += accel_tmp[0];
      accel_st[1] += accel_tmp[1];
      accel_st[2] += accel_tmp[2];
    }

  /* Average data */

  accel_st[0] /= XL_SELFTEST_ROUNDS;
  accel_st[1] /= XL_SELFTEST_ROUNDS;
  accel_st[2] /= XL_SELFTEST_ROUNDS;
  accel_nost[0] /= XL_SELFTEST_ROUNDS;
  accel_nost[1] /= XL_SELFTEST_ROUNDS;
  accel_nost[2] /= XL_SELFTEST_ROUNDS;

  /* Get absolute difference */

  accel_st[0] = abs(accel_st[0] - accel_nost[0]);
  accel_st[1] = abs(accel_st[1] - accel_nost[1]);
  accel_st[2] = abs(accel_st[2] - accel_nost[2]);

  /* Check for any results outside of bounds */

  if (!(XL_ST_MIN <= accel_st[0] && accel_st[0] <= XL_ST_MAX &&
        XL_ST_MIN <= accel_st[1] && accel_st[1] <= XL_ST_MAX &&
        XL_ST_MIN <= accel_st[2] && accel_st[2] <= XL_ST_MAX))
    {
      snerr("LSM6DSO32 failed accel self-test (%d X, %d Y, %d Z)\n",
            accel_st[0], accel_st[1], accel_st[2]);
      err = -EAGAIN; /* Failed self-test */
    }

  /* Turn off self-test and wait for stability */

  err = lsm6dso32_set_bits(dev, CTRL5_C, 0, BIT_XL_ST_POS);
  if (err < 0)
    {
      snerr("Couldn't turn off gyro self test: %d\n", err);
      goto early_restore;
    }

  nxsig_usleep(100000);

early_restore:

  /* Restore previous registers */

  restore_err = lsm6dso32_write_bytes(dev, CTRL1_XL, prev_control_regs,
                                      sizeof(prev_control_regs));
  if (!err && restore_err < 0)
    {
      /* If we didn't fail earlier in the self-test, we want to report any
       * restoration errors
       */

      err = restore_err;
      snerr("Failed to restore LSM6DSO32 state after accel selftest: %d\n",
            err);
    }

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
};

/****************************************************************************
 * Name: lsm6dso32_set_interval
 ****************************************************************************/

static int lsm6dso32_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR uint32_t *period_us)
{
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);
  FAR struct lsm6dso32_dev_s *dev = sens->dev;
  int err;
  enum lsm6dso32_odr_e odr;

  if (*period_us >= 625000 && lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      /* 1.6Hz is requested and this is the accelerometer: if we're low power
       * mode 1.6Hz will apply, but if we're in high performance mode then
       * the nearest rate of 12.5Hz will be chosen.
       */

      odr = ODR_1_6HZ;
    }
  else if (*period_us >= 80000)
    {
      odr = ODR_12_5HZ;
    }
  else if (*period_us >= 38462 && *period_us < 80000)
    {
      odr = ODR_26HZ;
    }
  else if (*period_us >= 19231 && *period_us < 38462)
    {
      odr = ODR_52HZ;
    }
  else if (*period_us >= 9615 && *period_us < 19231)
    {
      odr = ODR_104HZ;
    }
  else if (*period_us >= 4808 && *period_us < 9615)
    {
      odr = ODR_208HZ;
    }
  else if (*period_us >= 2404 && *period_us < 4808)
    {
      odr = ODR_416HZ;
    }
  else if (*period_us >= 1200 && *period_us < 2404)
    {
      odr = ODR_833HZ;
    }
  else if (*period_us >= 602 && *period_us < 1200)
    {
      odr = ODR_1660HZ;
    }
  else if (*period_us >= 300 && *period_us < 602)
    {
      odr = ODR_3330HZ;
    }
  else
    {
      odr = ODR_6660HZ;
    }

  /* Get exclusive device access before setting the ODR */

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      err = accel_set_odr(dev, odr);
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      err = gyro_set_odr(dev, odr);
    }

  if (err < 0)
    {
      goto early_ret;
    }

  /* Only set the interval value if successful */

  *period_us = ODR_INTERVAL[odr];

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lsm6dso32_set_info
 ****************************************************************************/

static int lsm6dso32_get_info(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR struct sensor_device_info_s *info)
{
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);

  memset(info, 0, sizeof(struct sensor_device_info_s));
  info->version = 0;
  info->power = 0.55f; /* 0.55mA in high performance */
  memcpy(info->name, "LSM6DSO32", sizeof("LSM6DSO32"));
  memcpy(info->vendor, "STMicro", sizeof("STMicro"));

  /* TODO FIFO information once implemented */

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      info->resolution = FSR_GYRO_SENS[sens->fsr];
      info->max_range = FSR_GYRO_SENS[sens->fsr] * INT16_MAX;
      info->min_delay = (int32_t)ODR_INTERVAL[ODR_12_5HZ];
      info->max_delay = (int32_t)ODR_INTERVAL[ODR_12_5HZ];
    }
  else if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      info->resolution = FSR_XL_SENS[sens->fsr];
      info->max_range = FSR_XL_SENS[sens->fsr] * INT16_MAX;
      info->max_delay = (int32_t)ODR_INTERVAL[ODR_12_5HZ];
      info->min_delay = (int32_t)ODR_INTERVAL[ODR_12_5HZ];
    }

  return 0;
}

/****************************************************************************
 * Name: lsm6dso32_control
 ****************************************************************************/

static int lsm6dso32_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);
  FAR struct lsm6dso32_dev_s *dev = sens->dev;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  switch (cmd)
    {
      /* Read WHO_AM_I value into 8-bit unsigned integer buffer */

    case SNIOC_WHO_AM_I:
      {
        uint8_t *id = (uint8_t *)(arg);
        if (id == NULL)
          {
            err = -EINVAL;
            break;
          }

        err = lsm6dso32_read_bytes(dev, WHO_AM_I, id, sizeof(uint8_t));
      }
      break;

    case SNIOC_SETFULLSCALE:
      {
        /* Accelerometer FSR */

        if (lower->type == SENSOR_TYPE_ACCELEROMETER)
          {
            switch (arg)
              {
              case 4:
                err = accel_set_fsr(dev, LSM6DSO32_FSR_XL_4G);
                break;
              case 8:
                err = accel_set_fsr(dev, LSM6DSO32_FSR_XL_8G);
                break;
              case 16:
                err = accel_set_fsr(dev, LSM6DSO32_FSR_XL_16G);
                break;
              case 32:
                err = accel_set_fsr(dev, LSM6DSO32_FSR_XL_32G);
                break;
              default:
                err = -EINVAL;
                break;
              }
          }

        /* Gyroscope FSR */

        else if (lower->type == SENSOR_TYPE_GYROSCOPE)
          {
            switch (arg)
              {
              case 125:
                err = gyro_set_fsr(dev, LSM6DSO32_FSR_GY_125DPS);
                break;
              case 250:
                err = gyro_set_fsr(dev, LSM6DSO32_FSR_GY_250DPS);
                break;
              case 500:
                err = gyro_set_fsr(dev, LSM6DSO32_FSR_GY_500DPS);
                break;
              case 1000:
                err = gyro_set_fsr(dev, LSM6DSO32_FSR_GY_1000DPS);
                break;
              case 2000:
                err = gyro_set_fsr(dev, LSM6DSO32_FSR_GY_2000DPS);
                break;
              default:
                err = -EINVAL;
                break;
              }
          }
      }
      break;

    default:
      {
        err = -EINVAL;
      }
      break;
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lsm6dso32_selftest
 *
 * Description:
 *     Performs the self-test for either the accelerometer or the gyroscope
 *     as described in the LSM6DSO32 AN5473 application note.
 *
 ****************************************************************************/

static int lsm6dso32_selftest(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, unsigned long arg)
{
  int err;
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);
  FAR struct lsm6dso32_dev_s *dev = sens->dev;

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      err = gyro_selftest(dev);
    }
  else
    {
      err = accel_selftest(dev);
    }

  return err;
}

/****************************************************************************
 * Name: lsm6dso32_set_calibvalue
 *
 * Description:
 *    Sets an offset for the gyro or the accelerometer. When called on the
 *    gyro lower half, the argument is a pointer to 3 floats representing the
 *    gyro offset to be subtracted in the XYZ axes in rad/s. When called on
 *    the accel lower half, the argument is a pointer to 3 floats
 *    representing the accel offset to be subtracted in the XYZ axes in
 *    m/s^2.
 *
 ****************************************************************************/

static int lsm6dso32_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    unsigned long arg)
{
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);
  FAR struct lsm6dso32_dev_s *dev = sens->dev;
  int err;
  FAR float *offsets = (FAR float *)(arg);
  float converted[3];
  int8_t regvals[3];

  DEBUGASSERT(offsets != NULL);

  /* Exclusive access */

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      /* Must be done via software, no registers for offset on sensor */

      dev->gy_off[0] = offsets[0];
      dev->gy_off[1] = offsets[1];
      dev->gy_off[2] = offsets[2];
    }
  else
    {
      /* Based on the input, decide if the weight is 2^{-10}g/LSB or
       * 2^{-6}g/LSB. Convert the offsets in m/s^2 to g for comparison
       * against min/max representable values.
       */

      converted[0] = offsets[0] * (1.0f / (MILLIG_TO_MS2 * 1000.0f));
      converted[1] = offsets[1] * (1.0f / (MILLIG_TO_MS2 * 1000.0f));
      converted[2] = offsets[2] * (1.0f / (MILLIG_TO_MS2 * 1000.0f));

      if ((XL_10_W_MIN <= converted[0] && converted[0] <= XL_10_W_MAX) &&
          (XL_10_W_MIN <= converted[1] && converted[1] <= XL_10_W_MAX) &&
          (XL_10_W_MIN <= converted[2] && converted[2] <= XL_10_W_MAX))
        {
          /* We can use the more precise weight of 2^-10 */

          err = lsm6dso32_set_bits(dev, CTRL6_C, 0, BIT_USR_OFF_W);
          if (err < 0)
            {
              snerr("Failed to set LSM6DSO32 offset weight: %d\n", err);
              goto early_ret;
            }

          /* Convert the offset from 1g/LSB to 2^{-10}g/LSB
           * NOTE: The -1 is because for some reason the offsets are always
           * off by a factor of 2. I cannot find this mentioned anywhere.
           */

          regvals[0] = (int8_t)(converted[0] * (1 << (10 - 1)));
          regvals[1] = (int8_t)(converted[1] * (1 << (10 - 1)));
          regvals[2] = (int8_t)(converted[2] * (1 << (10 - 1)));
        }
      else
        {
          /* Use the less precise weight of 2^-6 */

          err = lsm6dso32_set_bits(dev, CTRL6_C, BIT_USR_OFF_W, 0);
          if (err < 0)
            {
              snerr("Failed to set LSM6DSO32 offset weight: %d\n", err);
              goto early_ret;
            }

          /* Convert offset from 1g/LSB to 2^{-6}g/LSB
           * NOTE: The -1 is because for some reason the offsets are always
           * off by a factor of 2. I cannot find this mentioned anywhere.
           */

          regvals[0] = (int8_t)(converted[0] * (1 << (6 - 1)));
          regvals[1] = (int8_t)(converted[1] * (1 << (6 - 1)));
          regvals[2] = (int8_t)(converted[2] * (1 << (6 - 1)));
        }

      err = lsm6dso32_write_bytes(dev, X_OFS_USR, regvals, sizeof(regvals));
      if (err < 0)
        {
          snerr("Failed to set LSM6DSO32 user accel offset: %d\n", err);
          goto early_ret;
        }

      /* If all has gone well, enable the user offset */

      err = lsm6dso32_set_bits(dev, CTRL7_G, BIT_USR_OFF_ON_OUT, 0);
      if (err < 0)
        {
          snerr("Failed to enable LSM6DSO32 accel offset: %d\n", err);
          goto early_ret;
        }

      sninfo("Enabled LSM6DSO32 offsets\n");
    }

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dso32_register
 *
 * Description:
 *   Register the LSM6DSO32 character device as a UORB sensor with accel and
 *   gyro topics. If used with interrupts and device registration fails, it
 *   is the caller's responsibility to detach the interrupt handler.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LSM6DSO32
 *   addr    - The I2C address of the LSM6DSO32.
 *   devno   - The device number for the UORB topics registered (i.e.
 *             sensor_accel<n>)
 *   config  - Configuration setup for interrupt-driven or polling driven
 *             data fetching. Leave `*_attach` function NULL to use kthread
 *             polling instead of interrupt handling.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm6dso32_register(FAR struct i2c_master_s *i2c, uint8_t addr,
                       uint8_t devno, struct lsm6dso32_config_s *config)
{
  FAR struct lsm6dso32_dev_s *priv;
  int err;
  FAR char *argv[2];
  char arg1[32];
  int gyro_pid;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == 0x6b || addr == 0x6a);

  /* If HPWORK is not enabled and the attach functions are not NULL, let the
   * user know that HPWORK is required for interrupts.
   */

#if !defined(CONFIG_SCHED_HPWORK)
  if (config->gy_attach != NULL || config->xl_attach != NULL)
    {
      snerr("CONFIG_SCHED_HPWORK required for interrupt driven measuring.");
      return -ENOSYS;
    }
#endif

  /* If we're enabling interrupt driven mode, each sub-sensor must use a
   * different interrupt pin. Same pin for both is not supported by this
   * driver.
   */

  if (config->gy_attach != NULL && config->xl_attach != NULL &&
      (config->gy_int == config->xl_int))
    {
      snerr("Cannot use the same interrupt pin for accel and gyro.");
      return -EINVAL;
    }

  /* Interrupt pins must be one of the valid options */

  DEBUGASSERT(config->gy_int == LSM6DSO32_INT1 ||
              config->gy_int == LSM6DSO32_INT2);
  DEBUGASSERT(config->xl_int == LSM6DSO32_INT1 ||
              config->xl_int == LSM6DSO32_INT2);

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(struct lsm6dso32_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance of LSM6DSO32 driver.\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;

  /* Create mutex */

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("Failed to initialize mutex: %d\n", err);
      goto free_mem;
    }

  /* Create gyro semaphore */

  err = nxsem_init(&priv->gyro.run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to initialize gyro semaphore: %d\n", err);
      goto del_mutex;
    }

  /* Create accel semaphore */

  err = nxsem_init(&priv->accel.run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to initialize accel semaphore: %d\n", err);
      goto del_gyro_sem;
    }

  /* Create gyro lower half */

  priv->gyro.lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->gyro.lower.ops = &g_sensor_ops;
  priv->gyro.enabled = false;
  priv->gyro.odr = ODR_OFF;                 /* Default off */
  priv->gyro.fsr = LSM6DSO32_FSR_GY_125DPS; /* Default 125dps */
  priv->gyro.interrupts = false;
  priv->gyro.intpin = config->gy_int;
  priv->gyro.dev = priv;

  err = sensor_register(&priv->gyro.lower, devno);
  if (err < 0)
    {
      snerr("Failed to register LSM6DSO32 gyroscope lower half: %d\n", err);
      goto del_accel_sem;
    }

  /* Create accel lower half */

  priv->accel.lower.type = SENSOR_TYPE_ACCELEROMETER;
  priv->accel.lower.ops = &g_sensor_ops;
  priv->accel.enabled = false;
  priv->accel.odr = ODR_OFF;             /* Default off */
  priv->accel.fsr = LSM6DSO32_FSR_XL_4G; /* Default 4g */
  priv->accel.interrupts = false;
  priv->accel.intpin = config->xl_int;
  priv->accel.dev = priv;

  err = sensor_register(&priv->accel.lower, devno);
  if (err < 0)
    {
      snerr("Failed to register LSM6DSO32 accelerometer lower half: %d\n",
            err);
      goto unreg_gyro;
    }

  /* Gyroscope measuring setup */

  if (config->gy_attach != NULL)
    {
      /* Register gyro interrupt handler */

      err = config->gy_attach(gyro_int_handler, priv);
      if (err < 0)
        {
          snerr("Failed to register gyro interrupt handler: %d\n", err);
          goto unreg_accel;
        }

      err = gyro_int_enable(priv, true);
      if (err < 0)
        {
          snerr("Failed to register gyro interrupt handler: %d\n", err);
          goto unreg_accel;
        }

      sninfo("LSM6DSO32 gyro interrupt handler attached to %s.",
             config->gy_int == LSM6DSO32_INT1 ? "INT1" : "INT2");
    }
  else
    {
      /* Register gyro polling thread */

      snprintf(arg1, 16, "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      err = kthread_create("lsm6dso32_gy_thread", SCHED_PRIORITY_DEFAULT,
                           CONFIG_SENSORS_LSM6DSO32_THREAD_STACKSIZE,
                           gyro_thread, argv);
      if (err < 0)
        {
          snerr("Failed to register gyro polling thread: %d\n", err);
          goto unreg_accel;
        }

      /* Store PID of this kthread in case we need to unregister it */

      gyro_pid = err;
      sninfo("LSM6DSO32 gyro using polling thread.");
    }

  /* Accelerometer measuring setup */

  if (config->xl_attach != NULL)
    {
      /* Register accel interrupt handler */

      err = config->xl_attach(accel_int_handler, priv);
      if (err < 0)
        {
          snerr("Failed to register accel interrupt handler: %d\n", err);
          goto unreg_gyro_handler;
        }

      err = accel_int_enable(priv, true);
      if (err < 0)
        {
          snerr("Failed to register accel interrupt handler: %d\n", err);
          goto unreg_gyro_handler;
        }

      sninfo("LSM6DSO32 accel interrupt handler attached to %s.",
             config->xl_int == LSM6DSO32_INT1 ? "INT1" : "INT2");
    }
  else
    {
      /* Register accel polling thread */

      snprintf(arg1, 16, "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      err = kthread_create("lsm6dso32_xl_thread", SCHED_PRIORITY_DEFAULT,
                           CONFIG_SENSORS_LSM6DSO32_THREAD_STACKSIZE,
                           accel_thread, argv);
      if (err < 0)
        {
          snerr("Failed to register accel polling thread: %d\n", err);
          goto unreg_gyro_handler;
        }

      sninfo("LSM6DSO32 accel using polling thread.");
    }

  if (err < 0)
    {
    unreg_gyro_handler:
      if (config->xl_attach != NULL)
        {
          kthread_delete(gyro_pid);
        }

    unreg_accel:
      sensor_unregister(&priv->accel.lower, devno);
    unreg_gyro:
      sensor_unregister(&priv->gyro.lower, devno);
    del_accel_sem:
      nxsem_destroy(&priv->accel.run);
    del_gyro_sem:
      nxsem_destroy(&priv->gyro.run);
    del_mutex:
      nxmutex_destroy(&priv->devlock);
    free_mem:
      kmm_free(priv);
      snerr("ERROR: Failed to register LSM6DSO32 driver: %d\n", err);
    }
  else
    {
      sninfo("LSM6DSO32 driver registered!");
    }

  return err;
}
