/****************************************************************************
 * drivers/sensors/lsm6ds3trc_uorb.c
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

#include <nuttx/debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/lsm6ds3trc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>
#include <stdio.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only float data type supported now */

#ifdef CONFIG_SENSORS_USE_B16
#  error fixed-point data type not supported yet
#endif

/* The value that should be in the WHO_AM_I register. */

#define WHO_AM_I_VAL 0x6a

/* Convert milli Gs to m/s^2 */

#define MILLIG_TO_MS2 (0.0098067f)

/* Convert milli-dps to rad/s */

#define MDPS_TO_RADS (3.141592653f / (180.0f * 1000.0f))

/* Registers */

#define WHO_AM_I 0x0f   /* Hard-coded address on I2C bus. */
#define STATUS_REG 0x1e /* The status register */
#define CTRL1_XL 0x10   /* Accel control reg 1 */
#define CTRL2_G 0x11    /* Gyro control reg 2 */
#define CTRL3_C 0x12    /* Control reg 3 */
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

/* Bits */

#define BIT_STATUS_XLDA (1 << 0) /* Accel data ready */
#define BIT_STATUS_GDA (1 << 1)  /* Gyro data ready */
#define BIT_STATUS_TDA (1 << 2)  /* Temp data ready */

#define BIT_INT_DRDY_XL (1 << 0) /* INTn_CTRL: accel data-ready enable */
#define BIT_INT_DRDY_G (1 << 1)  /* INTn_CTRL: gyro data-ready enable */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ODRs common to the accelerometer and the gyroscope. NOTE: this driver
 * does not implement the 1.6Hz low power ODR for the accelerometer.
 */

enum lsm6ds3trc_odr_e
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
};

/* Gyroscope FSRs. Values already carry the FS_125 bit (bit1) combined
 * with the FS_G bits (bits[3:2]), matching CTRL2_G's layout.
 */

enum lsm6ds3trc_fsr_gyro_e
{
  LSM6DS3TRC_FSR_GY_245DPS = 0x0,  /* +-245dps */
  LSM6DS3TRC_FSR_GY_125DPS = 0x1,  /* +-125dps */
  LSM6DS3TRC_FSR_GY_500DPS = 0x2,  /* +-500dps */
  LSM6DS3TRC_FSR_GY_1000DPS = 0x4, /* +-1000dps */
  LSM6DS3TRC_FSR_GY_2000DPS = 0x6, /* +-2000dps */
};

/* Accelerometer FSRs, matching CTRL1_XL's FS_XL bits[3:2] encoding. */

enum lsm6ds3trc_fsr_xl_e
{
  LSM6DS3TRC_FSR_XL_2G = 0x0,  /* +-2g */
  LSM6DS3TRC_FSR_XL_16G = 0x1, /* +-16g */
  LSM6DS3TRC_FSR_XL_4G = 0x2,  /* +-4g */
  LSM6DS3TRC_FSR_XL_8G = 0x3,  /* +-8g */
};

/* Represents a lower half sensor driver of the LSM6DS3TR-C */

struct lsm6ds3trc_sens_s
{
  FAR struct sensor_lowerhalf_s lower; /* Lower-half sensor driver */
  FAR struct lsm6ds3trc_dev_s *dev;    /* Reference to parent device */
  bool enabled;                        /* If this sensor is enabled */
  enum lsm6ds3trc_odr_e odr;           /* Measurement interval of this
                                        * sensor */
  int fsr;                             /* Full scale range of this sensor.
                                        * Can be from either gyro or accel
                                        * FSR enum. */
  sem_t run;                           /* Polling cycle lock (kthread
                                        * mode only) */
};

/* Represents the LSM6DS3TR-C IMU device */

struct lsm6ds3trc_dev_s
{
  struct lsm6ds3trc_sens_s gyro;  /* Gyroscope lower half */
  struct lsm6ds3trc_sens_s accel; /* Accelerometer lower half */
  FAR struct i2c_master_s *i2c;   /* I2C interface. */
  uint8_t addr;                   /* I2C address. */
  mutex_t devlock;
  bool interrupt_mode;            /* True if using the INT pin instead of
                                   * kthread polling */
  enum lsm6ds3trc_int_e int_pin;  /* Shared INT pin (interrupt mode only) */
  struct work_s work;             /* Shared interrupt work queue
                                   * structure -- one burst read serves
                                   * both sub-sensors */
  uint64_t timestamp;             /* When the burst became ready, taken
                                   * in the ISR */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lsm6ds3trc_control(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, int cmd,
                              unsigned long arg);
static int lsm6ds3trc_activate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable);
static int lsm6ds3trc_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *period_us);
static int lsm6ds3trc_get_info(FAR struct sensor_lowerhalf_s *lower,
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
};

/* Accelerometer FSR sensitivities in m/s^2 per LSB */

static const float FSR_XL_SENS[] =
{
  0.061f * MILLIG_TO_MS2, /* 2g */
  0.488f * MILLIG_TO_MS2, /* 16g */
  0.122f * MILLIG_TO_MS2, /* 4g */
  0.244f * MILLIG_TO_MS2, /* 8g */
};

/* Gyro FSR sensitivities in rad/s per LSB */

static const float FSR_GYRO_SENS[] =
{
  8.75f * MDPS_TO_RADS,  /* LSM6DS3TRC_FSR_GY_245DPS */
  4.375f * MDPS_TO_RADS, /* LSM6DS3TRC_FSR_GY_125DPS */
  17.50f * MDPS_TO_RADS, /* LSM6DS3TRC_FSR_GY_500DPS */
  0.0f,                  /* No such setting (3) */
  35.0f * MDPS_TO_RADS,  /* LSM6DS3TRC_FSR_GY_1000DPS */
  0.0f,                  /* No such setting (5) */
  70.0f * MDPS_TO_RADS,  /* LSM6DS3TRC_FSR_GY_2000DPS */
};

/* Interrupt control registers, indexed by lsm6ds3trc_int_e */

static const uint8_t INT_CTRL[] =
{
  INT1_CTRL, /* INT1 */
  INT2_CTRL, /* INT2 */
};

/* Sensor operations. No FIFO yet (single-sample delivery per event), so
 * .fetch stays NULL -- data arrives exclusively via push_event() from
 * either the kthreads or the interrupt worker below. selftest/
 * set_calibvalue/calibrate are left unimplemented for now; the upper
 * half tolerates NULL here.
 */

static const struct sensor_ops_s g_sensor_ops =
{
  .fetch          = NULL,
  .activate       = lsm6ds3trc_activate,
  .control        = lsm6ds3trc_control,
  .set_interval   = lsm6ds3trc_set_interval,
  .selftest       = NULL,
  .set_calibvalue = NULL,
  .calibrate      = NULL,
  .get_info       = lsm6ds3trc_get_info,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6ds3trc_write_bytes
 *
 * Description:
 *   Write bytes to the LSM6DS3TR-C sensor. Providing more than one byte
 *   will write to sequential registers starting at the provided address.
 *
 ****************************************************************************/

static int lsm6ds3trc_write_bytes(FAR struct lsm6ds3trc_dev_s *priv,
                                  uint8_t addr, void *buf, size_t nbytes)
{
  struct i2c_msg_s cmd[2];

  /* Register addressing part of command. */

  cmd[0].frequency = CONFIG_SENSORS_LSM6DS3TRC_I2C_FREQUENCY;
  cmd[0].addr = priv->addr;
  cmd[0].flags = I2C_M_NOSTOP;
  cmd[0].buffer = &addr;
  cmd[0].length = sizeof(addr);

  /* Data to write. */

  cmd[1].frequency = CONFIG_SENSORS_LSM6DS3TRC_I2C_FREQUENCY;
  cmd[1].addr = priv->addr;
  cmd[1].flags = I2C_M_NOSTART;
  cmd[1].buffer = buf;
  cmd[1].length = nbytes;

  return I2C_TRANSFER(priv->i2c, cmd, 2);
}

/****************************************************************************
 * Name: lsm6ds3trc_read_bytes
 *
 * Description:
 *   Read bytes from the LSM6DS3TR-C sensor. Reading more than one byte
 *   will read from sequential registers starting at the provided address.
 *
 ****************************************************************************/

static int lsm6ds3trc_read_bytes(FAR struct lsm6ds3trc_dev_s *priv,
                                 uint8_t addr, void *buf, size_t nbytes)
{
  struct i2c_msg_s cmd[2];

  /* Register addressing part of command. */

  cmd[0].frequency = CONFIG_SENSORS_LSM6DS3TRC_I2C_FREQUENCY;
  cmd[0].addr = priv->addr;
  cmd[0].flags = I2C_M_NOSTOP;
  cmd[0].buffer = &addr;
  cmd[0].length = sizeof(addr);

  /* Read data into buffer. */

  cmd[1].frequency = CONFIG_SENSORS_LSM6DS3TRC_I2C_FREQUENCY;
  cmd[1].addr = priv->addr;
  cmd[1].flags = I2C_M_READ;
  cmd[1].buffer = buf;
  cmd[1].length = nbytes;

  return I2C_TRANSFER(priv->i2c, cmd, 2);
}

/****************************************************************************
 * Name: lsm6ds3trc_set_bits
 *
 * Description:
 *   Read current value of desired register and change specified bits
 *   while preserving previous ones.
 *   NOTE: Clear operation performed before set operation.
 *
 ****************************************************************************/

static int lsm6ds3trc_set_bits(FAR struct lsm6ds3trc_dev_s *priv,
                               uint8_t addr, uint8_t set_bits,
                               uint8_t clear_bits)
{
  int err;
  uint8_t reg;

  err = lsm6ds3trc_read_bytes(priv, addr, &reg, sizeof(reg));
  if (err < 0)
    {
      return err;
    }

  reg = (reg & ~clear_bits) | set_bits;
  return lsm6ds3trc_write_bytes(priv, addr, &reg, sizeof(reg));
}

/****************************************************************************
 * Name: accel_set_odr
 ****************************************************************************/

static int accel_set_odr(FAR struct lsm6ds3trc_dev_s *dev,
                         enum lsm6ds3trc_odr_e odr)
{
  int err;

  err = lsm6ds3trc_set_bits(dev, CTRL1_XL, (odr & 0xf) << 4, 0xf0);
  if (err < 0)
    {
      return err;
    }

  dev->accel.odr = odr;
  return err;
}

/****************************************************************************
 * Name: gyro_set_odr
 ****************************************************************************/

static int gyro_set_odr(FAR struct lsm6ds3trc_dev_s *dev,
                        enum lsm6ds3trc_odr_e odr)
{
  int err;

  err = lsm6ds3trc_set_bits(dev, CTRL2_G, (odr & 0xf) << 4, 0xf0);
  if (err < 0)
    {
      return err;
    }

  dev->gyro.odr = odr;
  return err;
}

/****************************************************************************
 * Name: accel_set_fsr
 ****************************************************************************/

static int accel_set_fsr(FAR struct lsm6ds3trc_dev_s *dev,
                         enum lsm6ds3trc_fsr_xl_e fsr)
{
  int err;

  err = lsm6ds3trc_set_bits(dev, CTRL1_XL, (fsr & 0x3) << 2, 0x0c);
  if (err < 0)
    {
      return err;
    }

  dev->accel.fsr = fsr;
  return err;
}

/****************************************************************************
 * Name: gyro_set_fsr
 ****************************************************************************/

static int gyro_set_fsr(FAR struct lsm6ds3trc_dev_s *dev,
                        enum lsm6ds3trc_fsr_gyro_e fsr)
{
  int err;

  err = lsm6ds3trc_set_bits(dev, CTRL2_G, (fsr & 0x7) << 1, 0x0e);
  if (err < 0)
    {
      return err;
    }

  dev->gyro.fsr = fsr;
  return err;
}

/****************************************************************************
 * Name: lsm6ds3trc_int_enable
 *
 * Description:
 *   Set or clear one sub-sensor's data-ready bit in the shared INTn_CTRL
 *   register (interrupt mode only). Both DRDY_XL and DRDY_G can be OR'd
 *   onto the same pin, so this only ever touches the single bit that
 *   belongs to the caller -- the other sub-sensor's bit, if set, is left
 *   alone by the read-modify-write in lsm6ds3trc_set_bits().
 *
 ****************************************************************************/

static int lsm6ds3trc_int_enable(FAR struct lsm6ds3trc_dev_s *dev,
                                 uint8_t bit, bool enable)
{
  uint8_t reg = INT_CTRL[dev->int_pin];

  return lsm6ds3trc_set_bits(dev, reg, enable ? bit : 0, bit);
}

/****************************************************************************
 * Name: lsm6ds3trc_convert_temp
 ****************************************************************************/

static float lsm6ds3trc_convert_temp(int16_t temp)
{
  return (float)((temp / 256) + 25);
}

/****************************************************************************
 * Name: lsm6ds3trc_read_gyro
 ****************************************************************************/

static int lsm6ds3trc_read_gyro(FAR struct lsm6ds3trc_dev_s *dev,
                                FAR struct sensor_gyro *data)
{
  int16_t raw_data[4]; /* Holds 1 temp, 3 gyro (xyz) */
  int err;

  err = lsm6ds3trc_read_bytes(dev, OUT_TEMP_L, raw_data, sizeof(raw_data));
  if (err < 0)
    {
      return err;
    }

  data->timestamp = sensor_get_timestamp();
  data->temperature = lsm6ds3trc_convert_temp(raw_data[0]);
  data->x = (float)(raw_data[1]) * FSR_GYRO_SENS[dev->gyro.fsr];
  data->y = (float)(raw_data[2]) * FSR_GYRO_SENS[dev->gyro.fsr];
  data->z = (float)(raw_data[3]) * FSR_GYRO_SENS[dev->gyro.fsr];

  return err;
}

/****************************************************************************
 * Name: lsm6ds3trc_read_accel
 ****************************************************************************/

static int lsm6ds3trc_read_accel(FAR struct lsm6ds3trc_dev_s *dev,
                                 FAR struct sensor_accel *data)
{
  int16_t raw_data[3]; /* 3 accel (xyz) */
  int16_t raw_temp;    /* Temperature */
  int err;

  err = lsm6ds3trc_read_bytes(dev, OUTX_L_A, raw_data, sizeof(raw_data));
  if (err < 0)
    {
      return err;
    }

  err = lsm6ds3trc_read_bytes(dev, OUT_TEMP_L, &raw_temp, sizeof(raw_temp));
  if (err < 0)
    {
      return err;
    }

  data->timestamp = sensor_get_timestamp();
  data->temperature = lsm6ds3trc_convert_temp(raw_temp);
  data->x = (float)(raw_data[0]) * FSR_XL_SENS[dev->accel.fsr];
  data->y = (float)(raw_data[1]) * FSR_XL_SENS[dev->accel.fsr];
  data->z = (float)(raw_data[2]) * FSR_XL_SENS[dev->accel.fsr];

  return err;
}

/****************************************************************************
 * Name: push_gyro
 ****************************************************************************/

static int push_gyro(FAR struct lsm6ds3trc_dev_s *dev)
{
  int err;
  struct sensor_gyro data;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  err = lsm6ds3trc_read_gyro(dev, &data);
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
 ****************************************************************************/

static int push_accel(FAR struct lsm6ds3trc_dev_s *dev)
{
  int err;
  struct sensor_accel data;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  err = lsm6ds3trc_read_accel(dev, &data);
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
 * Name: gyro_thread
 *
 * Description:
 *   Polling thread for gyroscope measurements (kthread mode only).
 *
 ****************************************************************************/

static int gyro_thread(int argc, char **argv)
{
  FAR struct lsm6ds3trc_dev_s *dev =
      (FAR struct lsm6ds3trc_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int err = 0;

  while (true)
    {
      if (!dev->gyro.enabled)
        {
          err = nxsem_wait(&dev->gyro.run);
          if (err < 0)
            {
              continue;
            }
        }

      err = push_gyro(dev);
      if (err < 0)
        {
          continue;
        }

      nxsched_usleep(ODR_INTERVAL[dev->gyro.odr]);
    }

  return err;
}

/****************************************************************************
 * Name: accel_thread
 *
 * Description:
 *   Polling thread for accelerometer measurements (kthread mode only).
 *   See gyro_thread.
 *
 ****************************************************************************/

static int accel_thread(int argc, char **argv)
{
  FAR struct lsm6ds3trc_dev_s *dev =
      (FAR struct lsm6ds3trc_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int err = 0;

  while (true)
    {
      if (!dev->accel.enabled)
        {
          err = nxsem_wait(&dev->accel.run);
          if (err < 0)
            {
              continue;
            }
        }

      err = push_accel(dev);
      if (err < 0)
        {
          continue;
        }

      nxsched_usleep(ODR_INTERVAL[dev->accel.odr]);
    }

  return err;
}

/****************************************************************************
 * Name: lsm6ds3trc_worker
 *
 * Description:
 *   Interrupt mode only. Bursts temperature + gyro xyz + accel xyz in one
 *   contiguous I2C read (OUT_TEMP_L..OUTZ_H_A is 14 sequential bytes) and
 *   pushes whichever of the two topics is currently subscribed. Mirrors
 *   the MPU6050 driver's single-worker design instead of running two
 *   independent interrupt paths for one physical chip.
 *
 ****************************************************************************/

static void lsm6ds3trc_worker(FAR void *arg)
{
  FAR struct lsm6ds3trc_dev_s *dev = arg;
  int16_t raw[7]; /* temp, gx, gy, gz, ax, ay, az */
  struct sensor_gyro gyro_data;
  struct sensor_accel accel_data;
  float temp_c;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return;
    }

  err = lsm6ds3trc_read_bytes(dev, OUT_TEMP_L, raw, sizeof(raw));
  nxmutex_unlock(&dev->devlock);

  if (err < 0)
    {
      snerr("ERROR: Failed to read measurement: %d\n", err);
      return;
    }

  temp_c = lsm6ds3trc_convert_temp(raw[0]);

  if (dev->gyro.enabled)
    {
      gyro_data.timestamp = dev->timestamp;
      gyro_data.temperature = temp_c;
      gyro_data.x = (float)raw[1] * FSR_GYRO_SENS[dev->gyro.fsr];
      gyro_data.y = (float)raw[2] * FSR_GYRO_SENS[dev->gyro.fsr];
      gyro_data.z = (float)raw[3] * FSR_GYRO_SENS[dev->gyro.fsr];

      dev->gyro.lower.push_event(dev->gyro.lower.priv, &gyro_data,
                                 sizeof(gyro_data));
    }

  if (dev->accel.enabled)
    {
      accel_data.timestamp = dev->timestamp;
      accel_data.temperature = temp_c;
      accel_data.x = (float)raw[4] * FSR_XL_SENS[dev->accel.fsr];
      accel_data.y = (float)raw[5] * FSR_XL_SENS[dev->accel.fsr];
      accel_data.z = (float)raw[6] * FSR_XL_SENS[dev->accel.fsr];

      dev->accel.lower.push_event(dev->accel.lower.priv, &accel_data,
                                  sizeof(accel_data));
    }
}

/****************************************************************************
 * Name: lsm6ds3trc_interrupt
 *
 * Description:
 *   ISR for the shared INT pin. Timestamps here, where the burst really
 *   became ready -- the I2C read cannot run in interrupt context, so it's
 *   deferred to lsm6ds3trc_worker() on HPWORK.
 *
 ****************************************************************************/

static int lsm6ds3trc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lsm6ds3trc_dev_s *dev = (FAR struct lsm6ds3trc_dev_s *)(arg);
  int err;

  (void)(context);

  DEBUGASSERT(arg != NULL);

  dev->timestamp = sensor_get_timestamp();

  err = work_queue(HPWORK, &dev->work, lsm6ds3trc_worker, dev, 0);
  if (err < 0)
    {
      snerr("Could not queue LSM6DS3TR-C work queue: %d\n", err);
    }

  return err;
}

/****************************************************************************
 * Name: lsm6ds3trc_activate
 ****************************************************************************/

static int lsm6ds3trc_activate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable)
{
  FAR struct lsm6ds3trc_sens_s *sens =
      container_of(lower, FAR struct lsm6ds3trc_sens_s, lower);
  FAR struct lsm6ds3trc_dev_s *dev = sens->dev;
  bool is_gyro = lower->type == SENSOR_TYPE_GYROSCOPE;
  bool start_thread = false;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Start collecting data continuously and enable the thread if not
   * already enabled.
   */

  if (enable && !sens->enabled)
    {
      start_thread = true;

      /* Set to a relatively low sampling rate to start up */

      err = is_gyro ? gyro_set_odr(dev, ODR_52HZ)
                     : accel_set_odr(dev, ODR_52HZ);
      if (err < 0)
        {
          goto early_ret;
        }

      if (dev->interrupt_mode)
        {
          err = lsm6ds3trc_int_enable(dev, is_gyro ? BIT_INT_DRDY_G
                                                    : BIT_INT_DRDY_XL, true);
          if (err < 0)
            {
              goto early_ret;
            }
        }
    }

  /* Turn off the sensor if we're disabling */

  if (!enable && sens->enabled)
    {
      err = is_gyro ? gyro_set_odr(dev, ODR_OFF)
                     : accel_set_odr(dev, ODR_OFF);
      if (err < 0)
        {
          goto early_ret;
        }

      if (dev->interrupt_mode)
        {
          err = lsm6ds3trc_int_enable(dev,
                                       is_gyro ? BIT_INT_DRDY_G :
                                                 BIT_INT_DRDY_XL,
                                       false);
          if (err < 0)
            {
              goto early_ret;
            }
        }
    }

  sens->enabled = enable;

  if (start_thread && !dev->interrupt_mode)
    {
      sninfo("Waking up LSM6DS3TR-C polling thread");
      err = nxsem_post(&sens->run);
    }

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lsm6ds3trc_set_interval
 ****************************************************************************/

static int lsm6ds3trc_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *period_us)
{
  FAR struct lsm6ds3trc_sens_s *sens =
      container_of(lower, FAR struct lsm6ds3trc_sens_s, lower);
  FAR struct lsm6ds3trc_dev_s *dev = sens->dev;
  int err;
  enum lsm6ds3trc_odr_e odr;

  if (*period_us >= 80000)
    {
      odr = ODR_12_5HZ;
    }
  else if (*period_us >= 38462)
    {
      odr = ODR_26HZ;
    }
  else if (*period_us >= 19231)
    {
      odr = ODR_52HZ;
    }
  else if (*period_us >= 9615)
    {
      odr = ODR_104HZ;
    }
  else if (*period_us >= 4808)
    {
      odr = ODR_208HZ;
    }
  else if (*period_us >= 2404)
    {
      odr = ODR_416HZ;
    }
  else if (*period_us >= 1200)
    {
      odr = ODR_833HZ;
    }
  else if (*period_us >= 602)
    {
      odr = ODR_1660HZ;
    }
  else if (*period_us >= 300)
    {
      odr = ODR_3330HZ;
    }
  else
    {
      odr = ODR_6660HZ;
    }

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

  *period_us = ODR_INTERVAL[odr];

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lsm6ds3trc_get_info
 ****************************************************************************/

static int lsm6ds3trc_get_info(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR struct sensor_device_info_s *info)
{
  FAR struct lsm6ds3trc_sens_s *sens =
      container_of(lower, FAR struct lsm6ds3trc_sens_s, lower);

  memset(info, 0, sizeof(struct sensor_device_info_s));
  info->version = 0;
  info->power = 0.29f; /* 0.29mA combo LP @52Hz, AN5130 typical @1.8V */
  memcpy(info->name, "LSM6DS3TR-C", sizeof("LSM6DS3TR-C"));
  memcpy(info->vendor, "STMicro", sizeof("STMicro"));

  /* TODO FIFO information once a future phase implements FIFO drain */

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
 * Name: lsm6ds3trc_control
 ****************************************************************************/

static int lsm6ds3trc_control(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, int cmd,
                              unsigned long arg)
{
  FAR struct lsm6ds3trc_sens_s *sens =
      container_of(lower, FAR struct lsm6ds3trc_sens_s, lower);
  FAR struct lsm6ds3trc_dev_s *dev = sens->dev;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  switch (cmd)
    {
      case SNIOC_WHO_AM_I:
        {
          FAR uint8_t *id = (FAR uint8_t *)(arg);

          if (id == NULL)
            {
              err = -EINVAL;
              break;
            }

          err = lsm6ds3trc_read_bytes(dev, WHO_AM_I, id, sizeof(uint8_t));
        }
        break;

      case SNIOC_SETFULLSCALE:
        {
          if (lower->type == SENSOR_TYPE_ACCELEROMETER)
            {
              switch (arg)
                {
                  case 2:
                    err = accel_set_fsr(dev, LSM6DS3TRC_FSR_XL_2G);
                    break;

                  case 4:
                    err = accel_set_fsr(dev, LSM6DS3TRC_FSR_XL_4G);
                    break;

                  case 8:
                    err = accel_set_fsr(dev, LSM6DS3TRC_FSR_XL_8G);
                    break;

                  case 16:
                    err = accel_set_fsr(dev, LSM6DS3TRC_FSR_XL_16G);
                    break;

                  default:
                    err = -EINVAL;
                    break;
                }
            }
          else if (lower->type == SENSOR_TYPE_GYROSCOPE)
            {
              switch (arg)
                {
                  case 125:
                    err = gyro_set_fsr(dev, LSM6DS3TRC_FSR_GY_125DPS);
                    break;

                  case 245:
                    err = gyro_set_fsr(dev, LSM6DS3TRC_FSR_GY_245DPS);
                    break;

                  case 500:
                    err = gyro_set_fsr(dev, LSM6DS3TRC_FSR_GY_500DPS);
                    break;

                  case 1000:
                    err = gyro_set_fsr(dev, LSM6DS3TRC_FSR_GY_1000DPS);
                    break;

                  case 2000:
                    err = gyro_set_fsr(dev, LSM6DS3TRC_FSR_GY_2000DPS);
                    break;

                  default:
                    err = -EINVAL;
                    break;
                }
            }
          else
            {
              err = -EINVAL;
            }
        }
        break;

      default:
        err = -EINVAL;
        break;
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6ds3trc_register
 ****************************************************************************/

int lsm6ds3trc_register(FAR struct i2c_master_s *i2c, uint8_t addr,
                        uint8_t devno,
                        FAR struct lsm6ds3trc_config_s *config)
{
  FAR struct lsm6ds3trc_dev_s *priv;
  int err;
  FAR char *argv[2];
  char arg1[32];
  int gyro_pid = -1;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(config != NULL);
  DEBUGASSERT(addr == 0x6a || addr == 0x6b);

#if !defined(CONFIG_SCHED_HPWORK)
  if (config->attach != NULL)
    {
      snerr("CONFIG_SCHED_HPWORK required for interrupt driven measuring.");
      return -ENOSYS;
    }
#endif

  priv = kmm_zalloc(sizeof(struct lsm6ds3trc_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance of LSM6DS3TR-C driver.\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;
  priv->interrupt_mode = config->attach != NULL;
  priv->int_pin = config->int_pin;

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("Failed to initialize mutex: %d\n", err);
      goto free_mem;
    }

  err = nxsem_init(&priv->gyro.run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to initialize gyro semaphore: %d\n", err);
      goto del_mutex;
    }

  err = nxsem_init(&priv->accel.run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to initialize accel semaphore: %d\n", err);
      goto del_gyro_sem;
    }

  /* Create gyro lower half */

  priv->gyro.lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->gyro.lower.ops = &g_sensor_ops;
  priv->gyro.lower.nbuffer = CONFIG_SENSORS_LSM6DS3TRC_GYRO_ORB_BUFSIZE;
  priv->gyro.enabled = false;
  priv->gyro.odr = ODR_OFF;
  priv->gyro.fsr = LSM6DS3TRC_FSR_GY_245DPS;
  priv->gyro.dev = priv;

  err = sensor_register(&priv->gyro.lower, devno);
  if (err < 0)
    {
      snerr("Failed to register LSM6DS3TR-C gyroscope lower half: %d\n",
            err);
      goto del_accel_sem;
    }

  /* Create accel lower half */

  priv->accel.lower.type = SENSOR_TYPE_ACCELEROMETER;
  priv->accel.lower.ops = &g_sensor_ops;
  priv->accel.lower.nbuffer = CONFIG_SENSORS_LSM6DS3TRC_ACCEL_ORB_BUFSIZE;
  priv->accel.enabled = false;
  priv->accel.odr = ODR_OFF;
  priv->accel.fsr = LSM6DS3TRC_FSR_XL_4G; /* Default 4g, per project notes */
  priv->accel.dev = priv;

  err = sensor_register(&priv->accel.lower, devno);
  if (err < 0)
    {
      snerr("Failed to register LSM6DS3TR-C accelerometer lower half: "
            "%d\n", err);
      goto unreg_gyro;
    }

  /* Write CTRL1_XL's FS_XL bits to match the software default above --
   * ODR is set later by activate(), but FSR needs to be right from the
   * first sample instead of only after an explicit SNIOC_SETFULLSCALE.
   */

  err = accel_set_fsr(priv, priv->accel.fsr);
  if (err < 0)
    {
      snerr("Failed to set default accel FSR: %d\n", err);
      goto unreg_accel;
    }

  if (priv->interrupt_mode)
    {
      /* One shared handler for both sub-sensors: each DRDY bit is
       * enabled/disabled independently by activate() as topics are
       * subscribed, but there's only one ISR and one HPWORK worker.
       */

      err = config->attach(lsm6ds3trc_interrupt, priv);
      if (err < 0)
        {
          snerr("Failed to register interrupt handler: %d\n", err);
          goto unreg_accel;
        }

      sninfo("LSM6DS3TR-C interrupt handler attached to %s.",
             priv->int_pin == LSM6DS3TRC_INT1 ? "INT1" : "INT2");
    }
  else
    {
      /* Gyro polling thread */

      snprintf(arg1, 16, "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      err = kthread_create("lsm6ds3trc_gy_thread", SCHED_PRIORITY_DEFAULT,
                           CONFIG_SENSORS_LSM6DS3TRC_THREAD_STACKSIZE,
                           gyro_thread, argv);
      if (err < 0)
        {
          snerr("Failed to register gyro polling thread: %d\n", err);
          goto unreg_accel;
        }

      gyro_pid = err;
      sninfo("LSM6DS3TR-C gyro using polling thread.");

      /* Accel polling thread */

      snprintf(arg1, 16, "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      err = kthread_create("lsm6ds3trc_xl_thread", SCHED_PRIORITY_DEFAULT,
                           CONFIG_SENSORS_LSM6DS3TRC_THREAD_STACKSIZE,
                           accel_thread, argv);
      if (err < 0)
        {
          snerr("Failed to register accel polling thread: %d\n", err);
          goto unreg_gyro_thread;
        }

      sninfo("LSM6DS3TR-C accel using polling thread.");
    }

  sninfo("LSM6DS3TR-C driver registered!");
  return OK;

unreg_gyro_thread:
  kthread_delete(gyro_pid);
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
  snerr("ERROR: Failed to register LSM6DS3TR-C driver: %d\n", err);
  return err;
}
