/****************************************************************************
 * drivers/sensors/ms5611.c
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
#include <nuttx/nuttx.h>

#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/signal.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ms5611.h>
#include <nuttx/sensors/msxxxx_crc4.h>

#if defined(CONFIG_SENSORS_MS5611) && \
    (defined(CONFIG_I2C) || defined(CONFIG_SPI))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MS5611_CMD_RESET              0x1e
#define MS5611_CMD_START_ADC_READ     0x00
#define MS5611_CMD_CONV_D1_OSR_256    0x40 /* D1 = uncompensated pressure */
#define MS5611_CMD_CONV_D1_OSR_512    0x42
#define MS5611_CMD_CONV_D1_OSR_1024   0x44
#define MS5611_CMD_CONV_D1_OSR_2048   0x46
#define MS5611_CMD_CONV_D1_OSR_4096   0x48
#define MS5611_CMD_CONV_D2_OSR_256    0x50 /* D2 = uncompensated pressure */
#define MS5611_CMD_CONV_D2_OSR_512    0x52
#define MS5611_CMD_CONV_D2_OSR_1024   0x54
#define MS5611_CMD_CONV_D2_OSR_2048   0x56
#define MS5611_CMD_CONV_D2_OSR_4096   0x58
#define MS5611_CMD_ADC_READ           0x00
#define MS5611_CMD_ADC_PROM_READ(i)   (0xa0 + (i)*2) /* 0xA0 - 0xAE */

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct ms5611_calib_s
{
  uint16_t reversed;
  uint16_t c1;
  uint16_t c2;
  uint16_t c3;
  uint16_t c4;
  uint16_t c5;
  uint16_t c6;
  uint16_t crc;
};

struct ms5611_dev_s
{
  FAR struct sensor_lowerhalf_s sensor_lower;

#ifdef CONFIG_MS5611_I2C
  FAR struct i2c_master_s *i2c;       /* I2C interface */
  uint8_t                  addr;      /* I2C address */
#endif

#ifdef CONFIG_MS5611_SPI
  FAR struct spi_dev_s    *spi;       /* SPI interface */
#endif

  uint32_t                 freq;      /* Bus Frequency I2C/SPI */
  struct ms5611_calib_s    calib;     /* Calib. params from ROM */
  unsigned long            interval;  /* Polling interval */
  bool                     enabled;   /* Enable/Disable MS5611 */
  sem_t                    run;       /* Locks measure cycle */
  mutex_t                  lock;      /* Manages exclusive to device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ms5611_sendcmd(FAR struct ms5611_dev_s *priv,
                          uint8_t cmd);
static int ms5611_read16(FAR struct ms5611_dev_s *priv,
                         FAR uint8_t *regval);
static int ms5611_read24(FAR struct ms5611_dev_s *priv,
                         FAR uint8_t *regval);

static int32_t ms5611_compensate_temp(FAR struct ms5611_dev_s *priv,
                                      uint32_t temp, int32_t *deltat);
static uint32_t ms5611_compensate_press(FAR struct ms5611_dev_s *priv,
                                        uint32_t press, uint32_t dt);

static unsigned long ms5611_curtime(void);

/* Sensor methods */

static int ms5611_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);
static int ms5611_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);

#if 0 /* Please read below */
static int ms5611_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  .activate      = ms5611_activate,
  .fetch         = NULL, /* ms5611_fetch */
  .set_interval  = ms5611_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms5611_curtime
 *
 * Description: Helper to get current timestamp.
 *
 * Return:
 *   Timestamp in microseconds
 ****************************************************************************/

static unsigned long ms5611_curtime(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Name: ms5611_sendcmd
 *
 * Description:
 *   Send a command (8-bit) to MS5611
 *
 ****************************************************************************/

static int ms5611_sendcmd(FAR struct ms5611_dev_s *priv, uint8_t cmd)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = priv->freq;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = &cmd;
  msg.length    = 1;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ms5611_read16
 *
 * Description:
 *   Read 16-bit from a MS5611 register
 *
 ****************************************************************************/

static int ms5611_read16(FAR struct ms5611_dev_s *priv, FAR uint8_t *regval)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = priv->freq;
  msg.addr      = priv->addr;
  msg.flags     = I2C_M_READ;
  msg.buffer    = regval;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ms5611_read24
 *
 * Description:
 *   Read 24-bit from a MS5611 register
 *
 ****************************************************************************/

static int ms5611_read24(FAR struct ms5611_dev_s *priv, uint8_t *regval)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = priv->freq;
  msg.addr      = priv->addr;
  msg.flags     = I2C_M_READ;
  msg.buffer    = regval;
  msg.length    = 3;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  return OK;
}

static inline void baro_measure_read(FAR struct ms5611_dev_s *priv,
                                     FAR struct sensor_baro *baro)
{
  uint32_t press;
  uint32_t temp;
  int32_t deltat;
  int ret;
  uint8_t buffer[3];

  /* Enforce exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return;
    }

  /* Send command to start a D1 (pressure) conversion */

  ret = ms5611_sendcmd(priv, MS5611_CMD_CONV_D1_OSR_4096);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS5611_CMD_CONV_D1_OSR_4096!\n");
      return;
    }

  /* Wait data acquisition */

  up_udelay(10000);

  /* Send command to start a read sequence */

  ret = ms5611_sendcmd(priv, MS5611_CMD_START_ADC_READ);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS5611_CMD_START_ADC_READ!\n");
      return;
    }

  /* Wait data get ready */

  up_udelay(4000);

  ret = ms5611_read24(priv, buffer);
  if (ret < 0)
    {
      snerr("Fail to read pressure!\n");
      return;
    }

  press = (uint32_t) buffer[0] << 16 |
          (uint32_t) buffer[1] << 8 |
          (uint32_t) buffer[2];

  /* Send command to start a D2 (temperature) conversion */

  ret = ms5611_sendcmd(priv, MS5611_CMD_CONV_D2_OSR_4096);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS5611_CMD_CONV_D2_OSR_4096!\n");
      return;
    }

  /* Wait data acquisition */

  up_udelay(10000);

  /* Send command to start a read sequence */

  ret = ms5611_sendcmd(priv, MS5611_CMD_START_ADC_READ);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS5611_CMD_START_ADC_READ!\n");
      return;
    }

  /* Wait data get ready */

  up_udelay(4000);

  ret = ms5611_read24(priv, buffer);
  if (ret < 0)
    {
      snerr("Fail to read temperature!\n");
      return;
    }

  temp = (uint32_t) buffer[0] << 16 |
         (uint32_t) buffer[1] << 8 |
         (uint32_t) buffer[2];

  /* Release the mutex */

  nxmutex_unlock(&priv->lock);

  /* Compensate the temp/press with calibration data */

  temp = ms5611_compensate_temp(priv, temp, &deltat);
  press = ms5611_compensate_press(priv, press, deltat);

  baro->timestamp = ms5611_curtime();
  baro->pressure = press / 100.0f;
  baro->temperature = temp / 100.0f;
}

/****************************************************************************
 * Name: ms5611_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 ****************************************************************************/

static int ms5611_thread(int argc, char **argv)
{
  FAR struct ms5611_dev_s *priv = (FAR struct ms5611_dev_s *)
        ((uintptr_t)strtoul(argv[1], NULL, 0));

  struct sensor_baro baro_data;

  while (true)
    {
      int ret;

      if (!priv->enabled)
        {
          /* Waiting to be woken up */

          ret = nxsem_wait(&priv->run);
          if (ret < 0)
            {
              continue;
            }
        }

       baro_measure_read(priv, &baro_data);

       priv->sensor_lower.push_event(priv->sensor_lower.priv, &baro_data,
                                     sizeof(struct sensor_baro));

      /* Sleeping thread before fetching the next sensor data */

      nxsig_usleep(priv->interval);
    }

  return OK;
}

/****************************************************************************
 * Name: ms5611_initialize
 *
 * Description:
 *   Initialize MS5611 device
 *
 ****************************************************************************/

static int ms5611_initialize(FAR struct ms5611_dev_s *priv)
{
  uint16_t prom[8];
  uint8_t data[2];
  uint8_t crc;
  int i;
  int ret;

  /* Get calibration data. */

  ret = ms5611_sendcmd(priv, MS5611_CMD_RESET);
  if (ret < 0)
    {
      snerr("ms5611 reset failed\n");
      return ret;
    }

  /* We have to wait before the prom is ready is be read */

  up_udelay(10000);

  for (i = 0; i < 8; i++)
    {
      ret = ms5611_sendcmd(priv, MS5611_CMD_ADC_PROM_READ(i));
      if (ret < 0)
        {
          snerr("ms5611_sendcmd failed\n");
          return ret;
        }

      ret = ms5611_read16(priv, data);
      if (ret < 0)
        {
          snerr("ms5611_read16 failed\n");
          return ret;
        }

      prom[i] = (uint16_t) data[0] << 8 | (uint16_t) data[1];
    }

  /* Get the 4-bit CRC from PROM */

  crc = (uint8_t)(prom[7] & 0xf);

  /* Verify if the calculated CRC is equal to PROM's CRC */

  if (crc != msxxxx_crc4(prom, 7, 0xff))
    {
      snerr("ERROR: Calculated CRC different from PROM's CRC!\n");
      return -ENODEV;
    }

  /* Fill read calibration coefficients */

  priv->calib.c1 = prom[1];
  priv->calib.c2 = prom[2];
  priv->calib.c3 = prom[3];
  priv->calib.c4 = prom[4];
  priv->calib.c5 = prom[5];
  priv->calib.c6 = prom[6];

  return ret;
}

/****************************************************************************
 * Name: ms5611_compensate_temp
 *
 * Description:
 *   calculate compensate temperature
 *
 * Input Parameters:
 *   temp - uncompensate value of temperature.
 *
 * Returned Value:
 *   calculate result of compensate temperature.
 *
 ****************************************************************************/

static int32_t ms5611_compensate_temp(FAR struct ms5611_dev_s *priv,
                                      uint32_t temp, int32_t *deltat)
{
  struct ms5611_calib_s *c = &priv->calib;
  int32_t dt;

  /* dt = d1 - c5 * 256 */

  dt = temp - ((int32_t) c->c5 << 8);

  /* temp = 2000 + (dt * c6) / 8388608 */

  temp = 2000 + (((int64_t) (dt * c->c6)) >> 23);

  /* Save dt that will be used for pressure calibration */

  *deltat = dt;

  return temp;
}

/****************************************************************************
 * Name: ms5611_compensate_press
 *
 * Description:
 *   calculate compensate pressure
 *
 * Input Parameters:
 *   press - uncompensate value of pressure.
 *
 * Returned Value:
 *   calculate result of compensate pressure.
 *
 ****************************************************************************/

static uint32_t ms5611_compensate_press(FAR struct ms5611_dev_s *priv,
                                        uint32_t press, uint32_t dt)
{
  struct ms5611_calib_s *c = &priv->calib;
  int64_t off;
  int64_t sens;

  off = ((int64_t) c->c2 * 65536) + ((int64_t) (c->c4 * dt) / 128);
  sens = ((int64_t) c->c1 * 32768) + ((int64_t) (c->c3 * dt) / 256);
  press = (((press * sens) / 2097152) - off) / 32768;

  return press;
}

/****************************************************************************
 * Name: ms5611_set_interval
 ****************************************************************************/

static int ms5611_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us)
{
  FAR struct ms5611_dev_s *priv = container_of(lower,
                                               FAR struct ms5611_dev_s,
                                               sensor_lower);

  priv->interval = *period_us;
  return OK;
}

/****************************************************************************
 * Name: ms5611_activate
 ****************************************************************************/

static int ms5611_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  bool start_thread = false;
  struct ms5611_dev_s *priv = (FAR struct ms5611_dev_s *)lower;

  if (enable)
    {
      if (!priv->enabled)
        {
          start_thread = true;
        }
    }

  priv->enabled = enable;

  if (start_thread)
    {
      /* Wake up the thread */

      nxsem_post(&priv->run);
    }

  return OK;
}

/****************************************************************************
 * Name: ms5611_fetch
 ****************************************************************************/

/* N.B. When fetch is enabled the sensortest doesn't respect the
 * interval (-i) parameter, so let keep it comment until further
 * discussion about the "issue".
 */

#if 0
static int ms5611_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR char *buffer, size_t buflen)
{
  FAR struct ms5611_dev_s *priv = container_of(lower,
                                               FAR struct ms5611_dev_s,
                                               sensor_lower);
  struct sensor_baro baro_data;

  if (buflen != sizeof(baro_data))
    {
      return -EINVAL;
    }

  baro_measure_read(priv, &baro_data);

  memcpy(buffer, &baro_data, sizeof(baro_data));

  return buflen;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms5611_register
 *
 * Description:
 *   Register the MS5611 character device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             MS5611
 *   devno   - Instance number for driver
 *   addr    - The I2C address of the MS5611.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms5611_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr)
{
  FAR struct ms5611_dev_s *priv;
  FAR char *argv[2];
  char arg1[32];

  int ret;

  /* Initialize the MS5611 device structure */

  priv = (FAR struct ms5611_dev_s *)kmm_zalloc(sizeof(struct ms5611_dev_s));
  if (priv == NULL)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c      = i2c;
  priv->addr     = addr;
  priv->freq     = CONFIG_MS5611_I2C_FREQUENCY;
  priv->interval = 1000000; /* Default interval 1s */

  nxsem_init(&priv->run, 0, 0);
  nxmutex_init(&priv->lock);

  priv->sensor_lower.ops = &g_sensor_ops;
  priv->sensor_lower.type = SENSOR_TYPE_BAROMETER;

  ret = ms5611_initialize(priv);
  if (ret < 0)
    {
      snerr("Failed to initialize physical device ms5611:%d\n", ret);
      nxmutex_destroy(&priv->lock);
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return ret;
    }

  /* Register the character driver */

  ret = sensor_register(&priv->sensor_lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return ret;
    }

  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "0x%" PRIxPTR, (uintptr_t)priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("ms5611_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_MS5611_THREAD_STACKSIZE,
                       ms5611_thread, argv);
  if (ret < 0)
    {
      snerr("Failed to create the notification kthread!\n");
      sensor_unregister(&priv->sensor_lower, devno);
      nxmutex_destroy(&priv->lock);
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return ret;
    }

  sninfo("MS5611 driver loaded successfully!\n");
  return OK;
}

#endif
