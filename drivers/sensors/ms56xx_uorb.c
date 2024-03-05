/****************************************************************************
 * drivers/sensors/ms56xx_uorb.c
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
#include <nuttx/sensors/ms56xx.h>
#include <nuttx/sensors/msxxxx_crc4.h>

#if defined(CONFIG_SENSORS_MS56XX) && \
    (defined(CONFIG_I2C) || defined(CONFIG_SPI))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MS56XX_CMD_RESET              0x1e
#define MS56XX_CMD_START_ADC_READ     0x00
#define MS56XX_CMD_CONV_D1_OSR_256    0x40 /* D1 = uncompensated pressure */
#define MS56XX_CMD_CONV_D1_OSR_512    0x42
#define MS56XX_CMD_CONV_D1_OSR_1024   0x44
#define MS56XX_CMD_CONV_D1_OSR_2048   0x46
#define MS56XX_CMD_CONV_D1_OSR_4096   0x48
#define MS56XX_CMD_CONV_D2_OSR_256    0x50 /* D2 = uncompensated pressure */
#define MS56XX_CMD_CONV_D2_OSR_512    0x52
#define MS56XX_CMD_CONV_D2_OSR_1024   0x54
#define MS56XX_CMD_CONV_D2_OSR_2048   0x56
#define MS56XX_CMD_CONV_D2_OSR_4096   0x58
#define MS56XX_CMD_ADC_READ           0x00
#define MS56XX_CMD_ADC_PROM_READ(i)   (0xa0 + (i)*2) /* 0xA0 - 0xAE */

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct ms56xx_calib_s
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

struct ms56xx_dev_s
{
  FAR struct sensor_lowerhalf_s sensor_lower;

#ifdef CONFIG_MS56XX_I2C
  FAR struct i2c_master_s *i2c;       /* I2C interface */
  uint8_t                  addr;      /* I2C address */
#endif

#ifdef CONFIG_MS56XX_SPI
  FAR struct spi_dev_s    *spi;       /* SPI interface */
#endif

  enum ms56xx_model_e      model;     /* Model of MS56XX */
  uint32_t                 freq;      /* Bus Frequency I2C/SPI */
  struct ms56xx_calib_s    calib;     /* Calib. params from ROM */
  unsigned long            interval;  /* Polling interval */
  bool                     enabled;   /* Enable/Disable MS56XX */
  sem_t                    run;       /* Locks measure cycle */
  mutex_t                  lock;      /* Manages exclusive to device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ms56xx_sendcmd(FAR struct ms56xx_dev_s *priv,
                          uint8_t cmd);
static int ms56xx_read16(FAR struct ms56xx_dev_s *priv,
                         FAR uint8_t *regval);
static int ms56xx_read24(FAR struct ms56xx_dev_s *priv,
                         FAR uint8_t *regval);

static int32_t ms56xx_compensate_temp(FAR struct ms56xx_dev_s *priv,
                                      uint32_t temp_raw, int32_t *deltat);
static uint32_t ms56xx_compensate_press(FAR struct ms56xx_dev_s *priv,
                                        uint32_t press, uint32_t dt,
                                        int32_t *temp);

static unsigned long ms56xx_curtime(void);

/* Sensor methods */

static int ms56xx_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);
static int ms56xx_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);

#if 0 /* Please read below */
static int ms56xx_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  .activate      = ms56xx_activate,
  .fetch         = NULL, /* ms56xx_fetch */
  .set_interval  = ms56xx_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms56xx_curtime
 *
 * Description: Helper to get current timestamp.
 *
 * Return:
 *   Timestamp in microseconds
 ****************************************************************************/

static unsigned long ms56xx_curtime(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Name: ms56xx_sendcmd
 *
 * Description:
 *   Send a command (8-bit) to MS56XX
 *
 ****************************************************************************/

static int ms56xx_sendcmd(FAR struct ms56xx_dev_s *priv, uint8_t cmd)
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
 * Name: ms56xx_read16
 *
 * Description:
 *   Read 16-bit from a MS56XX register
 *
 ****************************************************************************/

static int ms56xx_read16(FAR struct ms56xx_dev_s *priv, FAR uint8_t *regval)
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
 * Name: ms56xx_read24
 *
 * Description:
 *   Read 24-bit from a MS56XX register
 *
 ****************************************************************************/

static int ms56xx_read24(FAR struct ms56xx_dev_s *priv, uint8_t *regval)
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

static inline void baro_measure_read(FAR struct ms56xx_dev_s *priv,
                                     FAR struct sensor_baro *baro)
{
  uint32_t press;
  uint32_t temp_raw;
  int32_t temp;
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

  ret = ms56xx_sendcmd(priv, MS56XX_CMD_CONV_D1_OSR_4096);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS56XX_CMD_CONV_D1_OSR_4096!\n");
      return;
    }

  /* Wait data acquisition */

  up_udelay(10000);

  /* Send command to start a read sequence */

  ret = ms56xx_sendcmd(priv, MS56XX_CMD_START_ADC_READ);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS56XX_CMD_START_ADC_READ!\n");
      return;
    }

  /* Wait data get ready */

  up_udelay(4000);

  ret = ms56xx_read24(priv, buffer);
  if (ret < 0)
    {
      snerr("Fail to read pressure!\n");
      return;
    }

  press = (uint32_t) buffer[0] << 16 |
          (uint32_t) buffer[1] << 8 |
          (uint32_t) buffer[2];

  /* Send command to start a D2 (temperature) conversion */

  ret = ms56xx_sendcmd(priv, MS56XX_CMD_CONV_D2_OSR_4096);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS56XX_CMD_CONV_D2_OSR_4096!\n");
      return;
    }

  /* Wait data acquisition */

  up_udelay(10000);

  /* Send command to start a read sequence */

  ret = ms56xx_sendcmd(priv, MS56XX_CMD_START_ADC_READ);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS56XX_CMD_START_ADC_READ!\n");
      return;
    }

  /* Wait data get ready */

  up_udelay(4000);

  ret = ms56xx_read24(priv, buffer);
  if (ret < 0)
    {
      snerr("Fail to read temperature!\n");
      return;
    }

  temp_raw = (uint32_t) buffer[0] << 16 |
             (uint32_t) buffer[1] << 8 |
             (uint32_t) buffer[2];

  /* Release the mutex */

  nxmutex_unlock(&priv->lock);

  /* Compensate the temp/press with calibration data */

  temp = ms56xx_compensate_temp(priv, temp_raw, &deltat);
  press = ms56xx_compensate_press(priv, press, deltat, &temp);

  baro->timestamp = ms56xx_curtime();
  baro->pressure = press / 100.0f;
  baro->temperature = temp / 100.0f;
}

/****************************************************************************
 * Name: ms56xx_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number of arguments
 *   argv - Pointer to argument list
 ****************************************************************************/

static int ms56xx_thread(int argc, char **argv)
{
  FAR struct ms56xx_dev_s *priv = (FAR struct ms56xx_dev_s *)
        ((uintptr_t)strtoul(argv[1], NULL, 16));

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
 * Name: ms56xx_initialize
 *
 * Description:
 *   Initialize MS56XX device
 *
 ****************************************************************************/

static int ms56xx_initialize(FAR struct ms56xx_dev_s *priv)
{
  uint16_t prom[8];
  uint8_t data[2];
  uint8_t crc;
  int i;
  int ret;

  /* Get calibration data. */

  ret = ms56xx_sendcmd(priv, MS56XX_CMD_RESET);
  if (ret < 0)
    {
      snerr("ms56xx reset failed\n");
      return ret;
    }

  /* We have to wait before the prom is ready is be read */

  up_udelay(10000);

  for (i = 0; i < 8; i++)
    {
      ret = ms56xx_sendcmd(priv, MS56XX_CMD_ADC_PROM_READ(i));
      if (ret < 0)
        {
          snerr("ms56xx_sendcmd failed\n");
          return ret;
        }

      ret = ms56xx_read16(priv, data);
      if (ret < 0)
        {
          snerr("ms56xx_read16 failed\n");
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
 * Name: ms56xx_compensate_temp
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

static int32_t ms56xx_compensate_temp(FAR struct ms56xx_dev_s *priv,
                                      uint32_t temp_raw, int32_t *deltat)
{
  struct ms56xx_calib_s *c = &priv->calib;
  int32_t dt;
  int32_t temp;

  /* dt = d1 - c5 * 256 */

  dt = temp_raw - ((int32_t) c->c5 << 8);

  /* temp = 2000 + (dt * c6) / 8388608 */

  temp = 2000 + (((int64_t) (dt * c->c6)) >> 23);

  /* Save dt that will be used for pressure calibration */

  *deltat = dt;

  return temp;
}

/****************************************************************************
 * Name: ms56xx_compensate_press
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

static uint32_t ms56xx_compensate_press(FAR struct ms56xx_dev_s *priv,
                                        uint32_t press, uint32_t dt,
                                        int32_t *temp)
{
  struct ms56xx_calib_s *c = &priv->calib;
  int64_t off = 0;
  int64_t sens = 0;
#if defined(CONFIG_MS56XX_SECOND_ORDER_COMPENSATE)
  int64_t off2 = 0;
  int64_t sens2 = 0;
  int64_t t2 = 0;
  uint64_t delta;
#endif

  switch (priv->model)
    {
      case MS56XX_MODEL_MS5607:
        off = ((int64_t) c->c2 << 17) + ((int64_t) (c->c4 * dt) >> 6);
        sens = ((int64_t) c->c1 << 16) + ((int64_t) (c->c3 * dt) >> 7);
#if defined(CONFIG_MS56XX_SECOND_ORDER_COMPENSATE)
        if (*temp < 2000)
          {
            /* Low temperature */

            t2 = ((dt * dt) >> 31);
            delta = *temp - 2000;
            delta *= delta;
            off2 = (61 * delta) >> 4;
            sens2 = 2 * delta;

            if (*temp < -1500)
              {
                /* Very low temperature */

                delta = *temp + 1500;
                delta *= delta;
                off2 += 15 * delta;
                sens2 += 8 * delta;
              }
          }
#endif
        break;

      case MS56XX_MODEL_MS5611:
        off = ((int64_t) c->c2 << 16) + ((int64_t) (c->c4 * dt) >> 7);
        sens = ((int64_t) c->c1 << 15) + ((int64_t) (c->c3 * dt) >> 8);
#if defined(CONFIG_MS56XX_SECOND_ORDER_COMPENSATE)
        if (*temp < 2000)
          {
            /* Low temperature */

            t2 = ((dt * dt) >> 31);
            delta = *temp - 2000;
            delta *= delta;
            off2 = (5 * delta) >> 1;
            sens2 = (5 * delta) >> 2;

            if (*temp < -1500)
              {
                /* Very low temperature */

                delta = *temp + 1500;
                delta *= delta;
                off2 += 7 * delta;
                sens2 += (11 * delta) >> 1;
              }
          }
#endif
        break;
    }

#if defined(CONFIG_MS56XX_SECOND_ORDER_COMPENSATE)
  *temp -= t2;
  off -= off2;
  sens -= sens2;
#endif
  press = (((press * sens) >> 21) - off) >> 15;

  return press;
}

/****************************************************************************
 * Name: ms56xx_set_interval
 ****************************************************************************/

static int ms56xx_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us)
{
  FAR struct ms56xx_dev_s *priv = container_of(lower,
                                               FAR struct ms56xx_dev_s,
                                               sensor_lower);

  priv->interval = *period_us;
  return OK;
}

/****************************************************************************
 * Name: ms56xx_activate
 ****************************************************************************/

static int ms56xx_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  bool start_thread = false;
  struct ms56xx_dev_s *priv = (FAR struct ms56xx_dev_s *)lower;

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
 * Name: ms56xx_fetch
 ****************************************************************************/

/* N.B. When fetch is enabled the sensortest doesn't respect the
 * interval (-i) parameter, so let keep it comment until further
 * discussion about the "issue".
 */

#if 0
static int ms56xx_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR char *buffer, size_t buflen)
{
  FAR struct ms56xx_dev_s *priv = container_of(lower,
                                               FAR struct ms56xx_dev_s,
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
 * Name: ms56xx_register
 *
 * Description:
 *   Register the MS56XX character device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             MS56XX
 *   devno   - Instance number for driver
 *   addr    - The I2C address of the MS56XX.
 *   model   - The MS56XX model.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms56xx_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr,
                    enum ms56xx_model_e model)
{
  FAR struct ms56xx_dev_s *priv;
  FAR char *argv[2];
  char arg1[32];

  int ret;

  /* Initialize the MS56XX device structure */

  priv = kmm_zalloc(sizeof(struct ms56xx_dev_s));
  if (priv == NULL)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c      = i2c;
  priv->addr     = addr;
  priv->model    = model;
  priv->freq     = CONFIG_MS56XX_I2C_FREQUENCY;
  priv->interval = 1000000; /* Default interval 1s */

  nxsem_init(&priv->run, 0, 0);
  nxmutex_init(&priv->lock);

  priv->sensor_lower.ops = &g_sensor_ops;
  priv->sensor_lower.type = SENSOR_TYPE_BAROMETER;

  ret = ms56xx_initialize(priv);
  if (ret < 0)
    {
      snerr("Failed to initialize physical device ms56xx:%d\n", ret);
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

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("ms56xx_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_MS56XX_THREAD_STACKSIZE,
                       ms56xx_thread, argv);
  if (ret < 0)
    {
      snerr("Failed to create the notification kthread!\n");
      sensor_unregister(&priv->sensor_lower, devno);
      nxmutex_destroy(&priv->lock);
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return ret;
    }

  sninfo("MS56XX driver loaded successfully!\n");
  return OK;
}

#endif
