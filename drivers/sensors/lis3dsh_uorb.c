/****************************************************************************
 * drivers/sensors/lis3dsh_uorb.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <string.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>

#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/lis3dsh.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LIS3DSH_WHO_AM_I_VALUE  0x3f

/* Full-scale is configured to +/-2 g.  The nominal sensitivity is
 * 0.06 mg/digit; convert LSBs to m/s^2.
 */

#define LIS3DSH_ONE_G           9.80665f
#define LIS3DSH_SENS_2G         0.00006f  /* g/LSB */
#define LIS3DSH_SCALE           (LIS3DSH_SENS_2G * LIS3DSH_ONE_G)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lis3dsh_sensor_s
{
  struct sensor_lowerhalf_s  lower;
  FAR struct spi_dev_s      *spi;
  int                        devno;
#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
  bool                       enabled;
  uint32_t                   interval;
  sem_t                      run;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void lis3dsh_putreg8(FAR struct lis3dsh_sensor_s *priv,
                            uint8_t regaddr, uint8_t regval);
static uint8_t lis3dsh_getreg8(FAR struct lis3dsh_sensor_s *priv,
                               uint8_t regaddr);
static void lis3dsh_getregs(FAR struct lis3dsh_sensor_s *priv,
                            uint8_t regaddr, FAR uint8_t *regval,
                            int len);
static int lis3dsh_checkid(FAR struct lis3dsh_sensor_s *priv);
static void lis3dsh_start(FAR struct lis3dsh_sensor_s *priv);
static void lis3dsh_read_accel(FAR struct lis3dsh_sensor_s *priv,
                               FAR struct sensor_accel *accel);

static int lis3dsh_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
static int lis3dsh_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
static int lis3dsh_thread(int argc, FAR char **argv);
#else
static int lis3dsh_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_lis3dsh_accel_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  lis3dsh_activate,
  lis3dsh_set_interval,
  NULL,                 /* batch */
#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
  NULL,                 /* fetch */
#else
  lis3dsh_fetch,
#endif
  NULL,                 /* flush */
  NULL,                 /* selftest */
  NULL,                 /* set_calibvalue */
  NULL,                 /* calibrate */
  NULL,                 /* get_info */
  NULL,                 /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3dsh_putreg8
 ****************************************************************************/

static void lis3dsh_putreg8(FAR struct lis3dsh_sensor_s *priv,
                            uint8_t regaddr, uint8_t regval)
{
  SPI_LOCK(priv->spi, true);
  SPI_SETFREQUENCY(priv->spi, LIS3DSH_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, LIS3DSH_SPI_MODE);

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), true);
  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);
  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), false);

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: lis3dsh_getreg8
 ****************************************************************************/

static uint8_t lis3dsh_getreg8(FAR struct lis3dsh_sensor_s *priv,
                               uint8_t regaddr)
{
  uint8_t regval = 0;

  SPI_LOCK(priv->spi, true);
  SPI_SETFREQUENCY(priv->spi, LIS3DSH_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, LIS3DSH_SPI_MODE);

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), true);

  /* MSB set indicates a read */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), false);
  SPI_LOCK(priv->spi, false);

  return regval;
}

/****************************************************************************
 * Name: lis3dsh_getregs
 *
 * Description:
 *   Read a block of registers.  Address auto-increment (ADD_INC in
 *   CTRL_REG6) must be enabled for multi-byte transfers.
 *
 ****************************************************************************/

static void lis3dsh_getregs(FAR struct lis3dsh_sensor_s *priv,
                            uint8_t regaddr, FAR uint8_t *regval, int len)
{
  SPI_LOCK(priv->spi, true);
  SPI_SETFREQUENCY(priv->spi, LIS3DSH_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, LIS3DSH_SPI_MODE);

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), true);

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, regval, len);

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(priv->devno), false);
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: lis3dsh_checkid
 ****************************************************************************/

static int lis3dsh_checkid(FAR struct lis3dsh_sensor_s *priv)
{
  uint8_t id = lis3dsh_getreg8(priv, LIS3DSH_WHO_AM_I_REG);

  if (id != LIS3DSH_WHO_AM_I_VALUE)
    {
      snerr("ERROR: Wrong LIS3DSH WHO_AM_I %02x\n", id);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lis3dsh_start
 *
 * Description:
 *   Configure the device for continuous +/-2 g measurement at 100 Hz.
 *
 ****************************************************************************/

static void lis3dsh_start(FAR struct lis3dsh_sensor_s *priv)
{
  /* Full-scale +/-2 g, 4-wire SPI (CTRL_REG5 = 0) */

  lis3dsh_putreg8(priv, LIS3DSH_CTRL_REG_5, 0);

  /* Enable register address auto-increment for block reads */

  lis3dsh_putreg8(priv, LIS3DSH_CTRL_REG_6,
                  LIS3DSH_CTRL_REG_6_ADD_INC_BM);

  /* Enable X/Y/Z axes, block-data-update, ODR = 100 Hz (0101b) */

  lis3dsh_putreg8(priv, LIS3DSH_CTRL_REG_4,
                  LIS3DSH_CTRL_REG_4_XEN_BM |
                  LIS3DSH_CTRL_REG_4_YEN_BM |
                  LIS3DSH_CTRL_REG_4_ZEN_BM |
                  LIS3DSH_CTRL_REG_4_BDU_BM |
                  LIS3DSH_CTRL_REG_4_ODR_2_BM |
                  LIS3DSH_CTRL_REG_4_ODR_0_BM);
}

/****************************************************************************
 * Name: lis3dsh_read_accel
 *
 * Description:
 *   Read the current acceleration and convert it to a sensor_accel event.
 *
 ****************************************************************************/

static void lis3dsh_read_accel(FAR struct lis3dsh_sensor_s *priv,
                               FAR struct sensor_accel *accel)
{
  uint8_t data[6];
  int16_t x;
  int16_t y;
  int16_t z;

  lis3dsh_getregs(priv, LIS3DSH_OUT_X_L_REG, data, 6);

  x = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
  y = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
  z = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));

  accel->timestamp   = sensor_get_timestamp();
  accel->x           = (float)x * LIS3DSH_SCALE;
  accel->y           = (float)y * LIS3DSH_SCALE;
  accel->z           = (float)z * LIS3DSH_SCALE;
  accel->temperature = 0;
}

/****************************************************************************
 * Name: lis3dsh_activate
 ****************************************************************************/

static int lis3dsh_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct lis3dsh_sensor_s *priv = (FAR struct lis3dsh_sensor_s *)lower;
#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
  bool start_thread = false;
#endif

  if (enable)
    {
#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
      if (!priv->enabled)
        {
          start_thread = true;
        }
#endif
    }

  UNUSED(priv);

#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
  priv->enabled = enable;

  if (start_thread)
    {
      nxsem_post(&priv->run);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: lis3dsh_set_interval
 ****************************************************************************/

static int lis3dsh_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
  FAR struct lis3dsh_sensor_s *priv = (FAR struct lis3dsh_sensor_s *)lower;

  priv->interval = *period_us;
#endif

  return OK;
}

#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
/****************************************************************************
 * Name: lis3dsh_thread
 *
 * Description:
 *   Poll the sensor at the configured interval and push accelerometer
 *   samples to the uORB upper half (streaming mode).
 *
 ****************************************************************************/

static int lis3dsh_thread(int argc, FAR char **argv)
{
  FAR struct lis3dsh_sensor_s *priv =
    (FAR struct lis3dsh_sensor_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  struct sensor_accel accel;
  int ret;

  while (true)
    {
      if (!priv->enabled)
        {
          /* Wait to be woken up by activate() */

          ret = nxsem_wait(&priv->run);
          if (ret < 0)
            {
              continue;
            }
        }

      if (priv->enabled)
        {
          lis3dsh_read_accel(priv, &accel);
          priv->lower.push_event(priv->lower.priv, &accel, sizeof(accel));
        }

      nxsig_usleep(priv->interval);
    }

  return OK;
}

#else
/****************************************************************************
 * Name: lis3dsh_fetch
 *
 * Description:
 *   Read the most recent accelerometer sample on demand.
 *
 ****************************************************************************/

static int lis3dsh_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen)
{
  FAR struct lis3dsh_sensor_s *priv = (FAR struct lis3dsh_sensor_s *)lower;
  struct sensor_accel accel;

  if (buflen < sizeof(accel))
    {
      return -EINVAL;
    }

  lis3dsh_read_accel(priv, &accel);

  memcpy(buffer, &accel, sizeof(accel));
  return sizeof(accel);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3dsh_register_uorb
 ****************************************************************************/

int lis3dsh_register_uorb(int devno, FAR struct spi_dev_s *spi)
{
  FAR struct lis3dsh_sensor_s *priv;
  int                          ret;
#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
  FAR char                    *argv[2];
  char                         arg1[32];
#endif

  DEBUGASSERT(spi != NULL);

  priv = kmm_zalloc(sizeof(struct lis3dsh_sensor_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate LIS3DSH instance\n");
      return -ENOMEM;
    }

  priv->spi           = spi;
  priv->devno         = devno;
  priv->lower.ops     = &g_lis3dsh_accel_ops;
  priv->lower.type    = SENSOR_TYPE_ACCELEROMETER;
  priv->lower.nbuffer = 1;
#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
  priv->enabled       = false;
  priv->interval      = CONFIG_SENSORS_LIS3DSH_UORB_POLL_INTERVAL;
  nxsem_init(&priv->run, 0, 0);
#endif

  /* Configure SPI and verify the device id */

  SPI_LOCK(priv->spi, true);
  SPI_SETFREQUENCY(priv->spi, LIS3DSH_SPI_FREQUENCY);
  SPI_SETMODE(priv->spi, LIS3DSH_SPI_MODE);
  SPI_LOCK(priv->spi, false);

  ret = lis3dsh_checkid(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure the device for continuous +/-2 g measurement at 100 Hz */

  lis3dsh_start(priv);

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register LIS3DSH accel driver: %d\n", ret);
      goto errout;
    }

#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("lis3dsh_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_LIS3DSH_UORB_STACKSIZE,
                       lis3dsh_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to create LIS3DSH thread: %d\n", ret);
      sensor_unregister(&priv->lower, devno);
      goto errout;
    }
#endif

  return OK;

errout:
#ifdef CONFIG_SENSORS_LIS3DSH_UORB_POLL
  nxsem_destroy(&priv->run);
#endif
  kmm_free(priv);
  return ret;
}
