/****************************************************************************
 * drivers/sensors/ccs811_uorb.c
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
#include <debug.h>
#include <string.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ccs811.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Time to wait after a nWAKE falling edge before the CCS811 is ready for an
 * I2C transaction (datasheet t_WAKE is 50us).
 */

#define CCS811_TWAKE_US         (100)

/* Number of times a single I2C transaction is retried.  The first access
 * after an idle gap occasionally NAKs while the sensor wakes.
 */

#define CCS811_XFER_RETRIES     (3)

/* Number of times the whole boot-to-application bring-up is retried. */

#define CCS811_START_RETRIES    (2)

/* Registers */

#define CCS811_STATUS           (0x00)
#define CCS811_MEAS_MODE        (0x01)
#define CCS811_ALG_RESULT_DATA  (0x02)
#define CCS811_RAW_DATA         (0x03)
#define CCS811_ENV_DATA         (0x05)
#define CCS811_THRESHOLDS       (0x10)
#define CCS811_BASELINE         (0x11)
#define CCS811_HW_ID            (0x20)
#define CCS811_HW_VERSION       (0x21)
#define CCS811_FW_BOOT_VERSION  (0x23)
#define CCS811_FW_APP_VERSION   (0x24)
#define CCS811_INTERNAL_STATE   (0xa0)
#define CCS811_ERROR_ID         (0xe0)
#define CCS811_SW_RESET         (0xff)
#define CCS811_APP_ERASE        (0xf1)
#define CCS811_APP_DATA         (0xf2)
#define CCS811_APP_VERIFY       (0xf3)
#define CCS811_APP_START        (0xf4)

/* HW_ID value */

#define CCS811_DEVID            (0x81)

/* STATUS register bits */

#define CCS811_STATUS_ERROR     (1 << 0)
#define CCS811_STATUS_DATARDY   (1 << 3)
#define CCS811_STATUS_APPVALID  (1 << 4)
#define CCS811_STATUS_FWMODE    (1 << 7)

/* MEAS_MODE drive modes */

#define CCS811_DRIVE_MODE_IDLE  (0 << 4) /* Idle, measurements disabled */
#define CCS811_DRIVE_MODE_1SEC  (1 << 4) /* Constant power, every 1 s */
#define CCS811_DRIVE_MODE_10SEC (2 << 4) /* Pulse heating, every 10 s */
#define CCS811_DRIVE_MODE_60SEC (3 << 4) /* Pulse heating, every 60 s */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Each measurement channel needs its own lower-half. */

struct ccs811_sensor_s
{
  FAR struct sensor_lowerhalf_s lower;   /* Lower-half driver */
  FAR struct ccs811_dev_s      *dev;     /* Parent device */
  bool                          enabled; /* Channel activated */
};

struct ccs811_dev_s
{
  struct ccs811_sensor_s   co2;       /* eCO2 lower-half */
  struct ccs811_sensor_s   tvoc;      /* TVOC lower-half */
  FAR struct i2c_master_s *i2c;       /* I2C interface */
  CODE void (*wake)(bool on);         /* Board nWAKE control (may be NULL) */
  uint8_t                  addr;      /* I2C address */
  bool                     wake_held; /* nWAKE kept low across a burst */
  bool                     valid;     /* Cached measurement available */
  uint64_t                 timestamp; /* Cached measurement timestamp */
  uint16_t                 eco2;      /* Last eCO2 in ppm */
  uint16_t                 etvoc;     /* Last TVOC in ppb */
  unsigned long            interval;  /* Polling interval in us */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ccs811_read_regs(FAR struct ccs811_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *regvals, int len);
static int ccs811_write(FAR struct ccs811_dev_s *priv, uint8_t regaddr,
                        FAR const uint8_t *regvals, int len);
static int ccs811_appstart(FAR struct ccs811_dev_s *priv);
static int ccs811_start(FAR struct ccs811_dev_s *priv);
static int ccs811_measure(FAR struct ccs811_dev_s *priv);

/* Sensor lower-half operations */

static int ccs811_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);
static int ccs811_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us);
static int ccs811_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, FAR char *buffer,
                        size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_ccs811_ops =
{
  .activate     = ccs811_activate,
  .set_interval = ccs811_set_interval,
  .fetch        = ccs811_fetch,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ccs811_wake_hold
 *
 * Description:
 *   Assert nWAKE and keep it asserted across a whole sequence of I2C
 *   transactions (the per-transaction pulse is suppressed while held).  The
 *   boot->application transition must not be interrupted by nWAKE returning
 *   high, otherwise the sensor aborts the launch and stays in boot mode.
 *
 ****************************************************************************/

static void ccs811_wake_hold(FAR struct ccs811_dev_s *priv, bool hold)
{
  if (priv->wake == NULL)
    {
      return;
    }

  if (hold)
    {
      priv->wake(true);
      up_udelay(CCS811_TWAKE_US);
      priv->wake_held = true;
    }
  else
    {
      priv->wake_held = false;
      priv->wake(false);
    }
}

/****************************************************************************
 * Name: ccs811_read_regs
 ****************************************************************************/

static int ccs811_read_regs(FAR struct ccs811_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *regvals, int len)
{
  struct i2c_config_s config;
  int ret = -EIO;
  int i;

  config.frequency = CONFIG_CCS811_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  for (i = 0; i < CCS811_XFER_RETRIES; i++)
    {
      if (priv->wake != NULL && !priv->wake_held)
        {
          priv->wake(true);        /* nWAKE falling edge */
          up_udelay(CCS811_TWAKE_US);
        }

      ret = i2c_writeread(priv->i2c, &config, &regaddr, 1, regvals, len);

      if (priv->wake != NULL && !priv->wake_held)
        {
          priv->wake(false);       /* release nWAKE */
        }

      if (ret >= 0)
        {
          break;
        }

      nxsig_usleep(2000);
    }

  return ret;
}

/****************************************************************************
 * Name: ccs811_write
 *
 * Description:
 *   Write a command register optionally followed by data bytes.
 *
 ****************************************************************************/

static int ccs811_write(FAR struct ccs811_dev_s *priv, uint8_t regaddr,
                        FAR const uint8_t *regvals, int len)
{
  struct i2c_config_s config;
  uint8_t buf[2];
  int ret = -EIO;
  int i;

  config.frequency = CONFIG_CCS811_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  buf[0] = regaddr;
  if (len > 0)
    {
      memcpy(&buf[1], regvals, len);
    }

  for (i = 0; i < CCS811_XFER_RETRIES; i++)
    {
      if (priv->wake != NULL && !priv->wake_held)
        {
          priv->wake(true);        /* nWAKE falling edge */
          up_udelay(CCS811_TWAKE_US);
        }

      ret = i2c_write(priv->i2c, &config, buf, len + 1);

      if (priv->wake != NULL && !priv->wake_held)
        {
          priv->wake(false);       /* release nWAKE */
        }

      if (ret >= 0)
        {
          break;
        }

      nxsig_usleep(2000);
    }

  return ret;
}

/****************************************************************************
 * Name: ccs811_appstart
 *
 * Description:
 *   Verify the chip ID, switch to application mode and start periodic
 *   measurements.  A single attempt.
 *
 ****************************************************************************/

static int ccs811_appstart(FAR struct ccs811_dev_s *priv)
{
  uint8_t regval;
  int retries;
  int ret;

  /* Wait for the sensor to respond and verify the hardware ID.  These use
   * the normal per-transaction nWAKE pulse so each attempt re-wakes the
   * part.
   */

  retries = 20;
  do
    {
      nxsig_usleep(20000);
      ret = ccs811_read_regs(priv, CCS811_HW_ID, &regval, 1);
    }
  while (ret < 0 && --retries);

  if (ret < 0)
    {
      return ret;
    }

  if (regval != CCS811_DEVID)
    {
      snerr("ERROR: Wrong device ID: 0x%02x\n", regval);
      return -ENODEV;
    }

  /* Poll STATUS until a valid application is reported. */

  retries = 20;
  do
    {
      nxsig_usleep(20000);
      ret = ccs811_read_regs(priv, CCS811_STATUS, &regval, 1);
      if (ret < 0)
        {
          return ret;
        }
    }
  while ((regval & CCS811_STATUS_APPVALID) == 0 && --retries);

  if ((regval & CCS811_STATUS_APPVALID) == 0)
    {
      snerr("ERROR: CCS811 has no valid application (status=0x%02x)\n",
            regval);
      return -ENODEV;
    }

  /* Launch the application firmware.  nWAKE must stay asserted (low) across
   * the whole boot->application transition: the sensor aborts the launch
   * and falls back to boot mode if nWAKE returns high (i.e. it is allowed
   * to sleep) before the application is running.  Hold nWAKE low, issue
   * APP_START, then poll STATUS tightly (back-to-back reads keep the part
   * awake) until it reports application mode.  During the launch STATUS can
   * read back as 0xff/0xfd garbage, so only accept a clean value: FWMODE
   * set with all reserved bits clear.
   */

  ccs811_wake_hold(priv, true);

  retries = 20;
  do
    {
      ret = ccs811_write(priv, CCS811_APP_START, NULL, 0);
      if (ret < 0)
        {
          goto errout;
        }

      up_udelay(2000);
      ret = ccs811_read_regs(priv, CCS811_STATUS, &regval, 1);
    }
  while ((ret < 0 || (regval & 0x66) != 0 ||
          (regval & CCS811_STATUS_FWMODE) == 0) && --retries);

  if (ret < 0)
    {
      goto errout;
    }

  if ((regval & CCS811_STATUS_FWMODE) == 0)
    {
      snerr("ERROR: CCS811 failed to enter application mode (0x%02x)\n",
            regval);
      ret = -EIO;
      goto errout;
    }

  /* Select constant-power measurement mode, one sample every second. */

  regval = CCS811_DRIVE_MODE_1SEC;
  ret = ccs811_write(priv, CCS811_MEAS_MODE, &regval, 1);

errout:
  ccs811_wake_hold(priv, false);
  return ret;
}

/****************************************************************************
 * Name: ccs811_start
 *
 * Description:
 *   Bring the CCS811 into measurement mode.  The boot-to-application
 *   transition is occasionally unreliable after a warm reset, so retry the
 *   whole sequence a few times before giving up.
 *
 ****************************************************************************/

static int ccs811_start(FAR struct ccs811_dev_s *priv)
{
  int ret = -EIO;
  int i;

  for (i = 0; i < CCS811_START_RETRIES; i++)
    {
      ret = ccs811_appstart(priv);
      if (ret >= 0)
        {
          break;
        }

      nxsig_usleep(50000);
    }

  return ret;
}

/****************************************************************************
 * Name: ccs811_measure
 *
 * Description:
 *   Refresh the cached eCO2/TVOC values from the algorithm result data.
 *   A single ALG_RESULT_DATA read returns both values and clears DATARDY,
 *   so the result is cached and shared by both lower-halves.
 *
 ****************************************************************************/

static int ccs811_measure(FAR struct ccs811_dev_s *priv)
{
  uint8_t data[8];
  uint8_t status = 0;
  int ret;

  ret = ccs811_read_regs(priv, CCS811_STATUS, &status, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* If the sensor has dropped back into boot mode, re-run the application
   * bring-up before trying to read a measurement.
   */

  if ((status & CCS811_STATUS_FWMODE) == 0)
    {
      ret = ccs811_appstart(priv);
      if (ret < 0)
        {
          return ret;
        }

      ret = ccs811_read_regs(priv, CCS811_STATUS, &status, 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* No new sample is ready yet - keep the previously reported value. */

  if ((status & CCS811_STATUS_DATARDY) == 0)
    {
      return -EAGAIN;
    }

  ret = ccs811_read_regs(priv, CCS811_ALG_RESULT_DATA, data, 8);
  if (ret < 0)
    {
      return ret;
    }

  /* data[4] holds the status, data[5] the error id */

  if (data[4] & CCS811_STATUS_ERROR)
    {
      snerr("ERROR: CCS811 measurement error: 0x%02x\n", data[5]);
      return -EIO;
    }

  priv->eco2      = data[0] << 8 | data[1];
  priv->etvoc     = data[2] << 8 | data[3];
  priv->timestamp = sensor_get_timestamp();
  priv->valid     = true;

  return OK;
}

/****************************************************************************
 * Name: ccs811_activate
 ****************************************************************************/

static int ccs811_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct ccs811_sensor_s *priv =
      container_of(lower, FAR struct ccs811_sensor_s, lower);

  priv->enabled = enable;
  return OK;
}

/****************************************************************************
 * Name: ccs811_set_interval
 ****************************************************************************/

static int ccs811_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us)
{
  FAR struct ccs811_sensor_s *priv =
      container_of(lower, FAR struct ccs811_sensor_s, lower);

  priv->dev->interval = *period_us;
  return OK;
}

/****************************************************************************
 * Name: ccs811_fetch
 ****************************************************************************/

static int ccs811_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct ccs811_sensor_s *priv =
      container_of(lower, FAR struct ccs811_sensor_s, lower);
  FAR struct ccs811_dev_s *dev = priv->dev;
  int ret;

  ret = ccs811_measure(dev);
  if (ret < 0 && ret != -EAGAIN)
    {
      return ret;
    }

  if (!dev->valid)
    {
      return -EAGAIN;
    }

  if (lower->type == SENSOR_TYPE_CO2)
    {
      struct sensor_co2 co2;

      if (buflen != sizeof(co2))
        {
          return -EINVAL;
        }

      co2.timestamp = dev->timestamp;
      co2.co2       = (float)dev->eco2;
      memcpy(buffer, &co2, sizeof(co2));
      return sizeof(co2);
    }
  else
    {
      struct sensor_tvoc tvoc;

      if (buflen != sizeof(tvoc))
        {
          return -EINVAL;
        }

      tvoc.timestamp = dev->timestamp;
      tvoc.tvoc      = (float)dev->etvoc / 1000.0f;
      memcpy(buffer, &tvoc, sizeof(tvoc));
      return sizeof(tvoc);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ccs811_register_uorb
 *
 * Description:
 *   Register the CCS811 eCO2 and TVOC sensors as UORB devices.
 *
 * Input Parameters:
 *   devno  - The device number, used to build the device path.
 *   i2c    - The I2C bus driver instance.
 *   addr   - The I2C address of the CCS811.
 *   config - Board-specific configuration (may be NULL).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ccs811_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr,
                         FAR const struct ccs811_config_s *config)
{
  FAR struct ccs811_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct ccs811_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate CCS811 instance\n");
      return -ENOMEM;
    }

  priv->i2c      = i2c;
  priv->addr     = addr;
  priv->interval = 1000000;

  if (config != NULL)
    {
      priv->wake = config->wake;
    }

  ret = ccs811_start(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to start CCS811: %d\n", ret);
      goto errout;
    }

  /* Register the eCO2 lower-half */

  priv->co2.dev        = priv;
  priv->co2.lower.ops  = &g_ccs811_ops;
  priv->co2.lower.type = SENSOR_TYPE_CO2;

  ret = sensor_register(&priv->co2.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register CCS811 eCO2: %d\n", ret);
      goto errout;
    }

  /* Register the TVOC lower-half */

  priv->tvoc.dev        = priv;
  priv->tvoc.lower.ops  = &g_ccs811_ops;
  priv->tvoc.lower.type = SENSOR_TYPE_TVOC;

  ret = sensor_register(&priv->tvoc.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register CCS811 TVOC: %d\n", ret);
      sensor_unregister(&priv->co2.lower, devno);
      goto errout;
    }

  sninfo("CCS811 registered\n");
  return OK;

errout:
  kmm_free(priv);
  return ret;
}
