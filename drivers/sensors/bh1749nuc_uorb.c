/****************************************************************************
 * drivers/sensors/bh1749nuc_uorb.c
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

#include <sys/param.h>

#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/kthread.h>

#include <nuttx/sensors/sensor.h>

#include "bh1749nuc_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

enum bh1749nuc_idx_e
{
  BH1749NUC_RGB_IDX = 0,
  BH1749NUC_IR_IDX,
  BH1749NUC_MAX_IDX
};

struct bh1749nuc_sensor_dev_s;
struct bh1749nuc_sensor_s
{
  struct sensor_lowerhalf_s          lower;
  int                                gain;
  FAR struct bh1749nuc_sensor_dev_s *dev;
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  unsigned long                      interval;
  uint64_t                           last_update;
#endif
  bool                               enabled;
};

struct bh1749nuc_sensor_dev_s
{
  struct bh1749nuc_sensor_s priv[BH1749NUC_MAX_IDX];
  struct bh1749nuc_dev_s    dev;
  float                     scale_r;
  float                     scale_g;
  float                     scale_b;
  float                     scale_ir;
  mutex_t                   lock;
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  sem_t                     run;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bh1749nuc_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable);
#ifndef CONFIG_SENSORS_BH1749NUC_POLL
static int bh1749nuc_fetch(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR char *buffer, size_t buflen);
#endif
static int bh1749nuc_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR unsigned long *period_us);
static int bh1749nuc_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_bh1749nuc_sensor_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  bh1749nuc_activate,
  bh1749nuc_set_interval,
  NULL,                 /* batch */
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  NULL,                 /* fetch */
#else
  bh1749nuc_fetch,
#endif
  NULL,                 /* selftest */
  NULL,                 /* set_calibvalue */
  NULL,                 /* calibrate */
  bh1749nuc_control
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1749nuc_activate
 ****************************************************************************/

static int bh1749nuc_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable)
{
  FAR struct bh1749nuc_sensor_s *priv         = NULL;
  FAR struct bh1749nuc_dev_s    *dev          = NULL;
  uint8_t                        val          = 0;
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  bool                           start_thread = false;
#endif

  /* Get dev */

  priv = (FAR struct bh1749nuc_sensor_s *)lower;
  dev = &priv->dev->dev;

  nxmutex_lock(&priv->dev->lock);

  if (enable)
    {
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
      if (!priv->enabled)
        {
          start_thread      = true;
          priv->last_update = sensor_get_timestamp();
        }
#endif

      /* MODE_CONTROL1 */

      val = (BH1749NUC_MODE_CONTROL1_MEAS_TIME160MS |
             BH1749NUC_MODE_CONTROL1_IR_GAIN_X1 |
             BH1749NUC_MODE_CONTROL1_RGB_GAIN_X1);
      bh1749nuc_putreg8(dev, BH1749NUC_MODE_CONTROL1, val);

      /* MODE_CONTROL2 */

      val = BH1749NUC_MODE_CONTROL2_RGBI_EN;
      bh1749nuc_putreg8(dev, BH1749NUC_MODE_CONTROL2, val);
    }
  else
    {
      /* Stop sampling */

      val = 0;
      bh1749nuc_putreg8(dev, BH1749NUC_MODE_CONTROL2, val);
    }

  priv->enabled = enable;

#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  if (start_thread)
    {
      /* Wake up the thread */

      nxsem_post(&priv->dev->run);
    }
#endif

  nxmutex_unlock(&priv->dev->lock);

  return OK;
}

#ifndef CONFIG_SENSORS_BH1749NUC_POLL
/****************************************************************************
 * Name: bh1749nuc_fetch
 ****************************************************************************/

static int bh1749nuc_fetch(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR char *buffer, size_t buflen)
{
  FAR struct bh1749nuc_sensor_s *priv = NULL;
  FAR struct bh1749nuc_dev_s    *dev  = NULL;
  struct sensor_rgb              rgb_data;
  struct sensor_ir               ir_data;
  uint16_t                       tmp  = 0;
  uint64_t                       now  = sensor_get_timestamp();
  int                            ret  = 0;

  /* Get dev */

  priv = (FAR struct bh1749nuc_sensor_s *)lower;
  dev = &priv->dev->dev;

  nxmutex_lock(&priv->dev->lock);

  if (!priv->enabled)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Wait for data */

  while (!(bh1749nuc_getreg8(dev, BH1749NUC_MODE_CONTROL2) &
           BH1749NUC_MODE_CONTROL2_VALID));

  if (lower->type == SENSOR_TYPE_RGB)
    {
      if (buflen != sizeof(rgb_data))
        {
          ret = -EINVAL;
          goto errout;
        }

      rgb_data.timestamp = now;
      tmp = bh1749nuc_read16(dev, BH1749NUC_RED_DATA_LSB);
      rgb_data.r = (tmp * priv->dev->scale_r);
      tmp = bh1749nuc_read16(dev, BH1749NUC_GREEN_DATA_LSB);
      rgb_data.g = (tmp * priv->dev->scale_g);
      tmp = bh1749nuc_read16(dev, BH1749NUC_BLUE_DATA_LSB);
      rgb_data.b = (tmp * priv->dev->scale_b);

      memcpy(buffer, &rgb_data, sizeof(rgb_data));
      ret = sizeof(rgb_data);
    }
  else if (lower->type == SENSOR_TYPE_IR)
    {
      if (buflen != sizeof(ir_data))
        {
          ret = -EINVAL;
          goto errout;
        }

      ir_data.timestamp = now;
      tmp = bh1749nuc_read16(dev, BH1749NUC_IR_DATA_LSB);
      ir_data.ir = (tmp * priv->dev->scale_ir);

      memcpy(buffer, &ir_data, sizeof(ir_data));
      ret = sizeof(ir_data);
    }

errout:
  nxmutex_unlock(&priv->dev->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: bh1749nuc_cotrol
 ****************************************************************************/

static int bh1749nuc_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct bh1749nuc_sensor_s *priv = NULL;
  int                            ret  = OK;

  priv = (FAR struct bh1749nuc_sensor_s *)lower;
  UNUSED(priv);

  switch (cmd)
    {
      default:
        {
          snerr("ERROR: Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bh1749nuc_set_interval
 ****************************************************************************/

static int bh1749nuc_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR unsigned long *interval)
{
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  FAR struct bh1749nuc_sensor_s *priv = NULL;

  priv = (FAR struct bh1749nuc_sensor_s *)lower;

  priv->interval = *interval;
#endif

  return OK;
}

#ifdef CONFIG_SENSORS_BH1749NUC_POLL
/****************************************************************************
 * Name: bh1749nuc_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int bh1749nuc_thread(int argc, FAR char **argv)
{
  FAR struct bh1749nuc_sensor_dev_s *dev =
    (FAR struct bh1749nuc_sensor_dev_s *)((uintptr_t)strtoul(argv[1], NULL,
                                                        16));
  FAR struct bh1749nuc_sensor_s *rgb = &dev->priv[BH1749NUC_RGB_IDX];
  FAR struct bh1749nuc_sensor_s *ir  = &dev->priv[BH1749NUC_IR_IDX];
  struct sensor_rgb              rgb_data;
  struct sensor_ir               ir_data;
  uint64_t                       min_interval;
  uint16_t                       tmp  = 0;
  uint64_t                       now  = sensor_get_timestamp();
  int                            ret  = 0;

  while (true)
    {
      if ((!rgb->enabled) && (!ir->enabled))
        {
          /* Waiting to be woken up */

          ret = nxsem_wait(&dev->run);
          if (ret < 0)
            {
              continue;
            }
        }

      /* Wait for data */

      while (!(bh1749nuc_getreg8(&dev->dev, BH1749NUC_MODE_CONTROL2) &
               BH1749NUC_MODE_CONTROL2_VALID));

      /* Get timestamp */

      now  = sensor_get_timestamp();

      if (rgb->enabled)
        {
          rgb_data.timestamp = now;
          tmp = bh1749nuc_read16(&dev->dev, BH1749NUC_RED_DATA_LSB);
          rgb_data.r = (tmp * dev->scale_r);
          tmp = bh1749nuc_read16(&dev->dev, BH1749NUC_GREEN_DATA_LSB);
          rgb_data.g = (tmp * dev->scale_g);
          tmp = bh1749nuc_read16(&dev->dev, BH1749NUC_BLUE_DATA_LSB);
          rgb_data.b = (tmp * dev->scale_b);

          rgb->lower.push_event(rgb->lower.priv,
                                &rgb_data, sizeof(rgb_data));
        }

      if (ir->enabled)
        {
          ir_data.timestamp = now;
          tmp = bh1749nuc_read16(&dev->dev, BH1749NUC_IR_DATA_LSB);
          ir_data.ir = (tmp * dev->scale_ir);

          ir->lower.push_event(ir->lower.priv,
                               &ir_data, sizeof(ir_data));
        }

      /* Sleeping thread before fetching the next sensor data */

      min_interval = MIN(rgb->interval, ir->interval);
      nxsig_usleep(min_interval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1749nuc_register_uorb
 *
 * Description:
 *   Register the BH1749NUC uorb device as 'devpath'
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   config  - device configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1749nuc_register_uorb(int devno, FAR struct bh1749nuc_config_s *config)
{
  FAR struct bh1749nuc_sensor_dev_s *dev = NULL;
  FAR struct bh1749nuc_sensor_s     *tmp = NULL;
  int                                ret = OK;
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  FAR char                          *argv[2];
  char                               arg1[32];
#endif

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the device structure. */

  dev = kmm_malloc(sizeof(struct bh1749nuc_sensor_dev_s));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate bh1749nuc device instance\n");
      return -ENOMEM;
    }

  memset(dev, 0, sizeof(struct bh1749nuc_sensor_dev_s));
  nxmutex_init(&dev->lock);
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  nxsem_init(&dev->run, 0, 0);
#endif

  /* Configure dev */

  dev->dev.i2c  = config->i2c;
  dev->dev.addr = config->addr;
  dev->dev.freq = BH1749NUC_I2C_FREQ;

  /* Check Device ID */

  ret = bh1749nuc_checkid(&dev->dev);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  /* Return data in lux unit for RGB and IR */

  dev->scale_r  = (20.0f / 71.0f) * (1e-6 / 1.46e-7);
  dev->scale_g  = (20.0f / 99.0f) * (1e-6 / 1.46e-7);
  dev->scale_b  = (20.0f / 70.0f) * (1e-6 / 1.46e-7);
  dev->scale_ir = (20.0f / 25.0f) * (1e-6 / 1.46e-7);

  /*  Register sensor */

  tmp             = &dev->priv[BH1749NUC_RGB_IDX];
  tmp->lower.type = SENSOR_TYPE_RGB;
  tmp->lower.ops  = &g_bh1749nuc_sensor_ops;
  tmp->dev        = dev;
  tmp->gain       = 1;
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  tmp->enabled    = false;
  tmp->interval   = CONFIG_SENSORS_BH1749NUC_POLL_INTERVAL;
#endif

  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto rgb_err;
    }

  /*  Register sensor */

  tmp             = &dev->priv[BH1749NUC_IR_IDX];
  tmp->lower.type = SENSOR_TYPE_IR;
  tmp->lower.ops  = &g_bh1749nuc_sensor_ops;
  tmp->dev        = dev;
  tmp->gain       = 1;
#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  tmp->enabled    = false;
  tmp->interval   = CONFIG_SENSORS_BH1749NUC_POLL_INTERVAL;
#endif

  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto ir_err;
    }

#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("bh1749nuc_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_BH1749NUC_THREAD_STACKSIZE,
                       bh1749nuc_thread,
                       argv);
  if (ret < 0)
    {
      goto thr_err;
    }
#endif

  return ret;

#ifdef CONFIG_SENSORS_BH1749NUC_POLL
  thr_err:
#endif
  sensor_unregister(&dev->priv[BH1749NUC_IR_IDX].lower, devno);
ir_err:
  sensor_unregister(&dev->priv[BH1749NUC_RGB_IDX].lower, devno);
rgb_err:
  kmm_free(dev);
  return ret;
}
