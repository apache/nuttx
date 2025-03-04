/****************************************************************************
 * drivers/sensors/fs3000_uorb.c
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
#include <stdlib.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/fs3000.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_FS3000)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FS3000_ADDR           0x28

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct fs3000_dev_s
{
  struct sensor_lowerhalf_s lower; /* Sensor private data */
  FAR struct i2c_master_s *i2c;    /* I2C interface */
  mutex_t dev_lock;                /* Manages exclusive access to the device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* FS3000 is a very simple (dumb) device */

static const struct sensor_ops_s g_sensor_ops =
{
  NULL,               /* open */
  NULL,               /* close */
  NULL,               /* activate */
  NULL,               /* set_interval */
  NULL,               /* batch */
  NULL,               /* fetch */
  NULL,               /* flush */
  NULL,               /* selftest */
  NULL,               /* set_calibvalue */
  NULL,               /* calibrate */
  NULL,               /* get_info */
  NULL                /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs3000_get_data
 *
 * Description:
 *   Read from 5 8-bit FS3000 registers
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int fs3000_get_data(FAR struct fs3000_dev_s *priv,
                           FAR uint8_t *regval)
{
  struct i2c_msg_s msg[1];
  int ret;

  /* Enforce single access */

  nxmutex_lock(&priv->dev_lock);

  msg[0].frequency = CONFIG_SENSORS_FS3000_I2C_FREQUENCY;
  msg[0].addr      = FS3000_ADDR;
  msg[0].flags     = I2C_M_READ;
  msg[0].buffer    = regval;
  msg[0].length    = 5;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed (err = %d)\n", ret);
    }

  nxmutex_unlock(&priv->dev_lock);

  sninfo("D[0] = %02x, D[1] = %02x, D[2] = %02x, D[3] = %02x, D[4] = %02x\n",
         regval[0], regval[1], regval[2], regval[3], regval[4]);

  return ret;
}

/****************************************************************************
 * Name: fs3000_get_velocity
 *
 * Description:
 *   Convert raw data to air velocity
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int fs3000_get_velocity(FAR struct fs3000_dev_s *priv, uint8_t *data,
                               FAR float *airvelocity)
{
  uint16_t value;
  int sumup;

  sumup = data[0] + data[1] + data[2] + data[3] + data[4];

  /* If sum is not divisible by 256, data is invalid */

  if ((sumup % 256) != 0)
    {
      serr("Sum of all bytes is not divisible by 256: got %d\n", sumup);
      *airvelocity = -1.0f;
      return -EINVAL;
    }

  value = (data[2] << 4) | (data[1] & 0xf) ;

  /* Use the right correlation formula depending on sensor model */

#if defined(CONFIG_SENSOR_FS3000_1005)
  *airvelocity = 0.00219f * value - 1.17f;
#else
  *airvelocity = 0.00275f * value - 1.1f;
#endif

  sinfo("Air Speed = %f\n", *airvelocity);

  return 0;
}

/****************************************************************************
 * Name: fs3000_thread
 *
 * Description:
 *   Thread for performing data readings at fixed intervals, defined by
 *   CONFIG_SENSORS_FS3000_POLL_INTERVAL
 *   @argc - Number of arguments
 *   @argv - Pointer to argument list
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int fs3000_thread(int argc, char** argv)
{
  FAR struct fs3000_dev_s *priv = (FAR struct fs3000_dev_s *)
        ((uintptr_t)strtoul(argv[1], NULL, 16));
  struct sensor_velocity velocity;
  uint8_t data[5];
  float airvelocity = 0.0;
  int ret;

  while (true)
    {
      ret = fs3000_get_data(priv, data);
      if (ret < 0)
        {
          snerr("ERROR: Failed to read data from sensor\n");
          return ret;
        }

      ret = fs3000_get_velocity(priv, data, &airvelocity);
      if (ret < 0)
        {
          goto thread_sleep;
        }

      velocity.timestamp = sensor_get_timestamp();
      velocity.velocity = airvelocity;
      priv->dev.lower.push_event(priv->dev.lower.priv, &velocity,
                                 sizeof(struct sensor_velocity));

thread_sleep:
      nxsig_usleep(CONFIG_SENSORS_FS3000_POLL_INTERVAL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs3000_register
 *
 * Description:
 *   Register the FS3000 uORB driver
 *   @i2c   - An instance of the I2C interface to use to communicate with
 *           FS3000
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

int fs3000_register(FAR struct i2c_master_s *i2c)
{
  FAR struct sensor_lowerhalf_s *lower;
  FAR struct fs3000_dev_s *priv;
  FAR char *argv[2];
  char arg1[32];
  int ret = OK;

  DEBUGASSERT(i2c != NULL);

  /* Initialize the FS3000 device structure */

  priv = kmm_zalloc(sizeof(struct fs3000_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance (err = %d)\n", ret);
      return -ENOMEM;
    }

  priv->i2c = i2c;
  nxmutex_init(&priv->dev_lock);

  /* Register the character driver */

  lower = &priv->dev.lower;
  lower->ops = &g_sensor_ops;
  lower->type = SENSOR_TYPE_VELOCITY;

  ret = sensor_register(lower, 0);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver (err = %d)\n", ret);
      goto err_init;
    }

  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("fs3000_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_FS3000_THREAD_STACKSIZE,
                       fs3000_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to create poll thread (err = %d)\n", ret);
      goto err_register;
    }

  sninfo("FS3000 driver loaded successfully!\n");
  return OK;

err_register:
  sensor_unregister(lower, 0);
err_init:
  nxmutex_destroy(&priv->dev_lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_FS3000 */
