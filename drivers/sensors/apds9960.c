/****************************************************************************
 * drivers/sensors/apds9960.c
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

/* Character driver for the APDS9960 Gesture Sensor
 *
 * This driver is based on APDS-9960 Arduino library developed by
 * Shawn Hymel from SparkFun Electronics and released under public
 * domain.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/apds9960.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_APDS9960)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

#ifndef CONFIG_APDS9960_I2C_FREQUENCY
#  define CONFIG_APDS9960_I2C_FREQUENCY 400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct apds9960_dev_s
{
  FAR struct apds9960_config_s *config; /* Hardware Configuration     */
  struct work_s work;                   /* Supports ISR "bottom half" */
  struct gesture_data_s gesture_data;   /* Gesture data container     */
  int gesture_ud_delta;                 /* UP/DOWN delta              */
  int gesture_lr_delta;                 /* LEFT/RIGHT delta           */
  int gesture_ud_count;                 /* UP/DOWN counter            */
  int gesture_lr_count;                 /* LEFT/RIGHT counter         */
  int gesture_near_count;               /* Near distance counter      */
  int gesture_far_count;                /* Far distance counter       */
  int gesture_state;                    /* Gesture machine state      */
  int gesture_motion;                   /* Gesture motion direction   */
  sem_t sample_sem;                     /* Semaphore for sample data  */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Reset gesture values */

static void    apds9960_resetgesture(FAR struct apds9960_dev_s *priv);

/* Setup default initial values */

static int     apds9960_setdefault(FAR struct apds9960_dev_s *priv);

/* Probe function to verify if sensor is present */

static int     apds9960_probe(FAR struct apds9960_dev_s *priv);

/* Work queue */

static void    apds9960_worker(FAR void *arg);

/* Gesture processing/decoding functions */

static int     apds9960_readgesture(FAR struct apds9960_dev_s *priv);
static bool    apds9960_decodegesture(FAR struct apds9960_dev_s *priv);
static bool    apds9960_processgesture(FAR struct apds9960_dev_s *priv);
static bool    apds9960_isgestureavailable(FAR struct apds9960_dev_s *priv);

/* I2C Helpers */

static int     apds9960_i2c_read(FAR struct apds9960_dev_s *priv,
                                 uint8_t const regaddr, FAR uint8_t *regval,
                                 int len);
static int     apds9960_i2c_read8(FAR struct apds9960_dev_s *priv,
                                  uint8_t const regaddr,
                                  FAR uint8_t *regval);
static int     apds9960_i2c_write(FAR struct apds9960_dev_s *priv,
                                  uint8_t const *data, int len);
static int     apds9960_i2c_write8(FAR struct apds9960_dev_s *priv,
                                   uint8_t const regaddr, uint8_t regval);

/* Character driver methods */

static ssize_t apds9960_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t apds9960_write(FAR struct file *filep,
                 FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_apds9960_fops =
{
  NULL,            /* open */
  NULL,            /* close */
  apds9960_read,   /* read */
  apds9960_write,  /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apds9960_worker
 ****************************************************************************/

static void apds9960_worker(FAR void *arg)
{
  FAR struct apds9960_dev_s *priv = (FAR struct apds9960_dev_s *)arg;
  int ret;

  DEBUGASSERT(priv != NULL);

  ret = apds9960_readgesture(priv);
  if (ret != DIR_NONE)
    {
      sninfo("Got a valid gesture!\n");
    }
}

/****************************************************************************
 * Name: apds9960_int_handler
 *
 * Description:
 *   Interrupt handler (ISR) for APDS-99600 INT pin.
 *
 ****************************************************************************/

static int apds9960_int_handler(int irq, FAR void *context, FAR void *arg)
{
  int ret;

  FAR struct apds9960_dev_s *priv = (FAR struct apds9960_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Transfer processing to the worker thread.  Since APDS-9960 interrupts
   * are disabled while the work is pending, no special action should be
   * required to protect the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, apds9960_worker, priv, 0);
  if (ret != 0)
    {
      snerr("ERROR: Failed to queue work: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Name: apds9960_resetgesture
 *
 * Description:
 *   Reset gesture values
 *
 ****************************************************************************/

static void apds9960_resetgesture(FAR struct apds9960_dev_s *priv)
{
  priv->gesture_data.index = 0;
  priv->gesture_data.total_gestures = 0;

  priv->gesture_ud_delta   = 0;
  priv->gesture_lr_delta   = 0;

  priv->gesture_ud_count   = 0;
  priv->gesture_lr_count   = 0;

  priv->gesture_near_count = 0;
  priv->gesture_far_count  = 0;

  priv->gesture_state      = 0;
}

/****************************************************************************
 * Name: apds9960_setdefault
 *
 * Description:
 *   Verify if sensor is present. Check if ID is 0xAB.
 *
 ****************************************************************************/

static int apds9960_setdefault(FAR struct apds9960_dev_s *priv)
{
  int ret;

  /* Set default values for ambient light and proximity registers */

  ret = apds9960_i2c_write8(priv, APDS9960_ATIME, DEFAULT_ATIME);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_ATIME!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_WTIME, DEFAULT_WTIME);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_WTIME!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_PPULSE, DEFAULT_PPULSE);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_PPULSE!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_POFFSET_UR!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_POFFSET_DL!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_CONFIG1, DEFAULT_CONFIG1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_CONFIG1!\n");
      return ret;
    }

  /* Set LED driver strength to 100mA, AGAIN 4X and PGAIN 4X */

  ret = apds9960_i2c_write8(priv, APDS9960_CONTROL, DEFAULT_CONTROL);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_CONTROL!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_PILT, DEFAULT_PILT);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_PILT!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_PIHT, DEFAULT_PIHT);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_PIHT!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_AILTL, DEFAULT_AILTL);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_AILTL!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_AILTH, DEFAULT_AILTH);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_AILTH!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_AIHTL, DEFAULT_AIHTL);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_AIHTL!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_AIHTH, DEFAULT_AIHTH);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_AIHTH!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_PERS, DEFAULT_PERS);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_PERS!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_CONFIG2, DEFAULT_CONFIG2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_CONFIG2!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_CONFIG3, DEFAULT_CONFIG3);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_CONFIG3!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GPENTH, DEFAULT_GPENTH);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GPENTH!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GEXTH, DEFAULT_GEXTH);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GEXTH!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GCONFIG1, DEFAULT_GCONFIG1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GCONFIG1!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GCONFIG2, DEFAULT_GCONFIG2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GCONFIG2!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GOFFSET_U, DEFAULT_GOFFSET_U);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GOFFSET_U!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GOFFSET_D, DEFAULT_GOFFSET_D);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GOFFSET_D!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GOFFSET_L, DEFAULT_GOFFSET_L);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GOFFSET_L!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GOFFSET_R, DEFAULT_GOFFSET_R);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GOFFSET_R!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GPULSE, DEFAULT_GPULSE);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GPULSE!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GCONFIG3, DEFAULT_GCONFIG3);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GCONFIG3!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_GCONFIG4, DEFAULT_GCONFIG4);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GCONFIG3!\n");
      return ret;
    }

  ret = apds9960_i2c_write8(priv, APDS9960_WTIME, 0xff);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_WTIME!\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: apds9960_probe
 *
 * Description:
 *   Verify if sensor is present. Check if ID is 0xAB.
 *
 ****************************************************************************/

static int apds9960_probe(FAR struct apds9960_dev_s *priv)
{
  int ret;
  uint8_t id;

  ret = apds9960_i2c_read8(priv, APDS9960_ID, &id);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the APDS9960!\n");
      return ret;
    }

  if (id != APDS9960_ID_VAL)
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: apds9960_i2c_read
 *
 * Description:
 *   Read an arbitrary number of bytes starting at regaddr
 *
 ****************************************************************************/

static int apds9960_i2c_read(FAR struct apds9960_dev_s *priv,
                             uint8_t const regaddr,
                             FAR uint8_t *regval, int len)
{
  struct i2c_config_s config;
  int ret = -1;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_APDS9960_I2C_FREQUENCY;
  config.address   = priv->config->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address to read from */

  ret = i2c_write(priv->config->i2c_dev, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr ("i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read "len" bytes from regaddr */

  ret = i2c_read(priv->config->i2c_dev, &config, regval, len);
  if (ret < 0)
    {
      snerr ("i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: apds9960_i2c_read8
 *
 * Description:
 *   Read 8-bit register
 *
 ****************************************************************************/

static int apds9960_i2c_read8(FAR struct apds9960_dev_s *priv,
                            uint8_t const regaddr, FAR uint8_t *regval)
{
  int ret;

  ret = apds9960_i2c_read(priv, regaddr, regval, 1);

  return ret;
}

/****************************************************************************
 * Name: apds9960_i2c_write
 *
 * Description:
 *   Write an arbitrary number of bytes starting at regaddr.
 *
 ****************************************************************************/

static int apds9960_i2c_write(FAR struct apds9960_dev_s *priv,
                             uint8_t const *data, int len)
{
  struct i2c_config_s config;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_APDS9960_I2C_FREQUENCY;
  config.address   = priv->config->i2c_addr;
  config.addrlen   = 7;

  /* Write the data */

  ret = i2c_write(priv->config->i2c_dev, &config, data, len);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: apds9960_i2c_write8
 *
 * Description:
 *   Write an arbitrary number of bytes starting at regaddr.
 *
 ****************************************************************************/

static int apds9960_i2c_write8(FAR struct apds9960_dev_s *priv,
                             uint8_t const regaddr, uint8_t regval)
{
  int ret;
  uint8_t data[2];

  /* Create the addr:val data */

  data[0] = regaddr;
  data[1] = regval;

  ret = apds9960_i2c_write(priv, data, 2);

  return ret;
}

/****************************************************************************
 * Name: apds9960_isgestureavailable
 *
 * Description:
 *   Return true is gesture data is valid.
 *
 ****************************************************************************/

static bool apds9960_isgestureavailable(FAR struct apds9960_dev_s *priv)
{
  int ret;
  uint8_t val;

  /* Read value from GSTATUS register */

  ret = apds9960_i2c_read8(priv, APDS9960_GSTATUS, &val);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read APDS9960_GSTATUS!\n");
      return ret;
    }

  /* Return true/false based on GVALID bit */

  if ((val & GVALID) == GVALID)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: apds9960_processgesture
 *
 * Description:
 *   Process the data read from the photodiodes
 *
 ****************************************************************************/

static bool apds9960_processgesture(FAR struct apds9960_dev_s *priv)
{
  uint8_t u_first = 0;
  uint8_t d_first = 0;
  uint8_t l_first = 0;
  uint8_t r_first = 0;
  uint8_t u_last  = 0;
  uint8_t d_last  = 0;
  uint8_t l_last  = 0;
  uint8_t r_last  = 0;
  int ud_ratio_first;
  int lr_ratio_first;
  int ud_ratio_last;
  int lr_ratio_last;
  int ud_delta;
  int lr_delta;
  int i;

  /* If we have less than 4 total gestures, that's not enough */

  if (priv->gesture_data.total_gestures <= 4)
    {
      snerr("ERROR: We don't have enough gesture: %d\n",
            priv->gesture_data.total_gestures);
      return false;
    }

  /* Check to make sure our data isn't out of bounds */

  if ((priv->gesture_data.total_gestures <= 32) && \
      (priv->gesture_data.total_gestures > 0))
    {
      /* Find the first value in U/D/L/R above the threshold */

      for (i = 0; i < priv->gesture_data.total_gestures; i++)
        {
          if ((priv->gesture_data.u_data[i] > GESTURE_THRESHOLD_OUT) && \
              (priv->gesture_data.d_data[i] > GESTURE_THRESHOLD_OUT) && \
              (priv->gesture_data.l_data[i] > GESTURE_THRESHOLD_OUT) && \
              (priv->gesture_data.r_data[i] > GESTURE_THRESHOLD_OUT))
            {
              u_first = priv->gesture_data.u_data[i];
              d_first = priv->gesture_data.d_data[i];
              l_first = priv->gesture_data.l_data[i];
              r_first = priv->gesture_data.r_data[i];
              break;
            }
        }

      /* If one of the _first values is 0, then there is no good data */

      if ((u_first == 0) || (d_first == 0) || \
          (l_first == 0) || (r_first == 0))
        {
          snerr("ERROR: First value is zero! U=%d, D=%d, L=%d, R=%d\n", \
                u_first, d_first, l_first, r_first);
          return false;
        }

      /* Find the last value in U/D/L/R above the threshold */

      for (i = priv->gesture_data.total_gestures - 1; i >= 0; i--)
        {
          sninfo("Finding last:\n");
          sninfo("U: %03d\n", priv->gesture_data.u_data[i]);
          sninfo("D: %03d\n", priv->gesture_data.d_data[i]);
          sninfo("L: %03d\n", priv->gesture_data.l_data[i]);
          sninfo("R: %03d\n", priv->gesture_data.r_data[i]);

          if ((priv->gesture_data.u_data[i] > GESTURE_THRESHOLD_OUT) &&
              (priv->gesture_data.d_data[i] > GESTURE_THRESHOLD_OUT) &&
              (priv->gesture_data.l_data[i] > GESTURE_THRESHOLD_OUT) &&
              (priv->gesture_data.r_data[i] > GESTURE_THRESHOLD_OUT))
            {
              u_last = priv->gesture_data.u_data[i];
              d_last = priv->gesture_data.d_data[i];
              l_last = priv->gesture_data.l_data[i];
              r_last = priv->gesture_data.r_data[i];
              break;
            }
        }
    }

  /* Calculate the first vs. last ratio of up/down and left/right */

  ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
  lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
  ud_ratio_last  = ((u_last  - d_last)  * 100) / (u_last  + d_last);
  lr_ratio_last  = ((l_last  - r_last)  * 100) / (l_last  + r_last);

  sninfo("Last Values:\n");
  sninfo("U: %03d\n", u_last);
  sninfo("D: %03d\n", d_last);
  sninfo("L: %03d\n", l_last);
  sninfo("R: %03d\n", r_last);

  sninfo("Ratios:\n");
  sninfo("UD Fi: %03d\n", ud_ratio_first);
  sninfo("UD La: %03d\n", ud_ratio_last);
  sninfo("LR Fi: %03d\n", lr_ratio_first);
  sninfo("LR La: %03d\n", lr_ratio_last);

  /* Determine the difference between the first and last ratios */

  ud_delta = ud_ratio_last - ud_ratio_first;
  lr_delta = lr_ratio_last - lr_ratio_first;

  sninfo("Deltas:\n");
  sninfo("UD: %03d\n", ud_delta);
  sninfo("LR: %03d\n", lr_delta);

  /* Accumulate the UD and LR delta values */

  priv->gesture_ud_delta += ud_delta;
  priv->gesture_lr_delta += lr_delta;

  sninfo("Accumulations:\n");
  sninfo("UD: %03d\n", priv->gesture_ud_delta);
  sninfo("LR: %03d\n", priv->gesture_lr_delta);

  /* Determine U/D gesture */

  if (priv->gesture_ud_delta >= GESTURE_SENSITIVITY_1)
    {
      priv->gesture_ud_count = 1;
    }
  else
    {
      if (priv->gesture_ud_delta <= -GESTURE_SENSITIVITY_1)
        {
          priv->gesture_ud_count = -1;
        }
      else
        {
          priv->gesture_ud_count = 0;
        }
    }

  /* Determine L/R gesture */

  if (priv->gesture_lr_delta >= GESTURE_SENSITIVITY_1)
    {
      priv->gesture_lr_count = 1;
    }
  else
    {
      if (priv->gesture_lr_delta <= -GESTURE_SENSITIVITY_1)
        {
          priv->gesture_lr_count = -1;
        }
      else
        {
          priv->gesture_lr_count = 0;
        }
    }

  /* Determine Near/Far gesture */

  if ((priv->gesture_ud_count == 0) && (priv->gesture_lr_count == 0))
    {
      if ((abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
          (abs(lr_delta) < GESTURE_SENSITIVITY_2))
        {
          if ((ud_delta == 0) && (lr_delta == 0))
            {
              priv->gesture_near_count++;
            }
          else
            {
              if ((ud_delta != 0) || (lr_delta != 0))
                {
                  priv->gesture_far_count++;
                }
            }

          if ((priv->gesture_near_count >= 10) && \
              (priv->gesture_far_count >= 2))
            {
              if ((ud_delta == 0) && (lr_delta == 0))
                {
                  priv->gesture_state = NEAR_STATE;
                }
              else
                {
                  if ((ud_delta != 0) && (lr_delta != 0))
                    {
                      priv->gesture_state = FAR_STATE;
                    }
                }

              return true;
            }
        }
    }
  else
    {
      if ((abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
          (abs(lr_delta) < GESTURE_SENSITIVITY_2))
        {
          if ((ud_delta == 0) && (lr_delta == 0))
            {
              priv->gesture_near_count++;
            }

          if (priv->gesture_near_count >= 10)
            {
              priv->gesture_ud_count = 0;
              priv->gesture_lr_count = 0;
              priv->gesture_ud_delta = 0;
              priv->gesture_lr_delta = 0;
            }
        }
    }

  sninfo(" UD_CT: %03d\n", priv->gesture_ud_count);
  sninfo(" LR_CT: %03d\n", priv->gesture_lr_count);
  sninfo(" NEAR_CT: %03d\n", priv->gesture_near_count);
  sninfo(" FAR_CT:  %03d\n", priv->gesture_far_count);
  sninfo("----------------------\n");

  return false;
}

/****************************************************************************
 * Name: apds9960_decodegesture
 *
 * Description:
 *   Decode the sensor data and return true if there is some valid data
 *
 ****************************************************************************/

static bool apds9960_decodegesture(FAR struct apds9960_dev_s *priv)
{
  /* Return if near or far event is detected */

  if (priv->gesture_state == NEAR_STATE)
    {
      priv->gesture_motion = DIR_NEAR;
      return true;
    }
  else
    {
      if (priv->gesture_state == FAR_STATE)
        {
          priv->gesture_motion = DIR_FAR;
          return true;
        }
    }

  /* Determine swipe direction */

  if ((priv->gesture_ud_count == -1) && (priv->gesture_lr_count == 0))
    {
      priv->gesture_motion = DIR_UP;
    }
  else
    {
      if ((priv->gesture_ud_count == 1) && (priv->gesture_lr_count == 0))
        {
          priv->gesture_motion = DIR_DOWN;
        }
      else
        {
          if ((priv->gesture_ud_count == 0) && (priv->gesture_lr_count == 1))
            {
              priv->gesture_motion = DIR_RIGHT;
            }
          else
            {
              if ((priv->gesture_ud_count == 0) &&
                  (priv->gesture_lr_count == -1))
                {
                  priv->gesture_motion = DIR_LEFT;
                }
              else
                {
                  if ((priv->gesture_ud_count == -1) &&
                      (priv->gesture_lr_count == 1))
                    {
                      if (abs(priv->gesture_ud_delta) > \
                          abs(priv->gesture_lr_delta))
                        {
                          priv->gesture_motion = DIR_UP;
                        }
                      else
                        {
                          priv->gesture_motion = DIR_RIGHT;
                        }
                    }
                  else
                    {
                      if ((priv->gesture_ud_count == 1) && \
                          (priv->gesture_lr_count == -1))
                        {
                          if (abs(priv->gesture_ud_delta) > \
                              abs(priv->gesture_lr_delta))
                            {
                              priv->gesture_motion = DIR_DOWN;
                            }
                          else
                            {
                              priv->gesture_motion = DIR_LEFT;
                            }
                        }
                      else
                        {
                          if ((priv->gesture_ud_count == -1) && \
                              (priv->gesture_lr_count == -1))
                            {
                              if (abs(priv->gesture_ud_delta) > \
                                  abs(priv->gesture_lr_delta))
                                {
                                  priv->gesture_motion = DIR_UP;
                                }
                              else
                                {
                                  priv->gesture_motion = DIR_LEFT;
                                }
                            }
                          else
                            {
                              if ((priv->gesture_ud_count == 1) && \
                                  (priv->gesture_lr_count == 1))
                                {
                                  if (abs(priv->gesture_ud_delta) > \
                                      abs(priv->gesture_lr_delta))
                                    {
                                      priv->gesture_motion = DIR_DOWN;
                                    }
                                  else
                                    {
                                      priv->gesture_motion = DIR_RIGHT;
                                    }
                                }
                              else
                                {
                                  return false;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

  return true;
}

/****************************************************************************
 * Name: apds9960_readgesture
 *
 * Description:
 *   Read the photodiode data, process/decode it and return the guess
 *
 ****************************************************************************/

static int apds9960_readgesture(FAR struct apds9960_dev_s *priv)
{
  uint8_t fifo_level = 0;
  uint8_t bytes_read = 0;
  uint8_t fifo_data[128];
  uint8_t gstatus;
  int motion;
  int ret;
  int i;

  /* Make sure that power and gesture is on and data is valid */

  if (!apds9960_isgestureavailable(priv))
    {
      return DIR_NONE;
    }

  /* Keep looping as long as gesture data is valid */

  while (1)
    {
      /* Wait some time to collect next batch of FIFO data */

      nxsig_usleep(FIFO_PAUSE_TIME);

      /* Get the contents of the STATUS register. Is data still valid? */

      ret = apds9960_i2c_read8(priv, APDS9960_GSTATUS, &gstatus);
      if (ret < 0)
        {
          snerr("ERROR: Failed to read APDS9960_GSTATUS!\n");
          return ret;
        }

      /* If we have valid data, read in FIFO */

      if ((gstatus & GVALID) == GVALID)
        {
          /* Read the current FIFO level */

          ret = apds9960_i2c_read8(priv, APDS9960_GFLVL, &fifo_level);
          if (ret < 0)
            {
              snerr("ERROR: Failed to read APDS9960_GFLVL!\n");
              return ret;
            }

          sninfo("FIFO Level: %d\n", fifo_level);

          /* If there's stuff in the FIFO, read it into our data block */

          if (fifo_level > 0)
            {
              bytes_read = fifo_level * 4;
              ret = apds9960_i2c_read(priv, APDS9960_GFIFO_U,
                                      (uint8_t *) fifo_data, bytes_read);
              if (ret < 0)
                {
                  snerr("ERROR: Failed to read APDS9960_GFIFO_U!\n");
                  return ret;
                }

              sninfo("\nFIFO Dump:\n");
              for (i = 0; i < fifo_level; i++)
                {
                   sninfo("U: %03d | D: %03d | L: %03d | R: %03d\n",
                          fifo_data[i], fifo_data[i + 1],
                          fifo_data[i + 2], fifo_data[i + 3]);
                }

              sninfo("\n");

              /* If at least 1 set of data, sort the data into U/D/L/R */

              if (bytes_read >= 4)
                {
                  for (i = 0; i < bytes_read; i += 4)
                    {
                      priv->gesture_data.u_data[priv->gesture_data.index] =
                        fifo_data[i + 0];
                      priv->gesture_data.d_data[priv->gesture_data.index] =
                        fifo_data[i + 1];
                      priv->gesture_data.l_data[priv->gesture_data.index] =
                        fifo_data[i + 2];
                      priv->gesture_data.r_data[priv->gesture_data.index] =
                        fifo_data[i + 3];
                      priv->gesture_data.index++;
                      priv->gesture_data.total_gestures++;
                    }

                  sninfo("Up Data:\n");
                  for (i = 0; i < priv->gesture_data.total_gestures; i++)
                    {
                      sninfo("%03d\n", priv->gesture_data.u_data[i]);
                    }

                  sninfo("\n");

                  /* Filter and process gesture data. Decode near/far state */

                  if (apds9960_processgesture(priv))
                    {
                      if (apds9960_decodegesture(priv))
                        {
                          /* TODO: U-Turn Gestures */
                        }
                    }

                  /* Reset data */

                  priv->gesture_data.index = 0;
                  priv->gesture_data.total_gestures = 0;
                }
            }
        }
      else
        {
          /* Determine best guessed gesture and clean up */

          nxsig_usleep(FIFO_PAUSE_TIME);
          apds9960_decodegesture(priv);
          motion = priv->gesture_motion;

          snwarn("END: %d\n", priv->gesture_motion);

          if (motion == DIR_LEFT)
            {
              snwarn("RESULT = LEFT\n");
            }

          if (motion == DIR_RIGHT)
            {
              snwarn("RESULT = RIGHT\n");
            }

          if (motion == DIR_UP)
            {
              snwarn("RESULT = UP\n");
            }

          if (motion == DIR_DOWN)
            {
              snwarn("RESULT = DOWN\n");
            }

          /* Increase semaphore to indicate new data */

          nxsem_post(&priv->sample_sem);

          apds9960_resetgesture(priv);
          return motion;
        }
    }
}

/****************************************************************************
 * Name: apds9960_read
 ****************************************************************************/

static ssize_t apds9960_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode         *inode;
  FAR struct apds9960_dev_s *priv;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct apds9960_dev_s *)inode->i_private;

  /* Check if the user is reading the right size */

  if (buflen < 1)
    {
      snerr("ERROR: You need to read at least 1 byte from this sensor!\n");
      return -EINVAL;
    }

  /* Wait for data available */

  ret = nxsem_wait_uninterruptible(&priv->sample_sem);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  buffer[0] = (char) priv->gesture_motion;
  buflen    = 1;

  return buflen;
}

/****************************************************************************
 * Name: apds9960_write
 ****************************************************************************/

static ssize_t apds9960_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apds9960_register
 *
 * Description:
 *   Register the APDS9960 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gest0"
 *   i2c - An instance of the I2C interface to communicate with APDS9960
 *   addr - The I2C address of the APDS9960.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9960_register(FAR const char *devpath,
                      FAR struct apds9960_config_s *config)
{
  int ret;

  /* Initialize the APDS9960 device structure */

  FAR struct apds9960_dev_s *priv =
    (FAR struct apds9960_dev_s *)kmm_zalloc(sizeof(struct apds9960_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config         = config;
  priv->gesture_motion = DIR_NONE;
  nxsem_init(&priv->sample_sem, 0, 0);

  /* Probe APDS9960 device */

  ret = apds9960_probe(priv);
  if (ret != OK)
    {
      snerr("ERROR: APDS-9960 is not responding!\n");
      return ret;
    }

  /* Turn the device OFF to make it sane */

  ret = apds9960_i2c_write8(priv, APDS9960_ENABLE, 0);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the APDS9960!\n");
      return ret;
    }

  /* Wait 100ms */

  nxsig_usleep(100000);

  /* Initialize the device (leave RESET) */

  ret = apds9960_i2c_write8(priv, APDS9960_ENABLE, PON);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the APDS9960!\n");
      return ret;
    }

  /* Set default initial register values */

  ret = apds9960_setdefault(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the APDS9960!\n");
      return ret;
    }

  /* Reset gesture values */

  apds9960_resetgesture(priv);

  /* Enable the Gesture mode and interruptions */

  ret = apds9960_i2c_write8(priv, APDS9960_GCONFIG4, (GMODE | GIEN));
  if (ret < 0)
    {
      snerr("ERROR: Failed to write APDS9960_GCONFIG4!\n");
      return ret;
    }

  /* Enable the Gesture mode (Proximity mode is needed for gesture mode) */

  ret = apds9960_i2c_write8(priv, APDS9960_ENABLE, PON | PEN | GEN | WEN);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the APDS9960!\n");
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_apds9960_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  /* Attach to the interrupt */

  priv->config->irq_attach(priv->config, apds9960_int_handler, priv);

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_APDS9960 */
