/****************************************************************************
 * drivers/sensors/apds9922.c
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

/* Character driver for the APDS9922 Proximity and Ambient Light Sensor     */

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/signal.h>

#include <nuttx/sensors/apds9922.h>
#include <sys/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_APDS9922)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

#ifndef CONFIG_APDS9922_I2C_FREQUENCY
#  define CONFIG_APDS9922_I2C_FREQUENCY 400000
#endif

#ifndef CONFIG_APDS9922_ALS_NPOLLWAITERS
#  define CONFIG_APDS9922_ALS_NPOLLWAITERS 2
#endif

#ifndef CONFIG_APDS9922_PS_NPOLLWAITERS
#  define CONFIG_APDS9922_PS_NPOLLWAITERS 2
#endif

/* Helper macros */

#ifdef CONFIG_ENDIAN_BIG
#  define APDS9922_PACK_TO_UINT32(a) \
   (((a)[0] >> 24) | ((a)[1] >> 16) | ((a)[2] >> 8) | ((a)[3]))
#  define APDS9922_PACK_TO_UINT16(a) \
   (((a)[0] >> 0) | ((a)[1]))
#  define APDS9922_UNPACK_FROM_UINT32(w, b) \
  do \
    { \
      (b)[0] = ((w) >> 24) & 0xff; \
      (b)[1] = ((w) >> 16) & 0xff; \
      (b)[2] = ((w) >> 8) & 0xff; \
      (b)[3] = (w) & 0xff; \
    } \
  while (0)
#  define APDS9922_UNPACK_FROM_UINT16(w, b) \
  do \
    { \
      (b)[0] = ((w) >> 8) & 0xff; \
      (b)[1] = (w) & 0xff; \
    } \
  while (0)
#else
#  define APDS9922_PACK_TO_UINT32(a) \
   (((a)[3] << 24) | ((a)[2] << 16) | ((a)[1] << 8) | ((a)[0]))
#  define APDS9922_PACK_TO_UINT16(a) \
   (((a)[1] << 8) | ((a)[0]))
#  define APDS9922_UNPACK_FROM_UINT32(w, b) \
  do \
    { \
      (b)[3] = ((w) >> 24) & 0xff; \
      (b)[2] = ((w) >> 16) & 0xff; \
      (b)[1] = ((w) >> 8) & 0xff; \
      (b)[0] = (w) & 0xff; \
    } \
  while (0)
#  define APDS9922_UNPACK_FROM_UINT16(w, b) \
  do \
    { \
      (b)[1] = ((w) >> 8) & 0xff; \
      (b)[0] = (w) & 0xff; \
    } \
  while (0)
#endif

/* Register mappings */

#define APDS9922_MAIN_CTRL      (0x00) /* SW reset, ALS Enable, PS enable  */
#define APDS9922_PS_LED         (0x01) /* PS LED setup                     */
#define APDS9922_PS_PULSES      (0x02) /* PS pulses setup                  */
#define APDS9922_PS_MEAS_RATE   (0x03) /* PS Measurement rate              */
#define APDS9922_ALS_MEAS_RATE  (0x04) /* ALS Measurement rate             */
#define APDS9922_ALS_GAIN       (0x05) /* ALS gain                         */
#define APDS9922_ID             (0x06) /* Part and Revision ID             */
#define APDS9922_MAIN_STATUS    (0x07) /* Status register                  */
#define APDS9922_PS_DATA0       (0x08) /* LSB of measured PS data          */
#define APDS9922_ALS_DATA0      (0x0d) /* LSB of measured ALS data         */
#define APDS9922_INT_CFG        (0x19) /* Interrupt configuration          */
#define APDS9922_INT_PERSIST    (0x1a) /* Interrupt persistance            */
#define APDS9922_PS_THRESHU     (0x1b) /* PS threshold, upper limit        */
#define APDS9922_PS_THRESHL     (0x1d) /* PS threshold, lower limit        */
#define APDS9922_CANCEL_LVLL    (0x1f) /* Intelligent Cancellation level   */
#define APDS9922_CANCEL_LVLU    (0x20) /* Intelligent Cancellation level   */
#define APDS9922_ALS_THRESHU    (0x21) /* ALS threshold, upper limit       */
#define APDS9922_ALS_THRESHL    (0x24) /* ALS threshold, lower limit       */
#define APDS9922_ALS_THRESH_VAR (0x27) /* ALS threshold variation          */

/* APDS9922_MAIN_CTRL Register 0x01 */

#define PS_ACTIVE_SHIFT         (0)
#define PS_ACTIVE               (1 << PS_ACTIVE_SHIFT)
#define ALS_ACTIVE_SHIFT        (1)
#define ALS_ACTIVE              (1 << ALS_ACTIVE_SHIFT)
#define SW_RESET_SHIFT          (4)
#define APDS9922_SW_RESET       (1 << SW_RESET_SHIFT)

/* APDS922_PS_LED register 0x02 */

#define PS_LED_FREQ_SHIFT       (4)
#define PS_LED_FREQ_MASK        (7 << PS_LED_FREQ_SHIFT)
#define PS_SET_LED_FREQ(f)      ((f) << PS_LED_FREQ_SHIFT)
#define PS_LED_PEAKING_ON       (1 << 3)
#define PS_LED_CURRENT_SHIFT    (0)
#define PS_LED_CURRENT_MASK     (7 << PS_LED_CURRENT_SHIFT)
#define PS_SET_LED_CURRENT(i)   ((i) << PS_LED_CURRENT_SHIFT)

/* APDS922_PS_PULSES register  0x03 */

#define PS_LED_PULSES_MASK      (0x0fff)
#define PS_SET_LED_PULSES(p)    ((p) & PS_LED_PULSES_MASK)

/* APDS922_PS_MEAS_RATE 0x03 */

#define PS_RESOLUTION_SHIFT     (3)
#define PS_RESOLUTION_MASK      (3 << PS_RESOLUTION_SHIFT)
#define PS_SET_RESOLUTION(r)    (((r) << PS_RESOLUTION_SHIFT) | 0x20)
#define PS_MEASURERATE_SHIFT    (0)
#define PS_MEASURERATE_MASK     (7 << PS_MEASURERATE_SHIFT)
#define PS_SET_MEASURERATE(r)   ((r) << PS_MEASURERATE_SHIFT)

/* APDS922_ALS_MEAS_RATE 0x04 */

#define ALS_RESOLUTION_SHIFT    (4)
#define ALS_RESOLUTION_MASK     (7 << ALS_RESOLUTION_SHIFT)
#define ALS_SET_RESOLUTION(r)   ((r) << ALS_RESOLUTION_SHIFT)
#define ALS_MEASURERATE_SHIFT   (0)
#define ALS_MEASURERATE_MASK    (7 << ALS_MEASURERATE_SHIFT)
#define ALS_SET_MEASURERATE(r)  ((r) << ALS_MEASURERATE_SHIFT)

/* APDS922_ALS_GAIN 0x05 */

#define ALS_GAIN_SHIFT          (0)
#define ALS_GAIN_MASK           (7 << ALS_GAIN_SHIFT)
#define ALS_SET_GAIN(g)         ((g) << ALS_GAIN_SHIFT)

/* APDS_ALS_MAIN_STATUS 0x07 */

#define ALS_INT_STATUS          (16)
#define ALS_NEW_DATA            (8)
#define PS_LOGIC_STATUS         (4)
#define PS_INT_STATUS           (2)
#define PS_NEW_DATA             (1)

/* APDS9922_PS_DATA0 0x08 */

#define PS_DATA_OVERFLOW_SHIFT  (3)
#define PS_DATA_OVERFLOW        (1 << PS_DATA_OVERFLOW_SHIFT)

/* APDS9922_INT_CFG 0x19 */

#define PS_INT_EN_SHIFT         (0)
#define PS_INT_EN               (1 << PS_INT_EN_SHIFT)
#define PS_INT_MASK             (1 << PS_INT_EN_SHIFT)
#define PS_LOGIC_MODE_SHIFT     (1)
#define PS_LOGIC_MODE_NORMAL    (0 << PS_LOGIC_MODE_SHIFT)
#define PS_LOGIC_MODE_LOGIC     (1 << PS_LOGIC_MODE_SHIFT)

#define ALS_INT_EN_SHIFT        (2)
#define ALS_INT_EN              (1 << ALS_INT_EN_SHIFT)
#define ALS_INT_MASK            (5 << ALS_INT_EN_SHIFT)
#define ALS_INT_VAR_SHIFT       (3)
#define ALS_INT_VAR_MODE        (1 << ALS_INT_VAR_SHIFT)
#define ALS_INT_THRESH_MODE     (0 << ALS_INT_VAR_SHIFT)
#define ALS_INT_SRC_SHIFT       (4)
#define ALS_INT_SRC_MASK        (3 << ALS_INT_SRC_SHIFT)
#define ALS_INT_SET_SRC(s)      ((s) << ALS_INT_SRC_SHIFT)

/* APDS922_INT_PERSIST 0x1a */

#define ALS_PERSISTANCE_SHIFT   (4)
#define ALS_PERSISTANCE_MASK    (15 << ALS_PERSISTANCE_SHIFT)
#define ALS_SET_PERSISTANCE(p)  ((p) << ALS_PERSISTANCE_SHIFT)
#define ALS_PERSISTANCE_MAX     (255)
#define PS_PERSISTANCE_SHIFT    (0)
#define PS_PERSISTANCE_MASK     (15 << PS_PERSISTANCE_SHIFT)
#define PS_SET_PERSISTANCE(p)   ((p) << PS_PERSISTANCE_SHIFT)
#define PS_PERSISTANCE_MAX      (255)

/* APDS922_ALS_THRESH_VAR 0x27  */

#define ALS_THRESH_VAR_SHIFT    (0)
#define ALS_THRESH_VAR_MASK     (7 << ALS_THRESH_VAR_SHIFT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct als_data
{
  uint32_t gain;   /* Gain multiplier                                      */
  uint32_t rate;   /* Corresponding rate in ms                             */
  uint32_t maxval; /* Maximum value the als value can attain for this rate */
};

static const struct als_data als_data[] =
{
  {1,  400, 1048575},
  {3,  200, 524287},
  {6,  100, 262143},
  {9,  50, 131071},
  {18, 25, 65535},
};

struct apds9922_dev_s
{
  FAR struct pollfd            *fds_als[CONFIG_APDS9922_ALS_NPOLLWAITERS];
  FAR struct pollfd            *fds_ps[CONFIG_APDS9922_PS_NPOLLWAITERS];
  struct work_s                work;      /* Handles  interrupt         */
  mutex_t                      devlock;   /* Manages exclusive access   */
  FAR struct apds9922_config_s *config;   /* Platform specific config   */
  struct apds9922_als_setup_s  als_setup; /* Device ALS config          */
  struct apds9922_ps_setup_s   ps_setup;  /* Device PS config           */
  int                          als;       /* ALS data                   */
  FAR struct apds9922_ps_data  *ps_data;  /* PS data                    */
  uint8_t                      devid;     /* Device ID read at startup  */
  int                          crefs;     /* Number of opens, als or ps */
  };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Probe function to verify if sensor is present */

static int     apds9922_probe(FAR struct apds9922_dev_s *priv);

/* Work queue */

static void    apds9922_worker(FAR void *arg);

/* i2c read/write functions */

static int     apds9922_i2c_read(FAR struct apds9922_dev_s *priv,
                                 uint8_t const regaddr, FAR uint8_t *regval,
                                 int len);
static int     apds9922_i2c_read8(FAR struct apds9922_dev_s *priv,
                                  uint8_t const regaddr,
                                  FAR uint8_t *regval);
static int     apds9922_i2c_write(FAR struct apds9922_dev_s *priv,
                                  uint8_t const regaddr,
                                  uint8_t const *data, int len);
static int     apds9922_i2c_write8(FAR struct apds9922_dev_s *priv,
                                   uint8_t const regaddr, uint8_t regval);

/* local functions */

static int apds9922_reset(FAR struct apds9922_dev_s *priv);

/* Ambient light sensor functions */

static int apds9922_als_config(FAR struct apds9922_dev_s *priv,
                               FAR struct apds9922_als_setup_s *config);
static int apds9922_lux_calc(FAR struct apds9922_dev_s *priv);
static int apds9922_als_gain(FAR struct apds9922_dev_s *priv, int gain);
static int apds9922_autogain(FAR struct apds9922_dev_s *priv, bool enable);
static int apds9922_als_resolution(FAR struct apds9922_dev_s *priv, int res);
static int apds9922_als_rate(FAR struct apds9922_dev_s *priv, int rate);
static int apds9922_als_persistance(FAR struct apds9922_dev_s *priv,
                                    uint8_t persistance);
static int apds9922_als_variance(FAR struct apds9922_dev_s *priv,
                                 int variance);
static int apds9922_als_thresh(FAR struct apds9922_dev_s *priv,
                               FAR struct adps9922_als_thresh thresholds);
static int apds9922_als_int_mode(FAR struct apds9922_dev_s *priv, int mode);
static int apds9922_als_channel(FAR struct apds9922_dev_s *priv,
                                int channel);
static int apds9922_als_factor(FAR struct apds9922_dev_s *priv,
                               uint32_t factor);
static int apds9922_als_limit(FAR struct apds9922_dev_s *priv,
                              uint32_t limit);

/* Proximity sensor functions */

static int apds9922_ps_config(FAR struct apds9922_dev_s *priv,
                              FAR struct apds9922_ps_setup_s *config);
static int apds9922_ps_resolution(FAR struct apds9922_dev_s *priv, int res);
static int apds9922_ps_rate(FAR struct apds9922_dev_s *priv, int rate);
static int apds9922_ps_ledf(FAR struct apds9922_dev_s *priv, int freq);
static int apds9922_ps_ledi(FAR struct apds9922_dev_s *priv, int current);
static int apds9922_ps_ledpk(FAR struct apds9922_dev_s *priv, bool enable);
static int apds9922_ps_pulses(FAR struct apds9922_dev_s *priv,
                              uint8_t num_p);
static int apds9922_ps_thresh(FAR struct apds9922_dev_s *priv,
                              FAR struct adps9922_ps_thresh thresh);
static int apds9922_ps_canc_lev(FAR struct apds9922_dev_s *priv,
                                uint16_t lev);
static int apds9922_ps_int_mode(FAR struct apds9922_dev_s *priv, int mode);
static int apds9922_ps_persistance(FAR struct apds9922_dev_s *priv,
                                   uint8_t persistance);
static int apds9922_ps_notify_mode(FAR struct apds9922_dev_s *priv,
                                   int notify);

/* Character driver methods */

static int     apds9922_open(FAR struct file *filep);
static int     apds9922_close(FAR struct file *filep);
static ssize_t apds9922_als_read(FAR struct file *filep,
                                 FAR char *, size_t buflen);
static ssize_t apds9922_als_write(FAR struct file *filep,
                                  FAR const char *buffer, size_t buflen);
static int     apds9922_als_ioctl(FAR struct file *filep, int cmd,
                                  unsigned long arg);
static int     apds9922_als_poll(FAR struct file *filep,
                                 FAR struct pollfd *fds, bool setup);
static ssize_t apds9922_ps_read(FAR struct file *filep,
                                FAR char *, size_t buflen);
static ssize_t apds9922_ps_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen);
static int     apds9922_ps_ioctl(FAR struct file *filep, int cmd,
                                 unsigned long arg);
static int     apds9922_ps_poll(FAR struct file *filep,
                                FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_apds9922_alsfops =
{
  apds9922_open,      /* open  */
  apds9922_close,     /* close */
  apds9922_als_read,  /* read  */
  apds9922_als_write, /* write */
  NULL,               /* seek  */
  apds9922_als_ioctl, /* ioctl */
  NULL,               /* mmap  */
  NULL,               /* truncate */
  apds9922_als_poll,  /* poll  */
};

static const struct file_operations g_apds9922_psfops =
{
  apds9922_open,      /* open  */
  apds9922_close,     /* close */
  apds9922_ps_read,   /* read  */
  apds9922_ps_write,  /* write */
  NULL,               /* seek  */
  apds9922_ps_ioctl,  /* ioctl */
  NULL,               /* mmap  */
  NULL,               /* truncate */
  apds9922_ps_poll,   /* poll  */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apds9922_worker
 *
 * Description:
 *   Worker task to deal with new device interrupt
 *
 * Input Parameters:
 *   arg - Pointer to device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void apds9922_worker(FAR void *arg)
{
  FAR struct apds9922_dev_s *priv = (FAR struct apds9922_dev_s *)arg;
  int ret;
  uint8_t status;
  uint8_t data[4];
  bool notify_ps;

  DEBUGASSERT(priv);

  ret = apds9922_i2c_read8(priv, APDS9922_MAIN_STATUS, &status);
  if (ret < 0)
    {
      snerr("Failed to read status: %d\n", ret);
      goto err_out;
    }

  if (status & ALS_INT_STATUS)
    {
      ret = apds9922_i2c_read(priv, APDS9922_ALS_DATA0, data, 3);
      if (ret < 0)
        {
        snerr("Failed to read als data: %d\n", ret);
          goto err_out;
        }

      priv->als = APDS9922_PACK_TO_UINT32(data) & 0x0fffff;
      poll_notify(priv->fds_als, CONFIG_APDS9922_ALS_NPOLLWAITERS, POLLIN);
    }

  if (status & PS_INT_STATUS)
    {
      notify_ps = false;
      if (priv->ps_setup.notify != PS_FAR_OR_CLOSE_ONLY)
        {
          ret = apds9922_i2c_read(priv, APDS9922_PS_DATA0, data, 2);
          if (ret < 0)
            {
              snerr("Failed to read ps data: %d\n", ret);
              goto err_out;
            }

          priv->ps_data->ps = APDS9922_PACK_TO_UINT16(data) & 0x0fff;
          notify_ps = true;
        }

      if ((priv->ps_setup.notify != PS_PROXIMITY_DATA_ONLY) &&
         (priv->ps_data->close != (status & PS_LOGIC_STATUS)))
        {
          notify_ps = true;
          priv->ps_data->close = (status & PS_LOGIC_STATUS) ? true : false;
        }

      sninfo("INFO: ps=0x%x\t close=%d\n",
             priv->ps_data->ps, priv->ps_data->close);

      if (notify_ps)
        {
          poll_notify(priv->fds_ps, CONFIG_APDS9922_PS_NPOLLWAITERS, POLLIN);
        }
    }

  /* if there's been a fail, there's an issue with the device.
   * Set proximity and lux to error value and notify.
   */

err_out:

  if (ret < 0)
    {
      priv->als         = ret;
      priv->ps_data->ps = ret;
      snerr("ERR: Error while dealing with worker \n");

      poll_notify(priv->fds_als, CONFIG_APDS9922_ALS_NPOLLWAITERS, POLLIN);
      poll_notify(priv->fds_ps, CONFIG_APDS9922_PS_NPOLLWAITERS, POLLIN);
    }
}

/****************************************************************************
 * Name: apds9922_int_handler
 *
 * Description:
 *   Interrupt handler (ISR) for APDS-9922 INT pin.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg     - Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_int_handler(int irq, FAR void *context, FAR void *arg)
{
  int ret;

  FAR struct apds9922_dev_s *priv = (FAR struct apds9922_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Transfer processing to the worker thread.  Since APDS-9922 interrupts
   * are disabled until the data is read, no special action should be
   * required to protect the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);

  ret = work_queue(HPWORK, &priv->work, apds9922_worker, priv, 0);
  if (ret < 0)
    {
      snerr("ERROR: Failed to queue work: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: apds9922_reset
 *
 * Description:
 *   Reset the chip
 *
 * Input Parameters:
 *   priv    - pointer to device structure
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_reset(FAR struct apds9922_dev_s *priv)
{
  int ret;

  ret = apds9922_i2c_write8(priv, APDS9922_MAIN_CTRL, APDS9922_SW_RESET);
  if (ret < 0)
    {
      snerr("ERROR: Failed to reset the APDS9922\n");
      return ret;
    }

  /* initialise setup to match the reset defaults etc. */

  priv->als_setup.rate         = ALS_RATE100MS;
  priv->als_setup.res          = ALS_RES200MS;
  priv->als_setup.thresh.upper = ALS_DEF_THRESHU;
  priv->als_setup.thresh.lower = ALS_DEF_THRESHL;
  priv->als_setup.thresh_var   = ALS_DEF_VAR;
  priv->als_setup.int_mode     = ALS_INT_MODE_THRESHOLD;
  priv->als_setup.persistance  = ALS_DEF_PERSISTANCE;
  priv->als_setup.als_factor   = 1;
  priv->als_setup.range_lim    = 1;
  priv->als_setup.autogain     = false;
  priv->als_setup.channel      = ALS_VISIBLE;

  priv->ps_setup.rate          = PS_RATE100MS;
  priv->ps_setup.res           = PS_RES8;
  priv->ps_setup.led_f         = PS_LED_FREQ60K;
  priv->ps_setup.led_pk_on     = false;
  priv->ps_setup.led_i         = PS_LED_CURRENT100MA;
  priv->ps_setup.pulses        = PS_DEF_PULSES;
  priv->ps_setup.thresh.upper  = PS_DEF_THRESHU;
  priv->ps_setup.thresh.lower  = PS_DEF_THRESHL;
  priv->ps_setup.cancel_lev    = PS_DEF_CANCEL_LVL;
  priv->ps_setup.persistance   = PS_DEF_PERSISTANCE;
  priv->ps_setup.notify        = PS_ALL_INFO;
  priv->ps_setup.int_mode      = PS_INT_MODE_NORMAL;

  /* Wait for device to power up properly after reset */

  nxsig_usleep(50000);

  return OK;
}

/****************************************************************************
 * Name: apds9922_probe
 *
 * Description:
 *   Verify if sensor is present. Check if ID is 0xAB.
 *
 * Input Parameters:
 *   priv    - pointer to device structure
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_probe(FAR struct apds9922_dev_s *priv)
{
  int ret;
  uint8_t id = 0;

  ret = apds9922_i2c_read8(priv, APDS9922_ID, &id);
  if (ret < 0)
    {
      snerr("ERROR: Failed to probe the APDS9922\n");
      return ret;
    }

  if (id != APDS9922_ID_VAL)
    {
      snerr("ERROR: APDS9922 device ID is incorrect\n");
      return -ENODEV;
    }

  priv->devid = id;

  return OK;
}

/****************************************************************************
 * Name: apds_als_config
 *
 * Description:
 *   Set the measurement resolution required.
 *
 * Input Parameters:
 *  priv    - pointer to device structure
 *  config  - pointer to the apds9922_als_setup_s config struct
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

/* Ambient light sensor functions */

static int apds9922_als_config(FAR struct apds9922_dev_s *priv,
                               FAR struct apds9922_als_setup_s *config)
{
  int ret;

  ret = apds9922_als_factor(priv, config->als_factor);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_als_limit(priv, config->range_lim);
  if (ret < 0)
    {
      return ret;
    }

  /* Do gain before autogain as autogain will change gain as well */

  ret = apds9922_autogain(priv, config->autogain);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_als_gain(priv, config->gain);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_als_resolution(priv, config->res);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_als_rate(priv, config->rate);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_als_persistance(priv, config->persistance);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_als_variance(priv, config->thresh_var);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_als_thresh(priv, config->thresh);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_als_channel(priv, config->channel);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_als_int_mode(priv, config->int_mode);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: apds_als_resolution
 *
 * Description:
 *   Set the measurement resolution required.
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  res  - resolution to be used
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_resolution(FAR struct apds9922_dev_s *priv, int res)
{
  uint8_t regval;
  int ret;

  ret = apds9922_i2c_read8(priv, APDS9922_ALS_MEAS_RATE, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~ALS_RESOLUTION_MASK;
  regval |= ALS_SET_RESOLUTION(res);
  ret = apds9922_i2c_write8(priv, APDS9922_ALS_MEAS_RATE, regval);
  priv->als_setup.res = res;

  return ret;
}

/****************************************************************************
 * Name: apds9922_als_channel
 *
 * Description:
 *   Sets the ALS interrupt channel - visible or IR light.
 *
 * Input Parameters:
 *  priv    - pointer to device structure
 *  channel - interrupt channel source
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_channel(FAR struct apds9922_dev_s *priv, int channel)
{
  uint8_t regval;
  int ret;

  if (channel > ALS_VISIBLE)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_INT_CFG, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~ALS_INT_SRC_MASK;
  regval |= ALS_INT_SET_SRC(channel);

  ret = apds9922_i2c_write8(priv, APDS9922_INT_CFG, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->als_setup.channel = channel;

  return OK;
}

/****************************************************************************
 * Name: apds9922_als_factor
 *
 * Description:
 *   Sets the ALS correction factor, used for lux calculation
 *
 * Input Parameters:
 *  priv   - pointer to device structure
 *  factor - als factor to use
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_factor(FAR struct apds9922_dev_s *priv,
                               uint32_t factor)
{
  if (factor < 1)
    {
      return -EINVAL;
    }

  priv->als_setup.als_factor = factor;

  return OK;
}

/****************************************************************************
 * Name: apds9922_als_limit
 *
 * Description:
 *   Sets the ALS auto range limit - "limit percent" of full scale value
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  limi - limit to use (1-100 %)
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_limit(FAR struct apds9922_dev_s *priv,
                              uint32_t limit)
{
  if ((limit < 1) || (limit > 100))
    {
      return -EINVAL;
    }

  priv->als_setup.range_lim = limit;

  return OK;
}

/****************************************************************************
 * Name: apds9922_als_int_mode
 *
 * Description:
 *   Sets the ALS interrupt mode - disabled, threshold or variance.
 *
 * Input Parameters:
 *  priv    - pointer to device structure
 *  channel - interrupt mode
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_int_mode(FAR struct apds9922_dev_s *priv, int mode)
{
  uint8_t regval;
  int ret;

  if (mode > ALS_INT_MODE_VARIANCE)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_INT_CFG, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~ALS_INT_MASK;

  switch (mode)
    {
      case ALS_INT_MODE_VARIANCE:
        regval |= ALS_INT_VAR_MODE | ALS_INT_EN;
        break;
      case ALS_INT_MODE_THRESHOLD:
        regval |= ALS_INT_EN;
        break;
      case ALS_INT_MODE_DISABLED:
      default:
        break;
    }

  ret = apds9922_i2c_write8(priv, APDS9922_INT_CFG, regval);

  if (ret < 0)
    {
      return ret;
    }

  priv->als_setup.int_mode = mode;

  return OK;
}

/****************************************************************************
 * Name: apds9922_als_thresh
 *
 * Description:
 *   Sets the ALS thresholds, upper and lower.
 *
 * Input Parameters:
 *  priv       - pointer to device structure
 *  thresholds - struct of thresholds to set
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_thresh(FAR struct apds9922_dev_s *priv,
                                       FAR struct adps9922_als_thresh
                                                  thresholds)
{
  int res_index = priv->als_setup.res;
  uint32_t threshmax = als_data[res_index].maxval;
  int ret;
  uint8_t data[8];

  /* Make the values are within the current device resolution setting */

  if (thresholds.upper > threshmax)
    {
      snerr(
       "ALS upper threshold out of range: %" PRIu32 ", max: %" PRIu32 "\n",
        thresholds.upper, threshmax);
      return -EINVAL;
    }

  if (thresholds.lower > threshmax)
    {
      snerr(
       "ALS lower threshold out of range: %" PRIu32 ", max: %" PRIu32 "\n",
        thresholds.lower, threshmax);
      return -EINVAL;
    }

  APDS9922_UNPACK_FROM_UINT32(thresholds.upper, data);
  APDS9922_UNPACK_FROM_UINT32(thresholds.lower, data + 3);

  ret = apds9922_i2c_write(priv, APDS9922_ALS_THRESHU, data, 6);
  if (ret < 0)
    {
      return ret;
    }

  priv->als_setup.thresh = thresholds;

  return ret;
}

/****************************************************************************
 * Name: apds9922_als_variance
 *
 * Description:
 *   Sets the ALS threshold variance.
 *
 * Input Parameters:
 *  priv     - pointer to device structure
 *  variance - the value to set
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_variance(FAR struct apds9922_dev_s *priv,
                                 int variance)
{
  int ret;

  if (variance > ALS_VAR1024)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_write8(priv, APDS9922_ALS_THRESH_VAR, variance);
  if (ret < 0)
    {
      return ret;
    }

    priv->als_setup.thresh_var = variance;

  return OK;
}

/****************************************************************************
 * Name: apds9922_als_persistance
 *
 * Description:
 *   Set the number of consecutive int events needed before int is asserted.
 *
 * Input Parameters:
 *  priv        - pointer to device structure
 *  persistance - number of values to be out of range before int asserted
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_persistance(FAR struct apds9922_dev_s *priv,
                                    uint8_t persistance)
{
  uint8_t regval;
  int ret;

  if (persistance > ALS_PERSISTANCE_MAX)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_INT_PERSIST, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~ALS_PERSISTANCE_MASK;
  regval |= ALS_SET_PERSISTANCE(persistance);
  ret = apds9922_i2c_write8(priv, APDS9922_INT_PERSIST, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->als_setup.persistance = persistance;

  return OK;
}

/****************************************************************************
 * Name: apds_als_measure_rate
 *
 * Description:
 *   Set the measurement rate required.
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  rate - measurement rate required
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_rate(FAR struct apds9922_dev_s *priv, int rate)
{
  uint8_t regval;
  int ret;

  if (rate > ALS_RATE4000MS)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_ALS_MEAS_RATE, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~ALS_MEASURERATE_MASK;
  regval |= ALS_SET_MEASURERATE(rate);
  ret = apds9922_i2c_write8(priv, APDS9922_ALS_MEAS_RATE, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->als_setup.rate = rate;

  return OK;
}

/****************************************************************************
 * Name: apds9922_autogain
 *
 * Description:
 *   Enables/disables gain range adjustment.
 *   This keeps the ADC counts in optimum range and starts with max gain
 *
 * Input Parameters:
 *  priv   - pointer to device structure
 *  enable - enable/disable autogain
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_autogain(FAR struct apds9922_dev_s *priv,
                             bool enable)
{
  int ret;

  ret = apds9922_als_gain(priv, ALS_GAINX18);
  if (ret < 0)
    {
      return ret;
    }

  priv->als_setup.autogain = enable;

  return OK;
}

/****************************************************************************
 * Name: apds9922_als_gain
 *
 * Description:
 *   Sets the ALS  gain.
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  gain - the gain to set
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_als_gain(FAR struct apds9922_dev_s *priv, int gain)
{
  int ret;

  if (gain > ALS_GAINX18)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_write8(priv, APDS9922_ALS_GAIN, gain);
  if (ret < 0)
    {
      return ret;
    }

  priv->als_setup.gain = gain;

  return OK;
}

/****************************************************************************
 * Name: apds9922_lux_calc
 *
 * Description:
 *   Calculate lux value from the current als value.
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  als  - the raw value of als from the sensor to work with
 *
 *
 * Returned Value:
 *   Calculated lux or -errno if failed
 *
 ****************************************************************************/

static int apds9922_lux_calc(FAR struct apds9922_dev_s *priv)
{
  uint32_t lux;
  int      thresh_h;
  int      thresh_l;
  int      ret;
  uint32_t als      = (uint32_t)priv->als;
  int      gain_idx = priv->als_setup.gain;
  uint32_t gain     = als_data[gain_idx].gain;
  int      res_idx  = priv->als_setup.res;
  uint32_t limit    = priv->als_setup.range_lim;
  uint32_t factor   = priv->als_setup.als_factor;
  uint32_t res      = als_data[res_idx].rate;
  uint32_t fs       = als_data[res_idx].maxval;

  lux = (als * factor) / (res * gain);

  thresh_l = (fs * limit) / 100;
  thresh_h = (fs * (100 - limit)) / 100;

  if (priv->als_setup.autogain)
    {
      /* Ensure ALS gain is optimised to keep als value within "range_lim %"
       * of the maximum range of the ADC, as determined by the resolution
       * setting (and above 0 by the same amount).
       */

      gain_idx = priv->als_setup.gain;
      if (als >= thresh_h)
        {
          if (gain_idx > ALS_GAINX1)
            {
              gain_idx--;
            }
        }
      else if (als < thresh_l)
        {
          if (gain_idx < ALS_GAINX18)
            {
              gain_idx++;
            }
        }

      if (gain_idx != priv->als_setup.gain)
        {
          ret = apds9922_als_gain(priv, gain);
          if (ret < 0)
            {
              return ret;
            }

          priv->als_setup.gain = gain_idx;
          sninfo("Auto gain changed ok: %" PRIu32 "\n", gain);
        }
    }

  return (int)lux;
}

/* Proximity sensor functions */

/****************************************************************************
 * Name: apds_ps_config
 *
 * Description:
 *   Set the measurement resolution required.
 *
 * Input Parameters:
 *  priv    - pointer to device structure
 *  config  - pointer to the apds9922_ps_setup_s config struct
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_config(FAR struct apds9922_dev_s *priv,
                              FAR struct apds9922_ps_setup_s *config)
{
  int ret;

  ret = apds9922_ps_resolution(priv, config->res);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_rate(priv, config->rate);
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_ledf(priv, config->led_f)  ;
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_ledi(priv, config->led_i)  ;
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_ledpk(priv, config->led_pk_on)  ;
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_pulses(priv, config->pulses)  ;
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_thresh(priv, config->thresh)  ;
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_canc_lev(priv, config->cancel_lev)  ;
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_persistance(priv, config->persistance)  ;
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_notify_mode(priv, config->notify)  ;
  if (ret < 0)
    {
      return ret;
    }

  ret = apds9922_ps_int_mode(priv, config->int_mode)  ;
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: apds_ps_resolution
 *
 * Description:
 *   Set the measurement resolution required.
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  res  - resolution to be used
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_resolution(FAR struct apds9922_dev_s *priv, int res)
{
  int ret;
  uint8_t regval;

  if (res > PS_RES11)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_PS_MEAS_RATE, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~PS_RESOLUTION_MASK;
  regval |= PS_SET_RESOLUTION(res);
  ret = apds9922_i2c_write8(priv, APDS9922_PS_MEAS_RATE, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.res = res;

  return OK;
}

/****************************************************************************
 * Name: apds_ps_measure_rate
 *
 * Description:
 *   Set the measurement rate required.
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  rate - measurement rate required
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_rate(FAR struct apds9922_dev_s *priv, int rate)
{
  uint8_t regval;
  int ret;

  if (rate > PS_RATE400MS)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_PS_MEAS_RATE, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~PS_MEASURERATE_MASK;
  regval |= PS_SET_MEASURERATE(rate);
  ret = apds9922_i2c_write8(priv, APDS9922_PS_MEAS_RATE, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.rate = rate;

  return OK;
}

/****************************************************************************
 * Name: apds9922_ps_ledf
 *
 * Description:
 *   Set the LED pulse modulation rate required.
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  freq - LED frequency
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_ledf(FAR struct apds9922_dev_s *priv, int freq)
{
  uint8_t regval;
  int ret;

  if ((freq > PS_LED_FREQ100K) || (freq < PS_LED_FREQ60K))
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_PS_LED, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~PS_LED_FREQ_MASK;
  regval |= PS_SET_LED_FREQ(freq);
  ret = apds9922_i2c_write8(priv, APDS9922_PS_LED, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.led_f = freq;

  return OK;
}

/****************************************************************************
 * Name: apds9922_ps_ledi
 *
 * Description:
 *   Set the LED current required.
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  rate - LED current
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_ledi(FAR struct apds9922_dev_s *priv, int current)
{
  uint8_t regval;
  int ret;

  if (current > PS_LED_CURRENT125MA)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_PS_LED, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~PS_LED_CURRENT_MASK;
  regval |= PS_SET_LED_CURRENT(current);
  ret = apds9922_i2c_write8(priv, APDS9922_PS_LED, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.led_i = current;

  return OK;
}

/****************************************************************************
 * Name: apds9922_ps_ledpk
 *
 * Description:
 *   Turn LED peaking on/off.
 *
 * Input Parameters:
 *  priv   - pointer to device structure
 *  enable - enable or disable peaking
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_ledpk(FAR struct apds9922_dev_s *priv, bool enable)
{
  uint8_t regval;
  int ret;

  ret = apds9922_i2c_read8(priv, APDS9922_PS_LED, &regval);
  if (ret < 0)
    {
      return ret;
    }

  if (enable)
    {
      regval |= PS_LED_PEAKING_ON;
    }
  else
    {
      regval &= ~PS_LED_PEAKING_ON;
    }

  ret = apds9922_i2c_write8(priv, APDS9922_PS_LED, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.led_pk_on = enable;

  return OK;
}

/****************************************************************************
 * Name: apds9922_ps_pulses
 *
 * Description:
 *   Set the number of LED pulses.
 *
 * Input Parameters:
 *  priv   - pointer to device structure
 *  num_p  - the number of pulses
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_pulses(FAR struct apds9922_dev_s *priv, uint8_t num_p)
{
  int ret;

  ret = apds9922_i2c_write8(priv, APDS9922_PS_PULSES, num_p);

  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.pulses = num_p;

  return OK;
}

/****************************************************************************
 * Name: apds9922_ps_thresh
 *
 * Description:
 *   Sets the PS thresholds, upper and lower.
 *
 * Input Parameters:
 *  priv       - pointer to device structure
 *  thresholds - struct of thresholds to set
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_thresh(FAR struct apds9922_dev_s *priv,
                              FAR struct adps9922_ps_thresh thresholds)
{
  int res_index = priv->ps_setup.res;
  uint32_t threshmax = 256 << res_index;
  int ret;
  uint8_t data[4];

  /* Make the values are within the current device resolution setting */

  if (thresholds.upper > threshmax)
    {
      snerr("ERROR: ps upper threshold out of range: %d, max: %" PRIu32 "\n",
             thresholds.upper, threshmax);
      return -EINVAL;
    }

  if  (thresholds.lower > threshmax)
    {
      snerr("ERROR: ps lower threshold out of range: %d, max: %" PRIu32 "\n",
             thresholds.lower, threshmax);
      return -EINVAL;
    }

  APDS9922_UNPACK_FROM_UINT16(thresholds.upper, data);
  APDS9922_UNPACK_FROM_UINT16(thresholds.lower, data + 2);

  ret = apds9922_i2c_write(priv, APDS9922_PS_THRESHU, data, 4);
  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.thresh = thresholds;

  return OK;
}

/****************************************************************************
 * Name: apds9922_ps_canc_lev
 *
 * Description:
 *   Sets the PS cancellation level to compensate for reading when nothing is
 *   near to the sensor, due to housing. overlays, etc.
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  lev  - struct of thresholds to set
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_canc_lev(FAR struct apds9922_dev_s *priv,
                                uint16_t lev)
{
  int ret;
  int res_index = priv->ps_setup.res;
  int levmax = 256 << res_index;
  uint8_t data[2];

  /* Make the values are within the current device resolution setting */

  if (lev > levmax)
    {
      return -EINVAL;
    }

  APDS9922_UNPACK_FROM_UINT16(lev, data);
  ret = apds9922_i2c_write(priv, APDS9922_CANCEL_LVLL, data, 2);
  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.cancel_lev = lev;

  return OK;
}

/****************************************************************************
 * Name: apds9922_ps_int_mode
 *
 * Description:
 *   Sets the PS interrupt mode - disabled, logic or normal.
 *
 * Input Parameters:
 *  priv    - pointer to device structure
 *  channel - interrupt mode
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_int_mode(FAR struct apds9922_dev_s *priv, int mode)
{
  int ret;
  uint8_t regval;

  if (mode > PS_INT_MODE_NORMAL)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_INT_CFG, &regval);

  if (ret < 0)
    {
      return ret;
    }

  regval &= ~PS_INT_MASK;

  switch (mode)
    {
      case PS_INT_MODE_NORMAL:
        regval |= PS_LOGIC_MODE_NORMAL | PS_INT_EN;
        break;
      case PS_INT_MODE_LOGIC:
        regval |= PS_LOGIC_MODE_LOGIC | PS_INT_EN;
        break;
      case PS_INT_MODE_DISABLED:
      default:
        break;
    }

  ret = apds9922_i2c_write8(priv, APDS9922_INT_CFG, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.int_mode = mode;

  return OK;
}

/****************************************************************************
 * Name: apds9922_ps_persistance
 *
 * Description:
 *   Set the number of consecutive int events needed before int is asserted.
 *
 * Input Parameters:
 *  priv        - pointer to device structure
 *  persistance - number of values to be out of range before int asserted
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_persistance(FAR struct apds9922_dev_s *priv,
                                   uint8_t persistance)
{
  uint8_t regval;
  int ret;

  if (persistance > PS_PERSISTANCE_MAX)
    {
      return -EINVAL;
    }

  ret = apds9922_i2c_read8(priv, APDS9922_INT_PERSIST, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~PS_PERSISTANCE_MASK;
  regval |= PS_SET_PERSISTANCE(persistance);
  ret = apds9922_i2c_write8(priv, APDS9922_INT_PERSIST, regval);
  if (ret < 0)
    {
      return ret;
    }

  priv->ps_setup.persistance = persistance;

  return OK;
}

/****************************************************************************
 * Name: apds9922_ps_notify_mode
 *
 * Description:
 *   Set the rules for poll notify: proxmity value, far/close, or both.
 *
 * Input Parameters:
 *  priv        - pointer to device structure
 *  notify - number of values to be out of range before int asserted
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_ps_notify_mode(FAR struct apds9922_dev_s *priv,
                                   int notify)
{
  if (notify > PS_FAR_OR_CLOSE_ONLY)
    {
      return -EINVAL;
    }

  priv->ps_setup.notify = notify;

  return OK;
}

/* i2c helper functions */

/****************************************************************************
 * Name: apds9922_i2c_read
 *
 * Description:
 *   Read an arbitrary number of bytes starting at regaddr
 *
 ****************************************************************************/

static int apds9922_i2c_read(FAR struct apds9922_dev_s *priv,
                             uint8_t const regaddr,
                             FAR uint8_t *regval, int len)
{
  struct i2c_config_s config;
  int                 ret;
  irqstate_t          flags;

  DEBUGASSERT(priv);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_APDS9922_I2C_FREQUENCY;
  config.address   = priv->config->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address to read from */

  flags = spin_lock_irqsave(NULL);
  ret = i2c_write(priv->config->i2c, &config, &regaddr, 1);
  spin_unlock_irqrestore(NULL, flags);
  if (ret < 0)
    {
      snerr ("i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read "len" bytes from regaddr */

  flags = spin_lock_irqsave(NULL);
  ret = i2c_read(priv->config->i2c, &config, regval, len);
  spin_unlock_irqrestore(NULL, flags);
  if (ret < 0)
    {
      snerr ("i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: apds9922_i2c_read8
 *
 * Description:
 *   Read 8-bit register
 *
 ****************************************************************************/

static int apds9922_i2c_read8(FAR struct apds9922_dev_s *priv,
                              uint8_t const regaddr, FAR uint8_t *regval)
{
  int ret;

  ret = apds9922_i2c_read(priv, regaddr, regval, 1);

  return ret;
}

/****************************************************************************
 * Name: apds9922_i2c_write
 *
 * Description:
 *   Write an arbitrary number of bytes starting at regaddr.
 *
 ****************************************************************************/

static int apds9922_i2c_write(FAR struct apds9922_dev_s *priv,
                              uint8_t const regaddr,
                              FAR uint8_t const *data, int len)
{
  struct i2c_config_s config;
  int                 ret;
  irqstate_t          flags;
  uint8_t             *buffer;

  buffer = (uint8_t *)kmm_malloc((len + 1) * sizeof(uint8_t));
  if (!buffer)
    {
      snerr("ERROR: Failed to create i2c  write buffer space\n");
      return -ENOMEM;
    }

  /* Set up the I2C configuration */

  config.frequency = CONFIG_APDS9922_I2C_FREQUENCY;
  config.address   = priv->config->i2c_addr;
  config.addrlen   = 7;

  buffer[0] = regaddr;
  memcpy(&buffer[1], data, len);

  /* Write the data */

  flags = spin_lock_irqsave(NULL);
  ret = i2c_write(priv->config->i2c, &config, buffer, len + 1);
  spin_unlock_irqrestore(NULL, flags);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
    }

  kmm_free(buffer);

  return ret;
}

/****************************************************************************
 * Name: apds9922_i2c_write8
 *
 * Description:
 *   Write an single byte of date to regaddr.
 *
 ****************************************************************************/

static int apds9922_i2c_write8(FAR struct apds9922_dev_s *priv,
                             uint8_t const regaddr, uint8_t regval)
{
  int ret;

  ret = apds9922_i2c_write(priv, regaddr, &regval, 1);

  return ret;
}

/****************************************************************************
 * Name: apds9922_open
 *
 * Description:
 *   Standard character driver close method.
 *
 * Input Parameters:
 *   filep - file structure pointer
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct apds9922_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->crefs == 0)
    {
      priv->config->irq_attach(priv->config, apds9922_int_handler, priv);
      priv->config->irq_enable(priv->config, true);
    }

  priv->crefs++;

  nxmutex_unlock(&priv->devlock);

  return OK;
}

/****************************************************************************
 * Name: apds9922_close
 *
 * Description:
 *   Standard character driver close method.
 *
 * Input Parameters:
 *   filep - file structure pointer
 *
 * Returned Value:
 *   Success or failure
 *
 ****************************************************************************/

static int apds9922_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct apds9922_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(priv->crefs > 0);

  if (priv->crefs == 0)
    {
     apds9922_reset(priv);
     apds9922_als_int_mode(priv, ALS_INT_MODE_DISABLED);
     apds9922_ps_int_mode(priv, PS_INT_MODE_DISABLED);
     priv->config->irq_detach(priv->config);
    }

  nxmutex_unlock(&priv->devlock);

  return OK;
}

/****************************************************************************
 * Name: apds9922_als_read
 *
 * Description:
 *   Standard character driver read method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   buffer - Buffer to write
 *   buflen - The write length of the buffer
 *
 * Returned Value:
 *   Size of buffer read
 *
 ****************************************************************************/

static ssize_t apds9922_als_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct inode *inode;

  FAR struct apds9922_dev_s *priv;
  int *ptr;
  int ret;

  DEBUGASSERT(filep);

  inode = filep->f_inode;
  priv = inode->i_private;

  DEBUGASSERT(inode && inode->i_private);

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (buflen < 1)
    {
      snerr("ERROR: Buffer not large enough to read data\n");
      return (ssize_t)-EINVAL;
    }

  ptr = (int *)buffer;

  if (priv->als < 0)
    {
      *ptr = priv->als;
    }
  else
    {
      *ptr = apds9922_lux_calc(priv);
    }

  nxmutex_unlock(&priv->devlock);

  return buflen;
}

/****************************************************************************
 * Name: apds9922_als_write
 *
 * Description:
 *   Standard character driver write method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   buffer - Buffer to write
 *   buflen - The write length of the buffer
 *
 * Returned Value:
 *   -ENOSYS - this driver does not support the write method
 *
 ****************************************************************************/

static ssize_t apds9922_als_write(FAR struct file *filep,
                                  FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: apds9922_als_ioctl
 *
 *  Description:
 *    This routine is called when ioctl function call is performed for
 *    the ambient light sensor of the apds9922 device.
 *
 * Input Parameters:
 *   filep - file structure pointer.
 *   cmd   - The IOCTL command.
 *   arg - The argument of the IOCTL command.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int apds9922_als_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct apds9922_dev_s *priv = inode->i_private;
  int ret;
  FAR uint8_t *ptr;

  static struct apds9922_als_setup_s *als_setup;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  sninfo("cmd: 0x%" PRIx16 ", arg: %ld\n", cmd, arg);

  switch (cmd)
    {
      case SNIOC_RESET:
        ret = apds9922_reset(priv);
        break;
      case SNIOC_ALS_CONFIG:
        als_setup = (struct apds9922_als_setup_s *)arg;
        ret = apds9922_als_config(priv, als_setup);
        break;
      case SNIOC_GET_DEV_ID:
        {
          ptr = (FAR uint8_t *)arg;
          DEBUGASSERT(ptr != NULL);
          *ptr = priv->devid;
          ret = OK;
        }
        break;
      default:
        {
          snerr("ERROR: Unrecognized cmd: %x\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  nxmutex_unlock(&priv->devlock);

  return ret;
}

/****************************************************************************
 * Name: apds9922_als_poll
 *
 * Description:
 *   Standard character driver poll method
 *
 * Input Parameters:
 *   filep - file structure pointer
 *   fds   - Array of file descriptor
 *   setup - 1 if start poll, 0 if stop poll
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int apds9922_als_poll(FAR struct file *filep,
                             FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode;
  FAR struct apds9922_dev_s *priv;
  int ret;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct apds9922_dev_s *)inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll. Find an available
       * slot for the poll structure reference.
       */

      for (i = 0; i < CONFIG_APDS9922_ALS_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds_als[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds_als[i] = fds;
              fds->priv = &priv->fds_als[i];
              break;
            }
        }

      if (i >= CONFIG_APDS9922_ALS_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: apds9922_ps_read
 *
 * Description:
 *   Standard character driver read method.
 *
 * Input Parameters:
 *   filep  - File structure pointer
 *   buffer - Buffer to write
 *   buflen - The write length of the buffer
 *
 * Returned Value:
 *   Size of buffer read
 *
 ****************************************************************************/

static ssize_t apds9922_ps_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct inode            *inode = filep->f_inode;
  FAR struct apds9922_dev_s   *priv = inode->i_private;
  FAR struct apds9922_ps_data *ptr;
  int ret;

  DEBUGASSERT(filep);

  DEBUGASSERT(inode && inode->i_private);

  if (buflen < sizeof(struct apds9922_ps_data))
    {
      snerr("ERROR: Buffer not large enough to read data\n");
      return (ssize_t)-EINVAL;
    }

  ptr = (struct apds9922_ps_data *)buffer;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  *ptr = *priv->ps_data;

  nxmutex_unlock(&priv->devlock);

  return buflen;
}

/****************************************************************************
 * Name: apds9922_ps_write
 *
   * Description:
 *   Standard character driver write method.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   buffer - Buffer to write
 *   buflen - The write length of the buffer
 *
 * Returned Value:
 *   -ENOSYS - this driver does not support the write method
 *
 ****************************************************************************/

static ssize_t apds9922_ps_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: apds9922_ps_ioctl
 *
 *  Description:
 *    This routine is called when ioctl function call is performed for
 *    the proximity sensor of the apds9922 device.
 *
 * Input Parameters:
 *   filep - file structure pointer.
 *   cmd   - The IOCTL command.
 *   arg - The argument of the IOCTL command.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int apds9922_ps_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct apds9922_dev_s *priv = inode->i_private;
  int ret;
  FAR uint8_t *ptr;
  static struct apds9922_ps_setup_s  *ps_setup;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  sninfo("cmd: 0x%02X, arg:%lu\n", cmd, arg);

  switch (cmd)
    {
      case SNIOC_PS_CONFIG:
        ps_setup = (struct apds9922_ps_setup_s *)arg;
        ret = OK;
        ret = apds9922_ps_config(priv, ps_setup);
        break;
      case SNIOC_GET_DEV_ID:
        {
          ptr = (FAR uint8_t *)arg;
          DEBUGASSERT(ptr != NULL);
          *ptr = priv->devid;
          ret = OK;
        }
        break;
      case SNIOC_PS_CANC_LVL:
        {
          ret = apds9922_ps_canc_lev(priv, (uint16_t)arg);
        }
        break;
      default:
        {
          snerr("ERROR: Unrecognized cmd: %x\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  nxmutex_unlock(&priv->devlock);

  return ret;
}

/****************************************************************************
 * Name: apds9922_ps_poll
 *
 * Description:
 *   Standard character driver poll method
 *
 * Input Parameters:
 *   filep - file structure pointer
 *   fds   - Array of file descriptor
 *   setup - 1 if start poll, 0 if stop poll
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int apds9922_ps_poll(FAR struct file *filep,
                             FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode;
  FAR struct apds9922_dev_s *priv;
  int ret;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct apds9922_dev_s *)inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll. Find an available
       * slot for the poll structure reference.
       */

      for (i = 0; i < CONFIG_APDS9922_PS_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds_ps[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds_ps[i] = fds;
              fds->priv = &priv->fds_ps[i];
              break;
            }
        }

      if (i >= CONFIG_APDS9922_PS_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apds9922_register
 *
 * Description:
 *   Register the APDS9922 character devices.
 *
 * Input Parameters:
 *   devpath_als - The full path to the driver to register for the als,
 *   e.g., "/dev/als0". If NULL the dreiver will not be registered.
 *
 *   devpath_ps - The full path to the driver to register for the als,
 *   e.g., "/dev/ps0". If NULL the dreiver will not be registered.
 *
 *   config - Pointer to the device configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9922_register(FAR const char *devpath_als,
                      FAR const char *devpath_ps,
                      FAR struct apds9922_config_s *config)
{
  int ret;
  uint8_t regval;

  /* Initialize the APDS9922 device structure */

  FAR struct apds9922_dev_s *priv;

  priv = kmm_zalloc(sizeof(*priv));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  nxmutex_init(&priv->devlock);

  priv->config = config;

  /* Reset the device to make it sane */

  apds9922_reset(priv);

  /* Probe APDS9922 device */

  ret = apds9922_probe(priv);
  if (ret < 0)
    {
      goto err_out;
    }

  /* Enable ALS and/or PS */

  regval = (devpath_ps != NULL)   ? PS_ACTIVE  : 0;
  regval |= (devpath_als != NULL) ? ALS_ACTIVE : regval;
  ret = apds9922_i2c_write8(priv, APDS9922_MAIN_CTRL, regval);
  if (ret < 0)
    {
      snerr("ERROR: Failed to enable als and/or ps.\n");
      goto err_out;
    }

  /* device interrupts are enabled by default. Disable them. */

  ret = apds9922_als_int_mode(priv, ALS_INT_MODE_DISABLED);
  if (ret < 0)
    {
      snerr("ERROR: Failed to disable ALS interrupts.\n");
      goto err_out;
    }

  apds9922_ps_int_mode(priv, PS_INT_MODE_DISABLED);
  if (ret < 0)
    {
      snerr("ERROR: Failed to disable PS interrupts.\n");
      goto err_out;
    }

  /* Register the character driver */

  if (devpath_als != NULL)
    {
      ret = register_driver(devpath_als, &g_apds9922_alsfops, 0666, priv);
      if (ret < 0)
        {
          snerr("ERROR: Failed to register driver %s: %d\n",
                 devpath_als, ret);

          goto err_out;
        }
    }

  if (devpath_ps != NULL)
    {
      ret = register_driver(devpath_ps, &g_apds9922_psfops, 0666, priv);
      if (ret < 0)
        {
          snerr("ERROR: Failed to register driver %s: %d\n",
                 devpath_ps, ret);
          goto err_out;
        }
    }

  priv->ps_data->close = false;
  priv->ps_data->ps = 0;
  priv->als = 0;
  priv->crefs = 0;

  return OK;

err_out:
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_APDS9922 */
