/****************************************************************************
 * drivers/sensors/tc74ax.c
 * Driver for Microchip TC74Ax I2C thermal sensors
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
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/tc74ax.h>
#include <stdbool.h>
#include <debug.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_TC74AX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Documentation allows frequency range 10-100kHz. Safety check,
 * Kconfig should enforce this.
 */

#  if (CONFIG_SENSORS_TC74AX_I2C_FREQ > 100000) || \
       (CONFIG_SENSORS_TC74AX_I2C_FREQ < 10000)
#    error TX74AX I2C frequency outside of 10kHz-100kHz range
#  endif

#  if !( \
        defined(CONFIG_SENSORS_TC74AX_MULTIMASTER) || \
        defined(CONFIG_SENSORS_TC74AX_POWER_NONE) \
       )
#    define TC74AX_TRACK_STATE
#  endif

#  if defined(TC74AX_TRACK_STATE) || \
       defined(CONFIG_SENSORS_TC74AX_CLOSE_STANDBY)
#    define TC74AX_HAVE_DEVLOCK
#  endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#  ifdef TC74AX_TRACK_STATE

/* If the sensor operates on multimaster bus, there is no point
 * in keeping the internal state - something else can change it.
 *
 * If there is no power management, then there is also no point
 * in keeping the state. (We are not able to change it.)
 */

enum tc74ax_standby_state_e
{
  TC74AX_STANDBY_UNKNOWN,  /* Mode is (1) unknown, (2) our previous
                            * transmission failed or (3) we are starting up.
                            */
  TC74AX_STANDBY_STANDBY,  /* In standby mode, needs to be awakened.
                            * CONFIG register is addressed. */
  TC74AX_STANDBY_WAKING,   /* Was instructed to wake up but it may not be
                            * running yet - needs to be verified.
                            * CONFIG register is addressed. */
  TC74AX_STANDBY_RUNNING   /* In temperature measuring (active) mode,
                            * TEMP register is addressed. */
};

#  endif

struct tc74ax_dev_s
{
  FAR struct i2c_master_s     *i2c; /* I2C interface */
  uint8_t                     addr; /* I2C address */

#  ifdef CONFIG_SENSORS_TC74AX_CLOSE_STANDBY
  uint8_t                     crefs;
#  endif

#  ifdef TC74AX_TRACK_STATE
  enum tc74ax_standby_state_e standby_state;
#  endif

#  ifdef TC74AX_HAVE_DEVLOCK
  mutex_t                     devlock;
#  endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#  ifdef CONFIG_SENSORS_TC74AX_CLOSE_STANDBY
static ssize_t tc74ax_open(FAR struct file *filep);
static ssize_t tc74ax_close(FAR struct file *filep);
#  endif
static ssize_t tc74ax_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t tc74ax_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
#  ifdef CONFIG_SENSORS_TC74AX_POWER_IOCTL
static int tc74ax_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#  endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_tc74ax_fops =
{
#  ifdef CONFIG_SENSORS_TC74AX_CLOSE_STANDBY
  tc74ax_open,
  tc74ax_close,
#  else
  NULL, /* open */
  NULL, /* close */
#  endif
  tc74ax_read,
  tc74ax_write,
  NULL, /* seek */
#  ifdef CONFIG_SENSORS_TC74AX_POWER_IOCTL
  tc74ax_ioctl
#  else
  NULL  /* ioctl */
#  endif
};

/* Command explanation: set addressed register to 1, the CONFIG register.
 * Write value with MSB set, which triggers the standby mode. (Other bits
 * are either unused or read-only.)
 */

static const uint8_t g_tx74ax_cmd_standby[] = "\x1\x80";
static const uint8_t g_tx74ax_cmd_run[] = "\x1\x0";

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc74ax_set_state
 *
 * Description:
 *   Sets internal state of the device (as tracked by the driver)
 *
 * Input Parameters:
 *   priv - pointer to tc74ax_dev_s to operate on.
 *   state - new state to be set
 *
 * Assumptions/Limitations:
 *   Replaced by empty macro if multimaster mode is enabled.
 *   See comment for enum tc74ax_standby_state_e for explanation.
 *
 ****************************************************************************/

#  ifndef TC74AX_TRACK_STATE
#    define tc74ax_set_state(a, b)
#  else

void tc74ax_set_state(struct tc74ax_dev_s *priv,
                      enum tc74ax_standby_state_e new_state)
{
  priv->standby_state = new_state;
}

#  endif

/****************************************************************************
 * Name: tc74ax_i2c_transfer
 *
 * Description:
 *   Perform I2C transfer of given messages and I2C bus. Transition
 *   to unknown state on failure.
 *
 * Input Parameters:
 *
 *   i2c - pointer struct i2c_master_s for target I2C bus
 *   msgs - pointer to array of struct i2c_msg_s with I2C messages
 *   count - message count in msgs
 *
 * Returned Value:
 *
 *   Value returned by I2C_TRANSFER macro. (Ie. by I2C bus driver's
 *   transfer method.)
 *
 ****************************************************************************/

static int tc74ax_i2c_transfer(struct tc74ax_dev_s *priv,
                               struct i2c_msg_s *msgs,
                               int count)
{
  int ret;

  ret = I2C_TRANSFER(priv->i2c, msgs, count);
  if (ret < 0)
    {
      snwarn("TC74Ax I2C transfer failed (%i)\n", ret);
      tc74ax_set_state(priv, TC74AX_STANDBY_UNKNOWN);
    }

  return ret;
}

/****************************************************************************
 * Name: tc74ax_set_standby
 *
 * Description:
 *   Instructs the device to either go to standby mode with reduced power
 *   consumption or to leave that mode and start temperature readings.
 *
 * Input Parameters:
 *   priv - pointer to tc74ax_dev_s to operate on.
 *   standby - requested state, bool, true for standby, false for running
 *
 * Returned Value:
 *   OK or negated errno.
 *
 * Assumptions/Limitations:
 *   Not compiled in if there is power management provided. Note that this
 *   applies to multimaster mode too.
 ****************************************************************************/

#  ifndef CONFIG_SENSORS_TC74AX_POWER_NONE

static int tc74ax_set_standby(struct tc74ax_dev_s *priv,
                              bool standby)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = CONFIG_SENSORS_TC74AX_I2C_FREQ;
  msg.addr = priv->addr;
  msg.flags = 0;
  msg.length = 2;
  if (standby)
    {
      /* Discard const in the assignment - the buffer is read and therefore
       * not modified so we can do that
       */

      msg.buffer = (FAR uint8_t *) g_tx74ax_cmd_standby;
      tc74ax_set_state(priv, TC74AX_STANDBY_STANDBY);
    }
  else
    {
      msg.buffer = (FAR uint8_t *) g_tx74ax_cmd_run;
      tc74ax_set_state(priv, TC74AX_STANDBY_WAKING);
    }

  ret = tc74ax_i2c_transfer(priv, &msg, 1);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

#  endif

/****************************************************************************
 * Name: tc74ax_await_running
 *
 * Description:
 *   Reads CONFIG register from the device until the device
 *   reports that it is ready. As soon as the device becomes
 *   ready, sets its read/write address to TEMP register.
 *
 * Input Parameters:
 *   priv - pointer to tc74ax_dev_s to operate on.
 *
 * Returned Value:
 *   OK or negated errno from I2C bus.
 *
 * Assumptions/Limitations:
 *   Excluded from the build if multimaster mode is enabled. The device may
 *   be reconfigured by the other master after we determine it is running.
 *   (Also see comment for enum tc74ax_standby_state_e for explanation.)
 *
 *   Also excluded from the build if power management is not enabled.
 *   Since we cannot wake the device from standby mode, there is no point
 *   in waiting for it to go into running mode.
 *
 ****************************************************************************/

#  if !( \
        defined(CONFIG_SENSORS_TC74AX_MULTIMASTER) || \
        defined(CONFIG_SENSORS_TC74AX_POWER_NONE) \
       )

static int tc74ax_await_running(struct tc74ax_dev_s *priv)
{
  struct timespec timeout;
  struct timespec ts;
  const struct timespec timeout_duration =
    { .tv_sec = 0,
      .tv_nsec = 500000000UL
    };

  unsigned char data_buf;
  struct i2c_msg_s msg;
  int ret;

  /* Sanity check */

  if (priv->standby_state != TC74AX_STANDBY_WAKING)
    {
      PANIC();
    }

  msg.frequency = CONFIG_SENSORS_TC74AX_I2C_FREQ;
  msg.addr = priv->addr;
  msg.flags = I2C_M_READ;
  msg.length = 1;
  msg.buffer = &data_buf;

  /* Configure timeout into timeout variable. */

  clock_systime_timespec(&timeout);
  clock_timespec_add(&timeout, &timeout_duration, &timeout);

  /* For cycle initializes ts to current time.
   *
   * clock_timespec_compare returns value lesser than zero
   * if first value is lower than the second (current time
   * is lesser than timeout time.)
   *
   * On every pass, ts is updated with current time
   *
   * Every iteration sleeps for 10 milliseconds.
   */

  for (clock_systime_timespec(&ts); \
      clock_timespec_compare(&ts, &timeout) < 0 ; \
      clock_systime_timespec(&ts))
    {
      ret = tc74ax_i2c_transfer(priv, &msg, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* Magic constant - data ready is bit 6 */

      if (data_buf & (1 << 6))
        {
          /* Data ready. Set in-device address to TEMP */

          msg.frequency = CONFIG_SENSORS_TC74AX_I2C_FREQ;
          msg.addr = priv->addr;
          msg.flags = 0;
          msg.length = 1;
          data_buf = 0;
          msg.buffer = &data_buf;

          /* Here we can use the transfer function's return value */

          priv->standby_state = TC74AX_STANDBY_RUNNING;
          return tc74ax_i2c_transfer(priv, &msg, 1);
        }

      nxsched_usleep(10000);
    }

  /* Timed out - return an error */

  tc74ax_set_state(priv, TC74AX_STANDBY_UNKNOWN);
  return -EIO;
}

#  endif

/****************************************************************************
 * Name: tc74ax_open
 *
 * Description:
 *   File operations member - callback on device file open. It is only
 *   used to wake the device up from standby state when the device file
 *   is opened.
 *
 * Input Parameters:
 *   filep - struct file pointer
 *
 * Returned Value:
 *   Returns value from underlying tc74ax_set_standby call:
 *   OK if the device accepted I2C transfer instructing it to transition
 *   to running state, error otherwise
 *
 * Assumptions/Limitations:
 *   If the sensor driver is not configured to keep the device in running
 *   state only when it is opened, the function is not compiled in.
 *
 ****************************************************************************/

#  ifdef CONFIG_SENSORS_TC74AX_CLOSE_STANDBY

static ssize_t tc74ax_open(FAR struct file *filep)
{
  struct tc74ax_dev_s *priv;
  ssize_t ret;

  priv = filep->f_inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->crefs == 0)
    {
      /* First one to open the file, set mode */

      ret = tc74ax_set_standby(priv, false);
      priv->crefs = 1;
      goto tc74ax_open_log;
    }

  if (priv->crefs == 127)
    {
      ret = -EMFILE; /* 8-bit counter */
      goto tc74ax_open_unlock;
    }

  priv->crefs++;
  ret = OK;

tc74ax_open_log:
  sninfo("TC74Ax open refs %i\n", priv->crefs);

tc74ax_open_unlock:
  nxmutex_unlock(&priv->devlock);

  return ret;
}

#endif

/****************************************************************************
 * Name: tc74ax_close
 *
 * Description:
 *   File operations member - callback on device file close. It is only
 *   used to suspend the device to standby state when the device file
 *   is opened.
 *
 * Input Parameters:
 *   filep - struct file pointer
 *
 * Returned Value:
 *   Returns value from underlying tc74ax_set_standby call:
 *   OK if the device accepted I2C transfer instructing it to transition
 *   to running state, error otherwise
 *
 * Assumptions/Limitations:
 *   If the sensor driver is not configured to keep the device in running
 *   state only when it is opened, the function is not compiled in.
 *
 ****************************************************************************/

#  ifdef CONFIG_SENSORS_TC74AX_CLOSE_STANDBY

static ssize_t tc74ax_close(FAR struct file *filep)
{
  struct tc74ax_dev_s *priv;
  ssize_t ret;

  priv = filep->f_inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  priv->crefs--;
  if (priv->crefs == 0)
    {
      ret = tc74ax_set_standby(priv, true);
    }
  else
    {
      ret = OK;
    }

  sninfo("TC74Ax close refs %i\n", priv->crefs);
  nxmutex_unlock(&priv->devlock);

  return ret;
}

#  endif

/****************************************************************************
 * Name: tc74ax_write
 *
 * Description:
 *   Dummy member of file operations structure, device file is not writable
 *   in this driver.
 *
 * Input Parameters:
 *   See tc74ax_read, ignored in this function.
 *
 * Returned Value:
 *   Always -ENOTSUP
 *
 ****************************************************************************/

static ssize_t tc74ax_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: tc74ax_read
 *
 * Description:
 *   File operations member that gets called when the corresponding
 *   device file is read.
 *
 * Input Parameters:
 *   filep - struct file pointer
 *   buffer - buffer that receives read data
 *   buflen - length of the buffer
 *
 * Returned Value:
 *   Returned bytes count (always 1) or negated errno. Value in buffer
 *   is a temperature in degrees Celsius (signed 8-bit integer.)
 *
 ****************************************************************************/

static ssize_t tc74ax_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  struct tc74ax_dev_s *priv;   /* Device description struct */
  int ret;                     /* Return value */
  unsigned char data_buf;      /* Various buffers for I2C messages */

#  ifdef CONFIG_SENSORS_TC74AX_MULTIMASTER
  unsigned char cmd_buf;
  unsigned char cmd_buf2;
  unsigned char data_buf2;
#  endif

  priv = filep->f_inode->i_private;

  /* If we are tracking internal state, we also need to protect it
   * from race conditions.
   */

#  ifdef TC74AX_TRACK_STATE
  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }
#endif

#  ifdef CONFIG_SENSORS_TC74AX_MULTIMASTER

  /* Multimaster mode needs 4 I2C messages */

  struct i2c_msg_s msg[4];

  /* Timespecs for timeout in multimaster mode. Since we may need
   * to wake the device up and then wait for first conversion
   * to be ready, the timeout is somewhat high.
   */

  struct timespec timeout;
  struct timespec ts;
  const struct timespec timeout_duration =
    { .tv_sec = 2,
      .tv_nsec = 500000000UL
    };

#  else

  /* Non-multimaster mode only needs 1 message */

  struct i2c_msg_s msg;

#  endif

  sninfo("TC74Ax attempt read\n");
  if ((buffer == NULL) || (!buflen))
    {
      sninfo("TC74Ax provided null buffer or zero buflen\n");
      ret = -EINVAL;
      goto tc74ax_read_unlock_out;
    }

#  ifdef CONFIG_SENSORS_TC74AX_MULTIMASTER

  /* This case is complicated because we have to assume that the other
   * master can interfere with us any time we release the I2C bus. This
   * has implications:
   *
   * In theory, it is possible that we will never be able to complete
   * the read even if the device works properly - the other master may
   * keep setting it to standby mode. Arbitrary timeout of 500ms is
   * imposed to account for that.
   *
   * Any state we read from the device needs to be considered incorrect
   * as soon as we release the bus. It is impossible to verify the device
   * is in normal operating mode and then read temperature (when we finish
   * reading the mode and evaluate returned value, I2C bus is released
   * and the other master may put the device on standby.) We need
   * to perform all interaction with the device within a single
   * I2C transaction.
   */

  clock_systime_timespec(&timeout);
  clock_timespec_add(&timeout, &timeout_duration, &timeout);

  /* For cycle initializes ts to current time.
   *
   * clock_timespec_compare returns value lesser than zero
   * if first value is lower than the second (current time
   * is lesser than timeout time.)
   *
   * On every pass, the ts is updated with current time
   *
   * Every iteration sleeps for 10 milliseconds.
   */

  /* Prepare I2C messages for the cycle so we don't need to redo
   * it on every iteration.
   *
   * First message - set addressed register in the device to CONFIG
   */

  msg[0].frequency = CONFIG_SENSORS_TC74AX_I2C_FREQ;
  msg[0].addr = priv->addr;
  msg[0].flags = I2C_M_NOSTOP;
  msg[0].length = 1;
  cmd_buf = 1;
  msg[0].buffer = &cmd_buf;

  /* Second message - read it */

  msg[1].frequency = CONFIG_SENSORS_TC74AX_I2C_FREQ;
  msg[1].addr = priv->addr;
  msg[1].flags = I2C_M_NOSTOP | I2C_M_READ;
  msg[1].length = 1;
  msg[1].buffer = &data_buf;

  /* Then address TEMP */

  msg[2].frequency = CONFIG_SENSORS_TC74AX_I2C_FREQ;
  msg[2].addr = priv->addr;
  msg[2].flags = I2C_M_NOSTOP;
  msg[2].length = 1;
  cmd_buf2 = 0;
  msg[2].buffer = &cmd_buf2;

  /* And read from it */

  msg[3].frequency = CONFIG_SENSORS_TC74AX_I2C_FREQ;
  msg[3].addr = priv->addr;
  msg[3].flags = I2C_M_READ;
  msg[3].length = 1;
  msg[3].buffer = &data_buf2;

  for (clock_systime_timespec(&ts); \
      clock_timespec_compare(&ts, &timeout) < 0 ; \
      clock_systime_timespec(&ts))
    {
      /* Transmit prepared messages */

      ret = tc74ax_i2c_transfer(priv, msg, 4);
      if (ret < 0)
        {
          goto tc74ax_read_unlock_out;
        }

      /* Magic constant
       * - data ready is bit 6 (set for data ready)
       * - operating mode is bit 7 (set for standby)
       */

      if (data_buf & ((1 << 6) | (1 << 7)))
        {
          /* Device in normal operating mode, data ready */

          buffer[0] = data_buf2;

          /* Break from the timeout for cycle by returning (reaching
           * the end of the cycle is considered to be an error - timeout.)
           */

          ret = 1;
          goto tc74ax_read_unlock_out;
        }

      if (data_buf & (1 << 7))
        {
#  ifndef CONFIG_SENSORS_TC74AX_POWER_NONE
          /* Device is set to standby mode, we need to wake it up. */

          ret = tc74ax_set_standby(priv, false);
          if (ret < 0)
            {
              goto tc74ax_read_unlock_out;
            }

          /* Need to wait for t_conv time. The datasheet does not say
           * how much time that may be. Let's assume "Maximum ensured
           * conversion time after Power-on Reset (POR to DATA_RDY)
           * is 250 msec." is applicable here.
           */

          nxsched_usleep(250000);

          /* Now fall through to the next iteration - or timeout */
#  else
          /* Device is set to standby mode but we are unable to wake it up
           * because power management support is disabled in configuration.
           * Fail without waiting.
           */

          ret = -EIO;
          goto tc74ax_read_unlock_out;
#  endif
        }
      else
        {
          /* Device is set to be in normal operating mode but does
           * not have data yet. It is supposed to provide new temperature
           * 8 times a second so let's pick something along those lines
           * (but keeping latency in check too.)
           */

          nxsched_usleep(31250);
        }
    }

  /* End of for cycle - read attempt timed out */

  ret = -EIO;
  goto tc74ax_read_unlock_out;

#  else /* for ifdef CONFIG_SENSORS_TC74AX_MULTIMASTER */

#    ifndef CONFIG_SENSORS_TC74AX_POWER_NONE

  /* Some form of power management is configured. We track internal state
   * of the device and therefore know if it needs to be awakened because
   * there is no other MCU accessing it.
   */

  priv = filep->f_inode->i_private;
  if ((priv->standby_state == TC74AX_STANDBY_UNKNOWN) || \
      (priv->standby_state == TC74AX_STANDBY_STANDBY))
    {
      /* Do not try to use the device in unknown state (that is,
       * after it had an error - reset it first.) Also do not try
       * to use it in standby state. You are supposed to wake it
       * up first or have the kernel do it on open.
       */

      snwarn("TC74Ax attempted to read sensor in not running state\n");

      ret = -EIO;
      goto tc74ax_read_unlock_out;
    }

  /* Do not test for waking state and do not wait. With no power
   * management, we cannot be in waking state
   */

  if (priv->standby_state == TC74AX_STANDBY_WAKING)
    {
      /* Device was instructed to wake up but it is not
       * confirmed if it already did. In this state, CONFIG
       * register is addressed.
       *
       * This function sets TC74AX_STANDBY_UNKNOWN state
       * if it fails to verify that the device woke up.
       * (No need to do it here again.)
       */

      sninfo("TC74Ax in waking state, await running\n");
      ret = tc74ax_await_running(priv);
      if (ret < 0)
        {
          snwarn("TC74Ax failed waiting for waking " \
                 "to running switch (%i)\n", \
                 ret);
          goto tc74ax_read_unlock_out;
        }

      /* The device is now in running state. Its TEMP register
       * is addressed.
       */
    }

#    endif

  /* Device is running or we do not track its internal state because
   * power management support was not built in. Read the temperature.
   */

  msg.frequency = CONFIG_SENSORS_TC74AX_I2C_FREQ;
  msg.addr = priv->addr;
  msg.flags = I2C_M_READ;
  msg.length = 1;
  msg.buffer = &data_buf;

  ret = tc74ax_i2c_transfer(priv, &msg, 1);
  if (ret < 0)
    {
      snwarn("TC74Ax temperature read failed (%i)\n", ret);
      goto tc74ax_read_unlock_out;
    }

  sinfo("TC74Ax temperature read complete\n");
  buffer[0] = data_buf;

#  endif /* ifdef CONFIG_SENSORS_TC74AX_MULTIMASTER */

  /* If we got here, read succeeded. We are returning one byte of data. */

  ret = 1;

tc74ax_read_unlock_out:

#  ifdef TC74AX_TRACK_STATE
  nxmutex_unlock(&priv->devlock);
#  endif

  return ret;
}

/****************************************************************************
 * Name: tc74ax_ioctl
 *
 * Description:
 *   The ioctl method
 *
 * Input Parameters:
 *   filep - struct file pointer
 *   cmd - ioctl command
 *   arg - requested mode (running or standby)
 *
 * Returned Value:
 *   Propagates error from underlying I2C transfer, EINVAL
 *   for invalid input, OK otherwise
 *
 * Assumptions/Limitations:
 *   Not compiled in unless the Kconfig requests it.
 *
 ****************************************************************************/

#  ifdef CONFIG_SENSORS_TC74AX_POWER_IOCTL

static int tc74ax_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  struct tc74ax_dev_s *priv;
  int ret;

  priv = filep->f_inode->i_private;
  ret = -EINVAL;

#    ifdef TC74AX_TRACK_STATE
  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }
#    endif

  if (cmd != SNIOC_SET_OPERATIONAL_MODE)
    {
      goto tc74ax_ioctl_unlock_out; /* ret is set to -EINVAL by default */
    }

  switch ((tc74ax_operation_mode_e) arg)
    {
      case TC74AX_OPERATION_MODE_OPERATING:

#    ifndef CONFIG_SENSORS_TC74AX_MULTIMASTER
        /* In non-multimaster mode, we track current state of the device.
         * If it's already on, we are done. Same is true if it is already
         * waking up. This is also the way to reset unknown state after
         * an error.
         *
         * (If the state is unknown, we need to assume the device is
         * in standby mode.)
         */

        if ((priv->standby_state == TC74AX_STANDBY_RUNNING) || \
            (priv->standby_state == TC74AX_STANDBY_WAKING))
          {
            ret = OK;
            goto tc74ax_ioctl_unlock_out;
          }

#    endif

        /* Multimaster mode or device in standby mode, wake it up */

        ret = tc74ax_set_standby(priv, false);
        goto tc74ax_ioctl_unlock_out;

        break;

      case TC74AX_OPERATION_MODE_STANDBY:

#    ifndef CONFIG_SENSORS_TC74AX_MULTIMASTER
        /* Again, check current state of the device first (see above.)
         * Also a way to reset unknown state after an error.
         */

        if (priv->standby_state == TC74AX_STANDBY_STANDBY)
          {
            ret = OK;
            goto tc74ax_ioctl_unlock_out;
          }

        /* Standby mode can be requested even if we didn't reach running
         * state. Differs in which register is addressed in the device
         * but that is not a problem because attempt to change the mode
         * will set the address in either case
         */

#    endif

        ret = tc74ax_set_standby(priv, true);
        goto tc74ax_ioctl_unlock_out;

        break;

      default:
        goto tc74ax_ioctl_unlock_out; /* EINVAL default */
    }

tc74ax_ioctl_unlock_out:
#    ifdef TC74AX_TRACK_STATE
  nxmutex_unlock(&priv->devlock);
#    endif

  return ret;
}

#  endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc74ax_register
 *
 * Description:
 *   Registers TC74Ax temperature sensor at specified pathname.
 *
 * Input Parameters:
 *   devpath - full pathname where the driver will be registered
 *             (example: /dev/therm0)
 *   i2c     - instance of I2C driver controlling the bus this sensor
 *             is attached to.
 *   addr    - I2C address of the sensor. Corresponds to the chip name
 *             with x replaced by a digit in range of 0-7. Minimum
 *             is 72 for TC74A0, maximum is 79 for TC74A7.
 *
 * Returned Value:
 *   OK on success or negated errno on failure.
 *
 ****************************************************************************/

int tc74ax_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr)
{
  int ret;
  FAR struct tc74ax_dev_s *priv;

  /* Sanity checks */

  if (!i2c)
    {
      return -EINVAL;
    }

  if ((addr < 72) || (addr > 79))
    {
      return -EINVAL;
    }

  /* Allocate instance variable struct, assign its values */

  priv = \
    (FAR struct tc74ax_dev_s *) kmm_malloc(sizeof(struct tc74ax_dev_s));

  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;

  /* The sensor (probably) resets into running state.
   * Nevertheless, let's make sure to put it into
   * the state the configuration wants it in.
   *
   * Initialize state to unknown (tc74ax_set_standby will fix that.)
   * Exception - if there is no power management, then initialize
   * to running.
   */

#  ifdef TC74AX_TRACK_STATE
  tc74ax_set_state(priv, TC74AX_STANDBY_UNKNOWN);
#  endif
#  ifdef TC74AX_HAVE_DEVLOCK
  nxmutex_init(&(priv->devlock));
#  endif
#  ifdef CONFIG_SENSORS_TC74AX_CLOSE_STANDBY
  priv->crefs = 0;
#  endif

  /* Register the driver */

  sninfo("Registering TC74Ax driver at %s\n", devpath);
  ret = register_driver(devpath, &g_tc74ax_fops, 0666, priv);
  if (ret < 0)
    {
#  ifdef TC74AX_HAVE_DEVLOCK
      nxmutex_destroy(&(priv->devlock));
#  endif
      kmm_free(priv);
      return ret;
    }

  /* Initialization. Done after registration and may fail.
   * That's for the application to deal with (the sensor may come
   * online later for example.)
   */

#  if defined(CONFIG_SENSORS_TC74AX_RESET_STANDBY) \
      || defined(CONFIG_SENSORS_TC74AX_CLOSE_STANDBY)
  tc74ax_set_standby(priv, true);
#  else
#    ifndef CONFIG_SENSORS_TC74AX_POWER_NONE
  tc74ax_set_standby(priv, false);
#    endif /* not CONFIG_SENSORS_TC74AX_POWER_NONE */
#  endif

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_TC74AX */
