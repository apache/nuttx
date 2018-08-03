/****************************************************************************
 * drivers/sensors/lis2dh.c
 * LIS2DH accelerometer driver
 *
 *   Copyright (C) 2014-2017 Haltian Ltd. All rights reserved.
 *   Authors: Timo Voutilainen <timo.voutilainen@haltian.com>
 *            Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *            Juha Niskanen <juha.niskanen@haltian.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/sensors/lis2dh.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIS2DH_DEBUG
#  define lis2dh_dbg(x, ...)        _info(x, ##__VA_ARGS__)
#else
#  define lis2dh_dbg(x, ...)        sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_LIS2DH_I2C_FREQUENCY
#  define CONFIG_LIS2DH_I2C_FREQUENCY   400000
#endif

#ifdef CONFIG_LIS2DH_DRIVER_SELFTEST
#  define LSB_AT_10BIT_RESOLUTION       4
#  define LSB_AT_12BIT_RESOLUTION       1
#  define SELFTEST_BUF_SIZE             5
#  define SELFTEST_MAX_READ_ATTEMPTS    200
#  define SELFTEST_ABS_DIFF_MIN_10BIT   17
#  define SELFTEST_ABS_DIFF_MAX_10_BIT  360
#  define SELFTEST_ABS_DIFF_MIN_12BIT   (LSB_AT_10BIT_RESOLUTION * SELFTEST_ABS_DIFF_MIN_10BIT)
#  define SELFTEST_ABS_DIFF_MAX_12BIT   (LSB_AT_10BIT_RESOLUTION * SELFTEST_ABS_DIFF_MAX_10_BIT)
#  define SELFTEST_0                    0
#  define SELFTEST_1                    1
#endif

/* Miscellaneous macros */

#define LIS2DH_I2C_RETRIES  10
#define LIS2DH_COUNT_INTS

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

enum interrupts
{
  LIS2DH_INT1 = 1,
  LIS2DH_INT2 = 2
};

struct lis2dh_dev_s
{
  FAR struct i2c_master_s    *i2c;        /* I2C interface */
  uint8_t                    addr;        /* I2C address */
  FAR struct lis2dh_config_s *config;     /* Platform specific configuration */
  struct lis2dh_setup        *setup;      /* User defined device operation mode setup */
  struct lis2dh_vector_s     vector_data; /* Latest read data read from lis2dh */
  int                        scale;       /* Full scale in milliG */
  sem_t                      devsem;      /* Manages exclusive access to this structure */
  bool                       fifo_used;   /* LIS2DH configured to use FIFO */
  bool                       fifo_stopped;/* FIFO got full and has stopped. */
#ifdef LIS2DH_COUNT_INTS
  volatile int16_t           int_pending; /* Interrupt received but data not read, yet */
#else
  volatile bool              int_pending; /* Interrupt received but data not read, yet */
#endif
#ifndef CONFIG_DISABLE_POLL
  struct pollfd              *fds[CONFIG_LIS2DH_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static int            lis2dh_open(FAR struct file *filep);
static int            lis2dh_close(FAR struct file *filep);
static ssize_t        lis2dh_read(FAR struct file *, FAR char *, size_t);
static ssize_t        lis2dh_write(FAR struct file *filep,
                        FAR const char *buffer, size_t buflen);
static int            lis2dh_ioctl(FAR struct file *filep, int cmd,
                        unsigned long arg);
static int            lis2dh_access(FAR struct lis2dh_dev_s *dev,
                        uint8_t subaddr, FAR uint8_t *buf, int length);
static int            lis2dh_get_reading(FAR struct lis2dh_dev_s *dev,
                        FAR struct lis2dh_vector_s *res, bool force_read);
static int            lis2dh_powerdown(FAR struct lis2dh_dev_s *dev);
static int            lis2dh_reboot(FAR struct lis2dh_dev_s *dev);
#ifndef CONFIG_DISABLE_POLL
static int            lis2dh_poll(FAR struct file *filep,
                        FAR struct pollfd *fds, bool setup);
static void           lis2dh_notify(FAR struct lis2dh_dev_s *priv);
#endif
static int            lis2dh_int_handler(int irq, FAR void *context,
                        FAR void *arg);
static int            lis2dh_setup(FAR struct lis2dh_dev_s *dev,
                        FAR struct lis2dh_setup *new_setup);
static inline int16_t lis2dh_raw_to_mg(uint8_t raw_hibyte,
                        uint8_t raw_lobyte, int scale);
static int            lis2dh_read_temp(FAR struct lis2dh_dev_s *dev,
                        FAR int16_t *temper);
static int            lis2dh_clear_interrupts(FAR struct lis2dh_dev_s *priv,
                        uint8_t interrupts);
static unsigned int   lis2dh_get_fifo_readings(FAR struct lis2dh_dev_s *priv,
                        FAR struct lis2dh_result *res, unsigned int readcount,
                        FAR int *perr);
#ifdef CONFIG_LIS2DH_DRIVER_SELFTEST
static int            lis2dh_handle_selftest(FAR struct lis2dh_dev_s *priv);
static int16_t        lis2dh_raw_convert_to_12bit(uint8_t raw_hibyte,
                        uint8_t raw_lobyte);
static FAR const struct lis2dh_vector_s *
                       lis2dh_get_raw_readings(FAR struct lis2dh_dev_s *dev,
                        FAR int *err);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lis2dhops =
{
  lis2dh_open,   /* open */
  lis2dh_close,  /* close */
  lis2dh_read,   /* read */
  lis2dh_write,  /* write */
  NULL,          /* seek */
  lis2dh_ioctl   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , lis2dh_poll  /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lis2dh_who_am_i(FAR struct lis2dh_dev_s *dev, uint8_t *id)
{
  int ret;

  ret = lis2dh_access(dev, ST_LIS2DH_WHOAMI_REG, id, 1);
  if (ret < 0)
    {
      lis2dh_dbg("Cannot read who am i value\n");
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: lis2dh_open
 *
 * Description:
 *   This function is called whenever the LIS2DH device is opened.
 *
 ****************************************************************************/

static int lis2dh_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct lis2dh_dev_s *priv  = inode->i_private;
  uint8_t regval;
  int ret = OK;

  /* Probe device */

  if (lis2dh_access(priv, ST_LIS2DH_WHOAMI_REG, &regval, 1) > 0)
    {
      /* Check chip identification, in the future several more compatible parts
       * may be added here.
       */

      if (regval == ST_LIS2DH_WHOAMI_VALUE)
        {
          priv->config->irq_enable(priv->config, true);
          /* Normal exit point */
          ret = lis2dh_clear_interrupts(priv, LIS2DH_INT1 | LIS2DH_INT2);
          return ret;
        }

      /* Otherwise, we mark an invalid device found at given address */

      ret = -ENODEV;
    }
  else
    {
      /* No response at given address is marked as */

      ret = -EFAULT;
    }

  /* Error exit */

  return ret;
}

/****************************************************************************
 * Name: lis2dh_close
 *
 * Description:
 *   This routine is called when the LIS2DH device is closed.
 *
 ****************************************************************************/

static int lis2dh_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct lis2dh_dev_s *priv  = inode->i_private;

  priv->config->irq_enable(priv->config, false);
  return lis2dh_powerdown(priv);
}

/****************************************************************************
 * Name: lis2dh_fifo_start
 *
 * Description:
 *   This function restarts FIFO reading.
 *
 ****************************************************************************/

static int lis2dh_fifo_start(FAR struct lis2dh_dev_s *priv)
{
  uint8_t buf;
  int ret = OK;

  buf =  0x00 | priv->setup->trigger_selection |
      priv->setup->fifo_trigger_threshold;
  if (lis2dh_access(priv, ST_LIS2DH_FIFO_CTRL_REG, &buf, -1) != 1)
    {
      lis2dh_dbg("lis2dh: Failed to write FIFO control register\n");
      ret = -EIO;
    }
  else
    {
      buf =  priv->setup->fifo_mode | priv->setup->trigger_selection |
          priv->setup->fifo_trigger_threshold;
      if (lis2dh_access(priv, ST_LIS2DH_FIFO_CTRL_REG, &buf, -1) != 1)
        {
          lis2dh_dbg("lis2dh: Failed to write FIFO control register\n");
          ret = -EIO;
        }
      else
        {
          priv->fifo_stopped = false;

          lis2dh_dbg("lis2dh: FIFO restarted\n");
        }
    }

  return ret;
}

/****************************************************************************
 * Name: lis2dh_read
 *
 * Description:
 *   This routine is called when the LIS2DH device is read.
 *
 ****************************************************************************/

static ssize_t lis2dh_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode              *inode = filep->f_inode;
  FAR struct lis2dh_dev_s       *priv  = inode->i_private;
  FAR struct lis2dh_result      *ptr;
  int readcount = (buflen - sizeof(struct lis2dh_res_header)) / sizeof(struct lis2dh_vector_s);
  uint8_t buf;
  uint8_t int1_src = 0;
  uint8_t int2_src = 0;
  irqstate_t flags;
  int ret;

  if (buflen <  sizeof(struct lis2dh_result) ||
      (buflen - sizeof(struct lis2dh_res_header)) % sizeof(struct lis2dh_vector_s) != 0)
    {
      lis2dh_dbg("lis2dh: Illegal amount of bytes to read: %d\n", buflen);
      return -EINVAL;
    }

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Do not allow read() if no SNIOC_WRITESETUP first. */

  if (!priv->setup)
    {
      lis2dh_dbg("lis2dh: Read from unconfigured device\n");
      return -EINVAL;
    }

  flags = enter_critical_section();
#ifdef LIS2DH_COUNT_INTS
  if (priv->int_pending > 0)
    {
      priv->int_pending--;
    }

  DEBUGASSERT(priv->int_pending >= 0 && priv->int_pending < 10);
#else
  priv->int_pending = false;
#endif
  leave_critical_section(flags);

  /* Set pointer to first measurement data */

  ptr = (FAR struct lis2dh_result *)buffer;
  ptr->header.meas_count = 0;

  if (!priv->fifo_used)
    {
      /* FIFO not used, read only one sample. */

      if (readcount > 0)
        {
          ret = lis2dh_get_reading(priv, &ptr->measurements[0], true);
          if (ret < 0)
            {
              lis2dh_dbg("lis2dh: Failed to read xyz\n");
            }
          else
            {
              ptr->header.meas_count = 1;
            }
        }
    }
  else /* FIFO modes */
    {
      uint8_t fifo_mode = priv->setup->fifo_mode & ST_LIS2DH_FIFOCR_MODE_MASK;
      bool fifo_empty = false;
      uint8_t fifo_num_samples;

      ptr->header.meas_count = 0;

      do
        {
          /* Check if FIFO needs to be restarted after being read empty.
           * We need to read SRC_REG before reading measurement, as reading
           * sample from FIFO clears OVRN_FIFO flag.
           */

          if (lis2dh_access(priv, ST_LIS2DH_FIFO_SRC_REG, &buf, 1) != 1)
            {
              lis2dh_dbg("lis2dh: Failed to read FIFO source register\n");
              return -EIO;
            }

          if (fifo_mode != LIS2DH_STREAM_MODE)
            {
              /* FIFO is full and has stopped. */

              priv->fifo_stopped |= !!(buf & ST_LIS2DH_FIFOSR_OVRN_FIFO);
            }

          if (buf & ST_LIS2DH_FIFOSR_OVRN_FIFO)
            {
              lis2dh_dbg("lis2dh: FIFO overrun\n");
            }

          if (buf & ST_LIS2DH_FIFOSR_EMPTY)
            {
              lis2dh_dbg("lis2dh: FIFO empty\n");

              fifo_empty = true;

              if (fifo_mode != LIS2DH_STREAM_MODE)
                {
                  priv->fifo_stopped = true;
                }

              /* FIFO is empty, skip reading. */

              break;
            }

          /* How many samples available in FIFO? */

          fifo_num_samples = (buf & ST_LIS2DH_FIFOSR_NUM_SAMP_MASK) + 1;

          if (fifo_num_samples > (readcount - ptr->header.meas_count))
            {
              fifo_num_samples = (readcount - ptr->header.meas_count);
            }

          ptr->header.meas_count +=
              lis2dh_get_fifo_readings(priv, ptr, fifo_num_samples, &ret);
        }
      while (!fifo_empty && ptr->header.meas_count < readcount);

      if (!fifo_empty && fifo_mode != LIS2DH_TRIGGER_MODE)
        {
          /* FIFO was not read empty, more data available. */

          flags = enter_critical_section();

#ifdef LIS2DH_COUNT_INTS
          priv->int_pending++;
#else
          priv->int_pending = true;
#endif

#ifndef CONFIG_DISABLE_POLL
          lis2dh_notify(priv);
#endif

          leave_critical_section(flags);
        }
      else if (fifo_mode != LIS2DH_STREAM_MODE && priv->fifo_stopped)
        {
          /* FIFO is empty and has stopped by overrun event. Reset FIFO for
           * further reading.
           */

          ret = lis2dh_fifo_start(priv);
        }
    }

  /* Make sure interrupt will get cleared (by reading this register) in case of
   * latched configuration.
   */

  buf = 0;
  if (lis2dh_access(priv, ST_LIS2DH_INT1_SRC_REG, &buf, 1) != 1)
    {
      lis2dh_dbg("lis2dh: Failed to read INT1_SRC_REG\n");
      ret = -EIO;
    }
  if (buf & ST_LIS2DH_INT_SR_ACTIVE)
    {
      /* Interrupt has happened */

      int1_src = buf;
      ptr->header.int1_occurred = true;
    }
  else
    {
      ptr->header.int1_occurred = false;
    }

  /* Make sure interrupt will get cleared (by reading this register) in case of
   * latched configuration.
   */

  buf = 0;
  if (lis2dh_access(priv, ST_LIS2DH_INT2_SRC_REG, &buf, 1) != 1)
    {
      lis2dh_dbg("lis2dh: Failed to read INT2_SRC_REG\n");
      ret = -EIO;
    }
  if (buf & ST_LIS2DH_INT_SR_ACTIVE)
    {
      /* Interrupt has happened */

      int2_src = buf;
      ptr->header.int2_occurred = true;
    }
  else
    {
      ptr->header.int2_occurred = false;
    }
  ptr->header.int1_source = int1_src;
  ptr->header.int2_source = int2_src;

  nxsem_post(&priv->devsem);

  /* 'ret' was just for debugging, we do return partial reads here. */

  return sizeof(ptr->header) +
      ptr->header.meas_count * sizeof(struct lis2dh_vector_s);
}

/****************************************************************************
 * Name: lis2dh_write
 * Description:
 *   This routine is called when the LIS2DH device is written to.
 ****************************************************************************/

static ssize_t lis2dh_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  DEBUGASSERT(filep != NULL && buffer != NULL && buflen > 0);

  return -ENOSYS;
}

/****************************************************************************
 * Name: lis2dh_ioctl
 *
 * Description:
 *   This routine is called when ioctl function call
 *   for the LIS2DH device is done.
 *
 ****************************************************************************/

static int lis2dh_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct lis2dh_dev_s *priv;
  int ret;
  uint8_t buf;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct lis2dh_dev_s *)inode->i_private;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
  {
  case SNIOC_WRITESETUP:
    {
      /* Write to the configuration registers. */

      ret = lis2dh_setup(priv, (struct lis2dh_setup *)arg);
      lis2dh_dbg("lis2dh: conf: %p ret: %d\n", (struct lis2dh_setup *)arg, ret);

      /* Make sure interrupt will get cleared in
       * case of latched configuration.
       */

      lis2dh_clear_interrupts(priv, LIS2DH_INT1 | LIS2DH_INT2);
    }
    break;

  case SNIOC_WRITE_INT1THRESHOLD:
    {
      buf = (uint8_t)arg;

      if (lis2dh_access(priv, ST_LIS2DH_INT1_THS_REG, &buf, -1) != 1)
        {
          lis2dh_dbg("lis2dh: Failed to write INT1_THS_REG\n");
          ret = -EIO;
        }

      lis2dh_clear_interrupts(priv, LIS2DH_INT1);
    }
    break;

  case SNIOC_WRITE_INT2THRESHOLD:
    {
      buf = (uint8_t)arg;

      if (lis2dh_access(priv, ST_LIS2DH_INT2_THS_REG, &buf, -1) != 1)
        {
          lis2dh_dbg("lis2dh: Failed to write INT2_THS_REG\n");
          ret = -EIO;
        }

      lis2dh_clear_interrupts(priv, LIS2DH_INT2);
    }
    break;

  case SNIOC_RESET_HPFILTER:
    {
      /* Read reference register to reset/recalib DC offset for HP filter */

      if (lis2dh_access(priv, ST_LIS2DH_REFERENCE_REG, &buf, 1) != 1)
        {
          lis2dh_dbg("lis2dh: Failed to write reference register\n");
          ret = -EIO;
        }

      lis2dh_clear_interrupts(priv, LIS2DH_INT2);
    }
    break;

  case SNIOC_START_SELFTEST:
#ifdef CONFIG_LIS2DH_DRIVER_SELFTEST
    {
      priv->config->irq_enable(priv->config, false);
      lis2dh_clear_interrupts(priv, LIS2DH_INT1 | LIS2DH_INT2);
      ret = lis2dh_handle_selftest(priv);
      priv->config->irq_enable(priv->config, true);
    }
#else
    {
      ret = -EINVAL;
    }
#endif
    break;

  case SNIOC_READ_TEMP:
    {
      ret = lis2dh_read_temp(priv, (int16_t *)arg);
    }
    break;

  case SNIOC_WHO_AM_I:
    {
      ret = lis2dh_who_am_i(priv, (uint8_t *)arg);
    }
    break;

  default:
    {
      lis2dh_dbg("lis2dh: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
    }
    break;
  }

  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: lis2dh_poll
 *
 * Description:
 *   This routine is called during LIS2DH device poll
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int lis2dh_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode *inode;
  FAR struct lis2dh_dev_s *priv;
  int ret;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct lis2dh_dev_s *)inode->i_private;

  ret = nxsem_wait(&priv->devsem);
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

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_LIS2DH_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_LIS2DH_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }

      if (priv->int_pending)
        {
          lis2dh_notify(priv);
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
  nxsem_post(&priv->devsem);
  return ret;
}

static void lis2dh_notify(FAR struct lis2dh_dev_s *priv)
{
  DEBUGASSERT(priv != NULL);

  int i;

  /* If there are threads waiting on poll() for LIS2DH data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_LIS2DH_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          lis2dh_dbg("lis2dh: Report events: %02x\n", fds->revents);
          nxsem_post(fds->sem);
        }
    }
}
#endif /* !CONFIG_DISABLE_POLL */

/****************************************************************************
 * Name: lis2dh_callback
 *
 * Description:
 *   lis2dh interrupt handler
 *
 ****************************************************************************/

static int lis2dh_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lis2dh_dev_s *priv = (FAR struct lis2dh_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();

#ifdef LIS2DH_COUNT_INTS
  priv->int_pending++;
#else
  priv->int_pending = true;
#endif

#ifndef CONFIG_DISABLE_POLL
  lis2dh_notify(priv);
#endif

  leave_critical_section(flags);

  return OK;
}

#ifdef CONFIG_LIS2DH_DRIVER_SELFTEST
/****************************************************************************
 * Name: lis2dh_clear_registers
 *
 * Description:
 *   Clear lis2dh registers
 *
 * Input Parameters:
 *   priv   - pointer to LIS2DH Private Structure
 *
 * Returned Value:
 *   Returns OK in case of success, otherwise ERROR
 *
 ****************************************************************************/

static int lis2dh_clear_registers(FAR struct lis2dh_dev_s *priv)
{
  uint8_t i, buf = 0;

  DEBUGASSERT(priv);

  for (i = ST_LIS2DH_TEMP_CFG_REG; i <= ST_LIS2DH_ACT_DUR_REG; i++)
    {
      /* Skip read only registers */

      if ((i <= 0x1e) || (i >= 0x27 && i <= 0x2d) || (i == 0x2f) || (i == 0x31))
        {
          continue;
        }

      if (lis2dh_access(priv, i, &buf, -1) != 1)
        {
          lis2dh_dbg("lis2dh: Failed to clear register 0x%02x\n", i);
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: lis2dh_write_register
 *
 * Description:
 *   Clear lis2dh registers
 *
 * Input Parameters:
 *   priv   - pointer to LIS2DH Private Structure
 *   reg    - target register
 *   value  - value to write
 *
 * Returned Value:
 *   Returns OK in case of success, otherwise ERROR
 *
 ****************************************************************************/

static int lis2dh_write_register(FAR struct lis2dh_dev_s *priv, uint8_t reg,
                                 uint8_t value)
{
  DEBUGASSERT(priv);

  if (lis2dh_access(priv, reg, &value, -1) != 1)
    {
      lis2dh_dbg("lis2dh: Failed to write %d to register 0x%02x\n",
                 value, reg);
      return ERROR;
    }
  return OK;
}

/****************************************************************************
 * Name: lis2dh_read_register
 *
 * Description:
 *   read lis2dh register
 *
 * Input Parameters:
 *   priv   - pointer to LIS2DH Private Structure
 *   reg    - register to read
 *
 * Returned Value:
 *   Returns positive register value in case of success, otherwise ERROR ( < 0)
 ****************************************************************************/

static int lis2dh_read_register(FAR struct lis2dh_dev_s *priv, uint8_t reg)
{
  uint8_t buf;

  DEBUGASSERT(priv);

  if (lis2dh_access(priv, reg, &buf, sizeof(buf)) == sizeof(buf))
    {
      return buf;
    }

  return ERROR;
}

/****************************************************************************
 * Name: lis2dh_handle_selftest
 *
 * Description:
 *   Handle selftest. Note, that after running selftest lis2dh is left in
 *   shutdown mode without valid setup. Therefore SNIOC_WRITESETUP must be
 *   sent again to proceed with normal operations.
 *
 ****************************************************************************/

static int lis2dh_handle_selftest(FAR struct lis2dh_dev_s *priv)
{
  const struct lis2dh_vector_s *results;
  uint8_t i;
  uint8_t j;
  uint8_t buf;
  int16_t avg_x_no_st = 0;
  int16_t avg_y_no_st = 0;
  int16_t avg_z_no_st = 0;
  int16_t avg_x_with_st = 0;
  int16_t avg_y_with_st = 0;
  int16_t avg_z_with_st = 0;
  int16_t abs_st_x_value;
  int16_t abs_st_y_value;
  int16_t abs_st_z_value;
  int ret = OK;
  int err = OK;

  DEBUGASSERT(priv);

  lis2dh_powerdown(priv);

  if (lis2dh_clear_registers(priv) != OK)
    {
      ret = -EIO;
      goto out;
    }

  /* Set the control register (23h) to ±2g FS, normal mode with BDU (Block
   * Data Update) and HR (High Resolution) bits enabled.
   */

  if (lis2dh_write_register(priv, ST_LIS2DH_CTRL_REG4, 0x88) != OK)
    {
      lis2dh_dbg("lis2dh: Failed to write CTRL4 REG for selftest\n");
      ret = -EIO;
      goto out;
    }

  /* Set the control register (20h) to 50Hz ODR (Output Data Rate) with
   * X/Y/Z axis enabled.
   */

  if (lis2dh_write_register(priv, ST_LIS2DH_CTRL_REG1, 0x47) != OK)
    {
      lis2dh_dbg("lis2dh: Failed to write CTRL1 REG for selftest\n");
      ret = -EIO;
      goto out;
    }

  /* Dummy reads so that values have stabilized */

  for (i = 0; i < 20; i++)
    {
      if (lis2dh_get_raw_readings(priv, &err) == NULL)
        {
          ret = -EIO;
          goto out;
        }
    }

  for (i = 0; i < SELFTEST_BUF_SIZE; i++)
    {
      results = lis2dh_get_raw_readings(priv, &err);
      if (results == NULL)
        {
          ret = -EIO;
          goto out;
        }

      avg_x_no_st += results->x;
      avg_y_no_st += results->y;
      avg_z_no_st += results->z;
    }

  avg_x_no_st = avg_x_no_st / SELFTEST_BUF_SIZE;
  avg_y_no_st = avg_y_no_st / SELFTEST_BUF_SIZE;
  avg_z_no_st = avg_z_no_st / SELFTEST_BUF_SIZE;

  for (i = SELFTEST_0; i <= SELFTEST_1; i++)
    {
      avg_x_with_st = 0;
      avg_y_with_st = 0;
      avg_z_with_st = 0;

      /* Enable self-test 0 or 1 at +/-2g FS with BDU and HR bits enabled. */

      buf = (i == SELFTEST_0) ? 0x8a : 0x8c;

      if (lis2dh_write_register(priv, ST_LIS2DH_CTRL_REG4, buf) != OK)
        {
          lis2dh_dbg("lis2dh: Failed to write CTRL4 REG for selftest\n");
          ret = -EIO;
          goto out;
        }

      /* Dummy reads so that values have stabilized */

      for (i = 0; i < 10; i++)
        {
          if (lis2dh_get_raw_readings(priv, &err) == NULL)
            {
              ret = -EIO;
              goto out;
            }
        }

      for (j = 0; j < SELFTEST_BUF_SIZE; j++)
        {
          results = lis2dh_get_raw_readings(priv, &err);
          if (results == NULL)
            {
              ret = -EIO;
              goto out;
            }

          avg_x_with_st += results->x;
          avg_y_with_st += results->y;
          avg_z_with_st += results->z;
        }

      avg_x_with_st = avg_x_with_st / SELFTEST_BUF_SIZE;
      avg_y_with_st = avg_y_with_st / SELFTEST_BUF_SIZE;
      avg_z_with_st = avg_z_with_st / SELFTEST_BUF_SIZE;

      abs_st_x_value = abs(avg_x_with_st - avg_x_no_st);
      abs_st_y_value = abs(avg_y_with_st - avg_y_no_st);
      abs_st_z_value = abs(avg_z_with_st - avg_z_no_st);

      syslog(LOG_NOTICE, "ST %d, ABSX: %d, ABSY: %d, ABSZ: %d\n",
             i, abs_st_x_value, abs_st_y_value, abs_st_z_value);

      if (abs_st_x_value < SELFTEST_ABS_DIFF_MIN_12BIT ||
          abs_st_x_value > SELFTEST_ABS_DIFF_MAX_12BIT ||
          abs_st_y_value < SELFTEST_ABS_DIFF_MIN_12BIT ||
          abs_st_y_value > SELFTEST_ABS_DIFF_MAX_12BIT ||
          abs_st_z_value < SELFTEST_ABS_DIFF_MIN_12BIT ||
          abs_st_z_value > SELFTEST_ABS_DIFF_MAX_12BIT)
        {
          syslog(LOG_NOTICE, "Selftest %d fail! Limits (%d <= value <= %d). "
                 "Results: x: %d, y: %d, z: %d ",
                 i,
                 SELFTEST_ABS_DIFF_MIN_12BIT, SELFTEST_ABS_DIFF_MAX_12BIT,
                 abs_st_x_value, abs_st_y_value, abs_st_z_value);
          ret = -ERANGE;
          goto out;
        }
    }

  /* Verify INT1 and INT2 lines */

  if (lis2dh_clear_registers(priv) != OK)
    {
      ret = -EIO;
      goto out;
    }

  /* Both INT lines should be low */

  if (priv->config->read_int1_pin != NULL)
    {
      if (priv->config->read_int1_pin() != 0)
        {
          syslog(LOG_NOTICE, "INT1 line is HIGH - expected LOW\n");
          ret = -ENXIO;
          goto out;
        }
    }

  if (priv->config->read_int2_pin)
    {
      if (priv->config->read_int2_pin() != 0)
        {
          syslog(LOG_NOTICE, "INT2 line is HIGH - expected LOW\n");
          ret = -ENODEV;
          goto out;
        }
    }

  /* 400Hz ODR all axes enabled
     FIFO overrun & DATA READY on INT1
     FIFO enabled and INT1 & INT2 latched
     FIFO mode, INT1 , THS 0
     OR combination, all events enabled */

  if ((lis2dh_write_register(priv, ST_LIS2DH_CTRL_REG1, 0x77) != OK) ||
      (lis2dh_write_register(priv, ST_LIS2DH_CTRL_REG3, 0x12) != OK) ||
      (lis2dh_write_register(priv, ST_LIS2DH_CTRL_REG5, 0x4a) != OK) ||
      (lis2dh_write_register(priv, ST_LIS2DH_FIFO_CTRL_REG, 0x40) != OK) ||
      (lis2dh_write_register(priv, ST_LIS2DH_INT1_CFG_REG, 0x3f) != OK))
    {
      syslog(LOG_NOTICE, "Writing registers for INT line check failed\n");
      ret = -EIO;
      goto out;
    }

  /* Clear INT1 & INT2 */

  if ((lis2dh_read_register(priv, ST_LIS2DH_INT1_SRC_REG) == ERROR) ||
      (lis2dh_read_register(priv, ST_LIS2DH_INT2_SRC_REG) == ERROR))
    {
      syslog(LOG_NOTICE, "Failed to clear INT1 / INT2 registers\n");
      ret = -EIO;
      goto out;
    }

  nxsig_usleep(20000);

  /* Now INT1 should have been latched high and INT2 should be still low */

  if (priv->config->read_int1_pin)
    {
      if (priv->config->read_int1_pin() != 1)
        {
          syslog(LOG_NOTICE, "INT1 line is LOW - expected HIGH\n");
          ret = -ENXIO;
          goto out;
        }
    }

  if (priv->config->read_int2_pin != NULL)
    {
      if (priv->config->read_int2_pin() != 0)
        {
          syslog(LOG_NOTICE, "INT2 line is HIGH - expected LOW\n");
          ret = -ENODEV;
          goto out;
        }

      /* Enable interupt 1 on INT2 pin */

      if (lis2dh_write_register(priv, ST_LIS2DH_CTRL_REG6, 0x40) != OK)
        {
          syslog(LOG_NOTICE, "Failed to enable interrupt 1 on INT2 pin");
          ret = -EIO;
          goto out;
        }

      nxsig_usleep(20000);

      if (priv->config->read_int2_pin() != 1)
        {
          syslog(LOG_NOTICE, "INT2 line is LOW - expected HIGH\n");
          ret = -ENODEV;
          goto out;
        }
    }

out:
  (void)lis2dh_clear_registers(priv);
  lis2dh_powerdown(priv);

  return ret;
}

/****************************************************************************
 * Name: lis2dh_raw_to_mg
 *
 * Description:
 *   Convert raw acceleration value to mg
 *
 * Input Parameters:
 *   raw_hibyte   - Hi byte of raw data
 *   raw_lobyte   - Lo byte of raw data
 *
 * Returned Value:
 *   Returns acceleration value in mg
 ****************************************************************************/

static int16_t lis2dh_raw_convert_to_12bit(uint8_t raw_hibyte,
                                           uint8_t raw_lobyte)
{
  int16_t value;

  value = (raw_hibyte << 8) | raw_lobyte;
  value = value >> 4;

  value &= 0xfff;
  if (value & 0x800)
    {
      value = ~value;
      value &= 0xfff;
      value += 1;
      value = -value;
    }

  return value;
}

/****************************************************************************
 * Name: lis2dh_data_available
 *
 * Description:
 *   Check if new data is available to read
 *
 * Input Parameters:
 *   dev        - pointer to LIS2DH Private Structure
 *
 * Returned Value:
 *   Return true if new data is available. Otherwise returns false
 *
 ****************************************************************************/

static bool lis2dh_data_available(FAR struct lis2dh_dev_s *dev)
{
  uint8_t retval;

  DEBUGASSERT(dev);

  if (lis2dh_access(dev, ST_LIS2DH_STATUS_REG, &retval,
                    sizeof(retval)) == sizeof(retval))
    {
      return ((retval & ST_LIS2DH_SR_ZYXDA) != 0);
    }
  return false;
}

/****************************************************************************
 * Name: lis2dh_get_raw_readings
 *
 * Description:
 *   Read X, Y, Z - acceleration values from chip
 *
 * Input Parameters:
 *   dev        - pointer to LIS2DH Private Structure
 *
 * Returned Value:
 *   Returns acceleration vectors (High resolution = 12bit values) on
 *   success, NULL otherwise.
 *
 ****************************************************************************/

static FAR const struct lis2dh_vector_s *
  lis2dh_get_raw_readings(FAR struct lis2dh_dev_s *dev, int *err)
{
  uint8_t retval[6];
  uint8_t retries_left = SELFTEST_MAX_READ_ATTEMPTS;

  DEBUGASSERT(dev);

  *err = 0;

  while (--retries_left > 0)
    {
      nxsig_usleep(20000);
      if (lis2dh_data_available(dev))
        {
          if (lis2dh_access(dev, ST_LIS2DH_OUT_X_L_REG, retval,
                            sizeof(retval)) == sizeof(retval))
            {
              dev->vector_data.x = lis2dh_raw_convert_to_12bit(retval[1], retval[0]);
              dev->vector_data.y = lis2dh_raw_convert_to_12bit(retval[3], retval[2]);
              dev->vector_data.z = lis2dh_raw_convert_to_12bit(retval[5], retval[4]);
              return &dev->vector_data;
            }

          return NULL;
        }
    }

  return NULL;
}
#endif /* CONFIG_LIS2DH_DRIVER_SELFTEST */

/****************************************************************************
 * Name: lis2dh_clear_interrupts
 *
 * Description:
 *   Clear interrupts from LIS2DH chip
 *
 ****************************************************************************/

static int lis2dh_clear_interrupts(FAR struct lis2dh_dev_s *priv,
                                   uint8_t interrupts)
{
  uint8_t buf;
  int ret = OK;

  if (interrupts & LIS2DH_INT1)
    {
      /* Make sure interrupt will get cleared (by reading this register) in
       * case of latched configuration.
       */

      if (lis2dh_access(priv, ST_LIS2DH_INT1_SRC_REG, &buf, 1) != 1)
        {
          lis2dh_dbg("lis2dh: Failed to read INT1_SRC_REG\n");
          ret = -EIO;
        }
    }

  if (interrupts & LIS2DH_INT2)
    {
      /* Make sure interrupt will get cleared (by reading this register) in
       * case of latched configuration.
       */

      if (lis2dh_access(priv, ST_LIS2DH_INT2_SRC_REG, &buf, 1) != 1)
        {
          lis2dh_dbg("lis2dh: Failed to read INT2_SRC_REG\n");
          ret = -EIO;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: lis2dh_get_reading
 *
 * Description:
 *   Read X, Y, Z - acceleration value from chip
 *
 * Input Parameters:
 *   dev        - pointer to LIS2DH Private Structure
 *   force_read - Read even if new data is not available (old data)
 *
 * Returned Value:
 *   Returns OK if success, negative error code otherwise
 *
 ****************************************************************************/

static int lis2dh_get_reading(FAR struct lis2dh_dev_s *dev,
                              FAR struct lis2dh_vector_s *res,
                              bool force_read)
{
  int scale = dev->scale;
  uint8_t retval[7];
  int16_t x;
  int16_t y;
  int16_t z;

  if (lis2dh_access(dev, ST_LIS2DH_STATUS_REG, retval, 7) == 7)
    {
      /* If result is not yet ready, return NULL */

      if (!force_read && !(retval[0] & ST_LIS2DH_SR_ZYXDA))
        {
          lis2dh_dbg("lis2dh: Results were not ready\n");
          return -EAGAIN;
        }

      /* Add something to entropy pool. */

      add_sensor_randomness((((uint32_t)retval[6] << 25) |
                             ((uint32_t)retval[6] >> 7)) ^
                            ((uint32_t)retval[5] << 20) ^
                            ((uint32_t)retval[4] << 15) ^
                            ((uint32_t)retval[3] << 10) ^
                            ((uint32_t)retval[2] << 5) ^
                            ((uint32_t)retval[1] << 0));

      x = lis2dh_raw_to_mg(retval[2], retval[1], scale);
      y = lis2dh_raw_to_mg(retval[4], retval[3], scale);
      z = lis2dh_raw_to_mg(retval[6], retval[5], scale);

      if (dev->setup->xy_axis_fixup)
        {
          res->x = y;
          res->y = -x;
        }
      else
        {
          res->x = x;
          res->y = y;
        }

      res->z = z;
      return OK;
    }

  return -EIO;
}

/****************************************************************************
 * Name: lis2dh_get_fifo_readings
 *
 * Description:
 *   Bulk read from FIFO
 *
 ****************************************************************************/

static unsigned int lis2dh_get_fifo_readings(FAR struct lis2dh_dev_s *priv,
                                             FAR struct lis2dh_result *res,
                                             unsigned int readcount,
                                             FAR int *perr)
{
  int scale = priv->scale;
  union
    {
      uint8_t                raw[6];
      struct lis2dh_vector_s sample;
    } *buf = (void *)&res->measurements[res->header.meas_count];
  bool xy_axis_fixup = priv->setup->xy_axis_fixup;
  size_t buflen = readcount * 6;
  int16_t x;
  int16_t y;
  int16_t z;
  unsigned int i;

  if (readcount == 0)
    {
      return 0;
    }

  if (lis2dh_access(priv, ST_LIS2DH_OUT_X_L_REG, (void *)buf, buflen) != buflen)
    {
      lis2dh_dbg("lis2dh: Failed to read FIFO (%d bytes, %d samples)\n",
                 buflen, readcount);
      *perr = -EIO;
      return 0;
    }

  /* Add something to entropy pool. */

  up_rngaddentropy(RND_SRC_SENSOR, (void *)buf, buflen / 4);

  /* Convert raw values to mG */

  for (i = 0; i < readcount; i++)
    {
      x = lis2dh_raw_to_mg(buf[i].raw[1], buf[i].raw[0], scale);
      y = lis2dh_raw_to_mg(buf[i].raw[3], buf[i].raw[2], scale);
      z = lis2dh_raw_to_mg(buf[i].raw[5], buf[i].raw[4], scale);

      if (xy_axis_fixup)
        {
          buf[i].sample.x = y;
          buf[i].sample.y = -x;
        }
      else
        {
          buf[i].sample.x = x;
          buf[i].sample.y = y;
        }

      buf[i].sample.z = z;
    }

  return readcount;
}

/****************************************************************************
 * Name: lis2dh_raw_to_mg
 *
 * Description:
 *   Convert raw acceleration value to mg
 *
 * Input Parameters:
 *   raw_hibyte   - Hi byte of raw data
 *   raw_lobyte   - Lo byte of raw data
 *   scale        - full scale in milliG
 *
 * Returned Value:
 *   Returns acceleration value in mg
 *
 ****************************************************************************/

static inline int16_t lis2dh_raw_to_mg(uint8_t raw_hibyte, uint8_t raw_lobyte,
                                       int scale)
{
  int16_t value;

  /* Value is signed integer, range INT16_MIN..INT16_MAX. */

  value = (raw_hibyte << 8) |  raw_lobyte;

  /* Scale to mg, INT16_MIN..INT16_MAX => -scale..scale */

  return (int32_t)value * scale / INT16_MAX;
}

/****************************************************************************
 * Name: lis2dh_read_temp
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int lis2dh_read_temp(FAR struct lis2dh_dev_s *dev, FAR int16_t *temper)
{
  int ret;
  uint8_t buf[2] = { 0 };

  ret = lis2dh_access(dev, ST_LIS2DH_OUT_TEMP_L_REG, buf, 2);
  if (ret < 0)
    {
      lis2dh_dbg("Cannot read temperature\n");
      return -EIO;
    }

  *temper = buf[0] | ((int16_t)buf[1] << 8);

  return OK;
}

/****************************************************************************
 * LIS2DH Access with range check
 *
 * Description:
 *   Read or write data via I2C
 *
 * Input Parameters:
 *   dev     LIS2DH Private Structure
 *   subaddr LIS2DH Sub Address
 *   buf     Pointer to buffer, either for read or write access
 *   length  When >0 it denotes read access, when <0 it denotes write access
 *           of -length
 *
 * Returned Value:
 *   Returns actual length of data on success or negated errno.
 *
 ****************************************************************************/

static int lis2dh_access(FAR struct lis2dh_dev_s *dev, uint8_t subaddr,
                         FAR uint8_t *buf, int length)
{
  uint16_t flags = 0;
  int retval;
  int retries;

  DEBUGASSERT(dev != NULL && buf != NULL && length != 0);

  if (length > 0)
    {
      flags = I2C_M_READ;
    }
  else
    {
      flags = I2C_M_NOSTART;
      length = -length;
    }

  /* Check valid address ranges and set auto address increment flag */

  if (subaddr == ST_LIS2DH_STATUS_AUX_REG)
    {
      if (length > 1)
        {
          length = 1;
        }
    }
  else if (subaddr >= ST_LIS2DH_OUT_TEMP_L_REG && subaddr < 0x10)
    {
      if (length > (0x10 - subaddr))
        {
          length = 0x10 - subaddr;
        }
    }

  else if (subaddr >= ST_LIS2DH_TEMP_CFG_REG && subaddr <= ST_LIS2DH_ACT_DUR_REG)
    {
      if (subaddr == ST_LIS2DH_OUT_X_L_REG)
        {
          /* FIFO bulk read, length maximum 6*32 = 192 bytes. */
          if (length > 6 * 32)
            {
              length = 6 * 32;
            }
        }
      else
        {
          if (length > (ST_LIS2DH_ACT_DUR_REG + 1 - subaddr))
            {
              length = ST_LIS2DH_ACT_DUR_REG + 1 - subaddr;
            }
        }
    }
  else
    {
      return -EFAULT;
    }

  if (length > 1)
    {
      subaddr |= 0x80;
    }

  for (retries = 0; retries < LIS2DH_I2C_RETRIES; retries++)
    {
      /* Create message and send */

      struct i2c_msg_s msgv[2] =
      {
          {
              .frequency = CONFIG_LIS2DH_I2C_FREQUENCY,
              .addr      = dev->addr,
              .flags     = 0,
              .buffer    = &subaddr,
              .length    = 1
          },
          {
              .frequency = CONFIG_LIS2DH_I2C_FREQUENCY,
              .addr      = dev->addr,
              .flags     = flags,
              .buffer    = buf,
              .length    = length
          }
      };

      retval = I2C_TRANSFER(dev->i2c, msgv, 2);
      if (retval >= 0)
        {
          return length;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */
#ifdef CONFIG_I2C_RESET
          int ret = I2C_RESET(dev->i2c);
          if (ret < 0)
            {
              lis2dh_dbg("I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
          continue;
        }
    }

  lis2dh_dbg("failed, error: %d\n", retval);
  return retval;
}

/****************************************************************************
 * Name: lis2dh_reboot
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int lis2dh_reboot(FAR struct lis2dh_dev_s *dev)
{
  struct timespec start, curr;
  int32_t diff_msec;
  uint8_t value;

  /* Prefer monotonic for timeout calculation when enabled. */

#ifdef CONFIG_CLOCK_MONOTONIC
  (void)clock_gettime(CLOCK_MONOTONIC, &start);
#else
  (void)clock_gettime(CLOCK_REALTIME, &start);
#endif

  /* Reboot to reset chip. */

  value = ST_LIS2DH_CR5_BOOT;
  if (lis2dh_access(dev, ST_LIS2DH_CTRL_REG5, &value, -1) != 1)
    {
      return -EIO;
    }

  /* Reboot is completed when reboot bit is cleared. */

  do
    {
      value = 0;
      if (lis2dh_access(dev, ST_LIS2DH_CTRL_REG5, &value, 1) != 1)
        {
          return -EIO;
        }

      if (!(value & ST_LIS2DH_CR5_BOOT))
        {
          break;
        }

#ifdef CONFIG_CLOCK_MONOTONIC
      (void)clock_gettime(CLOCK_MONOTONIC, &curr);
#else
      (void)clock_gettime(CLOCK_REALTIME, &curr);
#endif

      diff_msec = (curr.tv_sec - start.tv_sec) * 1000;
      diff_msec += (curr.tv_nsec - start.tv_nsec) / (1000 * 1000);

      if (diff_msec > 100)
        {
          return -ETIMEDOUT;
        }

       nxsig_usleep(1);
    }
  while (true);

  /* Reboot completed, chip is now in power-down state. */

  return OK;
}

/****************************************************************************
 * Name: lis2dh_powerdown
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int lis2dh_powerdown(FAR struct lis2dh_dev_s * dev)
{
  uint8_t buf = 0;
  int ret = OK;

  if (lis2dh_access(dev, ST_LIS2DH_CTRL_REG1, &buf, -1) != 1)
    {
      lis2dh_dbg("Failed to clear CTRL_REG1\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * LIS2DH Setup
 *
 * Description:
 *   Apply new register setup
 *
 * Input Parameters:
 *   dev        - pointer to LIS2DH Private Structure
 *   new_setup  - pointer to new setup data to be configured
 *
 * Returned Value:
 *   Returns OK on success, ERROR otherwise.
 *
 ****************************************************************************/

static int lis2dh_setup(FAR struct lis2dh_dev_s * dev,
                        FAR struct lis2dh_setup *new_setup)
{
  uint8_t value;

  dev->setup = new_setup;

  /* Clear old configuration. On first boot after power-loss, reboot bit does
   * not get cleared, and lis2dh_reboot() times out. Anyway, chip accepts
   * new configuration and functions correctly. */

  (void)lis2dh_reboot(dev);

  /* TEMP_CFG_REG */

  value = dev->setup->temp_enable ? (0x3 << 6): 0;
  if (lis2dh_access(dev, ST_LIS2DH_TEMP_CFG_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* CTRL_REG2 */

  value = dev->setup->hpmode | dev->setup->hpcf | dev->setup->fds |
      dev->setup->hpclick | dev->setup->hpis2 | dev->setup->hpis1;
  if (lis2dh_access(dev, ST_LIS2DH_CTRL_REG2, &value, -1) != 1)
    {
      goto error;
    }

  /* CTRL_REG3 */

  value = dev->setup->int1_click_enable | dev->setup->int1_aoi_enable |
      dev->setup->int2_aoi_enable | dev->setup->int1_drdy_enable |
      dev->setup->int2_drdy_enable | dev->setup->int_wtm_enable |
      dev->setup->int_overrun_enable;
  if (lis2dh_access(dev, ST_LIS2DH_CTRL_REG3, &value, -1) != 1)
    {
      goto error;
    }

  /* CTRL_REG4 */

  value = dev->setup->bdu | dev->setup->endian | dev->setup->fullscale |
      dev->setup->high_resolution_enable | dev->setup->selftest |
      dev->setup->spi_mode;
  if (lis2dh_access(dev, ST_LIS2DH_CTRL_REG4, &value, -1) != 1)
    {
      goto error;
    }

  /* CTRL_REG5 */

  value = dev->setup->reboot | dev->setup->fifo_enable | dev->setup->int1_latch |
      dev->setup->int1_4d_enable | dev->setup->int2_latch |
      dev->setup->int2_4d_enable;
  if (lis2dh_access(dev, ST_LIS2DH_CTRL_REG5, &value, -1) != 1)
    {
      goto error;
    }

  /* CTRL_REG6 */

  value = dev->setup->int2_click_enable | dev->setup->int_enable |
      dev->setup->boot_int1_enable | dev->setup->high_low_active;
  if (lis2dh_access(dev, ST_LIS2DH_CTRL_REG6, &value, -1) != 1)
    {
      goto error;
    }

  /* REFERENCE */

  value = dev->setup->reference;
  if (lis2dh_access(dev, ST_LIS2DH_REFERENCE_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* FIFO_CTRL_REG */

  value = dev->setup->fifo_mode | dev->setup->trigger_selection |
      dev->setup->fifo_trigger_threshold;
  if (lis2dh_access(dev, ST_LIS2DH_FIFO_CTRL_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* INT1_CFG */

  value = dev->setup->int1_interrupt_mode | dev->setup->int1_enable_6d | dev->setup->int1_int_z_high_enable |
      dev->setup->int1_int_z_low_enable | dev->setup->int1_int_y_high_enable |
      dev->setup->int1_int_y_low_enable | dev->setup->int1_int_x_high_enable |
      dev->setup->int1_int_x_low_enable;
  if (lis2dh_access(dev, ST_LIS2DH_INT1_CFG_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* INT1_THS */

  value = dev->setup->int1_int_threshold;
  if (lis2dh_access(dev, ST_LIS2DH_INT1_THS_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* INT1_DURATION */

  value = dev->setup->int1_int_duration;
  if (lis2dh_access(dev, ST_LIS2DH_INT1_DUR_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* INT2_CFG */

  value = dev->setup->int2_interrupt_mode | dev->setup->int2_enable_6d | dev->setup->int2_int_z_high_enable |
      dev->setup->int2_int_z_low_enable | dev->setup->int2_int_y_high_enable |
      dev->setup->int2_int_y_low_enable | dev->setup->int2_int_x_high_enable |
      dev->setup->int2_int_x_low_enable;
  if (lis2dh_access(dev, ST_LIS2DH_INT2_CFG_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* INT2_THS */

  value = dev->setup->int2_int_threshold;
  if (lis2dh_access(dev, ST_LIS2DH_INT2_THS_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* INT2_DURATION */

  value = dev->setup->int2_int_duration;
  if (lis2dh_access(dev, ST_LIS2DH_INT2_DUR_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* CLICK_CFG */

  value = dev->setup->z_double_click_enable | dev->setup->z_single_click_enable |
      dev->setup->y_double_click_enable | dev->setup->y_single_click_enable |
      dev->setup->x_double_click_enable | dev->setup->x_single_click_enable;
  if (lis2dh_access(dev, ST_LIS2DH_CLICK_CFG_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* CLICK_THS */

  value = dev->setup->click_threshold;
  if (lis2dh_access(dev, ST_LIS2DH_CLICK_THS_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* TIME_LIMIT */

  value = dev->setup->click_time_limit;
  if (lis2dh_access(dev, ST_LIS2DH_TIME_LIMIT_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* TIME_LATENCY */

  value = dev->setup->click_time_latency;
  if (lis2dh_access(dev, ST_LIS2DH_TIME_LATENCY_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* TIME_WINDOW */

  value = dev->setup->click_time_window;
  if (lis2dh_access(dev, ST_LIS2DH_TIME_WINDOW_REG, &value, -1) != 1)
    {
      goto error;
    }

  /* CTRL_REG1 */

  value = dev->setup->data_rate | dev->setup->low_power_mode_enable |
      dev->setup->zen | dev->setup->yen | dev->setup->xen;
  if (lis2dh_access(dev, ST_LIS2DH_CTRL_REG1, &value, -1) != 1)
    {
      goto error;
    }

  switch (dev->setup->fullscale & 0x30)
    {
    default:
    case ST_LIS2DH_CR4_FULL_SCALE_2G:
      dev->scale = 2000;
      break;

    case ST_LIS2DH_CR4_FULL_SCALE_4G:
      dev->scale = 4000;
      break;

    case ST_LIS2DH_CR4_FULL_SCALE_8G:
      dev->scale = 8000;
      break;

    case ST_LIS2DH_CR4_FULL_SCALE_16G:
      dev->scale = 16000;
      break;
    }

  if (dev->setup->fifo_enable)
    {
      dev->fifo_used = true;

      if (lis2dh_fifo_start(dev) < 0)
        {
          goto error;
        }
    }
  else
    {
      dev->fifo_used = false;
    }

  return OK;

error:

  /* Setup failed - power down */

  lis2dh_powerdown(dev);
  return -EIO;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis2dh_register
 *
 * Description:
 *   Register the LIS2DH character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/acc0"
 *   i2c     - An instance of the I2C interface to use to communicate with LIS2DH
 *   addr    - The I2C address of the LIS2DH.  The base I2C address of the LIS2DH
 *             is 0x18.  Bit 0 can be controlled via SA0 pad - when connected to
 *             voltage supply the address is 0x19.
 *   config  - Pointer to LIS2DH configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis2dh_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, FAR struct lis2dh_config_s *config)
{
  FAR struct lis2dh_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL && config != NULL);

  priv = (FAR struct lis2dh_dev_s *)kmm_zalloc(sizeof(struct lis2dh_dev_s));
  if (!priv)
    {
      lis2dh_dbg("lis2dh: Failed to allocate instance\n");
      return -ENOMEM;
    }

  nxsem_init(&priv->devsem, 0, 1);

  priv->fifo_used = false;
#ifdef LIS2DH_COUNT_INTS
  priv->int_pending = 0;
#else
  priv->int_pending = false;
#endif

  priv->i2c = i2c;
  priv->addr = addr;
  priv->config = config;

  ret = register_driver(devpath, &g_lis2dhops, 0666, priv);
  if (ret < 0)
    {
      lis2dh_dbg("lis2dh: Failed to register driver: %d\n", ret);
      goto errout_with_priv;
    }

  if (priv->config->irq_clear)
    {
      priv->config->irq_clear(config);
    }
  priv->config->irq_attach(config, lis2dh_int_handler, priv);
  priv->config->irq_enable(config, false);
  return OK;

errout_with_priv:
  nxsem_destroy(&priv->devsem);
  kmm_free(priv);

  return ret;
}
