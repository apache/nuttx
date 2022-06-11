/****************************************************************************
 * drivers/sensors/hc_sr04.c
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
#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/sensors/hc_sr04.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifdef CONFIG_HCSR04_DEBUG
#  define hcsr04_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define hcsr04_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hcsr04_open(FAR struct file *filep);
static int hcsr04_close(FAR struct file *filep);
static ssize_t hcsr04_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t hcsr04_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int hcsr04_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int hcsr04_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hcsr04_dev_s
{
  FAR struct hcsr04_config_s *config;
  sem_t devsem;
  sem_t conv_donesem;
  int time_start_pulse;
  int time_finish_pulse;
  volatile bool rising;
  struct pollfd *fds[CONFIG_HCSR04_NPOLLWAITERS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hcsr04ops =
{
  hcsr04_open,   /* open */
  hcsr04_close,  /* close */
  hcsr04_read,   /* read */
  hcsr04_write,  /* write */
  NULL,          /* seek */
  hcsr04_ioctl,  /* ioctl */
  hcsr04_poll    /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int hcsr04_read_distance(FAR struct hcsr04_dev_s *priv)
{
  int done;

  nxsem_get_value(&priv->conv_donesem, &done);

  if (done == 0)
    {
      return (priv->time_finish_pulse - priv->time_start_pulse);
    }
  else
    {
      return -EAGAIN;
    }
}

static int hcsr04_start_measuring(FAR struct hcsr04_dev_s *priv)
{
  /* Configure the interruption */

  priv->rising = true;
  priv->config->irq_setmode(priv->config, priv->rising);
  priv->config->irq_enable(priv->config, true);

  /* Send to 10uS trigger pulse */

  priv->config->set_trigger(priv->config, true);
  nxsig_usleep(10);
  priv->config->set_trigger(priv->config, false);

  return 0;
}

static int hcsr04_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hcsr04_dev_s *priv = inode->i_private;
  int ret;

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  nxsem_post(&priv->devsem);
  hcsr04_dbg("OPENED\n");
  return OK;
}

static int hcsr04_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hcsr04_dev_s *priv = inode->i_private;
  int ret;

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  nxsem_post(&priv->devsem);
  hcsr04_dbg("CLOSED\n");
  return OK;
}

static ssize_t hcsr04_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hcsr04_dev_s *priv = inode->i_private;
  int distance = 0;
  ssize_t length = 0;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Setup and send a pulse to start measuring */

  hcsr04_start_measuring(priv);

  /* Wait the conversion to finish */

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->conv_donesem);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  distance = hcsr04_read_distance(priv);
  if (distance < 0)
    {
      hcsr04_dbg("failed to read the distance\n");
    }
  else
    {
      /* This interface is mainly intended for easy debugging in nsh. */

      length = snprintf(buffer, buflen, "%d\n", distance);
      if (length > buflen)
        {
          length = buflen;
        }
    }

  nxsem_post(&priv->devsem);
  return length;
}

static ssize_t hcsr04_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static int hcsr04_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hcsr04_dev_s *priv = inode->i_private;
  int ret = OK;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
    case SNIOC_START_CONVERSION:
      ret = hcsr04_start_measuring(priv);
      break;

    case SNIOC_READ_RAW_DATA:
      break;

#ifdef CONFIG_HCSR04_DEBUG
    case SNIOC_DUMP_REGS:
      ret = hcsr04_dump_registers(priv);
      break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  nxsem_post(&priv->devsem);
  return ret;
}

static bool hcsr04_sample(FAR struct hcsr04_dev_s *priv)
{
  int done;

  nxsem_get_value(&priv->conv_donesem, &done);

  return (done == 0);
}

static void hcsr04_notify(FAR struct hcsr04_dev_s *priv)
{
  DEBUGASSERT(priv != NULL);

  int i;

  /* If there are threads waiting on poll() for data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the
   * data, then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_HCSR04_NPOLLWAITERS; i++)
    {
      FAR struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          hcsr04_dbg("Report events: %08" PRIx32 "\n", fds->revents);
          nxsem_post(fds->sem);
        }
    }
}

static int hcsr04_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode *inode;
  FAR struct hcsr04_dev_s *priv;
  uint32_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct hcsr04_dev_s *)inode->i_private;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
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

      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference.
       */

      for (i = 0; i < CONFIG_HCSR04_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_HCSR04_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }

      flags = enter_critical_section();
      if (hcsr04_sample(priv))
        {
          hcsr04_notify(priv);
        }

      leave_critical_section(flags);
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

static int hcsr04_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct hcsr04_dev_s *priv = (FAR struct hcsr04_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Is this the start of the pulse used to encode the distance ? */

  if (priv->rising)
    {
      /* Get the clock ticks from the free running timer */

      priv->time_start_pulse = priv->config->get_clock(priv->config);

      /* Now we need to wait for the falling edge interruption */

      priv->rising = false;
      priv->config->irq_setmode(priv->config, priv->rising);
      priv->config->irq_enable(priv->config, true);
    }
  else
    {
      /* Get the clock ticks from the free running timer */

      priv->time_finish_pulse = priv->config->get_clock(priv->config);

      /* Disable interruptions */

      priv->config->irq_enable(priv->config, false);

      /* Conversion is done */

      nxsem_post(&priv->conv_donesem);
    }

  hcsr04_dbg("HC-SR04 interrupt\n");
  hcsr04_notify(priv);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int hcsr04_register(FAR const char *devpath,
                    FAR struct hcsr04_config_s *config)
{
  int ret = 0;
  FAR struct hcsr04_dev_s *priv;

  priv = (struct hcsr04_dev_s *)kmm_zalloc(sizeof(struct hcsr04_dev_s));

  if (!priv)
    {
      hcsr04_dbg("Memory cannot be allocated for HC-SR04 sensor");
      return -ENOMEM;
    }

  priv->config = config;
  nxsem_init(&priv->devsem, 0, 1);
  nxsem_init(&priv->conv_donesem, 0, 0);

  ret = register_driver(devpath, &g_hcsr04ops, 0666, priv);
  if (ret < 0)
    {
      kmm_free(priv);
      hcsr04_dbg("Error occurred during the driver registering = %d\n", ret);
      return ret;
    }

  if (priv->config->irq_clear)
    {
      priv->config->irq_clear(priv->config);
    }

  priv->config->irq_attach(priv->config, hcsr04_int_handler, priv);
  priv->config->irq_enable(priv->config, false);
  return OK;
}
