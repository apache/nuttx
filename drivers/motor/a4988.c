/****************************************************************************
 * drivers/motor/a4988.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/motor/a4988.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct a4988_dev_s
{
  FAR struct a4988_ops_s *ops;   /* A4988 ops */
  uint8_t  auto_idle;            /* If true, go in idle mode between movement */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int a4988_setup(FAR struct stepper_lowerhalf_s *dev);
static int a4988_shutdown(FAR struct stepper_lowerhalf_s *dev);
static int a4988_work(FAR struct stepper_lowerhalf_s *dev,
                      FAR struct stepper_job_s const *param);
static int a4988_update_status(FAR struct stepper_lowerhalf_s *dev);
static int a4988_clear(FAR struct stepper_lowerhalf_s *dev, uint8_t fault);
static int a4988_idle(FAR struct stepper_lowerhalf_s *dev, uint8_t idle);
static int a4988_microstepping(FAR struct stepper_lowerhalf_s *dev,
                               uint16_t resolution);
static int a4988_ioctl(FAR struct stepper_lowerhalf_s *dev, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stepper_ops_s g_a4988_ops =
{
  a4988_setup,          /* setup */
  a4988_shutdown,       /* shutdown */
  a4988_work,           /* work */
  a4988_update_status,  /* update status */
  a4988_clear,          /* clear */
  a4988_idle,           /* idle */
  a4988_microstepping,  /* microstepping */
  a4988_ioctl           /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int a4988_setup(FAR struct stepper_lowerhalf_s *dev)
{
  FAR struct a4988_dev_s *priv = (FAR struct a4988_dev_s *)dev->priv;
  priv->ops->idle(false);
  priv->ops->enable(true);

  return 0;
}

static int a4988_shutdown(FAR struct stepper_lowerhalf_s *dev)
{
  FAR struct a4988_dev_s *priv = (FAR struct a4988_dev_s *)dev->priv;
  priv->ops->idle(true);
  priv->ops->enable(false);

  return 0;
}

static int a4988_work(FAR struct stepper_lowerhalf_s *dev,
                      FAR struct stepper_job_s const *job)
{
  FAR struct a4988_dev_s *priv = (FAR struct a4988_dev_s *)dev->priv;
  int delay;
  int count;

  if (job->steps == 0)
    {
      /* Nothing to do */

      return 0;
    }

  /* Compute delay between pulse */

  delay = USEC_PER_SEC / job->speed;
  if (delay < 1)
    {
      delay = 1;
      stpwarn("Delay is clamped to 1 us\n");
    }

  stpinfo("Delay is %d us\n", delay);

  /* Set direction */

  if (job->steps > 0)
    {
      priv->ops->direction(true);
      count = job->steps;
    }
  else
    {
      priv->ops->direction(false);
      count = -job->steps;
    }

  if (priv->auto_idle)
    {
      priv->ops->idle(false);
      nxsig_usleep(USEC_PER_MSEC);
    }

  dev->status.state = STEPPER_STATE_RUN;
  for (int32_t i = 0; i < count; ++i)
    {
      priv->ops->step(true);
      up_udelay(1);
      priv->ops->step(false);
      up_udelay(delay);
    }

  dev->status.state = STEPPER_STATE_READY;

  if (priv->auto_idle)
    {
      priv->ops->idle(true);
    }

  /* Update steps done (a4988 cannot detect miss steps) */

  dev->status.position += job->steps;

  return 0;
}

static int a4988_update_status(FAR struct stepper_lowerhalf_s *dev)
{
  /* No state to fetch */

  return 0;
}

static int a4988_clear(FAR struct stepper_lowerhalf_s *dev, uint8_t fault)
{
  /* No fault to clear ever */

  return 0;
}

static int a4988_idle(FAR struct stepper_lowerhalf_s *dev, uint8_t idle)
{
  FAR struct a4988_dev_s *priv = (FAR struct a4988_dev_s *)dev->priv;

  if (idle == STEPPER_AUTO_IDLE)
    {
      priv->auto_idle = true;
      return 0;
    }

  priv->auto_idle = false;

  if (idle == STEPPER_ENABLE_IDLE)
    {
      priv->ops->idle(true);
      dev->status.state = STEPPER_STATE_IDLE;
    }
  else
    {
      priv->ops->idle(false);
      nxsig_usleep(USEC_PER_MSEC);
      dev->status.state = STEPPER_STATE_READY;
    }

  return 0;
}

static int a4988_microstepping(FAR struct stepper_lowerhalf_s *dev,
                               uint16_t resolution)
{
  FAR struct a4988_dev_s *priv = (FAR struct a4988_dev_s *)dev->priv;

  switch (resolution)
    {
      case 1:
        {
          priv->ops->microstepping(false, false, false);
        }
        break;

      case 2:
        {
          priv->ops->microstepping(true, false, false);
        }
        break;

      case 4:
        {
          priv->ops->microstepping(false, true, false);
        }
        break;

      case 8:
        {
          priv->ops->microstepping(true, true, false);
        }
        break;

      case 16:
        {
          priv->ops->microstepping(true, true, true);
        }
        break;

      default:
        {
          return -EINVAL;
        }
    }

  return 0;
}

static int a4988_ioctl(FAR struct stepper_lowerhalf_s *dev, int cmd,
                       unsigned long arg)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int a4988_register(FAR const char *devpath, FAR struct a4988_ops_s *ops)
{
  FAR struct a4988_dev_s *priv;
  FAR struct stepper_lowerhalf_s *lower;
  int ret = 0;

  /* Sanity check */

  DEBUGASSERT(ops != NULL);

  /* Initialize the a4988 dev structure */

  priv = kmm_malloc(sizeof(struct a4988_dev_s));
  if (priv == NULL)
    {
      stperr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->ops = ops;

  lower = kmm_malloc(sizeof(struct stepper_lowerhalf_s));
  if (lower == NULL)
    {
      stperr("Failed to allocate instance\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  lower->priv = priv;
  lower->status.fault = STEPPER_FAULT_CLEAR;
  lower->status.state = STEPPER_STATE_READY;
  lower->status.position = 0;
  lower->ops = &g_a4988_ops;

  /* Initialize lower layer (only once) */

  priv->ops->initialize();

  /* Register the character driver */

  ret = stepper_register(devpath, lower);
  if (ret < 0)
    {
      stperr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      kmm_free(lower);
      return ret;
    }

  stpinfo("a4988 registered at %s\n", devpath);
  return ret;
}
