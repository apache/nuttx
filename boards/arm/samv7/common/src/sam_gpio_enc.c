/****************************************************************************
 * boards/arm/samv7/common/src/sam_gpio_enc.c
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
#include <stdio.h>

#include <nuttx/board.h>
#include <nuttx/sensors/qencoder.h>
#include <arch/board/board.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "arm_internal.h"

#include "board_gpio_enc.h"

#if defined(CONFIG_SENSORS_QENCODER) && defined(CONFIG_SAMV7_GPIO_ENC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_qeconfig_s
{
  gpio_pinset_t enca;         /* ENC_A pin */
  gpio_pinset_t encb;         /* ENC_B pin */
  int enca_irq;               /* ENC_A irq */
  int encb_irq;               /* ENC_B irq */
  uint32_t  position;         /* Current position */
  uint32_t  position_base;    /* Base position */
  uint32_t  error;            /* Error count */
};

struct sam_gpio_enc_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  FAR const struct qe_ops_s *ops;           /* Lower half callback structure */
  FAR struct sam_qeconfig_s *config;        /* static configuration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int board_gpio_enc_irqx(gpio_pinset_t pinset, int irq,
                               xcpt_t irqhandler, void *arg);
static int sam_gpio_enc_interrupt(int irq, FAR void *context,
                                     FAR void *arg);
static int sam_gpio_enc_position(FAR struct qe_lowerhalf_s *lower,
                                 FAR int32_t *pos);
static int sam_gpio_enc_setup(FAR struct qe_lowerhalf_s *lower);
static int sam_gpio_enc_shutdown(FAR struct qe_lowerhalf_s *lower);
static int sam_gpio_enc_reset(FAR struct qe_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qe_ops_s g_qecallbacks =
{
  .setup     = sam_gpio_enc_setup,
  .shutdown  = sam_gpio_enc_shutdown,
  .position  = sam_gpio_enc_position,
  .setposmax = NULL,
  .reset     = sam_gpio_enc_reset,
  .ioctl     = NULL,
};

static struct sam_qeconfig_s sam_gpio_enc_config =
{
  .position = 0,
  .position_base = 0,
  .error = 0,
};

static struct sam_gpio_enc_lowerhalf_s sam_gpio_enc_priv =
{
  .ops = &g_qecallbacks,
  .config = &sam_gpio_enc_config,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gpio_enc_irqx
 *
 * Description:
 *   This function implements the core of the board_button_irq() logic.
 *
 ****************************************************************************/

static int board_gpio_enc_irqx(gpio_pinset_t pinset, int irq,
                             xcpt_t irqhandler, void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done. This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Are we attaching or detaching? */

  if (irqhandler != NULL)
    {
      /* Configure the interrupt */

      sam_gpioirq(pinset);
      irq_attach(irq, irqhandler, arg);
      sam_gpioirqenable(irq);
    }
  else
    {
      /* Detach and disable the interrupt */

      irq_detach(irq);
      sam_gpioirqdisable(irq);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_gpio_enc_interrupt
 *
 * Description:
 *   This function implements the core of the board_button_irq() logic.
 *
 ****************************************************************************/

static int sam_gpio_enc_interrupt(int irq, FAR void *context,
                                     FAR void *arg)
{
  FAR struct sam_gpio_enc_lowerhalf_s *dev =
    (FAR struct sam_gpio_enc_lowerhalf_s *)arg;
  FAR struct sam_qeconfig_s *priv = (struct sam_qeconfig_s *)dev->config;

  unsigned int state_a;
  unsigned int state_b;
  int32_t incr;
  uint32_t new;
  uint32_t incr_mask;

  /* Read the status of encoder pins */

  state_a = sam_gpioread(priv->enca);
  state_b = sam_gpioread(priv->encb);

  new = (state_b << 1 | (state_a ^ state_b));
  incr = ((new - priv->position + 1) & 3) - 1;
  incr_mask = (int32_t)(1 - incr) >> 31;

  /* Increment position */

  priv->position += incr & ~incr_mask;

  /* Count error */

  priv->error -= incr_mask;

  return OK;
}

/****************************************************************************
 * Name: sam_gpio_enc_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ****************************************************************************/

static int sam_gpio_enc_position(FAR struct qe_lowerhalf_s *lower,
                                 FAR int32_t *pos)
{
  FAR struct sam_gpio_enc_lowerhalf_s *priv =
    (FAR struct sam_gpio_enc_lowerhalf_s *)lower;
  FAR struct sam_qeconfig_s *config = priv->config;

  *pos = config->position - config->position_base;
  return OK;
}

/****************************************************************************
 * Name: sam_gpio_enc_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value should be zero.
 *
 ****************************************************************************/

static int sam_gpio_enc_setup(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct sam_gpio_enc_lowerhalf_s *priv =
    (FAR struct sam_gpio_enc_lowerhalf_s *)lower;
  FAR struct sam_qeconfig_s *config = priv->config;
  int ret;
  unsigned int state_a;
  unsigned int state_b;
  uint32_t new;

  /* Configure GPIOs */

  sam_configgpio(config->encb);
  sam_configgpio(config->enca);

  /* Reset the encoder position */

  state_a = sam_gpioread(config->enca);
  state_b = sam_gpioread(config->encb);
  new = (state_b << 1 | (state_a ^ state_b));
  config->position_base = config->position = new;

  /* Setup interrups for ENC_A and ENC_B pins. */

  ret = board_gpio_enc_irqx(config->enca, config->enca_irq,
                            sam_gpio_enc_interrupt, priv);
  if (ret != OK)
    {
      snerr("ERROR: board_gpio_enc_irqx for ENC_A failed %d\n", ret);
      return -ERROR;
    }

  ret = board_gpio_enc_irqx(config->encb, config->encb_irq,
                            sam_gpio_enc_interrupt, priv);
  if (ret != OK)
    {
      snerr("ERROR: board_gpio_enc_irqx for ENC_B failed %d\n", ret);
      return -ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: sam_gpio_enc_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop data collection, free any resources, disable timer hardware,
 *   and put the system into the lowest possible power usage state
 *
 ****************************************************************************/

static int sam_gpio_enc_shutdown(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct sam_gpio_enc_lowerhalf_s *priv =
    (FAR struct sam_gpio_enc_lowerhalf_s *)lower;
  FAR struct sam_qeconfig_s *config = priv->config;
  int ret;

  /* Disable GPIO interrupts. */

  ret = board_gpio_enc_irqx(config->enca, config->enca_irq,
                            NULL, priv);
  if (ret != OK)
    {
      snerr("ERROR: board_gpio_enc_irqx disable for ENC_A failed %d\n",
            ret);
      return -ERROR;
    }

  ret = board_gpio_enc_irqx(config->encb, config->encb_irq,
                            NULL, priv);
  if (ret != OK)
    {
      snerr("ERROR: board_gpio_enc_irqx disable for ENC_B failed %d\n",
            ret);
      return -ERROR;
    }

  return OK;
}

static int sam_gpio_enc_reset(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct sam_gpio_enc_lowerhalf_s *priv =
    (FAR struct sam_gpio_enc_lowerhalf_s *)lower;
  FAR struct sam_qeconfig_s *config =
    (FAR struct sam_qeconfig_s *)priv->config;

  config->position = config->position_base;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gpio_enc_init
 *
 * Description:
 *   Initialize and register the ENC driver.
 *
 * Input Parameters:
 *   enca_cfg - ENC_A pin
 *   encb_cfg - ENC_B pin
 *   enca_irq - ENC_A interrupt
 *   encb_irq - ENC_B interrupt
 *
 ****************************************************************************/

int sam_gpio_enc_init(gpio_pinset_t enca_cfg, gpio_pinset_t encb_cfg,
                      int enca_irq, int encb_irq)
{
  int ret;

  FAR struct sam_gpio_enc_lowerhalf_s *dev =
    (struct sam_gpio_enc_lowerhalf_s *)&sam_gpio_enc_priv;
  FAR struct sam_qeconfig_s *priv = (struct sam_qeconfig_s *)dev->config;

  /* Register the device as "dev/gpio_enc". */

  ret = qe_register("/dev/gpio_enc", (FAR struct qe_lowerhalf_s *)dev);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
      return -ERROR;
    }

  priv->enca = enca_cfg;
  priv->encb = encb_cfg;
  priv->enca_irq = enca_irq;
  priv->encb_irq = encb_irq;

  return OK;
}

#endif /* CONFIG_SENSORS_QENCODER && CONFIG_SAMV7_GPIO_ENC */
