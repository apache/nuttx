/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_qencoder.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/sensors/qencoder.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/timers/capture.h>

#include <arch/board/board.h>

#include "soc/gpio_sig_map.h"
#include "periph_ctrl.h"
#include "esp_gpio.h"
#include "hal/pcnt_ll.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Input filter *************************************************************/

#ifdef CONFIG_ESP_PCNT_U0_QE
#  ifndef CONFIG_ESP_PCNT_U0_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U0"
#  endif
#endif

#ifdef CONFIG_ESP_PCNT_U1_QE
#  ifndef CONFIG_ESP_PCNT_U1_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U1"
#  endif
#endif

#ifdef CONFIG_ESP_PCNT_U2_QE
#  ifndef CONFIG_ESP_PCNT_U2_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U2"
#  endif
#endif

#ifdef CONFIG_ESP_PCNT_U3_QE
#  ifndef CONFIG_ESP_PCNT_U3_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U3"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* NOTE: we are using Quadrature Encoder in X4 mode on ESP PCNT, then
 * instead of using 'pulse_gpio' and 'ctrl_gpio' names, we only use ch0_gpio
 * and ch1_gpio names. It avoids confusion, since the same signal that is
 * used on pin 'pulse' of CH0 is also connected to 'ctrl' pin of the CH1 and
 * 'ctrl' pin of CH0 is also connected on 'pulse' pin of CH1.
 */

/* Overall, RAM-based state structure */

struct qe_dev_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the
   * lower-half callback structure:
   */

  const struct qe_ops_s *ops;          /* Lower half callback structure */

  /* ESP driver-specific fields: */

  struct cap_lowerhalf_s *pcnt;        /* Device handle */
  int pcnt_num;                        /* Pulse Counter (PCNT) unit number */
  bool inuse;                          /* True: The lower-half driver is in-use */
  volatile int32_t position;           /* The current position offset */
  spinlock_t lock;                     /* Device specific lock. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Lower-half Quadrature Encoder Driver Methods */

static int esp_setup(struct qe_lowerhalf_s *lower);
static int esp_shutdown(struct qe_lowerhalf_s *lower);
static int esp_position(struct qe_lowerhalf_s *lower,
                          int32_t *pos);
static int esp_setposmax(struct qe_lowerhalf_s *lower, uint32_t pos);
static int esp_reset(struct qe_lowerhalf_s *lower);
static int esp_setindex(struct qe_lowerhalf_s *lower, uint32_t pos);
static int esp_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lower half callback structure */

static const struct qe_ops_s g_qecallbacks =
{
  .setup     = esp_setup,
  .shutdown  = esp_shutdown,
  .position  = esp_position,
  .setposmax = esp_setposmax,
  .reset     = esp_reset,
  .setindex  = esp_setindex,
  .ioctl     = esp_ioctl,
};

/* Per-pcnt state structures */

#ifdef CONFIG_ESP_PCNT_U0_QE
static struct qe_dev_lowerhalf_s g_pcnt0lower =
{
  .ops      = &g_qecallbacks,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP_PCNT_U1_QE
static struct qe_dev_lowerhalf_s g_pcnt1lower =
{
  .ops      = &g_qecallbacks,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP_PCNT_U2_QE
static struct qe_dev_lowerhalf_s g_pcnt2lower =
{
  .ops      = &g_qecallbacks,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP_PCNT_U3_QE
static struct qe_dev_lowerhalf_s g_pcnt3lower =
{
  .ops      = &g_qecallbacks,
  .inuse    = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value should be zero. *
 *
 * Input Parameters:
 *   lower - Pointer to Lower half driver structure
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int esp_setup(struct qe_lowerhalf_s *lower)
{
  DEBUGASSERT(lower);

  struct qe_dev_lowerhalf_s *priv = (struct qe_dev_lowerhalf_s *)lower;
  irqstate_t flags;

  /* Protected access to the registers */

  flags = spin_lock_irqsave(&priv->lock);

  /* Make sure that it is available */

  if (priv->inuse)
    {
      snerr("ERROR: PCNT%d is in-use\n", priv->pcnt_num);
      spin_unlock_irqrestore(&priv->lock, flags);
      return -EBUSY;
    }

  /* Clear the Reset bit to enable the Pulse Counter */

  priv->pcnt->ops->ioctl(priv->pcnt, CAPIOC_CLR_CNT, 0);
  priv->pcnt->ops->start(priv->pcnt);

  /* Mark as being used to prevent double opening of the same unit which
   * would reset position
   */

  priv->inuse = true;

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop data collection, free any resources, disable timer hardware,
 *   and put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   lower - Pointer to Lower half driver structure
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int esp_shutdown(struct qe_lowerhalf_s *lower)
{
  DEBUGASSERT(lower);

  struct qe_dev_lowerhalf_s *priv = (struct qe_dev_lowerhalf_s *)lower;
  irqstate_t flags;

  /* Protected access to the registers */

  flags = spin_lock_irqsave(&priv->lock);

  /* Disable interrupts */

  priv->pcnt->ops->ioctl(priv->pcnt, SNIOC_DISABLE, 0);

  /* Make sure initial position is 0 */

  priv->position = 0;

  priv->inuse = false;

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp_position
 *
 * Description:
 *   Return the current position measurement.
 *
 * Input Parameters:
 *   lower - Pointer to Lower half driver structure
 *   pos   - Pointer to updated position measurement
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int esp_position(struct qe_lowerhalf_s *lower, int32_t *pos)
{
  DEBUGASSERT(lower && pos);

  struct qe_dev_lowerhalf_s *priv = (struct qe_dev_lowerhalf_s *)lower;
  irqstate_t flags;
  int32_t position;
  int count = 0;
  int ret = OK;

  flags = spin_lock_irqsave(&priv->lock);

  position = priv->position;
  ret = (priv->pcnt->ops->ioctl(priv->pcnt,
                                CAPIOC_PULSES,
                                (unsigned long)&count) & 0xffff);

  if (ret < 0)
    {
      snerr("ERROR: Failed to get PCNT%d count value \n", priv->pcnt_num);
    }

  /* Update the position measurement */

  *pos = position + count;

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp_setposmax
 *
 * Description:
 *   Set the maximum encoder position.
 *
 * Input Parameters:
 *   lower - Pointer to Lower half driver structure
 *   pos   - Requested maximum position
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int esp_setposmax(struct qe_lowerhalf_s *lower, uint32_t pos)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: esp_reset
 *
 * Description:
 *   Reset the position measurement to zero.
 *
 * Input Parameters:
 *   lower - Pointer to Lower half driver structure
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int esp_reset(struct qe_lowerhalf_s *lower)
{
  DEBUGASSERT(lower);

  struct qe_dev_lowerhalf_s *priv = (struct qe_dev_lowerhalf_s *)lower;
  irqstate_t flags;

  sninfo("Resetting position to zero\n");

  /* Reset this Pulse Counter Unit. */

  flags = spin_lock_irqsave(&priv->lock);

  /* Reset RST bit and clear RST bit to enable counting again */

  priv->pcnt->ops->ioctl(priv->pcnt, CAPIOC_CLR_CNT, 0);

  priv->position = 0;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: esp_setindex
 *
 * Description:
 *   Set the index pin position
 *
 * Input Parameters:
 *   lower - Pointer to Lower half driver structure
 *   pos   - Requested index position
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int esp_setindex(struct qe_lowerhalf_s *lower, uint32_t pos)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: esp_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   lower   - Pointer to Lower half driver structure
 *   cmd     - Ioctl command
 *   arg     - command argument
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int esp_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  /* No ioctl commands supported */

  /* TODO add an IOCTL to control the encoder pulse count prescaler */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface. This function must be
 *   called from board-specific logic.
 *
 * Input Parameters:
 *   devpath  - The full path to the driver to register. E.g., "/dev/qe0"
 *   pcnt     - Pointer to the pcnt driver struct
 *   pcnt_num - The PCNT number to used.  'pcnt' must be an element of
 *              {0,1,2,3}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int esp_qeinitialize(const char *devpath, struct cap_lowerhalf_s *pcnt,
                     int pcnt_num)
{
  struct qe_dev_lowerhalf_s *priv;
  int ret;

  /* Find the pre-allocated timer state structure corresponding to this
   * timer
   */

  switch (pcnt_num)
    {
#ifdef CONFIG_ESP_PCNT_U0_QE
    case 0:
      priv = &g_pcnt0lower;
      break;
#endif
#ifdef CONFIG_ESP_PCNT_U1_QE
    case 1:
      priv = &g_pcnt1lower;
      break;
#endif
#ifdef CONFIG_ESP_PCNT_U2_QE
    case 2:
      priv = &g_pcnt2lower;
      break;
#endif
#ifdef CONFIG_ESP_PCNT_U3_QE
    case 3:
      priv = &g_pcnt3lower;
      break;
#endif
    default:
      priv = NULL;
    }

  if (priv == NULL)
    {
      snerr("ERROR: PCNT%d support not configured\n", pcnt_num);
      return -ENXIO;
    }

  /* Register the upper-half driver */

  priv->pcnt = pcnt;
  ret = qe_register(devpath, (struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
      return ret;
    }

  priv->pcnt_num = pcnt_num;

  return OK;
}

#endif /* CONFIG_SENSORS_QENCODER */
