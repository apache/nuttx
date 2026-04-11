/****************************************************************************
 * arch/arm/src/nrf52/nrf52_qdec.c
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

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/sensors/qencoder.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "nrf52_gpio.h"
#include "nrf52_qdec.h"

#include "hardware/nrf52_qdec.h"
#include "hardware/nrf52_utils.h"

#ifdef CONFIG_NRF52_QENCODER_INDEX
#  include <nuttx/irq.h>
#  include "nrf52_gpiote.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_qdec_s
{
  const struct qe_ops_s         *ops;
  const struct nrf52_qeconfig_s *config;
  uint32_t                       base;
  uint32_t                       pin_a;
  uint32_t                       pin_b;
  int32_t                        position;
  uint32_t                       posmax;
  bool                           inuse;
  bool                           posmax_enabled;
#ifdef CONFIG_NRF52_QENCODER_INDEX
  uint16_t                       index_count;
  uint32_t                       pin_index;
  uint32_t                       index_pos;
#endif
  spinlock_t                     lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void nrf52_qdec_putreg(struct nrf52_qdec_s *priv,
                                     uint32_t offset, uint32_t value);
static inline uint32_t nrf52_qdec_getreg(struct nrf52_qdec_s *priv,
                                         uint32_t offset);

static int nrf52_qdec_setup(struct qe_lowerhalf_s *lower);
static int nrf52_qdec_shutdown(struct qe_lowerhalf_s *lower);
static int nrf52_qdec_position(struct qe_lowerhalf_s *lower, int32_t *pos);
static int nrf52_qdec_setposmax(struct qe_lowerhalf_s *lower, uint32_t pos);
static int nrf52_qdec_reset(struct qe_lowerhalf_s *lower);
static int nrf52_qdec_setindex(struct qe_lowerhalf_s *lower, uint32_t pos);
static int nrf52_qdec_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                            unsigned long arg);

#ifdef CONFIG_NRF52_QENCODER_INDEX
static int nrf52_qdec_index_interrupt(int irq, void *context, void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qe_ops_s g_nrf52_qdecops =
{
  .setup     = nrf52_qdec_setup,
  .shutdown  = nrf52_qdec_shutdown,
  .position  = nrf52_qdec_position,
  .setposmax = nrf52_qdec_setposmax,
  .reset     = nrf52_qdec_reset,
  .setindex  = nrf52_qdec_setindex,
  .ioctl     = nrf52_qdec_ioctl,
};

#ifdef CONFIG_NRF52_QDEC0
struct nrf52_qdec_s g_nrf52_qdec0 =
{
  .ops       = &g_nrf52_qdecops,
  .base      = NRF52_QDEC_BASE,
  .pin_a     = BOARD_QDEC0_A_PIN,
  .pin_b     = BOARD_QDEC0_B_PIN,
#ifdef CONFIG_NRF52_QENCODER_INDEX
  .pin_index = BOARD_QDEC0_INDEX_PIN,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_qdec_putreg
 ****************************************************************************/

static inline void nrf52_qdec_putreg(struct nrf52_qdec_s *priv,
                                     uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_qdec_getreg
 ****************************************************************************/

static inline uint32_t nrf52_qdec_getreg(struct nrf52_qdec_s *priv,
                                         uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_qdec_setup
 ****************************************************************************/

static int nrf52_qdec_setup(struct qe_lowerhalf_s *lower)
{
  struct nrf52_qdec_s *priv = (struct nrf52_qdec_s *)lower;
  uint32_t pin;
  uint32_t port;
  uint32_t regval;
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&priv->lock);

  if (priv->inuse)
    {
      ret = -EBUSY;
      goto errout;
    }

  nrf52_gpio_config(priv->pin_a);
  nrf52_gpio_config(priv->pin_b);

  /* Configure PSEL A */

  pin  = GPIO_PIN_DECODE(priv->pin_a);
  port = GPIO_PORT_DECODE(priv->pin_a);
  regval = (port << QDEC_PSEL_PORT_SHIFT);
  regval |= (pin << QDEC_PSEL_PIN_SHIFT);
  nrf52_qdec_putreg(priv, NRF52_QDEC_PSEL_A_OFFSET, regval);

  /* Configure PSEL B */

  pin  = GPIO_PIN_DECODE(priv->pin_b);
  port = GPIO_PORT_DECODE(priv->pin_b);
  regval = (port << QDEC_PSEL_PORT_SHIFT);
  regval |= (pin << QDEC_PSEL_PIN_SHIFT);
  nrf52_qdec_putreg(priv, NRF52_QDEC_PSEL_B_OFFSET, regval);

  /* Disable LED */

  nrf52_qdec_putreg(priv, NRF52_QDEC_PSEL_LED_OFFSET,
                    QDEC_PSEL_DISCONNECTED);

  /* Configure decoder */

  nrf52_qdec_putreg(priv, NRF52_QDEC_SAMPLEPER_OFFSET,
                    priv->config->sample_period & QDEC_SAMPLEPER_MASK);
  nrf52_qdec_putreg(priv, NRF52_QDEC_REPORTPER_OFFSET,
                    priv->config->report_period & QDEC_REPORTPER_MASK);

  nrf52_qdec_putreg(priv, NRF52_QDEC_DBFEN_OFFSET,
                    priv->config->enable_debounce ? QDEC_DBFEN_ENABLE :
                    QDEC_DBFEN_DISABLE);

  /* Start decoder */

  nrf52_qdec_putreg(priv, NRF52_QDEC_ENABLE_OFFSET, QDEC_ENABLE_ENABLE);
  nrf52_qdec_putreg(priv, NRF52_QDEC_TASKS_START_OFFSET, QDEC_TASKS_START);

#ifdef CONFIG_NRF52_QENCODER_INDEX
  /* Configure Index pin */

  nrf52_gpio_config(priv->pin_index);
  ret = nrf52_gpiote_set_event(priv->pin_index, true, false,
                                nrf52_qdec_index_interrupt, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure index pin interrupt: %d\n", ret);
      goto errout;
    }
#endif

  priv->inuse          = true;
  priv->position       = 0;
  priv->posmax_enabled = false;
#ifdef CONFIG_NRF52_QENCODER_INDEX
  priv->index_pos      = 0;
  priv->index_count    = 0;
#endif
  ret                  = OK;

errout:
  spin_unlock_irqrestore(&priv->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: nrf52_qdec_shutdown
 ****************************************************************************/

static int nrf52_qdec_shutdown(struct qe_lowerhalf_s *lower)
{
  struct nrf52_qdec_s *priv = (struct nrf52_qdec_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  nrf52_qdec_putreg(priv, NRF52_QDEC_TASKS_STOP_OFFSET, QDEC_TASKS_STOP);
  nrf52_qdec_putreg(priv, NRF52_QDEC_ENABLE_OFFSET, QDEC_ENABLE_DISABLE);

  nrf52_gpio_unconfig(priv->pin_a);
  nrf52_gpio_unconfig(priv->pin_b);

#ifdef CONFIG_NRF52_QENCODER_INDEX
  nrf52_gpiote_set_event(priv->pin_index, false, false, NULL, NULL);
  nrf52_gpio_unconfig(priv->pin_index);
#endif

  priv->inuse = false;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_qdec_position
 ****************************************************************************/

static int nrf52_qdec_position(struct qe_lowerhalf_s *lower, int32_t *pos)
{
  struct nrf52_qdec_s *priv = (struct nrf52_qdec_s *)lower;
  irqstate_t flags;
  int32_t accread;

  DEBUGASSERT(lower && pos);

  flags = spin_lock_irqsave(&priv->lock);

  nrf52_qdec_putreg(priv, NRF52_QDEC_TASKS_RDCLRACC_OFFSET,
                    QDEC_TASKS_RDCLRACC);

  accread = (int32_t)nrf52_qdec_getreg(priv, NRF52_QDEC_ACCREAD_OFFSET);

  priv->position += accread;

  if (priv->posmax_enabled && priv->posmax > 0)
    {
      priv->position = priv->position % (int32_t)priv->posmax;
      if (priv->position < 0)
        {
          priv->position += (int32_t)priv->posmax;
        }
    }

  *pos = priv->position;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_qdec_setposmax
 ****************************************************************************/

static int nrf52_qdec_setposmax(struct qe_lowerhalf_s *lower, uint32_t pos)
{
  struct nrf52_qdec_s *priv = (struct nrf52_qdec_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  priv->posmax = pos;
  priv->posmax_enabled = (pos > 0);

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_qdec_reset
 ****************************************************************************/

static int nrf52_qdec_reset(struct qe_lowerhalf_s *lower)
{
  struct nrf52_qdec_s *priv = (struct nrf52_qdec_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  nrf52_qdec_putreg(priv, NRF52_QDEC_TASKS_READCLRACC_OFFSET,
                    QDEC_TASKS_READCLRACC);

  priv->position = 0;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_qdec_setindex
 ****************************************************************************/

static int nrf52_qdec_setindex(struct qe_lowerhalf_s *lower, uint32_t pos)
{
#ifdef CONFIG_NRF52_QENCODER_INDEX
  struct nrf52_qdec_s *priv = (struct nrf52_qdec_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->index_pos = pos;
  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
#else
  return -ENOTTY;
#endif
}

#ifdef CONFIG_NRF52_QENCODER_INDEX

/****************************************************************************
 * Name: nrf52_qdec_index_interrupt
 ****************************************************************************/

static int nrf52_qdec_index_interrupt(int irq, void *context, void *arg)
{
  struct nrf52_qdec_s *priv = (struct nrf52_qdec_s *)arg;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  priv->position = priv->index_pos;
  priv->index_count++;

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

#endif

/****************************************************************************
 * Name: nrf52_qdec_ioctl
 ****************************************************************************/

static int nrf52_qdec_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                            unsigned long arg)
{
#ifdef CONFIG_NRF52_QENCODER_INDEX
  struct nrf52_qdec_s *priv = (struct nrf52_qdec_s *)lower;
  struct qe_index_s *index;

  if (cmd == QEIOC_GETINDEX)
    {
      index = (struct qe_index_s *)arg;
      index->qenc_pos = priv->position;
      index->indx_pos = priv->index_pos;
      index->indx_cnt = priv->index_count;
      return OK;
    }
#endif

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_qeinitialize
 ****************************************************************************/

struct qe_lowerhalf_s *
nrf52_qeinitialize(int qdec, const struct nrf52_qeconfig_s *config)
{
  struct nrf52_qdec_s *priv = NULL;

  DEBUGASSERT(config != NULL);

  switch (qdec)
    {
#ifdef CONFIG_NRF52_QDEC0
      case 0:
        {
          priv = &g_nrf52_qdec0;
          break;
        }
#endif

      default:
        {
          snerr("ERROR: Invalid QDEC instance: %d\n", qdec);
          return NULL;
        }
    }

  priv->config = config;

  return (struct qe_lowerhalf_s *)priv;
}
