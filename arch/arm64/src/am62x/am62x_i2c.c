/****************************************************************************
 * arch/arm64/src/am62x/am62x_i2c.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "hardware/am62x_i2c.h"
#include "hardware/am62x_memorymap.h"
#include "am62x_i2c.h"
#include "am62x_tisci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM62X_I2C_INPUT_CLOCK  96000000
#define AM62X_I2C_INTERNAL_CLK 12000000

#ifndef CONFIG_AM62X_I2CTIMEOMS
#  define CONFIG_AM62X_I2CTIMEOMS 500
#endif

#ifndef CONFIG_AM62X_I2CTIMEOTICKS
#  define CONFIG_AM62X_I2CTIMEOTICKS MSEC2TICK(CONFIG_AM62X_I2CTIMEOMS)
#endif

#define AM62X_I2C_INTERRUPTS   (I2C_IRQ_AL | I2C_IRQ_NACK | I2C_IRQ_ARDY | \
                                I2C_IRQ_RRDY | I2C_IRQ_XRDY | \
                                I2C_IRQ_AERR | I2C_IRQ_XUDF | I2C_IRQ_ROVR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum am62x_i2c_state_e
{
  INTSTATE_IDLE = 0,
  INTSTATE_WAITING,
  INTSTATE_DONE,
};

struct am62x_i2c_config_s
{
  uintptr_t base;
  int irq;
  uint32_t devid;     /* TISCI device id (power/clock domain)              */
  uint32_t clkid;     /* TISCI functional clock id                         */
};

struct am62x_i2c_priv_s
{
  const struct i2c_ops_s *ops;
  const struct am62x_i2c_config_s *config;
  int refs;
  mutex_t lock;
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;
#endif
  volatile uint8_t intstate;
  struct i2c_msg_s *msgv;
  int msgc;
  int msgidx;
  uint8_t *ptr;
  int dcnt;
  uint16_t flags;
  uint32_t frequency;
  uint32_t status;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int am62x_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int am62x_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2c_ops =
{
  .transfer = am62x_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = am62x_i2c_reset,
#endif
};

#ifdef CONFIG_AM62X_I2C0
static const struct am62x_i2c_config_s g_i2c0_config =
{
  .base = AM62X_I2C0_BASE,
  .irq = AM62X_IRQ_I2C0,
  .devid = AM62X_DEV_I2C0,
  .clkid = 2,
};

static struct am62x_i2c_priv_s g_i2c0_priv =
{
  .ops = &g_i2c_ops,
  .config = &g_i2c0_config,
  .lock = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_AM62X_I2C1
static const struct am62x_i2c_config_s g_i2c1_config =
{
  .base = AM62X_I2C1_BASE,
  .irq = AM62X_IRQ_I2C1,
  .devid = AM62X_DEV_I2C1,
  .clkid = 2,
};

static struct am62x_i2c_priv_s g_i2c1_priv =
{
  .ops = &g_i2c_ops,
  .config = &g_i2c1_config,
  .lock = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_AM62X_I2C2
static const struct am62x_i2c_config_s g_i2c2_config =
{
  .base = AM62X_I2C2_BASE,
  .irq = AM62X_IRQ_I2C2,
  .devid = AM62X_DEV_I2C2,
  .clkid = 2,
};

static struct am62x_i2c_priv_s g_i2c2_priv =
{
  .ops = &g_i2c_ops,
  .config = &g_i2c2_config,
  .lock = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr = SEM_INITIALIZER(0),
#endif
};
#endif

#ifdef CONFIG_AM62X_I2C3
static const struct am62x_i2c_config_s g_i2c3_config =
{
  .base = AM62X_I2C3_BASE,
  .irq = AM62X_IRQ_I2C3,
  .devid = AM62X_DEV_I2C3,
  .clkid = 2,
};

static struct am62x_i2c_priv_s g_i2c3_priv =
{
  .ops = &g_i2c_ops,
  .config = &g_i2c3_config,
  .lock = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr = SEM_INITIALIZER(0),
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t am62x_i2c_getreg(struct am62x_i2c_priv_s *priv,
                                        uint16_t offset)
{
  return getreg32(priv->config->base + offset);
}

static inline void am62x_i2c_putreg(struct am62x_i2c_priv_s *priv,
                                    uint16_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

static inline void am62x_i2c_modifyreg(struct am62x_i2c_priv_s *priv,
                                       uint16_t offset, uint32_t clearbits,
                                       uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

static bool am62x_i2c_wait_bus_free(struct am62x_i2c_priv_s *priv)
{
  clock_t start = clock_systime_ticks();

  do
    {
      if ((am62x_i2c_getreg(priv, AM62X_I2C_IRQSTATUS_RAW_OFFSET) &
           I2C_IRQ_BB) == 0)
        {
          return true;
        }
    }
  while ((clock_systime_ticks() - start) < CONFIG_AM62X_I2CTIMEOTICKS);

  return false;
}

static void am62x_i2c_setclock(struct am62x_i2c_priv_s *priv,
                               uint32_t frequency)
{
  uint32_t internal;
  uint32_t psc;
  uint32_t divider;
  uint32_t scll;
  uint32_t sclh;
  uint32_t enabled;

  if (frequency == 0)
    {
      frequency = 100000;
    }

  if (frequency == priv->frequency)
    {
      return;
    }

  enabled = am62x_i2c_getreg(priv, AM62X_I2C_CON_OFFSET) & I2C_CON_EN;
  if (enabled != 0)
    {
      am62x_i2c_modifyreg(priv, AM62X_I2C_CON_OFFSET, I2C_CON_EN, 0);
    }

  psc = (AM62X_I2C_INPUT_CLOCK / AM62X_I2C_INTERNAL_CLK) - 1;
  internal = AM62X_I2C_INPUT_CLOCK / (psc + 1);
  divider = internal / frequency;

  if (divider < 14)
    {
      divider = 14;
    }

  scll = (divider / 2) - 7;
  sclh = (divider - (divider / 2)) - 5;

  if (scll > 255)
    {
      scll = 255;
    }

  if (sclh > 255)
    {
      sclh = 255;
    }

  am62x_i2c_putreg(priv, AM62X_I2C_PSC_OFFSET, psc);
  am62x_i2c_putreg(priv, AM62X_I2C_SCLL_OFFSET, scll);
  am62x_i2c_putreg(priv, AM62X_I2C_SCLH_OFFSET, sclh);

  if (enabled != 0)
    {
      am62x_i2c_modifyreg(priv, AM62X_I2C_CON_OFFSET, 0, I2C_CON_EN);
    }

  priv->frequency = frequency;
}

static void am62x_i2c_start_message(struct am62x_i2c_priv_s *priv)
{
  struct i2c_msg_s *msg = &priv->msgv[priv->msgidx];
  uint32_t con = I2C_CON_EN | I2C_CON_MST | I2C_CON_STT;
  bool last = (priv->msgidx + 1) >= priv->msgc;

  priv->ptr = msg->buffer;
  priv->dcnt = msg->length;
  priv->flags = msg->flags;

  if ((msg->flags & I2C_M_READ) == 0)
    {
      con |= I2C_CON_TRX;
    }

  if ((msg->flags & I2C_M_TEN) != 0)
    {
      con |= I2C_CON_XSA;
    }

  if (last && (msg->flags & I2C_M_NOSTOP) == 0)
    {
      con |= I2C_CON_STP;
    }

  am62x_i2c_putreg(priv, AM62X_I2C_SA_OFFSET, msg->addr);
  am62x_i2c_putreg(priv, AM62X_I2C_CNT_OFFSET, msg->length);
  am62x_i2c_putreg(priv, AM62X_I2C_CON_OFFSET, con);
}

static void am62x_i2c_complete(struct am62x_i2c_priv_s *priv)
{
#ifndef CONFIG_I2C_POLLED
  am62x_i2c_putreg(priv, AM62X_I2C_IRQENABLE_CLR_OFFSET,
                   I2C_IRQ_CLEARMASK);
  if (priv->intstate == INTSTATE_WAITING)
    {
      priv->intstate = INTSTATE_DONE;
      nxsem_post(&priv->sem_isr);
    }
#else
  priv->intstate = INTSTATE_DONE;
#endif
}

static int am62x_i2c_isr_process(struct am62x_i2c_priv_s *priv)
{
#ifndef CONFIG_I2C_POLLED
  uint32_t status = am62x_i2c_getreg(priv, AM62X_I2C_IRQSTATUS_OFFSET);
#else
  uint32_t status = am62x_i2c_getreg(priv, AM62X_I2C_IRQSTATUS_RAW_OFFSET);
#endif

  if (status == 0)
    {
      return OK;
    }

  priv->status = status;

  if ((status & I2C_IRQ_ERRORMASK) != 0)
    {
      am62x_i2c_modifyreg(priv, AM62X_I2C_CON_OFFSET, 0, I2C_CON_STP);
      am62x_i2c_complete(priv);
      am62x_i2c_putreg(priv, AM62X_I2C_IRQSTATUS_OFFSET, status);
      return OK;
    }

  if ((status & I2C_IRQ_XRDY) != 0 && (priv->flags & I2C_M_READ) == 0)
    {
      if (priv->dcnt > 0)
        {
          am62x_i2c_putreg(priv, AM62X_I2C_DATA_OFFSET, *priv->ptr++);
          priv->dcnt--;
        }
    }

  if ((status & I2C_IRQ_RRDY) != 0 && (priv->flags & I2C_M_READ) != 0)
    {
      if (priv->dcnt > 0)
        {
          *priv->ptr++ = am62x_i2c_getreg(priv, AM62X_I2C_DATA_OFFSET) &
                         I2C_DATA_MASK;
          priv->dcnt--;
        }
      else
        {
          (void)am62x_i2c_getreg(priv, AM62X_I2C_DATA_OFFSET);
        }
    }

  if ((status & I2C_IRQ_ARDY) != 0)
    {
      if (priv->msgidx + 1 < priv->msgc)
        {
          priv->msgidx++;
          am62x_i2c_start_message(priv);
        }
      else
        {
          am62x_i2c_complete(priv);
        }
    }

  am62x_i2c_putreg(priv, AM62X_I2C_IRQSTATUS_OFFSET, status);
  return OK;
}

#ifndef CONFIG_I2C_POLLED
static int am62x_i2c_isr(int irq, void *context, void *arg)
{
  struct am62x_i2c_priv_s *priv = arg;

  DEBUGASSERT(priv != NULL);
  return am62x_i2c_isr_process(priv);
}
#endif

static int am62x_i2c_waitdone(struct am62x_i2c_priv_s *priv)
{
#ifndef CONFIG_I2C_POLLED
  int ret;

  priv->intstate = INTSTATE_WAITING;
  am62x_i2c_putreg(priv, AM62X_I2C_IRQENABLE_SET_OFFSET,
                   AM62X_I2C_INTERRUPTS);
  am62x_i2c_start_message(priv);

  do
    {
      ret = nxsem_tickwait(&priv->sem_isr, CONFIG_AM62X_I2CTIMEOTICKS);
    }
  while (ret == -EINTR && priv->intstate != INTSTATE_DONE);

  if (ret < 0)
    {
      am62x_i2c_putreg(priv, AM62X_I2C_IRQENABLE_CLR_OFFSET,
                       I2C_IRQ_CLEARMASK);
      am62x_i2c_modifyreg(priv, AM62X_I2C_CON_OFFSET, 0, I2C_CON_STP);
      priv->intstate = INTSTATE_IDLE;
      return ret;
    }

  priv->intstate = INTSTATE_IDLE;
  return OK;
#else
  clock_t start = clock_systime_ticks();

  priv->intstate = INTSTATE_WAITING;
  am62x_i2c_start_message(priv);

  while (priv->intstate != INTSTATE_DONE &&
         (clock_systime_ticks() - start) < CONFIG_AM62X_I2CTIMEOTICKS)
    {
      am62x_i2c_isr_process(priv);
    }

  if (priv->intstate != INTSTATE_DONE)
    {
      priv->intstate = INTSTATE_IDLE;
      return -ETIMEDOUT;
    }

  priv->intstate = INTSTATE_IDLE;
  return OK;
#endif
}

static int am62x_i2c_init(struct am62x_i2c_priv_s *priv)
{
#ifdef CONFIG_AM62X_TISCI
  /* Power, release reset, and clock I2C before any register access. */

  int pret = am62x_tisci_module_enable(priv->config->devid,
                                       priv->config->clkid);
  if (pret < 0)
    {
      return pret;
    }
#endif

  am62x_i2c_putreg(priv, AM62X_I2C_CON_OFFSET, 0);
  am62x_i2c_modifyreg(priv, AM62X_I2C_SYSC_OFFSET, I2C_SYSC_AUTOIDLE, 0);
  priv->frequency = 0;
  am62x_i2c_setclock(priv, 100000);
  am62x_i2c_putreg(priv, AM62X_I2C_IRQSTATUS_OFFSET, I2C_IRQ_CLEARMASK);
  am62x_i2c_putreg(priv, AM62X_I2C_IRQENABLE_CLR_OFFSET,
                   I2C_IRQ_CLEARMASK);

#ifndef CONFIG_I2C_POLLED
  int ret = irq_attach(priv->config->irq, am62x_i2c_isr, priv);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ARCH_IRQPRIO
  (void)up_prioritize_irq(priv->config->irq, 0);
#endif
#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
  (void)up_set_irq_type(priv->config->irq, IRQ_HIGH_LEVEL);
#endif
  up_enable_irq(priv->config->irq);
#endif

  am62x_i2c_modifyreg(priv, AM62X_I2C_CON_OFFSET, 0, I2C_CON_EN);
  am62x_i2c_modifyreg(priv, AM62X_I2C_SYSTEST_OFFSET, 0, I2C_SYSTEST_FREE);
  return OK;
}

static int am62x_i2c_deinit(struct am62x_i2c_priv_s *priv)
{
  am62x_i2c_putreg(priv, AM62X_I2C_IRQENABLE_CLR_OFFSET,
                   I2C_IRQ_CLEARMASK);
  am62x_i2c_putreg(priv, AM62X_I2C_CON_OFFSET, 0);

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irq);
  irq_detach(priv->config->irq);
#endif

  return OK;
}

static int am62x_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct am62x_i2c_priv_s *priv = (struct am62x_i2c_priv_s *)dev;
  int ret;

  DEBUGASSERT(priv != NULL && msgs != NULL && count > 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!am62x_i2c_wait_bus_free(priv))
    {
      nxmutex_unlock(&priv->lock);
      return -EBUSY;
    }

  am62x_i2c_putreg(priv, AM62X_I2C_IRQSTATUS_OFFSET, I2C_IRQ_CLEARMASK);
  am62x_i2c_setclock(priv, msgs[0].frequency);

  priv->msgv = msgs;
  priv->msgc = count;
  priv->msgidx = 0;
  priv->ptr = NULL;
  priv->dcnt = 0;
  priv->flags = msgs[0].flags;
  priv->status = 0;

  ret = am62x_i2c_waitdone(priv);
  if (ret < 0)
    {
      ret = -ETIMEDOUT;
    }
  else if ((priv->status & I2C_IRQ_AL) != 0)
    {
      ret = -EAGAIN;
    }
  else if ((priv->status & I2C_IRQ_NACK) != 0)
    {
      ret = -ENXIO;
    }
  else if ((priv->status & (I2C_IRQ_AERR | I2C_IRQ_XUDF |
                            I2C_IRQ_ROVR)) != 0)
    {
      ret = -EIO;
    }
  else
    {
      ret = OK;
    }

  priv->ptr = NULL;
  priv->dcnt = 0;
  nxmutex_unlock(&priv->lock);
  return ret;
}

#ifdef CONFIG_I2C_RESET
static int am62x_i2c_reset(struct i2c_master_s *dev)
{
  struct am62x_i2c_priv_s *priv = (struct am62x_i2c_priv_s *)dev;

  DEBUGASSERT(priv != NULL);
  return am62x_i2c_init(priv);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct i2c_master_s *am62x_i2cbus_initialize(int port)
{
  struct am62x_i2c_priv_s *priv = NULL;

  switch (port)
    {
#ifdef CONFIG_AM62X_I2C0
      case 0:
        priv = &g_i2c0_priv;
        break;
#endif
#ifdef CONFIG_AM62X_I2C1
      case 1:
        priv = &g_i2c1_priv;
        break;
#endif
#ifdef CONFIG_AM62X_I2C2
      case 2:
        priv = &g_i2c2_priv;
        break;
#endif
#ifdef CONFIG_AM62X_I2C3
      case 3:
        priv = &g_i2c3_priv;
        break;
#endif
      default:
        return NULL;
    }

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      if (am62x_i2c_init(priv) < 0)
        {
          priv->refs--;
          nxmutex_unlock(&priv->lock);
          return NULL;
        }
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)priv;
}

int am62x_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct am62x_i2c_priv_s *priv = (struct am62x_i2c_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  nxmutex_lock(&priv->lock);
  if (priv->refs == 0)
    {
      nxmutex_unlock(&priv->lock);
      return -EINVAL;
    }

  if (--priv->refs == 0)
    {
      am62x_i2c_deinit(priv);
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

