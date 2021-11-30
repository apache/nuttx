/****************************************************************************
 * arch/arm/src/stm32h7/stm32_lptim.c
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "arm_arch.h"

#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_lptim.h"

#if defined(CONFIG_STM32H7_LPTIM1)  || defined(CONFIG_STM32H7_LPTIM2)  || \
    defined(CONFIG_STM32H7_LPTIM3)  || defined(CONFIG_STM32H7_LPTIM4)  || \
    defined(CONFIG_STM32H7_LPTIM5)

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* Low-power timer methods */

static void stm32_lptim_setcfgr(FAR struct stm32_lptim_dev_s *dev,
                                uint32_t regval);
static void stm32_lptim_setperiod(FAR struct stm32_lptim_dev_s *dev,
                                uint32_t period);
static void stm32_lptim_setcompare(FAR struct stm32_lptim_dev_s *dev,
                                uint32_t cmp);
static uint32_t stm32_lptim_getcounter(FAR struct stm32_lptim_dev_s *dev);
static int  stm32_lptim_setinput(FAR struct stm32_lptim_dev_s *dev,
                                uint32_t input, uint32_t mux);
static void stm32_lptim_enable(FAR struct stm32_lptim_dev_s *dev, bool on);
static void stm32_lptim_start(FAR struct stm32_lptim_dev_s *dev,
                                 stm32_lptim_start_mode_t mode);
static void stm32_lptim_resetcounter(FAR struct stm32_lptim_dev_s *dev);
static void stm32_lptim_enablerar(FAR struct stm32_lptim_dev_s *dev,
                                  bool on);

static int  stm32_lptim_setisr(FAR struct stm32_lptim_dev_s *dev,
                               xcpt_t handler, void *arg, int source);
static void stm32_lptim_enableint(FAR struct stm32_lptim_dev_s *dev,
                                  int source);
static void stm32_lptim_disableint(FAR struct stm32_lptim_dev_s *dev,
                                 int source);
static void stm32_lptim_ackint(FAR struct stm32_lptim_dev_s *dev,
                               int source);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stm32_lptim_ops_s stm32_lptim_ops =
{
  .setcfgr         = &stm32_lptim_setcfgr,
  .setperiod       = &stm32_lptim_setperiod,
  .setcompare      = &stm32_lptim_setcompare,
  .getcounter      = &stm32_lptim_getcounter,
  .setinput        = &stm32_lptim_setinput,
  .enable          = &stm32_lptim_enable,
  .start           = &stm32_lptim_start,
  .resetcounter    = &stm32_lptim_resetcounter,
  .enablerar       = &stm32_lptim_enablerar,
  .setisr          = &stm32_lptim_setisr,
  .enableint       = &stm32_lptim_enableint,
  .disableint      = &stm32_lptim_disableint,
  .ackint          = &stm32_lptim_ackint
};

#ifdef CONFIG_STM32H7_LPTIM1
struct stm32_lptim_dev_s stm32_lptim1_priv =
{
  .ops        = &stm32_lptim_ops,
  .base       = STM32_LPTIM1_BASE,
};
#endif
#ifdef CONFIG_STM32H7_LPTIM2
struct stm32_lptim_dev_s stm32_lptim2_priv =
{
  .ops        = &stm32_lptim_ops,
  .base       = STM32_LPTIM2_BASE,
};
#endif
#ifdef CONFIG_STM32H7_LPTIM3
struct stm32_lptim_dev_s stm32_lptim3_priv =
{
  .ops        = &stm32_lptim_ops,
  .base       = STM32_LPTIM3_BASE,
};
#endif
#ifdef CONFIG_STM32H7_LPTIM4
struct stm32_lptim_dev_s stm32_lptim4_priv =
{
  .ops        = &stm32_lptim_ops,
  .base       = STM32_LPTIM4_BASE,
};
#endif
#ifdef CONFIG_STM32H7_LPTIM5
struct stm32_lptim_dev_s stm32_lptim5_priv =
{
  .ops        = &stm32_lptim_ops,
  .base       = STM32_LPTIM5_BASE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Get a 32-bit register value by offset.
 */

static inline uint32_t stm32_getreg32(FAR struct stm32_lptim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg32(dev->base + offset);
}

/* Put a 32-bit register value by offset.
 */

static inline void stm32_putreg32(FAR struct stm32_lptim_dev_s *dev,
                                  uint8_t offset, uint32_t value)
{
  putreg32(value, dev->base + offset);
}

static void stm32_lptim_setcfgr(FAR struct stm32_lptim_dev_s *dev,
                                uint32_t regval)
{
  stm32_putreg32(dev, STM32_LPTIM_CFGR_OFFSET, regval);
}

static void stm32_lptim_setperiod(FAR struct stm32_lptim_dev_s *dev,
                                  uint32_t period)
{
  stm32_putreg32(dev, STM32_LPTIM_ARR_OFFSET, period);
}

static void stm32_lptim_setcompare(FAR struct stm32_lptim_dev_s *dev,
                                   uint32_t cmp)
{
  stm32_putreg32(dev, STM32_LPTIM_CMP_OFFSET, cmp);
}

static uint32_t stm32_lptim_getcounter(FAR struct stm32_lptim_dev_s *dev)
{
  /* For a reliable LPTIM_CNT register read access, two consecutive
   *  read accesses must be performed and compared
   */

  uint32_t cnt1;
  uint32_t cnt2;

  cnt2 = stm32_getreg32(dev, STM32_LPTIM_CNT_OFFSET);

  do
    {
      cnt1 = cnt2;
      cnt2 = stm32_getreg32(dev, STM32_LPTIM_CNT_OFFSET);
    }
  while (cnt1 != cnt2);

  return cnt1;
}

static int stm32_lptim_setinput(FAR struct stm32_lptim_dev_s *dev,
                                uint32_t input, uint32_t mux)
{
  switch (dev->base)
    {
#if defined(CONFIG_STM32H7_LPTIM1)  || defined(CONFIG_STM32H7_LPTIM2)
      case STM32_LPTIM1_BASE:
      case STM32_LPTIM2_BASE:
        if (input == 0)
          {
            modifyreg32(dev->base + STM32_LPTIM_CFGR2_OFFSET,
                        LPTIM_CFGR2_IN1SEL_MASK, mux);
          }
        else
          {
            modifyreg32(dev->base + STM32_LPTIM_CFGR2_OFFSET,
                        LPTIM_CFGR2_IN2SEL_MASK,
                        mux << LPTIM_CFGR2_IN2SEL_SHIFT);
          }
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM3
      case STM32_LPTIM3_BASE:
        modifyreg32(dev->base + STM32_LPTIM_CFGR2_OFFSET,
                    LPTIM_CFGR2_IN1SEL_MASK, mux);
       break;
#endif
      default:
        return -EINVAL;
  }

  return OK;
}

static void stm32_lptim_enable(FAR struct stm32_lptim_dev_s *dev, bool on)
{
  uint32_t val = stm32_getreg32(dev, STM32_LPTIM_CR_OFFSET);

  if (on)
    {
      val |= LPTIM_CR_ENABLE;
    }
  else
    {
      val &= ~LPTIM_CR_ENABLE;
    }

  stm32_putreg32(dev, STM32_LPTIM_CR_OFFSET, val);
}

static void stm32_lptim_start(FAR struct stm32_lptim_dev_s *dev,
                              stm32_lptim_start_mode_t mode)
{
  uint32_t val = stm32_getreg32(dev, STM32_LPTIM_CR_OFFSET);

  switch (mode)
    {
      case STM32_LPTIM_MODE_CONTINUOUS:
        val |= LPTIM_CR_CNTSTRT;
        break;
      case STM32_LPTIM_MODE_SINGLE:
        val |= LPTIM_CR_SNGSTRT;
        break;
    }

  stm32_putreg32(dev, STM32_LPTIM_CR_OFFSET, val);
}

static void stm32_lptim_resetcounter(FAR struct stm32_lptim_dev_s *dev)
{
  uint32_t val = stm32_getreg32(dev, STM32_LPTIM_CR_OFFSET);
  val |= LPTIM_CR_COUNTRST;
  stm32_putreg32(dev, STM32_LPTIM_CR_OFFSET, val);
}

static void stm32_lptim_enablerar(FAR struct stm32_lptim_dev_s *dev, bool on)
{
  uint32_t val = stm32_getreg32(dev, STM32_LPTIM_CR_OFFSET);

  if (on)
    {
      val |= LPTIM_CR_RSTARE;
    }
  else
    {
      val &= ~LPTIM_CR_RSTARE;
    }

  stm32_putreg32(dev, STM32_LPTIM_CR_OFFSET, val);
}

static int stm32_lptim_setisr(FAR struct stm32_lptim_dev_s *dev,
                              xcpt_t handler, void *arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (dev->base)
    {
#ifdef CONFIG_STM32H7_LPTIM1
      case STM32_LPTIM1_BASE:
        vectorno = STM32_IRQ_LPTIM1;
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM2
      case STM32_LPTIM2_BASE:
        vectorno = STM32_IRQ_LPTIM2;
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM3
      case STM32_LPTIM3_BASE:
        vectorno = STM32_IRQ_LPTIM3;
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM4
      case STM32_LPTIM4_BASE:
        vectorno = STM32_IRQ_LPTIM4;
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM5
      case STM32_LPTIM5_BASE:
        vectorno = STM32_IRQ_LPTIM5;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(vectorno);
      irq_detach(vectorno);
      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(vectorno, handler, arg);
  up_enable_irq(vectorno);

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the interrupt priority */

  up_prioritize_irq(vectorno, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

  return OK;
}

static void stm32_lptim_enableint(FAR struct stm32_lptim_dev_s *dev,
                                  int source)
{
  DEBUGASSERT(dev != NULL);

  uint32_t val = stm32_getreg32(dev, STM32_LPTIM_IER_OFFSET);
  val |= source;
  stm32_putreg32(dev, STM32_LPTIM_IER_OFFSET, val);
}

static void stm32_lptim_disableint(FAR struct stm32_lptim_dev_s *dev,
                                   int source)
{
  DEBUGASSERT(dev != NULL);

  uint32_t val = stm32_getreg32(dev, STM32_LPTIM_IER_OFFSET);
  val &= ~source;
  stm32_putreg32(dev, STM32_LPTIM_IER_OFFSET, val);
}

static void stm32_lptim_ackint(FAR struct stm32_lptim_dev_s *dev, int source)
{
  stm32_putreg32(dev, STM32_LPTIM_ICR_OFFSET, source);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct stm32_lptim_dev_s *stm32_lptim_init(int lptimer)
{
  struct stm32_lptim_dev_s *dev = NULL;

  /* Get structure and enable power */

  switch (lptimer)
    {
#ifdef CONFIG_STM32H7_LPTIM1
      case 1:
        dev = &stm32_lptim1_priv;
        modifyreg32(STM32_RCC_APB1LENR, 0, RCC_APB1LENR_LPTIM1EN);
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM2
      case 2:
        dev = &stm32_lptim2_priv;
        modifyreg32(STM32_RCC_APB4ENR, 0, RCC_APB4ENR_LPTIM2EN);
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM3
      case 3:
        dev = &stm32_lptim3_priv;
        modifyreg32(STM32_RCC_APB4ENR, 0, RCC_APB4ENR_LPTIM3EN);
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM4
      case 4:
        dev = &stm32_lptim4_priv;
        modifyreg32(STM32_RCC_APB4ENR, 0, RCC_APB4ENR_LPTIM4EN);
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM5
      case 5:
        dev = &stm32_lptim5_priv;
        modifyreg32(STM32_RCC_APB4ENR, 0, RCC_APB4ENR_LPTIM5EN);
        break;
#endif
      default:
        return NULL;
    }

  return dev;
}

int stm32_lptim_deinit(FAR struct stm32_lptim_dev_s * dev)
{
  DEBUGASSERT(dev != NULL);

  /* Disable power */

  switch (dev->base)
    {
#ifdef CONFIG_STM32H7_LPTIM1
      case STM32_LPTIM1_BASE:
        modifyreg32(STM32_RCC_APB1LLPENR, RCC_APB1LENR_LPTIM1EN, 0);
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM2
      case STM32_LPTIM2_BASE:
        modifyreg32(STM32_RCC_APB4LPENR, RCC_APB4ENR_LPTIM2EN, 0);
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM3
      case STM32_LPTIM3_BASE:
        modifyreg32(STM32_RCC_APB4LPENR, RCC_APB4ENR_LPTIM3EN, 0);
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM4
      case STM32_LPTIM4_BASE:
        modifyreg32(STM32_RCC_APB4LPENR, RCC_APB4ENR_LPTIM4EN, 0);
        break;
#endif
#ifdef CONFIG_STM32H7_LPTIM5
      case STM32_LPTIM5_BASE:
        modifyreg32(STM32_RCC_APB4LPENR, RCC_APB4ENR_LPTIM5EN, 0);
        break;
#endif
      default:
        return -EINVAL;
    }

  return OK;
}

#endif /* defined(CONFIG_STM32H7_LPTIM1 || ... || CONFIG_STM32H7_LPTIM5) */
