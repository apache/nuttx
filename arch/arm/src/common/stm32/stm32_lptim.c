/****************************************************************************
 * arch/arm/src/common/stm32/stm32_lptim.c
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/param.h>

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32_lptim.h"
#include "stm32_lptim_clk.h"

#if defined(CONFIG_STM32_LPTIM1) || \
    defined(CONFIG_STM32_LPTIM2) || \
    defined(CONFIG_STM32_LPTIM3) || \
    defined(CONFIG_STM32_LPTIM4) || \
    defined(CONFIG_STM32_LPTIM5) || \
    defined(CONFIG_STM32_LPTIM6)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPTIM kernel clock frequency, optionally provided by board.h.  It is
 * needed only by setclock().
 */

#ifdef STM32_LPTIM1_CLKIN
#  define LPTIM1_CLKIN STM32_LPTIM1_CLKIN
#else
#  define LPTIM1_CLKIN 0
#endif
#ifdef STM32_LPTIM2_CLKIN
#  define LPTIM2_CLKIN STM32_LPTIM2_CLKIN
#else
#  define LPTIM2_CLKIN 0
#endif
#ifdef STM32_LPTIM3_CLKIN
#  define LPTIM3_CLKIN STM32_LPTIM3_CLKIN
#else
#  define LPTIM3_CLKIN 0
#endif
#ifdef STM32_LPTIM4_CLKIN
#  define LPTIM4_CLKIN STM32_LPTIM4_CLKIN
#else
#  define LPTIM4_CLKIN 0
#endif
#ifdef STM32_LPTIM5_CLKIN
#  define LPTIM5_CLKIN STM32_LPTIM5_CLKIN
#else
#  define LPTIM5_CLKIN 0
#endif
#ifdef STM32_LPTIM6_CLKIN
#  define LPTIM6_CLKIN STM32_LPTIM6_CLKIN
#else
#  define LPTIM6_CLKIN 0
#endif

/* Output channel pins, GPIO_LPTIMn_CHmOUT from board.h (0: none) */

#define LPTIM_NCHANNELS 4

#ifdef GPIO_LPTIM1_CH1OUT
#  define LPTIM1_CH1OUT GPIO_LPTIM1_CH1OUT
#else
#  define LPTIM1_CH1OUT 0
#endif
#ifdef GPIO_LPTIM1_CH2OUT
#  define LPTIM1_CH2OUT GPIO_LPTIM1_CH2OUT
#else
#  define LPTIM1_CH2OUT 0
#endif
#ifdef GPIO_LPTIM1_CH3OUT
#  define LPTIM1_CH3OUT GPIO_LPTIM1_CH3OUT
#else
#  define LPTIM1_CH3OUT 0
#endif
#ifdef GPIO_LPTIM1_CH4OUT
#  define LPTIM1_CH4OUT GPIO_LPTIM1_CH4OUT
#else
#  define LPTIM1_CH4OUT 0
#endif
#ifdef GPIO_LPTIM2_CH1OUT
#  define LPTIM2_CH1OUT GPIO_LPTIM2_CH1OUT
#else
#  define LPTIM2_CH1OUT 0
#endif
#ifdef GPIO_LPTIM2_CH2OUT
#  define LPTIM2_CH2OUT GPIO_LPTIM2_CH2OUT
#else
#  define LPTIM2_CH2OUT 0
#endif
#ifdef GPIO_LPTIM2_CH3OUT
#  define LPTIM2_CH3OUT GPIO_LPTIM2_CH3OUT
#else
#  define LPTIM2_CH3OUT 0
#endif
#ifdef GPIO_LPTIM2_CH4OUT
#  define LPTIM2_CH4OUT GPIO_LPTIM2_CH4OUT
#else
#  define LPTIM2_CH4OUT 0
#endif
#ifdef GPIO_LPTIM3_CH1OUT
#  define LPTIM3_CH1OUT GPIO_LPTIM3_CH1OUT
#else
#  define LPTIM3_CH1OUT 0
#endif
#ifdef GPIO_LPTIM3_CH2OUT
#  define LPTIM3_CH2OUT GPIO_LPTIM3_CH2OUT
#else
#  define LPTIM3_CH2OUT 0
#endif
#ifdef GPIO_LPTIM3_CH3OUT
#  define LPTIM3_CH3OUT GPIO_LPTIM3_CH3OUT
#else
#  define LPTIM3_CH3OUT 0
#endif
#ifdef GPIO_LPTIM3_CH4OUT
#  define LPTIM3_CH4OUT GPIO_LPTIM3_CH4OUT
#else
#  define LPTIM3_CH4OUT 0
#endif
#ifdef GPIO_LPTIM4_CH1OUT
#  define LPTIM4_CH1OUT GPIO_LPTIM4_CH1OUT
#else
#  define LPTIM4_CH1OUT 0
#endif
#ifdef GPIO_LPTIM4_CH2OUT
#  define LPTIM4_CH2OUT GPIO_LPTIM4_CH2OUT
#else
#  define LPTIM4_CH2OUT 0
#endif
#ifdef GPIO_LPTIM4_CH3OUT
#  define LPTIM4_CH3OUT GPIO_LPTIM4_CH3OUT
#else
#  define LPTIM4_CH3OUT 0
#endif
#ifdef GPIO_LPTIM4_CH4OUT
#  define LPTIM4_CH4OUT GPIO_LPTIM4_CH4OUT
#else
#  define LPTIM4_CH4OUT 0
#endif
#ifdef GPIO_LPTIM5_CH1OUT
#  define LPTIM5_CH1OUT GPIO_LPTIM5_CH1OUT
#else
#  define LPTIM5_CH1OUT 0
#endif
#ifdef GPIO_LPTIM5_CH2OUT
#  define LPTIM5_CH2OUT GPIO_LPTIM5_CH2OUT
#else
#  define LPTIM5_CH2OUT 0
#endif
#ifdef GPIO_LPTIM5_CH3OUT
#  define LPTIM5_CH3OUT GPIO_LPTIM5_CH3OUT
#else
#  define LPTIM5_CH3OUT 0
#endif
#ifdef GPIO_LPTIM5_CH4OUT
#  define LPTIM5_CH4OUT GPIO_LPTIM5_CH4OUT
#else
#  define LPTIM5_CH4OUT 0
#endif
#ifdef GPIO_LPTIM6_CH1OUT
#  define LPTIM6_CH1OUT GPIO_LPTIM6_CH1OUT
#else
#  define LPTIM6_CH1OUT 0
#endif
#ifdef GPIO_LPTIM6_CH2OUT
#  define LPTIM6_CH2OUT GPIO_LPTIM6_CH2OUT
#else
#  define LPTIM6_CH2OUT 0
#endif
#ifdef GPIO_LPTIM6_CH3OUT
#  define LPTIM6_CH3OUT GPIO_LPTIM6_CH3OUT
#else
#  define LPTIM6_CH3OUT 0
#endif
#ifdef GPIO_LPTIM6_CH4OUT
#  define LPTIM6_CH4OUT GPIO_LPTIM6_CH4OUT
#else
#  define LPTIM6_CH4OUT 0
#endif

/* Largest PRESC field value (division by 128) */

#define LPTIM_PRESC_MAX 7

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lptim_priv_s
{
  const struct stm32_lptim_ops_s *ops;
  bool     inuse;                  /* Returned by stm32_lptim_init() */
  uint32_t base;                   /* LPTIMn base address */
  int      irq;                    /* LPTIMn interrupt */
  uint32_t freq;                   /* LPTIMn kernel clock frequency */
  uint32_t enreg;                  /* RCC clock enable register */
  uint32_t enbit;                  /* RCC clock enable bit */
  uint32_t rstreg;                 /* RCC reset register */
  uint32_t rstbit;                 /* RCC reset bit */
  uint32_t pins[LPTIM_NCHANNELS];  /* Output channel pin configurations */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_lptim_setmode(struct stm32_lptim_dev_s *dev,
                               stm32_lptim_mode_t mode);
static int stm32_lptim_setclock(struct stm32_lptim_dev_s *dev,
                                uint32_t freq);
static int stm32_lptim_setchannel(struct stm32_lptim_dev_s *dev,
                                  stm32_lptim_channel_t channel,
                                  int enable);
static int stm32_lptim_setclocksource(struct stm32_lptim_dev_s *dev,
                                      stm32_lptim_clksrc_t clksrc);
static int stm32_lptim_setpolarity(struct stm32_lptim_dev_s *dev,
                                   stm32_lptim_clkpol_t polarity);
static uint32_t stm32_lptim_getcounter(struct stm32_lptim_dev_s *dev);
static int stm32_lptim_setcountmode(struct stm32_lptim_dev_s *dev,
                                    stm32_lptim_cntmode_t cntmode);
static void stm32_lptim_setperiod(struct stm32_lptim_dev_s *dev,
                                  uint32_t period);
static uint32_t stm32_lptim_getperiod(struct stm32_lptim_dev_s *dev);
static void stm32_lptim_setcfgr(struct stm32_lptim_dev_s *dev,
                                uint32_t regval);
static void stm32_lptim_setcompare(struct stm32_lptim_dev_s *dev,
                                   uint32_t cmp);
static int stm32_lptim_setinput(struct stm32_lptim_dev_s *dev,
                                uint32_t input, uint32_t mux);
static void stm32_lptim_enable(struct stm32_lptim_dev_s *dev, bool on);
static void stm32_lptim_start(struct stm32_lptim_dev_s *dev,
                              stm32_lptim_mode_t mode);
static void stm32_lptim_resetcounter(struct stm32_lptim_dev_s *dev);
static void stm32_lptim_enablerar(struct stm32_lptim_dev_s *dev, bool on);
static int stm32_lptim_setisr(struct stm32_lptim_dev_s *dev,
                              xcpt_t handler, void *arg, int source);
static void stm32_lptim_enableint(struct stm32_lptim_dev_s *dev,
                                  int source);
static void stm32_lptim_disableint(struct stm32_lptim_dev_s *dev,
                                   int source);
static void stm32_lptim_ackint(struct stm32_lptim_dev_s *dev, int source);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stm32_lptim_ops_s g_lptim_ops =
{
  .setmode        = stm32_lptim_setmode,
  .setclock       = stm32_lptim_setclock,
  .setchannel     = stm32_lptim_setchannel,
  .setclocksource = stm32_lptim_setclocksource,
  .setpolarity    = stm32_lptim_setpolarity,
  .getcounter     = stm32_lptim_getcounter,
  .setcountmode   = stm32_lptim_setcountmode,
  .setperiod      = stm32_lptim_setperiod,
  .getperiod      = stm32_lptim_getperiod,
  .setcfgr        = stm32_lptim_setcfgr,
  .setcompare     = stm32_lptim_setcompare,
  .setinput       = stm32_lptim_setinput,
  .enable         = stm32_lptim_enable,
  .start          = stm32_lptim_start,
  .resetcounter   = stm32_lptim_resetcounter,
  .enablerar      = stm32_lptim_enablerar,
  .setisr         = stm32_lptim_setisr,
  .enableint      = stm32_lptim_enableint,
  .disableint     = stm32_lptim_disableint,
  .ackint         = stm32_lptim_ackint,
};

#ifdef CONFIG_STM32_LPTIM1
static struct stm32_lptim_priv_s g_lptim1 =
{
  .ops        = &g_lptim_ops,
  .base       = STM32_LPTIM1_BASE,
  .irq        = STM32_IRQ_LPTIM1,
  .freq       = LPTIM1_CLKIN,
  .enreg      = STM32_RCC_LPTIM1_EN_REG,
  .enbit      = STM32_RCC_LPTIM1_EN,
  .rstreg     = STM32_RCC_LPTIM1_RST_REG,
  .rstbit     = STM32_RCC_LPTIM1_RST,
  .pins       =
  {
    LPTIM1_CH1OUT, LPTIM1_CH2OUT, LPTIM1_CH3OUT, LPTIM1_CH4OUT
  },
};
#endif

#ifdef CONFIG_STM32_LPTIM2
static struct stm32_lptim_priv_s g_lptim2 =
{
  .ops        = &g_lptim_ops,
  .base       = STM32_LPTIM2_BASE,
  .irq        = STM32_IRQ_LPTIM2,
  .freq       = LPTIM2_CLKIN,
  .enreg      = STM32_RCC_LPTIM2_EN_REG,
  .enbit      = STM32_RCC_LPTIM2_EN,
  .rstreg     = STM32_RCC_LPTIM2_RST_REG,
  .rstbit     = STM32_RCC_LPTIM2_RST,
  .pins       =
  {
    LPTIM2_CH1OUT, LPTIM2_CH2OUT, LPTIM2_CH3OUT, LPTIM2_CH4OUT
  },
};
#endif

#ifdef CONFIG_STM32_LPTIM3
static struct stm32_lptim_priv_s g_lptim3 =
{
  .ops        = &g_lptim_ops,
  .base       = STM32_LPTIM3_BASE,
  .irq        = STM32_IRQ_LPTIM3,
  .freq       = LPTIM3_CLKIN,
  .enreg      = STM32_RCC_LPTIM3_EN_REG,
  .enbit      = STM32_RCC_LPTIM3_EN,
  .rstreg     = STM32_RCC_LPTIM3_RST_REG,
  .rstbit     = STM32_RCC_LPTIM3_RST,
  .pins       =
  {
    LPTIM3_CH1OUT, LPTIM3_CH2OUT, LPTIM3_CH3OUT, LPTIM3_CH4OUT
  },
};
#endif

#ifdef CONFIG_STM32_LPTIM4
static struct stm32_lptim_priv_s g_lptim4 =
{
  .ops        = &g_lptim_ops,
  .base       = STM32_LPTIM4_BASE,
  .irq        = STM32_IRQ_LPTIM4,
  .freq       = LPTIM4_CLKIN,
  .enreg      = STM32_RCC_LPTIM4_EN_REG,
  .enbit      = STM32_RCC_LPTIM4_EN,
  .rstreg     = STM32_RCC_LPTIM4_RST_REG,
  .rstbit     = STM32_RCC_LPTIM4_RST,
  .pins       =
  {
    LPTIM4_CH1OUT, LPTIM4_CH2OUT, LPTIM4_CH3OUT, LPTIM4_CH4OUT
  },
};
#endif

#ifdef CONFIG_STM32_LPTIM5
static struct stm32_lptim_priv_s g_lptim5 =
{
  .ops        = &g_lptim_ops,
  .base       = STM32_LPTIM5_BASE,
  .irq        = STM32_IRQ_LPTIM5,
  .freq       = LPTIM5_CLKIN,
  .enreg      = STM32_RCC_LPTIM5_EN_REG,
  .enbit      = STM32_RCC_LPTIM5_EN,
  .rstreg     = STM32_RCC_LPTIM5_RST_REG,
  .rstbit     = STM32_RCC_LPTIM5_RST,
  .pins       =
  {
    LPTIM5_CH1OUT, LPTIM5_CH2OUT, LPTIM5_CH3OUT, LPTIM5_CH4OUT
  },
};
#endif

#ifdef CONFIG_STM32_LPTIM6
static struct stm32_lptim_priv_s g_lptim6 =
{
  .ops        = &g_lptim_ops,
  .base       = STM32_LPTIM6_BASE,
  .irq        = STM32_IRQ_LPTIM6,
  .freq       = LPTIM6_CLKIN,
  .enreg      = STM32_RCC_LPTIM6_EN_REG,
  .enbit      = STM32_RCC_LPTIM6_EN,
  .rstreg     = STM32_RCC_LPTIM6_RST_REG,
  .rstbit     = STM32_RCC_LPTIM6_RST,
  .pins       =
  {
    LPTIM6_CH1OUT, LPTIM6_CH2OUT, LPTIM6_CH3OUT, LPTIM6_CH4OUT
  },
};
#endif

/* Instances indexed by LPTIM number - 1 */

static struct stm32_lptim_priv_s * const g_lptim[] =
{
#ifdef CONFIG_STM32_LPTIM1
  [0] = &g_lptim1,
#endif
#ifdef CONFIG_STM32_LPTIM2
  [1] = &g_lptim2,
#endif
#ifdef CONFIG_STM32_LPTIM3
  [2] = &g_lptim3,
#endif
#ifdef CONFIG_STM32_LPTIM4
  [3] = &g_lptim4,
#endif
#ifdef CONFIG_STM32_LPTIM5
  [4] = &g_lptim5,
#endif
#ifdef CONFIG_STM32_LPTIM6
  [5] = &g_lptim6,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_getreg32 / stm32_putreg32 / stm32_modifyreg32
 ****************************************************************************/

noinstrument_function
static inline uint32_t stm32_getreg32(struct stm32_lptim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg32(((struct stm32_lptim_priv_s *)dev)->base + offset);
}

noinstrument_function
static inline void stm32_putreg32(struct stm32_lptim_dev_s *dev,
                                  uint8_t offset, uint32_t value)
{
  putreg32(value, ((struct stm32_lptim_priv_s *)dev)->base + offset);
}

noinstrument_function
static inline void stm32_modifyreg32(struct stm32_lptim_dev_s *dev,
                                     uint8_t offset, uint32_t clearbits,
                                     uint32_t setbits)
{
  modifyreg32(((struct stm32_lptim_priv_s *)dev)->base + offset,
              clearbits, setbits);
}

#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V2
/****************************************************************************
 * Name: stm32_lptim_writesync
 *
 * Description:
 *   Write ARR or CMP and, when the timer is enabled, wait until the write
 *   to the register in the LPTIM kernel clock domain is complete.
 *
 ****************************************************************************/

static void stm32_lptim_writesync(struct stm32_lptim_dev_s *dev,
                                  uint8_t offset, uint32_t value,
                                  uint32_t okflag)
{
  bool enabled = (stm32_getreg32(dev, STM32_LPTIM_CR_OFFSET) &
                  LPTIM_CR_ENABLE) != 0;
  uint32_t timeout;

  if (enabled)
    {
      stm32_putreg32(dev, STM32_LPTIM_ICR_OFFSET, okflag);
    }

  stm32_putreg32(dev, offset, value);

  if (enabled)
    {
      timeout = 10000;
      while (!(stm32_getreg32(dev, STM32_LPTIM_ISR_OFFSET) & okflag) &&
             timeout > 0)
        {
          timeout--;
        }
    }
}

/****************************************************************************
 * Name: stm32_lptim_modifydier
 *
 * Description:
 *   Modify DIER.  DIER can only be written while the timer is enabled, so
 *   enable it for the write if needed and wait for DIEROK.
 *
 ****************************************************************************/

static void stm32_lptim_modifydier(struct stm32_lptim_dev_s *dev,
                                   uint32_t clearbits, uint32_t setbits)
{
  bool enabled = (stm32_getreg32(dev, STM32_LPTIM_CR_OFFSET) &
                  LPTIM_CR_ENABLE) != 0;
  uint32_t timeout;

  if (!enabled)
    {
      stm32_modifyreg32(dev, STM32_LPTIM_CR_OFFSET, 0, LPTIM_CR_ENABLE);
    }

  stm32_putreg32(dev, STM32_LPTIM_ICR_OFFSET, LPTIM_ISR_DIEROK);
  stm32_modifyreg32(dev, STM32_LPTIM_DIER_OFFSET, clearbits, setbits);

  timeout = 10000;
  while (!(stm32_getreg32(dev, STM32_LPTIM_ISR_OFFSET) &
           LPTIM_ISR_DIEROK) && timeout > 0)
    {
      timeout--;
    }

  if (!enabled)
    {
      stm32_modifyreg32(dev, STM32_LPTIM_CR_OFFSET, LPTIM_CR_ENABLE, 0);
    }
}
#endif

/****************************************************************************
 * Name: stm32_lptim_setmode
 *
 * Description:
 *   Disable the timer, or enable it and start counting once or
 *   continuously.
 *
 ****************************************************************************/

static int stm32_lptim_setmode(struct stm32_lptim_dev_s *dev,
                               stm32_lptim_mode_t mode)
{
  DEBUGASSERT(dev != NULL);

  mode &= STM32_LPTIM_MODE_MASK;
  if (mode == STM32_LPTIM_MODE_DISABLED)
    {
      stm32_lptim_enable(dev, false);
    }
  else if (mode == STM32_LPTIM_MODE_SINGLE ||
           mode == STM32_LPTIM_MODE_CONTINUOUS)
    {
      stm32_lptim_enable(dev, true);
      stm32_lptim_start(dev, mode);
    }
  else
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_setclock
 *
 * Description:
 *   Set the counter clock to the kernel clock divided by the largest power
 *   of two (up to 128) whose result is not above freq, and return the
 *   resulting frequency.  freq == 0 stops the LPTIM clock.  Requires
 *   STM32_LPTIMn_CLKIN in board.h.
 *
 ****************************************************************************/

static int stm32_lptim_setclock(struct stm32_lptim_dev_s *dev,
                                uint32_t freq)
{
  struct stm32_lptim_priv_s *priv = (struct stm32_lptim_priv_s *)dev;
  uint32_t presc;

  DEBUGASSERT(dev != NULL);

  if (freq == 0)
    {
      modifyreg32(priv->enreg, priv->enbit, 0);
      return 0;
    }

  if (priv->freq == 0)
    {
      return -EINVAL;
    }

  presc = 0;
  while (presc < LPTIM_PRESC_MAX && (priv->freq >> presc) > freq)
    {
      presc++;
    }

  modifyreg32(priv->enreg, 0, priv->enbit);
  stm32_modifyreg32(dev, STM32_LPTIM_CFGR_OFFSET, LPTIM_CFGR_PRESC_MASK,
                    presc << LPTIM_CFGR_PRESC_SHIFT);

  return priv->freq >> presc;
}

/****************************************************************************
 * Name: stm32_lptim_setchannel
 *
 * Description:
 *   Configure (enable != 0) or release the pin of an output channel.
 *
 ****************************************************************************/

static int stm32_lptim_setchannel(struct stm32_lptim_dev_s *dev,
                                  stm32_lptim_channel_t channel,
                                  int enable)
{
  struct stm32_lptim_priv_s *priv = (struct stm32_lptim_priv_s *)dev;
  uint32_t pin;

  DEBUGASSERT(dev != NULL);

  channel &= STM32_LPTIM_CH_MASK;
  if (channel < 1 || channel > LPTIM_NCHANNELS)
    {
      return -EINVAL;
    }

  pin = priv->pins[channel - 1];
  if (pin == 0)
    {
      return -EINVAL;
    }

  if (enable)
    {
      stm32_configgpio(pin);
    }
  else
    {
      stm32_unconfiggpio(pin);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_setclocksource
 *
 * Description:
 *   Count the external clock on input 1 (STM32_LPTIM_CLK_EXT) or the
 *   kernel clock selected in RCC (any other value).
 *
 ****************************************************************************/

static int stm32_lptim_setclocksource(struct stm32_lptim_dev_s *dev,
                                      stm32_lptim_clksrc_t clksrc)
{
  DEBUGASSERT(dev != NULL);

  stm32_modifyreg32(dev, STM32_LPTIM_CFGR_OFFSET, LPTIM_CFGR_CKSEL,
                    clksrc == STM32_LPTIM_CLK_EXT ? LPTIM_CFGR_CKSEL : 0);
  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_setpolarity
 ****************************************************************************/

static int stm32_lptim_setpolarity(struct stm32_lptim_dev_s *dev,
                                   stm32_lptim_clkpol_t polarity)
{
  DEBUGASSERT(dev != NULL);

  if (polarity > STM32_LPTIM_CLKPOL_BOTH)
    {
      return -EINVAL;
    }

  /* The enumeration values are the CKPOL field values */

  stm32_modifyreg32(dev, STM32_LPTIM_CFGR_OFFSET, LPTIM_CFGR_CKPOL_MASK,
                    (uint32_t)polarity << LPTIM_CFGR_CKPOL_SHIFT);
  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_getcounter
 ****************************************************************************/

static uint32_t stm32_lptim_getcounter(struct stm32_lptim_dev_s *dev)
{
  uint32_t cnt1;
  uint32_t cnt2;

  DEBUGASSERT(dev != NULL);

  /* For a reliable LPTIM_CNT register read access, two consecutive
   * read accesses must be performed and compared
   */

  cnt2 = stm32_getreg32(dev, STM32_LPTIM_CNT_OFFSET);

  do
    {
      cnt1 = cnt2;
      cnt2 = stm32_getreg32(dev, STM32_LPTIM_CNT_OFFSET);
    }
  while (cnt1 != cnt2);

  return cnt1;
}

/****************************************************************************
 * Name: stm32_lptim_setcountmode
 *
 * Description:
 *   Increment the counter on each counter clock pulse or on each valid
 *   pulse on input 1.
 *
 ****************************************************************************/

static int stm32_lptim_setcountmode(struct stm32_lptim_dev_s *dev,
                                    stm32_lptim_cntmode_t cntmode)
{
  DEBUGASSERT(dev != NULL);

  if (cntmode != STM32_LPTIM_COUNT_CLOCK &&
      cntmode != STM32_LPTIM_COUNT_EXTTRIG)
    {
      return -EINVAL;
    }

  stm32_modifyreg32(dev, STM32_LPTIM_CFGR_OFFSET, LPTIM_CFGR_COUNTMODE,
                    cntmode == STM32_LPTIM_COUNT_EXTTRIG ?
                    LPTIM_CFGR_COUNTMODE : 0);
  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_setperiod / stm32_lptim_getperiod
 ****************************************************************************/

static void stm32_lptim_setperiod(struct stm32_lptim_dev_s *dev,
                                  uint32_t period)
{
  DEBUGASSERT(dev != NULL);

#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V2
  stm32_lptim_writesync(dev, STM32_LPTIM_ARR_OFFSET, period,
                        LPTIM_ISR_ARROK);
#else
  stm32_putreg32(dev, STM32_LPTIM_ARR_OFFSET, period);
#endif
}

static uint32_t stm32_lptim_getperiod(struct stm32_lptim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  return stm32_getreg32(dev, STM32_LPTIM_ARR_OFFSET);
}

/****************************************************************************
 * Name: stm32_lptim_setcfgr
 ****************************************************************************/

static void stm32_lptim_setcfgr(struct stm32_lptim_dev_s *dev,
                                uint32_t regval)
{
  stm32_putreg32(dev, STM32_LPTIM_CFGR_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32_lptim_setcompare
 ****************************************************************************/

static void stm32_lptim_setcompare(struct stm32_lptim_dev_s *dev,
                                   uint32_t cmp)
{
#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V2
  stm32_lptim_writesync(dev, STM32_LPTIM_CMP_OFFSET, cmp, LPTIM_ISR_CMPOK);
#else
  stm32_putreg32(dev, STM32_LPTIM_CMP_OFFSET, cmp);
#endif
}

/****************************************************************************
 * Name: stm32_lptim_setinput
 *
 * Description:
 *   Select the input 1 (input == 0) or input 2 (input != 0) multiplexer
 *   setting in CFGR2.  The valid settings are instance specific.
 *
 ****************************************************************************/

static int stm32_lptim_setinput(struct stm32_lptim_dev_s *dev,
                                uint32_t input, uint32_t mux)
{
  if (input == 0)
    {
      stm32_modifyreg32(dev, STM32_LPTIM_CFGR2_OFFSET,
                        LPTIM_CFGR2_IN1SEL_MASK,
                        mux << LPTIM_CFGR2_IN1SEL_SHIFT);
    }
  else
    {
      stm32_modifyreg32(dev, STM32_LPTIM_CFGR2_OFFSET,
                        LPTIM_CFGR2_IN2SEL_MASK,
                        mux << LPTIM_CFGR2_IN2SEL_SHIFT);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_enable
 ****************************************************************************/

static void stm32_lptim_enable(struct stm32_lptim_dev_s *dev, bool on)
{
  if (on)
    {
      stm32_modifyreg32(dev, STM32_LPTIM_CR_OFFSET, 0, LPTIM_CR_ENABLE);
    }
  else
    {
      stm32_modifyreg32(dev, STM32_LPTIM_CR_OFFSET, LPTIM_CR_ENABLE, 0);
    }
}

/****************************************************************************
 * Name: stm32_lptim_start
 ****************************************************************************/

static void stm32_lptim_start(struct stm32_lptim_dev_s *dev,
                              stm32_lptim_mode_t mode)
{
  switch (mode & STM32_LPTIM_MODE_MASK)
    {
      case STM32_LPTIM_MODE_CONTINUOUS:
        stm32_modifyreg32(dev, STM32_LPTIM_CR_OFFSET, 0, LPTIM_CR_CNTSTRT);
        break;

      case STM32_LPTIM_MODE_SINGLE:
        stm32_modifyreg32(dev, STM32_LPTIM_CR_OFFSET, 0, LPTIM_CR_SNGSTRT);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: stm32_lptim_resetcounter
 ****************************************************************************/

static void stm32_lptim_resetcounter(struct stm32_lptim_dev_s *dev)
{
  stm32_modifyreg32(dev, STM32_LPTIM_CR_OFFSET, 0, LPTIM_CR_COUNTRST);
}

/****************************************************************************
 * Name: stm32_lptim_enablerar
 ****************************************************************************/

static void stm32_lptim_enablerar(struct stm32_lptim_dev_s *dev, bool on)
{
  if (on)
    {
      stm32_modifyreg32(dev, STM32_LPTIM_CR_OFFSET, 0, LPTIM_CR_RSTARE);
    }
  else
    {
      stm32_modifyreg32(dev, STM32_LPTIM_CR_OFFSET, LPTIM_CR_RSTARE, 0);
    }
}

/****************************************************************************
 * Name: stm32_lptim_setisr
 ****************************************************************************/

static int stm32_lptim_setisr(struct stm32_lptim_dev_s *dev,
                              xcpt_t handler, void *arg, int source)
{
  int irq;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  irq = ((struct stm32_lptim_priv_s *)dev)->irq;

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(irq);
      irq_detach(irq);
      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(irq, handler, arg);
  up_enable_irq(irq);

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the interrupt priority */

  up_prioritize_irq(irq, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_enableint / stm32_lptim_disableint / stm32_lptim_ackint
 ****************************************************************************/

static void stm32_lptim_enableint(struct stm32_lptim_dev_s *dev,
                                  int source)
{
  DEBUGASSERT(dev != NULL);

#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V2
  stm32_lptim_modifydier(dev, 0, source);
#else
  stm32_modifyreg32(dev, STM32_LPTIM_IER_OFFSET, 0, source);
#endif
}

static void stm32_lptim_disableint(struct stm32_lptim_dev_s *dev,
                                   int source)
{
  DEBUGASSERT(dev != NULL);

#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V2
  stm32_lptim_modifydier(dev, source, 0);
#else
  stm32_modifyreg32(dev, STM32_LPTIM_IER_OFFSET, source, 0);
#endif
}

static void stm32_lptim_ackint(struct stm32_lptim_dev_s *dev, int source)
{
  stm32_putreg32(dev, STM32_LPTIM_ICR_OFFSET, source);

  /* Read back so the flag is cleared before the interrupt returns */

  stm32_getreg32(dev, STM32_LPTIM_ISR_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_lptim_init
 ****************************************************************************/

struct stm32_lptim_dev_s *stm32_lptim_init(int lptimer)
{
  struct stm32_lptim_priv_s *priv;

  if (lptimer < 1 || lptimer > (int)nitems(g_lptim))
    {
      return NULL;
    }

  priv = g_lptim[lptimer - 1];
  if (priv == NULL || priv->inuse)
    {
      return NULL;
    }

  modifyreg32(priv->enreg, 0, priv->enbit);
  modifyreg32(priv->rstreg, 0, priv->rstbit);
  modifyreg32(priv->rstreg, priv->rstbit, 0);

  priv->inuse = true;
  return (struct stm32_lptim_dev_s *)priv;
}

/****************************************************************************
 * Name: stm32_lptim_deinit
 ****************************************************************************/

int stm32_lptim_deinit(struct stm32_lptim_dev_s *dev)
{
  struct stm32_lptim_priv_s *priv = (struct stm32_lptim_priv_s *)dev;

  DEBUGASSERT(dev != NULL);

  modifyreg32(priv->enreg, priv->enbit, 0);
  priv->inuse = false;
  return OK;
}

#endif /* CONFIG_STM32_LPTIM1 || ... || CONFIG_STM32_LPTIM6 */
