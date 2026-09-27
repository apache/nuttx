/****************************************************************************
 * arch/arm/src/n32h7/n32_capture.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "n32_rcc.h"
#include "n32_gpio.h"
#include "n32_capture.h"
#include "hardware/n32h7_rcc.h"
#include "hardware/n32h76x_pinmap.h"

#ifdef CONFIG_N32H7_CAPTURE

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Timer device private structure */

struct n32_cap_priv_s
{
  const struct n32_cap_ops_s *ops;
  uint32_t base;          /* timer base address */
  int irq;                /* main capture/compare IRQ */
  int irq_of;             /* update IRQ (0 if combined) */
  uint32_t input_clock;   /* timer input clock (before prescaler) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint16_t n32_getreg16(const struct n32_cap_priv_s *priv,
                                    uint8_t offset);
static inline void n32_putreg16(const struct n32_cap_priv_s *priv,
                                uint8_t offset, uint16_t value);
static inline void n32_modifyreg16(const struct n32_cap_priv_s *priv,
                                   uint8_t offset, uint16_t clearbits,
                                   uint16_t setbits);
static inline uint32_t n32_getreg32(const struct n32_cap_priv_s *priv,
                                    uint8_t offset);
static inline void n32_putreg32(const struct n32_cap_priv_s *priv,
                                uint8_t offset, uint32_t value);

static uint32_t n32_cap_gpio(const struct n32_cap_priv_s *priv, int channel);
static int n32_cap_set_rcc(const struct n32_cap_priv_s *priv, bool on);

static int n32_cap_setsmc(struct n32_cap_dev_s *dev, n32_cap_smc_cfg_t cfg);
static int n32_cap_setclock(struct n32_cap_dev_s *dev, uint32_t freq,
                            uint32_t max);
static int n32_cap_setchannel(struct n32_cap_dev_s *dev, uint8_t channel,
                              n32_cap_ch_cfg_t cfg);
static uint32_t n32_cap_getcapture(struct n32_cap_dev_s *dev,
                                   uint8_t channel);
static uint32_t n32_cap_getedges(struct n32_cap_dev_s *dev, uint8_t channel);
static int n32_cap_setisr(struct n32_cap_dev_s *dev, xcpt_t handler,
                          void *arg);
static void n32_cap_enableint(struct n32_cap_dev_s *dev, n32_cap_flags_t src,
                              bool on);
static void n32_cap_ackflags(struct n32_cap_dev_s *dev, int flags);
static n32_cap_flags_t n32_cap_getflags(struct n32_cap_dev_s *dev);

static const struct n32_cap_priv_s * n32_cap_get_priv(int timer);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct n32_cap_ops_s n32_cap_ops =
{
  .setsmc       = n32_cap_setsmc,
  .setclock     = n32_cap_setclock,
  .setchannel   = n32_cap_setchannel,
  .getcapture   = n32_cap_getcapture,
  .getedges     = n32_cap_getedges,
  .setisr       = n32_cap_setisr,
  .enableint    = n32_cap_enableint,
  .ackflags     = n32_cap_ackflags,
  .getflags     = n32_cap_getflags,
};

/* Timer private structures */

#ifdef CONFIG_N32H7_ATIM1_CAP
const struct n32_cap_priv_s n32_atim1_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_ATIMER1_BASE,
  .irq          = N32_IRQ_ATIM1_CC,
  .irq_of       = N32_IRQ_ATIM1_UP,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_ATIM2_CAP
const struct n32_cap_priv_s n32_atim2_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_ATIMER2_BASE,
  .irq          = N32_IRQ_ATIM2_CC,
  .irq_of       = N32_IRQ_ATIM2_UP,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_ATIM3_CAP
const struct n32_cap_priv_s n32_atim3_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_ATIMER3_BASE,
  .irq          = N32_IRQ_ATIM3_CC,
  .irq_of       = N32_IRQ_ATIM3_UP,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_ATIM4_CAP
const struct n32_cap_priv_s n32_atim4_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_ATIMER4_BASE,
  .irq          = N32_IRQ_ATIM4_CC,
  .irq_of       = N32_IRQ_ATIM4_UP,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA1_CAP
const struct n32_cap_priv_s n32_gtima1_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERA1_BASE,
  .irq          = N32_IRQ_GTIMA1,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA2_CAP
const struct n32_cap_priv_s n32_gtima2_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERA2_BASE,
  .irq          = N32_IRQ_GTIMA2,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA3_CAP
const struct n32_cap_priv_s n32_gtima3_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERA3_BASE,
  .irq          = N32_IRQ_GTIMA3,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA4_CAP
const struct n32_cap_priv_s n32_gtima4_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERA4_BASE,
  .irq          = N32_IRQ_GTIMA4,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA5_CAP
const struct n32_cap_priv_s n32_gtima5_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERA5_BASE,
  .irq          = N32_IRQ_GTIMA5,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA6_CAP
const struct n32_cap_priv_s n32_gtima6_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERA6_BASE,
  .irq          = N32_IRQ_GTIMA6,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA7_CAP
const struct n32_cap_priv_s n32_gtima7_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERA7_BASE,
  .irq          = N32_IRQ_GTIMA7,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMB1_CAP
const struct n32_cap_priv_s n32_gtimb1_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERB1_BASE,
  .irq          = N32_IRQ_GTIMB1,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMB2_CAP
const struct n32_cap_priv_s n32_gtimb2_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERB2_BASE,
  .irq          = N32_IRQ_GTIMB2,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMB3_CAP
const struct n32_cap_priv_s n32_gtimb3_priv =
{
  .ops          = &n32_cap_ops,
  .base         = N32_GTIMERB3_BASE,
  .irq          = N32_IRQ_GTIMB3,
  .irq_of       = 0,
  .input_clock  = N32_AHB_FREQUENCY,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint16_t n32_getreg16(const struct n32_cap_priv_s *priv,
                                    uint8_t offset)
{
  return getreg16(priv->base + offset);
}

static inline void n32_putreg16(const struct n32_cap_priv_s *priv,
                                uint8_t offset, uint16_t value)
{
  putreg16(value, priv->base + offset);
}

static inline void n32_modifyreg16(const struct n32_cap_priv_s *priv,
                                   uint8_t offset, uint16_t clearbits,
                                   uint16_t setbits)
{
  modifyreg16(priv->base + offset, clearbits, setbits);
}

static inline void n32_modifyreg32(const struct n32_cap_priv_s *priv,
                                   uint8_t offset, uint32_t clearbits,
                                   uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

static inline uint32_t n32_getreg32(const struct n32_cap_priv_s *priv,
                                    uint8_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void n32_putreg32(const struct n32_cap_priv_s *priv,
                                uint8_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: n32_cap_gpio
 *
 * Description:
 *   Return the GPIO configuration for a given timer channel.
 *   channel = -1: return first available channel
 ****************************************************************************/

static uint32_t n32_cap_gpio(const struct n32_cap_priv_s *priv, int channel)
{
  if (channel < 0)
    {
      /* Try counter channel first (ETR) then channels 1..4 */

      uint32_t gpio = n32_cap_gpio(priv, N32_CAP_CHANNEL_COUNTER);

      if (gpio)
        {
          return gpio;
        }

      for (int ch = 1; ch <= 4; ch++)
        {
          gpio = n32_cap_gpio(priv, ch);
          if (gpio)
            {
              return gpio;
            }
        }

      return 0;
    }

  switch (priv->base)
    {
  #ifdef CONFIG_N32H7_ATIM1_CAP
      case N32_ATIMER1_BASE:
        switch (channel)
          {
    #ifdef GPIO_ATIM1_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_ATIM1_ETR;
    #endif
    #ifdef GPIO_ATIM1_CH1IN
            case 1:
              return GPIO_ATIM1_CH1IN;
    #endif
    #ifdef GPIO_ATIM1_CH2IN
            case 2:
              return GPIO_ATIM1_CH2IN;
    #endif
    #ifdef GPIO_ATIM1_CH3IN
            case 3:
              return GPIO_ATIM1_CH3IN;
    #endif
    #ifdef GPIO_ATIM1_CH4IN
            case 4:
              return GPIO_ATIM1_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM2_CAP
      case N32_ATIMER2_BASE:
        switch (channel)
          {
    #ifdef GPIO_ATIM2_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_ATIM2_ETR;
    #endif
    #ifdef GPIO_ATIM2_CH1IN
            case 1:
              return GPIO_ATIM2_CH1IN;
    #endif
    #ifdef GPIO_ATIM2_CH2IN
            case 2:
              return GPIO_ATIM2_CH2IN;
    #endif
    #ifdef GPIO_ATIM2_CH3IN
            case 3:
              return GPIO_ATIM2_CH3IN;
    #endif
    #ifdef GPIO_ATIM2_CH4IN
            case 4:
              return GPIO_ATIM2_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM3_CAP
      case N32_ATIMER3_BASE:
        switch (channel)
          {
    #ifdef GPIO_ATIM3_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_ATIM3_ETR;
    #endif
    #ifdef GPIO_ATIM3_CH1IN
            case 1:
              return GPIO_ATIM3_CH1IN;
    #endif
    #ifdef GPIO_ATIM3_CH2IN
            case 2:
              return GPIO_ATIM3_CH2IN;
    #endif
    #ifdef GPIO_ATIM3_CH3IN
            case 3:
              return GPIO_ATIM3_CH3IN;
    #endif
    #ifdef GPIO_ATIM3_CH4IN
            case 4:
              return GPIO_ATIM3_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM4_CAP
      case N32_ATIMER4_BASE:
        switch (channel)
          {
    #ifdef GPIO_ATIM4_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_ATIM4_ETR;
    #endif
    #ifdef GPIO_ATIM4_CH1IN
            case 1:
              return GPIO_ATIM4_CH1IN;
    #endif
    #ifdef GPIO_ATIM4_CH2IN
            case 2:
              return GPIO_ATIM4_CH2IN;
    #endif
    #ifdef GPIO_ATIM4_CH3IN
            case 3:
              return GPIO_ATIM4_CH3IN;
    #endif
    #ifdef GPIO_ATIM4_CH4IN
            case 4:
              return GPIO_ATIM4_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA1_CAP
      case N32_GTIMERA1_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMA1_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMA1_ETR;
    #endif
    #ifdef GPIO_GTIMA1_CH1IN
            case 1:
              return GPIO_GTIMA1_CH1IN;
    #endif
    #ifdef GPIO_GTIMA1_CH2IN
            case 2:
              return GPIO_GTIMA1_CH2IN;
    #endif
    #ifdef GPIO_GTIMA1_CH3IN
            case 3:
              return GPIO_GTIMA1_CH3IN;
    #endif
    #ifdef GPIO_GTIMA1_CH4IN
            case 4:
              return GPIO_GTIMA1_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA2_CAP
      case N32_GTIMERA2_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMA2_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMA2_ETR;
    #endif
    #ifdef GPIO_GTIMA2_CH1IN
            case 1:
              return GPIO_GTIMA2_CH1IN;
    #endif
    #ifdef GPIO_GTIMA2_CH2IN
            case 2:
              return GPIO_GTIMA2_CH2IN;
    #endif
    #ifdef GPIO_GTIMA2_CH3IN
            case 3:
              return GPIO_GTIMA2_CH3IN;
    #endif
    #ifdef GPIO_GTIMA2_CH4IN
            case 4:
              return GPIO_GTIMA2_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA3_CAP
      case N32_GTIMERA3_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMA3_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMA3_ETR;
    #endif
    #ifdef GPIO_GTIMA3_CH1IN
            case 1:
              return GPIO_GTIMA3_CH1IN;
    #endif
    #ifdef GPIO_GTIMA3_CH2IN
            case 2:
              return GPIO_GTIMA3_CH2IN;
    #endif
    #ifdef GPIO_GTIMA3_CH3IN
            case 3:
              return GPIO_GTIMA3_CH3IN;
    #endif
    #ifdef GPIO_GTIMA3_CH4IN
            case 4:
              return GPIO_GTIMA3_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA4_CAP
      case N32_GTIMERA4_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMA4_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMA4_ETR;
    #endif
    #ifdef GPIO_GTIMA4_CH1IN
            case 1:
              return GPIO_GTIMA4_CH1IN;
    #endif
    #ifdef GPIO_GTIMA4_CH2IN
            case 2:
              return GPIO_GTIMA4_CH2IN;
    #endif
    #ifdef GPIO_GTIMA4_CH3IN
            case 3:
              return GPIO_GTIMA4_CH3IN;
    #endif
    #ifdef GPIO_GTIMA4_CH4IN
            case 4:
              return GPIO_GTIMA4_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA5_CAP
      case N32_GTIMERA5_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMA5_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMA5_ETR;
    #endif
    #ifdef GPIO_GTIMA5_CH1IN
            case 1:
              return GPIO_GTIMA5_CH1IN;
    #endif
    #ifdef GPIO_GTIMA5_CH2IN
            case 2:
              return GPIO_GTIMA5_CH2IN;
    #endif
    #ifdef GPIO_GTIMA5_CH3IN
            case 3:
              return GPIO_GTIMA5_CH3IN;
    #endif
    #ifdef GPIO_GTIMA5_CH4IN
            case 4:
              return GPIO_GTIMA5_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA6_CAP
      case N32_GTIMERA6_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMA6_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMA6_ETR;
    #endif
    #ifdef GPIO_GTIMA6_CH1IN
            case 1:
              return GPIO_GTIMA6_CH1IN;
    #endif
    #ifdef GPIO_GTIMA6_CH2IN
            case 2:
              return GPIO_GTIMA6_CH2IN;
    #endif
    #ifdef GPIO_GTIMA6_CH3IN
            case 3:
              return GPIO_GTIMA6_CH3IN;
    #endif
    #ifdef GPIO_GTIMA6_CH4IN
            case 4:
              return GPIO_GTIMA6_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA7_CAP
      case N32_GTIMERA7_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMA7_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMA7_ETR;
    #endif
    #ifdef GPIO_GTIMA7_CH1IN
            case 1:
              return GPIO_GTIMA7_CH1IN;
    #endif
    #ifdef GPIO_GTIMA7_CH2IN
            case 2:
              return GPIO_GTIMA7_CH2IN;
    #endif
    #ifdef GPIO_GTIMA7_CH3IN
            case 3:
              return GPIO_GTIMA7_CH3IN;
    #endif
    #ifdef GPIO_GTIMA7_CH4IN
            case 4:
              return GPIO_GTIMA7_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB1_CAP
      case N32_GTIMERB1_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMB1_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMB1_ETR;
    #endif
    #ifdef GPIO_GTIMB1_CH1PIN
            case 1:
              return GPIO_GTIMB1_CH1PIN;
    #endif
    #ifdef GPIO_GTIMB1_CH2IN
            case 2:
              return GPIO_GTIMB1_CH2IN;
    #endif
    #ifdef GPIO_GTIMB1_CH3IN
            case 3:
              return GPIO_GTIMB1_CH3IN;
    #endif
    #ifdef GPIO_GTIMB1_CH4IN
            case 4:
              return GPIO_GTIMB1_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB2_CAP
      case N32_GTIMERB2_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMB2_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMB2_ETR;
    #endif
    #ifdef GPIO_GTIMB2_CH1PIN
            case 1:
              return GPIO_GTIMB2_CH1PIN;
    #endif
    #ifdef GPIO_GTIMB2_CH2IN
            case 2:
              return GPIO_GTIMB2_CH2IN;
    #endif
    #ifdef GPIO_GTIMB2_CH3IN
            case 3:
              return GPIO_GTIMB2_CH3IN;
    #endif
    #ifdef GPIO_GTIMB2_CH4IN
            case 4:
              return GPIO_GTIMB2_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB3_CAP
      case N32_GTIMERB3_BASE:
        switch (channel)
          {
    #ifdef GPIO_GTIMB3_ETR
            case N32_CAP_CHANNEL_COUNTER:
              return GPIO_GTIMB3_ETR;
    #endif
    #ifdef GPIO_GTIMB3_CH1PIN
            case 1:
              return GPIO_GTIMB3_CH1PIN;
    #endif
    #ifdef GPIO_GTIMB3_CH2IN
            case 2:
              return GPIO_GTIMB3_CH2IN;
    #endif
    #ifdef GPIO_GTIMB3_CH3IN
            case 3:
              return GPIO_GTIMB3_CH3IN;
    #endif
    #ifdef GPIO_GTIMB3_CH4IN
            case 4:
              return GPIO_GTIMB3_CH4IN;
    #endif
            default:
              break;
          }
        break;
  #endif
      default:
        break;
    }

  return 0;
}

/****************************************************************************
 * Name: n32_cap_set_rcc
 ****************************************************************************/

static int n32_cap_set_rcc(const struct n32_cap_priv_s *priv, bool on)
{
  uint32_t reg = 0;
  uint32_t bit = 0;

  switch (priv->base)
    {
  #ifdef CONFIG_N32H7_ATIM1_CAP
      case N32_ATIMER1_BASE:
        reg = N32_RCC_APB2EN1;
        bit = RCC_APB2EN1_M7ATIM1EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM2_CAP
      case N32_ATIMER2_BASE:
        reg = N32_RCC_APB2EN1;
        bit = RCC_APB2EN1_M7ATIM2EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM3_CAP
      case N32_ATIMER3_BASE:
        reg = N32_RCC_APB5EN1;
        bit = RCC_APB5EN1_M7ATIM3EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM4_CAP
      case N32_ATIMER4_BASE:
        reg = N32_RCC_APB5EN1;
        bit = RCC_APB5EN1_M7ATIM4EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA1_CAP
      case N32_GTIMERA1_BASE:
        reg = N32_RCC_APB2EN1;
        bit = RCC_APB2EN1_M7GTIMA1EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA2_CAP
      case N32_GTIMERA2_BASE:
        reg = N32_RCC_APB2EN1;
        bit = RCC_APB2EN1_M7GTIMA2EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA3_CAP
      case N32_GTIMERA3_BASE:
        reg = N32_RCC_APB2EN1;
        bit = RCC_APB2EN1_M7GTIMA3EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA4_CAP
      case N32_GTIMERA4_BASE:
        reg = N32_RCC_APB1EN1;
        bit = RCC_APB1EN1_M7GTIMA4EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA5_CAP
      case N32_GTIMERA5_BASE:
        reg = N32_RCC_APB1EN2;
        bit = RCC_APB1EN2_M7GTIMA5EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA6_CAP
      case N32_GTIMERA6_BASE:
        reg = N32_RCC_APB1EN2;
        bit = RCC_APB1EN2_M7GTIMA6EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA7_CAP
      case N32_GTIMERA7_BASE:
        reg = N32_RCC_APB1EN2;
        bit = RCC_APB1EN2_M7GTIMA7EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB1_CAP
      case N32_GTIMERB1_BASE:
        reg = N32_RCC_APB1EN1;
        bit = RCC_APB1EN1_M7GTIMB1EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB2_CAP
      case N32_GTIMERB2_BASE:
        reg = N32_RCC_APB1EN1;
        bit = RCC_APB1EN1_M7GTIMB2EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB3_CAP
      case N32_GTIMERB3_BASE:
        reg = N32_RCC_APB1EN1;
        bit = RCC_APB1EN1_M7GTIMB3EN;
        break;
  #endif
      default:
        return -EINVAL;
    }

  if (on)
    {
      modifyreg32(reg, 0, bit);
    }
  else
    {
      modifyreg32(reg, bit, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: n32_cap_setsmc
 ****************************************************************************/

static int n32_cap_setsmc(struct n32_cap_dev_s *dev, n32_cap_smc_cfg_t cfg)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;
  uint32_t regval = 0;
  uint32_t mask = 0;

  switch (cfg & N32_ATIM_SMCTRL_SMSEL_MASK)
    {
      case N32_ATIM_SMCTRL_DISAB:
        regval |= N32_ATIM_SMCTRL_DISAB;
        break;
      case N32_ATIM_SMCTRL_ENCMD1:
        regval |= N32_ATIM_SMCTRL_ENCMD1;
        break;
      case N32_ATIM_SMCTRL_ENCMD2:
        regval |= N32_ATIM_SMCTRL_ENCMD2;
        break;
      case N32_ATIM_SMCTRL_ENCMD3:
        regval |= N32_ATIM_SMCTRL_ENCMD3;
        break;
      case N32_ATIM_SMCTRL_RESET:
        regval |= N32_ATIM_SMCTRL_RESET;
        break;
      case N32_ATIM_SMCTRL_GATED:
        regval |= N32_ATIM_SMCTRL_GATED;
        break;
      case N32_ATIM_SMCTRL_TRIGGER:
        regval |= N32_ATIM_SMCTRL_TRIGGER;
        break;
      case N32_ATIM_SMCTRL_EXTCLK1:
        regval |= N32_ATIM_SMCTRL_EXTCLK1;
        break;
      default:
        break;
    }

  switch (cfg & N32_ATIM_SMCTRL_TSEL_MASK)
    {
      case N32_ATIM_SMCTRL_ITR:
        regval |= N32_ATIM_SMCTRL_ITR;
        break;
      case N32_ATIM_SMCTRL_TI1FED:
        regval |= N32_ATIM_SMCTRL_TI1FED;
        break;
      case N32_ATIM_SMCTRL_TI1FP1:
        regval |= N32_ATIM_SMCTRL_TI1FP1;
        break;
      case N32_ATIM_SMCTRL_TI2FP2:
        regval |= N32_ATIM_SMCTRL_TI2FP2;
        break;
      case N32_ATIM_SMCTRL_ETRF:
        regval |= N32_ATIM_SMCTRL_ETRF;
        break;
      default:
        break;
    }

  if (cfg & N32_ATIM_SMCTRL_MSMD)
    {
      regval |= N32_ATIM_SMCTRL_MSMD;
    }

  mask = (uint32_t)(N32_ATIM_SMCTRL_SMSEL_MASK | N32_ATIM_SMCTRL_TSEL_MASK
                     | N32_ATIM_SMCTRL_MSMD);
  n32_modifyreg32(priv, N32_ATIM_SMCTRL_OFFSET, mask, regval);
  return OK;
}

/****************************************************************************
 * Name: n32_cap_setclock
 *
 * Description:
 *   Set timer prescaler and auto-reload. 'freq' is the desired counting
 *   frequency (after prescaler), 'max' is the auto-reload value.
 *   Returns the actual prescaler value.
 ****************************************************************************/

static int n32_cap_setclock(struct n32_cap_dev_s *dev, uint32_t freq,
                            uint32_t max)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;
  uint32_t freqin = priv->input_clock;
  int prescaler;

  if (freq == 0)
    {
      /* Zero frequency means no prescaling, count at input clock */

      prescaler = 0;
    }
  else
    {
      prescaler = freqin / freq;
      if (prescaler > 0)
        {
          prescaler--;
        }

      if (prescaler > 0xffff)
        {
          prescaler = 0xffff;
        }
    }

  n32_putreg32(priv, N32_ATIM_AR_OFFSET, max);
  n32_putreg16(priv, N32_ATIM_PSC_OFFSET, prescaler);
  n32_putreg16(priv, N32_ATIM_ETGEN_OFFSET, N32_ATIM_ETGEN_UDGN);
  n32_modifyreg16(priv, N32_ATIM_CTRL1_OFFSET, 0, N32_ATIM_CTRL1_CNTEN);
  return prescaler;
}

/****************************************************************************
 * Name: n32_cap_setchannel
 ****************************************************************************/

static int n32_cap_setchannel(struct n32_cap_dev_s *dev, uint8_t channel,
                              n32_cap_ch_cfg_t cfg)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;
  uint32_t gpio;
  uint16_t mask;
  uint16_t regval;
  uint16_t ccer_en_bit;

  gpio = n32_cap_gpio(priv, channel);
  if (!gpio)
    {
      return -EINVAL;
    }

  channel--;

  switch (cfg & N32_CAP_EDGE_MASK)
    {
      case N32_CAP_EDGE_DISABLED:
        regval = 0;
        ccer_en_bit = 0;
        break;
      case N32_CAP_EDGE_RISING:
        ccer_en_bit = N32_ATIM_CCEN_CC1EN;
        regval = 0;
        break;
      case N32_CAP_EDGE_FALLING:
        ccer_en_bit = N32_ATIM_CCEN_CC1EN;
        regval = N32_ATIM_CCEN_CC1P;
        break;
      case N32_CAP_EDGE_BOTH:
        ccer_en_bit = N32_ATIM_CCEN_CC1EN;
        regval = N32_ATIM_CCEN_CC1P | N32_ATIM_CCEN_CC1NP;
        break;
      default:
        return -EINVAL;
    }

  mask = (N32_ATIM_CCEN_CC1EN | N32_ATIM_CCEN_CC1P | N32_ATIM_CCEN_CC1NP);
  mask <<= (channel * 4);
  regval <<= (channel * 4);
  ccer_en_bit <<= (channel * 4);

  n32_modifyreg16(priv, N32_ATIM_CCEN_OFFSET, mask, regval);

  regval = cfg;
  mask = (N32_ATIM_CCMOD1_IC1F_MASK |
          N32_ATIM_CCMOD1_IC1PSC_MASK |
          N32_ATIM_CCMOD1_CC1SEL_MASK);
  regval &= mask;
  if (channel & 1)
    {
      regval <<= 8;
      mask <<= 8;
    }

  if (channel < 2)
    {
      n32_modifyreg16(priv, N32_ATIM_CCMOD1_OFFSET, mask, regval);
    }
  else
    {
      n32_modifyreg16(priv, N32_ATIM_CCMOD2_OFFSET, mask, regval);
    }

  if ((cfg & N32_CAP_EDGE_MASK) == N32_CAP_EDGE_DISABLED)
    {
      n32_unconfiggpio(gpio);
    }
  else
    {
      n32_configgpio(gpio);
    }

  n32_modifyreg16(priv, N32_ATIM_CCEN_OFFSET, 0, ccer_en_bit);
  return OK;
}

/****************************************************************************
 * Name: n32_cap_getcapture
 ****************************************************************************/

static uint32_t n32_cap_getcapture(struct n32_cap_dev_s *dev,
                                   uint8_t channel)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;
  uint8_t offset;

  switch (channel)
    {
      case N32_CAP_CHANNEL_COUNTER:
        offset = N32_ATIM_CNT_OFFSET;
        break;
      case 1:
        offset = N32_ATIM_CCDAT1_OFFSET;
        break;
      case 2:
        offset = N32_ATIM_CCDAT2_OFFSET;
        break;
      case 3:
        offset = N32_ATIM_CCDAT3_OFFSET;
        break;
      case 4:
        offset = N32_ATIM_CCDAT4_OFFSET;
        break;
      default:
        return 0;
    }

  /* All N32 timers here are 16-bit */

  return n32_getreg16(priv, offset);
}

/****************************************************************************
 * Name: n32_cap_getedges
 ****************************************************************************/

static uint32_t n32_cap_getedges(struct n32_cap_dev_s *dev, uint8_t channel)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;
  uint16_t ccer;
  uint8_t shift;

  DEBUGASSERT(dev != NULL && channel >= 1 && channel <= 4);

  shift = (channel - 1) * 4;
  ccer = n32_getreg16(priv, N32_ATIM_CCEN_OFFSET);
  ccer = (ccer >> shift) & (N32_ATIM_CCEN_CC1P | N32_ATIM_CCEN_CC1NP);

  switch (ccer)
    {
      case 0:                                    /* Rising edge */
        return (uint32_t)N32_CAP_EDGE_RISING;
      case N32_ATIM_CCEN_CC1P:                   /* Falling edge */
        return (uint32_t)N32_CAP_EDGE_FALLING;
      case (N32_ATIM_CCEN_CC1P | N32_ATIM_CCEN_CC1NP): /* Both edges */
        return (uint32_t)N32_CAP_EDGE_BOTH;
      default:
        return (uint32_t)N32_CAP_EDGE_DISABLED;
    }
}

/****************************************************************************
 * Name: n32_cap_setisr
 ****************************************************************************/

static int n32_cap_setisr(struct n32_cap_dev_s *dev, xcpt_t handler,
                          void *arg)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;

  if (!handler)
    {
      up_disable_irq(priv->irq);
      irq_detach(priv->irq);
      if (priv->irq_of)
        {
          up_disable_irq(priv->irq_of);
          irq_detach(priv->irq_of);
        }

      return OK;
    }

  irq_attach(priv->irq, handler, arg);
  up_enable_irq(priv->irq);
  if (priv->irq_of)
    {
      irq_attach(priv->irq_of, handler, arg);
      up_enable_irq(priv->irq_of);
    }

  return OK;
}

/****************************************************************************
 * Name: n32_cap_enableint
 ****************************************************************************/

static void n32_cap_enableint(struct n32_cap_dev_s *dev, n32_cap_flags_t src,
                              bool on)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;
  uint32_t mask = 0;

  if (src & N32_CAP_FLAG_IRQ_COUNTER)
    {
      mask |= N32_ATIM_DINTEN_UIEN;
    }

  if (src & N32_CAP_FLAG_IRQ_CH_1)
    {
      mask |= N32_ATIM_DINTEN_CC1IEN;
    }

  if (src & N32_CAP_FLAG_IRQ_CH_2)
    {
      mask |= N32_ATIM_DINTEN_CC2IEN;
    }

  if (src & N32_CAP_FLAG_IRQ_CH_3)
    {
      mask |= N32_ATIM_DINTEN_CC3IEN;
    }

  if (src & N32_CAP_FLAG_IRQ_CH_4)
    {
      mask |= N32_ATIM_DINTEN_CC4IEN;
    }

  if (on)
    {
      n32_modifyreg32(priv, N32_ATIM_DINTEN_OFFSET, 0, mask);
    }
  else
    {
      n32_modifyreg32(priv, N32_ATIM_DINTEN_OFFSET, mask, 0);
    }
}

/****************************************************************************
 * Name: n32_cap_ackflags
 ****************************************************************************/

static void n32_cap_ackflags(struct n32_cap_dev_s *dev, int flags)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;
  uint32_t mask = 0;

  /* Clear by writing 0 to the corresponding bits */

  if (flags & N32_CAP_FLAG_IRQ_COUNTER)
    {
      mask |= N32_ATIM_STS_UDITF;
    }

  if (flags & N32_CAP_FLAG_IRQ_CH_1)
    {
      mask |= N32_ATIM_STS_CC1ITF;
    }

  if (flags & N32_CAP_FLAG_IRQ_CH_2)
    {
      mask |= N32_ATIM_STS_CC2ITF;
    }

  if (flags & N32_CAP_FLAG_IRQ_CH_3)
    {
      mask |= N32_ATIM_STS_CC3ITF;
    }

  if (flags & N32_CAP_FLAG_IRQ_CH_4)
    {
      mask |= N32_ATIM_STS_CC4ITF;
    }

  if (flags & N32_CAP_FLAG_OF_CH_1)
    {
      mask |= N32_ATIM_STS_CC1OCF;
    }

  if (flags & N32_CAP_FLAG_OF_CH_2)
    {
      mask |= N32_ATIM_STS_CC2OCF;
    }

  if (flags & N32_CAP_FLAG_OF_CH_3)
    {
      mask |= N32_ATIM_STS_CC3OCF;
    }

  if (flags & N32_CAP_FLAG_OF_CH_4)
    {
      mask |= N32_ATIM_STS_CC4OCF;
    }

  n32_putreg32(priv, N32_ATIM_STS_OFFSET, ~mask);
}

/****************************************************************************
 * Name: n32_cap_getflags
 ****************************************************************************/

static n32_cap_flags_t n32_cap_getflags(struct n32_cap_dev_s *dev)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;
  uint32_t regval = n32_getreg32(priv, N32_ATIM_STS_OFFSET);
  n32_cap_flags_t flags = 0;

  if (regval & N32_ATIM_STS_UDITF)
    {
      flags |= N32_CAP_FLAG_IRQ_COUNTER;
    }

  if (regval & N32_ATIM_STS_CC1ITF)
    {
      flags |= N32_CAP_FLAG_IRQ_CH_1;
    }

  if (regval & N32_ATIM_STS_CC2ITF)
    {
      flags |= N32_CAP_FLAG_IRQ_CH_2;
    }

  if (regval & N32_ATIM_STS_CC3ITF)
    {
      flags |= N32_CAP_FLAG_IRQ_CH_3;
    }

  if (regval & N32_ATIM_STS_CC4ITF)
    {
      flags |= N32_CAP_FLAG_IRQ_CH_4;
    }

  if (regval & N32_ATIM_STS_CC1OCF)
    {
      flags |= N32_CAP_FLAG_OF_CH_1;
    }

  if (regval & N32_ATIM_STS_CC2OCF)
    {
      flags |= N32_CAP_FLAG_OF_CH_2;
    }

  if (regval & N32_ATIM_STS_CC3OCF)
    {
      flags |= N32_CAP_FLAG_OF_CH_3;
    }

  if (regval & N32_ATIM_STS_CC4OCF)
    {
      flags |= N32_CAP_FLAG_OF_CH_4;
    }

  return flags;
}

/****************************************************************************
 * Name: n32_cap_get_priv (generated)
 ****************************************************************************/

static inline const struct n32_cap_priv_s * n32_cap_get_priv(int timer)
{
  switch (timer)
    {
  #ifdef CONFIG_N32H7_ATIM1_CAP
      case 1:
        return &n32_atim1_priv;
  #endif
  #ifdef CONFIG_N32H7_ATIM2_CAP
      case 2:
        return &n32_atim2_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMA1_CAP
      case 3:
        return &n32_gtima1_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMA2_CAP
      case 4:
        return &n32_gtima2_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMA3_CAP
      case 5:
        return &n32_gtima3_priv;
  #endif
  #ifdef CONFIG_N32H7_ATIM3_CAP
      case 6:
        return &n32_atim3_priv;
  #endif
  #ifdef CONFIG_N32H7_ATIM4_CAP
      case 7:
        return &n32_atim4_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMA4_CAP
      case 8:
        return &n32_gtima4_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMA5_CAP
      case 9:
        return &n32_gtima5_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMA6_CAP
      case 10:
        return &n32_gtima6_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMA7_CAP
      case 11:
        return &n32_gtima7_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMB1_CAP
      case 12:
        return &n32_gtimb1_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMB2_CAP
      case 13:
        return &n32_gtimb2_priv;
  #endif
  #ifdef CONFIG_N32H7_GTIMB3_CAP
      case 14:
        return &n32_gtimb3_priv;
  #endif
      default:
        return NULL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct n32_cap_dev_s *n32_cap_init(int timer)
{
  const struct n32_cap_priv_s *priv = n32_cap_get_priv(timer);
  uint32_t gpio;

  if (priv)
    {
      n32_cap_set_rcc(priv, true);
      gpio = n32_cap_gpio(priv, -1);
      if (gpio)
        {
          n32_configgpio(gpio);
        }

      n32_modifyreg16(priv, N32_ATIM_CTRL1_OFFSET, N32_ATIM_CTRL1_CNTEN, 0);
    }

  return (struct n32_cap_dev_s *)priv;
}

int n32_cap_deinit(struct n32_cap_dev_s *dev)
{
  const struct n32_cap_priv_s *priv = (const struct n32_cap_priv_s *)dev;
  uint32_t gpio;

  if (!priv)
    {
      return -EINVAL;
    }

  n32_modifyreg16(priv, N32_ATIM_CTRL1_OFFSET, N32_ATIM_CTRL1_CNTEN, 0);
  gpio = n32_cap_gpio(priv, -1);
  if (gpio)
    {
      n32_unconfiggpio(gpio);
    }

  n32_cap_set_rcc(priv, false);
  return OK;
}

#endif /* CONFIG_N32H7_CAPTURE */
