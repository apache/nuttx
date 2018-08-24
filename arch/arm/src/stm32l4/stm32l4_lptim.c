/************************************************************************************
 * arm/arm/src/stm3l42/stm32l4_lptim.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
 *
 *   Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/
/************************************************************************************
 *   Copyright (c) 2015 Google, Inc.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <arch/board/board.h>

#include "stm32l4.h"
#include "stm32l4_gpio.h"
#include "stm32l4_lptim.h"

#if defined(CONFIG_STM32L4_LPTIM1) || defined(CONFIG_STM32L4_LPTIM2)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* TIM Device Structure */

struct stm32l4_lptim_priv_s
{
  const struct stm32l4_lptim_ops_s *ops;
  stm32l4_lptim_mode_t mode;
  uint32_t base;                   /* LPTIMn base address */
  uint32_t freq;                   /* Clocking for the LPTIM module */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static struct stm32l4_lptim_dev_s *stm32l4_lptim_getstruct(int timer);
static inline void stm32l4_modifyreg32(FAR struct stm32l4_lptim_dev_s *dev,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits);
static int stm32l4_lptim_enable(FAR struct stm32l4_lptim_dev_s *dev);
static int stm32l4_lptim_disable(FAR struct stm32l4_lptim_dev_s *dev);
static int stm32l4_lptim_reset(FAR struct stm32l4_lptim_dev_s *dev);
static int stm32l4_lptim_get_gpioconfig(FAR struct stm32l4_lptim_dev_s *dev,
                                        stm32l4_lptim_channel_t channel,
                                        uint32_t *cfg);
static int stm32l4_lptim_setmode(FAR struct stm32l4_lptim_dev_s *dev,
                                 stm32l4_lptim_mode_t mode);
static int stm32l4_lptim_setclock(FAR struct stm32l4_lptim_dev_s *dev,
                                  uint32_t freq);
static int stm32l4_lptim_setchannel(FAR struct stm32l4_lptim_dev_s *dev,
                                    stm32l4_lptim_channel_t channel, int enable);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct stm32l4_lptim_ops_s stm32l4_lptim_ops =
{
  .setmode    = &stm32l4_lptim_setmode,
  .setclock   = &stm32l4_lptim_setclock,
  .setchannel = &stm32l4_lptim_setchannel,
};

#if CONFIG_STM32L4_LPTIM1
static struct stm32l4_lptim_priv_s stm32l4_lptim1_priv =
{
  .ops        = &stm32l4_lptim_ops,
  .mode       = STM32L4_LPTIM_MODE_UNUSED,
  .base       = STM32L4_LPTIM1_BASE,
  .freq       = STM32L4_LPTIM1_FREQUENCY,  /* Must be efined in board.h */
};
#endif

#if CONFIG_STM32L4_LPTIM2
static struct stm32l4_lptim_priv_s stm32l4_lptim2_priv =
{
  .ops        = &stm32l4_lptim_ops,
  .mode       = STM32L4_LPTIM_MODE_UNUSED,
  .base       = STM32L4_LPTIM2_BASE,
  .freq       = STM32L4_LPTIM2_FREQUENCY,  /* Must be efined in board.h */
};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_lptim_getstruct
 ************************************************************************************/

static struct stm32l4_lptim_dev_s *stm32l4_lptim_getstruct(int timer)
{
  switch (timer)
    {
#if CONFIG_STM32L4_LPTIM1
      case 1:
        return (struct stm32l4_lptim_dev_s *)&stm32l4_lptim1_priv;
#endif
#if CONFIG_STM32L4_LPTIM2
      case 2:
        return (struct stm32l4_lptim_dev_s *)&stm32l4_lptim2_priv;
#endif
      default:
        return NULL;
    }
}

/************************************************************************************
 * Name: stm32l4_modifyreg32
 ************************************************************************************/

static inline void stm32l4_modifyreg32(FAR struct stm32l4_lptim_dev_s *dev,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits)
{
  modifyreg32(((struct stm32l4_lptim_priv_s *)dev)->base + offset, clearbits, setbits);
}

/************************************************************************************
 * Name: stm32l4_lptim_enable
 ************************************************************************************/

static int stm32l4_lptim_enable(FAR struct stm32l4_lptim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  switch (((struct stm32l4_lptim_priv_s *)dev)->base)
    {
#if CONFIG_STM32L4_LPTIM1
      case STM32L4_LPTIM1_BASE:
        modifyreg32(STM32L4_RCC_APB1ENR1, 0, RCC_APB1ENR1_LPTIM1EN);
        break;
#endif
#if CONFIG_STM32L4_LPTIM2
      case STM32L4_LPTIM2_BASE:
        modifyreg32(STM32L4_RCC_APB1ENR2, 0, RCC_APB1ENR2_LPTIM2EN);
        break;
#endif

      default:
        return ERROR;
    }

  return OK;
}

/************************************************************************************
 * Name: stm32l4_lptim_disable
 ************************************************************************************/

static int stm32l4_lptim_disable(FAR struct stm32l4_lptim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  switch (((struct stm32l4_lptim_priv_s *)dev)->base)
    {
#if CONFIG_STM32L4_LPTIM1
      case STM32L4_LPTIM1_BASE:
        modifyreg32(STM32L4_RCC_APB1ENR1, RCC_APB1ENR1_LPTIM1EN, 0);
        break;
#endif
#if CONFIG_STM32L4_LPTIM2
      case STM32L4_LPTIM2_BASE:
        modifyreg32(STM32L4_RCC_APB1ENR2, RCC_APB1ENR2_LPTIM2EN, 0);
        break;
#endif

      default:
        return ERROR;
    }

  return OK;
}

/************************************************************************************
 * Name: stm32l4_lptim_reset
 ************************************************************************************/

static int stm32l4_lptim_reset(FAR struct stm32l4_lptim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  switch (((struct stm32l4_lptim_priv_s *)dev)->base)
    {
#if CONFIG_STM32L4_LPTIM1
      case STM32L4_LPTIM1_BASE:
        modifyreg32(STM32L4_RCC_APB1RSTR1, 0, RCC_APB1RSTR1_LPTIM1RST);
        modifyreg32(STM32L4_RCC_APB1RSTR1, RCC_APB1RSTR1_LPTIM1RST, 0);
        break;
#endif
#if CONFIG_STM32L4_LPTIM2
      case STM32L4_LPTIM2_BASE:
        modifyreg32(STM32L4_RCC_APB1RSTR2, 0, RCC_APB1RSTR2_LPTIM2RST);
        modifyreg32(STM32L4_RCC_APB1RSTR2, RCC_APB1RSTR2_LPTIM2RST, 0);
        break;
#endif
    }

  return OK;
}

/************************************************************************************
 * Name: stm32l4_lptim_get_gpioconfig
 ************************************************************************************/

static int stm32l4_lptim_get_gpioconfig(FAR struct stm32l4_lptim_dev_s *dev,
                                        stm32l4_lptim_channel_t channel,
                                        uint32_t *cfg)
{
  DEBUGASSERT(dev != NULL && cfg != NULL);

  channel &= STM32L4_LPTIM_CH_MASK;

  switch (((struct stm32l4_lptim_priv_s *)dev)->base)
    {
#if CONFIG_STM32L4_LPTIM1
      case STM32L4_LPTIM1_BASE:
        switch (channel)
          {
# if defined(GPIO_LPTIM1_OUT_1)
            case 1:
              *cfg = GPIO_LPTIM1_OUT_1;
              break;
# endif
# if defined(GPIO_LPTIM1_OUT_2)
            case 2:
              *cfg = GPIO_LPTIM1_OUT_2;
              break;
# endif
# if defined(GPIO_LPTIM1_OUT_3)
            case 3:
              *cfg = GPIO_LPTIM1_OUT_3;
              break;
# endif
            default:
              return ERROR;
          }
        break;
#endif /* CONFIG_STM32L4_LPTIM1 */

#if CONFIG_STM32L4_LPTIM2
      case STM32L4_LPTIM2_BASE:
        switch (channel)
          {
# if defined(GPIO_LPTIM2_OUT_1)
            case 1:
              *cfg = GPIO_LPTIM2_OUT_1;
              break;
# endif
# if defined(GPIO_LPTIM2_OUT_2)
            case 2:
              *cfg = GPIO_LPTIM2_OUT_2;
              break;
# endif
# if defined(GPIO_LPTIM2_OUT_3)
            case 3:
              *cfg = GPIO_LPTIM2_OUT_3;
              break;
# endif
            default:
              return ERROR;
          }
        break;
#endif /* CONFIG_STM32L4_LPTIM2 */

      default:
        return ERROR;
    }

  return OK;
}

/************************************************************************************
 * Name: stm32l4_lptim_setmode
 ************************************************************************************/

static int stm32l4_lptim_setmode(FAR struct stm32l4_lptim_dev_s *dev,
                                 stm32l4_lptim_mode_t mode)
{
  const uint32_t addr = ((struct stm32l4_lptim_priv_s *)dev)->base +
                        STM32L4_LPTIM_CR_OFFSET;

  DEBUGASSERT(dev != NULL);

  /* Mode */

  switch (mode & STM32L4_LPTIM_MODE_MASK)
    {
      case STM32L4_LPTIM_MODE_DISABLED:
          modifyreg32(addr, LPTIM_CR_ENABLE, 0);
          break;

      case STM32L4_LPTIM_MODE_SINGLE:
          modifyreg32(addr, 0, LPTIM_CR_ENABLE);
          modifyreg32(addr, 0, LPTIM_CR_SNGSTRT);
          break;

      case STM32L4_LPTIM_MODE_CONTINUOUS:
          modifyreg32(addr, 0, LPTIM_CR_ENABLE);
          modifyreg32(addr, 0, LPTIM_CR_CNTSTRT);
          break;

      default:
          return ERROR;
    }

  /* Save mode */

  ((struct stm32l4_lptim_priv_s *)dev)->mode = mode;

  return OK;
}

/************************************************************************************
 * Name: stm32l4_lptim_setclock
 ************************************************************************************/

static int stm32l4_lptim_setclock(FAR struct stm32l4_lptim_dev_s *dev,
                                  uint32_t freq)
{
  FAR struct stm32l4_lptim_priv_s *priv = (FAR struct stm32l4_lptim_priv_s *)dev;
  uint32_t setbits;
  uint32_t actual;

  DEBUGASSERT(dev != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      stm32l4_lptim_disable(dev);
      return 0;
    }

  if (freq >= priv->freq >> 0)
    {
      /* More than clock source.  This is as fast as we can go */

      setbits = LPTIM_CFGR_PRESCd1;
      actual = priv->freq >> 0;
    }
  else if (freq >= priv->freq >> 1)
    {
      setbits = LPTIM_CFGR_PRESCd2;
      actual = priv->freq >> 1;
    }
  else if (freq >= priv->freq >> 2)
    {
      setbits = LPTIM_CFGR_PRESCd4;
      actual = priv->freq >> 2;
    }
  else if (freq >= priv->freq >> 3)
    {
      setbits = LPTIM_CFGR_PRESCd8;
      actual = priv->freq >> 3;
    }
  else if (freq >= priv->freq >> 4)
    {
      setbits = LPTIM_CFGR_PRESCd16;
      actual = priv->freq >> 4;
    }
  else if (freq >= priv->freq >> 5)
    {
      setbits = LPTIM_CFGR_PRESCd32;
      actual = priv->freq >> 5;
    }
  else if (freq >= priv->freq >> 6)
    {
      setbits = LPTIM_CFGR_PRESCd64;
      actual = priv->freq >> 6;
    }
  else
    {
      /* This is as slow as we can go */

      setbits = LPTIM_CFGR_PRESCd128;
      actual = priv->freq >> 7;
    }

  stm32l4_modifyreg32(dev, STM32L4_LPTIM_CFGR_OFFSET, LPTIM_CFGR_PRESC_MASK,
                    setbits);
  stm32l4_lptim_enable(dev);

  return actual;
}

/************************************************************************************
 * Name: stm32l4_lptim_setchannel
 ************************************************************************************/

static int stm32l4_lptim_setchannel(FAR struct stm32l4_lptim_dev_s *dev,
                                    stm32l4_lptim_channel_t channel, int enable)
{
  int ret = OK;
  uint32_t cfg = 0;

  DEBUGASSERT(dev);

  /* Configure GPIOs */

  ret = stm32l4_lptim_get_gpioconfig(dev, channel, &cfg);
  if (!ret)
    {
      if (enable)
        {
          stm32l4_configgpio(cfg);
        }
      else
        {
          stm32l4_unconfiggpio(cfg);
        }
    }

  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_lptim_init
 ************************************************************************************/

FAR struct stm32l4_lptim_dev_s *stm32l4_lptim_init(int timer)
{
  struct stm32l4_lptim_dev_s *dev = NULL;

  /* Get structure and enable power */

  dev = stm32l4_lptim_getstruct(timer);
  if (!dev)
    {
      return NULL;
    }

  /* Is device already allocated */

  if (((struct stm32l4_lptim_priv_s *)dev)->mode != STM32L4_LPTIM_MODE_UNUSED)
    {
      return NULL;
    }

  /* Enable power */

  stm32l4_lptim_enable(dev);

  /* Reset timer */

  stm32l4_lptim_reset(dev);

  /* Mark it as used */

  ((struct stm32l4_lptim_priv_s *)dev)->mode = STM32L4_LPTIM_MODE_DISABLED;

  return dev;
}

/************************************************************************************
 * Name: stm32l4_lptim_deinit
 ************************************************************************************/

int stm32l4_lptim_deinit(FAR struct stm32l4_lptim_dev_s * dev)
{
  DEBUGASSERT(dev);

  /* Disable power */

  stm32l4_lptim_disable(dev);

  /* Mark it as free */

  ((struct stm32l4_lptim_priv_s *)dev)->mode = STM32L4_LPTIM_MODE_UNUSED;

  return OK;
}

#endif /* CONFIG_STM32L4_LPTIM1 || CONFIG_STM32L4_LPTIM2 */
