/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wtd.c
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
#include <stdbool.h>
#include "xtensa.h"
#include "hardware/esp32_tim.h" 
#include "hardware/esp32_rtccntl.h"
#include "esp32_wtd.h"
#include "esp32_cpuint.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32_wtd_priv_s
  {
    FAR struct esp32_wtd_ops_s *ops;
    uint32_t                    base;    /* WTD register base address */
    uint8_t                     periph;  /* Peripheral ID */
    uint8_t                     irq;     /* Interrupt ID */
    int                         cpuint;  /* CPU interrupt assigned to this wtd */
    bool                        inuse;   /* Flag indicating if this wtd is in use */
  };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* WTD registers access *****************************************************/

static uint32_t esp32_wtd_getreg(FAR struct esp32_wtd_dev_s *dev,
                                 uint32_t offset);
static void esp32_wtd_putreg(FAR struct esp32_wtd_dev_s *dev,
                             uint32_t offset,
                             uint32_t value);
static void esp32_wtd_modifyreg32(FAR struct esp32_wtd_dev_s *dev,
                                  uint32_t offset,
                                  uint32_t clearbits,
                                  uint32_t setbits);

/* WTD operations ***********************************************************/

static int esp32_wtd_start(FAR struct esp32_wtd_dev_s *dev);
static int esp32_wtd_stop(FAR struct esp32_wtd_dev_s *dev);
static int esp32_wtd_enablewp(FAR struct esp32_wtd_dev_s *dev);
static int esp32_wtd_disablewp(FAR struct esp32_wtd_dev_s *dev);
static int esp32_wtd_initconf(FAR struct esp32_wtd_dev_s *dev);
static int esp32_wtd_pre(FAR struct esp32_wtd_dev_s *dev, uint16_t value);
static int esp32_wtd_settimeout(FAR struct esp32_wtd_dev_s *dev,
                                uint32_t value, uint8_t stage);
static int esp32_wtd_feed_dog(FAR struct esp32_wtd_dev_s *dev);
static int esp32_wtd_set_stg_conf(FAR struct esp32_wtd_dev_s *dev,
                                   uint8_t stage, uint8_t conf);
static uint16_t esp32_rtc_clk(FAR struct esp32_wtd_dev_s *dev);
static int esp32_wtd_setisr(FAR struct esp32_wtd_dev_s *dev, xcpt_t handler,
                            FAR void * arg);
static int esp32_wtd_enableint(FAR struct esp32_wtd_dev_s *dev);
static int esp32_wtd_disableint(FAR struct esp32_wtd_dev_s *dev);
static int esp32_wtd_ackint(FAR struct esp32_wtd_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32 WTD ops */

struct esp32_wtd_ops_s esp32_mwtd_ops =
{
  .start         = esp32_wtd_start,
  .stop          = esp32_wtd_stop,
  .initconf      = esp32_wtd_initconf,
  .enablewp      = esp32_wtd_enablewp,
  .disablewp     = esp32_wtd_disablewp,
  .pre           = esp32_wtd_pre,
  .settimeout    = esp32_wtd_settimeout,
  .feed          = esp32_wtd_feed_dog,
  .stg_conf      = esp32_wtd_set_stg_conf,
  .rtc_clk       = NULL,
  .setisr        = esp32_wtd_setisr,
  .enableint     = esp32_wtd_enableint,
  .disableint    = esp32_wtd_disableint,
  .ackint        = esp32_wtd_ackint,
};

struct esp32_wtd_ops_s esp32_rwtd_ops =
{
  .start         = esp32_wtd_start,
  .stop          = esp32_wtd_stop,
  .initconf      = esp32_wtd_initconf,
  .enablewp      = esp32_wtd_enablewp,
  .disablewp     = esp32_wtd_disablewp,
  .pre           = NULL,
  .settimeout    = esp32_wtd_settimeout,
  .feed          = esp32_wtd_feed_dog,
  .stg_conf      = esp32_wtd_set_stg_conf,
  .rtc_clk       = esp32_rtc_clk,
  .setisr        = esp32_wtd_setisr,
  .enableint     = esp32_wtd_enableint,
  .disableint    = esp32_wtd_disableint,
  .ackint        = esp32_wtd_ackint,
};

#ifdef CONFIG_ESP32_MWDT0

struct esp32_wtd_priv_s g_esp32_mwtd0_priv =
{
  .ops    = &esp32_mwtd_ops,
  .base   = TIMG_WDTCONFIG0_REG(0),
  .periph = ESP32_PERIPH_TG_WDT_LEVEL, /* Peripheral ID */
  .irq    = ESP32_IRQ_TG_WDT_LEVEL,    /* Interrupt ID */
  .cpuint = -ENOMEM,                   /* CPU interrupt assigned to this wtd */
  .inuse = false,
};
#endif

#ifdef CONFIG_ESP32_MWDT1

struct esp32_wtd_priv_s g_esp32_mwtd1_priv =
{
  .ops   = &esp32_mwtd_ops,
  .base  = TIMG_WDTCONFIG0_REG(1),
  .periph = ESP32_PERIPH_TG1_WDT_LEVEL, /* Peripheral ID */
  .irq    = ESP32_IRQ_TG1_WDT_LEVEL,    /* Interrupt ID */
  .cpuint = -ENOMEM,                    /* CPU interrupt assigned to this wtd */
  .inuse = false,
};
#endif

#ifdef CONFIG_ESP32_RWDT

struct esp32_wtd_priv_s g_esp32_rwtd_priv =
{
  .ops   = &esp32_rwtd_ops,
  .base  = RTC_CNTL_OPTIONS0_REG,
  .periph = ESP32_PERIPH_RTC_CORE,  /* Peripheral ID */
  .irq    = ESP32_IRQ_RTC_CORE,     /* Interrupt ID */
  .cpuint = -ENOMEM,                /* CPU interrupt assigned to this wtd */
  .inuse = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wtd_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static uint32_t esp32_wtd_getreg(FAR struct esp32_wtd_dev_s *dev,
                                 uint32_t offset)
{
  DEBUGASSERT(dev);

  return getreg32(((struct esp32_wtd_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: esp32_wtd_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static void esp32_wtd_putreg(FAR struct esp32_wtd_dev_s *dev,
                             uint32_t offset,
                             uint32_t value)
{
  DEBUGASSERT(dev);

  putreg32(value, ((struct esp32_wtd_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: esp32_wtd_modifyreg32
 *
 * Description:
 *   Modify a reg of 32 bits
 *
 ****************************************************************************/

static void esp32_wtd_modifyreg32(FAR struct esp32_wtd_dev_s *dev,
                                  uint32_t offset,
                                  uint32_t clearbits,
                                  uint32_t setbits)
{
  DEBUGASSERT(dev);

  modifyreg32(((struct esp32_wtd_priv_s *)dev)->base + offset,
                clearbits, setbits);
}

/****************************************************************************
 * Name: esp32_wtd_start
 *
 * Description:
 *   Releases the counter
 *
 ****************************************************************************/

static int esp32_wtd_start(FAR struct esp32_wtd_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wtd_priv_s *)dev)->base ==
        RTC_CNTL_WDTCONFIG0_REG)
    {
      esp32_wtd_modifyreg32(dev, RWDT_CONFIG0_OFFSET, 0, RTC_CNTL_WDT_EN);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wtd_modifyreg32(dev, MWDT_CONFIG0_OFFSET, 0, TIMG_WDT_EN);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wtd_set_stg_conf
 *
 * Description:
 *   It configures the action to be triggered by a stage on expiration.
 *   The available actions are:
 *   0 - off
 *   1 - interrupt
 *   2 - reset CPU
 *   3 - reset main System (CPU + Peripherals - RTC)
 *   4 - reset main System and RTC ( NOTE: only available for RWDT)
 *
 ****************************************************************************/

static int esp32_wtd_set_stg_conf(FAR struct esp32_wtd_dev_s *dev,
                                   uint8_t stage, uint8_t conf)
{
  int ret = OK;
  uint32_t mask;
  DEBUGASSERT(dev);

    switch (stage)
    {
      case 0:
        {
          /* If it is a RWDT */

          if (((struct esp32_wtd_priv_s *)dev)->base ==
                RTC_CNTL_WDTCONFIG0_REG)
            {
              mask = (uint32_t)conf << RTC_CNTL_WDT_STG0_S;
              esp32_wtd_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG0_M, mask);
            }

          /* If it is a MWDT */

          else
            {
              mask = (uint32_t)conf << TIMG_WDT_STG0_S;
              esp32_wtd_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG0_M, mask);
            }
          break;
        }

      case 1:
        {
          /* If it is a RWDT */

          if (((struct esp32_wtd_priv_s *)dev)->base ==
                RTC_CNTL_WDTCONFIG0_REG)
            {
              mask = (uint32_t)conf << RTC_CNTL_WDT_STG1_S;
              esp32_wtd_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG1_M, mask);
            }

          /* If it is a MWDT */

          else
            {
              mask = (uint32_t)conf << TIMG_WDT_STG1_S;
              esp32_wtd_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG1_M, mask);
            }
          break;
        }

      case 2:
        {
          /* If it is a RWDT */

          if (((struct esp32_wtd_priv_s *)dev)->base ==
               RTC_CNTL_WDTCONFIG0_REG)
            {
              mask = (uint32_t)conf << RTC_CNTL_WDT_STG2_S;
              esp32_wtd_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG2_M, mask);
            }

          /* If it is a MWDT */

          else
            {
              mask = (uint32_t)conf << TIMG_WDT_STG2_S;
              esp32_wtd_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG2_M, mask);
            }
          break;
        }

      case 3:
        {
          /* If it is a RWDT */

          if (((struct esp32_wtd_priv_s *)dev)->base ==
               RTC_CNTL_WDTCONFIG0_REG)
            {
              mask = (uint32_t)conf << RTC_CNTL_WDT_STG3_S;
              esp32_wtd_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG3_M, mask);
            }

          /* If it is a MWDT */

          else
            {
              mask = (uint32_t)conf << TIMG_WDT_STG3_S;
              esp32_wtd_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG3_M, mask);
            }
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported stage %d\n", stage);
          ret = EINVAL;
          goto errout;
        }
    }

  errout:
    return ret;
}

/****************************************************************************
 * Name: esp32_wtd_stop
 *
 * Description:
 *   Disables the watchdog
 *
 ****************************************************************************/

static int esp32_wtd_stop(FAR struct esp32_wtd_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wtd_priv_s *)dev)->base == RTC_CNTL_WDTCONFIG0_REG)
    {
      esp32_wtd_modifyreg32(dev, RWDT_CONFIG0_OFFSET, RTC_CNTL_WDT_EN, 0);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wtd_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_EN, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wtd_enablewp
 *
 * Description:
 *   Enables write protection (WP) on registers against accidental writing.
 *   TRM recommends to change any WDT register thorugh this sequence:
 *   - Disable WP
 *   - Do the op
 *   - Reenable WP
 *
 ****************************************************************************/

static int esp32_wtd_enablewp(FAR struct esp32_wtd_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wtd_priv_s *)dev)->base == RTC_CNTL_WDTCONFIG0_REG)
    {
      esp32_wtd_putreg(dev, RWDT_WP_REG, 0);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wtd_putreg(dev, MWDT_WP_REG, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wtd_disablewp
 *
 * Description:
 *   Disables write protection (WP) on registers against accidental writing.
 *   TRM recommends to change any WDT register thorugh this sequence:
 *   - Disable WP
 *   - Do the op
 *   - Reenable WP
 *
 ****************************************************************************/

static int esp32_wtd_disablewp(FAR struct esp32_wtd_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wtd_priv_s *)dev)->base == RTC_CNTL_WDTCONFIG0_REG)
    {
      esp32_wtd_putreg(dev, RWDT_WP_REG, WRITE_PROTECTION_KEY);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wtd_putreg(dev, MWDT_WP_REG, WRITE_PROTECTION_KEY);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wtd_initconf
 *
 * Description:
 *   It turn off all the stages and ensure Flash Boot Protection is disabled.
 *   In case of RWDT, it also turns off the WDT, in case it was already
 *   turned on before. NOTE: The main system reset does not reset RTC, so
 *   all the registers values are kept.
 *
 ****************************************************************************/

static int esp32_wtd_initconf(FAR struct esp32_wtd_dev_s *dev)
{
  uint32_t mask = 0;

  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wtd_priv_s *)dev)->base == RTC_CNTL_WDTCONFIG0_REG)
    {
      mask = RTC_CNTL_WDT_INT_ENA_M | RTC_CNTL_WDT_STG0_M
       | RTC_CNTL_WDT_STG1_M | RTC_CNTL_WDT_STG2_M | RTC_CNTL_WDT_STG3_M
       | RTC_CNTL_WDT_FLASHBOOT_MOD_EN_M;
      esp32_wtd_modifyreg32(dev, RWDT_CONFIG0_OFFSET, mask, 0);
    }

  /* If it is a MWDT */

  else
    {
      mask = TIMG_WDT_STG0_M | TIMG_WDT_STG1_M | TIMG_WDT_STG2_M
       | TIMG_WDT_STG3_M | TIMG_WDT_FLASHBOOT_MOD_EN_M;
      esp32_wtd_modifyreg32(dev, MWDT_CONFIG0_OFFSET, mask, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wtd_pre
 *
 * Description:
 *   Sets  a prescale value. The MWDT clock period is 12.5 ns * value (pre).
 *   NOTE: There's no prescaler register for RWDT and its source clock is
 *   clocked from the RTC slow clock.
 *
 ****************************************************************************/

static int esp32_wtd_pre(FAR struct esp32_wtd_dev_s *dev, uint16_t pre)
{
  uint32_t mask = (uint32_t)pre << TIMG_WDT_CLK_PRESCALE_S;

  DEBUGASSERT(dev);

  esp32_wtd_modifyreg32(dev, MWDT_CLK_PRESCALE_OFFSET,
                        TIMG_WDT_CLK_PRESCALE_M, mask);

  return OK;
}

/****************************************************************************
 * Name: esp32_rtc_clk
 *
 * Description:
 * Check the RTC clock source and return the necessary cycles to complete
 * 1 ms. NOTE: TODO.
 *
 ****************************************************************************/

static uint16_t esp32_rtc_clk(FAR struct esp32_wtd_dev_s *dev)
{
  uint32_t reg_value = 0;
  uint8_t cycles_ms = 0;
  uint32_t corrected_frequency = 0;

  DEBUGASSERT(dev);

  reg_value = esp32_wtd_getreg(dev, RCLK_CONF_REG_OFFSET);

  if ((reg_value & CK8M_D256_OUT_MASK) == CK8M_D256_OUT_MASK)
    {
      /* TODO: get the correct RTC frequency using the RTC driver API */
    }
  else if ((reg_value & CK_XTAL_32K_MASK) == CK_XTAL_32K_MASK)
    {
      /* TODO: get the correct RTC frequency using the RTC driver API */
    }
  else
    {
      /* TODO: get the correct RTC frequency using the RTC driver API */
    }

  return cycles_ms = (uint8_t)(corrected_frequency / 1000);
}

/****************************************************************************
 * Name: esp32_wtd_settimeout
 *
 * Description:
 *   Sets the wdt timeout.
 *
 ****************************************************************************/

static int esp32_wtd_settimeout(FAR struct esp32_wtd_dev_s *dev,
                                uint32_t value, uint8_t stage)
{
  int ret = OK;
  DEBUGASSERT(dev);

    switch (stage)
    {
      case 0:
        {
          /* If it is a RWDT */

          if (((struct esp32_wtd_priv_s *)dev)->base ==
                RTC_CNTL_WDTCONFIG0_REG)
            {
              esp32_wtd_putreg(dev, RWDT_STAGE0_TIMEOUT_OFFSET, value);
            }

          /* If it is a MWDT */

          else
            {
              esp32_wtd_putreg(dev, MWDT_STAGE0_TIMEOUT_OFFSET, value);
            }
          break;
        }

      case 1:
        {
          /* If it is a RWDT */

          if (((struct esp32_wtd_priv_s *)dev)->base ==
                RTC_CNTL_WDTCONFIG0_REG)
            {
              esp32_wtd_putreg(dev, RWDT_STAGE1_TIMEOUT_OFFSET, value);
            }

          /* If it is a MWDT */

          else
            {
              esp32_wtd_putreg(dev, MWDT_STAGE1_TIMEOUT_OFFSET, value);
            }
          break;
        }

      case 2:
        {
          /* If it is a RWDT */

          if (((struct esp32_wtd_priv_s *)dev)->base ==
                RTC_CNTL_WDTCONFIG0_REG)
            {
              esp32_wtd_putreg(dev, RWDT_STAGE2_TIMEOUT_OFFSET, value);
            }

          /* If it is a MWDT */

          else
            {
              esp32_wtd_putreg(dev, MWDT_STAGE2_TIMEOUT_OFFSET, value);
            }
          break;
        }

      case 3:
        {
          /* If it is a RWDT */

          if (((struct esp32_wtd_priv_s *)dev)->base ==
                RTC_CNTL_WDTCONFIG0_REG)
            {
              esp32_wtd_putreg(dev, RWDT_STAGE3_TIMEOUT_OFFSET, value);
            }

          /* If it is a MWDT */

          else
            {
              esp32_wtd_putreg(dev, MWDT_STAGE3_TIMEOUT_OFFSET, value);
            }
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported stage %d\n", stage);
          ret = EINVAL;
          goto errout;
        }
    }

  errout:
    return ret;
}

/****************************************************************************
 * Name: esp32_wtd_feed_dog
 *
 * Description:
 *   Feeds the dog.  When software feeds the watchdog timer, it returns to
 *   stage 0 and its counter restarts from 0.
 *
 ****************************************************************************/

static int esp32_wtd_feed_dog(FAR struct esp32_wtd_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wtd_priv_s *)dev)->base == RTC_CNTL_WDTCONFIG0_REG)
    {
      esp32_wtd_putreg(dev, RWDT_FEED_OFFSET , FEED_DOG);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wtd_putreg(dev, MWDT_FEED_OFFSET , FEED_DOG);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wtd_setisr
 *
 * Description:
 *   Allocates a Level CPU Interrupt, connects the peripheral source to this
 *   Interrupt, register the callback and enables the Interruption. It does
 *   the opposite if the handler and arg are NULL.
 *
 ****************************************************************************/

static int esp32_wtd_setisr(FAR struct esp32_wtd_dev_s *dev, xcpt_t handler,
                            FAR void *arg)
{
  FAR struct esp32_wtd_priv_s *wtd = NULL;
  int ret = OK;
  uint8_t cpu;

  DEBUGASSERT(dev);

  wtd = (FAR struct esp32_wtd_priv_s *)dev;

  /* Disable interrupt when callback is removed */

  if (handler == NULL)
    {
      /* If a CPU Interrupt was previously allocated, then deallocate it */

      if (wtd->cpuint >= 0)
        {
          /* Disable CPU Interrupt, free a previously allocated
           * CPU Interrupt
           */

          up_disable_irq(wtd->cpuint);
          cpu = up_cpu_index();
          esp32_detach_peripheral(cpu, wtd->periph, wtd->cpuint);
          esp32_free_cpuint(wtd->cpuint);
          irq_detach(wtd->irq);
        }

      ret = OK;
      goto errout;
    }

  /* Otherwise set callback and enable interrupt */

  else
    {
      /* Verify the available CPU Interrupt */

      wtd->cpuint = esp32_alloc_levelint(1);
      if (wtd->cpuint < 0)
        {
          tmrerr("ERROR: No CPU Interrupt available");
          ret = wtd->cpuint;
          goto errout;
        }

      /* Disable the provided CPU Interrupt to configure it */

      up_disable_irq(wtd->cpuint);

      /* Attach a peripheral interrupt to the available CPU interrupt in
       * the current core
       */

      cpu = up_cpu_index();
      esp32_attach_peripheral(cpu, wtd->periph, wtd->cpuint);

      /* Associate an IRQ Number (from the WDT) to an ISR */

      ret = irq_attach(wtd->irq, handler, arg);

      if (ret != OK)
        {
          esp32_detach_peripheral(cpu, wtd->periph, wtd->cpuint);
          esp32_free_cpuint(wtd->cpuint);
          tmrerr("ERROR: Failed to associate an IRQ Number");
          goto errout;
        }

      /* Enable the CPU Interrupt that is linked to the wdt */

      up_enable_irq(wtd->cpuint);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: esp32_wtd_enableint
 *
 * Description:
 *   Enables a Level Interrupt at timeout.
 *
 ****************************************************************************/

static int esp32_wtd_enableint(FAR struct esp32_wtd_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Set the level interrupt bit */

  /* If it is a RWDT */

  if (((struct esp32_wtd_priv_s *)dev)->base == RTC_CNTL_WDTCONFIG0_REG)
    {
      /* Level Interrupt */

      esp32_wtd_modifyreg32(dev, RWDT_CONFIG0_OFFSET, 0,
                            RTC_CNTL_WDT_LEVEL_INT_EN);

      /* Enable Interrupt */

      esp32_wtd_modifyreg32(dev, RWDT_INT_ENA_REG_OFFSET, 0,
                            RTC_CNTL_WDT_INT_ENA);
    }

  /* If it is a MWDT */

  else
    {
      /* Level Interrupt */

      esp32_wtd_modifyreg32(dev, MWDT_CONFIG0_OFFSET, 0,
                            TIMG_WDT_LEVEL_INT_EN);

      /* Enable Interrupt */

      esp32_wtd_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET, 0,
                            TIMG_WDT_INT_ENA);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wtd_disableint
 *
 * Description:
 *   Disables a Level Interrupt at timeout.
 *
 ****************************************************************************/

static int esp32_wtd_disableint(FAR struct esp32_wtd_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wtd_priv_s *)dev)->base == RTC_CNTL_WDTCONFIG0_REG)
    {
      /* Level Interrupt */

      esp32_wtd_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                            RTC_CNTL_WDT_LEVEL_INT_EN, 0);

      /* Enable Interrupt */

      esp32_wtd_modifyreg32(dev, RWDT_INT_ENA_REG_OFFSET,
                            RTC_CNTL_WDT_INT_ENA, 0);
    }

  /* If it is a MWDT */

  else
    {
      /* Level Interrupt */

      esp32_wtd_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                            TIMG_WDT_LEVEL_INT_EN, 0);

      /* Enable Interrupt */

      esp32_wtd_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET,
                            TIMG_WDT_INT_ENA, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wtd_ackint
 *
 *   Description:
 *   Acknowledges an interrupt
 *
 ****************************************************************************/

static int esp32_wtd_ackint(FAR struct esp32_wtd_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wtd_priv_s *)dev)->base == RTC_CNTL_WDTCONFIG0_REG)
    {
      esp32_wtd_putreg(dev, RWDT_INT_CLR_REG_OFFSET, RTC_CNTL_WDT_INT_CLR);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wtd_putreg(dev, MWDT_INT_CLR_REG_OFFSET, TIMG_WDT_INT_CLR);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wtd_init
 *
 * Description:
 *   Initialize WDT device
 *
 ****************************************************************************/

FAR struct esp32_wtd_dev_s *esp32_wtd_init(uint8_t wdt_id)
{
  FAR struct esp32_wtd_priv_s *wtd = NULL;

  /* Get wdt instance */

  switch (wdt_id)
    {
#ifdef CONFIG_ESP32_MWDT0
      case 0:
        {
          wtd = &g_esp32_mwtd0_priv;
          break;
        }

#endif
#ifdef CONFIG_ESP32_MWDT1
      case 1:
        {
          wtd = &g_esp32_mwtd1_priv;
          break;
        }

#endif
#ifdef CONFIG_ESP32_RWDT
      case 2:
        {
          wtd = &g_esp32_rwtd_priv;
          break;
        }

#endif
      default:
        {
          tmrerr("ERROR: unsupported WDT %d\n", wdt_id);
          goto errout;
        }
    }

  /* If some code is using it then sends an error message,
   * Otherwise, inform it has been used.
   */

  if (wtd->inuse == true)
    {
      tmrerr("ERROR: WDT %d is already in use\n", wdt_id);
      wtd = NULL;
    }
  else
    {
      wtd->inuse = true;
    }

  errout:
    return (FAR struct esp32_wtd_dev_s *)wtd;
}

/****************************************************************************
 * Name: esp32_wtd_deinit
 *
 * Description:
 *   Deinit WDT device
 *
 ****************************************************************************/

int esp32_wtd_deinit(FAR struct esp32_wtd_dev_s *dev)
{
  FAR struct esp32_wtd_priv_s *wtd = NULL;

  DEBUGASSERT(dev);

  wtd = (FAR struct esp32_wtd_priv_s *)dev;

  wtd->inuse = false;

  return OK;
}
