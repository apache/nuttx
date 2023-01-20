/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wdt.c
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
#include <assert.h>
#include <debug.h>
#include "xtensa.h"
#include "hardware/esp32_tim.h"
#include "hardware/esp32_rtccntl.h"
#include "esp32_wdt.h"
#include "esp32_irq.h"
#include "esp32_rtc.h"
#include "esp32_rtc_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers for converting from Q13.19 fixed-point format to float */

#define N 19
#define Q_TO_FLOAT(x) ((float)x/(float)(1<<N))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32_wdt_priv_s
  {
    struct esp32_wdt_ops_s *ops;
    uint32_t                    base;    /* WDT register base address */
    uint8_t                     cpu;     /* CPU ID */
    uint8_t                     periph;  /* Peripheral ID */
    uint8_t                     irq;     /* Interrupt ID */
    int                         cpuint;  /* CPU interrupt assigned to this wdt */
    bool                        inuse;   /* Flag indicating if this wdt is in use */
  };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* WDT registers access *****************************************************/

static void esp32_wdt_putreg(struct esp32_wdt_dev_s *dev,
                             uint32_t offset,
                             uint32_t value);
static void esp32_wdt_modifyreg32(struct esp32_wdt_dev_s *dev,
                                  uint32_t offset,
                                  uint32_t clearbits,
                                  uint32_t setbits);
static uint32_t esp32_wdt_getreg(struct esp32_wdt_dev_s *dev,
                                 uint32_t offset);

/* WDT operations ***********************************************************/

static int esp32_wdt_start(struct esp32_wdt_dev_s *dev);
static int esp32_wdt_stop(struct esp32_wdt_dev_s *dev);
static int esp32_wdt_enablewp(struct esp32_wdt_dev_s *dev);
static int esp32_wdt_disablewp(struct esp32_wdt_dev_s *dev);
static int esp32_wdt_pre(struct esp32_wdt_dev_s *dev, uint16_t value);
static int esp32_wdt_settimeout(struct esp32_wdt_dev_s *dev,
                                uint32_t value, uint8_t stage);
static int esp32_wdt_feed_dog(struct esp32_wdt_dev_s *dev);
static int esp32_wdt_set_stg_conf(struct esp32_wdt_dev_s *dev,
                                   uint8_t stage, uint8_t conf);
static uint16_t esp32_rtc_clk(struct esp32_wdt_dev_s *dev);
static int esp32_wdt_setisr(struct esp32_wdt_dev_s *dev, xcpt_t handler,
                            void * arg);
static int esp32_wdt_enableint(struct esp32_wdt_dev_s *dev);
static int esp32_wdt_disableint(struct esp32_wdt_dev_s *dev);
static int esp32_wdt_ackint(struct esp32_wdt_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32 WDT ops */

struct esp32_wdt_ops_s esp32_mwdt_ops =
{
  .start         = esp32_wdt_start,
  .stop          = esp32_wdt_stop,
  .enablewp      = esp32_wdt_enablewp,
  .disablewp     = esp32_wdt_disablewp,
  .pre           = esp32_wdt_pre,
  .settimeout    = esp32_wdt_settimeout,
  .feed          = esp32_wdt_feed_dog,
  .stg_conf      = esp32_wdt_set_stg_conf,
  .rtc_clk       = NULL,
  .setisr        = esp32_wdt_setisr,
  .enableint     = esp32_wdt_enableint,
  .disableint    = esp32_wdt_disableint,
  .ackint        = esp32_wdt_ackint,
};

struct esp32_wdt_ops_s esp32_rwdt_ops =
{
  .start         = esp32_wdt_start,
  .stop          = esp32_wdt_stop,
  .enablewp      = esp32_wdt_enablewp,
  .disablewp     = esp32_wdt_disablewp,
  .pre           = NULL,
  .settimeout    = esp32_wdt_settimeout,
  .feed          = esp32_wdt_feed_dog,
  .stg_conf      = esp32_wdt_set_stg_conf,
  .rtc_clk       = esp32_rtc_clk,
  .setisr        = esp32_wdt_setisr,
  .enableint     = esp32_wdt_enableint,
  .disableint    = esp32_wdt_disableint,
  .ackint        = esp32_wdt_ackint,
};

#ifdef CONFIG_ESP32_MWDT0

struct esp32_wdt_priv_s g_esp32_mwdt0_priv =
{
  .ops    = &esp32_mwdt_ops,
  .base   = TIMG_WDTCONFIG0_REG(0),
  .periph = ESP32_PERIPH_TG_WDT_LEVEL, /* Peripheral ID */
  .irq    = ESP32_IRQ_TG_WDT_LEVEL,    /* Interrupt ID */
  .cpuint = -ENOMEM,                   /* CPU interrupt assigned to this wdt */
  .inuse = false,
};
#endif

#ifdef CONFIG_ESP32_MWDT1

struct esp32_wdt_priv_s g_esp32_mwdt1_priv =
{
  .ops   = &esp32_mwdt_ops,
  .base  = TIMG_WDTCONFIG0_REG(1),
  .periph = ESP32_PERIPH_TG1_WDT_LEVEL, /* Peripheral ID */
  .irq    = ESP32_IRQ_TG1_WDT_LEVEL,    /* Interrupt ID */
  .cpuint = -ENOMEM,                    /* CPU interrupt assigned to this wdt */
  .inuse = false,
};
#endif

#ifdef CONFIG_ESP32_RWDT

struct esp32_wdt_priv_s g_esp32_rwdt_priv =
{
  .ops   = &esp32_rwdt_ops,
  .base  = RTC_CNTL_OPTIONS0_REG,
  .periph = ESP32_PERIPH_RTC_CORE,  /* Peripheral ID */
  .irq    = ESP32_IRQ_RTC_WDT,      /* Interrupt ID */
  .cpuint = -ENOMEM,                /* CPU interrupt assigned to this wdt */
  .inuse = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wdt_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static void esp32_wdt_putreg(struct esp32_wdt_dev_s *dev,
                             uint32_t offset,
                             uint32_t value)
{
  DEBUGASSERT(dev);

  putreg32(value, ((struct esp32_wdt_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: esp32_wdt_modifyreg32
 *
 * Description:
 *   Modify a reg of 32 bits
 *
 ****************************************************************************/

static void esp32_wdt_modifyreg32(struct esp32_wdt_dev_s *dev,
                                  uint32_t offset,
                                  uint32_t clearbits,
                                  uint32_t setbits)
{
  DEBUGASSERT(dev);

  modifyreg32(((struct esp32_wdt_priv_s *)dev)->base + offset,
                clearbits, setbits);
}

/****************************************************************************
 * Name: esp32_wdt_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static uint32_t esp32_wdt_getreg(struct esp32_wdt_dev_s *dev,
                                 uint32_t offset)
{
  DEBUGASSERT(dev);

  return getreg32(((struct esp32_wdt_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: esp32_wdt_start
 *
 * Description:
 *   Releases the counter
 *
 ****************************************************************************/

static int esp32_wdt_start(struct esp32_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wdt_priv_s *)dev)->base ==
        RTC_CNTL_OPTIONS0_REG)
    {
      esp32_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, 0, RTC_CNTL_WDT_EN);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, 0, TIMG_WDT_EN);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_set_stg_conf
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

static int esp32_wdt_set_stg_conf(struct esp32_wdt_dev_s *dev,
                                   uint8_t stage, uint8_t conf)
{
  uint32_t mask;
  DEBUGASSERT(dev);

    switch (stage)
    {
      case 0:
        {
          /* If it is a RWDT */

          if (((struct esp32_wdt_priv_s *)dev)->base ==
                RTC_CNTL_OPTIONS0_REG)
            {
              mask = (uint32_t)conf << RTC_CNTL_WDT_STG0_S;
              esp32_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG0_M, mask);
            }

          /* If it is a MWDT */

          else
            {
              mask = (uint32_t)conf << TIMG_WDT_STG0_S;
              esp32_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG0_M, mask);
            }
          break;
        }

      case 1:
        {
          /* If it is a RWDT */

          if (((struct esp32_wdt_priv_s *)dev)->base ==
                RTC_CNTL_OPTIONS0_REG)
            {
              mask = (uint32_t)conf << RTC_CNTL_WDT_STG1_S;
              esp32_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG1_M, mask);
            }

          /* If it is a MWDT */

          else
            {
              mask = (uint32_t)conf << TIMG_WDT_STG1_S;
              esp32_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG1_M, mask);
            }
          break;
        }

      case 2:
        {
          /* If it is a RWDT */

          if (((struct esp32_wdt_priv_s *)dev)->base ==
               RTC_CNTL_OPTIONS0_REG)
            {
              mask = (uint32_t)conf << RTC_CNTL_WDT_STG2_S;
              esp32_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG2_M, mask);
            }

          /* If it is a MWDT */

          else
            {
              mask = (uint32_t)conf << TIMG_WDT_STG2_S;
              esp32_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG2_M, mask);
            }
          break;
        }

      case 3:
        {
          /* If it is a RWDT */

          if (((struct esp32_wdt_priv_s *)dev)->base ==
               RTC_CNTL_OPTIONS0_REG)
            {
              mask = (uint32_t)conf << RTC_CNTL_WDT_STG3_S;
              esp32_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG3_M, mask);
            }

          /* If it is a MWDT */

          else
            {
              mask = (uint32_t)conf << TIMG_WDT_STG3_S;
              esp32_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG3_M, mask);
            }
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported stage %d\n", stage);
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_stop
 *
 * Description:
 *   Disables the watchdog
 *
 ****************************************************************************/

static int esp32_wdt_stop(struct esp32_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wdt_priv_s *)dev)->base == RTC_CNTL_OPTIONS0_REG)
    {
      esp32_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, RTC_CNTL_WDT_EN, 0);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_EN, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_enablewp
 *
 * Description:
 *   Enables write protection (WP) on registers against accidental writing.
 *   TRM recommends to change any WDT register through this sequence:
 *   - Disable WP
 *   - Do the op
 *   - Re-enable WP
 *
 ****************************************************************************/

static int esp32_wdt_enablewp(struct esp32_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wdt_priv_s *)dev)->base == RTC_CNTL_OPTIONS0_REG)
    {
      esp32_wdt_putreg(dev, RWDT_WP_REG, 0);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wdt_putreg(dev, MWDT_WP_REG, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_disablewp
 *
 * Description:
 *   Disables write protection (WP) on registers against accidental writing.
 *   TRM recommends to change any WDT register through this sequence:
 *   - Disable WP
 *   - Do the op
 *   - Re-enable WP
 *
 ****************************************************************************/

static int esp32_wdt_disablewp(struct esp32_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wdt_priv_s *)dev)->base == RTC_CNTL_OPTIONS0_REG)
    {
      esp32_wdt_putreg(dev, RWDT_WP_REG, WRITE_PROTECTION_KEY);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wdt_putreg(dev, MWDT_WP_REG, WRITE_PROTECTION_KEY);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_pre
 *
 * Description:
 *   Sets  a prescale value. The MWDT clock period is 12.5 ns * value (pre).
 *   NOTE: There's no prescaler register for RWDT and its source clock is
 *   clocked from the RTC slow clock.
 *
 ****************************************************************************/

static int esp32_wdt_pre(struct esp32_wdt_dev_s *dev, uint16_t pre)
{
  uint32_t mask = (uint32_t)pre << TIMG_WDT_CLK_PRESCALE_S;

  DEBUGASSERT(dev);

  esp32_wdt_modifyreg32(dev, MWDT_CLK_PRESCALE_OFFSET,
                        TIMG_WDT_CLK_PRESCALE_M, mask);

  return OK;
}

/****************************************************************************
 * Name: esp32_rtc_clk
 *
 * Description:
 *   Check the RTC clock source and return the necessary cycles to complete
 *   1 ms.
 *
 ****************************************************************************/

static uint16_t esp32_rtc_clk(struct esp32_wdt_dev_s *dev)
{
  enum esp32_rtc_slow_freq_e slow_clk_rtc;
  uint32_t period_13q19;
  float period;
  float cycles_ms;
  uint16_t cycles_ms_int;

  /* Calibration map: Maps each RTC SLOW_CLK source to the number
   * used to calibrate this source.
   */

  static const enum esp32_rtc_cal_sel_e cal_map[] =
  {
    RTC_CAL_RTC_MUX,
    RTC_CAL_32K_XTAL,
    RTC_CAL_8MD256
  };

  DEBUGASSERT(dev);

  /* Check which clock is sourcing the slow_clk_rtc */

  slow_clk_rtc = esp32_rtc_get_slow_clk();

  /* Get the slow_clk_rtc period in us in Q13.19 fixed point format */

  period_13q19 = esp32_rtc_clk_cal(cal_map[slow_clk_rtc],
                                   SLOW_CLK_CAL_CYCLES);

  /* Assert no error happened during the calibration */

  DEBUGASSERT(period_13q19 != 0);

  /* Convert from Q13.19 format to float */

  period = Q_TO_FLOAT(period_13q19);

  /* Get the number of cycles necessary to count 1 ms */

  cycles_ms = 1000.0 / period;

  /* Get the integer number of cycles */

  cycles_ms_int = (uint16_t)cycles_ms;

  return cycles_ms_int;
}

/****************************************************************************
 * Name: esp32_wdt_settimeout
 *
 * Description:
 *   Sets the wdt timeout.
 *
 ****************************************************************************/

static int esp32_wdt_settimeout(struct esp32_wdt_dev_s *dev,
                                uint32_t value, uint8_t stage)
{
  DEBUGASSERT(dev);

    switch (stage)
    {
      case 0:
        {
          /* If it is a RWDT */

          if (((struct esp32_wdt_priv_s *)dev)->base ==
                RTC_CNTL_OPTIONS0_REG)
            {
              esp32_wdt_putreg(dev, RWDT_STAGE0_TIMEOUT_OFFSET, value);
            }

          /* If it is a MWDT */

          else
            {
              esp32_wdt_putreg(dev, MWDT_STAGE0_TIMEOUT_OFFSET, value);
            }
          break;
        }

      case 1:
        {
          /* If it is a RWDT */

          if (((struct esp32_wdt_priv_s *)dev)->base ==
                RTC_CNTL_OPTIONS0_REG)
            {
              esp32_wdt_putreg(dev, RWDT_STAGE1_TIMEOUT_OFFSET, value);
            }

          /* If it is a MWDT */

          else
            {
              esp32_wdt_putreg(dev, MWDT_STAGE1_TIMEOUT_OFFSET, value);
            }
          break;
        }

      case 2:
        {
          /* If it is a RWDT */

          if (((struct esp32_wdt_priv_s *)dev)->base ==
                RTC_CNTL_OPTIONS0_REG)
            {
              esp32_wdt_putreg(dev, RWDT_STAGE2_TIMEOUT_OFFSET, value);
            }

          /* If it is a MWDT */

          else
            {
              esp32_wdt_putreg(dev, MWDT_STAGE2_TIMEOUT_OFFSET, value);
            }
          break;
        }

      case 3:
        {
          /* If it is a RWDT */

          if (((struct esp32_wdt_priv_s *)dev)->base ==
                RTC_CNTL_OPTIONS0_REG)
            {
              esp32_wdt_putreg(dev, RWDT_STAGE3_TIMEOUT_OFFSET, value);
            }

          /* If it is a MWDT */

          else
            {
              esp32_wdt_putreg(dev, MWDT_STAGE3_TIMEOUT_OFFSET, value);
            }
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported stage %d\n", stage);
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_feed_dog
 *
 * Description:
 *   Feeds the dog.  When software feeds the watchdog timer, it returns to
 *   stage 0 and its counter restarts from 0.
 *
 ****************************************************************************/

static int esp32_wdt_feed_dog(struct esp32_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wdt_priv_s *)dev)->base == RTC_CNTL_OPTIONS0_REG)
    {
      esp32_wdt_putreg(dev, RWDT_FEED_OFFSET , FEED_DOG);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wdt_putreg(dev, MWDT_FEED_OFFSET , FEED_DOG);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_setisr
 *
 * Description:
 *   Allocates a Level CPU Interrupt, connects the peripheral source to this
 *   Interrupt, register the callback and enables the Interruption. It does
 *   the opposite if the handler and arg are NULL.
 *
 ****************************************************************************/

static int esp32_wdt_setisr(struct esp32_wdt_dev_s *dev, xcpt_t handler,
                            void *arg)
{
  struct esp32_wdt_priv_s *wdt = NULL;
  int ret = OK;

  DEBUGASSERT(dev);

  wdt = (struct esp32_wdt_priv_s *)dev;

  /* Disable interrupt when callback is removed */

  if (handler == NULL)
    {
      /* If a CPU Interrupt was previously allocated, then deallocate it */

#ifdef CONFIG_ESP32_RWDT
      if (wdt->irq == ESP32_IRQ_RTC_WDT)
        {
          esp32_rtcioirqdisable(wdt->irq);
          irq_detach(wdt->irq);
        }
      else
#endif
        {
          if (wdt->cpuint >= 0)
            {
              /* Disable CPU Interrupt, free a previously allocated
               * CPU Interrupt
               */

              up_disable_irq(wdt->irq);
              esp32_teardown_irq(wdt->cpu, wdt->periph, wdt->cpuint);
              irq_detach(wdt->irq);
            }

          goto errout;
        }
    }

  /* Otherwise set callback and enable interrupt */

  else
    {
      /* Set up to receive peripheral interrupts on the current CPU */

#ifdef CONFIG_ESP32_RWDT
      if (wdt->irq == ESP32_IRQ_RTC_WDT)
        {
          ret = irq_attach(wdt->irq, handler, arg);

          if (ret != OK)
            {
              esp32_rtcioirqdisable(wdt->irq);
              tmrerr("ERROR: Failed to associate an IRQ Number");
              goto errout;
            }

          esp32_rtcioirqenable(wdt->irq);
        }
      else
#endif
        {
          wdt->cpu = up_cpu_index();
          wdt->cpuint = esp32_setup_irq(wdt->cpu, wdt->periph,
                                        1, ESP32_CPUINT_LEVEL);
          if (wdt->cpuint < 0)
            {
              tmrerr("ERROR: No CPU Interrupt available");
              ret = wdt->cpuint;
              goto errout;
            }

          /* Associate an IRQ Number (from the WDT) to an ISR */

          ret = irq_attach(wdt->irq, handler, arg);

          if (ret != OK)
            {
              esp32_teardown_irq(wdt->cpu, wdt->periph, wdt->cpuint);
              tmrerr("ERROR: Failed to associate an IRQ Number");
              goto errout;
            }

          /* Enable the CPU Interrupt that is linked to the wdt */

          up_enable_irq(wdt->irq);
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: esp32_wdt_enableint
 *
 * Description:
 *   Enables a Level Interrupt at timeout.
 *
 ****************************************************************************/

static int esp32_wdt_enableint(struct esp32_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Set the level interrupt bit */

  /* If it is a RWDT */

  if (((struct esp32_wdt_priv_s *)dev)->base == RTC_CNTL_OPTIONS0_REG)
    {
      /* Level Interrupt */

      esp32_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, 0,
                            RTC_CNTL_WDT_LEVEL_INT_EN);

      /* Enable Interrupt */

      esp32_wdt_modifyreg32(dev, RWDT_INT_ENA_REG_OFFSET, 0,
                            RTC_CNTL_WDT_INT_ENA);
    }

  /* If it is a MWDT */

  else
    {
      /* Level Interrupt */

      esp32_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, 0,
                            TIMG_WDT_LEVEL_INT_EN);

      /* Enable Interrupt */

      esp32_wdt_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET, 0,
                            TIMG_WDT_INT_ENA);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_disableint
 *
 * Description:
 *   Disables a Level Interrupt at timeout.
 *
 ****************************************************************************/

static int esp32_wdt_disableint(struct esp32_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wdt_priv_s *)dev)->base == RTC_CNTL_OPTIONS0_REG)
    {
      /* Level Interrupt */

      esp32_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                            RTC_CNTL_WDT_LEVEL_INT_EN, 0);

      /* Enable Interrupt */

      esp32_wdt_modifyreg32(dev, RWDT_INT_ENA_REG_OFFSET,
                            RTC_CNTL_WDT_INT_ENA, 0);
    }

  /* If it is a MWDT */

  else
    {
      /* Level Interrupt */

      esp32_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                            TIMG_WDT_LEVEL_INT_EN, 0);

      /* Enable Interrupt */

      esp32_wdt_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET,
                            TIMG_WDT_INT_ENA, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_ackint
 *
 *   Description:
 *   Acknowledges an interrupt
 *
 ****************************************************************************/

static int esp32_wdt_ackint(struct esp32_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wdt_priv_s *)dev)->base == RTC_CNTL_OPTIONS0_REG)
    {
      esp32_wdt_putreg(dev, RWDT_INT_CLR_REG_OFFSET, RTC_CNTL_WDT_INT_CLR);
    }

  /* If it is a MWDT */

  else
    {
      esp32_wdt_putreg(dev, MWDT_INT_CLR_REG_OFFSET, TIMG_WDT_INT_CLR);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wdt_init
 *
 * Description:
 *   Initialize WDT device
 *
 ****************************************************************************/

struct esp32_wdt_dev_s *esp32_wdt_init(uint8_t wdt_id)
{
  struct esp32_wdt_priv_s *wdt = NULL;

  /* Get wdt instance */

  switch (wdt_id)
    {
#ifdef CONFIG_ESP32_MWDT0
      case 0:
        {
          wdt = &g_esp32_mwdt0_priv;
          break;
        }

#endif
#ifdef CONFIG_ESP32_MWDT1
      case 1:
        {
          wdt = &g_esp32_mwdt1_priv;
          break;
        }

#endif
#ifdef CONFIG_ESP32_RWDT
      case 2:
        {
          wdt = &g_esp32_rwdt_priv;

          /* If RTC was not initialized in a previous
           * stage by the PM or by clock_initialize()
           * Then, init the RTC clock configuration here.
           */

#if !defined(CONFIG_PM) && !defined(CONFIG_RTC)
          /* Initialize RTC controller parameters */

          esp32_rtc_init();
          esp32_rtc_clk_set();
#endif
          break;
        }

#endif
      default:
        {
          tmrerr("ERROR: unsupported WDT %d\n", wdt_id);
          return NULL;
        }
    }

  /* If some code is using it then sends an error message,
   * Otherwise, inform it has been used.
   */

  if (wdt->inuse == true)
    {
      tmrerr("ERROR: WDT %d is already in use\n", wdt_id);
      return NULL;
    }
  else
    {
      wdt->inuse = true;
    }

  return (struct esp32_wdt_dev_s *)wdt;
}

/****************************************************************************
 * Name: esp32_wdt_early_deinit
 *
 * Description:
 *   Disable the WDT(s) that was/were enabled by the bootloader.
 *
 ****************************************************************************/

void esp32_wdt_early_deinit(void)
{
  uint32_t regval;
  putreg32(RTC_CNTL_WDT_WKEY_VALUE, RTC_CNTL_WDTWPROTECT_REG);
  regval  = getreg32(RTC_CNTL_WDTCONFIG0_REG);
  regval &= ~RTC_CNTL_WDT_EN;
  putreg32(regval, RTC_CNTL_WDTCONFIG0_REG);
  putreg32(0, RTC_CNTL_WDTWPROTECT_REG);
}

/****************************************************************************
 * Name: esp32_wdt_deinit
 *
 * Description:
 *   Deinit WDT device
 *
 ****************************************************************************/

int esp32_wdt_deinit(struct esp32_wdt_dev_s *dev)
{
  struct esp32_wdt_priv_s *wdt = NULL;

  DEBUGASSERT(dev);

  wdt = (struct esp32_wdt_priv_s *)dev;

  wdt->inuse = false;

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_is_running
 *
 * Description:
 *   Checks if the wdt was already turned on. For example, RTC may has been
 *   enabled in bootloader.
 *
 ****************************************************************************/

bool esp32_wdt_is_running(struct esp32_wdt_dev_s *dev)
{
  uint32_t status = 0;
  DEBUGASSERT(dev);

  /* If it is a RWDT */

  if (((struct esp32_wdt_priv_s *)dev)->base ==
        RTC_CNTL_OPTIONS0_REG)
    {
      status = esp32_wdt_getreg(dev, RWDT_CONFIG0_OFFSET);
      if (status & RTC_CNTL_WDT_EN)
        {
          return true;
        }
    }

  /* If it is a MWDT */

  else
    {
      status = esp32_wdt_getreg(dev, MWDT_CONFIG0_OFFSET);
      if (status & TIMG_WDT_EN)
        {
          return true;
        }
    }

  return false;
}
