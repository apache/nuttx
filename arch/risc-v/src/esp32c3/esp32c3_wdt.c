/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_wdt.c
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

#include "riscv_internal.h"
#include "hardware/esp32c3_rtccntl.h"
#include "hardware/esp32c3_tim.h"
#include "hardware/esp32c3_efuse.h"

#include "esp32c3_irq.h"
#include "esp32c3_rtc.h"
#include "esp32c3_rtc_gpio.h"
#include "esp32c3_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers for converting from Q13.19 fixed-point format to float */

#define N 19
#define Q_TO_FLOAT(x) ((float)x/(float)(1<<N))

/* Check whether the provided device is a RTC Watchdog Timer */

#define IS_RWDT(dev)    ((struct esp32c3_wdt_priv_s *)dev)->base == \
                        RTC_CNTL_OPTIONS0_REG

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32c3_wdt_priv_s
{
  struct esp32c3_wdt_ops_s *ops;
  uint32_t                  base;    /* WDT register base address */
  uint8_t                   periph;  /* Peripheral ID */
  uint8_t                   irq;     /* Interrupt ID */
  int32_t                   cpuint;  /* CPU interrupt assigned to this WDT */
  bool                      inuse;   /* Flag indicating if this WDT is in use */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* WDT registers access *****************************************************/

static void esp32c3_wdt_putreg(struct esp32c3_wdt_dev_s *dev,
                               uint32_t offset,
                               uint32_t value);
static void esp32c3_wdt_modifyreg32(struct esp32c3_wdt_dev_s *dev,
                                    uint32_t offset,
                                    uint32_t clearbits,
                                    uint32_t setbits);
static uint32_t esp32c3_wdt_getreg(struct esp32c3_wdt_dev_s *dev,
                                   uint32_t offset);

/* WDT operations ***********************************************************/

static void esp32c3_wdt_start(struct esp32c3_wdt_dev_s *dev);
static void esp32c3_wdt_stop(struct esp32c3_wdt_dev_s *dev);
static void esp32c3_wdt_enablewp(struct esp32c3_wdt_dev_s *dev);
static void esp32c3_wdt_disablewp(struct esp32c3_wdt_dev_s *dev);
static void esp32c3_wdt_pre(struct esp32c3_wdt_dev_s *dev,
                            uint16_t value);
static int32_t esp32c3_wdt_settimeout(struct esp32c3_wdt_dev_s *dev,
                                      uint32_t value,
                                      enum esp32c3_wdt_stage_e stage);
static void esp32c3_wdt_feed(struct esp32c3_wdt_dev_s *dev);
static int32_t esp32c3_wdt_config_stage(struct esp32c3_wdt_dev_s *dev,
                                        enum esp32c3_wdt_stage_e stage,
                                        enum esp32c3_wdt_stage_action_e cfg);
static void esp32c3_wdt_update_conf(struct esp32c3_wdt_dev_s *dev);
static uint16_t esp32c3_wdt_rtc_clk(struct esp32c3_wdt_dev_s *dev);
static int32_t esp32c3_wdt_setisr(struct esp32c3_wdt_dev_s *dev,
                                  xcpt_t handler, void *arg);
static void esp32c3_wdt_enableint(struct esp32c3_wdt_dev_s *dev);
static void esp32c3_wdt_disableint(struct esp32c3_wdt_dev_s *dev);
static void esp32c3_wdt_ackint(struct esp32c3_wdt_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32-C3 WDT ops */

struct esp32c3_wdt_ops_s esp32c3_mwdt_ops =
{
  .start         = esp32c3_wdt_start,
  .stop          = esp32c3_wdt_stop,
  .enablewp      = esp32c3_wdt_enablewp,
  .disablewp     = esp32c3_wdt_disablewp,
  .pre           = esp32c3_wdt_pre,
  .settimeout    = esp32c3_wdt_settimeout,
  .feed          = esp32c3_wdt_feed,
  .stg_conf      = esp32c3_wdt_config_stage,
  .upd_conf      = esp32c3_wdt_update_conf,
  .rtc_clk       = NULL,
  .setisr        = esp32c3_wdt_setisr,
  .enableint     = esp32c3_wdt_enableint,
  .disableint    = esp32c3_wdt_disableint,
  .ackint        = esp32c3_wdt_ackint,
};

struct esp32c3_wdt_ops_s esp32c3_rwdt_ops =
{
  .start         = esp32c3_wdt_start,
  .stop          = esp32c3_wdt_stop,
  .enablewp      = esp32c3_wdt_enablewp,
  .disablewp     = esp32c3_wdt_disablewp,
  .pre           = NULL,
  .settimeout    = esp32c3_wdt_settimeout,
  .feed          = esp32c3_wdt_feed,
  .stg_conf      = esp32c3_wdt_config_stage,
  .upd_conf      = NULL,
  .rtc_clk       = esp32c3_wdt_rtc_clk,
  .setisr        = esp32c3_wdt_setisr,
  .enableint     = esp32c3_wdt_enableint,
  .disableint    = esp32c3_wdt_disableint,
  .ackint        = esp32c3_wdt_ackint,
};

#ifdef CONFIG_ESP32C3_MWDT0
struct esp32c3_wdt_priv_s g_esp32c3_mwdt0_priv =
{
  .ops    = &esp32c3_mwdt_ops,
  .base   = TIMG_T0CONFIG_REG(0),
  .periph = ESP32C3_PERIPH_TG0_WDT,
  .irq    = ESP32C3_IRQ_TG0_WDT,
  .cpuint = -ENOMEM,
  .inuse  = false,
};
#endif

#ifdef CONFIG_ESP32C3_MWDT1
struct esp32c3_wdt_priv_s g_esp32c3_mwdt1_priv =
{
  .ops    = &esp32c3_mwdt_ops,
  .base   = TIMG_T0CONFIG_REG(1),
  .periph = ESP32C3_PERIPH_TG1_WDT,
  .irq    = ESP32C3_IRQ_TG1_WDT,
  .cpuint = -ENOMEM,
  .inuse  = false,
};
#endif

#ifdef CONFIG_ESP32C3_RWDT
struct esp32c3_wdt_priv_s g_esp32c3_rwdt_priv =
{
  .ops    = &esp32c3_rwdt_ops,
  .base   = RTC_CNTL_OPTIONS0_REG,
  .periph = ESP32C3_PERIPH_RTC_CORE,
  .irq    = ESP32C3_IRQ_RTC_WDT,
  .cpuint = -ENOMEM,
  .inuse  = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_wdt_putreg
 *
 * Description:
 *   Write a 32-bit register value by offset.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *   offset        - Offset value to the base address of the WDT device.
 *   value         - Value to written to the specified memory region.
 *
 ****************************************************************************/

static void esp32c3_wdt_putreg(struct esp32c3_wdt_dev_s *dev,
                               uint32_t offset,
                               uint32_t value)
{
  DEBUGASSERT(dev);

  putreg32(value, ((struct esp32c3_wdt_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: esp32c3_wdt_modifyreg32
 *
 * Description:
 *   Atomically modify a 32-bit register value by offset.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *   offset        - Offset value to the base address of the WDT device.
 *   clearbits     - Bits to be cleared on the specified memory region.
 *   setbits       - Bits to be set on the specified memory region.
 *
 ****************************************************************************/

static void esp32c3_wdt_modifyreg32(struct esp32c3_wdt_dev_s *dev,
                                    uint32_t offset,
                                    uint32_t clearbits,
                                    uint32_t setbits)
{
  DEBUGASSERT(dev);

  modifyreg32(((struct esp32c3_wdt_priv_s *)dev)->base + offset,
                clearbits, setbits);
}

/****************************************************************************
 * Name: esp32c3_wdt_getreg
 *
 * Description:
 *   Read a 32-bit register value by offset.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *   offset        - Offset value to the base address of the WDT device.
 *
 *  Returned Values:
 *    A 32-bit value from the provided memory region of the WDT device.
 *
 ****************************************************************************/

static uint32_t esp32c3_wdt_getreg(struct esp32c3_wdt_dev_s *dev,
                                   uint32_t offset)
{
  DEBUGASSERT(dev);

  return getreg32(((struct esp32c3_wdt_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: esp32c3_wdt_start
 *
 * Description:
 *   Release the counter.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void esp32c3_wdt_start(struct esp32c3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  if (IS_RWDT(dev))
    {
      esp32c3_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, 0, RTC_CNTL_WDT_EN);
    }
  else
    {
      esp32c3_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, 0, TIMG_WDT_EN);
    }
}

/****************************************************************************
 * Name: esp32c3_wdt_config_stage
 *
 * Description:
 *   Configure the action to be triggered by a stage on expiration.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *   stage         - WDT stage to be configured.
 *   cfg           - Action to be executed on stage expiration.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

static int32_t esp32c3_wdt_config_stage(struct esp32c3_wdt_dev_s *dev,
                                        enum esp32c3_wdt_stage_e stage,
                                        enum esp32c3_wdt_stage_action_e cfg)
{
  uint32_t mask;
  DEBUGASSERT(dev);

  switch (stage)
  {
    case ESP32C3_WDT_STAGE0:
      {
        if (IS_RWDT(dev))
          {
            mask = (uint32_t)cfg << RTC_CNTL_WDT_STG0_S;
            esp32c3_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG0_M, mask);
          }
        else
          {
            mask = (uint32_t)cfg << TIMG_WDT_STG0_S;
            esp32c3_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG0_M, mask);
          }
        break;
      }

    case ESP32C3_WDT_STAGE1:
      {
        if (IS_RWDT(dev))
          {
            mask = (uint32_t)cfg << RTC_CNTL_WDT_STG1_S;
            esp32c3_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG1_M, mask);
          }
        else
          {
            mask = (uint32_t)cfg << TIMG_WDT_STG1_S;
            esp32c3_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG1_M, mask);
          }
        break;
      }

    case ESP32C3_WDT_STAGE2:
      {
        if (IS_RWDT(dev))
          {
            mask = (uint32_t)cfg << RTC_CNTL_WDT_STG2_S;
            esp32c3_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG2_M, mask);
          }
        else
          {
            mask = (uint32_t)cfg << TIMG_WDT_STG2_S;
            esp32c3_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG2_M, mask);
          }
        break;
      }

    case ESP32C3_WDT_STAGE3:
      {
        if (IS_RWDT(dev))
          {
            mask = (uint32_t)cfg << RTC_CNTL_WDT_STG3_S;
            esp32c3_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET,
                                    RTC_CNTL_WDT_STG3_M, mask);
          }
        else
          {
            mask = (uint32_t)cfg << TIMG_WDT_STG3_S;
            esp32c3_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET,
                                    TIMG_WDT_STG3_M, mask);
          }
        break;
      }

    default:
      {
        wderr("ERROR: unsupported stage %d\n", stage);
        return -EINVAL;
      }
  }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_wdt_update_conf
 *
 * Description:
 *   Update the Watchdog Timer configuration registers.
 *   Configuration registers for the MWDT are updated asynchronously.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void esp32c3_wdt_update_conf(struct esp32c3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  esp32c3_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, 0,
                          TIMG_WDT_CONF_UPDATE_EN);
}

/****************************************************************************
 * Name: esp32c3_wdt_stop
 *
 * Description:
 *   Disable the watchdog.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void esp32c3_wdt_stop(struct esp32c3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  if (IS_RWDT(dev))
    {
      esp32c3_wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, RTC_CNTL_WDT_EN, 0);
    }
  else
    {
      esp32c3_wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_EN, 0);
    }
}

/****************************************************************************
 * Name: esp32c3_wdt_enablewp
 *
 * Description:
 *   Enable write protection (WP) on registers against accidental writing.
 *   TRM recommends to change any WDT register through this sequence:
 *   - Disable WP
 *   - Do the op
 *   - Re-enable WP
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void esp32c3_wdt_enablewp(struct esp32c3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  if (IS_RWDT(dev))
    {
      esp32c3_wdt_putreg(dev, RWDT_WP_REG, 0);
    }
  else
    {
      esp32c3_wdt_putreg(dev, MWDT_WP_REG, 0);
    }
}

/****************************************************************************
 * Name: esp32c3_wdt_disablewp
 *
 * Description:
 *   Disable write protection (WP) on registers against accidental writing.
 *   TRM recommends to change any WDT register through this sequence:
 *   - Disable WP
 *   - Do the op
 *   - Re-enable WP
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void esp32c3_wdt_disablewp(struct esp32c3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  if (IS_RWDT(dev))
    {
      esp32c3_wdt_putreg(dev, RWDT_WP_REG, RTC_CNTL_WDT_WKEY_VALUE);
    }
  else
    {
      esp32c3_wdt_putreg(dev, MWDT_WP_REG, TIMG_WDT_WKEY_VALUE);
    }
}

/****************************************************************************
 * Name: esp32c3_wdt_pre
 *
 * Description:
 *   Set a prescale value.
 *   The MWDT clock period is 12.5 ns * value (pre).
 *   NOTE: There's no prescaler register for RWDT and its source clock is
 *   clocked from the RTC slow clock.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *   pre           - Prescaler value to be configured.
 *
 ****************************************************************************/

static void esp32c3_wdt_pre(struct esp32c3_wdt_dev_s *dev,
                               uint16_t pre)
{
  uint32_t mask = (uint32_t)pre << TIMG_WDT_CLK_PRESCALE_S;

  DEBUGASSERT(dev);

  esp32c3_wdt_modifyreg32(dev, MWDT_CLK_PRESCALE_OFFSET,
                          TIMG_WDT_CLK_PRESCALE_M, mask);
}

/****************************************************************************
 * Name: esp32c3_wdt_settimeout
 *
 * Description:
 *   Set the WDT timeout.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *   value         - Timeout value in number of WDT cycles.
 *   stage         - Stage whose timeout value needs to be configured.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

static int32_t esp32c3_wdt_settimeout(struct esp32c3_wdt_dev_s *dev,
                                      uint32_t value,
                                      enum esp32c3_wdt_stage_e stage)
{
  int32_t ret = OK;
  DEBUGASSERT(dev);

  switch (stage)
  {
    case ESP32C3_WDT_STAGE0:
      {
        if (IS_RWDT(dev))
          {
            /* The timeout of only stage 0 happens at:
             * Thold0 = RTC_CNTL_WDT_STG0_HOLD << (EFUSE_WDT_DELAY_SEL + 1)
             */

            uint32_t delay;
            delay = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA1_REG,
                                  EFUSE_WDT_DELAY_SEL);
            value = value >> (delay + 1);
            esp32c3_wdt_putreg(dev, RWDT_STAGE0_TIMEOUT_OFFSET, value);
          }
        else
          {
            esp32c3_wdt_putreg(dev, MWDT_STAGE0_TIMEOUT_OFFSET, value);
          }
        break;
      }

    case ESP32C3_WDT_STAGE1:
      {
        if (IS_RWDT(dev))
          {
            esp32c3_wdt_putreg(dev, RWDT_STAGE1_TIMEOUT_OFFSET, value);
          }
        else
          {
            esp32c3_wdt_putreg(dev, MWDT_STAGE1_TIMEOUT_OFFSET, value);
          }
        break;
      }

    case ESP32C3_WDT_STAGE2:
      {
        if (IS_RWDT(dev))
          {
            esp32c3_wdt_putreg(dev, RWDT_STAGE2_TIMEOUT_OFFSET, value);
          }
        else
          {
            esp32c3_wdt_putreg(dev, MWDT_STAGE2_TIMEOUT_OFFSET, value);
          }
        break;
      }

    case ESP32C3_WDT_STAGE3:
      {
        if (IS_RWDT(dev))
          {
            esp32c3_wdt_putreg(dev, RWDT_STAGE3_TIMEOUT_OFFSET, value);
          }
        else
          {
            esp32c3_wdt_putreg(dev, MWDT_STAGE3_TIMEOUT_OFFSET, value);
          }
        break;
      }

    default:
      {
        wderr("ERROR: unsupported stage %d\n", stage);
        ret = -EINVAL;
      }
  }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_wdt_feed
 *
 * Description:
 *   Feed the watchdog.
 *   The watchdog timer returns to stage 0 and its counter restarts from 0.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void esp32c3_wdt_feed(struct esp32c3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  if (IS_RWDT(dev))
    {
      esp32c3_wdt_modifyreg32(dev, RWDT_FEED_OFFSET, 0, RTC_CNTL_WDT_FEED);
    }
  else
    {
      esp32c3_wdt_putreg(dev, MWDT_FEED_OFFSET, TIMG_WDT_FEED);
    }
}

/****************************************************************************
 * Name: esp32c3_wdt_rtc_clk
 *
 * Description:
 *   Calculate the necessary cycles of RTC SLOW_CLK to complete
 *   1 ms.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 * Returned Values:
 *   Return the number of cycles that completes 1 ms.
 *
 ****************************************************************************/

static uint16_t esp32c3_wdt_rtc_clk(struct esp32c3_wdt_dev_s *dev)
{
  enum esp32c3_rtc_slow_freq_e slow_clk_rtc;
  uint32_t period_13q19;
  float period;
  float cycles_ms;
  uint16_t cycles_ms_int;

  /* Calibration map: Maps each RTC SLOW_CLK source to the number
   * used to calibrate this source.
   */

  static const enum esp32c3_rtc_cal_sel_e cal_map[] =
  {
    RTC_CAL_RTC_MUX,
    RTC_CAL_32K_XTAL,
    RTC_CAL_8MD256
  };

  DEBUGASSERT(dev);

  /* Check which clock is sourcing the slow_clk_rtc */

  slow_clk_rtc = esp32c3_rtc_clk_slow_freq_get();

  /* Get the slow_clk_rtc period in us in Q13.19 fixed point format */

  period_13q19 = esp32c3_rtc_clk_cal(cal_map[slow_clk_rtc],
                                     SLOW_CLK_CAL_CYCLES);

  /* Assert no error happened during the calibration */

  DEBUGASSERT(period_13q19 != 0);

  /* Convert from Q13.19 format to float */

  period = Q_TO_FLOAT(period_13q19);

  wdinfo("PERIOD: %f  %" PRIu32"\n", period, period_13q19);

  /* Get the number of cycles necessary to count 1 ms */

  cycles_ms = 1000.0f / period;

  /* Get the integer number of cycles */

  cycles_ms_int = (uint16_t)cycles_ms;

  return cycles_ms_int;
}

/****************************************************************************
 * Name: esp32c3_wdt_setisr
 *
 * Description:
 *   Allocate a Level CPU Interrupt, connect the peripheral source to this
 *   Interrupt, register the callback and enable the interrupt.
 *   In case a NULL handler is provided, deallocate the interrupt and
 *   unregister the previously provided handler.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *   handler       - Callback to be invoked on watchdog timer interrupt.
 *   arg           - Argument to be passed to the handler callback.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

static int32_t esp32c3_wdt_setisr(struct esp32c3_wdt_dev_s *dev,
                                  xcpt_t handler,
                                  void *arg)
{
  struct esp32c3_wdt_priv_s *wdt = NULL;
  int32_t ret = OK;

  DEBUGASSERT(dev);

  wdt = (struct esp32c3_wdt_priv_s *)dev;

  /* Disable interrupt when callback is removed. */

  if (handler == NULL)
    {
#ifdef CONFIG_ESP32C3_RWDT
      if (wdt->irq == ESP32C3_IRQ_RTC_WDT)
        {
          esp32c3_rtcioirqdisable(wdt->irq);
          irq_detach(wdt->irq);
        }
      else
#endif
      if (wdt->cpuint != -ENOMEM)
        {
            {
              /* If a CPU Interrupt was previously allocated,
               * then deallocate it.
               */

              up_disable_irq(wdt->irq);
              irq_detach(wdt->irq);
              esp32c3_teardown_irq(wdt->periph, wdt->cpuint);
              wdt->cpuint = -ENOMEM;
            }
        }
    }

  /* Otherwise set callback and enable interrupt. */

  else
    {
#ifdef CONFIG_ESP32C3_RWDT
      if (wdt->irq == ESP32C3_IRQ_RTC_WDT)
        {
          ret = irq_attach(wdt->irq, handler, arg);

          if (ret != OK)
            {
              esp32c3_rtcioirqdisable(wdt->irq);
              tmrerr("ERROR: Failed to associate an IRQ Number");
            }

          esp32c3_rtcioirqenable(wdt->irq);
        }
      else
#endif
        {
          if (wdt->cpuint != -ENOMEM)
            {
              /* Disable the provided CPU interrupt to configure it. */

              up_disable_irq(wdt->irq);

              /* Free CPU interrupt that is attached to this peripheral
               * because we will get another from esp32c3_setup_irq()
               */

              esp32c3_teardown_irq(wdt->periph, wdt->cpuint);
            }

          wdt->cpuint = esp32c3_setup_irq(wdt->periph,
                                            ESP32C3_INT_PRIO_DEF,
                                            ESP32C3_INT_LEVEL);

          if (wdt->cpuint < 0)
            {
              return wdt->cpuint;
            }

          /* Attach and enable the IRQ. */

          ret = irq_attach(wdt->irq, handler, arg);
          if (ret != OK)
            {
              /* Failed to attach IRQ, so CPU interrupt must be freed. */

              esp32c3_teardown_irq(wdt->periph, wdt->cpuint);
              wdt->cpuint = -ENOMEM;
              return ret;
            }

          /* Enable the CPU interrupt that is linked to the WDT. */

          up_enable_irq(wdt->irq);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_wdt_enableint
 *
 * Description:
 *   Enable a Level Interrupt at timeout.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void esp32c3_wdt_enableint(struct esp32c3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  if (IS_RWDT(dev))
    {
      esp32c3_wdt_modifyreg32(dev, RWDT_INT_ENA_REG_OFFSET, 0,
                              RTC_CNTL_WDT_INT_ENA);
    }
  else
    {
      esp32c3_wdt_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET, 0,
                              TIMG_WDT_INT_ENA);
    }
}

/****************************************************************************
 * Name: esp32c3_wdt_disableint
 *
 * Description:
 *   Disable a Level Interrupt at timeout.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void esp32c3_wdt_disableint(struct esp32c3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  if (IS_RWDT(dev))
    {
      esp32c3_wdt_modifyreg32(dev, RWDT_INT_ENA_REG_OFFSET,
                              RTC_CNTL_WDT_INT_ENA, 0);
    }
  else
    {
      esp32c3_wdt_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET,
                              TIMG_WDT_INT_ENA, 0);
    }
}

/****************************************************************************
 * Name: esp32c3_wdt_ackint
 *
 *   Description:
 *   Acknowledge an interrupt.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void esp32c3_wdt_ackint(struct esp32c3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev);

  if (IS_RWDT(dev))
    {
      esp32c3_wdt_putreg(dev, RWDT_INT_CLR_REG_OFFSET, RTC_CNTL_WDT_INT_CLR);
    }
  else
    {
      esp32c3_wdt_putreg(dev, MWDT_INT_CLR_REG_OFFSET, TIMG_WDT_INT_CLR);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_wdt_init
 *
 * Description:
 *   Initialize WDT device.
 *
 * Parameters:
 *   wdt_id           - A Watchdog Timer instance to be initialized.
 *
 * Return Values:
 *   Pointer to the driver state structure.
 *
 ****************************************************************************/

struct esp32c3_wdt_dev_s *esp32c3_wdt_init(enum esp32c3_wdt_inst_e wdt_id)
{
  struct esp32c3_wdt_priv_s *wdt = NULL;

  /* Get WDT instance */

  switch (wdt_id)
    {
#ifdef CONFIG_ESP32C3_MWDT0
      case ESP32C3_WDT_MWDT0:
        {
          wdt = &g_esp32c3_mwdt0_priv;
          break;
        }

#endif

#ifdef CONFIG_ESP32C3_MWDT1
      case ESP32C3_WDT_MWDT1:
        {
          wdt = &g_esp32c3_mwdt1_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32C3_RWDT
      case ESP32C3_WDT_RWDT:
        {
          wdt = &g_esp32c3_rwdt_priv;

  /* If RTC was not initialized in a previous
   * stage by the PM or by clock_initialize()
   * Then, init the RTC clock configuration here.
   */

#if !defined(CONFIG_PM) && !defined(CONFIG_RTC)
  /* Initialize RTC controller parameters */

  esp32c3_rtc_init();
  esp32c3_rtc_clk_set();
#endif
          break;
        }

#endif

      default:
        {
          wderr("ERROR: unsupported WDT %d\n", wdt_id);
          return NULL;
        }
    }

  /* If some code is using it then sends an error message,
   * Otherwise, inform it has been used.
   */

  if (wdt->inuse == true)
    {
      wderr("ERROR: WDT %d is already in use\n", wdt_id);
      return NULL;
    }
  else
    {
      wdt->inuse = true;
    }

  return (struct esp32c3_wdt_dev_s *)wdt;
}

/****************************************************************************
 * Name: esp32c3_wdt_early_deinit
 *
 * Description:
 *   Disable the WDT(s) that was/were enabled by the bootloader.
 *
 ****************************************************************************/

void esp32c3_wdt_early_deinit(void)
{
  uint32_t regval;
  putreg32(RTC_CNTL_WDT_WKEY_VALUE, RTC_CNTL_WDTWPROTECT_REG);
  regval  = getreg32(RTC_CNTL_WDTCONFIG0_REG);
  regval &= ~RTC_CNTL_WDT_EN;
  putreg32(regval, RTC_CNTL_WDTCONFIG0_REG);
  putreg32(0, RTC_CNTL_WDTWPROTECT_REG);
}

/****************************************************************************
 * Name: esp32c3_wdt_deinit
 *
 * Description:
 *   Deinitialize a WDT device.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

void esp32c3_wdt_deinit(struct esp32c3_wdt_dev_s *dev)
{
  struct esp32c3_wdt_priv_s *wdt = NULL;

  DEBUGASSERT(dev);

  wdt = (struct esp32c3_wdt_priv_s *)dev;

  wdt->inuse = false;
}

/****************************************************************************
 * Name: esp32c3_wdt_is_running
 *
 * Description:
 *   Check whether the WDT is already started.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 * Returned Values:
 *   true if the Watchdog Timer is already started, false otherwise.
 *
 ****************************************************************************/

bool esp32c3_wdt_is_running(struct esp32c3_wdt_dev_s *dev)
{
  uint32_t status = 0;
  DEBUGASSERT(dev);

  if (IS_RWDT(dev))
    {
      status = esp32c3_wdt_getreg(dev, RWDT_CONFIG0_OFFSET);
      if (status & RTC_CNTL_WDT_EN)
        {
          return true;
        }
    }
  else
    {
      status = esp32c3_wdt_getreg(dev, MWDT_CONFIG0_OFFSET);
      if (status & TIMG_WDT_EN)
        {
          return true;
        }
    }

  return false;
}
