/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_wdt.c
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
#include "hardware/esp32s3_rtccntl.h"
#include "hardware/esp32s3_tim.h"
#include "hardware/esp32s3_efuse.h"

#include "esp32s3_irq.h"
#include "esp32s3_rtc_gpio.h"
#include "esp32s3_wdt.h"
#include <esp32s3_rtc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers for converting from Q13.19 fixed-point format to float */

#define N 19
#define Q_TO_FLOAT(x) ((float)x/(float)(1<<N))

/* Check whether the provided device is a RTC Watchdog Timer */

#define IS_RWDT(dev)    (((struct esp32s3_wdt_priv_s *)dev)->type == RTC)

/* Check whether the provided device is a Main Watchdog Timer */

#define IS_MWDT(dev)    (((struct esp32s3_wdt_priv_s *)dev)->type == TIMER)

/* Check whether the provided device is a XTAL32K Watchdog Timer */

#define IS_XTWDT(dev)    (((struct esp32s3_wdt_priv_s *)dev)->type == XTAL32K)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum wdt_peripheral_e
{
  RTC,
  TIMER,
  XTAL32K,
};

struct esp32s3_wdt_priv_s
{
  struct esp32s3_wdt_ops_s *ops;
  uint32_t                  base;    /* WDT register base address */
  uint8_t                   cpu;     /* CPU ID */
  uint8_t                   periph;  /* Peripheral ID */
  uint8_t                   irq;     /* Interrupt ID */
  int32_t                   cpuint;  /* CPU interrupt assigned to this WDT */
  bool                      inuse;   /* Flag indicating if this WDT is in use */
  enum wdt_peripheral_e     type;    /* Type of the WDT Peripheral */
};

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern void esp_rom_delay_us(uint32_t us);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* WDT registers access *****************************************************/

static void wdt_putreg(struct esp32s3_wdt_dev_s *dev, uint32_t offset,
                       uint32_t value);
static void wdt_modifyreg32(struct esp32s3_wdt_dev_s *dev, uint32_t offset,
                            uint32_t clearbits, uint32_t setbits);
static uint32_t wdt_getreg(struct esp32s3_wdt_dev_s *dev, uint32_t offset);

/* WDT operations ***********************************************************/

static void wdt_start(struct esp32s3_wdt_dev_s *dev);
static void wdt_stop(struct esp32s3_wdt_dev_s *dev);
static void wdt_enablewp(struct esp32s3_wdt_dev_s *dev);
static void wdt_disablewp(struct esp32s3_wdt_dev_s *dev);
static void wdt_pre(struct esp32s3_wdt_dev_s *dev,
                    uint16_t value);
static int32_t wdt_settimeout(struct esp32s3_wdt_dev_s *dev,
                              uint32_t value,
                              enum esp32s3_wdt_stage_e stage);
static void wdt_feed(struct esp32s3_wdt_dev_s *dev);
static int32_t wdt_config_stage(struct esp32s3_wdt_dev_s *dev,
                                enum esp32s3_wdt_stage_e stage,
                                enum esp32s3_wdt_stage_action_e cfg);
static int32_t wdt_setisr(struct esp32s3_wdt_dev_s *dev,
                          xcpt_t handler, void *arg);
static void wdt_enableint(struct esp32s3_wdt_dev_s *dev);
static void wdt_disableint(struct esp32s3_wdt_dev_s *dev);
static void wdt_ackint(struct esp32s3_wdt_dev_s *dev);
static uint16_t wdt_rtc_clk(struct esp32s3_wdt_dev_s *dev);
static void wdt_rstclk(struct esp32s3_wdt_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32-S3 WDT ops */

struct esp32s3_wdt_ops_s esp32s3_wdt_ops =
{
  .start         = wdt_start,
  .stop          = wdt_stop,
  .enablewp      = wdt_enablewp,
  .disablewp     = wdt_disablewp,
  .pre           = wdt_pre,
  .settimeout    = wdt_settimeout,
  .feed          = wdt_feed,
  .stg_conf      = wdt_config_stage,
  .rtc_clk       = wdt_rtc_clk,
  .setisr        = wdt_setisr,
  .enableint     = wdt_enableint,
  .disableint    = wdt_disableint,
  .ackint        = wdt_ackint,
  .rstclk        = wdt_rstclk,
};

#ifdef CONFIG_ESP32S3_MWDT0
struct esp32s3_wdt_priv_s g_esp32s3_mwdt0_priv =
{
  .ops    = &esp32s3_wdt_ops,
  .base   = TIMG_T0CONFIG_REG(0),
  .periph = ESP32S3_PERIPH_TG_WDT_LEVEL,
  .irq    = ESP32S3_IRQ_TG_WDT_LEVEL,
  .cpuint = -ENOMEM,
  .inuse  = false,
  .type   = TIMER,
};
#endif

#ifdef CONFIG_ESP32S3_MWDT1
struct esp32s3_wdt_priv_s g_esp32s3_mwdt1_priv =
{
  .ops    = &esp32s3_wdt_ops,
  .base   = TIMG_T0CONFIG_REG(1),
  .periph = ESP32S3_PERIPH_TG1_WDT_LEVEL,
  .irq    = ESP32S3_IRQ_TG1_WDT_LEVEL,
  .cpuint = -ENOMEM,
  .inuse  = false,
  .type   = TIMER,
};
#endif

#ifdef CONFIG_ESP32S3_RWDT
struct esp32s3_wdt_priv_s g_esp32s3_rwdt_priv =
{
  .ops    = &esp32s3_wdt_ops,
  .base   = RTC_CNTL_RTC_OPTIONS0_REG,
  .periph = ESP32S3_PERIPH_RTC_CORE,
  .irq    = ESP32S3_IRQ_RTC_WDT,
  .cpuint = -ENOMEM,
  .inuse  = false,
  .type   = RTC,
};
#endif

#ifdef CONFIG_ESP32S3_XTWDT
struct esp32s3_wdt_priv_s g_esp32s3_xtwdt_priv =
{
  .ops    = &esp32s3_wdt_ops,
  .base   = RTC_CNTL_RTC_OPTIONS0_REG,
  .periph = ESP32S3_PERIPH_RTC_CORE,
  .irq    = ESP32S3_IRQ_RTC_XTAL32K_DEAD,
  .cpuint = -ENOMEM,
  .inuse  = false,
  .type   = XTAL32K,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wdt_putreg
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

static void wdt_putreg(struct esp32s3_wdt_dev_s *dev, uint32_t offset,
                       uint32_t value)
{
  DEBUGASSERT(dev != NULL);

  putreg32(value, ((struct esp32s3_wdt_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: wdt_modifyreg32
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

static void wdt_modifyreg32(struct esp32s3_wdt_dev_s *dev, uint32_t offset,
                            uint32_t clearbits, uint32_t setbits)
{
  DEBUGASSERT(dev != NULL);

  modifyreg32(((struct esp32s3_wdt_priv_s *)dev)->base + offset,
                clearbits, setbits);
}

/****************************************************************************
 * Name: wdt_getreg
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

static uint32_t wdt_getreg(struct esp32s3_wdt_dev_s *dev, uint32_t offset)
{
  DEBUGASSERT(dev != NULL);

  return getreg32(((struct esp32s3_wdt_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: wdt_start
 *
 * Description:
 *   Release the counter.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void wdt_start(struct esp32s3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  if (IS_RWDT(dev))
    {
      wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, 0, RTC_CNTL_WDT_EN);
    }
  else if (IS_MWDT(dev))
    {
      wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, 0, TIMG_WDT_EN);
    }
  else
    {
      wdt_modifyreg32(dev, XTWDT_CONFIG0_OFFSET, 0, RTC_CNTL_XTAL32K_WDT_EN);
#ifdef CONFIG_ESP32S3_XTWDT_BACKUP_CLK_ENABLE
      wdt_modifyreg32(dev, XTWDT_CONFIG0_OFFSET,
                      0, RTC_CNTL_XTAL32K_AUTO_BACKUP);
#endif
    }
}

/****************************************************************************
 * Name: wdt_config_stage
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

static int32_t wdt_config_stage(struct esp32s3_wdt_dev_s *dev,
                                enum esp32s3_wdt_stage_e stage,
                                enum esp32s3_wdt_stage_action_e cfg)
{
  uint32_t mask;

  DEBUGASSERT(dev != NULL);

  switch (stage)
  {
    case ESP32S3_WDT_STAGE0:
      {
        if (IS_RWDT(dev))
          {
            mask = (uint32_t)cfg << RTC_CNTL_WDT_STG0_S;
            wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, RTC_CNTL_WDT_STG0_M,
                            mask);
          }
        else if (IS_MWDT(dev))
          {
            mask = (uint32_t)cfg << TIMG_WDT_STG0_S;
            wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_STG0_M, mask);
          }
        break;
      }

    case ESP32S3_WDT_STAGE1:
      {
        if (IS_RWDT(dev))
          {
            mask = (uint32_t)cfg << RTC_CNTL_WDT_STG1_S;
            wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, RTC_CNTL_WDT_STG1_M,
                            mask);
          }
        else if (IS_MWDT(dev))
          {
            mask = (uint32_t)cfg << TIMG_WDT_STG1_S;
            wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_STG1_M, mask);
          }
        break;
      }

    case ESP32S3_WDT_STAGE2:
      {
        if (IS_RWDT(dev))
          {
            mask = (uint32_t)cfg << RTC_CNTL_WDT_STG2_S;
            wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, RTC_CNTL_WDT_STG2_M,
                            mask);
          }
        else if (IS_MWDT(dev))
          {
            mask = (uint32_t)cfg << TIMG_WDT_STG2_S;
            wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_STG2_M, mask);
          }
        break;
      }

    case ESP32S3_WDT_STAGE3:
      {
        if (IS_RWDT(dev))
          {
            mask = (uint32_t)cfg << RTC_CNTL_WDT_STG3_S;
            wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, RTC_CNTL_WDT_STG3_M,
                            mask);
          }
        else if (IS_MWDT(dev))
          {
            mask = (uint32_t)cfg << TIMG_WDT_STG3_S;
            wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_STG3_M, mask);
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
 * Name: wdt_stop
 *
 * Description:
 *   Disable the watchdog.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void wdt_stop(struct esp32s3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  if (IS_RWDT(dev))
    {
      wdt_modifyreg32(dev, RWDT_CONFIG0_OFFSET, RTC_CNTL_WDT_EN, 0);
    }
  else if (IS_MWDT(dev))
    {
      wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_EN, 0);
    }
  else
    {
      wdt_modifyreg32(dev, XTWDT_CONFIG0_OFFSET, RTC_CNTL_XTAL32K_WDT_EN, 0);
    }
}

/****************************************************************************
 * Name: wdt_enablewp
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

static void wdt_enablewp(struct esp32s3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  if (IS_RWDT(dev))
    {
      wdt_putreg(dev, RWDT_WP_REG, 0);
    }
  else if (IS_MWDT(dev))
    {
      wdt_putreg(dev, MWDT_WP_REG, 0);
    }
}

/****************************************************************************
 * Name: wdt_disablewp
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

static void wdt_disablewp(struct esp32s3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  if (IS_RWDT(dev))
    {
      wdt_putreg(dev, RWDT_WP_REG, RTC_CNTL_WDT_WKEY_VALUE);
    }
  else if (IS_MWDT(dev))
    {
      wdt_putreg(dev, MWDT_WP_REG, TIMG_WDT_WKEY_VALUE);
    }
}

/****************************************************************************
 * Name: wdt_pre
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

static void wdt_pre(struct esp32s3_wdt_dev_s *dev, uint16_t pre)
{
  uint32_t mask = 0;
  DEBUGASSERT(dev != NULL);

  if (IS_MWDT(dev))
    {
      mask = (uint32_t)pre << TIMG_WDT_CLK_PRESCALE_S;
      wdt_modifyreg32(dev, MWDT_CLK_PRESCALE_OFFSET, TIMG_WDT_CLK_PRESCALE_M,
                      mask);
    }
#ifdef CONFIG_ESP32S3_XTWDT_BACKUP_CLK_ENABLE
  else if (IS_XTWDT(dev))
    {
      mask = (uint32_t)pre;
      wdt_modifyreg32(dev, XTWDT_CLK_PRESCALE_OFFSET,
                      RTC_CNTL_XTAL32K_CLK_FACTOR_M, mask);
    }
#endif
}

/****************************************************************************
 * Name: wdt_settimeout
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

static int32_t wdt_settimeout(struct esp32s3_wdt_dev_s *dev, uint32_t value,
                              enum esp32s3_wdt_stage_e stage)
{
  DEBUGASSERT(dev != NULL);

  if (IS_XTWDT(dev))
    {
      value = value << RTC_CNTL_XTAL32K_WDT_TIMEOUT_S;
      wdt_modifyreg32(dev, XTWDT_TIMEOUT_OFFSET,
                      RTC_CNTL_XTAL32K_WDT_TIMEOUT_M, value);
      return OK;
    }

  switch (stage)
  {
    case ESP32S3_WDT_STAGE0:
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
            wdt_putreg(dev, RWDT_STAGE0_TIMEOUT_OFFSET, value);
          }
        else
          {
            wdt_putreg(dev, MWDT_STAGE0_TIMEOUT_OFFSET, value);
          }
        break;
      }

    case ESP32S3_WDT_STAGE1:
      {
        if (IS_RWDT(dev))
          {
            wdt_putreg(dev, RWDT_STAGE1_TIMEOUT_OFFSET, value);
          }
        else
          {
            wdt_putreg(dev, MWDT_STAGE1_TIMEOUT_OFFSET, value);
          }
        break;
      }

    case ESP32S3_WDT_STAGE2:
      {
        if (IS_RWDT(dev))
          {
            wdt_putreg(dev, RWDT_STAGE2_TIMEOUT_OFFSET, value);
          }
        else
          {
            wdt_putreg(dev, MWDT_STAGE2_TIMEOUT_OFFSET, value);
          }
        break;
      }

    case ESP32S3_WDT_STAGE3:
      {
        if (IS_RWDT(dev))
          {
            wdt_putreg(dev, RWDT_STAGE3_TIMEOUT_OFFSET, value);
          }
        else
          {
            wdt_putreg(dev, MWDT_STAGE3_TIMEOUT_OFFSET, value);
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
 * Name: wdt_feed
 *
 * Description:
 *   Feed the watchdog.
 *   The watchdog timer returns to stage 0 and its counter restarts from 0.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void wdt_feed(struct esp32s3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  if (IS_RWDT(dev))
    {
      wdt_modifyreg32(dev, RWDT_FEED_OFFSET, 0, RTC_CNTL_RTC_WDT_FEED);
    }
  else if (IS_MWDT(dev))
    {
      wdt_putreg(dev, MWDT_FEED_OFFSET, TIMG_WDT_FEED);
    }
}

/****************************************************************************
 * Name: wdt_rtc_clk
 *
 * Description:
 *   Check the RTC clock source and return the necessary cycles to complete
 *   1 ms.
 *
 * Parameters:
 *   dev - Pointer to the driver state structure.
 *
 * Returned Values:
 *   Number of cycles to complete 1 ms.
 *
 ****************************************************************************/

static uint16_t wdt_rtc_clk(struct esp32s3_wdt_dev_s *dev)
{
  enum esp32s3_rtc_slow_freq_e slow_clk_rtc;
  uint32_t period_13q19;
  float period;
  float cycles_ms;
  uint16_t cycles_ms_int;

  /* Calibration map: Maps each RTC SLOW_CLK source to the number
   * used to calibrate this source.
   */

  static const enum esp32s3_rtc_cal_sel_e cal_map[] =
  {
    RTC_CAL_RTC_MUX,
    RTC_CAL_32K_XTAL,
    RTC_CAL_8MD256
  };

  DEBUGASSERT(dev);

  /* Check which clock is sourcing the slow_clk_rtc */

  slow_clk_rtc = esp32s3_rtc_get_slow_clk();

  /* Get the slow_clk_rtc period in us in Q13.19 fixed point format */

  period_13q19 = esp32s3_rtc_clk_cal(cal_map[slow_clk_rtc],
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
 * Name: wdt_setisr
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

static int32_t wdt_setisr(struct esp32s3_wdt_dev_s *dev, xcpt_t handler,
                          void *arg)
{
  struct esp32s3_wdt_priv_s *wdt = NULL;
  int32_t ret = OK;

  DEBUGASSERT(dev != NULL);

  wdt = (struct esp32s3_wdt_priv_s *)dev;

  /* Disable interrupt when callback is removed. */

  if (handler == NULL)
    {
      /* If a CPU Interrupt was previously allocated, then deallocate it */

      if (wdt->cpuint >= 0)
        {
#if defined(CONFIG_ESP32S3_RWDT) || defined(CONFIG_ESP32S3_XTWDT)
          if (wdt->irq == ESP32S3_IRQ_RTC_WDT ||
              wdt->irq == ESP32S3_IRQ_RTC_XTAL32K_DEAD)
            {
              esp32s3_rtcioirqdisable(wdt->irq);
              irq_detach(wdt->irq);
            }
          else
#endif
            {
              /* Disable CPU Interrupt, free a previously allocated
               * CPU Interrupt
               */

              up_disable_irq(wdt->irq);
              esp32s3_teardown_irq(wdt->cpu, wdt->periph, wdt->cpuint);
              irq_detach(wdt->irq);
              wdt->cpuint = -ENOMEM;
            }
        }

      goto errout;
    }

  /* Otherwise set callback and enable interrupt. */

  else
    {
      /* Set up to receive peripheral interrupts on the current CPU */

#if defined(CONFIG_ESP32S3_RWDT) || defined(CONFIG_ESP32S3_XTWDT)
      if (wdt->irq == ESP32S3_IRQ_RTC_WDT ||
          wdt->irq == ESP32S3_IRQ_RTC_XTAL32K_DEAD)
        {
          ret = irq_attach(wdt->irq, handler, arg);

          if (ret != OK)
            {
              esp32s3_rtcioirqdisable(wdt->irq);
              tmrerr("ERROR: Failed to associate an IRQ Number");
              goto errout;
            }

          esp32s3_rtcioirqenable(wdt->irq);
        }
      else
#endif
        {
          wdt->cpu = this_cpu();
          wdt->cpuint = esp32s3_setup_irq(wdt->cpu, wdt->periph,
                                          1, ESP32S3_CPUINT_LEVEL);
          if (wdt->cpuint < 0)
            {
              wderr("ERROR: No CPU Interrupt available");
              ret = wdt->cpuint;
              goto errout;
            }

          /* Associate an IRQ Number (from the WDT) to an ISR */

          ret = irq_attach(wdt->irq, handler, arg);
          if (ret != OK)
            {
              esp32s3_teardown_irq(wdt->cpu, wdt->periph, wdt->cpuint);
              wderr("ERROR: Failed to associate an IRQ Number");
              goto errout;
            }

          /* Enable the CPU Interrupt that is linked to the WDT */

          up_enable_irq(wdt->irq);
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: wdt_enableint
 *
 * Description:
 *   Enable a Level Interrupt at timeout.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void wdt_enableint(struct esp32s3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  if (IS_RWDT(dev))
    {
      wdt_modifyreg32(dev, RWDT_INT_ENA_REG_OFFSET, 0,
                      RTC_CNTL_RTC_WDT_INT_ENA);
    }
  else if (IS_MWDT(dev))
    {
      wdt_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET, 0, TIMG_WDT_INT_ENA);
    }
  else
    {
      wdt_modifyreg32(dev, XTWDT_INT_ENA_REG_OFFSET, 0,
                      RTC_CNTL_RTC_XTAL32K_DEAD_INT_ENA);
    }
}

/****************************************************************************
 * Name: wdt_disableint
 *
 * Description:
 *   Disable a Level Interrupt at timeout.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void wdt_disableint(struct esp32s3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  if (IS_RWDT(dev))
    {
      wdt_modifyreg32(dev, RWDT_INT_ENA_REG_OFFSET, RTC_CNTL_RTC_WDT_INT_ENA,
                      0);
    }
  else if (IS_MWDT(dev))
    {
      wdt_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET, TIMG_WDT_INT_ENA, 0);
    }
  else
    {
      wdt_modifyreg32(dev, XTWDT_INT_ENA_REG_OFFSET,
                      RTC_CNTL_RTC_XTAL32K_DEAD_INT_ENA, 0);
    }
}

/****************************************************************************
 * Name: wdt_ackint
 *
 *   Description:
 *   Acknowledge an interrupt.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void wdt_ackint(struct esp32s3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  if (IS_RWDT(dev))
    {
      wdt_putreg(dev, RWDT_INT_CLR_REG_OFFSET, RTC_CNTL_RTC_WDT_INT_CLR);
    }
  else if (IS_MWDT(dev))
    {
      wdt_putreg(dev, MWDT_INT_CLR_REG_OFFSET, TIMG_WDT_INT_CLR);
    }
  else
    {
      wdt_putreg(dev, MWDT_INT_CLR_REG_OFFSET,
                 RTC_CNTL_RTC_XTAL32K_DEAD_INT_CLR);
    }
}

/****************************************************************************
 * Name: wdt_rstclk
 *
 * Description:
 *   Restores the xtal32k clock.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

static void wdt_rstclk(struct esp32s3_wdt_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  struct esp32s3_wdt_priv_s *wdt = (struct esp32s3_wdt_priv_s *)dev;

  if (IS_XTWDT(dev))
    {
      wdt->ops->stop(dev);

      wdt_modifyreg32(dev, XTWDT_CONFIG0_OFFSET, RTC_CNTL_XPD_XTAL_32K, 0);
      wdt_modifyreg32(dev, XTWDT_CONFIG0_OFFSET, 0, RTC_CNTL_XPD_XTAL_32K);

      /* Needs some time after switching to 32khz XTAL
       * before turning on WDT again
       */

      esp_rom_delay_us(300);

      wdt->ops->start(dev);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_wdt_init
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

struct esp32s3_wdt_dev_s *esp32s3_wdt_init(enum esp32s3_wdt_inst_e wdt_id)
{
  struct esp32s3_wdt_priv_s *wdt = NULL;

  /* Get WDT instance */

  switch (wdt_id)
    {
#ifdef CONFIG_ESP32S3_MWDT0
      case ESP32S3_WDT_MWDT0:
        {
          wdt = &g_esp32s3_mwdt0_priv;
          break;
        }

#endif

#ifdef CONFIG_ESP32S3_MWDT1
      case ESP32S3_WDT_MWDT1:
        {
          wdt = &g_esp32s3_mwdt1_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32S3_RWDT
      case ESP32S3_WDT_RWDT:
        {
          wdt = &g_esp32s3_rwdt_priv;
          break;
        }

#endif

#ifdef CONFIG_ESP32S3_XTWDT
      case ESP32S3_WDT_XTWDT:
        {
          wdt = &g_esp32s3_xtwdt_priv;
          break;
        }

#endif

      default:
        {
          wderr("ERROR: unsupported WDT %d\n", wdt_id);
          goto errout;
        }
    }

  /* If some code is using it then sends an error message,
   * Otherwise, inform it has been used.
   */

  if (wdt->inuse == true)
    {
      wderr("ERROR: WDT %d is already in use\n", wdt_id);
      wdt = NULL;
    }
  else
    {
      wdt->inuse = true;
    }

errout:
  return (struct esp32s3_wdt_dev_s *)wdt;
}

/****************************************************************************
 * Name: esp32s3_wdt_early_deinit
 *
 * Description:
 *   Disable the WDT(s) that was/were enabled by the bootloader.
 *
 ****************************************************************************/

void esp32s3_wdt_early_deinit(void)
{
  uint32_t regval;
  putreg32(RTC_CNTL_WDT_WKEY_VALUE, RTC_CNTL_RTC_WDTWPROTECT_REG);
  regval  = getreg32(RTC_CNTL_RTC_WDTCONFIG0_REG);
  regval &= ~RTC_CNTL_WDT_EN;
  putreg32(regval, RTC_CNTL_RTC_WDTCONFIG0_REG);
  putreg32(0, RTC_CNTL_RTC_WDTWPROTECT_REG);
}

/****************************************************************************
 * Name: esp32s3_wdt_deinit
 *
 * Description:
 *   Deinitialize a WDT device.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *
 ****************************************************************************/

void esp32s3_wdt_deinit(struct esp32s3_wdt_dev_s *dev)
{
  struct esp32s3_wdt_priv_s *wdt = NULL;

  DEBUGASSERT(dev != NULL);

  wdt = (struct esp32s3_wdt_priv_s *)dev;
  wdt->inuse = false;
}

/****************************************************************************
 * Name: esp32s3_wdt_is_running
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

bool esp32s3_wdt_is_running(struct esp32s3_wdt_dev_s *dev)
{
  uint32_t status = 0;

  DEBUGASSERT(dev != NULL);

  if (IS_RWDT(dev))
    {
      status = wdt_getreg(dev, RWDT_CONFIG0_OFFSET);
      if ((status & RTC_CNTL_WDT_EN) == RTC_CNTL_WDT_EN)
        {
          return true;
        }
    }
  else if (IS_MWDT(dev))
    {
      status = wdt_getreg(dev, MWDT_CONFIG0_OFFSET);
      if ((status & TIMG_WDT_EN) == TIMG_WDT_EN)
        {
          return true;
        }
    }
  else
    {
      status = wdt_getreg(dev, XTWDT_CONFIG0_OFFSET);
      if ((status & RTC_CNTL_XTAL32K_WDT_EN) == RTC_CNTL_XTAL32K_WDT_EN)
        {
          return true;
        }
    }

  return false;
}
