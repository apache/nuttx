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
#include "esp32s3_wdt.h"

#ifdef CONFIG_ESP32S3_RWDT
#  error "RWDT not yet supported due to missing RTC driver!"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check whether the provided device is a RTC Watchdog Timer */

#define IS_RWDT(dev)    (((struct esp32s3_wdt_priv_s *)dev)->base == \
                         RTC_CNTL_RTC_OPTIONS0_REG)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_wdt_priv_s
{
  struct esp32s3_wdt_ops_s *ops;
  uint32_t                  base;    /* WDT register base address */
  uint8_t                   cpu;     /* CPU ID */
  uint8_t                   periph;  /* Peripheral ID */
  uint8_t                   irq;     /* Interrupt ID */
  int32_t                   cpuint;  /* CPU interrupt assigned to this WDT */
  bool                      inuse;   /* Flag indicating if this WDT is in use */
};

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32-S3 WDT ops */

struct esp32s3_wdt_ops_s esp32s3_mwdt_ops =
{
  .start         = wdt_start,
  .stop          = wdt_stop,
  .enablewp      = wdt_enablewp,
  .disablewp     = wdt_disablewp,
  .pre           = wdt_pre,
  .settimeout    = wdt_settimeout,
  .feed          = wdt_feed,
  .stg_conf      = wdt_config_stage,
  .rtc_clk       = NULL,
  .setisr        = wdt_setisr,
  .enableint     = wdt_enableint,
  .disableint    = wdt_disableint,
  .ackint        = wdt_ackint,
};

struct esp32s3_wdt_ops_s esp32s3_rwdt_ops =
{
  .start         = wdt_start,
  .stop          = wdt_stop,
  .enablewp      = wdt_enablewp,
  .disablewp     = wdt_disablewp,
  .pre           = NULL,
  .settimeout    = wdt_settimeout,
  .feed          = wdt_feed,
  .stg_conf      = wdt_config_stage,
  .rtc_clk       = NULL,
  .setisr        = wdt_setisr,
  .enableint     = wdt_enableint,
  .disableint    = wdt_disableint,
  .ackint        = wdt_ackint,
};

#ifdef CONFIG_ESP32S3_MWDT0
struct esp32s3_wdt_priv_s g_esp32s3_mwdt0_priv =
{
  .ops    = &esp32s3_mwdt_ops,
  .base   = TIMG_T0CONFIG_REG(0),
  .periph = ESP32S3_PERIPH_TG_WDT_LEVEL,
  .irq    = ESP32S3_IRQ_TG_WDT_LEVEL,
  .cpuint = -ENOMEM,
  .inuse  = false,
};
#endif

#ifdef CONFIG_ESP32S3_MWDT1
struct esp32s3_wdt_priv_s g_esp32s3_mwdt1_priv =
{
  .ops    = &esp32s3_mwdt_ops,
  .base   = TIMG_T0CONFIG_REG(1),
  .periph = ESP32S3_PERIPH_TG1_WDT_LEVEL,
  .irq    = ESP32S3_IRQ_TG1_WDT_LEVEL,
  .cpuint = -ENOMEM,
  .inuse  = false,
};
#endif

#ifdef CONFIG_ESP32S3_RWDT
struct esp32s3_wdt_priv_s g_esp32s3_rwdt_priv =
{
  .ops    = &esp32s3_rwdt_ops,
  .base   = RTC_CNTL_RTC_OPTIONS0_REG,
  .periph = ESP32S3_PERIPH_RTC_CORE,
  .irq    = ESP32S3_IRQ_RTC_CORE,
  .cpuint = -ENOMEM,
  .inuse  = false,
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
  else
    {
      wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, 0, TIMG_WDT_EN);
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
  int32_t ret = OK;
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
        else
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
        else
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
        else
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
        else
          {
            mask = (uint32_t)cfg << TIMG_WDT_STG3_S;
            wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_STG3_M, mask);
          }
        break;
      }

    default:
      {
        wderr("ERROR: unsupported stage %d\n", stage);
        ret = -EINVAL;
        goto errout;
      }
  }

  errout:
    return ret;
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
  else
    {
      wdt_modifyreg32(dev, MWDT_CONFIG0_OFFSET, TIMG_WDT_EN, 0);
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
  else
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
  else
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
  uint32_t mask = (uint32_t)pre << TIMG_WDT_CLK_PRESCALE_S;

  DEBUGASSERT(dev != NULL);

  wdt_modifyreg32(dev, MWDT_CLK_PRESCALE_OFFSET, TIMG_WDT_CLK_PRESCALE_M,
                  mask);
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
  int32_t ret = OK;

  DEBUGASSERT(dev != NULL);

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
        ret = -EINVAL;
        goto errout;
      }
  }

  errout:
    return ret;
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
  else
    {
      wdt_putreg(dev, MWDT_FEED_OFFSET, TIMG_WDT_FEED);
    }
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
          /* Disable CPU Interrupt, free a previously allocated
           * CPU Interrupt
           */

          up_disable_irq(wdt->irq);
          esp32s3_teardown_irq(wdt->cpu, wdt->periph, wdt->cpuint);
          irq_detach(wdt->irq);
        }

      ret = OK;
      goto errout;
    }

  /* Otherwise set callback and enable interrupt. */

  else
    {
      /* Set up to receive peripheral interrupts on the current CPU */

      wdt->cpu = up_cpu_index();
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
  else
    {
      wdt_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET, 0, TIMG_WDT_INT_ENA);
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
  else
    {
      wdt_modifyreg32(dev, MWDT_INT_ENA_REG_OFFSET, TIMG_WDT_INT_ENA, 0);
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
  else
    {
      wdt_putreg(dev, MWDT_INT_CLR_REG_OFFSET, TIMG_WDT_INT_CLR);
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
  else
    {
      status = wdt_getreg(dev, MWDT_CONFIG0_OFFSET);
      if ((status & TIMG_WDT_EN) == TIMG_WDT_EN)
        {
          return true;
        }
    }

  return false;
}
