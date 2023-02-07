/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_rtc_gpio.c
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

#include <assert.h>
#include <debug.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "xtensa.h"
#include "esp32s3_irq.h"
#include "esp32s3_rtc_gpio.h"
#include "hardware/esp32s3_pinmap.h"
#include "hardware/esp32s3_rtc_io.h"
#include "hardware/esp32s3_sens.h"
#include "hardware/esp32s3_usb_serial_jtag.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define setbits(a, bs)   modifyreg32(a, 0, bs)
#define resetbits(a, bs) modifyreg32(a, bs, 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum rtcio_lh_out_mode_e
{
  RTCIO_OUTPUT_NORMAL = 0,    /* RTCIO output mode is normal. */
  RTCIO_OUTPUT_OD = 0x1,      /* RTCIO output mode is open-drain. */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_RTCIO_IRQ
static int g_rtcio_cpuint;
static uint32_t last_status;
#endif

static const uint32_t rtc_gpio_to_addr[] =
{
  RTCIO_RTC_GPIO_PIN0_REG,
  RTCIO_RTC_GPIO_PIN1_REG,
  RTCIO_RTC_GPIO_PIN2_REG,
  RTCIO_RTC_GPIO_PIN3_REG,
  RTCIO_RTC_GPIO_PIN4_REG,
  RTCIO_RTC_GPIO_PIN5_REG,
  RTCIO_RTC_GPIO_PIN6_REG,
  RTCIO_RTC_GPIO_PIN7_REG,
  RTCIO_RTC_GPIO_PIN8_REG,
  RTCIO_RTC_GPIO_PIN9_REG,
  RTCIO_RTC_GPIO_PIN10_REG,
  RTCIO_RTC_GPIO_PIN11_REG,
  RTCIO_RTC_GPIO_PIN12_REG,
  RTCIO_RTC_GPIO_PIN13_REG,
  RTCIO_RTC_GPIO_PIN14_REG,
  RTCIO_RTC_GPIO_PIN15_REG,
  RTCIO_RTC_GPIO_PIN16_REG,
  RTCIO_RTC_GPIO_PIN17_REG,
  RTCIO_RTC_GPIO_PIN18_REG,
  RTCIO_RTC_GPIO_PIN19_REG,
  RTCIO_RTC_GPIO_PIN20_REG,
  RTCIO_RTC_GPIO_PIN21_REG
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: is_valid_rtc_gpio
 *
 * Description:
 *   Determine if the specified rtcio_num is a valid RTC GPIO.
 *
 * Input Parameters:
 *   rtcio_num - RTC GPIO to be checked.
 *
 * Returned Value:
 *   True if valid. False otherwise.
 *
 ****************************************************************************/

static inline bool is_valid_rtc_gpio(uint32_t rtcio_num)
{
  return (rtcio_num < RTC_GPIO_NUMBER);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtcio_dispatch
 *
 * Description:
 *   Second level dispatch for the RTC interrupt.
 *
 * Input Parameters:
 *   irq - The IRQ number;
 *   reg_status - Pointer to a copy of the interrupt status register.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_RTCIO_IRQ
static void rtcio_dispatch(int irq, uint32_t *reg_status)
{
  uint32_t status = *reg_status;
  uint32_t mask;
  int i;

  /* Check each bit in the status register */

  for (i = 0; i < ESP32S3_NIRQ_RTCIO_PERIPH && status != 0; i++)
    {
      /* Check if there is an interrupt pending for this type */

      mask = (UINT32_C(1) << i);
      if ((status & mask) != 0)
        {
          /* Yes... perform the second level dispatch. The IRQ context will
           * contain the contents of the status register.
           */

          irq_dispatch(irq + i, (void *)reg_status);

          /* Clear the bit in the status so that we might execute this loop
           * sooner.
           */

          status &= ~mask;
        }
    }
}
#endif

/****************************************************************************
 * Name: rtcio_interrupt
 *
 * Description:
 *   RTC interrupt handler.
 *
 * Input Parameters:
 *   irq - The IRQ number;
 *   context - The interruption context;
 *   args - The arguments passed to the handler.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_RTCIO_IRQ
static int rtcio_interrupt(int irq, void *context, void *arg)
{
  /* Read and clear the lower RTC interrupt status */

  last_status = getreg32(RTC_CNTL_INT_ST_RTC_REG);
  putreg32(last_status, RTC_CNTL_INT_CLR_RTC_REG);

  /* Dispatch pending interrupts in the RTC status register */

  rtcio_dispatch(ESP32S3_FIRST_RTCIOIRQ_PERIPH, &last_status);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp32s3_configrtcio
 *
 * Description:
 *   Configure a RTCIO rtcio_num based on encoded rtcio_num attributes.
 *
 * Input Parameters:
 *   rtcio_num     - RTCIO rtcio_num to be configured.
 *   attr          - Attributes to be configured for the selected rtcio_num.
 *                   The following attributes are accepted:
 *                   - Direction (OUTPUT or INPUT)
 *                   - Pull (PULLUP, PULLDOWN or OPENDRAIN)
 *                   - Function (if not provided, assume function RTCIO by
 *                     default)
 *                   - Drive strength (if not provided, assume DRIVE_2 by
 *                     default)
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

int esp32s3_configrtcio(int rtcio_num, rtcio_pinattr_t attr)
{
  ASSERT(is_valid_rtc_gpio(rtcio_num));

  rtc_io_desc_t rtc_reg_desc = g_rtc_io_desc[rtcio_num];

  /* Configure the pad's function */

  if ((attr & RTC_FUNCTION_MASK) == RTC_FUNCTION_DIGITAL)
    {
      /* USB Serial JTAG pad re-enable won't be done here (it requires both
       * DM and DP pins not in rtc function). Instead,
       * USB_SERIAL_JTAG_USB_PAD_ENABLE needs to be guaranteed to be set in
       * the USB Serial JTAG driver.
       */

      resetbits(rtc_reg_desc.reg, rtc_reg_desc.mux);
      REG_SET_FIELD(SENS_SAR_PERI_CLK_GATE_CONF_REG,
                    SENS_IOMUX_CLK_EN,
                    false);
    }
  else
    {
      /* Disable USB Serial JTAG if pin 19 or pin 20 selects RTC */

      if (rtcio_num == JTAG_IOMUX_USB_DM || rtcio_num == JTAG_IOMUX_USB_DP)
        {
          REG_SET_FIELD(USB_SERIAL_JTAG_CONF0_REG,
                        USB_SERIAL_JTAG_USB_PAD_ENABLE,
                        false);
        }

      REG_SET_FIELD(SENS_SAR_PERI_CLK_GATE_CONF_REG,
                    SENS_IOMUX_CLK_EN,
                    true);

      setbits(rtc_reg_desc.reg, rtc_reg_desc.mux);
      modifyreg32(rtc_reg_desc.reg,
        ((RTCIO_TOUCH_PAD1_FUN_SEL_V) << (rtc_reg_desc.func)),
        (((RTCIO_PIN_FUNC) & RTCIO_TOUCH_PAD1_FUN_SEL_V) <<
        (rtc_reg_desc.func)));
    }

  if ((attr & RTC_INPUT) != 0)
    {
      /* Input enable */

      setbits(rtc_reg_desc.reg, rtc_reg_desc.ie);

      if ((attr & RTC_PULLUP) != 0)
        {
          if (rtc_reg_desc.pullup)
            {
              setbits(rtc_reg_desc.reg, rtc_reg_desc.pullup);
            }

          if (rtc_reg_desc.pulldown)
            {
              resetbits(rtc_reg_desc.reg, rtc_reg_desc.pulldown);
            }
        }
      else if ((attr & RTC_PULLDOWN) != 0)
        {
          /* The pull-up value of the USB pins are controlled by the
           * pins’ pull-up value together with USB pull-up value. USB DP
           * pin is default to PU enabled. Note that from ESP32-S2 ECO1,
           * USB_EXCHG_PINS feature has been supported. If this efuse is
           * burnt, the gpio pin which should be checked is
           * JTAG_IOMUX_USB_DM instead.
           */

          if (rtcio_num == JTAG_IOMUX_USB_DP)
            {
              REG_SET_FIELD(USB_SERIAL_JTAG_CONF0_REG,
                            USB_SERIAL_JTAG_PAD_PULL_OVERRIDE,
                            true);

              resetbits(USB_SERIAL_JTAG_CONF0_REG,
                        USB_SERIAL_JTAG_DP_PULLUP_M);
            }

          if (rtc_reg_desc.pullup)
            {
              resetbits(rtc_reg_desc.reg, rtc_reg_desc.pullup);
            }

          if (rtc_reg_desc.pulldown)
            {
              setbits(rtc_reg_desc.reg, rtc_reg_desc.pulldown);
            }
        }
    }
  else
    {
      if (rtcio_num == JTAG_IOMUX_USB_DP)
        {
          REG_SET_FIELD(USB_SERIAL_JTAG_CONF0_REG,
                        USB_SERIAL_JTAG_PAD_PULL_OVERRIDE,
                        true);

          resetbits(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_DP_PULLUP_M);
        }

      if (rtc_reg_desc.pullup)
        {
          resetbits(rtc_reg_desc.reg, rtc_reg_desc.pullup);
        }

      if (rtc_reg_desc.pulldown)
        {
          resetbits(rtc_reg_desc.reg, rtc_reg_desc.pulldown);
        }

      resetbits(rtc_reg_desc.reg, rtc_reg_desc.ie);
    }

  /* Handle output pins */

  if ((attr & RTC_OUTPUT) != 0)
    {
      REG_SET_FIELD(RTCIO_RTC_GPIO_ENABLE_W1TS_REG,
                    RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TS,
                    UINT32_C(1) << rtcio_num);
    }
  else
    {
      REG_SET_FIELD(RTCIO_RTC_GPIO_ENABLE_W1TC_REG,
                    RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TC,
                    UINT32_C(1) << rtcio_num);
    }

  /* Configure the pad's drive strength */

  if ((attr & RTC_DRIVE_MASK) != 0)
    {
      uint32_t val = ((attr & RTC_DRIVE_MASK) >> RTC_DRIVE_SHIFT) - 1;
      modifyreg32(rtc_reg_desc.reg,
        ((rtc_reg_desc.drv_v) << (rtc_reg_desc.drv_s)),
        (((val) & rtc_reg_desc.drv_v) << (rtc_reg_desc.drv_s)));
    }
  else
    {
      /* Drive strength not provided, assuming strength 2 by default */

      modifyreg32(rtc_reg_desc.reg,
        ((rtc_reg_desc.drv_v) << (rtc_reg_desc.drv_s)),
        (((2) & rtc_reg_desc.drv_v) << (rtc_reg_desc.drv_s)));
    }

  if ((attr & RTC_OPEN_DRAIN) != 0)
    {
      REG_SET_FIELD(rtc_gpio_to_addr[rtcio_num],
                    RTCIO_GPIO_PIN1_PAD_DRIVER,
                    RTCIO_OUTPUT_OD);
    }
  else
    {
      REG_SET_FIELD(rtc_gpio_to_addr[rtcio_num],
                    RTCIO_GPIO_PIN1_PAD_DRIVER,
                    RTCIO_OUTPUT_NORMAL);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_rtcioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   RTC IRQs.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_RTCIO_IRQ
void esp32s3_rtcioirqinitialize(void)
{
  /* Setup the RTCIO interrupt. */

  int cpu = up_cpu_index();
  g_rtcio_cpuint = esp32s3_setup_irq(cpu, ESP32S3_PERIPH_RTC_CORE,
                                     1, ESP32S3_CPUINT_LEVEL);
  DEBUGASSERT(g_rtcio_cpuint >= 0);

  /* Attach and enable the interrupt handler */

  DEBUGVERIFY(irq_attach(ESP32S3_IRQ_RTC_CORE, rtcio_interrupt, NULL));
  up_enable_irq(ESP32S3_IRQ_RTC_CORE);
}
#endif

/****************************************************************************
 * Name: esp32s3_rtcioirqenable
 *
 * Description:
 *   Enable the interrupt for the specified RTC peripheral IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_RTCIO_IRQ
void esp32s3_rtcioirqenable(int irq)
{
  uintptr_t regaddr = RTC_CNTL_INT_ENA_RTC_REG;
  uint32_t regval;
  int bit;

  DEBUGASSERT(irq >= ESP32S3_FIRST_RTCIOIRQ_PERIPH &&
              irq <= ESP32S3_LAST_RTCIOIRQ_PERIPH);

  /* Convert the IRQ number to the corresponding bit */

  bit = irq - ESP32S3_FIRST_RTCIOIRQ_PERIPH;

  /* Get the address of the GPIO PIN register for this pin */

  up_disable_irq(ESP32S3_IRQ_RTC_CORE);

  regval = getreg32(regaddr) | (UINT32_C(1) << bit);
  putreg32(regval, regaddr);

  up_enable_irq(ESP32S3_IRQ_RTC_CORE);
}
#endif

/****************************************************************************
 * Name: esp32s3_rtcioirqdisable
 *
 * Description:
 *   Disable the interrupt for the specified RTC peripheral IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_RTCIO_IRQ
void esp32s3_rtcioirqdisable(int irq)
{
  uintptr_t regaddr = RTC_CNTL_INT_ENA_RTC_REG;
  uint32_t regval;
  int bit;

  DEBUGASSERT(irq >= ESP32S3_FIRST_RTCIOIRQ_PERIPH &&
              irq <= ESP32S3_LAST_RTCIOIRQ_PERIPH);

  /* Convert the IRQ number to the corresponding bit */

  bit = irq - ESP32S3_FIRST_RTCIOIRQ_PERIPH;

  /* Disable IRQ */

  up_disable_irq(ESP32S3_IRQ_RTC_CORE);

  regval = getreg32(regaddr) & (~(UINT32_C(1) << bit));
  putreg32(regval, regaddr);

  up_enable_irq(ESP32S3_IRQ_RTC_CORE);
}
#endif
