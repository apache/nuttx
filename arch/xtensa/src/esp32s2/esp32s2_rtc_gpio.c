/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_rtc_gpio.c
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
#include "espressif/esp_irq.h"
#include "esp32s2_rtc_gpio.h"
#include "hardware/esp32s2_rtc_io.h"
#include "hardware/esp32s2_sens.h"
#include "driver/rtc_io.h"

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

/* Structure to store RTC GPIO interrupt handlers */

struct rtcio_handler_s
{
  xcpt_t        handler;      /* User interrupt handler */
  void          *arg;         /* Argument for handler */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_RTCIO_IRQ
static int g_rtcio_cpuint;
static uint32_t last_status;
static struct rtcio_handler_s g_rtcio_handlers[ESP32S2_NIRQ_RTCIO_PERIPH];
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
 * Name: rtcio_dispatch
 *
 * Description:
 *   Second level dispatch for the RTC interrupt.
 *
 * Input Parameters:
 *   irq - The IRQ number;
 *   context - The interrupt context;
 *   reg_status - Pointer to a copy of the interrupt status register.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_RTCIO_IRQ
static void rtcio_dispatch(int irq, void *context, uint32_t *reg_status)
{
  uint32_t status = *reg_status;
  uint32_t mask;
  int i;

  /* Check each bit in the status register */

  for (i = 0; i < ESP32S2_NIRQ_RTCIO_PERIPH && status != 0; i++)
    {
      /* Check if there is an interrupt pending for this type */

      mask = (UINT32_C(1) << i);
      if ((status & mask) != 0)
        {
          /* Call the registered handler if one exists */

          if (g_rtcio_handlers[i].handler != NULL)
            {
              g_rtcio_handlers[i].handler(irq,
                                          (void *)reg_status,
                                          g_rtcio_handlers[i].arg);
            }

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
 *   context - The interrupt context;
 *   args - The arguments passed to the handler.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_RTCIO_IRQ
static int rtcio_interrupt(int irq, void *context, void *arg)
{
  /* Read and clear the lower RTC interrupt status */

  last_status = getreg32(RTC_CNTL_INT_ST_REG);
  putreg32(last_status, RTC_CNTL_INT_CLR_REG);

  /* Dispatch pending interrupts in the RTC status register */

  rtcio_dispatch(irq, context, &last_status);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_configrtcio
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

int esp32s2_configrtcio(int rtcio_num, rtcio_pinattr_t attr)
{
  ASSERT(is_valid_rtc_gpio(rtcio_num));

  rtc_io_desc_t rtc_reg_desc = g_rtc_io_desc[rtcio_num];

  /* Configure the pad's function */

  if ((attr & RTC_FUNCTION_MASK) == RTC_FUNCTION_DIGITAL)
    {
      resetbits(rtc_reg_desc.reg, rtc_reg_desc.mux);
      REG_SET_FIELD(SENS_SAR_IO_MUX_CONF_REG,
                    SENS_IOMUX_CLK_GATE_EN,
                    false);
    }
  else
    {
      REG_SET_FIELD(SENS_SAR_IO_MUX_CONF_REG,
                    SENS_IOMUX_CLK_GATE_EN,
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
 * Name: esp32s2_rtcioread
 *
 * Description:
 *   Read one or zero from the selected RTC GPIO pin
 *
 * Input Parameters:
 *   rtcio_num     - RTCIO rtcio_num to be read.
 *
 * Returned Value:
 *   The boolean representation of the input value (true/false).
 *
 ****************************************************************************/

int esp32s2_rtcioread(int rtcio_num)
{
  return rtc_gpio_get_level(rtcio_num);
}

/****************************************************************************
 * Name: esp32s2_rtciowrite
 *
 * Description:
 *   Write one or zero to the selected RTC GPIO pin
 *
 * Input Parameters:
 *   rtcio_num - GPIO pin to be modified.
 *   value     - The value to be written (0 or 1).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s2_rtciowrite(int rtcio_num, bool value)
{
  rtc_gpio_set_level(rtcio_num, value);
}

/****************************************************************************
 * Name: esp_rtcioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   RTC interrupts.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_RTCIO_IRQ
void esp_rtcioirqinitialize(void)
{
  int i;

  /* Initialize handler array */

  for (i = 0; i < ESP32S2_NIRQ_RTCIO_PERIPH; i++)
    {
      g_rtcio_handlers[i].handler = NULL;
      g_rtcio_handlers[i].arg = NULL;
    }

  /* Setup the RTCIO interrupt with handler. */

  g_rtcio_cpuint = esp_setup_irq(ESP32S2_PERIPH_RTC_CORE,
                                 1, ESP_IRQ_TRIGGER_LEVEL,
                                 rtcio_interrupt, NULL);
  DEBUGASSERT(g_rtcio_cpuint >= 0);

  /* Enable the interrupt */

  up_enable_irq(ESP32S2_IRQ_RTC_CORE);
}
#endif

/****************************************************************************
 * Name: esp32s2_rtcioirqenable
 *
 * Description:
 *   Enable the interrupt for a specified RTC IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_RTCIO_IRQ
void esp32s2_rtcioirqenable(int irq)
{
  uintptr_t regaddr = RTC_CNTL_INT_ENA_REG;
  uint32_t regval;
  int bit;

  DEBUGASSERT(irq >= ESP32S2_FIRST_RTCIOIRQ_PERIPH &&
              irq <= ESP32S2_LAST_RTCIOIRQ_PERIPH);

  /* Convert the IRQ number to the corresponding bit */

  bit = irq - ESP32S2_FIRST_RTCIOIRQ_PERIPH;

  /* Get the address of the GPIO PIN register for this pin */

  up_disable_irq(ESP32S2_IRQ_RTC_CORE);

  regval = getreg32(regaddr) | (UINT32_C(1) << bit);
  putreg32(regval, regaddr);

  up_enable_irq(ESP32S2_IRQ_RTC_CORE);
}
#endif

/****************************************************************************
 * Name: esp32s2_rtcioirqdisable
 *
 * Description:
 *   Disable the interrupt for a specified RTC IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_RTCIO_IRQ
void esp32s2_rtcioirqdisable(int irq)
{
  uintptr_t regaddr = RTC_CNTL_INT_ENA_REG;
  uint32_t regval;
  int bit;

  DEBUGASSERT(irq >= ESP32S2_FIRST_RTCIOIRQ_PERIPH &&
              irq <= ESP32S2_LAST_RTCIOIRQ_PERIPH);

  /* Convert the IRQ number to the corresponding bit */

  bit = irq - ESP32S2_FIRST_RTCIOIRQ_PERIPH;

  /* Disable IRQ */

  up_disable_irq(ESP32S2_IRQ_RTC_CORE);

  regval = getreg32(regaddr) & (~(UINT32_C(1) << bit));
  putreg32(regval, regaddr);

  up_enable_irq(ESP32S2_IRQ_RTC_CORE);
}
#endif

/****************************************************************************
 * Name: esp32s2_rtcioirqattach
 *
 * Description:
 *   Attach an interrupt handler to a specified RTC IRQ
 *
 * Input Parameters:
 *   irq     - RTC IRQ number to attach the handler to
 *   handler - Interrupt handler function
 *   arg     - Argument to pass to the handler
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_RTCIO_IRQ
int esp32s2_rtcioirqattach(int irq, xcpt_t handler, void *arg)
{
  int bit;

  DEBUGASSERT(irq >= ESP32S2_FIRST_RTCIOIRQ_PERIPH &&
              irq <= ESP32S2_LAST_RTCIOIRQ_PERIPH);

  /* Convert the IRQ number to the corresponding bit */

  bit = irq - ESP32S2_FIRST_RTCIOIRQ_PERIPH;

  DEBUGASSERT(bit >= 0 && bit < ESP32S2_NIRQ_RTCIO_PERIPH);

  /* Store the handler and argument */

  g_rtcio_handlers[bit].handler = handler;
  g_rtcio_handlers[bit].arg = arg;

  return OK;
}
#endif

/****************************************************************************
 * Name: esp32s2_rtcioirqdetach
 *
 * Description:
 *   Detach an interrupt handler from a specified RTC IRQ
 *
 * Input Parameters:
 *   irq - RTC IRQ number to detach the handler from
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_RTCIO_IRQ
int esp32s2_rtcioirqdetach(int irq)
{
  int bit;

  DEBUGASSERT(irq >= ESP32S2_FIRST_RTCIOIRQ_PERIPH &&
              irq <= ESP32S2_LAST_RTCIOIRQ_PERIPH);

  /* Convert the IRQ number to the corresponding bit */

  bit = irq - ESP32S2_FIRST_RTCIOIRQ_PERIPH;

  DEBUGASSERT(bit >= 0 && bit < ESP32S2_NIRQ_RTCIO_PERIPH);

  /* Clear the handler and argument */

  g_rtcio_handlers[bit].handler = NULL;
  g_rtcio_handlers[bit].arg = NULL;

  return OK;
}
#endif
