/****************************************************************************
 * arch/xtensa/src/esp32/esp32_gpio.c
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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "xtensa.h"

#include "hardware/esp32_iomux.h"
#include "hardware/esp32_gpio.h"

#include "esp32_irq.h"
#include "esp32_rtc_gpio.h"

#include "esp32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NGPIO_HPINS  (ESP32_NIRQ_GPIO - 32)
#define NGPIO_HMASK  ((UINT32_C(1) << NGPIO_HPINS) - 1)
#define _NA_         0xff

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
static int g_gpio_cpuint[CONFIG_SMP_NCPUS];
#endif

static const uint8_t g_pin2func[40] =
{
  0x44, 0x88, 0x40, 0x84, 0x48, 0x6c, 0x60, 0x64,  /* 0-7 */
  0x68, 0x54, 0x58, 0x5c, 0x34, 0x38, 0x30, 0x3c,  /* 8-15 */
  0x4c, 0x50, 0x70, 0x74, _NA_, 0x7c, 0x80, 0x8c,  /* 16-19, N/A, 21-23 */
  _NA_, 0x24, 0x28, 0x2c, _NA_, _NA_, _NA_, _NA_,  /* N/A, 25-27, N/A, N/A, N/A, N/A */
  0x1c, 0x20, 0x14, 0x18, 0x04, 0x08, 0x0c, 0x10   /* 32-39 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_is_valid_rtc_gpio
 *
 * Description:
 *   Determine if the specified GPIO is a valid RTC GPIO.
 *
 * Input Parameters:
 *   gpio_num - GPIO pin to be checked.
 *
 * Returned Value:
 *   True if valid. False otherwise.
 *
 ****************************************************************************/

static inline bool gpio_is_valid_rtc_gpio(uint32_t gpio_num)
{
  return (gpio_num < GPIO_PIN_COUNT && g_gpio_to_rtcio_map[gpio_num] >= 0);
}

/****************************************************************************
 * Name: rtc_gpio_is_pull_supported
 *
 * Description:
 *   Determine if the specified rtcio_num supports pull-up/pull-down.
 *
 * Input Parameters:
 *   rtcio_num - RTC GPIO to be checked.
 *
 * Returned Value:
 *   True if pull-up/pull-down supported. False otherwise.
 *
 ****************************************************************************/

static inline bool rtc_gpio_is_pull_supported(uint32_t rtcio_num)
{
  /* Pins 34 through 39 use RTC channels 0 to 5 and don't support PU/PD */

  return (rtcio_num > 5);
}

/****************************************************************************
 * Name: gpio_dispatch
 *
 * Description:
 *   Second level dispatch for GPIO interrupt handling.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
static void gpio_dispatch(int irq, uint32_t status, uint32_t *regs)
{
  uint32_t mask;
  int i;

  /* Check each bit in the status register */

  for (i = 0; i < 32 && status != 0; i++)
    {
      /* Check if there is an interrupt pending for this pin */

      mask = (UINT32_C(1) << i);
      if ((status & mask) != 0)
        {
          /* Yes... perform the second level dispatch */

          irq_dispatch(irq + i, regs);

          /* Clear the bit in the status so that we might execute this loop
           * sooner.
           */

          status &= ~mask;
        }
    }
}
#endif

/****************************************************************************
 * Name: gpio_interrupt
 *
 * Description:
 *   GPIO interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
static int gpio_interrupt(int irq, void *context, void *arg)
{
  uint32_t status;

  /* Read and clear the lower GPIO interrupt status */

  status = getreg32(GPIO_STATUS_REG);
  putreg32(status, GPIO_STATUS_W1TC_REG);

  /* Dispatch pending interrupts in the lower GPIO status register */

  gpio_dispatch(ESP32_FIRST_GPIOIRQ, status, (uint32_t *)context);

  /* Read and clear the upper GPIO interrupt status */

  status = getreg32(GPIO_STATUS1_REG) & NGPIO_HMASK;
  putreg32(status, GPIO_STATUS1_W1TC_REG);

  /* Dispatch pending interrupts in the lower GPIO status register */

  gpio_dispatch(ESP32_FIRST_GPIOIRQ + 32, status, (uint32_t *)context);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 * Input Parameters:
 *   pin  - GPIO pin to be configured
 *   attr - Attributes to be configured for the selected GPIO pin.
 *          The following attributes are accepted:
 *          - Direction (OUTPUT or INPUT)
 *          - Pull (PULLUP, PULLDOWN or OPENDRAIN)
 *          - Function (if not provided, assume function GPIO by default)
 *          - Drive strength (if not provided, assume DRIVE_2 by default)
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

int esp32_configgpio(int pin, gpio_pinattr_t attr)
{
  uintptr_t regaddr;
  uint32_t func;
  uint32_t cntrl;

  DEBUGASSERT(pin >= 0 && pin <= ESP32_NGPIOS);

  /* Handle input pins */

  func  = 0;
  cntrl = 0;

  if ((attr & INPUT) != 0)
    {
      if (pin < 32)
        {
          putreg32((UINT32_C(1) << pin), GPIO_ENABLE_W1TC_REG);
        }
      else
        {
          putreg32((UINT32_C(1) << (pin - 32)), GPIO_ENABLE1_W1TC_REG);
        }

      /* Input enable */

      func |= FUN_IE;

      /* Some pins only support Pull-Up and Pull-Down resistor on RTC GPIO */

      if (gpio_is_valid_rtc_gpio(pin))
        {
          uint32_t rtc_gpio_idx = g_gpio_to_rtcio_map[pin];
          uint32_t regval;
          uint32_t rtc_gpio_pin;
          bool en_pu = false;
          bool en_pd = false;

          if ((attr & PULLUP) != 0)
            {
              ASSERT(rtc_gpio_is_pull_supported(rtc_gpio_idx));
              en_pu = true;
            }
          else if ((attr & PULLDOWN) != 0)
            {
              ASSERT(rtc_gpio_is_pull_supported(rtc_gpio_idx));
              en_pd = true;
            }

          /* Get the pin register */

          rtc_gpio_pin = g_rtc_io_desc[rtc_gpio_idx].reg;

          /* Read the current value from RTC GPIO pin */

          regval = getreg32(rtc_gpio_pin);

          /* RTC_IO_X32P (GPIO32) uses different PU/PD bits */

          if (rtc_gpio_idx == RTCIO_GPIO32_CHANNEL)
            {
              /* First, disable PU/PD */

              regval &= ~SPECIAL_RTC_PU_BIT;
              regval &= ~SPECIAL_RTC_PD_BIT;

              /* Enable PU/PD, if needed */

              regval |= en_pu ? SPECIAL_RTC_PU_BIT : 0;
              regval |= en_pd ? SPECIAL_RTC_PD_BIT : 0;
            }
          else
            {
              /* First, disable PU/PD */

              regval &= ~DEFAULT_RTC_PU_BIT;
              regval &= ~DEFAULT_RTC_PD_BIT;

              /* Enable PU/PD, if needed */

              regval |= en_pu ? DEFAULT_RTC_PU_BIT : 0;
              regval |= en_pd ? DEFAULT_RTC_PD_BIT : 0;
            }

          putreg32(regval, rtc_gpio_pin);
        }
      else if ((attr & PULLUP) != 0)
        {
          func |= FUN_PU;
        }
      else if (attr & PULLDOWN)
        {
          func |= FUN_PD;
        }
    }

  /* Handle output pins */

  if ((attr & OUTPUT) != 0)
    {
      if (pin < 32)
        {
          putreg32((UINT32_C(1) << pin), GPIO_ENABLE_W1TS_REG);
        }
      else
        {
          putreg32((UINT32_C(1) << (pin - 32)), GPIO_ENABLE1_W1TS_REG);
        }
    }

  /* Configure the pad's function */

  if ((attr & FUNCTION_MASK) != 0)
    {
      uint32_t val = ((attr & FUNCTION_MASK) >> FUNCTION_SHIFT) - 1;
      func |= val << MCU_SEL_S;
    }
  else
    {
      /* Function not provided, assuming function GPIO by default */

      func |= (uint32_t)(PIN_FUNC_GPIO << MCU_SEL_S);
    }

  /* Configure the pad's drive strength */

  if ((attr & DRIVE_MASK) != 0)
    {
      uint32_t val = ((attr & DRIVE_MASK) >> DRIVE_SHIFT) - 1;
      func |= val << FUN_DRV_S;
    }
  else
    {
      /* Drive strength not provided, assuming strength 2 by default */

      func |= UINT32_C(2) << FUN_DRV_S;
    }

  if ((attr & OPEN_DRAIN) != 0)
    {
      cntrl |= (1 << GPIO_PIN_PAD_DRIVER_S);
    }

  regaddr = DR_REG_IO_MUX_BASE + g_pin2func[pin];
  putreg32(func, regaddr);

  regaddr = GPIO_REG(pin);
  putreg32(cntrl, regaddr);
  return OK;
}

/****************************************************************************
 * Name: esp32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void esp32_gpiowrite(int pin, bool value)
{
  DEBUGASSERT(pin >= 0 && pin <= ESP32_NGPIOS);

  if (value)
    {
      if (pin < 32)
        {
          putreg32((uint32_t)(UINT32_C(1) << pin), GPIO_OUT_W1TS_REG);
        }
      else
        {
          putreg32((uint32_t)(UINT32_C(1) << (pin - 32)),
                   GPIO_OUT1_W1TS_REG);
        }
    }
  else
    {
      if (pin < 32)
        {
          putreg32((uint32_t)(UINT32_C(1) << pin), GPIO_OUT_W1TC_REG);
        }
      else
        {
          putreg32((uint32_t)(UINT32_C(1) << (pin - 32)),
                   GPIO_OUT1_W1TC_REG);
        }
    }
}

/****************************************************************************
 * Name: esp32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool esp32_gpioread(int pin)
{
  uint32_t regval;

  DEBUGASSERT(pin >= 0 && pin <= ESP32_NGPIOS);

  if (pin < 32)
    {
      regval = getreg32(GPIO_IN_REG);
      return ((regval >> pin) & 1) != 0;
    }
  else
    {
      regval = getreg32(GPIO_IN1_REG);
      return ((regval >> (pin - 32)) & 1) != 0;
    }
}

/****************************************************************************
 * Name: esp32_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
void esp32_gpioirqinitialize(int cpu)
{
#ifdef CONFIG_SMP
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);
#else
  DEBUGASSERT(cpu == 0);
#endif

  /* Setup the GPIO interrupt. */

  g_gpio_cpuint[cpu] = esp32_setup_irq(cpu, ESP32_PERIPH_CPU_GPIO,
                                       1, ESP32_CPUINT_LEVEL);
  DEBUGASSERT(g_gpio_cpuint[cpu] >= 0);

  /* Attach and enable the interrupt handler */

  DEBUGVERIFY(irq_attach(esp32_irq_gpio(cpu), gpio_interrupt, NULL));
  up_enable_irq(esp32_irq_gpio(cpu));
}
#endif

/****************************************************************************
 * Name: esp32_gpioirqenable
 *
 * Description:
 *   Enable the COPY interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
void esp32_gpioirqenable(int irq, gpio_intrtype_t intrtype)
{
  uintptr_t regaddr;
  uint32_t regval;
  int pin;
  int cpu = up_cpu_index();

  DEBUGASSERT(irq >= ESP32_FIRST_GPIOIRQ && irq <= ESP32_LAST_GPIOIRQ);

  /* Convert the IRQ number to a pin number */

  pin = ESP32_IRQ2PIN(irq);

  /* Get the address of the GPIO PIN register for this pin */

  up_disable_irq(esp32_irq_gpio(cpu));

  regaddr = GPIO_REG(pin);
  regval  = getreg32(regaddr);
  regval &= ~(GPIO_PIN_INT_ENA_M | GPIO_PIN_INT_TYPE_M);

  /* Set the pin ENA field:
   *
   *   Bit 0: APP CPU interrupt enable
   *   Bit 1: APP CPU non-maskable interrupt enable
   *   Bit 3: PRO CPU interrupt enable
   *   Bit 4: PRO CPU non-maskable interrupt enable
   *   Bit 5: SDIO's extent interrupt enable.
   */

#ifdef CONFIG_SMP
  if (cpu != 0)
    {
      /* APP_CPU */

      regval |= ((1 << 0) << GPIO_PIN_INT_ENA_S);
    }
  else
#endif
    {
      /* PRO_CPU */

      regval |= ((1 << 2) << GPIO_PIN_INT_ENA_S);
    }

  regval |= (intrtype << GPIO_PIN_INT_TYPE_S);
  putreg32(regval, regaddr);

  up_enable_irq(esp32_irq_gpio(cpu));
}
#endif

/****************************************************************************
 * Name: esp32_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
void esp32_gpioirqdisable(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  int pin;
  int cpu = up_cpu_index();

  DEBUGASSERT(irq >= ESP32_FIRST_GPIOIRQ && irq <= ESP32_LAST_GPIOIRQ);

  /* Convert the IRQ number to a pin number */

  pin = ESP32_IRQ2PIN(irq);

  /* Get the address of the GPIO PIN register for this pin */

  up_disable_irq(esp32_irq_gpio(cpu));

  regaddr = GPIO_REG(pin);
  regval  = getreg32(regaddr);
  regval &= ~(GPIO_PIN_INT_ENA_M | GPIO_PIN_INT_TYPE_M);
  putreg32(regval, regaddr);

  up_enable_irq(esp32_irq_gpio(cpu));
}
#endif

/****************************************************************************
 * Name: esp32_gpio_matrix_in
 *
 * Description:
 *   Set gpio input to a signal
 *   NOTE: one gpio can input to several signals
 *   If gpio == 0x30, cancel input to the signal, input 0 to signal
 *   If gpio == 0x38, cancel input to the signal, input 1 to signal,
 *   for I2C pad
 *
 ****************************************************************************/

void esp32_gpio_matrix_in(uint32_t gpio, uint32_t signal_idx, bool inv)
{
  uint32_t regaddr = GPIO_FUNC0_IN_SEL_CFG_REG + (signal_idx * 4);
  uint32_t regval = (gpio << GPIO_FUNC0_IN_SEL_S);

  if (inv)
    {
      regval |= GPIO_FUNC0_IN_INV_SEL;
    }

  if (gpio != 0x34)
    {
      regval |= GPIO_SIG0_IN_SEL;
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: esp32_gpio_matrix_out
 *
 * Description:
 *   Set signal output to gpio
 *   NOTE: one signal can output to several gpios
 *   If signal_idx == 0x100, cancel output put to the gpio
 *
 ****************************************************************************/

void esp32_gpio_matrix_out(uint32_t gpio, uint32_t signal_idx, bool out_inv,
                           bool oen_inv)
{
  uint32_t regaddr = GPIO_FUNC0_OUT_SEL_CFG_REG + (gpio * 4);
  uint32_t regval = signal_idx << GPIO_FUNC0_OUT_SEL_S;

  if (gpio >= GPIO_PIN_COUNT)
    {
      return;
    }

  if (gpio < 32)
    {
      putreg32((UINT32_C(1) << gpio), GPIO_ENABLE_W1TS_REG);
    }
  else
    {
      putreg32((UINT32_C(1) << (gpio - 32)), GPIO_ENABLE1_W1TS_REG);
    }

  if (out_inv)
    {
      regval |= GPIO_FUNC0_OUT_INV_SEL;
    }

  if (oen_inv)
    {
      regval |= GPIO_FUNC0_OEN_INV_SEL;
    }

  putreg32(regval, regaddr);
}
