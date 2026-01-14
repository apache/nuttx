/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_lowputc.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "chip.h"
#include "riscv_internal.h"

#include "esp_config.h"
#include "esp_gpio.h"
#include "esp_irq.h"
#include "esp_lowputc.h"
#include "esp_usbserial.h"
#include "esp_private/critical_section.h"
#include "esp_private/uart_share_hw_ctrl.h"

#include "hal/uart_hal.h"
#include "soc/uart_periph.h"
#include "periph_ctrl.h"
#include "soc/gpio_sig_map.h"
#ifdef CONFIG_ESPRESSIF_LP_UART
#  include "lp_core_uart.h"
#  include "soc/uart_pins.h"
#  include "hal/rtc_io_hal.h"
#  include "soc/uart_periph.h"
#  include "driver/rtc_io.h"
#  include "io_mux.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_LP_UART
#  define ESP_LP_UART0_ID     LP_UART_NUM_0
#  define RTCIO_RCC_ATOMIC()  PERIPH_RCC_ATOMIC()
#else
#  define ESP_LP_UART0_ID     UART_NUM_MAX
#endif /* CONFIG_ESPRESSIF_LP_UART */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_CONTEXT_INIT_DEF(uart_num) \
{ \
  .port_id = uart_num, \
  .hal.dev = UART_LL_GET_HW(uart_num), \
  INIT_CRIT_SECTION_LOCK_IN_STRUCT(spinlock) \
  .hw_enabled = false, \
}

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE

#ifdef CONFIG_ESPRESSIF_UART0

static uart_hal_context_t g_uart0_hal =
{
  .dev = &UART0
};

struct esp_uart_s g_uart0_config =
{
  .cpuint = -ENOMEM,
  .int_pri = ESP_IRQ_PRIORITY_DEFAULT,
  .id = 0,
  .baud = CONFIG_UART0_BAUD,
  .stop_b2 =  CONFIG_UART0_2STOP,
  .bits = CONFIG_UART0_BITS,
  .parity = CONFIG_UART0_PARITY,
  .txpin = CONFIG_ESPRESSIF_UART0_TXPIN,
  .rxpin = CONFIG_ESPRESSIF_UART0_RXPIN,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rtspin = CONFIG_ESPRESSIF_UART0_RTSPIN,
#ifdef CONFIG_UART0_IFLOWCONTROL
  .iflow  = true,    /* input flow control (RTS) enabled */
#else
  .iflow  = false,   /* input flow control (RTS) disabled */
#endif
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .ctspin = CONFIG_ESPRESSIF_UART0_CTSPIN,
#ifdef CONFIG_UART0_OFLOWCONTROL
  .oflow  = true,    /* output flow control (CTS) enabled */
#else
  .oflow  = false,   /* output flow control (CTS) disabled */
#endif
#endif
#ifdef CONFIG_ESPRESSIF_UART0_RS485
  .rs485_dir_gpio = CONFIG_ESPRESSIF_UART0_RS485_DIR_PIN,
#if (CONFIG_ESPRESSIF_UART0_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#else
  .rs485_dir_polarity = true,
#endif
#endif
  .clk_src = UART_SCLK_DEFAULT,
  .hal = &g_uart0_hal,
  .lock = SP_UNLOCKED
};

#endif /* CONFIG_ESPRESSIF_UART0 */

#ifdef CONFIG_ESPRESSIF_UART1

static uart_hal_context_t g_uart1_hal =
{
  .dev = &UART1
};

struct esp_uart_s g_uart1_config =
{
  .cpuint = -ENOMEM,
  .int_pri = ESP_IRQ_PRIORITY_DEFAULT,
  .id = 1,
  .baud = CONFIG_UART1_BAUD,
  .stop_b2 = CONFIG_UART1_2STOP,
  .bits = CONFIG_UART1_BITS,
  .parity = CONFIG_UART1_PARITY,
  .txpin = CONFIG_ESPRESSIF_UART1_TXPIN,
  .rxpin = CONFIG_ESPRESSIF_UART1_RXPIN,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rtspin = CONFIG_ESPRESSIF_UART1_RTSPIN,
#ifdef CONFIG_UART1_IFLOWCONTROL
  .iflow  = true,    /* input flow control (RTS) enabled */
#else
  .iflow  = false,   /* input flow control (RTS) disabled */
#endif
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .ctspin = CONFIG_ESPRESSIF_UART1_CTSPIN,
#ifdef CONFIG_UART1_OFLOWCONTROL
  .oflow  = true,    /* output flow control (CTS) enabled */
#else
  .oflow  = false,   /* output flow control (CTS) disabled */
#endif
#endif
#ifdef CONFIG_ESPRESSIF_UART1_RS485
  .rs485_dir_gpio = CONFIG_ESPRESSIF_UART1_RS485_DIR_PIN,
#if (CONFIG_ESPRESSIF_UART1_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#else
  .rs485_dir_polarity = true,
#endif
#endif
  .clk_src = UART_SCLK_DEFAULT,
  .hal = &g_uart1_hal,
  .lock = SP_UNLOCKED
};

#endif /* CONFIG_ESPRESSIF_UART1 */

#ifdef CONFIG_ESPRESSIF_LP_UART0

static uart_hal_context_t g_lp_uart0_hal =
{
  .dev = (hal_uart_dev_t *)&LP_UART
};

struct esp_uart_s g_lp_uart0_config =
{
  .cpuint = -ENOMEM,
  .int_pri = ESP_IRQ_PRIORITY_DEFAULT,
  .id = ESP_LP_UART0_ID,
  .baud = CONFIG_LPUART0_BAUD,
  .stop_b2 = CONFIG_LPUART0_2STOP,
  .bits = CONFIG_LPUART0_BITS,
  .parity = CONFIG_LPUART0_PARITY,
  .txpin = LP_UART_DEFAULT_TX_GPIO_NUM,
  .rxpin = LP_UART_DEFAULT_RX_GPIO_NUM,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rtspin = LP_UART_DEFAULT_RTS_GPIO_NUM,
#ifdef CONFIG_LPUART0_IFLOWCONTROL
  .iflow  = true,    /* input flow control (RTS) enabled */
#else
  .iflow  = false,   /* input flow control (RTS) disabled */
#endif
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .ctspin = LP_UART_DEFAULT_CTS_GPIO_NUM,
#ifdef CONFIG_LPUART0_OFLOWCONTROL
  .oflow  = true,    /* output flow control (CTS) enabled */
#else
  .oflow  = false,   /* output flow control (CTS) disabled */
#endif
#endif
  .clk_src = LP_UART_SCLK_DEFAULT,
  .hal = &g_lp_uart0_hal,
  .lock = SP_UNLOCKED
};

#endif /* CONFIG_ESPRESSIF_LP_UART0 */

#endif /* HAVE_UART_DEVICE */

/****************************************************************************
 * Public Data
 ****************************************************************************/

uart_context_t g_uart_context[UART_NUM_MAX] =
{
  UART_CONTEXT_INIT_DEF(UART_NUM_0),
  UART_CONTEXT_INIT_DEF(UART_NUM_1),
#if SOC_UART_HP_NUM > 2
  UART_CONTEXT_INIT_DEF(UART_NUM_2),
#endif
#if SOC_UART_HP_NUM > 3
  UART_CONTEXT_INIT_DEF(UART_NUM_3),
#endif
#if SOC_UART_HP_NUM > 4
  UART_CONTEXT_INIT_DEF(UART_NUM_4),
#endif
#if (SOC_UART_LP_NUM >= 1)
  UART_CONTEXT_INIT_DEF(LP_UART_NUM_0),
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_LP_UART
/****************************************************************************
 * Name: esp_lowputc_lp_uart_config_io
 *
 * Description:
 *   Configures LP UART pin.
 *
 * Parameters:
 *   priv      - Pointer to the private driver struct.
 *   pin       - Pin number to configure.
 *   direction - Pin direction to configure.
 *   idx       - Pin idx to configure.
 *
 * Return Value:
 *   None.
 *
 ****************************************************************************/

static void esp_lowputc_lp_uart_config_io(const struct esp_uart_s *priv,
                                          int8_t pin,
                                          rtc_gpio_mode_t direction,
                                          uint32_t idx)
{
  irqstate_t flags = spin_lock_irqsave(&priv->lock);
  int lp_pin = rtc_io_num_map[pin];

  DEBUGASSERT(lp_pin != -1);

#if SOC_LP_IO_CLOCK_IS_INDEPENDENT
  io_mux_enable_lp_io_clock(lp_pin, true);
#endif
  rtcio_hal_function_select(lp_pin, RTCIO_LL_FUNC_RTC);
  rtcio_hal_set_direction(pin, direction);

  const uart_periph_sig_t *upin =
    &uart_periph_signal[LP_UART_NUM_0].pins[idx];
#if !SOC_LP_GPIO_MATRIX_SUPPORTED
  rtc_gpio_iomux_func_sel(pin, upin->iomux_func);
#else
  /* ToDo: Add LP UART for LP GPIO Matrix supported devices (e.g ESP32-P4) */
#endif /* SOC_LP_GPIO_MATRIX_SUPPORTED */

  spin_unlock_irqrestore(&priv->lock, flags);
}

#endif /* CONFIG_ESPRESSIF_LP_UART */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool esp_lowputc_uart_module_enable(const struct esp_uart_s *priv)
{
  int uart_num = priv->id;
  bool newly_enabled = false;
  mutex_t lock;

  nxmutex_init(&lock);

  g_uart_context[uart_num].mutex = (_lock_t)&lock;

  _lock_acquire(&(g_uart_context[uart_num].mutex));

  if (g_uart_context[uart_num].hw_enabled != true)
    {
      if (uart_num < SOC_UART_HP_NUM)
        {
          HP_UART_BUS_CLK_ATOMIC()
            {
              uart_ll_enable_bus_clock(uart_num, true);
            }

          if (uart_num != CONFIG_ESP_CONSOLE_UART_NUM)
            {
              HP_UART_BUS_CLK_ATOMIC()
                {
                  uart_ll_reset_register(uart_num);
                }

              HP_UART_SRC_CLK_ATOMIC()
                {
                  uart_ll_sclk_enable(g_uart_context[uart_num].hal.dev);
                }
            }
        }

      g_uart_context[uart_num].hw_enabled = true;
      newly_enabled = true;
    }

  _lock_release(&(g_uart_context[uart_num].mutex));
  nxmutex_destroy(&lock);
  g_uart_context[uart_num].mutex = NULL;
  return newly_enabled;
}

/****************************************************************************
 * Name: esp_lowputc_send_byte
 *
 * Description:
 *   Send one byte.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   byte           - Byte to be sent.
 *
 ****************************************************************************/

void esp_lowputc_send_byte(const struct esp_uart_s *priv, char byte)
{
  uint32_t write_size;
  uart_hal_write_txfifo(priv->hal, (const uint8_t *)&byte, 1, &write_size);
}

/****************************************************************************
 * Name: esp_lowputc_disable_all_uart_int
 *
 * Description:
 *   Disable all UART interrupts.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   current_status - Pointer to a variable to store the current status of
 *                    the interrupt enable register before disabling
 *                    UART interrupts.
 *
 ****************************************************************************/

void esp_lowputc_disable_all_uart_int(struct esp_uart_s *priv,
                                      uint32_t *current_status)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  if (current_status != NULL)
    {
      /* Save current status */

      *current_status = uart_hal_get_intr_ena_status(priv->hal);
    }

  /* Disable all UART int */

  uart_hal_disable_intr_mask(priv->hal, UINT32_MAX);

  /* Clear all ints */

  uart_hal_clr_intsts_mask(priv->hal, UINT32_MAX);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp_lowputc_restore_all_uart_int
 *
 * Description:
 *   Restore all UART interrupts.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *   last_status - Pointer to a variable that stored the last state of the
 *                 interrupt enable register.
 *
 ****************************************************************************/

void esp_lowputc_restore_all_uart_int(const struct esp_uart_s *priv,
                                      uint32_t *last_status)
{
  /* Restore the previous behaviour */

  uart_hal_ena_intr_mask(priv->hal, *last_status);
}

/****************************************************************************
 * Name: esp_lowputc_config_pins
 *
 * Description:
 *   Configure TX and RX UART pins.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp_lowputc_config_pins(const struct esp_uart_s *priv)
{
  /* Configure the pins */

  if (priv->id < ESP_LP_UART0_ID)
    {
      esp_configgpio(priv->rxpin, INPUT | PULLUP);
      esp_gpio_matrix_in(priv->rxpin,
                         UART_PERIPH_SIGNAL(priv->id, SOC_UART_RX_PIN_IDX),
                         0);

      esp_configgpio(priv->txpin, OUTPUT);
      esp_gpio_matrix_out(priv->txpin,
                          UART_PERIPH_SIGNAL(priv->id, SOC_UART_TX_PIN_IDX),
                          0, 0);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          uint32_t sig = UART_PERIPH_SIGNAL(priv->id, SOC_UART_RTS_PIN_IDX);

          esp_configgpio(priv->rtspin, OUTPUT);
          esp_gpio_matrix_out(priv->rtspin, sig, 0, 0);
        }

#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
      if (priv->oflow)
        {
          uint32_t sig = UART_PERIPH_SIGNAL(priv->id, SOC_UART_CTS_PIN_IDX);

          esp_configgpio(priv->ctspin, INPUT | PULLUP);
          esp_gpio_matrix_in(priv->ctspin, sig, 0);
        }
#endif

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      esp_configgpio(priv->rs485_dir_gpio, OUTPUT);
      esp_gpio_matrix_out(priv->rs485_dir_gpio, SIG_GPIO_OUT_IDX, 0, 0);
      esp_gpiowrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
    }
#endif
    }
#ifdef CONFIG_ESPRESSIF_LP_UART
  else
    {
      esp_lowputc_lp_uart_config_io(priv,
                                    priv->rxpin,
                                    RTC_GPIO_MODE_INPUT_ONLY,
                                    SOC_UART_RX_PIN_IDX);

      esp_lowputc_lp_uart_config_io(priv,
                                    priv->txpin,
                                    RTC_GPIO_MODE_OUTPUT_ONLY,
                                    SOC_UART_TX_PIN_IDX);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          esp_lowputc_lp_uart_config_io(priv,
                                        priv->rtspin,
                                        RTC_GPIO_MODE_OUTPUT_ONLY,
                                        SOC_UART_RTS_PIN_IDX);
        }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
      if (priv->oflow)
        {
          esp_lowputc_lp_uart_config_io(priv,
                                        priv->ctspin,
                                        RTC_GPIO_MODE_INPUT_ONLY,
                                        SOC_UART_CTS_PIN_IDX);
        }
#endif
    }
#endif /* CONFIG_ESPRESSIF_LP_UART */
}

/****************************************************************************
 * Name: esp_lowputc_restore_pins
 *
 * Description:
 *   Configure both pins back to INPUT mode and detach the TX pin from the
 *   output signal and the RX pin from the input signal.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp_lowputc_restore_pins(const struct esp_uart_s *priv)
{
  /* Configure the pins */

  esp_configgpio(priv->txpin, INPUT);
  esp_gpio_matrix_out(priv->txpin, 0x100, false, false);

  esp_configgpio(priv->rxpin, INPUT);
  esp_gpio_matrix_in(priv->rxpin, 0x3c, false);
}

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console.
 *
 * Parameters:
 *   ch        - Byte to be sent.
 *
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#ifdef CONSOLE_UART
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
  struct esp_uart_s *priv = &g_uart0_config;
#  elif defined (CONFIG_UART1_SERIAL_CONSOLE)
  struct esp_uart_s *priv = &g_uart1_config;
#  elif defined (CONFIG_LPUART0_SERIAL_CONSOLE)
  struct esp_uart_s *priv = &g_lp_uart0_config;
#  endif

  /* Wait until the TX FIFO has space to insert new char */

  while (uart_hal_get_txfifo_len(priv->hal) == 0);

  /* Then send the character */

  esp_lowputc_send_byte(priv, ch);
#elif defined(CONFIG_ESPRESSIF_USBSERIAL)
  esp_usbserial_write(ch);
#endif /* CONSOLE_UART */
}

/****************************************************************************
 * Name: esp_lowsetup
 *
 * Description:
 *   This performs only the basic configuration for UART pins.
 *
 ****************************************************************************/

void esp_lowsetup(void)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG

#ifdef CONFIG_ESPRESSIF_UART0
  esp_lowputc_uart_module_enable(&g_uart0_config);
  esp_lowputc_config_pins(&g_uart0_config);
#endif

#ifdef CONFIG_ESPRESSIF_UART1
  esp_lowputc_uart_module_enable(&g_uart1_config);
  esp_lowputc_config_pins(&g_uart1_config);
#endif

#ifdef CONFIG_ESPRESSIF_LP_UART0
  esp_lowputc_enable_sysclk(&g_lp_uart0_config);
  esp_lowputc_config_pins(&g_lp_uart0_config);
#endif

#endif /* !CONFIG_SUPPRESS_UART_CONFIG */
}
