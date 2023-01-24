/****************************************************************************
 * arch/risc-v/src/espressif/esp_lowputc.c
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

#include "chip.h"
#include "riscv_internal.h"

#include "esp_config.h"
#include "esp_gpio.h"
#include "esp_irq.h"
#include "esp_lowputc.h"

#include "hal/uart_hal.h"
#include "periph_ctrl.h"
#include "soc/gpio_sig_map.h"

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
  .source = UART0_INTR_SOURCE,
  .cpuint = -ENOMEM,
  .int_pri = ESP_IRQ_PRIORITY_DEFAULT,
  .id = 0,
  .irq = ESP_IRQ_UART0,
  .baud = CONFIG_UART0_BAUD,
  .stop_b2 =  CONFIG_UART0_2STOP,
  .bits = CONFIG_UART0_BITS,
  .parity = CONFIG_UART0_PARITY,
  .txpin = CONFIG_ESPRESSIF_UART0_TXPIN,
  .txsig = U0TXD_OUT_IDX,
  .rxpin = CONFIG_ESPRESSIF_UART0_RXPIN,
  .rxsig = U0RXD_IN_IDX,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rtspin = CONFIG_ESPRESSIF_UART0_RTSPIN,
  .rtssig = U0RTS_OUT_IDX,
#ifdef CONFIG_UART0_IFLOWCONTROL
  .iflow  = true,    /* input flow control (RTS) enabled */
#else
  .iflow  = false,   /* input flow control (RTS) disabled */
#endif
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .ctspin = CONFIG_ESPRESSIF_UART0_CTSPIN,
  .ctssig = U0CTS_IN_IDX,
#ifdef CONFIG_UART0_OFLOWCONTROL
  .oflow  = true,    /* output flow control (CTS) enabled */
#else
  .oflow  = false,   /* output flow control (CTS) disabled */
#endif
#endif
  .hal = &g_uart0_hal
};

#endif /* CONFIG_ESPRESSIF_UART0 */

#ifdef CONFIG_ESPRESSIF_UART1

static uart_hal_context_t g_uart1_hal =
{
  .dev = &UART1
};

struct esp_uart_s g_uart1_config =
{
  .source = UART1_INTR_SOURCE,
  .cpuint = -ENOMEM,
  .int_pri = ESP_IRQ_PRIORITY_DEFAULT,
  .id = 1,
  .irq = ESP_IRQ_UART1,
  .baud = CONFIG_UART1_BAUD,
  .stop_b2 = CONFIG_UART1_2STOP,
  .bits = CONFIG_UART1_BITS,
  .parity = CONFIG_UART1_PARITY,
  .txpin = CONFIG_ESPRESSIF_UART1_TXPIN,
  .txsig = U1TXD_OUT_IDX,
  .rxpin = CONFIG_ESPRESSIF_UART1_RXPIN,
  .rxsig = U1RXD_IN_IDX,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rtspin = CONFIG_ESPRESSIF_UART1_RTSPIN,
  .rtssig = U1RTS_OUT_IDX,
#ifdef CONFIG_UART1_IFLOWCONTROL
  .iflow  = true,    /* input flow control (RTS) enabled */
#else
  .iflow  = false,   /* input flow control (RTS) disabled */
#endif
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .ctspin = CONFIG_ESPRESSIF_UART1_CTSPIN,
  .ctssig = U1CTS_IN_IDX,
#ifdef CONFIG_UART1_OFLOWCONTROL
  .oflow  = true,    /* output flow control (CTS) enabled */
#else
  .oflow  = false,   /* output flow control (CTS) disabled */
#endif
#endif
  .hal = &g_uart1_hal
};

#endif /* CONFIG_ESPRESSIF_UART1 */
#endif /* HAVE_UART_DEVICE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 * Name: esp_lowputc_enable_sysclk
 *
 * Description:
 *   Enable clock for the UART using the System register.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp_lowputc_enable_sysclk(const struct esp_uart_s *priv)
{
  periph_module_enable(PERIPH_UART0_MODULE + priv->id);
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

void esp_lowputc_disable_all_uart_int(const struct esp_uart_s *priv,
                                      uint32_t *current_status)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (current_status != NULL)
    {
      /* Save current status */

      *current_status = uart_hal_get_intr_ena_status(priv->hal);
    }

  /* Disable all UART int */

  uart_hal_disable_intr_mask(priv->hal, UINT32_MAX);

  /* Clear all ints */

  uart_hal_clr_intsts_mask(priv->hal, UINT32_MAX);

  leave_critical_section(flags);
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

  esp_configgpio(priv->txpin, OUTPUT);
  esp_gpio_matrix_out(priv->txpin, priv->txsig, 0, 0);

  esp_configgpio(priv->rxpin, INPUT | PULLUP);
  esp_gpio_matrix_in(priv->rxpin, priv->rxsig, 0);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      esp_configgpio(priv->rtspin, OUTPUT);
      esp_gpio_matrix_out(priv->rtspin, priv->rtssig, 0, 0);
    }

#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->oflow)
    {
      esp_configgpio(priv->ctspin, INPUT | PULLUP);
      esp_gpio_matrix_in(priv->ctspin, priv->ctssig, 0);
    }
#endif
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
#endif

  /* Wait until the TX FIFO has space to insert new char */

  while (uart_hal_get_txfifo_len(priv->hal) == 0);

  /* Then send the character */

  esp_lowputc_send_byte(priv, ch);
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
  esp_lowputc_enable_sysclk(&g_uart0_config);
  esp_lowputc_config_pins(&g_uart0_config);
#endif

#ifdef CONFIG_ESPRESSIF_UART1
  esp_lowputc_enable_sysclk(&g_uart1_config);
  esp_lowputc_config_pins(&g_uart1_config);
#endif

#endif /* !CONFIG_SUPPRESS_UART_CONFIG */
}
