/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_lowputc.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "chip.h"
#include "riscv_arch.h"

#include "hardware/esp32c3_system.h"
#include "hardware/esp32c3_uart.h"
#include "hardware/esp32c3_soc.h"

#include "esp32c3_clockconfig.h"
#include "esp32c3_config.h"
#include "esp32c3_gpio.h"

#include "esp32c3_lowputc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)

static const struct esp32c3_uart_s g_console_config =
{
  .periph = ESP32C3_PERIPH_UART0,
  .cpuint = -ENOMEM,
  .id = 0,
  .irq = ESP32C3_IRQ_UART0,
  .baud = CONFIG_UART0_BAUD,
  .bits = CONFIG_UART0_BITS,
  .parity = CONFIG_UART0_PARITY,
  .stop_b2 =  CONFIG_UART0_2STOP,
  .int_pri = ESP32C3_INT_PRIO_DEF,
  .txpin = CONFIG_ESP32C3_UART0_TXPIN,
  .txsig = U0TXD_OUT_IDX,
  .rxpin = CONFIG_ESP32C3_UART0_RXPIN,
  .rxsig = U0RXD_IN_IDX,
};

# elif defined(CONFIG_UART1_SERIAL_CONSOLE)

static const struct esp32c3_uart_s g_console_config =
{
  .periph = ESP32C3_PERIPH_UART1,
  .cpuint = -ENOMEM,
  .id = 1,
  .irq = ESP32C3_IRQ_UART1,
  .baud = CONFIG_UART1_BAUD,
  .bits = CONFIG_UART1_BITS,
  .parity = CONFIG_UART1_PARITY,
  .stop_b2 =  CONFIG_UART1_2STOP,
  .int_pri = ESP32C3_INT_PRIO_DEF,
  .txpin = CONFIG_ESP32C3_UART1_TXPIN,
  .txsig = U1TXD_OUT_IDX,
  .rxpin = CONFIG_ESP32C3_UART1_RXPIN,
  .rxsig = U1RXD_IN_IDX,
};
#endif /* CONFIG_UART0_SERIAL_CONSOLE */
#endif /* HAVE_SERIAL_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_lowputc_reset_core
 *
 * Description:
 *   Reset both TX and RX cores.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_reset_cores(const struct esp32c3_uart_s *priv)
{
  uint32_t set_bit = 1 << UART_RST_CORE_S;
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RST_CORE_M, set_bit);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RST_CORE_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_tx
 *
 * Description:
 *   Reset TX core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_tx(const struct esp32c3_uart_s *priv)
{
  uint32_t set_bit = 1 << UART_TX_RST_CORE_S;

  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_TX_RST_CORE_M, set_bit);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_TX_RST_CORE_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_rx
 *
 * Description:
 *   Reset RX core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_rx(const struct esp32c3_uart_s *priv)
{
  uint32_t set_bit = 1 << UART_RX_RST_CORE_S;

  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RX_RST_CORE_M, set_bit);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RX_RST_CORE_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_enable_sclk
 *
 * Description:
 *   Enable clock for whole core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_enable_sclk(const struct esp32c3_uart_s *priv)
{
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_SCLK_EN_M,
              1 << UART_SCLK_EN_S);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RX_SCLK_EN_M,
              1 << UART_RX_SCLK_EN_S);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_TX_SCLK_EN_M,
              1 << UART_TX_SCLK_EN_S);
}

/****************************************************************************
 * Name: esp32c3_lowputc_disable_sclk
 *
 * Description:
 *   Disable clock for whole core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_disable_sclk(const struct esp32c3_uart_s *priv)
{
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_SCLK_EN_M, 0);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RX_SCLK_EN_M, 0);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_TX_SCLK_EN_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_set_sclk
 *
 * Description:
 *   Set a source clock for UART.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   source         - APB_CLK  = 1  80 MHz
 *                    CLK_8    = 2  8 MHz
 *                    XTAL_CLK = 3
 *
 ****************************************************************************/

void esp32c3_lowputc_set_sclk(const struct esp32c3_uart_s *priv,
                              enum uart_sclk source)
{
  uint32_t clk = (uint32_t)source << UART_SCLK_SEL_S;
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_SCLK_SEL_M, clk);
}

/****************************************************************************
 * Name: esp32c3_lowputc_get_sclk
 *
 * Description:
 *   Get the source clock for UART.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 * Returned Value:
 *   The frequency of the clock in Hz.
 *
 ****************************************************************************/

uint32_t esp32c3_lowputc_get_sclk(const struct esp32c3_uart_s * priv)
{
  uint32_t clk_conf_reg;
  uint32_t ret = -ENODATA;
  clk_conf_reg   = getreg32(UART_CLK_CONF_REG(priv->id));
  clk_conf_reg  &= UART_SCLK_SEL_M;
  clk_conf_reg >>= UART_SCLK_SEL_S;
  switch (clk_conf_reg)
    {
      case 1:
        ret = esp32c3_clk_apb_freq();
        break;
      case 2:
        ret = RTC_CLK_FREQ;
        break;
      case 3:
        ret = XTAL_CLK_FREQ;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_lowputc_baud
 *
 * Description:
 *   Set the baud rate according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_baud(const struct esp32c3_uart_s * priv)
{
  const int sclk_div = 1;
  uint32_t sclk_freq = esp32c3_lowputc_get_sclk(priv);
  uint32_t clk_div = ((sclk_freq) << 4) / priv->baud;
  uint32_t int_part = clk_div >> 4;
  uint32_t frag_part = clk_div &  0xf;

  /* The baud rate configuration register is divided into
   * an integer part and a fractional part.
   */

  modifyreg32(UART_CLKDIV_REG(priv->id), UART_CLKDIV_M, int_part);
  modifyreg32(UART_CLKDIV_REG(priv->id), UART_CLKDIV_FRAG_M,
                              frag_part << UART_CLKDIV_FRAG_S);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_SCLK_DIV_NUM_M,
                                (sclk_div - 1) << UART_SCLK_DIV_NUM_S);
}

/****************************************************************************
 * Name: esp32c3_lowputc_normal_mode
 *
 * Description:
 *   Set the UART to operate in normal mode, i.e., disable the RS485 mode and
 *   IRDA mode.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_normal_mode(const struct esp32c3_uart_s * priv)
{
  /* Disable RS485 mode */

  modifyreg32(UART_RS485_CONF_REG(priv->id), UART_RS485_EN_M, 0);
  modifyreg32(UART_RS485_CONF_REG(priv->id), UART_RS485TX_RX_EN_M, 0);
  modifyreg32(UART_RS485_CONF_REG(priv->id), UART_RS485RXBY_TX_EN_M, 0);

  /* Disable IRDA mode */

  modifyreg32(UART_CONF0_REG(priv->id), UART_IRDA_EN_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_parity
 *
 * Description:
 *   Set the parity, according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_parity(const struct esp32c3_uart_s * priv)
{
  if (priv->parity == UART_PARITY_DISABLE)
    {
      modifyreg32(UART_CONF0_REG(priv->id), UART_PARITY_EN_M, 0);
    }
  else
    {
      modifyreg32(UART_CONF0_REG(priv->id), UART_PARITY_M,
                  ((priv->parity & 0x1) << UART_PARITY_S));
      modifyreg32(UART_CONF0_REG(priv->id), UART_PARITY_EN_M,
                                 1 << UART_PARITY_EN_S);
    }
}

/****************************************************************************
 * Name: esp32c3_lowputc_data_length
 *
 * Description:
 *   Set the data bits length, according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

int esp32c3_lowputc_data_length(const struct esp32c3_uart_s * priv)
{
  int ret = OK;
  uint32_t length = (priv->bits - 5);

  /* If it is the allowed range */

  if (length >= UART_DATA_5_BITS && length <= UART_DATA_8_BITS)
    {
      modifyreg32(UART_CONF0_REG(priv->id), UART_BIT_NUM_M,
                    length << UART_BIT_NUM_S);
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_lowputc_stop_length
 *
 * Description:
 *   Set the stop bits length, according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_stop_length(const struct esp32c3_uart_s *priv)
{
  if (priv->stop_b2 == 0)
    {
      modifyreg32(UART_CONF0_REG(priv->id), UART_STOP_BIT_NUM_M,
                    UART_STOP_BITS_1 << UART_STOP_BIT_NUM_S);
    }
  else
    {
      modifyreg32(UART_CONF0_REG(priv->id), UART_STOP_BIT_NUM_M,
                    UART_STOP_BITS_2 << UART_STOP_BIT_NUM_S);
    }
}

/****************************************************************************
 * Name: esp32c3_lowputc_set_tx_idle_time
 *
 * Description:
 *   Set the idle time between transfers.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   time           - Desired time interval between the transfers.
 *
 ****************************************************************************/

void esp32c3_lowputc_set_tx_idle_time(const struct esp32c3_uart_s *priv,
                                      uint32_t time)
{
  time = time << UART_TX_IDLE_NUM_S;
  time = time & UART_TX_IDLE_NUM_M; /* Just in case value overloads */
  modifyreg32(UART_IDLE_CONF_REG(priv->id), UART_TX_IDLE_NUM_M,
              time);
}

/****************************************************************************
 * Name: esp32c3_lowputc_send_byte
 *
 * Description:
 *   Send one byte.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   byte           - Byte to be sent.
 *
 ****************************************************************************/

void esp32c3_lowputc_send_byte(const struct esp32c3_uart_s * priv,
                               char byte)
{
  putreg32((uint32_t) byte, UART_FIFO_REG(priv->id));
}

/****************************************************************************
 * Name: esp32c3_lowputc_is_tx_fifo_full
 *
 * Description:
 *   Verify if TX FIFO is full.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 * Returned Value:
 *   True if it is full, otherwise false.
 *
 ****************************************************************************/

bool esp32c3_lowputc_is_tx_fifo_full(const struct esp32c3_uart_s *priv)
{
  uint32_t reg;
  reg = getreg32(UART_STATUS_REG(priv->id));
  reg = reg >> UART_TXFIFO_CNT_S;
  reg = reg & UART_TXFIFO_CNT_V;
  if (reg < (UART_TX_FIFO_SIZE -1))
    {
      return false;
    }
  else
    {
      return true;
    }
}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_peripheral
 *
 * Description:
 *   Reset the UART peripheral by using System reg.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_peripheral(const struct esp32c3_uart_s *priv)
{
  if (priv->id == 0)
    {
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_UART_RST_M,
                  SYSTEM_UART_RST_M);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_UART_RST_M, 0);
    }
  else
    {
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_UART1_RST_M,
                  SYSTEM_UART1_RST_M);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_UART1_RST_M, 0);
    }
}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_txfifo
 *
 * Description:
 *   Reset TX FIFO.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_txfifo(const struct esp32c3_uart_s *priv)
{
  modifyreg32(UART_CONF0_REG(priv->id), UART_TXFIFO_RST_M,
                             UART_TXFIFO_RST_M);
  modifyreg32(UART_CONF0_REG(priv->id), UART_TXFIFO_RST_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_rxfifo
 *
 * Description:
 *   Reset RX FIFO.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_rst_rxfifo(const struct esp32c3_uart_s *priv)
{
  modifyreg32(UART_CONF0_REG(priv->id), UART_RXFIFO_RST_M,
                             UART_RXFIFO_RST_M);
  modifyreg32(UART_CONF0_REG(priv->id), UART_RXFIFO_RST_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_disable_all_uart_int
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

void esp32c3_lowputc_disable_all_uart_int(const struct esp32c3_uart_s *priv,
                                          uint32_t *current_status)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (current_status != NULL)
    {
      /* Save current status */

      *current_status = getreg32(UART_INT_ENA_REG(priv->id));
    }

  /* Disable all UART int */

  putreg32(0, UART_INT_ENA_REG(priv->id));

  /* Clear all ints */

  putreg32(0xffffffff, UART_INT_CLR_REG(priv->id));

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32c3_lowputc_restore_all_uart_int
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

void esp32c3_lowputc_restore_all_uart_int(const struct esp32c3_uart_s *priv,
                                          uint32_t *last_status)
{
  /* Restore the previous behaviour */

  putreg32(*last_status, UART_INT_ENA_REG(priv->id));
}

/****************************************************************************
 * Name: esp32c3_lowputc_config_pins
 *
 * Description:
 *   Configure TX and RX UART pins.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_config_pins(const struct esp32c3_uart_s *priv)
{
  /* Configure the pins */

  esp32c3_configgpio(priv->txpin, OUTPUT_FUNCTION_1);
  esp32c3_gpio_matrix_out(priv->txpin, priv->txsig, 0, 0);

  esp32c3_configgpio(priv->rxpin, INPUT_FUNCTION_1);
  esp32c3_gpio_matrix_in(priv->rxpin, priv->rxsig, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_restore_pins
 *
 * Description:
 *   Configure both pins back to INPUT mode and detach the TX pin from the
 *   output signal and the RX pin from the input signal.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32c3_lowputc_restore_pins(const struct esp32c3_uart_s *priv)
{
  /* Configure the pins */

  esp32c3_configgpio(priv->txpin, INPUT);
  esp32c3_gpio_matrix_out(priv->txpin, MATRIX_DETACH_OUT_SIG, false, false);

  esp32c3_configgpio(priv->rxpin, INPUT);
  esp32c3_gpio_matrix_in(priv->rxpin, MATRIX_DETACH_IN_LOW_PIN, false);
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
#ifdef HAVE_SERIAL_CONSOLE

  /* Wait until the TX FIFO has space to insert new char */

  while (esp32c3_lowputc_is_tx_fifo_full(&g_console_config));

  /* Then send the character */

  esp32c3_lowputc_send_byte(&g_console_config, ch);

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: esp32c3_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void esp32c3_lowsetup(void)
{
  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Initialize UART module */

  /* Configure the UART Baud Rate */

  esp32c3_lowputc_baud(&g_console_config);

  /* Set a mode */

  esp32c3_lowputc_normal_mode(&g_console_config);

  /* Parity */

  esp32c3_lowputc_parity(&g_console_config);

  /* Data Frame size */

  esp32c3_lowputc_data_length(&g_console_config);

  /* Stop bit */

  esp32c3_lowputc_stop_length(&g_console_config);

  /* No Tx idle interval */

  esp32c3_lowputc_set_tx_idle_time(&g_console_config, 0);

  /* Set pins */

  esp32c3_lowputc_config_pins(&g_console_config);

  /* Enable cores */

  esp32c3_lowputc_enable_sclk(&g_console_config);

#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
}
