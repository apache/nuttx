/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_lowputc.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "chip.h"
#include "xtensa.h"

#include "hardware/esp32s3_pinmap.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_uart.h"
#include "hardware/esp32s3_soc.h"

#include "esp32s3_clockconfig.h"
#include "esp32s3_config.h"
#include "esp32s3_gpio.h"

#include "esp32s3_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE

#ifdef CONFIG_ESP32S3_UART0

struct esp32s3_uart_s g_uart0_config =
{
  .periph = ESP32S3_PERIPH_UART0,
  .id = 0,
  .cpuint = -ENOMEM,
  .irq = ESP32S3_IRQ_UART0,
  .baud = CONFIG_UART0_BAUD,
  .bits = CONFIG_UART0_BITS,
  .parity = CONFIG_UART0_PARITY,
  .stop_b2 = CONFIG_UART0_2STOP,
  .int_pri = ESP32S3_INT_PRIO_DEF,
  .txpin = CONFIG_ESP32S3_UART0_TXPIN,
  .txsig = U0TXD_OUT_IDX,
  .rxpin = CONFIG_ESP32S3_UART0_RXPIN,
  .rxsig = U0RXD_IN_IDX,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rtspin = CONFIG_ESP32S3_UART0_RTSPIN,
  .rtssig = U0RTS_OUT_IDX,
#ifdef CONFIG_UART0_IFLOWCONTROL
  .iflow = true,    /* input flow control (RTS) enabled */
#else
  .iflow = false,   /* input flow control (RTS) disabled */
#endif
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .ctspin = CONFIG_ESP32S3_UART0_CTSPIN,
  .ctssig = U0CTS_IN_IDX,
#ifdef CONFIG_UART0_OFLOWCONTROL
  .oflow = true,    /* output flow control (CTS) enabled */
#else
  .oflow = false,   /* output flow control (CTS) disabled */
#endif
#endif
};

#endif /* CONFIG_ESP32S3_UART0 */

#ifdef CONFIG_ESP32S3_UART1

struct esp32s3_uart_s g_uart1_config =
{
  .periph = ESP32S3_PERIPH_UART1,
  .id = 1,
  .cpuint = -ENOMEM,
  .irq = ESP32S3_IRQ_UART1,
  .baud = CONFIG_UART1_BAUD,
  .bits = CONFIG_UART1_BITS,
  .parity = CONFIG_UART1_PARITY,
  .stop_b2 = CONFIG_UART1_2STOP,
  .int_pri = ESP32S3_INT_PRIO_DEF,
  .txpin = CONFIG_ESP32S3_UART1_TXPIN,
  .txsig = U1TXD_OUT_IDX,
  .rxpin = CONFIG_ESP32S3_UART1_RXPIN,
  .rxsig = U1RXD_IN_IDX,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rtspin = CONFIG_ESP32S3_UART1_RTSPIN,
  .rtssig = U1RTS_OUT_IDX,
#ifdef CONFIG_UART1_IFLOWCONTROL
  .iflow = true,    /* input flow control (RTS) enabled */
#else
  .iflow = false,   /* input flow control (RTS) disabled */
#endif
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .ctspin = CONFIG_ESP32S3_UART1_CTSPIN,
  .ctssig = U1CTS_IN_IDX,
#ifdef CONFIG_UART1_OFLOWCONTROL
  .oflow = true,    /* output flow control (CTS) enabled */
#else
  .oflow = false,   /* output flow control (CTS) disabled */
#endif
#endif
};

#endif /* CONFIG_ESP32S3_UART1 */
#endif /* HAVE_UART_DEVICE */

/****************************************************************************
 * Name: uart_is_iomux
 *
 * Description:
 *   Check if the selected UART pins can use IOMUX directly. Otherwise, UART
 *   signals will be routed via GPIO Matrix.
 *
 * Input Parameters:
 *   priv          - Pointer to the private driver struct.
 *
 * Returned Value:
 *   True if can use IOMUX or false if can't.
 *
 ****************************************************************************/

static inline bool uart_is_iomux(const struct esp32s3_uart_s *priv)
{
  bool mapped = false;

  if (priv->id == 0)
    {
      if (priv->txpin == UART0_IOMUX_TXPIN
          && priv->rxpin == UART0_IOMUX_RXPIN
#ifdef CONFIG_UART0_IFLOWCONTROL
          && priv->rtspin == UART0_IOMUX_RTSPIN
#endif
#ifdef CONFIG_UART0_OFLOWCONTROL
          && priv->ctspin == UART0_IOMUX_CTSPIN
#endif
      )
        {
          mapped = true;
        }
    }
  else if (priv->id == 1)
    {
      if (priv->txpin == UART1_IOMUX_TXPIN
          && priv->rxpin == UART1_IOMUX_RXPIN
#ifdef CONFIG_UART1_IFLOWCONTROL
          && priv->rtspin == UART1_IOMUX_RTSPIN
#endif
#ifdef CONFIG_UART1_OFLOWCONTROL
          && priv->ctspin == UART1_IOMUX_CTSPIN
#endif
      )
        {
          mapped = true;
        }
    }

  return mapped;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_lowputc_set_iflow
 *
 * Description:
 *   Configure the input hardware flow control.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   threshold      - RX FIFO value from which RST will automatically be
 *                    asserted.
 *   enable         - true = enable, false = disable
 *
 ****************************************************************************/

void esp32s3_lowputc_set_iflow(const struct esp32s3_uart_s *priv,
                               uint8_t threshold, bool enable)
{
  uint32_t mask;
  if (enable)
    {
      /* Enable RX flow control */

      modifyreg32(UART_CONF1_REG(priv->id), 0, UART_RX_FLOW_EN);

      /* Configure the threshold */

      mask = VALUE_TO_FIELD(threshold, UART_RX_FLOW_THRHD);
      modifyreg32(UART_MEM_CONF_REG(priv->id), UART_RX_FLOW_THRHD_M, mask);
    }
  else
    {
      /* Disable RX flow control */

      modifyreg32(UART_CONF1_REG(priv->id), UART_RX_FLOW_EN, 0);
    }
}

/****************************************************************************
 * Name: esp32s3_lowputc_set_oflow
 *
 * Description:
 *   Configure the output hardware flow control.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   enable         - true = enable, false = disable
 *
 ****************************************************************************/

void esp32s3_lowputc_set_oflow(const struct esp32s3_uart_s *priv,
                               bool enable)
{
  if (enable)
    {
      /* Enable TX flow control */

      modifyreg32(UART_CONF0_REG(priv->id), 0, UART_TX_FLOW_EN);
    }
  else
    {
      /* Disable TX flow control */

      modifyreg32(UART_CONF0_REG(priv->id), UART_TX_FLOW_EN, 0);
    }
}

/****************************************************************************
 * Name: esp32s3_lowputc_reset_core
 *
 * Description:
 *   Reset both TX and RX cores.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_reset_cores(const struct esp32s3_uart_s *priv)
{
  uint32_t set_bit = 1 << UART_RST_CORE_S;
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RST_CORE_M, set_bit);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RST_CORE_M, 0);
}

/****************************************************************************
 * Name: esp32s3_lowputc_rst_tx
 *
 * Description:
 *   Reset TX core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_rst_tx(const struct esp32s3_uart_s *priv)
{
  uint32_t set_bit = 1 << UART_TX_RST_CORE_S;

  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_TX_RST_CORE_M, set_bit);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_TX_RST_CORE_M, 0);
}

/****************************************************************************
 * Name: esp32s3_lowputc_rst_rx
 *
 * Description:
 *   Reset RX core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_rst_rx(const struct esp32s3_uart_s *priv)
{
  uint32_t set_bit = 1 << UART_RX_RST_CORE_S;

  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RX_RST_CORE_M, set_bit);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RX_RST_CORE_M, 0);
}

/****************************************************************************
 * Name: esp32s3_lowputc_enable_sclk
 *
 * Description:
 *   Enable clock for whole core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_enable_sclk(const struct esp32s3_uart_s *priv)
{
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_SCLK_EN_M,
              1 << UART_SCLK_EN_S);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RX_SCLK_EN_M,
              1 << UART_RX_SCLK_EN_S);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_TX_SCLK_EN_M,
              1 << UART_TX_SCLK_EN_S);
}

/****************************************************************************
 * Name: esp32s3_lowputc_disable_sclk
 *
 * Description:
 *   Disable clock for whole core.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_disable_sclk(const struct esp32s3_uart_s *priv)
{
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_SCLK_EN_M, 0);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_RX_SCLK_EN_M, 0);
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_TX_SCLK_EN_M, 0);
}

/****************************************************************************
 * Name: esp32s3_lowputc_set_sclk
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

void esp32s3_lowputc_set_sclk(const struct esp32s3_uart_s *priv,
                              enum uart_sclk source)
{
  uint32_t clk = (uint32_t)source << UART_SCLK_SEL_S;
  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_SCLK_SEL_M, clk);
}

/****************************************************************************
 * Name: esp32s3_lowputc_get_sclk
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

uint32_t esp32s3_lowputc_get_sclk(const struct esp32s3_uart_s * priv)
{
  uint32_t clk_conf_reg;
  uint32_t ret   = -ENODATA;
  clk_conf_reg   = getreg32(UART_CLK_CONF_REG(priv->id));
  clk_conf_reg  &= UART_SCLK_SEL_M;
  clk_conf_reg >>= UART_SCLK_SEL_S;
  switch (clk_conf_reg)
    {
      case 1:
        ret = esp_clk_apb_freq();
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
 * Name: esp32s3_lowputc_baud
 *
 * Description:
 *   Set the baud rate according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_baud(const struct esp32s3_uart_s * priv)
{
  int sclk_div;
  uint32_t sclk_freq;
  uint32_t clk_div;
  uint32_t int_part;
  uint32_t frag_part;

  /* Get serial clock */

  sclk_freq = esp32s3_lowputc_get_sclk(priv);

  /* Calculate integral part of the frequency divider factor.
   * For low baud rates, the sclk must be less than half.
   * For high baud rates, the sclk must be the higher.
   */

  sclk_div =  DIV_UP(sclk_freq, MAX_UART_CLKDIV * priv->baud);

  /* Calculate the clock divisor to achieve the baud rate.
   * baud = f/clk_div
   * f = sclk_freq/sclk_div
   * clk_div                 = 16*int_part + frag_part
   * 16*int_part + frag_part = 16*(sclk_freq/sclk_div)/baud
   */

  clk_div = (sclk_freq << 4) / (priv->baud * sclk_div);

  /* Get the integer part of it. */

  int_part = clk_div >> 4;

  /* Get the frag part of it. */

  frag_part = clk_div & 0xf;

  /* Set integer part of the clock divisor for baud rate. */

  modifyreg32(UART_CLKDIV_REG(priv->id), UART_CLKDIV_M, int_part);

  /* Set decimal part of the clock divisor for baud rate. */

  modifyreg32(UART_CLKDIV_REG(priv->id), UART_CLKDIV_FRAG_M,
              (frag_part & UART_CLKDIV_FRAG_V) << UART_CLKDIV_FRAG_S);

  /* Set the the integral part of the frequency divider factor. */

  modifyreg32(UART_CLK_CONF_REG(priv->id), UART_SCLK_DIV_NUM_M,
              (sclk_div - 1) << UART_SCLK_DIV_NUM_S);
}

/****************************************************************************
 * Name: esp32s3_lowputc_normal_mode
 *
 * Description:
 *   Set the UART to operate in normal mode, i.e., disable the RS485 mode and
 *   IRDA mode.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_normal_mode(const struct esp32s3_uart_s * priv)
{
  /* Disable RS485 mode */

  modifyreg32(UART_RS485_CONF_REG(priv->id), UART_RS485_EN_M, 0);
  modifyreg32(UART_RS485_CONF_REG(priv->id), UART_RS485TX_RX_EN_M, 0);
  modifyreg32(UART_RS485_CONF_REG(priv->id), UART_RS485RXBY_TX_EN_M, 0);

  /* Disable IRDA mode */

  modifyreg32(UART_CONF0_REG(priv->id), UART_IRDA_EN_M, 0);
}

/****************************************************************************
 * Name: esp32s3_lowputc_parity
 *
 * Description:
 *   Set the parity, according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_parity(const struct esp32s3_uart_s * priv)
{
  if (priv->parity == UART_PARITY_DISABLE)
    {
      modifyreg32(UART_CONF0_REG(priv->id), UART_PARITY_EN_M, 0);
    }
  else
    {
      modifyreg32(UART_CONF0_REG(priv->id), UART_PARITY_M,
                  (priv->parity & 0x1) << UART_PARITY_S);
      modifyreg32(UART_CONF0_REG(priv->id), UART_PARITY_EN_M,
                  1 << UART_PARITY_EN_S);
    }
}

/****************************************************************************
 * Name: esp32s3_lowputc_data_length
 *
 * Description:
 *   Set the data bits length, according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

int esp32s3_lowputc_data_length(const struct esp32s3_uart_s * priv)
{
  int ret = OK;
  uint32_t length = priv->bits - 5;

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
 * Name: esp32s3_lowputc_stop_length
 *
 * Description:
 *   Set the stop bits length, according to the value in the private driver
 *   struct.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_stop_length(const struct esp32s3_uart_s *priv)
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
 * Name: esp32s3_lowputc_set_tx_idle_time
 *
 * Description:
 *   Set the idle time between transfers.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   time           - Desired time interval between the transfers.
 *
 ****************************************************************************/

void esp32s3_lowputc_set_tx_idle_time(const struct esp32s3_uart_s *priv,
                                      uint32_t time)
{
  time = time << UART_TX_IDLE_NUM_S;
  time = time & UART_TX_IDLE_NUM_M; /* Just in case value overloads */
  modifyreg32(UART_IDLE_CONF_REG(priv->id), UART_TX_IDLE_NUM_M,
              time);
}

/****************************************************************************
 * Name: esp32s3_lowputc_send_byte
 *
 * Description:
 *   Send one byte.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *   byte           - Byte to be sent.
 *
 ****************************************************************************/

void esp32s3_lowputc_send_byte(const struct esp32s3_uart_s * priv,
                               char byte)
{
  putreg32((uint32_t) byte, UART_FIFO_REG(priv->id));
}

/****************************************************************************
 * Name: esp32s3_lowputc_is_tx_fifo_full
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

bool esp32s3_lowputc_is_tx_fifo_full(const struct esp32s3_uart_s *priv)
{
  uint32_t reg;

  reg = getreg32(UART_STATUS_REG(priv->id));
  reg = reg >> UART_TXFIFO_CNT_S;
  reg = reg & UART_TXFIFO_CNT_V;

  return !(reg < (UART_TX_FIFO_SIZE - 1));
}

/****************************************************************************
 * Name: esp32s3_lowputc_rst_peripheral
 *
 * Description:
 *   Reset the UART peripheral by using System reg.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_rst_peripheral(const struct esp32s3_uart_s *priv)
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
 * Name: esp32s3_lowputc_rst_txfifo
 *
 * Description:
 *   Reset TX FIFO.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_rst_txfifo(const struct esp32s3_uart_s *priv)
{
  modifyreg32(UART_CONF0_REG(priv->id), UART_TXFIFO_RST_M,
                             UART_TXFIFO_RST_M);
  modifyreg32(UART_CONF0_REG(priv->id), UART_TXFIFO_RST_M, 0);
}

/****************************************************************************
 * Name: esp32s3_lowputc_rst_rxfifo
 *
 * Description:
 *   Reset RX FIFO.
 *
 * Parameters:
 *   priv           - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_rst_rxfifo(const struct esp32s3_uart_s *priv)
{
  modifyreg32(UART_CONF0_REG(priv->id), UART_RXFIFO_RST_M,
                             UART_RXFIFO_RST_M);
  modifyreg32(UART_CONF0_REG(priv->id), UART_RXFIFO_RST_M, 0);
}

/****************************************************************************
 * Name: esp32s3_lowputc_disable_all_uart_int
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

void esp32s3_lowputc_disable_all_uart_int(struct esp32s3_uart_s *priv,
                                          uint32_t *current_status)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  if (current_status != NULL)
    {
      /* Save current status */

      *current_status = getreg32(UART_INT_ENA_REG(priv->id));
    }

  /* Disable all UART int */

  putreg32(0, UART_INT_ENA_REG(priv->id));

  /* Clear all ints */

  putreg32(0xffffffff, UART_INT_CLR_REG(priv->id));

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32s3_lowputc_restore_all_uart_int
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

void esp32s3_lowputc_restore_all_uart_int(const struct esp32s3_uart_s *priv,
                                          uint32_t *last_status)
{
  /* Restore the previous behaviour */

  putreg32(*last_status, UART_INT_ENA_REG(priv->id));
}

/****************************************************************************
 * Name: esp32s3_lowputc_config_pins
 *
 * Description:
 *   Configure TX and RX UART pins.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_config_pins(const struct esp32s3_uart_s *priv)
{
  /* Configure the pins */

  /* Keep TX pin in high level to avoid "?" trash character
   * This "?" is the Unicode replacement character (U+FFFD)
   */

  esp32s3_gpiowrite(priv->txpin, true);

  if (uart_is_iomux(priv))
    {
      esp32s3_configgpio(priv->txpin, OUTPUT_FUNCTION_1);
      esp32s3_gpio_matrix_out(priv->txpin, SIG_GPIO_OUT_IDX, 0, 0);

      esp32s3_configgpio(priv->rxpin, INPUT_FUNCTION_1);
      esp32s3_gpio_matrix_out(priv->rxpin, SIG_GPIO_OUT_IDX, 0, 0);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          esp32s3_configgpio(priv->rtspin, OUTPUT_FUNCTION_3);
          esp32s3_gpio_matrix_out(priv->rtspin, SIG_GPIO_OUT_IDX, 0, 0);
        }

#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
      if (priv->oflow)
        {
          esp32s3_configgpio(priv->ctspin, INPUT_FUNCTION_3);
          esp32s3_gpio_matrix_out(priv->ctspin, SIG_GPIO_OUT_IDX, 0, 0);
        }
#endif
    }
  else
    {
      esp32s3_configgpio(priv->txpin, OUTPUT_FUNCTION_2);
      esp32s3_gpio_matrix_out(priv->txpin, priv->txsig, 0, 0);

      esp32s3_configgpio(priv->rxpin, INPUT_FUNCTION_2);
      esp32s3_gpio_matrix_in(priv->rxpin, priv->rxsig, 0);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          esp32s3_configgpio(priv->rtspin, OUTPUT_FUNCTION_2);
          esp32s3_gpio_matrix_out(priv->rtspin, priv->rtssig, 0, 0);
        }

#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
      if (priv->oflow)
        {
          esp32s3_configgpio(priv->ctspin, INPUT_FUNCTION_2);
          esp32s3_gpio_matrix_in(priv->ctspin, priv->ctssig, 0);
        }
#endif
    }
}

/****************************************************************************
 * Name: esp32s3_lowputc_restore_pins
 *
 * Description:
 *   Configure both pins back to INPUT mode and detach the TX pin from the
 *   output signal and the RX pin from the input signal.
 *
 * Parameters:
 *   priv        - Pointer to the private driver struct.
 *
 ****************************************************************************/

void esp32s3_lowputc_restore_pins(const struct esp32s3_uart_s *priv)
{
  /* Configure the pins */

  esp32s3_configgpio(priv->txpin, INPUT);
  esp32s3_gpio_matrix_out(priv->txpin, MATRIX_DETACH_OUT_SIG, false, false);

  esp32s3_configgpio(priv->rxpin, INPUT);
  esp32s3_gpio_matrix_in(priv->rxpin, MATRIX_DETACH_IN_LOW_PIN, false);
}

/****************************************************************************
 * Name: xtensa_lowputc
 *
 * Description:
 *   Output one byte on the serial console.
 *
 * Parameters:
 *   ch        - Byte to be sent.
 *
 ****************************************************************************/

void xtensa_lowputc(char ch)
{
#ifdef CONSOLE_UART

#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
  struct esp32s3_uart_s *priv = &g_uart0_config;
#elif defined (CONFIG_UART1_SERIAL_CONSOLE)
  struct esp32s3_uart_s *priv = &g_uart1_config;
#endif

  /* Wait until the TX FIFO has space to insert new char */

  while (esp32s3_lowputc_is_tx_fifo_full(priv));

  /* Then send the character */

  esp32s3_lowputc_send_byte(priv, ch);
#endif /* CONSOLE_UART */
}

/****************************************************************************
 * Name: esp32s3_lowsetup
 *
 * Description:
 *   This performs only the basic configuration for UART pins.
 *
 ****************************************************************************/

void esp32s3_lowsetup(void)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG

#ifdef CONFIG_ESP32S3_UART0

  esp32s3_lowputc_config_pins(&g_uart0_config);

#endif

#ifdef CONFIG_ESP32S3_UART1

  esp32s3_lowputc_config_pins(&g_uart1_config);

#endif

#endif /* !CONFIG_SUPPRESS_UART_CONFIG */
}
