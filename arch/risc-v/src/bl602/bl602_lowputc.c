/****************************************************************************
 * arch/risc-v/src/bl602/bl602_lowputc.c
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
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "hardware/bl602_gpio.h"
#include "hardware/bl602_glb.h"
#include "hardware/bl602_hbn.h"
#include "hardware/bl602_uart.h"

#include "bl602_lowputc.h"
#include "riscv_arch.h"
#include "riscv_internal.h"

#include "bl602_config.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define BL602_CONSOLE_IDX    0
#define BL602_CONSOLE_BASE   UART0_BASE
#define BL602_CONSOLE_BAUD   CONFIG_UART0_BAUD
#define BL602_CONSOLE_BITS   CONFIG_UART0_BITS
#define BL602_CONSOLE_PARITY CONFIG_UART0_PARITY
#define BL602_CONSOLE_2STOP  CONFIG_UART0_2STOP
#ifdef UART0_IFLOWCONTROL
#define BL602_CONSOLE_IFLOWCTL UART0_IFLOWCONTROL
#else
#define BL602_CONSOLE_IFLOWCTL 0
#endif
#ifdef UART0_OFLOWCONTROL
#define BL602_CONSOLE_OFLOWCTL UART0_OFLOWCONTROL
#else
#define BL602_CONSOLE_OFLOWCTL 0
#endif
#define BL602_CONSOLE_TX_PIN  CONFIG_BL602_UART0_TX_PIN
#define BL602_CONSOLE_RX_PIN  CONFIG_BL602_UART0_RX_PIN
#define BL602_CONSOLE_RTS_PIN CONFIG_BL602_UART0_RTS_PIN
#define BL602_CONSOLE_CTS_PIN CONFIG_BL602_UART0_CTS_PIN
#define HAVE_UART
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define BL602_CONSOLE_IDX    1
#define BL602_CONSOLE_BASE   UART1_BASE
#define BL602_CONSOLE_BAUD   CONFIG_UART1_BAUD
#define BL602_CONSOLE_BITS   CONFIG_UART1_BITS
#define BL602_CONSOLE_PARITY CONFIG_UART1_PARITY
#define BL602_CONSOLE_2STOP  CONFIG_UART1_2STOP
#ifdef UART1_IFLOWCONTROL
#define BL602_CONSOLE_IFLOWCTL UART1_IFLOWCONTROL
#else
#define BL602_CONSOLE_IFLOWCTL 0
#endif
#ifdef UART1_OFLOWCONTROL
#define BL602_CONSOLE_OFLOWCTL UART1_OFLOWCONTROL
#else
#define BL602_CONSOLE_OFLOWCTL 0
#endif
#define BL602_CONSOLE_TX_PIN  CONFIG_BL602_UART1_TX_PIN
#define BL602_CONSOLE_RX_PIN  CONFIG_BL602_UART1_RX_PIN
#define BL602_CONSOLE_RTS_PIN CONFIG_BL602_UART1_RTS_PIN
#define BL602_CONSOLE_CTS_PIN CONFIG_BL602_UART1_CTS_PIN
#define HAVE_UART
#endif
#endif /* HAVE_CONSOLE */

#define _BL602_UART_CLOCK (160 * 1000 * 1000UL) /* UART clock */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
static const struct uart_config_s g_bl602_console_config =
{
  .idx       = BL602_CONSOLE_IDX,
  .baud      = BL602_CONSOLE_BAUD,
  .parity    = BL602_CONSOLE_PARITY,
  .data_bits = BL602_CONSOLE_BITS,
  .stop_bits = BL602_CONSOLE_2STOP,
#ifdef BL602_CONSOLE_IFLOWCTL
  .iflow_ctl = BL602_CONSOLE_IFLOWCTL,
#else
  .iflow_ctl = 0,
#endif

#ifdef BL602_CONSOLE_OFLOWCTL
  .oflow_ctl = BL602_CONSOLE_OFLOWCTL,
#else
  .oflow_ctl = 0,
#endif
  .tx_pin  = BL602_CONSOLE_TX_PIN,
  .rx_pin  = BL602_CONSOLE_RX_PIN,
  .rts_pin = BL602_CONSOLE_RTS_PIN,
  .cts_pin = BL602_CONSOLE_CTS_PIN,
};
#endif /* HAVE_SERIAL_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void uart_gpio_init(uint8_t id,
                           uint8_t tx_pin,
                           uint8_t rx_pin,
                           uint8_t cts_pin,
                           uint8_t rts_pin)
{
  struct gpio_cfg_s cfg;
  int               tx_sigfun;
  int               rx_sigfun;

  cfg.drive    = 1;
  cfg.smt_ctrl = 1;
  cfg.gpio_fun = 7;

  cfg.gpio_pin  = rx_pin;
  cfg.gpio_mode = GPIO_MODE_AF;
  cfg.pull_type = GPIO_PULL_UP;
  bl602_gpio_init(&cfg);

  cfg.gpio_pin  = tx_pin;
  cfg.gpio_mode = GPIO_MODE_AF;
  cfg.pull_type = GPIO_PULL_UP;
  bl602_gpio_init(&cfg);

  /* select uart gpio function */

  if (id == 0)
    {
      tx_sigfun = GLB_UART_SIG_FUN_UART0_TXD;
      rx_sigfun = GLB_UART_SIG_FUN_UART0_RXD;
    }
  else
    {
      tx_sigfun = GLB_UART_SIG_FUN_UART1_TXD;
      rx_sigfun = GLB_UART_SIG_FUN_UART1_RXD;
    }

  bl602_glb_uart_fun_sel(tx_pin % 8, tx_sigfun);
  bl602_glb_uart_fun_sel(rx_pin % 8, rx_sigfun);
}

static void bl602_enable_uart_clk(uint8_t enable, int clk_sel, uint8_t div)
{
  uint32_t tmp_val;

  /* disable UART clock first */

  bl602_up_serialmodify(GLB_BASE, GLB_CLK_CFG2_OFFSET, (1 << 4), 0);

  /* Set div */

  bl602_up_serialmodify(GLB_BASE, GLB_CLK_CFG2_OFFSET, 0x7, div);

  /* Select clock source for uart */

  bl602_hbn_set_uart_clk_sel(clk_sel);

  /* Set enable or disable */

  tmp_val = bl602_up_serialin(GLB_BASE, GLB_CLK_CFG2_OFFSET);
  if (enable)
    {
      tmp_val |= (1 << 4);
    }
  else
    {
      tmp_val &= ~(1 << 4);
    }

  bl602_up_serialout(GLB_BASE, GLB_CLK_CFG2_OFFSET, tmp_val);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t bl602_up_serialin(uint32_t reg_base, int offset)
{
  return getreg32(reg_base + offset);
}

void bl602_up_serialout(uint32_t reg_base, int offset, uint32_t value)
{
  putreg32(value, reg_base + offset);
}

void bl602_up_serialmodify(uint32_t reg_base,
                           int      offset,
                           uint32_t clearbits,
                           uint32_t setbits)
{
  modifyreg32(reg_base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: bl602_data_setbits
 ****************************************************************************/

static uint32_t bl602_data_setbits(uint32_t data,
                                   uint32_t start,
                                   uint32_t len,
                                   uint32_t value)
{
  return (((data) & ~((~((~0) << (len))) << (start))) |
          (((value) & ((~((~0) << (len))))) << (start)));
}

void bl602_uart_configure(uint32_t                    uartbase,
                          const struct uart_config_s *config)
{
  uint32_t tmp_val;
  uint32_t div        = 0;
  uint32_t fraction   = 0;
  uint32_t tmp_tx_cfg = 0;
  uint32_t tmp_rx_cfg = 0;
  int      data_bits;
  int      stop_bits;

  bl602_enable_uart_clk(1, HBN_UART_CLK_160M, 3);

  uart_gpio_init(config->idx,
                 config->tx_pin,
                 config->rx_pin,
                 config->cts_pin,
                 config->rts_pin);

  /* Disable all interrupt */

  bl602_up_serialmodify(uartbase, UART_INT_MASK_OFFSET, 0, 0xff);

  /* Disable uart before config */

  bl602_up_serialmodify(uartbase, UART_UTX_CONFIG_OFFSET, 1, 0);
  bl602_up_serialmodify(uartbase, UART_URX_CONFIG_OFFSET, 1, 0);

  /* cal the baud rate divisor */

  fraction = (_BL602_UART_CLOCK / (3 + 1)) * 10 / config->baud % 10;
  div      = (_BL602_UART_CLOCK / (3 + 1)) / config->baud;
  if (fraction >= 5)
    {
      ++div;
    }

  /* set the baud rate register value */

  bl602_up_serialout(uartbase,
                     UART_BIT_PRD_OFFSET,
                     ((div - 1) << 0x10) | ((div - 1) & 0xffff));

  /* configure parity type */

  tmp_tx_cfg = bl602_up_serialin(uartbase, UART_UTX_CONFIG_OFFSET);
  tmp_rx_cfg = bl602_up_serialin(uartbase, UART_URX_CONFIG_OFFSET);

  switch (config->parity)
    {
    case UART_PARITY_NONE:
      tmp_tx_cfg &= ~(1 << 4);
      tmp_rx_cfg &= ~(1 << 4);
      break;
    case UART_PARITY_ODD:
      tmp_tx_cfg |= 1 << 4;
      tmp_tx_cfg |= 1 << 5;
      tmp_rx_cfg |= 1 << 4;
      tmp_rx_cfg |= 1 << 5;
      break;
    case UART_PARITY_EVEN:
      tmp_tx_cfg |= 1 << 4;
      tmp_tx_cfg &= ~(1 << 5);
      tmp_rx_cfg |= 1 << 4;
      tmp_rx_cfg &= ~(1 << 5);
      break;
    default:
      break;
    }

  if (config->data_bits == 5)
    {
      data_bits = UART_DATABITS_5;
    }
  else if (config->data_bits == 6)
    {
      data_bits = UART_DATABITS_6;
    }
  else if (config->data_bits == 7)
    {
      data_bits = UART_DATABITS_7;
    }
  else
    {
      data_bits = UART_DATABITS_8;
    }

  if (config->stop_bits == 1)
    {
      stop_bits = UART_STOPBITS_2;
    }
  else
    {
      stop_bits = UART_STOPBITS_1;
    }

  /* Configure data bits */

  tmp_tx_cfg = bl602_data_setbits(tmp_tx_cfg, 8, 3, (data_bits + 4));
  tmp_rx_cfg = bl602_data_setbits(tmp_tx_cfg, 8, 3, (data_bits + 4));

  /* Configure tx stop bits */

  tmp_tx_cfg = bl602_data_setbits(tmp_tx_cfg, 12, 2, (stop_bits + 1));

  /* Configure tx cts flow control function */

  if (config->oflow_ctl)
    {
      tmp_tx_cfg |= 1 << 1;
    }
  else
    {
      tmp_tx_cfg &= ~(1 << 1);
    }

  /* Disable rx input de-glitch function */

  tmp_rx_cfg &= ~(1 << 11);

  if (config->iflow_ctl)
    {
      tmp_rx_cfg |= 1 << 1;
    }
  else
    {
      tmp_rx_cfg &= ~(1 << 1);
    }

  /* Write back */

  bl602_up_serialout(uartbase, UART_UTX_CONFIG_OFFSET, tmp_tx_cfg);
  bl602_up_serialout(uartbase, UART_URX_CONFIG_OFFSET, tmp_rx_cfg);

  /* Configure LSB-first */

  bl602_up_serialmodify(uartbase, UART_DATA_CONFIG_OFFSET, 1, 0);

  /* Enable tx free run mode */

  bl602_up_serialmodify(uartbase, UART_UTX_CONFIG_OFFSET, 0, 1 << 2);

  /* Deal with uart fifo configure register */

  tmp_val = bl602_up_serialin(uartbase, UART_FIFO_CONFIG_1_OFFSET);
  tmp_val = bl602_data_setbits(tmp_val, UART_TX_FIFO_TH_POS, 5, 0x10 - 1);
  tmp_val = bl602_data_setbits(tmp_val, UART_RX_FIFO_TH_POS, 5, 0x10 - 1);
  bl602_up_serialout(uartbase, UART_FIFO_CONFIG_1_OFFSET, tmp_val);

  /* Enable UART tx rx unit */

  bl602_up_serialmodify(uartbase, UART_UTX_CONFIG_OFFSET, 0, 1);
  bl602_up_serialmodify(uartbase, UART_URX_CONFIG_OFFSET, 0, 1);
}

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait for FIFO */

  while (
    ((bl602_up_serialin(BL602_CONSOLE_BASE, UART_FIFO_CONFIG_1_OFFSET)) >>
     (UART_TX_FIFO_CNT_POS)) &
    (~((~0) << (6))))
    ;

  bl602_up_serialout(BL602_CONSOLE_BASE, UART_FIFO_WDATA_OFFSET, ch);
#endif /* HAVE_CONSOLE */
}

void bl602_lowsetup(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Configure the console UART (if any) */

  bl602_uart_configure(BL602_CONSOLE_BASE, &g_bl602_console_config);

#endif /* HAVE_SERIAL_CONSOLE */
}
