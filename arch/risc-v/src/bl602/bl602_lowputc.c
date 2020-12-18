/****************************************************************************
 * boards/risc-v/bl602/evb/src/bl602_lowputc.c
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
  struct gpio_cfg_s       cfg;
  enum glb_uart_sig_fun_e tx_sigfun;
  enum glb_uart_sig_fun_e rx_sigfun;

  cfg.drive    = 1;
  cfg.smt_ctrl = 1;
  cfg.gpio_fun = 7;

  cfg.gpio_pin  = rx_pin;
  cfg.gpio_mode = GPIO_MODE_AF;
  cfg.pull_type = GPIO_PULL_UP;
  gpio_init(&cfg);

  cfg.gpio_pin  = tx_pin;
  cfg.gpio_mode = GPIO_MODE_AF;
  cfg.pull_type = GPIO_PULL_UP;
  gpio_init(&cfg);

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

  glb_uart_fun_sel(tx_pin % 8, tx_sigfun);
  glb_uart_fun_sel(rx_pin % 8, rx_sigfun);
}

static void bl602_enable_uart_clk(uint8_t                  enable,
                                  enum hbn_uart_clk_type_e clk_sel,
                                  uint8_t                  div)
{
  uint32_t tmp_val;

  /* disable UART clock first */

  tmp_val = BL_RD_REG(GLB_BASE, GLB_CLK_CFG2);
  tmp_val = BL_CLR_REG_BIT(tmp_val, GLB_UART_CLK_EN);
  BL_WR_REG(GLB_BASE, GLB_CLK_CFG2, tmp_val);

  /* Set div */

  tmp_val = BL_RD_REG(GLB_BASE, GLB_CLK_CFG2);
  tmp_val = BL_SET_REG_BITS_VAL(tmp_val, GLB_UART_CLK_DIV, div);
  BL_WR_REG(GLB_BASE, GLB_CLK_CFG2, tmp_val);

  /* Select clock source for uart */

  hbn_set_uart_clk_sel(clk_sel);

  /* Set enable or disable */

  tmp_val = BL_RD_REG(GLB_BASE, GLB_CLK_CFG2);
  if (enable)
    {
      tmp_val = BL_SET_REG_BIT(tmp_val, GLB_UART_CLK_EN);
    }
  else
    {
      tmp_val = BL_CLR_REG_BIT(tmp_val, GLB_UART_CLK_EN);
    }

  BL_WR_REG(GLB_BASE, GLB_CLK_CFG2, tmp_val);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bl602_uart_configure(uint32_t                    uartbase,
                          const struct uart_config_s *config)
{
  uint32_t             tmp_val;
  uint32_t             div        = 0;
  uint32_t             fraction   = 0;
  uint32_t             tmp_tx_cfg = 0;
  uint32_t             tmp_rx_cfg = 0;
  enum uart_databits_e data_bits;
  enum uart_stopbits_e stop_bits;

  bl602_enable_uart_clk(1, HBN_UART_CLK_160M, 3);

  uart_gpio_init(config->idx,
                 config->tx_pin,
                 config->rx_pin,
                 config->cts_pin,
                 config->rts_pin);

  /* Disable all interrupt */

  tmp_val = BL_RD_REG(uartbase, UART_INT_MASK);
  tmp_val |= 0xff;
  BL_WR_REG(uartbase, UART_INT_MASK, tmp_val);

  /* Disable uart before config */

  tmp_val = BL_RD_REG(uartbase, UART_UTX_CONFIG);
  BL_WR_REG(
    uartbase, UART_UTX_CONFIG, BL_CLR_REG_BIT(tmp_val, UART_CR_UTX_EN));
  tmp_val = BL_RD_REG(uartbase, UART_URX_CONFIG);
  BL_WR_REG(
    uartbase, UART_URX_CONFIG, BL_CLR_REG_BIT(tmp_val, UART_CR_URX_EN));

  /* cal the baud rate divisor */

  fraction = (_BL602_UART_CLOCK / (3 + 1)) * 10 / config->baud % 10;
  div      = (_BL602_UART_CLOCK / (3 + 1)) / config->baud;
  if (fraction >= 5)
    {
      ++div;
    }

  /* set the baud rate register value */

  BL_WR_REG(
    uartbase, UART_BIT_PRD, ((div - 1) << 0x10) | ((div - 1) & 0xffff));

  /* configure parity type */

  tmp_tx_cfg = BL_RD_REG(uartbase, UART_UTX_CONFIG);
  tmp_rx_cfg = BL_RD_REG(uartbase, UART_URX_CONFIG);

  switch (config->parity)
    {
    case UART_PARITY_NONE:
      tmp_tx_cfg = BL_CLR_REG_BIT(tmp_tx_cfg, UART_CR_UTX_PRT_EN);
      tmp_rx_cfg = BL_CLR_REG_BIT(tmp_rx_cfg, UART_CR_URX_PRT_EN);
      break;
    case UART_PARITY_ODD:
      tmp_tx_cfg = BL_SET_REG_BIT(tmp_tx_cfg, UART_CR_UTX_PRT_EN);
      tmp_tx_cfg = BL_SET_REG_BIT(tmp_tx_cfg, UART_CR_UTX_PRT_SEL);
      tmp_rx_cfg = BL_SET_REG_BIT(tmp_rx_cfg, UART_CR_URX_PRT_EN);
      tmp_rx_cfg = BL_SET_REG_BIT(tmp_rx_cfg, UART_CR_URX_PRT_SEL);
      break;
    case UART_PARITY_EVEN:
      tmp_tx_cfg = BL_SET_REG_BIT(tmp_tx_cfg, UART_CR_UTX_PRT_EN);
      tmp_tx_cfg = BL_CLR_REG_BIT(tmp_tx_cfg, UART_CR_UTX_PRT_SEL);
      tmp_rx_cfg = BL_SET_REG_BIT(tmp_rx_cfg, UART_CR_URX_PRT_EN);
      tmp_rx_cfg = BL_CLR_REG_BIT(tmp_rx_cfg, UART_CR_URX_PRT_SEL);
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

  tmp_tx_cfg =
    BL_SET_REG_BITS_VAL(tmp_tx_cfg, UART_CR_UTX_BIT_CNT_D, (data_bits + 4));
  tmp_rx_cfg =
    BL_SET_REG_BITS_VAL(tmp_rx_cfg, UART_CR_URX_BIT_CNT_D, (data_bits + 4));

  /* Configure tx stop bits */

  tmp_tx_cfg =
    BL_SET_REG_BITS_VAL(tmp_tx_cfg, UART_CR_UTX_BIT_CNT_P, (stop_bits + 1));

  /* Configure tx cts flow control function */

  if (config->oflow_ctl)
    {
      tmp_tx_cfg = BL_SET_REG_BIT(tmp_tx_cfg, UART_CR_UTX_CTS_EN);
    }
  else
    {
      tmp_tx_cfg = BL_CLR_REG_BIT(tmp_tx_cfg, UART_CR_UTX_CTS_EN);
    }

  /* Disable rx input de-glitch function */

  tmp_rx_cfg = BL_CLR_REG_BIT(tmp_rx_cfg, UART_CR_URX_DEG_EN);

  if (config->iflow_ctl)
    {
      tmp_tx_cfg = BL_SET_REG_BIT(tmp_tx_cfg, UART_CR_URX_RTS_SW_MODE);
    }
  else
    {
      tmp_rx_cfg = BL_CLR_REG_BIT(tmp_rx_cfg, UART_CR_URX_RTS_SW_MODE);
    }

  /* Write back */

  BL_WR_REG(uartbase, UART_UTX_CONFIG, tmp_tx_cfg);
  BL_WR_REG(uartbase, UART_URX_CONFIG, tmp_rx_cfg);

  /* Configure LSB-first */

  tmp_tx_cfg = BL_RD_REG(uartbase, UART_DATA_CONFIG);
  tmp_tx_cfg = BL_CLR_REG_BIT(tmp_tx_cfg, UART_CR_UART_BIT_INV);
  BL_WR_REG(uartbase, UART_DATA_CONFIG, tmp_tx_cfg);

  /* Enable tx free run mode */

  tmp_val = BL_RD_REG(uartbase, UART_UTX_CONFIG);
  BL_WR_REG(
    uartbase, UART_UTX_CONFIG, BL_SET_REG_BIT(tmp_val, UART_CR_UTX_FRM_EN));

  /* Deal with uart fifo configure register */

  tmp_val = BL_RD_REG(uartbase, UART_FIFO_CONFIG_1);

  /* Configure dma tx fifo threshold */

  tmp_val = BL_SET_REG_BITS_VAL(tmp_val, UART_TX_FIFO_TH, 0x10 - 1);

  /* Configure dma rx fifo threshold */

  tmp_val = BL_SET_REG_BITS_VAL(tmp_val, UART_RX_FIFO_TH, 0x10 - 1);
  BL_WR_REG(uartbase, UART_FIFO_CONFIG_1, tmp_val);

  /* Enable UART tx rx unit */

  tmp_val = BL_RD_REG(uartbase, UART_UTX_CONFIG);
  BL_WR_REG(
    uartbase, UART_UTX_CONFIG, BL_SET_REG_BIT(tmp_val, UART_CR_UTX_EN));
  tmp_val = BL_RD_REG(uartbase, UART_URX_CONFIG);
  BL_WR_REG(
    uartbase, UART_URX_CONFIG, BL_SET_REG_BIT(tmp_val, UART_CR_URX_EN));
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
    BL_GET_REG_BITS_VAL(BL_RD_REG(BL602_CONSOLE_BASE, UART_FIFO_CONFIG_1),
                        UART_TX_FIFO_CNT) == 0)
    ;

  BL_WR_BYTE(BL602_CONSOLE_BASE + UART_FIFO_WDATA_OFFSET, ch);
#endif /* HAVE_CONSOLE */
}

void bl602_lowsetup(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Configure the console UART (if any) */

  bl602_uart_configure(BL602_CONSOLE_BASE, &g_bl602_console_config);

#endif /* HAVE_SERIAL_CONSOLE */
}

