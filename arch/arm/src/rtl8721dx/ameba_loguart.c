/***************************************************************************
 * arch/arm/src/rtl8721dx/ameba_loguart.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ameba_irq.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* This driver targets the RTL8721Dx LOG-UART, which NuttX (on
 * the KM4 application core) owns directly.
 *
 * The Realtek SDK's app_start() runs BEFORE NuttX and has already
 * configured the LOG-UART pinmux/clock/baud and printed its boot banner, so
 * the hardware is not re-initialized here.
 *
 * The LOG-UART is physically shared with the NP, but the NP image is built
 * with CONFIG_SHELL=n: the NP neither runs its monitor shell nor claims the
 * LOG-UART RX interrupt.  NuttX therefore routes the LOG-UART interrupt to
 * the AP with LOGUART_INTCoreConfig() and reads RX directly via the ROM
 * helpers, giving the AP sole ownership of the console (no IPC relay).
 *
 * NuttX installs its own vector table and re-points VTOR in
 * up_irqinitialize(), so once NuttX is running it fully owns the NVIC.
 */

/* LOGUART_INTConfig() interrupt-source bit (RX data available). */

#define LOGUART_BIT_ERBI           (1u << 0)

/* LOGUART_INTCoreConfig() per-core routing masks (SDK KM0/KM4 mask bits):
 * select which core's NVIC the LOG-UART interrupt is delivered to.  NP is
 * the network core (km0), AP is the host core (km4 / NuttX).
 */

#define LOGUART_BIT_INTR_MASK_NP   (1u << 0)
#define LOGUART_BIT_INTR_MASK_AP   (1u << 1)

/* SDK ENABLE/DISABLE values. */

#define AMEBA_ENABLE               1
#define AMEBA_DISABLE              0

/* LOG-UART device base (UARTLOG_REG_BASE, from SDK hal_platform.h). */

#define LOGUART_DEV                ((void *)0x4100f000)

/***************************************************************************
 * External ROM/SDK Function Prototypes
 ***************************************************************************/

/* These live in the RTL8721Dx ROM and are resolved at link time from the
 * SDK's ROM symbol script.  Declared here (instead of including the SDK
 * headers) to keep the SDK include tree out of the NuttX build.
 */

extern void LOGUART_PutChar(unsigned char c);
extern unsigned char LOGUART_Readable(void);
extern unsigned char LOGUART_GetChar(int pullmode);
extern void LOGUART_INTConfig(void *dev, unsigned int it,
                              unsigned int state);
extern void LOGUART_INTCoreConfig(void *dev, unsigned int mask,
                                  unsigned int state);

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

static int  loguart_setup(struct uart_dev_s *dev);
static void loguart_shutdown(struct uart_dev_s *dev);
static int  loguart_attach(struct uart_dev_s *dev);
static void loguart_detach(struct uart_dev_s *dev);
static int  loguart_interrupt(int irq, void *context, void *arg);
static int  loguart_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  loguart_receive(struct uart_dev_s *dev, unsigned int *status);
static void loguart_rxint(struct uart_dev_s *dev, bool enable);
static bool loguart_rxavailable(struct uart_dev_s *dev);
static void loguart_send(struct uart_dev_s *dev, int ch);
static void loguart_txint(struct uart_dev_s *dev, bool enable);
static bool loguart_txready(struct uart_dev_s *dev);
static bool loguart_txempty(struct uart_dev_s *dev);

/***************************************************************************
 * Private Data
 ***************************************************************************/

static const struct uart_ops_s g_loguart_ops =
{
  .setup       = loguart_setup,
  .shutdown    = loguart_shutdown,
  .attach      = loguart_attach,
  .detach      = loguart_detach,
  .ioctl       = loguart_ioctl,
  .receive     = loguart_receive,
  .rxint       = loguart_rxint,
  .rxavailable = loguart_rxavailable,
  .send        = loguart_send,
  .txint       = loguart_txint,
  .txready     = loguart_txready,
  .txempty     = loguart_txempty,
};

/* I/O buffers for the LOG-UART console. */

static char g_loguart_rxbuffer[CONFIG_RTL8721DX_LOGUART_RXBUFSIZE];
static char g_loguart_txbuffer[CONFIG_RTL8721DX_LOGUART_TXBUFSIZE];

/* The LOG-UART port device. */

static struct uart_dev_s g_loguart_port =
{
  .isconsole = true,
  .recv      =
  {
    .size    = CONFIG_RTL8721DX_LOGUART_RXBUFSIZE,
    .buffer  = g_loguart_rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_RTL8721DX_LOGUART_TXBUFSIZE,
    .buffer  = g_loguart_txbuffer,
  },
  .ops       = &g_loguart_ops,
};

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Name: loguart_setup
 ***************************************************************************/

static int loguart_setup(struct uart_dev_s *dev)
{
  UNUSED(dev);
  return OK;
}

/***************************************************************************
 * Name: loguart_shutdown
 ***************************************************************************/

static void loguart_shutdown(struct uart_dev_s *dev)
{
  loguart_rxint(dev, false);
}

/***************************************************************************
 * Name: loguart_attach
 *
 * Description:
 *   Route the LOG-UART interrupt to KM4 and attach the handler.  The
 *   data-available (RX) source itself is enabled later via rxint().
 *
 ***************************************************************************/

static int loguart_attach(struct uart_dev_s *dev)
{
  int ret;

  ret = irq_attach(RTL8721DX_IRQ_UART_LOG, loguart_interrupt, dev);
  if (ret == OK)
    {
      /* Claim the LOG-UART interrupt for the AP (the NP is built
       * CONFIG_SHELL=n and does not claim it) and unmask it in the NVIC.
       */

      LOGUART_INTCoreConfig(LOGUART_DEV, LOGUART_BIT_INTR_MASK_NP,
                            AMEBA_DISABLE);
      LOGUART_INTCoreConfig(LOGUART_DEV, LOGUART_BIT_INTR_MASK_AP,
                            AMEBA_ENABLE);
      up_enable_irq(RTL8721DX_IRQ_UART_LOG);
    }

  return ret;
}

/***************************************************************************
 * Name: loguart_detach
 ***************************************************************************/

static void loguart_detach(struct uart_dev_s *dev)
{
  UNUSED(dev);
  up_disable_irq(RTL8721DX_IRQ_UART_LOG);
  irq_detach(RTL8721DX_IRQ_UART_LOG);
}

/***************************************************************************
 * Name: loguart_interrupt
 *
 * Description:
 *   Common LOG-UART interrupt handler.  Drains the RX FIFO into the NuttX
 *   receive buffer.
 *
 ***************************************************************************/

static int loguart_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;

  UNUSED(irq);
  UNUSED(context);

  if (LOGUART_Readable())
    {
      uart_recvchars(dev);
    }

  return OK;
}

/***************************************************************************
 * Name: loguart_ioctl
 ***************************************************************************/

static int loguart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  UNUSED(filep);
  UNUSED(cmd);
  UNUSED(arg);
  return -ENOTTY;
}

/***************************************************************************
 * Name: loguart_receive
 ***************************************************************************/

static int loguart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  UNUSED(dev);

  *status = 0;

  /* Read one byte in polled (non-pull) mode. */

  return (int)LOGUART_GetChar(false);
}

/***************************************************************************
 * Name: loguart_rxint
 ***************************************************************************/

static void loguart_rxint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  UNUSED(dev);

  flags = enter_critical_section();

  if (enable)
    {
      LOGUART_INTConfig(LOGUART_DEV, LOGUART_BIT_ERBI, AMEBA_ENABLE);
    }
  else
    {
      LOGUART_INTConfig(LOGUART_DEV, LOGUART_BIT_ERBI, AMEBA_DISABLE);
    }

  leave_critical_section(flags);
}

/***************************************************************************
 * Name: loguart_rxavailable
 ***************************************************************************/

static bool loguart_rxavailable(struct uart_dev_s *dev)
{
  UNUSED(dev);
  return LOGUART_Readable() != 0;
}

/***************************************************************************
 * Name: loguart_send
 ***************************************************************************/

static void loguart_send(struct uart_dev_s *dev, int ch)
{
  UNUSED(dev);
  LOGUART_PutChar((unsigned char)ch);
}

/***************************************************************************
 * Name: loguart_txint
 *
 * Description:
 *   TX is synchronous (LOGUART_PutChar blocks for FIFO space), so there is
 *   no hardware TX interrupt; pump the transmit buffer when asked to
 *   enable.
 *
 ***************************************************************************/

static void loguart_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  if (enable)
    {
      flags = enter_critical_section();
      uart_xmitchars(dev);
      leave_critical_section(flags);
    }
}

/***************************************************************************
 * Name: loguart_txready
 ***************************************************************************/

static bool loguart_txready(struct uart_dev_s *dev)
{
  UNUSED(dev);
  return true;
}

/***************************************************************************
 * Name: loguart_txempty
 ***************************************************************************/

static bool loguart_txempty(struct uart_dev_s *dev)
{
  UNUSED(dev);
  return true;
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm_lowputc
 ***************************************************************************/

void arm_lowputc(char ch)
{
  if (ch == '\n')
    {
      LOGUART_PutChar((unsigned char)'\r');
    }

  LOGUART_PutChar((unsigned char)ch);
}

/***************************************************************************
 * Name: arm_earlyserialinit
 ***************************************************************************/

void arm_earlyserialinit(void)
{
}

/***************************************************************************
 * Name: arm_serialinit
 ***************************************************************************/

void arm_serialinit(void)
{
  uart_register("/dev/console", &g_loguart_port);
  uart_register("/dev/ttyS0", &g_loguart_port);
}

/***************************************************************************
 * Name: up_putc
 ***************************************************************************/

void up_putc(int ch)
{
  arm_lowputc((char)ch);
}
