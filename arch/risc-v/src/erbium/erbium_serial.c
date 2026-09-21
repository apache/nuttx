/****************************************************************************
 * arch/risc-v/src/erbium/erbium_serial.c
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

#include <sys/types.h>

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>

#include <arch/irq.h>

#include "chip.h"
#include "hardware/erbium_memorymap.h"
#include "hardware/erbium_sysreg.h"
#include "hardware/erbium_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERBIUM_UART_BAUD_DIV \
  (CONFIG_ERBIUM_UART_CLOCK / (16 * CONFIG_UART0_BAUD))

#if CONFIG_UART0_BITS != 8 || CONFIG_UART0_PARITY != 0 || \
    CONFIG_UART0_2STOP != 0
#  error "The Erbium UART driver supports only 8N1"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct erbium_uart_s
{
  uintptr_t base;
  uint8_t irq;
  uint32_t ien;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  erbium_setup(struct uart_dev_s *dev);
static void erbium_shutdown(struct uart_dev_s *dev);
static int  erbium_attach(struct uart_dev_s *dev);
static void erbium_detach(struct uart_dev_s *dev);
static int  erbium_interrupt(int irq, void *context, void *arg);
static int  erbium_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  erbium_receive(struct uart_dev_s *dev, unsigned int *status);
static void erbium_rxint(struct uart_dev_s *dev, bool enable);
static bool erbium_rxavailable(struct uart_dev_s *dev);
static void erbium_send(struct uart_dev_s *dev, int ch);
static void erbium_txint(struct uart_dev_s *dev, bool enable);
static bool erbium_txready(struct uart_dev_s *dev);
static bool erbium_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct erbium_uart_s g_uart0priv =
{
  .base = ERBIUM_UART0_BASE,
  .irq = ERBIUM_IRQ_UART0,
};

static const struct uart_ops_s g_uart_ops =
{
  .setup        = erbium_setup,
  .shutdown     = erbium_shutdown,
  .attach       = erbium_attach,
  .detach       = erbium_detach,
  .ioctl        = erbium_ioctl,
  .receive      = erbium_receive,
  .rxint        = erbium_rxint,
  .rxavailable  = erbium_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send         = erbium_send,
  .txint        = erbium_txint,
  .txready      = erbium_txready,
  .txempty      = erbium_txempty,
};

static uart_dev_t g_uart0port =
{
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  .isconsole = true,
#endif
  .recv =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart0priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: erbium_serialin
 *
 * Description:
 *   Read a 32-bit UART register.
 *
 * Input Parameters:
 *   priv - UART register base and cached interrupt mask.
 *   offset - Register offset from the UART base address.
 *
 * Returned Value:
 *   The register value.
 *
 ****************************************************************************/

static uint32_t erbium_serialin(struct erbium_uart_s *priv, uintptr_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: erbium_serialout
 *
 * Description:
 *   Write a 32-bit UART register.
 *
 * Input Parameters:
 *   priv - UART register base and cached interrupt mask.
 *   offset - Register offset from the UART base address.
 *   value - Value to write.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void erbium_serialout(struct erbium_uart_s *priv, uintptr_t offset,
                            uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: erbium_setuartint
 *
 * Description:
 *   Update the cached UART interrupt mask and the hardware enable register.
 *
 * Input Parameters:
 *   priv - UART register base and cached interrupt mask.
 *   ien - UART interrupt sources to enable.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   The caller must serialize access to the cached interrupt mask.
 *
 ****************************************************************************/

static void erbium_setuartint(struct erbium_uart_s *priv, uint32_t ien)
{
  priv->ien = ien;
  erbium_serialout(priv, ERBIUM_UART_IEN_OFFSET, ien);
}

#ifndef CONFIG_SUPPRESS_UART_CONFIG

/****************************************************************************
 * Name: erbium_enable_uart
 *
 * Description:
 *   Enable the UART peripheral through the system configuration register.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void erbium_enable_uart(void)
{
  modifyreg32(ERBIUM_SYSREG_SYSTEM_CONFIG, 0,
              ERBIUM_SYSREG_UART_ENABLE);
}
#endif

/****************************************************************************
 * Name: erbium_disableuartint
 *
 * Description:
 *   Disable all UART interrupt sources, optionally saving the old mask.
 *
 * Input Parameters:
 *   priv - UART register base and cached interrupt mask.
 *   ien - Optional location to save the previous interrupt mask.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void erbium_disableuartint(struct erbium_uart_s *priv, uint32_t *ien)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (ien != NULL)
    {
      *ien = priv->ien;
    }

  erbium_setuartint(priv, 0);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: erbium_restoreuartint
 *
 * Description:
 *   Restore a previously saved UART interrupt mask.
 *
 * Input Parameters:
 *   priv - UART register base and cached interrupt mask.
 *   ien - Saved UART interrupt mask.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void erbium_restoreuartint(struct erbium_uart_s *priv, uint32_t ien)
{
  irqstate_t flags;

  flags = enter_critical_section();
  erbium_setuartint(priv, ien);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: erbium_putc
 *
 * Description:
 *   Wait for space in the transmit FIFO and write one character.
 *
 * Input Parameters:
 *   priv - UART register base and cached interrupt mask.
 *   ch - Character to transmit; only the low eight bits are used.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   The caller must serialize access to the transmit FIFO.
 *
 ****************************************************************************/

static void erbium_putc(struct erbium_uart_s *priv, int ch)
{
  while ((erbium_serialin(priv, ERBIUM_UART_STATUS_OFFSET) &
          ERBIUM_UART_STATUS_TX_FULL) != 0)
    {
    }

  erbium_serialout(priv, ERBIUM_UART_TX_OFFSET, (uint32_t)ch & 0xffu);
}

/****************************************************************************
 * Name: erbium_setup
 *
 * Description:
 *   Configure the UART for 8N1 operation and apply its interrupt mask.
 *   Preserve the existing line configuration when UART setup is suppressed.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int erbium_setup(struct uart_dev_s *dev)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  erbium_enable_uart();
  erbium_serialout(priv, ERBIUM_UART_BAUD_OFFSET, ERBIUM_UART_BAUD_DIV);
  erbium_serialout(priv, ERBIUM_UART_CONTROL_OFFSET,
                  ERBIUM_UART_CONTROL_8N1);
  erbium_serialout(priv, ERBIUM_UART_RX_THRESHOLD_OFFSET, 0);
#endif
  erbium_setuartint(priv, priv->ien);

  return OK;
}

/****************************************************************************
 * Name: erbium_shutdown
 *
 * Description:
 *   Disable UART interrupts when the serial device is closed.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void erbium_shutdown(struct uart_dev_s *dev)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;

  erbium_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: erbium_attach
 *
 * Description:
 *   Attach the UART interrupt handler and enable its PLIC source.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *
 * Returned Value:
 *   OK on success; a negated errno value from irq_attach() on failure.
 *
 ****************************************************************************/

static int erbium_attach(struct uart_dev_s *dev)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;
  int ret;

  ret = irq_attach(priv->irq, erbium_interrupt, dev);

  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: erbium_detach
 *
 * Description:
 *   Disable the UART PLIC source and detach its interrupt handler.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void erbium_detach(struct uart_dev_s *dev)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: erbium_interrupt
 *
 * Description:
 *   Service enabled receive and transmit events through the serial upper
 *   half. Limit the number of passes so a persistent event cannot trap the
 *   CPU in this handler indefinitely.
 *
 * Input Parameters:
 *   irq - NuttX interrupt number.
 *   context - Saved interrupt context.
 *   arg - Serial device supplied when the handler was attached.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int erbium_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct erbium_uart_s *priv;
  uint32_t pending;
  int passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct erbium_uart_s *)dev->priv;

  for (passes = 0; passes < 256; passes++)
    {
      pending = erbium_serialin(priv, ERBIUM_UART_STATUS_OFFSET) & priv->ien;

      if (pending == 0)
        {
          break;
        }

      if ((pending & ERBIUM_UART_IEN_RX_NOT_EMPTY) != 0)
        {
          uart_recvchars(dev);
        }

      if ((pending & ERBIUM_UART_IEN_TX_EMPTY) != 0)
        {
          uart_xmitchars(dev);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: erbium_ioctl
 *
 * Description:
 *   Reject unsupported device-specific control requests.
 *
 * Input Parameters:
 *   filep - Open serial device file.
 *   cmd - Requested control operation.
 *   arg - Operation-specific argument.
 *
 * Returned Value:
 *   -ENOTTY; this driver implements no device-specific ioctl commands.
 *
 ****************************************************************************/

static int erbium_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: erbium_receive
 *
 * Description:
 *   Read one character from the receive FIFO and optionally report status.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *   status - Optional location to save the UART status register.
 *
 * Returned Value:
 *   The received byte, in the range 0 through 255.
 *
 * Assumptions/Limitations:
 *   The serial upper half has established that receive data is available.
 *
 ****************************************************************************/

static int erbium_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;

  if (status != NULL)
    {
      *status = erbium_serialin(priv, ERBIUM_UART_STATUS_OFFSET);
    }

  return (int)(erbium_serialin(priv, ERBIUM_UART_RX_OFFSET) & 0xffu);
}

/****************************************************************************
 * Name: erbium_rxint
 *
 * Description:
 *   Enable or disable receive interrupts. Honor suppression of serial
 *   interrupts during bring-up.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *   enable - True to enable reception interrupts; false to disable them.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void erbium_rxint(struct uart_dev_s *dev, bool enable)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      erbium_setuartint(priv, priv->ien | ERBIUM_UART_IEN_RX_NOT_EMPTY);
#endif
    }
  else
    {
      erbium_setuartint(priv, priv->ien & ~ERBIUM_UART_IEN_RX_NOT_EMPTY);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: erbium_rxavailable
 *
 * Description:
 *   Check whether the UART receive FIFO contains data.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *
 * Returned Value:
 *   True if at least one character is available; otherwise false.
 *
 ****************************************************************************/

static bool erbium_rxavailable(struct uart_dev_s *dev)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;

  return (erbium_serialin(priv, ERBIUM_UART_STATUS_OFFSET) &
          ERBIUM_UART_STATUS_RX_NOT_EMPTY) != 0;
}

/****************************************************************************
 * Name: erbium_send
 *
 * Description:
 *   Transmit one character from the serial upper half.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *   ch - Character to transmit.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void erbium_send(struct uart_dev_s *dev, int ch)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;

  erbium_putc(priv, ch);
}

/****************************************************************************
 * Name: erbium_txint
 *
 * Description:
 *   Enable or disable transmit interrupts. Prime the transmit FIFO when
 *   enabling interrupts, unless serial interrupts are suppressed.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *   enable - True to enable transmission interrupts; false to disable.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void erbium_txint(struct uart_dev_s *dev, bool enable)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      erbium_setuartint(priv, priv->ien | ERBIUM_UART_IEN_TX_EMPTY);
      uart_xmitchars(dev);
#endif
    }
  else
    {
      erbium_setuartint(priv, priv->ien & ~ERBIUM_UART_IEN_TX_EMPTY);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: erbium_txready
 *
 * Description:
 *   Check whether the transmit FIFO has room for another character.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *
 * Returned Value:
 *   True if the transmit FIFO is not full; otherwise false.
 *
 ****************************************************************************/

static bool erbium_txready(struct uart_dev_s *dev)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;

  return (erbium_serialin(priv, ERBIUM_UART_STATUS_OFFSET) &
          ERBIUM_UART_STATUS_TX_FULL) == 0;
}

/****************************************************************************
 * Name: erbium_txempty
 *
 * Description:
 *   Check whether the UART transmit FIFO is empty.
 *
 * Input Parameters:
 *   dev - Serial lower-half device.
 *
 * Returned Value:
 *   True if the transmit FIFO is empty; otherwise false.
 *
 ****************************************************************************/

static bool erbium_txempty(struct uart_dev_s *dev)
{
  struct erbium_uart_s *priv = (struct erbium_uart_s *)dev->priv;

  return (erbium_serialin(priv, ERBIUM_UART_STATUS_OFFSET) &
          ERBIUM_UART_STATUS_TX_EMPTY) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Prepare UART0 for early console output with UART interrupts disabled.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  erbium_disableuartint(&g_uart0priv, NULL);
  erbium_setup(&g_uart0port);
}

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register UART0 as ttyS0 and, when selected, as the system console.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  uart_register("/dev/console", &g_uart0port);
#endif

  uart_register("/dev/ttyS0", &g_uart0port);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Write one low-level console character while preserving the UART
 *   interrupt mask and the caller's interrupt state.
 *
 * Input Parameters:
 *   ch - Character to transmit.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_putc(int ch)
{
  irqstate_t flags;
  uint32_t ien;

  flags = enter_critical_section();
  erbium_disableuartint(&g_uart0priv, &ien);
  erbium_putc(&g_uart0priv, ch);
  erbium_restoreuartint(&g_uart0priv, ien);
  leave_critical_section(flags);
}
