/****************************************************************************
 * arch/tricore/src/tc3xx/tc3xx_serial.c
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
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "tricore_internal.h"

#include "Asclin/Asc/IfxAsclin_Asc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_TC3XX_UART0)
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port     /* UART0 is console */
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    define SERIAL_CONSOLE  1
#  else
#    error "I'm confused... Do we have a serial console or not?"
#  endif
#else
#  undef  CONSOLE_DEV                        /* No console */
#  undef  CONFIG_UART0_SERIAL_CONSOLE
#  if defined(CONFIG_TC3XX_UART0)
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    define SERIAL_CONSOLE  1
#  else
#    undef  TTYS0_DEV
#  endif
#endif

#undef HAVE_UART_DEVICE
#if defined(CONFIG_TC3XX_UART0)
#  define HAVE_UART_DEVICE 1
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of tricore_earlyserialinit(),
 * tricore_serialinit(), and up_putc().
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  volatile void *uartbase; /* Base address of UART registers */
  const void    *pins;     /* Pin configuration */
  uint32_t       baud;     /* Configured baud */
  uint8_t        irq;      /* IRQ associated with this UART */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_TC3XX_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_TC3XX_UART0

#define UART_PIN_RX IfxAsclin0_RXA_P14_1_IN     /* UART receive port pin  */
#define UART_PIN_TX IfxAsclin0_TX_P14_0_OUT     /* UART transmit port pin */

/* Pin configuration */

static const IfxAsclin_Asc_Pins g_uart0_pins =
{
  NULL,           IfxPort_InputMode_pullUp,     /* CTS pin not used */
  &UART_PIN_RX,   IfxPort_InputMode_pullUp,     /* RX pin           */
  NULL,           IfxPort_OutputMode_pushPull,  /* RTS pin not used */
  &UART_PIN_TX,   IfxPort_OutputMode_pushPull,  /* TX pin           */
  IfxPort_PadDriver_cmosAutomotiveSpeed1
};

static struct up_dev_s g_uart0priv =
{
  .uartbase  = &MODULE_ASCLIN0,
  .pins      = &g_uart0_pins,
  .baud      = CONFIG_UART0_BAUD,
  .irq       = 21,
};

static uart_dev_t g_uart0port =
{
#if SERIAL_CONSOLE == 1
  .isconsole = 1,
#endif
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: asclin_init
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static void asclin_init(struct up_dev_s *priv)
{
  Ifx_ASCLIN *asclin = priv->uartbase;
  const IfxAsclin_Asc_Pins *pins = priv->pins;

  /* enabling the module */

  IfxAsclin_enableModule(asclin);

  /* disabling the clock */

  IfxAsclin_setClockSource(asclin, IfxAsclin_ClockSource_noClock);

  /* setting the module in Initialise mode */

  IfxAsclin_setFrameMode(asclin, IfxAsclin_FrameMode_initialise);

  /* sets the prescaler */

  IfxAsclin_setPrescaler(asclin, 1);

  /* temporary set the clock source for baudrate configuration */

  IfxAsclin_setClockSource(asclin, IfxAsclin_ClockSource_ascFastClock);

  /* setting the baudrate bit fields to generate the required baudrate */

  IfxAsclin_setBitTiming(asclin, priv->baud,
                         IfxAsclin_OversamplingFactor_16,
                         IfxAsclin_SamplePointPosition_8,
                         IfxAsclin_SamplesPerBit_three);

  /* disabling the clock again */

  IfxAsclin_setClockSource(asclin, IfxAsclin_ClockSource_noClock);

  /* selecting the loopback mode */

  IfxAsclin_enableLoopBackMode(asclin, false);

  /* setting parity enable */

  IfxAsclin_enableParity(asclin, false);

  /* setting parity type (odd/even) */

  IfxAsclin_setParityType(asclin, IfxAsclin_ParityType_even);

  /* setting the stop bit */

  IfxAsclin_setStopBit(asclin, IfxAsclin_StopBit_1);

  /* setting the shift direction */

  IfxAsclin_setShiftDirection(asclin,
    IfxAsclin_ShiftDirection_lsbFirst);

  /* setting the data length */

  IfxAsclin_setDataLength(asclin, IfxAsclin_DataLength_8);

  /* setting Tx FIFO inlet width */

  IfxAsclin_setTxFifoInletWidth(asclin,
    IfxAsclin_TxFifoInletWidth_1);

  /* setting Rx FIFO outlet width */

  IfxAsclin_setRxFifoOutletWidth(asclin,
    IfxAsclin_RxFifoOutletWidth_1);

  /* setting idle delay */

  IfxAsclin_setIdleDelay(asclin, IfxAsclin_IdleDelay_0);

  /* setting Tx FIFO level at which a Tx interrupt will be triggered */

  IfxAsclin_setTxFifoInterruptLevel(asclin,
    IfxAsclin_TxFifoInterruptLevel_0);

  /* setting Rx FIFO interrupt level at which a Rx
   * interrupt will be triggered
   */

  IfxAsclin_setRxFifoInterruptLevel(asclin,
    IfxAsclin_RxFifoInterruptLevel_1);

  /* setting Tx FIFO interrupt generation mode */

  IfxAsclin_setTxFifoInterruptMode(asclin,
    IfxAsclin_FifoInterruptMode_combined);

  /* setting Rx FIFO interrupt generation mode */

  IfxAsclin_setRxFifoInterruptMode(asclin,
    IfxAsclin_FifoInterruptMode_combined);

  /* selecting the frame mode */

  IfxAsclin_setFrameMode(asclin, IfxAsclin_FrameMode_asc);

  /* Pin mapping */

  if (pins != NULL)
    {
      IfxAsclin_Cts_In *cts = pins->cts;

      if (cts != NULL)
        {
          IfxAsclin_initCtsPin(cts, pins->ctsMode, pins->pinDriver);
        }

      IfxAsclin_Rx_In *rx = pins->rx;

      if (rx != NULL)
        {
          IfxAsclin_initRxPin(rx, pins->rxMode, pins->pinDriver);
        }

      IfxAsclin_Rts_Out *rts = pins->rts;

      if (rts != NULL)
        {
          IfxAsclin_initRtsPin(rts, pins->rtsMode, pins->pinDriver);
        }

      IfxAsclin_Tx_Out *tx = pins->tx;

      if (tx != NULL)
        {
          IfxAsclin_initTxPin(tx, pins->txMode, pins->pinDriver);
        }
    }

  /* select the clock source */

  IfxAsclin_setClockSource(asclin, IfxAsclin_ClockSource_ascFastClock);

  /* disable all flags */

  IfxAsclin_disableAllFlags(asclin);

  /* clear all flags */

  IfxAsclin_clearAllFlags(asclin);

  /* HW error flags */

  IfxAsclin_enableParityErrorFlag(asclin, true);
  IfxAsclin_enableFrameErrorFlag(asclin, true);
  IfxAsclin_enableRxFifoOverflowFlag(asclin, true);
  IfxAsclin_enableRxFifoUnderflowFlag(asclin, true);
  IfxAsclin_enableTxFifoOverflowFlag(asclin, true);

  /* enable transfers */

  IfxAsclin_enableRxFifoInlet(asclin, true);
  IfxAsclin_enableTxFifoOutlet(asclin, true);

  IfxAsclin_flushRxFifo(asclin);
  IfxAsclin_flushTxFifo(asclin);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  asclin_init(dev->priv);

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = dev->priv;
  int ret;

  /* Initialize interrupt generation on the peripheral */

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = arg;

  if (up_rxavailable(dev))
    {
      uart_recvchars(dev);
    }

  if (up_txready(dev))
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = dev->priv;

  return IfxAsclin_readRxData(priv->uartbase);
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = dev->priv;
  irqstate_t flags = enter_critical_section();

  IfxAsclin_enableRxFifoFillLevelFlag(priv->uartbase, enable);
  IfxAsclin_enableRxFifoInlet(priv->uartbase, enable);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = dev->priv;

  return IfxAsclin_getRxFifoFillLevel(priv->uartbase) > 0;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = dev->priv;

  /* Wait for FIFO */

  if (dev == &CONSOLE_DEV)
    {
      up_putc(ch);
      return;
    }

  while (IfxAsclin_getTxFifoFillLevel(priv->uartbase) != 0);

  IfxAsclin_clearAllFlags(priv->uartbase);
  IfxAsclin_writeTxData(priv->uartbase, ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
      /* Enable the TX interrupt */

      uart_xmitchars(dev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = dev->priv;

  /* Return true if the TX wartermak is pending */

  return IfxAsclin_getTxFifoFillLevel(priv->uartbase) != 0;
}

/****************************************************************************
 * Name: tricore_lowputc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void tricore_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  struct up_dev_s *priv = CONSOLE_DEV.priv;

  /* Wait for FIFO */

  while (IfxAsclin_getTxFifoFillLevel(priv->uartbase) != 0);

  IfxAsclin_clearAllFlags(priv->uartbase);
  IfxAsclin_writeTxData(priv->uartbase, ch);
#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: tricore_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before tricore_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void tricore_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: tricore_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that tricore_earlyserialinit was called previously.
 *
 ****************************************************************************/

void tricore_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  tricore_lowputc(ch);
#endif
}
#endif /* USE_SERIALDRIVER */
