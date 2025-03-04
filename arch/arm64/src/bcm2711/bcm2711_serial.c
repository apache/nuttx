/***************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_serial.c
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

#include "bcm2711_gpio.h"
#include "bcm2711_serial.h"
#include "hardware/bcm2711_aux.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Mini UART settings. */

#ifndef CONFIG_MINIUART_BAUD
#define CONFIG_MINIUART_BAUD 115200
#endif

#ifndef CONFIG_MINIUART_BITS
#define CONFIG_MINIUART_BITS 8
#endif

#ifndef CONFIG_MINIUART_PARITY
#define CONFIG_MINIUART_PARITY 0
#endif

#ifndef CONFIG_MINIUART_RXBUFSIZE
#define CONFIG_MINIUART_RXBUFSIZE 256
#endif

#ifndef CONFIG_MINIUART_TXBUFSIZE
#define CONFIG_MINIUART_TXBUFSIZE 256
#endif

#define CONSOLE_DEV g_miniuartport /* Mini UART is console */
#define TTYS0_DEV g_miniuartport   /* Mini UART is ttys0 */

/* System clock frequency for Mini UART in Hz */

#define SYSTEM_CLOCK_FREQUENCY 500000000

/* Baud rate calculation */

#define AUX_MU_BAUD(baud) ((SYSTEM_CLOCK_FREQUENCY / (baud * 8)) - 1)

/***************************************************************************
 * Private Types
 ***************************************************************************/

/* UART configuration parameters for all UART ports */

struct bcm2711_uart_config_s
{
  uint32_t baud_rate; /* UART baud rate */
  uint8_t parity;     /* 0 = N, 1 = Odd, 2 = even */
  uint8_t data_bits;  /* Number of data bits per baud */
};

/* UART port definition for Mini UART port */

struct bcm2711_miniuart_port_s
{
  struct bcm2711_uart_config_s config; /* UART port configuration */
};

/* UART port definition for PL011 UART port */

struct bcm2711_uart_port_s
{
  struct bcm2711_uart_config_s config; /* UART port configuration */
  unsigned long base_addr;             /* UART port base address */
};

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

/* Mini UART helper functions */

static void bcm2711_miniuart_wait_send(struct uart_dev_s *dev, char c);
static int bcm2711_miniuart_irq_handler(int irq, void *context, void *arg);

/* Mini UART operations */

static int bcm2711_miniuart_setup(struct uart_dev_s *dev);
static void bcm2711_miniuart_shutdown(struct uart_dev_s *dev);
static int bcm2711_miniuart_attach(struct uart_dev_s *dev);
static void bcm2711_miniuart_detach(struct uart_dev_s *dev);
static int bcm2711_miniuart_ioctl(struct file *filep, int cmd,
                                  unsigned long arg);
static int bcm2711_miniuart_receive(struct uart_dev_s *dev,
                                    unsigned int *status);
static void bcm2711_miniuart_rxint(struct uart_dev_s *dev, bool enable);
static bool bcm2711_miniuart_rxavailable(struct uart_dev_s *dev);
static void bcm2711_miniuart_send(struct uart_dev_s *dev, int c);
static void bcm2711_miniuart_txint(struct uart_dev_s *dev, bool enable);
static bool bcm2711_miniuart_txready(struct uart_dev_s *dev);
static bool bcm2711_miniuart_txempty(struct uart_dev_s *dev);

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* UART operations for serial driver */

static const struct uart_ops_s g_miniuart_ops =
{
    .setup = bcm2711_miniuart_setup,
    .shutdown = bcm2711_miniuart_shutdown,
    .attach = bcm2711_miniuart_attach,
    .detach = bcm2711_miniuart_detach,
    .ioctl = bcm2711_miniuart_ioctl,
    .receive = bcm2711_miniuart_receive,
    .rxint = bcm2711_miniuart_rxint,
    .rxavailable = bcm2711_miniuart_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rxflowcontrol = NULL,
#endif
    .send = bcm2711_miniuart_send,
    .txint = bcm2711_miniuart_txint,
    .txready = bcm2711_miniuart_txready,
    .txempty = bcm2711_miniuart_txempty,
};

/* Mini UART I/O Buffers (Console) */

static char g_miniuartrxbuffer[CONFIG_MINIUART_RXBUFSIZE];
static char g_miniuarttxbuffer[CONFIG_MINIUART_TXBUFSIZE];

static struct bcm2711_miniuart_port_s g_miniuartpriv =
{
    .config =
        {
            .baud_rate = CONFIG_MINIUART_BAUD,
            .parity = CONFIG_MINIUART_PARITY,
            .data_bits = CONFIG_MINIUART_BITS,
        },
};

static struct uart_dev_s g_miniuartport =
{
    .recv =
        {
            .size = CONFIG_MINIUART_RXBUFSIZE,
            .buffer = g_miniuartrxbuffer,
        },

    .xmit =
        {
            .size = CONFIG_MINIUART_TXBUFSIZE,
            .buffer = g_miniuarttxbuffer,
        },

    .ops = &g_miniuart_ops,
    .priv = &g_miniuartpriv,
};

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Name: bcm2711_miniuart_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 * Input Parameters:
 *   dev    - UART Device
 *   enable - True to enable TX interrupts; false to disable
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void bcm2711_miniuart_txint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      modreg32(BCM_AUX_MU_IER_TXD, BCM_AUX_MU_IER_TXD, BCM_AUX_MU_IER_REG);
    }
  else
    {
      modreg32(0, BCM_AUX_MU_IER_TXD, BCM_AUX_MU_IER_REG);
    }
}

/***************************************************************************
 * Name: bcm2711_miniuart_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 * Input Parameters:
 *   dev    - UART Device
 *   enable - True to enable RX interrupts; false to disable
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void bcm2711_miniuart_rxint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      modreg32(BCM_AUX_MU_IER_RXD, BCM_AUX_MU_IER_RXD, BCM_AUX_MU_IER_REG);
    }
  else
    {
      modreg32(0, BCM_AUX_MU_IER_RXD, BCM_AUX_MU_IER_REG);
    }
}

/***************************************************************************
 * Name: bcm2711_miniuart_shutdown
 *
 * Description:
 *   Disable the UART Port.  This method is called when the serial
 *   port is closed.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void bcm2711_miniuart_shutdown(struct uart_dev_s *dev)
{
  /* Disable interrupts */

  bcm2711_miniuart_rxint(dev, false);
  bcm2711_miniuart_txint(dev, false);

  /* Disable Mini UART peripheral. */

  modreg32(BCM_AUX_ENABLE_MU, BCM_AUX_ENABLE_MU, BCM_AUX_ENABLES);
}

/***************************************************************************
 * Name: bcm2711_miniuart_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int bcm2711_miniuart_setup(struct uart_dev_s *dev)
{
  struct bcm2711_miniuart_port_s *port =
      (struct bcm2711_miniuart_port_s *)dev->priv;

  /* Enable Mini UART */

  modreg32(BCM_AUX_ENABLE_MU, BCM_AUX_ENABLE_MU, BCM_AUX_ENABLES);

  /* Disable interrupts. */

  bcm2711_miniuart_txint(dev, false);
  bcm2711_miniuart_rxint(dev, false);

  /* Disable TX and RX of the UART */

  modreg32(0, BCM_AUX_MU_CNTL_RXENABLE, BCM_AUX_MU_CNTL_REG);
  modreg32(0, BCM_AUX_MU_CNTL_TXENABLE, BCM_AUX_MU_CNTL_REG);

  /* Set data bit count */

  if (port->config.data_bits == 7)
    {
      /* 7 data bits */

      modreg32(0, BCM_AUX_MU_LCR_DATA8B, BCM_AUX_MU_LCR_REG);
    }
  else
    {
      /* 8 data bits */

      modreg32(BCM_AUX_MU_LCR_DATA8B, BCM_AUX_MU_LCR_DATA8B,
               BCM_AUX_MU_LCR_REG);
    }

  /* Ensure RTS line is low. */

  modreg32(0, BCM_AUX_MU_MCR_RTS, BCM_AUX_MU_MCR_REG);

  /* Clear the TX and RX FIFOs */

  modreg32(BCM_AUX_MU_IIR_RXCLEAR, BCM_AUX_MU_IIR_RXCLEAR,
           BCM_AUX_MU_IIR_REG);
  modreg32(BCM_AUX_MU_IIR_TXCLEAR, BCM_AUX_MU_IIR_TXCLEAR,
           BCM_AUX_MU_IIR_REG);

  /* Set baud rate. */

  putreg32(AUX_MU_BAUD(port->config.baud_rate), BCM_AUX_MU_BAUD_REG);

  /* GPIO 14 and GPIO 15 are used as TX and RX.
   * TODO: Make pins configurable.
   */

  /* Turn off pull-up/pull-down resistors. */

  bcm2711_gpio_set_pulls(14, false, false);
  bcm2711_gpio_set_pulls(15, false, false);

  /* Use alternative function 5 (UART1). */

  bcm2711_gpio_set_func(14, BCM_GPIO_FUNC5);
  bcm2711_gpio_set_func(15, BCM_GPIO_FUNC5);

  /* Enable TX and RX again. */

  modreg32(BCM_AUX_MU_CNTL_TXENABLE, BCM_AUX_MU_CNTL_TXENABLE,
           BCM_AUX_MU_CNTL_REG);
  modreg32(BCM_AUX_MU_CNTL_RXENABLE, BCM_AUX_MU_CNTL_RXENABLE,
           BCM_AUX_MU_CNTL_REG);

  return 0;
}

/***************************************************************************
 * Name: bcm2711_miniuart_txready
 *
 * Description:
 *   Return true if the Transmit FIFO is not full
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Transmit FIFO is not full; false otherwise
 *
 ***************************************************************************/

static bool bcm2711_miniuart_txready(struct uart_dev_s *dev)
{
  return getreg32(BCM_AUX_MU_STAT_REG) & BCM_AUX_MU_STAT_SPACEAVAIL;
}

/***************************************************************************
 * Name: bcm2711_miniuart_txempty
 *
 * Description:
 *   Return true if the Transmit FIFO is empty
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Transmit FIFO is empty; false otherwise
 *
 ***************************************************************************/

static bool bcm2711_miniuart_txempty(struct uart_dev_s *dev)
{
  return getreg32(BCM_AUX_MU_STAT_REG) & BCM_AUX_MU_STAT_TXEMPTY;
}

/***************************************************************************
 * Name: bcm2711_miniuart_rxavailable
 *
 * Description:
 *   Return true if the Receive FIFO is not empty
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Receive FIFO is not empty; false otherwise
 *
 ***************************************************************************/

static bool bcm2711_miniuart_rxavailable(struct uart_dev_s *dev)
{
  return getreg32(BCM_AUX_MU_STAT_REG) & BCM_AUX_MU_STAT_SYMAVAIL;
}

/***************************************************************************
 * Name: bcm2711_miniuart_wait_send
 *
 * Description:
 *   Wait for Transmit FIFO until it is not full, then transmit the
 *   character over UART.
 *
 * Input Parameters:
 *   dev - UART Device
 *   c  - Character to be sent
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void bcm2711_miniuart_wait_send(struct uart_dev_s *dev, char c)
{
  while (!bcm2711_miniuart_txready(dev))
    ;

  /* Write byte (do I need to mask to avoid writing to LS8 baud rate
   * bits?)
   */

  bcm2711_miniuart_send(dev, c);
}

/***************************************************************************
 * Name: bcm2711_miniuart_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 * Input Parameters:
 *   dev - UART Device
 *   c  - Character to be sent
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void bcm2711_miniuart_send(struct uart_dev_s *dev, int c)
{
  putreg32(c, BCM_AUX_MU_IO_REG);
}

/***************************************************************************
 * Name: bcm2711_miniuart_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 * Input Parameters:
 *   dev    - UART Device
 *   status - Return status, zero on success
 *
 * Returned Value:
 *   Received character
 *
 ***************************************************************************/

static int bcm2711_miniuart_receive(struct uart_dev_s *dev,
                                    unsigned int *status)
{
  /* TODO proper status */

  *status = 0;                               /* OK */
  return getreg32(BCM_AUX_MU_IO_REG) & 0xff; /* Read byte */
}

/***************************************************************************
 * Name: bcm2711_miniuart_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   filep - File Struct
 *   cmd   - ioctl Command
 *   arg   - ioctl Argument
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int bcm2711_miniuart_ioctl(struct file *filep, int cmd,
                                  unsigned long arg)
{
  int ret = OK;
  UNUSED(filep);
  UNUSED(arg);

  switch (cmd)
    {
    case TIOCSBRK: /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK: /* BSD compatibility: Turn break off, unconditionally */
    default:
      ret = -ENOTTY;
      break;
    }

  /* TODO: implement actual commands */

  return ret;
}

/***************************************************************************
 * Name: bcm2711_miniuart_attach
 *
 * Description:
 *   Configure the UART to operate in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the setup() method is called,
 *   however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method
 *   (unless the hardware supports multiple levels of interrupt
 *   enabling).  The RX and TX interrupts are not enabled until
 *   the txint() and rxint() methods are called.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int bcm2711_miniuart_attach(struct uart_dev_s *dev)
{
  int ret = -ENOSYS;
  const struct bcm2711_miniuart_port_s *port =
      (struct bcm2711_miniuart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Attach interrupt handler. TODO: this is for all AUX interrupts */

  ret = irq_attach(BCM_IRQ_VC_AUX, bcm2711_miniuart_irq_handler, dev);

  /* Set interrupt priority in GICv2 */

  arm64_gic_irq_set_priority(BCM_IRQ_VC_AUX, 0, IRQ_TYPE_LEVEL);

  /* Enable UART interrupt */

  if (ret == OK)
    {
      up_enable_irq(BCM_IRQ_VC_AUX);
      irqinfo("Attached AUX interrupt handler");
    }
  else
    {
      irqerr("Could not attach Mini UART IRQ handler, ret=%d\n", ret);
    }

  return ret;
}

/***************************************************************************
 * Name: bcm2711_miniuart_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void bcm2711_miniuart_detach(struct uart_dev_s *dev)
{
  const struct bcm2711_miniuart_port_s *port =
      (struct bcm2711_miniuart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Disable the Mini UART interrupt. TODO: This will disable all AUX
   * interrupts
   */

  up_disable_irq(BCM_IRQ_VC_AUX);

  /* Detach the interrupt handler */

  irq_detach(BCM_IRQ_VC_AUX);
}

/***************************************************************************
 * Name: bcm2711_miniuart_irq_handler
 *
 * Description:
 *   Handles Mini UART IRQ. Performs the appropriate data transfers.
 *
 * Input Parameters:
 *   irq - The IRQ number
 *   context - The interrupt context
 *   arg - A UART device for the Mini UART port
 *
 * Returned Value:
 *   OK, or -EIO if the interrupt ID is invalid.
 *
 ***************************************************************************/

static int bcm2711_miniuart_irq_handler(int irq, void *context, void *arg)
{
  int ret = OK;
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  const struct bcm2711_miniuart_port_s *port =
      (struct bcm2711_miniuart_port_s *)dev->priv;

  DEBUGASSERT(dev != NULL && port != NULL);

  /* Check if the interrupt is the Mini UART, return early otherwise. */

  if (!(getreg32(BCM_AUX_IRQ) & BCM_AUX_IRQ_MU))
    {
      return ret;
    }

  /* Check the Mini UART interrupt status */

  uint32_t aux_iir = getreg32(BCM_AUX_MU_IIR_REG);
  if (aux_iir & BCM_AUX_MU_IIR_PENDING)
    {
      /* Bit is set when no interrupt is pending */

      return ret;
    }

  /* Check interrupt ID */

  switch (aux_iir & 0b110)
    {
    case BCM_AUX_MU_IIR_RXBYTE:

      /* Receiver holds valid byte */

      uart_recvchars(dev);
      break;

    case BCM_AUX_MU_IIR_TXEMPTY:

      /* Transmit holding register is empty */

      uart_xmitchars(dev);
      break;

    case BCM_AUX_MU_IIR_NONE:

      /* No interrupt, do nothing */

      break;

    default:

      /* Impossible case of 0b11 */

      ret = -EIO;
      break;
    }

  return ret;
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm64_serialinit.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void arm64_earlyserialinit(void)
{
#ifdef CONSOLE_DEV
  bcm2711_miniuart_setup(&CONSOLE_DEV);
#endif
}

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 * Input Parameters:
 *   ch - Character to be transmitted over UART
 *
 ***************************************************************************/

void up_putc(int ch)
{
#ifdef CONSOLE_DEV
  struct uart_dev_s *dev = &CONSOLE_DEV;

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      bcm2711_miniuart_wait_send(dev, '\r');
    }

  bcm2711_miniuart_wait_send(dev, ch);
#endif /* CONSOLE_DEV */
}

/***************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   Register serial console and serial ports. This assumes
 *   that bcm2711_earlyserialinit was called previously.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void arm64_serialinit(void)
{
#if defined(CONSOLE_DEV)

  /* Mark the console. */

  CONSOLE_DEV.isconsole = 1;

  int ret;
  ret = uart_register("/dev/console", &CONSOLE_DEV);
  if (ret < 0)
    {
      _err("Could not register /dev/console, ret=%d\n", ret);
    }
#endif /* defined(CONSOLE_DEV) */
}
