/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_serial.c
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
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "riscv_internal.h"
#include "chip.h"
#include "hardware/esp32c6_uart.h"

#include "esp32c6_lowputc.h"
#include "esp32c6_config.h"
#include "esp32c6_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The console is enabled, and it's not the syslog device,
 * so, it should be a serial device.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1? */

/* First pick the console and ttys0.
 * Console can be UART0 or UART1, but will always be ttys0.
 */

/* In case a UART was assigned to be
 * the console and the corresponding peripheral was also selected.
 */

#ifdef CONSOLE_UART
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0_dev     /* UART0 is console */
#    define TTYS0_DEV       g_uart0_dev     /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
# elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1_dev  /* UART1 is console */
#    define TTYS0_DEV           g_uart1_dev  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  endif /* CONFIG_UART0_SERIAL_CONSOLE */
#else /* No UART console */
#  undef  CONSOLE_DEV
#  if defined(CONFIG_ESP32C6_UART0)
#    define TTYS0_DEV           g_uart0_dev  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_ESP32C6_UART1)
#    define TTYS0_DEV           g_uart1_dev  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  endif
#endif /* CONSOLE_UART */

/* Pick ttys1 */

#if defined(CONFIG_ESP32C6_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0_dev  /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_ESP32C5_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1_dev  /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#endif

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32C6_UART

/* Serial driver methods */

static int  esp32c6_setup(struct uart_dev_s *dev);
static void esp32c6_shutdown(struct uart_dev_s *dev);
static int  esp32c6_attach(struct uart_dev_s *dev);
static void esp32c6_detach(struct uart_dev_s *dev);
static void esp32c6_txint(struct uart_dev_s *dev, bool enable);
static void esp32c6_rxint(struct uart_dev_s *dev, bool enable);
static bool esp32c6_rxavailable(struct uart_dev_s *dev);
static bool esp32c6_txready(struct uart_dev_s *dev);
static bool esp32c6_txempty(struct uart_dev_s *dev);
static void esp32c6_send(struct uart_dev_s *dev, int ch);
static int  esp32c6_receive(struct uart_dev_s *dev, unsigned int *status);
static int  esp32c6_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool esp32c6_rxflowcontrol(struct uart_dev_s *dev,
                                  unsigned int nbuffered, bool upper);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32C6_UART

/* Operations */

static struct uart_ops_s g_uart_ops =
{
    .setup       = esp32c6_setup,
    .shutdown    = esp32c6_shutdown,
    .attach      = esp32c6_attach,
    .detach      = esp32c6_detach,
    .txint       = esp32c6_txint,
    .rxint       = esp32c6_rxint,
    .rxavailable = esp32c6_rxavailable,
    .txready     = esp32c6_txready,
    .txempty     = esp32c6_txempty,
    .send        = esp32c6_send,
    .receive     = esp32c6_receive,
    .ioctl       = esp32c6_ioctl,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rxflowcontrol  = esp32c6_rxflowcontrol,
#endif
};

/* UART 0 */

#ifdef CONFIG_ESP32C6_UART0

static char g_uart0_rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0_txbuffer[CONFIG_UART0_TXBUFSIZE];

/* Fill only the requested fields */

static uart_dev_t g_uart0_dev =
{
#ifdef CONFIG_UART0_SERIAL_CONSOLE
    .isconsole = true,
#else
    .isconsole = false,
#endif
    .xmit =
    {
        .size   = CONFIG_UART0_TXBUFSIZE,
        .buffer = g_uart0_txbuffer,
    },
    .recv =
    {
        .size   = CONFIG_UART0_RXBUFSIZE,
        .buffer = g_uart0_rxbuffer,
    },

    .ops = &g_uart_ops,
    .priv = &g_uart0_config
};

#endif

/* UART 1 */

#ifdef CONFIG_ESP32C6_UART1

static char g_uart1_rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1_txbuffer[CONFIG_UART1_TXBUFSIZE];

/* Fill only the requested fields */

static uart_dev_t g_uart1_dev =
{
#ifdef CONFIG_UART1_SERIAL_CONSOLE
    .isconsole = true,
#else
    .isconsole = false,
#endif
    .xmit =
    {
        .size   = CONFIG_UART1_TXBUFSIZE,
        .buffer = g_uart1_txbuffer,
    },
    .recv =
    {
        .size   = CONFIG_UART1_RXBUFSIZE,
        .buffer = g_uart1_rxbuffer,
    },

    .ops = &g_uart_ops,
    .priv = &g_uart1_config
};

#endif

#endif /* CONFIG_ESP32C6_UART */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESP32C6_UART

/****************************************************************************
 * Name: uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int uart_handler(int irq, FAR void *context, FAR void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct esp32c6_uart_s *priv = dev->priv;
  uint32_t tx_mask = UART_TXFIFO_EMPTY_INT_ST_M | UART_TX_DONE_INT_ST_M;
  uint32_t rx_mask = UART_RXFIFO_TOUT_INT_ST_M | UART_RXFIFO_FULL_INT_ST_M;
  uint32_t int_status;

  int_status = getreg32(UART_INT_ST_REG(priv->id));

  /* Tx fifo empty interrupt or UART tx done int */

  if (int_status & tx_mask)
    {
      uart_xmitchars(dev);
      modifyreg32(UART_INT_CLR_REG(priv->id), tx_mask, tx_mask);
    }

  /* Rx fifo timeout interrupt or rx fifo full interrupt */

  if (int_status & rx_mask)
    {
      uart_recvchars(dev);
      modifyreg32(UART_INT_CLR_REG(priv->id), rx_mask, rx_mask);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c6_setup
 *
 * Description:
 *      Configure the UART baud, bits, parity, fifos, etc. This method is
 *      called the first time that the serial port is opened.
 *      For the serial console, this will occur very early in initialization,
 *      for other serial ports this will occur when the port is first opened.
 *      This setup does not include attaching or enabling interrupts.
 *      That portion of the UART setup is performed when the attach() method
 *      is called.
 *
 ****************************************************************************/

static int esp32c6_setup(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: esp32c6_shutdown
 *
 * Description:
 * Disable the UART.  This method is called when the serial port is closed.
 * This method reverses the operation the setup method.  NOTE that the serial
 * console is never shutdown.
 *
 ****************************************************************************/

static void esp32c6_shutdown(struct uart_dev_s *dev)
{
  struct esp32c6_uart_s *priv = dev->priv;

  /* Disable ints */

  esp32c6_lowputc_disable_all_uart_int(priv, NULL);
}

/****************************************************************************
 * Name: esp32c6_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int esp32c6_attach(struct uart_dev_s *dev)
{
  struct esp32c6_uart_s *priv = dev->priv;
  int ret;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

  /* Set up to receive peripheral interrupts */

  priv->cpuint = esp32c6_setup_irq(priv->periph, priv->int_pri,
                                   ESP32C6_INT_LEVEL);
  if (priv->cpuint < 0)
    {
      return priv->cpuint;
    }

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, uart_handler, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }
  else
    {
      up_disable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void esp32c6_detach(struct uart_dev_s *dev)
{
  struct esp32c6_uart_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  /* Disable and detach the CPU interrupt */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  /* Disassociate the peripheral interrupt from the CPU interrupt */

  esp32c6_teardown_irq(priv->periph, priv->cpuint);
  priv->cpuint = -ENOMEM;
}

/****************************************************************************
 * Name: esp32c6_txint
 *
 * Description:
 * Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void esp32c6_txint(struct uart_dev_s *dev, bool enable)
{
  struct esp32c6_uart_s *priv = dev->priv;
  uint32_t ints_mask = UART_TXFIFO_EMPTY_INT_ENA_M | UART_TX_DONE_INT_ENA_M;

  if (enable)
    {
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      modifyreg32(UART_INT_ENA_REG(priv->id), ints_mask, ints_mask);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      modifyreg32(UART_INT_ENA_REG(priv->id), ints_mask, 0);
    }
}

/****************************************************************************
 * Name: esp32c6_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void esp32c6_rxint(struct uart_dev_s *dev, bool enable)
{
  struct esp32c6_uart_s *priv = dev->priv;
  uint32_t ints_mask = UART_RXFIFO_TOUT_INT_ENA_M |
                       UART_RXFIFO_FULL_INT_ENA_M;

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      modifyreg32(UART_CONF1_REG(priv->id), UART_RX_TOUT_EN_M,
                  UART_RX_TOUT_EN_M);
      modifyreg32(UART_INT_ENA_REG(priv->id), ints_mask, ints_mask);
#endif
    }
  else
    {
      modifyreg32(UART_CONF1_REG(priv->id), UART_RX_TOUT_EN_M, 0);

      /* Disable the RX interrupts */

      modifyreg32(UART_INT_ENA_REG(priv->id), ints_mask, 0);
    }
}

/****************************************************************************
 * Name: esp32c6_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool esp32c6_rxavailable(struct uart_dev_s *dev)
{
  struct esp32c6_uart_s *priv = dev->priv;
  uint32_t status_reg;
  uint32_t bytes;

  status_reg = getreg32(UART_STATUS_REG(priv->id));
  bytes = status_reg & UART_RXFIFO_CNT_M;

  return (bytes > 0) ? true : false;
}

/****************************************************************************
 * Name: esp32c6_txready
 *
 * Description:
 * Return true if the tranmsit hardware is ready to send another byte.  This
 * is used to determine if send() method can be called.
 *
 ****************************************************************************/

static bool esp32c6_txready(struct uart_dev_s *dev)
{
  return (esp32c6_lowputc_is_tx_fifo_full(dev->priv)) ? false : true;
}

/****************************************************************************
 * Name: esp32c6_txempty
 *
 * Description:
 * Return true if all characters have been sent.  If for example, the UART
 * hardware implements FIFOs, then this would mean the transmit FIFO is
 * empty.  This method is called when the driver needs to make sure that
 * all characters are "drained" from the TX hardware.
 *
 ****************************************************************************/

static bool esp32c6_txempty(struct uart_dev_s *dev)
{
  uint32_t reg;
  struct esp32c6_uart_s *priv = dev->priv;

  reg = getreg32(UART_INT_RAW_REG(priv->id));
  reg = reg & UART_TXFIFO_EMPTY_INT_RAW_M;

  return (reg > 0) ? true : false;
}

/****************************************************************************
 * Name: esp32c6_send
 *
 * Description:
 *    Send a unique character
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *   ch         -  Byte to be sent.
 *
 ****************************************************************************/

static void esp32c6_send(struct uart_dev_s *dev, int ch)
{
  /* Then send the character */

  esp32c6_lowputc_send_byte(dev->priv, ch);
}

/****************************************************************************
 * Name: esp32c6_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int esp32c6_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint32_t rx_fifo;
  struct esp32c6_uart_s *priv = dev->priv;

  rx_fifo = getreg32(UART_FIFO_REG(priv->id));
  rx_fifo = rx_fifo & UART_RXFIFO_RD_BYTE_M;

  /* Since we don't have error bits associated with receipt, we set zero */

  *status = 0;

  return (int)rx_fifo;
}

/****************************************************************************
 * Name: esp32c6_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *   Here it's employed to implement the TERMIOS ioctls and TIOCSERGSTRUCT.
 *
 * Parameters:
 *   filep    Pointer to a file structure instance.
 *   cmd      The ioctl command.
 *   arg      The argument of the ioctl cmd.
 *
 * Returned Value:
 *   Returns a non-negative number on success;  A negated errno value is
 *   returned on any failure (see comments ioctl() for a list of appropriate
 *   errno values).
 *
 ****************************************************************************/

static int esp32c6_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return OK;
}

#endif /* CONFIG_ESP32C6_UART */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the UARTs was performed in
   * esp32c6_lowsetup
   */

  /* Disable all UARTS interrupts */

#ifdef TTYS0_DEV
  esp32c6_lowputc_disable_all_uart_int(TTYS0_DEV.priv, NULL);
#endif

#ifdef TTYS1_DEV
  esp32c6_lowputc_disable_all_uart_int(TTYS1_DEV.priv, NULL);
#endif

  /* Configure console in early step.
   * Setup for other serials will be perfomed when the serial driver is
   * open.
   */

#ifdef CONSOLE_UART
  esp32c6_setup(&CONSOLE_DEV);
#endif
}

#endif /* USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif

#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef CONSOLE_UART
  uint32_t int_status;

  esp32c6_lowputc_disable_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);

#ifdef CONSOLE_UART
  esp32c6_lowputc_restore_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif
  return ch;
}

#endif /* HAVE_UART_DEVICE */

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef CONSOLE_UART
  uint32_t int_status;

  esp32c6_lowputc_disable_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);

#ifdef CONSOLE_UART
  esp32c6_lowputc_restore_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
