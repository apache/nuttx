/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_serial.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "hardware/esp32c3_uart.h"
#include "riscv_internal.h"
#include "riscv_arch.h"
#include "chip.h"
#include "esp32c3_lowputc.h"
#include "esp32c3_config.h"
#include "esp32c3_irq.h"

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

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0_dev     /* UART0 is console */
#    define TTYS0_DEV       g_uart0_dev     /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
# elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1_dev  /* UART1 is console */
#    define TTYS0_DEV           g_uart1_dev  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  endif /* CONFIG_UART0_SERIAL_CONSOLE */
#else /* No console */
#  undef  CONSOLE_DEV
#  if defined(CONFIG_ESP32C3_UART0)
#    define TTYS0_DEV           g_uart0_dev  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_ESP32C3_UART1)
#    define TTYS0_DEV           g_uart1_dev  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  endif
#endif /* HAVE_SERIAL_CONSOLE */

/* Pick ttys1 */

#if defined(CONFIG_ESP32C3_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0_dev  /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_ESP32C3_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1_dev  /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#endif

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

static int  esp32c3_setup(struct uart_dev_s *dev);
static void esp32c3_shutdown(struct uart_dev_s *dev);
static int  esp32c3_attach(struct uart_dev_s *dev);
static void esp32c3_detach(struct uart_dev_s *dev);
static void esp32c3_txint(struct uart_dev_s *dev, bool enable);
static void esp32c3_rxint(struct uart_dev_s *dev, bool enable);
static bool esp32c3_rxavailable(struct uart_dev_s *dev);
static bool esp32c3_txready(struct uart_dev_s *dev);
static bool esp32c3_txempty(struct uart_dev_s *dev);
static void esp32c3_send(struct uart_dev_s *dev, int ch);
static int  esp32c3_receive(struct uart_dev_s *dev, unsigned int *status);
static int  esp32c3_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Operations */

static struct uart_ops_s g_uart_ops =
{
    .setup       = esp32c3_setup,
    .shutdown    = esp32c3_shutdown,
    .attach      = esp32c3_attach,
    .detach      = esp32c3_detach,
    .txint       = esp32c3_txint,
    .rxint       = esp32c3_rxint,
    .rxavailable = esp32c3_rxavailable,
    .txready     = esp32c3_txready,
    .txempty     = esp32c3_txempty,
    .send        = esp32c3_send,
    .receive     = esp32c3_receive,
    .ioctl       = esp32c3_ioctl,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rxflowcontrol = NULL,
#endif
};

/* UART 0 */

#ifdef CONFIG_ESP32C3_UART0

static char g_uart0_rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0_txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct esp32c3_uart_s g_uart0_config =
{
  .base = REG_UART_BASE(0),
  .periph = ESP32C3_PERIPH_UART0,
  .id = 0,
  .irq = ESP32C3_IRQ_UART0,
  .baud = CONFIG_UART0_BAUD,
  .bits = CONFIG_UART0_BITS,
  .parity = CONFIG_UART0_PARITY,
  .stop_b2 =  CONFIG_UART0_2STOP,
  .int_pri = 1
};

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

#ifdef CONFIG_ESP32C3_UART1

static char g_uart1_rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1_txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct esp32c3_uart_s g_uart1_config =
{
  .base = REG_UART_BASE(1),
  .periph = ESP32C3_PERIPH_UART1,
  .id = 1,
  .irq = ESP32C3_IRQ_UART1,
  .baud = CONFIG_UART1_BAUD,
  .bits = CONFIG_UART1_BITS,
  .parity = CONFIG_UART1_PARITY,
  .stop_b2 =  CONFIG_UART1_2STOP,
  .int_pri = 1
};

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int uart_handler(int irq, FAR void *context, FAR void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct esp32c3_uart_s *priv = dev->priv;
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
 * Name: esp32c3_setup
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

static int esp32c3_setup(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: esp32c3_shutdown
 *
 * Description:
 * Disable the UART.  This method is called when the serial port is closed.
 * This method reverses the operation the setup method.  NOTE that the serial
 * console is never shutdown.
 *
 ****************************************************************************/

static void esp32c3_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: esp32c3_attach
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

static int esp32c3_attach(struct uart_dev_s *dev)
{
  struct esp32c3_uart_s *priv = dev->priv;
  int ret;

  /* Try to attach the IRQ to a CPU int */

  priv->cpuint = esp32c3_request_irq(priv->periph, priv->int_pri,
                                     ESP32C3_INT_LEVEL);
  if (priv->cpuint < 0)
    {
      return priv->cpuint;
    }

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, uart_handler, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->cpuint);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void esp32c3_detach(struct uart_dev_s *dev)
{
  struct esp32c3_uart_s *priv = dev->priv;

  up_disable_irq(priv->cpuint);
  irq_detach(priv->irq);
  esp32c3_free_cpuint(priv->periph);
}

/****************************************************************************
 * Name: esp32c3_txint
 *
 * Description:
 * Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void esp32c3_txint(struct uart_dev_s *dev, bool enable)
{
  struct esp32c3_uart_s *priv = dev->priv;
  irqstate_t flags;
  uint32_t ints_mask = UART_TXFIFO_EMPTY_INT_ENA_M | UART_TX_DONE_INT_ENA_M;

  flags = enter_critical_section();

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

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32c3_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void esp32c3_rxint(struct uart_dev_s *dev, bool enable)
{
  struct esp32c3_uart_s *priv = dev->priv;
  irqstate_t flags;
  uint32_t ints_mask = UART_RXFIFO_TOUT_INT_ENA_M |
                       UART_RXFIFO_FULL_INT_ENA_M;

  flags = enter_critical_section();

  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      modifyreg32(UART_INT_ENA_REG(priv->id), ints_mask, ints_mask);
#endif
    }
  else
    {
      /* Disable the RX interrupts */

      modifyreg32(UART_INT_ENA_REG(priv->id), ints_mask, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32c3_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool esp32c3_rxavailable(struct uart_dev_s *dev)
{
  struct esp32c3_uart_s *priv = dev->priv;
  uint32_t status_reg;
  uint32_t bytes;

  status_reg = getreg32(UART_STATUS_REG(priv->id));
  bytes = status_reg & UART_RXFIFO_CNT_M;

  return (bytes > 0) ? true : false;
}

/****************************************************************************
 * Name: esp32c3_txready
 *
 * Description:
 * Return true if the tranmsit hardware is ready to send another byte.  This
 * is used to determine if send() method can be called.
 *
 ****************************************************************************/

static bool esp32c3_txready(struct uart_dev_s *dev)
{
  return (esp32c3_lowputc_is_tx_fifo_full(dev->priv)) ? false : true;
}

/****************************************************************************
 * Name: esp32c3_txempty
 *
 * Description:
 * Return true if all characters have been sent.  If for example, the UART
 * hardware implements FIFOs, then this would mean the transmit FIFO is
 * empty.  This method is called when the driver needs to make sure that
 * all characters are "drained" from the TX hardware.
 *
 ****************************************************************************/

static bool esp32c3_txempty(struct uart_dev_s *dev)
{
  uint32_t reg;
  struct esp32c3_uart_s *priv = dev->priv;

  reg = getreg32(UART_INT_RAW_REG(priv->id));
  reg = reg & UART_TXFIFO_EMPTY_INT_RAW_M;

  return (reg > 0) ? true : false;
}

/****************************************************************************
 * Name: esp32c3_shutdown
 *
 * Description:
 * Disable the UART.  This method is called when the serial port is closed.
 * This method reverses the operation the setup method.  NOTE that the serial
 * console is never shutdown.
 *
 ****************************************************************************/

static void esp32c3_send(struct uart_dev_s *dev, int ch)
{
  /* Then send the character */

  esp32c3_lowputc_send_byte(dev->priv, ch);
}

/****************************************************************************
 * Name: esp32c3_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int esp32c3_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint32_t rx_fifo;
  struct esp32c3_uart_s *priv = dev->priv;

  rx_fifo = getreg32(UART_FIFO_REG(priv->id));
  rx_fifo = rx_fifo & UART_RXFIFO_RD_BYTE_M;

  return (int)rx_fifo;
}

static int esp32c3_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

/* TODO */

void up_earlyserialinit(void)
{
  /* I've been looking at others chips/arches and I noticed
   * that <chips>_lowsetup performs almost the same of this func and it's
   * called earlier than this one in <chip>_start
   * So, I am not sure what to do here
   */
}

#endif /* USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

/* TODO */

void up_serialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* At least one UART char driver will logically be registered */

  uart_register("/dev/ttyS0", &TTYS0_DEV);

#ifdef	TTYS1_DEV
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

/* TODO - To finish later with interrupt */

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE

  /* TODO disable uart ints */

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);

  /* TODO restore ints */
#endif
  return ch;
}

#else /* HAVE_UART_DEVICE */

/****************************************************************************
 * Name: up_earlyserialinit, up_serialinit, and up_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs will be used if all UARTs are
 *   disabled.  In that case, the logic in common/up_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
}

void up_serialinit(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
#else /* USE_SERIALDRIVER */

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of up_earlyserialinit(), up_serialinit(), and
 * up_putc().
 */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

/* TODO - Finish it disabling interrupt and restoring it later */

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
