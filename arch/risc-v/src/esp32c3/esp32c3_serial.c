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
#include <nuttx/fs/ioctl.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include "riscv_internal.h"
#include "riscv_arch.h"
#include "chip.h"

#include "hardware/esp32c3_uart.h"
#include "hardware/esp32c3_system.h"

#include "esp32c3_config.h"
#include "esp32c3_irq.h"
#include "esp32c3_lowputc.h"

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
  .periph = ESP32C3_PERIPH_UART0,
  .id = 0,
  .cpuint = -ENOMEM,
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
  .periph = ESP32C3_PERIPH_UART1,
  .id = 1,
  .cpuint = -ENOMEM,
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
 *   interrupt is received on the 'irq'  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
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
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Zero (OK) is returned.
 *
 ****************************************************************************/

static int esp32c3_setup(struct uart_dev_s *dev)
{
  struct esp32c3_uart_s *priv = dev->priv;

  /* Initialize UART module */

  /* Configure the UART Baud Rate */

  esp32c3_lowputc_baud(priv);

  /* Set a mode */

  esp32c3_lowputc_normal_mode(priv);

  /* Parity */

  esp32c3_lowputc_parity(priv);

  /* Data Frame size */

  esp32c3_lowputc_data_length(priv);

  /* Stop bit */

  esp32c3_lowputc_stop_length(priv);

  /* No Tx idle interval */

  esp32c3_lowputc_set_tx_idle_time(priv, 0);

  /* Set pins */

  esp32c3_lowputc_config_pins(priv);

  /* Enable cores */

  esp32c3_lowputc_enable_sclk(priv);

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
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *
 ****************************************************************************/

static void esp32c3_shutdown(struct uart_dev_s *dev)
{
  struct esp32c3_uart_s *priv = dev->priv;

  /* Clear FIFOs */

  esp32c3_lowputc_rst_txfifo(priv);
  esp32c3_lowputc_rst_rxfifo(priv);

  /* Disable ints */

  esp32c3_lowputc_disable_all_uart_int(priv, NULL);

  /* Back pins to normal */

  esp32c3_lowputc_restore_pins(priv);
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
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

static int esp32c3_attach(struct uart_dev_s *dev)
{
  struct esp32c3_uart_s *priv = dev->priv;
  int ret;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

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
  else
    {
      up_disable_irq(priv->cpuint);
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
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *
 ****************************************************************************/

static void esp32c3_detach(struct uart_dev_s *dev)
{
  struct esp32c3_uart_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  up_disable_irq(priv->cpuint);
  irq_detach(priv->irq);
  esp32c3_free_cpuint(priv->periph);
  priv->cpuint = -ENOMEM;
}

/****************************************************************************
 * Name: esp32c3_txint
 *
 * Description:
 *    Enable or disable TX interrupts.
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *   enable     -  If true enables the TX interrupt, if false disables it.
 *
 ****************************************************************************/

static void esp32c3_txint(struct uart_dev_s *dev, bool enable)
{
  struct esp32c3_uart_s *priv = dev->priv;
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
 * Name: esp32c3_rxint
 *
 * Description:
 *   Enable or disable RX interrupts.
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *   enable     -  If true enables the RX interrupt, if false disables it.
 *
 ****************************************************************************/

static void esp32c3_rxint(struct uart_dev_s *dev, bool enable)
{
  struct esp32c3_uart_s *priv = dev->priv;
  uint32_t ints_mask = UART_RXFIFO_TOUT_INT_ENA_M |
                       UART_RXFIFO_FULL_INT_ENA_M;

  if (enable)
    {
      /* Receive an interrupt when there is anything in the Rx data register
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
 * Name: esp32c3_rxavailable
 *
 * Description:
 *   Check if there is any data available to be read.
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Return true if the RX FIFO is not empty and false if RX FIFO is empty.
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
 *    Check if the transmit hardware is ready to send another byte.
 *    This is used to determine if send() method can be called.
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Return true if the transmit hardware is ready to send another byte,
 *   false otherwise.
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
 *    Verify if all characters have been sent. If for example, the UART
 *    hardware implements FIFOs, then this would mean the transmit FIFO is
 *    empty. This method is called when the driver needs to make sure that
 *    all characters are "drained" from the TX hardware.
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Return true if the TX FIFO is empty, false if it is not.
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
 * Name: esp32c3_send
 *
 * Description:
 *    Send a unique character
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *   ch         -  Byte to be sent.
 *
 ****************************************************************************/

static void esp32c3_send(struct uart_dev_s *dev, int ch)
{
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
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *   status     -  Pointer to a variable to store eventual error bits.
 *
 * Returned Values:
 *   Return the byte read from the RX FIFO.
 *
 ****************************************************************************/

static int esp32c3_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint32_t rx_fifo;
  struct esp32c3_uart_s *priv = dev->priv;

  rx_fifo = getreg32(UART_FIFO_REG(priv->id));
  rx_fifo = rx_fifo & UART_RXFIFO_RD_BYTE_M;

  /* Since we don't have error bits associated with receipt, we set zero */

  *status = 0;

  return (int)rx_fifo;
}

/****************************************************************************
 * Name: esp32c3_ioctl
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

static int esp32c3_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  /* Get access to the internal instance of the driver through the file
   *  pointer.
   */

#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int ret = OK;

  /* Run the requested ioctl command. */

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT

    /* Get the internal driver data structure for debug purposes. */

    case TIOCSERGSTRUCT:
      {
         struct esp32c3_uart_s *user = (struct esp32c3_uart_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev->priv, sizeof(struct esp32c3_uart_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS

    /* Fill a termios structure with the required information. */

    case TCGETS:
      {
        struct termios  *termiosp    = (struct termios *)arg;
        struct esp32c3_uart_s *priv  = (struct esp32c3_uart_s *)dev->priv;
        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return parity (0 = no parity, 1 = odd parity, 2 = even parity). */

        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                            ((priv->parity == 1) ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= (priv->stop_b2) ? CSTOPB : 0;

        /* Set the baud rate in ther termiosp using the
         * cfsetispeed interface.
         */

        cfsetispeed(termiosp, priv->baud);

        /* Return number of bits. */

        switch (priv->bits)
          {
          case 5:
            termiosp->c_cflag |= CS5;
            break;

          case 6:
            termiosp->c_cflag |= CS6;
            break;

          case 7:
            termiosp->c_cflag |= CS7;
            break;

          default:
          case 8:
            termiosp->c_cflag |= CS8;
            break;
          }
      }
      break;

    case TCSETS:
      {
        struct termios  *termiosp    = (struct termios *)arg;
        struct esp32c3_uart_s *priv  = (struct esp32c3_uart_s *)dev->priv;
        uint32_t baud;
        uint32_t current_int_sts;
        uint8_t  parity;
        uint8_t  bits;
        uint8_t  stop2;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Get the target baud rate to change. */

        baud = cfgetispeed(termiosp);

        /* Decode number of bits. */

        switch (termiosp->c_cflag & CSIZE)
          {
          case CS5:
            bits = 5;
            break;

          case CS6:
            bits = 6;
            break;

          case CS7:
            bits = 7;
            break;

          case CS8:
            bits = 8;
            break;

          default:
            ret = -EINVAL;
            break;
          }

        /* Decode parity. */

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        /* Decode stop bits. */

        stop2 = (termiosp->c_cflag & CSTOPB) ? 1 : 0;

        /* Verify that all settings are valid before
         * performing the changes.
         */

        if (ret == OK)
          {
            /* Fill the private struct fields. */

            priv->baud      = baud;
            priv->parity    = parity;
            priv->bits      = bits;
            priv->stop_b2   = stop2;

            /* Effect the changes immediately - note that we do not
             * implement TCSADRAIN or TCSAFLUSH, only TCSANOW option.
             * See nuttx/libs/libc/termios/lib_tcsetattr.c
             */

            esp32c3_lowputc_disable_all_uart_int(priv, &current_int_sts);
            ret = esp32c3_setup(dev);

            /* Restore the interrupt state */

            esp32c3_lowputc_restore_all_uart_int(priv, &current_int_sts);
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

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

/* TODO */

void riscv_earlyserialinit(void)
{
  /* I've been looking at others chips/arches and I noticed
   * that <chips>_lowsetup performs almost the same of this func and it's
   * called earlier than this one in <chip>_start
   * So, I am not sure what to do here
   */
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

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  uint32_t int_status;

  esp32c3_lowputc_disable_all_uart_int(CONSOLE_DEV.priv, &int_status);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
  esp32c3_lowputc_restore_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif
  return ch;
}

#else /* HAVE_UART_DEVICE */

/****************************************************************************
 * Name: riscv_earlyserialinit, riscv_serialinit, and up_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs will be used if all UARTs are
 *   disabled.  In that case, the logic in common/up_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
}

void riscv_serialinit(void)
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
 * stub implementations of riscv_earlyserialinit(), riscv_serialinit(), and
 * up_putc().
 */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  uint32_t int_status;

  esp32c3_lowputc_disable_all_uart_int(CONSOLE_DEV.priv, &int_status);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
  esp32c3_lowputc_restore_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
