/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_serial.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>

#include "xtensa.h"
#include "esp32s3_config.h"
#include "esp32s3_irq.h"
#include "esp32s3_lowputc.h"
#include "hardware/esp32s3_uart.h"
#include "hardware/esp32s3_system.h"

#ifdef CONFIG_ESP32S3_USBSERIAL
#  include "esp32s3_usbserial.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The console is enabled, and it's not the syslog device, so, it should be a
 * serial device.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1? */

/* First pick the console and ttys0.
 * Console can be UART0 or UART1, but will always be ttys0.
 */

/* In case a UART was assigned to be the console and the corresponding
 * peripheral was also selected.
 */

#ifdef CONSOLE_UART
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0_dev     /* UART0 is console */
#    define TTYS0_DEV           g_uart0_dev     /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
# elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1_dev  /* UART1 is console */
#    define TTYS0_DEV           g_uart1_dev  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  endif /* CONFIG_UART0_SERIAL_CONSOLE */
#else /* No UART console */
#  undef  CONSOLE_DEV
#  if defined(CONFIG_ESP32S3_UART0)
#    define TTYS0_DEV           g_uart0_dev  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_ESP32S3_UART1)
#    define TTYS0_DEV           g_uart1_dev  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  endif
#endif /* CONSOLE_UART */

#ifdef CONFIG_ESP32S3_USBSERIAL
#  define CONSOLE_DEV           g_uart_usbserial
#  define TTYACM0_DEV           g_uart_usbserial
#endif

/* Pick ttyS1 */

#if defined(CONFIG_ESP32S3_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0_dev  /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_ESP32S3_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1_dev  /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#endif

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_UART

/* Serial driver methods */

static int  esp32s3_setup(struct uart_dev_s *dev);
static void esp32s3_shutdown(struct uart_dev_s *dev);
static int  esp32s3_attach(struct uart_dev_s *dev);
static void esp32s3_detach(struct uart_dev_s *dev);
static void esp32s3_txint(struct uart_dev_s *dev, bool enable);
static void esp32s3_rxint(struct uart_dev_s *dev, bool enable);
static bool esp32s3_rxavailable(struct uart_dev_s *dev);
static bool esp32s3_txready(struct uart_dev_s *dev);
static bool esp32s3_txempty(struct uart_dev_s *dev);
static void esp32s3_send(struct uart_dev_s *dev, int ch);
static int  esp32s3_receive(struct uart_dev_s *dev, unsigned int *status);
static int  esp32s3_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool esp32s3_rxflowcontrol(struct uart_dev_s *dev,
                                  unsigned int nbuffered, bool upper);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_UART

/* Operations */

static struct uart_ops_s g_uart_ops =
{
  .setup         = esp32s3_setup,
  .shutdown      = esp32s3_shutdown,
  .attach        = esp32s3_attach,
  .detach        = esp32s3_detach,
  .ioctl         = esp32s3_ioctl,
  .receive       = esp32s3_receive,
  .rxint         = esp32s3_rxint,
  .rxavailable   = esp32s3_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = esp32s3_rxflowcontrol,
#endif
  .send          = esp32s3_send,
  .txint         = esp32s3_txint,
  .txready       = esp32s3_txready,
  .txempty       = esp32s3_txempty
};

/* UART 0 */

#ifdef CONFIG_ESP32S3_UART0

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

  .ops  = &g_uart_ops,
  .priv = &g_uart0_config
};

#endif

/* UART 1 */

#ifdef CONFIG_ESP32S3_UART1

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

  .ops  = &g_uart_ops,
  .priv = &g_uart1_config
};

#endif

#endif /* CONFIG_ESP32S3_UART */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_UART

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

static int uart_handler(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct esp32s3_uart_s *priv = dev->priv;
  uint32_t tx_mask = UART_TXFIFO_EMPTY_INT_ST_M | UART_TX_DONE_INT_ST_M;
  uint32_t rx_mask = UART_RXFIFO_TOUT_INT_ST_M | UART_RXFIFO_FULL_INT_ST_M;
  uint32_t int_status;

  int_status = getreg32(UART_INT_ST_REG(priv->id));

  /* Tx fifo empty interrupt or UART tx done int */

  if ((int_status & tx_mask) != 0)
    {
      uart_xmitchars(dev);
      modifyreg32(UART_INT_CLR_REG(priv->id), tx_mask, tx_mask);
    }

  /* Rx fifo timeout interrupt or rx fifo full interrupt */

  if ((int_status & rx_mask) != 0)
    {
      uart_recvchars(dev);
      modifyreg32(UART_INT_CLR_REG(priv->id), rx_mask, rx_mask);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_setup
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

static int esp32s3_setup(struct uart_dev_s *dev)
{
  struct esp32s3_uart_s *priv = dev->priv;

  /* Initialize UART module */

  /* Discard corrupt RX data and
   * disable UART memory clock gate enable signal.
   */

  modifyreg32(UART_CONF0_REG(priv->id), UART_ERR_WR_MASK_M |
              UART_MEM_CLK_EN_M, UART_ERR_WR_MASK_M);

  /* Define 0 as the threshold that means TX FIFO buffer is empty. */

  modifyreg32(UART_CONF1_REG(priv->id), UART_TXFIFO_EMPTY_THRHD_M, 0);

  /* Define a threshold to trigger an RX FIFO FULL interrupt.
   * Define just one byte to read data immediately.
   */

  modifyreg32(UART_CONF1_REG(priv->id), UART_RXFIFO_FULL_THRHD_M,
              1 << UART_RXFIFO_FULL_THRHD_S);

  /* Define the maximum FIFO size for RX and TX FIFO.
   * That means, 1 block = 128 bytes.
   * As a consequence, software serial FIFO can unload the bytes and
   * not wait too much on polling activity.
   */

  modifyreg32(UART_MEM_CONF_REG(priv->id), UART_TX_SIZE_M | UART_RX_SIZE_M,
              (1 << UART_TX_SIZE_S) | (1 << UART_RX_SIZE_S));

  /* Configure the UART Baud Rate */

  esp32s3_lowputc_baud(priv);

  /* Set a mode */

  esp32s3_lowputc_normal_mode(priv);

  /* Parity */

  esp32s3_lowputc_parity(priv);

  /* Data Frame size */

  esp32s3_lowputc_data_length(priv);

  /* Stop bit */

  esp32s3_lowputc_stop_length(priv);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  /* Configure the input flow control */

  if (priv->iflow)
    {
      /* Enable input flow control and set the RX FIFO threshold
       * to assert the RTS line to half the RX FIFO buffer.
       * It will then save some space on the hardware fifo to
       * remaining bytes that may arrive after RTS be asserted
       * and before the transmitter stops sending data.
       */

      esp32s3_lowputc_set_iflow(priv, (uint8_t)(UART_RX_FIFO_SIZE / 2),
                                true);
    }
  else
    {
      /* Just disable input flow control, threshold parameter
       * will be discarded.
       */

      esp32s3_lowputc_set_iflow(priv, 0 , false);
    }

#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  /* Configure the ouput flow control */

  if (priv->oflow)
    {
      esp32s3_lowputc_set_oflow(priv, true);
    }
  else
    {
      esp32s3_lowputc_set_oflow(priv, false);
    }
#endif

  /* No Tx idle interval */

  esp32s3_lowputc_set_tx_idle_time(priv, 0);

  /* Enable cores */

  esp32s3_lowputc_enable_sclk(priv);

  /* Clear FIFOs */

  esp32s3_lowputc_rst_txfifo(priv);
  esp32s3_lowputc_rst_rxfifo(priv);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_shutdown
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

static void esp32s3_shutdown(struct uart_dev_s *dev)
{
  struct esp32s3_uart_s *priv = dev->priv;

  /* Disable ints */

  esp32s3_lowputc_disable_all_uart_int(priv, NULL);
}

/****************************************************************************
 * Name: esp32s3_attach
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

static int esp32s3_attach(struct uart_dev_s *dev)
{
  struct esp32s3_uart_s *priv = dev->priv;
  int ret;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

  /* Set up to receive peripheral interrupts on the current CPU */

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32s3_setup_irq(priv->cpu, priv->periph, priv->int_pri,
                                   ESP32S3_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type */

      return priv->cpuint;
    }

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, uart_handler, dev);
  if (ret == OK)
    {
      /* Enable the CPU interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32s3_detach
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

static void esp32s3_detach(struct uart_dev_s *dev)
{
  struct esp32s3_uart_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  /* Disable and detach the CPU interrupt */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  /* Disassociate the peripheral interrupt from the CPU interrupt */

  esp32s3_teardown_irq(priv->cpu, priv->periph, priv->cpuint);
  priv->cpuint = -ENOMEM;
}

/****************************************************************************
 * Name: esp32s3_txint
 *
 * Description:
 *    Enable or disable TX interrupts.
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *   enable     -  If true enables the TX interrupt, if false disables it.
 *
 ****************************************************************************/

static void esp32s3_txint(struct uart_dev_s *dev, bool enable)
{
  struct esp32s3_uart_s *priv = dev->priv;
  uint32_t ints_mask = UART_TXFIFO_EMPTY_INT_ENA_M | UART_TX_DONE_INT_ENA_M;
  irqstate_t flags = spin_lock_irqsave(&priv->lock);

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

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32s3_rxint
 *
 * Description:
 *   Enable or disable RX interrupts.
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *   enable     -  If true enables the RX interrupt, if false disables it.
 *
 ****************************************************************************/

static void esp32s3_rxint(struct uart_dev_s *dev, bool enable)
{
  struct esp32s3_uart_s *priv = dev->priv;
  uint32_t ints_mask = UART_RXFIFO_TOUT_INT_ENA_M |
                       UART_RXFIFO_FULL_INT_ENA_M;
  irqstate_t flags = spin_lock_irqsave(&priv->lock);

  if (enable)
    {
      /* Receive an interrupt when there is anything in the RX data register
       * (or an RX timeout occurs).
       * NOTE: RX timeout feature needs to be enabled.
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

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32s3_rxavailable
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

static bool esp32s3_rxavailable(struct uart_dev_s *dev)
{
  struct esp32s3_uart_s *priv = dev->priv;
  uint32_t status_reg;
  uint32_t bytes;

  status_reg = getreg32(UART_STATUS_REG(priv->id));
  bytes = status_reg & UART_RXFIFO_CNT_M;

  return (bytes > 0);
}

/****************************************************************************
 * Name: esp32s3_txready
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

static bool esp32s3_txready(struct uart_dev_s *dev)
{
  return !esp32s3_lowputc_is_tx_fifo_full(dev->priv);
}

/****************************************************************************
 * Name: esp32s3_txempty
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

static bool esp32s3_txempty(struct uart_dev_s *dev)
{
  uint32_t reg;
  struct esp32s3_uart_s *priv = dev->priv;

  reg = getreg32(UART_INT_RAW_REG(priv->id));
  reg = reg & UART_TXFIFO_EMPTY_INT_RAW_M;

  return (reg > 0);
}

/****************************************************************************
 * Name: esp32s3_send
 *
 * Description:
 *    Send a unique character
 *
 * Parameters:
 *   dev        -  Pointer to the serial driver struct.
 *   ch         -  Byte to be sent.
 *
 ****************************************************************************/

static void esp32s3_send(struct uart_dev_s *dev, int ch)
{
  esp32s3_lowputc_send_byte(dev->priv, (char)ch);
}

/****************************************************************************
 * Name: esp32s3_receive
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

static int esp32s3_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint32_t rx_fifo;
  struct esp32s3_uart_s *priv = dev->priv;

  rx_fifo = getreg32(UART_FIFO_REG(priv->id));
  rx_fifo = rx_fifo & UART_RXFIFO_RD_BYTE_M;

  /* Since we don't have error bits associated with receipt, we set zero */

  *status = 0;

  return (int)rx_fifo;
}

/****************************************************************************
 * Name: esp32s3_ioctl
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

static int esp32s3_ioctl(struct file *filep, int cmd, unsigned long arg)
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
         struct esp32s3_uart_s *user = (struct esp32s3_uart_s *)arg;
         if (user == NULL)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev->priv, sizeof(struct esp32s3_uart_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS

    /* Fill a termios structure with the required information. */

    case TCGETS:
      {
        struct termios  *termiosp   = (struct termios *)arg;
        struct esp32s3_uart_s *priv = (struct esp32s3_uart_s *)dev->priv;
        if (termiosp == NULL)
          {
            ret = -EINVAL;
            break;
          }

        /* Return parity (0 = no parity, 1 = odd parity, 2 = even parity). */

        termiosp->c_cflag = (priv->parity != 0 ? PARENB : 0) |
                            (priv->parity == 1 ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= priv->stop_b2 != 0 ? CSTOPB : 0;

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        termiosp->c_cflag |=  priv->oflow != 0 ? CCTS_OFLOW : 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        termiosp->c_cflag |=  priv->iflow != 0 ? CRTS_IFLOW : 0;
#endif

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
        struct termios  *termiosp   = (struct termios *)arg;
        struct esp32s3_uart_s *priv = (struct esp32s3_uart_s *)dev->priv;
        uint32_t baud;
        uint32_t current_int_sts;
        uint8_t  parity;
        uint8_t  bits;
        uint8_t  stop2;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        bool iflow;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
        bool oflow;
#endif

        if (termiosp == NULL)
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
            parity = (termiosp->c_cflag & PARODD) != 0 ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        /* Decode stop bits. */

        stop2 = (termiosp->c_cflag & CSTOPB) != 0 ? 1 : 0;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
        iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
        oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif

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
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            priv->iflow     = iflow;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow     = oflow;
#endif

            /* Effect the changes immediately - note that we do not
             * implement TCSADRAIN or TCSAFLUSH, only TCSANOW option.
             * See nuttx/libs/libc/termios/lib_tcsetattr.c
             */

            esp32s3_lowputc_disable_all_uart_int(priv, &current_int_sts);
            ret = esp32s3_setup(dev);

            /* Restore the interrupt state */

            esp32s3_lowputc_restore_all_uart_int(priv, &current_int_sts);
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
 * Name: esp32s3_rxflowcontrol
 *
 * Description:
 *   Called when upper half RX buffer is full (or exceeds configured
 *   watermark levels if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data.
 *   NOTE: ESP32-S3 has a hardware RX FIFO threshold mechanism to control
 *   RTS line and to stop receiving data. This is very similar to the concept
 *   behind upper watermark level. The hardware threshold is used here
 *   to control the RTS line. When setting the threshold to zero, RTS will
 *   immediately be asserted. If nbuffered = 0 or the lower watermark is
 *   crossed and the serial driver decides to disable RX flow control, the
 *   threshold will be changed to UART_RX_FLOW_THRHD_VALUE, which is almost
 *   half the HW RX FIFO capacity. It keeps some space to keep the data
 *   received between the RTS assertion and the stop by the sender.
 *
 * Input Parameters:
 *   dev       - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool esp32s3_rxflowcontrol(struct uart_dev_s *dev,
                                  unsigned int nbuffered, bool upper)
{
  bool ret = false;
  struct esp32s3_uart_s *priv = dev->priv;
  if (priv->iflow)
    {
      if (nbuffered == 0 || upper == false)
        {
          /* Empty buffer, RTS should be de-asserted and logic in above
           * layers should re-enable RX interrupt.
           */

          esp32s3_lowputc_set_iflow(priv, (uint8_t)(UART_RX_FIFO_SIZE / 2),
                                    true);
          esp32s3_rxint(dev, true);
          ret = false;
        }
      else
        {
          /* If the RX buffer is not zero and watermarks are not enabled,
           * then this function is called to announce RX buffer is full.
           * The first thing it should do is to immediately assert RTS.
           * Software RX FIFO is full, so besides asserting RTS, it's
           * necessary to disable RX interrupts to prevent remaining bytes
           * (that arrive after asserting RTS) to be pushed to the
           * SW RX FIFO.
           */

          esp32s3_lowputc_set_iflow(priv, 0 , true);
          esp32s3_rxint(dev, false);
          ret = true;
        }
    }

  return ret;
}
#endif
#endif /* CONFIG_ESP32S3_UART */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: xtensa_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before xtensa_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void xtensa_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the UARTs was performed in
   * esp32s3_lowsetup
   */

  /* Disable all UARTS interrupts */

#ifdef TTYS0_DEV
  esp32s3_lowputc_disable_all_uart_int(TTYS0_DEV.priv, NULL);
#endif

#ifdef TTYS1_DEV
  esp32s3_lowputc_disable_all_uart_int(TTYS1_DEV.priv, NULL);
#endif

  /* Configure console in early step.
   * Setup for other serials will be perfomed when the serial driver is
   * open.
   */

#ifdef CONSOLE_UART
  esp32s3_setup(&CONSOLE_DEV);
#endif
}

#endif /* USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: xtensa_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that xtensa_earlyserialinit was called previously.
 *
 ****************************************************************************/

void xtensa_serialinit(void)
{
#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif

#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif

#ifdef CONFIG_ESP32S3_USBSERIAL
  uart_register("/dev/ttyACM0", &TTYACM0_DEV);
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

  esp32s3_lowputc_disable_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc((char)ch);

#ifdef CONSOLE_UART
  esp32s3_lowputc_restore_all_uart_int(CONSOLE_DEV.priv, &int_status);
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

  esp32s3_lowputc_disable_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);

#ifdef CONSOLE_UART
  esp32s3_lowputc_restore_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
