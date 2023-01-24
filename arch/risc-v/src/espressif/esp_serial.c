/****************************************************************************
 * arch/risc-v/src/espressif/esp_serial.c
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

#include "riscv_internal.h"
#include "chip.h"
#include "esp_config.h"
#include "esp_irq.h"
#include "esp_lowputc.h"

#include "clk_tree.h"
#include "hal/uart_hal.h"
#include "soc/clk_tree_defs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The console is enabled and it's not the syslog device, so it should be a
 * serial device.
 */

#ifdef USE_SERIALDRIVER

/* Which UART will be designated to ttyS0/console and which one will be
 * designated to ttyS1?
 */

/* First pick the console and ttyS0.
 * Console can use either UART0 or UART1, but will always be ttyS0.
 */

/* In case a UART was assigned to be the console and the corresponding
 * peripheral was also selected.
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
#  if defined(CONFIG_ESPRESSIF_UART0)
#    define TTYS0_DEV           g_uart0_dev  /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_ESPRESSIF_UART1)
#    define TTYS0_DEV           g_uart1_dev  /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  endif
#endif /* CONSOLE_UART */

/* Pick ttyS1 */

#if defined(CONFIG_ESPRESSIF_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0_dev  /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_ESPRESSIF_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1_dev  /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#endif

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_UART

/* Serial driver methods */

static int  esp_setup(uart_dev_t *dev);
static void esp_shutdown(uart_dev_t *dev);
static int  esp_attach(uart_dev_t *dev);
static void esp_detach(uart_dev_t *dev);
static void esp_txint(uart_dev_t *dev, bool enable);
static void esp_rxint(uart_dev_t *dev, bool enable);
static bool esp_rxavailable(uart_dev_t *dev);
static bool esp_txready(uart_dev_t *dev);
static bool esp_txempty(uart_dev_t *dev);
static void esp_send(uart_dev_t *dev, int ch);
static int  esp_receive(uart_dev_t *dev, unsigned int *status);
static int  esp_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool esp_rxflowcontrol(uart_dev_t *dev,
                              unsigned int nbuffered, bool upper);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_UART

/* Operations */

static struct uart_ops_s g_uart_ops =
{
  .setup         = esp_setup,
  .shutdown      = esp_shutdown,
  .attach        = esp_attach,
  .detach        = esp_detach,
  .txint         = esp_txint,
  .rxint         = esp_rxint,
  .rxavailable   = esp_rxavailable,
  .txready       = esp_txready,
  .txempty       = esp_txempty,
  .send          = esp_send,
  .receive       = esp_receive,
  .ioctl         = esp_ioctl,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = esp_rxflowcontrol
#endif
};

/* UART 0 */

#ifdef CONFIG_ESPRESSIF_UART0

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

#ifdef CONFIG_ESPRESSIF_UART1

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

#endif /* CONFIG_ESPRESSIF_UART */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_UART

/****************************************************************************
 * Name: uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler. It will be invoked when an
 *   interrupt is received on the 'irq'. It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers. The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 * Input Parameters:
 *   irq           - IRQ associated to that interrupt.
 *   context       - Interrupt register state save info.
 *   arg           - A pointer to the argument provided when the interrupt
 *                   was registered.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int uart_handler(int irq, void *context, void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  struct esp_uart_s *priv = dev->priv;
  uint32_t tx_mask = UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_DONE;
  uint32_t rx_mask = UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL;
  uint32_t int_status = uart_hal_get_intsts_mask(priv->hal);

  /* Tx fifo empty interrupt or UART tx done int */

  if ((int_status & tx_mask) != 0)
    {
      uart_xmitchars(dev);
      uart_hal_clr_intsts_mask(priv->hal, tx_mask);
    }

  /* Rx fifo timeout interrupt or rx fifo full interrupt */

  if ((int_status & rx_mask) != 0)
    {
      uart_recvchars(dev);
      uart_hal_clr_intsts_mask(priv->hal, rx_mask);
    }

  return OK;
}

/****************************************************************************
 * Name: set_data_length
 *
 * Description:
 *   Set the data bits length, according to the value in the private driver
 *   struct.
 *
 * Input Parameters:
 *   priv          - Pointer to the private driver struct.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_data_length(const struct esp_uart_s *priv)
{
  uint32_t length = (priv->bits - 5);

  /* If it is the allowed range */

  DEBUGASSERT(length >= UART_DATA_5_BITS && length <= UART_DATA_8_BITS);

  uart_hal_set_data_bit_num(priv->hal, length);
}

/****************************************************************************
 * Name: set_stop_length
 *
 * Description:
 *   Set the stop bits length, according to the value in the private driver
 *   struct.
 *
 * Input Parameters:
 *   priv          - Pointer to the private driver struct.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_stop_length(const struct esp_uart_s *priv)
{
  uart_stop_bits_t stop_bits;

  if (priv->stop_b2)
    {
      stop_bits = UART_STOP_BITS_2;
    }
  else
    {
      stop_bits = UART_STOP_BITS_1;
    }

  uart_hal_set_stop_bits(priv->hal, stop_bits);
}

/****************************************************************************
 * Name: esp_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *   For the serial console, this will occur very early in initialization,
 *   for other serial ports this will occur when the port is first opened.
 *   This setup does not include attaching or enabling interrupts.
 *   That portion of the UART setup is performed when the attach() method
 *   is called.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Zero (OK) is returned.
 *
 ****************************************************************************/

static int esp_setup(uart_dev_t *dev)
{
  struct esp_uart_s *priv = dev->priv;
  uint32_t sclk_freq;

  /* Enable the UART Clock */

  esp_lowputc_enable_sysclk(priv);

  clk_tree_src_get_freq_hz((soc_module_clk_t)UART_SCLK_DEFAULT,
                           CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                           &sclk_freq);

  /* Initialize UART module */

  uart_hal_init(priv->hal, priv->id);
  uart_hal_set_mode(priv->hal, UART_MODE_UART);
  uart_hal_set_sclk(priv->hal, UART_SCLK_DEFAULT);
  uart_hal_set_baudrate(priv->hal, priv->baud, sclk_freq);
  uart_hal_set_parity(priv->hal, priv->parity);
  set_data_length(priv);
  set_stop_length(priv);
  uart_hal_set_tx_idle_num(priv->hal, 0);

  /* Define 0 as the threshold that means TX FIFO buffer is empty. */

  uart_hal_set_txfifo_empty_thr(priv->hal, 10);

  /* Define a threshold to trigger an RX FIFO FULL interrupt.
   * Define just one byte to read data immediately.
   */

  uart_hal_set_rxfifo_full_thr(priv->hal, 120);

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || \
    defined(CONFIG_SERIAL_OFLOWCONTROL)
  uart_hw_flowcontrol_t flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uint32_t rx_thrs = 0;

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

      flow_ctrl |= UART_HW_FLOWCTRL_RTS;
      rx_thrs = SOC_UART_FIFO_LEN / 2;
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  /* Configure the output flow control */

  if (priv->oflow)
    {
      flow_ctrl |= UART_HW_FLOWCTRL_CTS;
    }
#endif

  uart_hal_set_hw_flow_ctrl(priv->hal, flow_ctrl, rx_thrs);
#endif /* CONFIG_SERIAL_IFLOWCONTROL || CONFIG_SERIAL_OFLOWCONTROL */

  /* Clear FIFOs */

  uart_hal_rxfifo_rst(priv->hal);
  uart_hal_txfifo_rst(priv->hal);

  return OK;
}

/****************************************************************************
 * Name: esp_shutdown
 *
 * Description:
 *   Disable the UART. This method is called when the serial port is closed.
 *   This method reverses the operation of the setup method. Note that the
 *   serial console is never shutdown.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_shutdown(uart_dev_t *dev)
{
  struct esp_uart_s *priv = dev->priv;

  /* Disable interrupts */

  esp_lowputc_disable_all_uart_int(priv, NULL);
}

/****************************************************************************
 * Name: esp_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method
 *   is called when the serial port is opened. Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling). The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

static int esp_attach(uart_dev_t *dev)
{
  struct esp_uart_s *priv = dev->priv;
  int ret;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

  /* Set up to receive peripheral interrupts */

  priv->cpuint = esp_setup_irq(priv->source, priv->int_pri,
                               ESP_IRQ_TRIGGER_LEVEL);
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
 * Name: esp_detach
 *
 * Description:
 *   Detach UART interrupts. This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The
 *   exception is the serial console which is never shutdown.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_detach(uart_dev_t *dev)
{
  struct esp_uart_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  /* Disable and detach the CPU interrupt */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  /* Disassociate the peripheral interrupt from the CPU interrupt */

  esp_teardown_irq(priv->source, priv->cpuint);
  priv->cpuint = -ENOMEM;
}

/****************************************************************************
 * Name: esp_txint
 *
 * Description:
 *   Enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *   enable        - If true enables the TX interrupt, if false disables it.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_txint(uart_dev_t *dev, bool enable)
{
  struct esp_uart_s *priv = dev->priv;
  uint32_t ints_mask = UART_INTR_TXFIFO_EMPTY | UART_INTR_TX_DONE;

  if (enable)
    {
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uart_hal_ena_intr_mask(priv->hal, ints_mask);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      uart_hal_disable_intr_mask(priv->hal, ints_mask);
    }
}

/****************************************************************************
 * Name: esp_rxint
 *
 * Description:
 *   Enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *   enable        - If true enables the RX interrupt, if false disables it.
 *
 ****************************************************************************/

static void esp_rxint(uart_dev_t *dev, bool enable)
{
  struct esp_uart_s *priv = dev->priv;
  uint32_t ints_mask = UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL;

  if (enable)
    {
      /* Receive an interrupt when there is anything in the RX data register
       * (or an RX timeout occurs).
       * NOTE: RX timeout feature needs to be enabled.
       */
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uart_hal_set_rx_timeout(priv->hal, 0xa);
      uart_hal_ena_intr_mask(priv->hal, ints_mask);
#endif
    }
  else
    {
      uart_hal_set_rx_timeout(priv->hal, 0);
      uart_hal_disable_intr_mask(priv->hal, ints_mask);
    }
}

/****************************************************************************
 * Name: esp_rxavailable
 *
 * Description:
 *   Check if there is any data available to be read.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Return true if the RX FIFO is not empty and false if RX FIFO is empty.
 *
 ****************************************************************************/

static bool esp_rxavailable(uart_dev_t *dev)
{
  struct esp_uart_s *priv = dev->priv;

  return uart_hal_get_rxfifo_len(priv->hal) > 0;
}

/****************************************************************************
 * Name: esp_txready
 *
 * Description:
 *   Check if the transmit hardware is ready to send another byte.
 *   This is used to determine if send() method can be called.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Return true if the transmit hardware is ready to send another byte,
 *   false otherwise.
 *
 ****************************************************************************/

static bool esp_txready(uart_dev_t *dev)
{
  struct esp_uart_s *priv = dev->priv;

  return uart_hal_get_txfifo_len(priv->hal) > 0;
}

/****************************************************************************
 * Name: esp_txempty
 *
 * Description:
 *   Verify if all characters have been sent. If for example, the UART
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty. This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *
 * Returned Values:
 *   Return true if the TX FIFO is empty, false if it is not.
 *
 ****************************************************************************/

static bool esp_txempty(uart_dev_t *dev)
{
  struct esp_uart_s *priv = dev->priv;

  return priv->hal->dev->int_raw.txfifo_empty != 0;
}

/****************************************************************************
 * Name: esp_send
 *
 * Description:
 *   Send a unique character.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *   ch            - Byte to be sent.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_send(uart_dev_t *dev, int ch)
{
  esp_lowputc_send_byte(dev->priv, ch);
}

/****************************************************************************
 * Name: esp_receive
 *
 * Description:
 *   Called (usually) from the interrupt handler to receive one character
 *   from the UART. Error bits associated with receive are provided in the
 *   return 'status'.
 *
 * Input Parameters:
 *   dev           - Pointer to the serial driver struct.
 *
 * Output Parameters:
 *   status        - Pointer to a variable to store eventual error bits.
 *
 * Returned Values:
 *   Return the byte read from the RX FIFO.
 *
 ****************************************************************************/

static int esp_receive(uart_dev_t *dev, unsigned int *status)
{
  struct esp_uart_s *priv = dev->priv;
  int inout_rd_len = 1;
  uint8_t buf;

  uart_hal_read_rxfifo(priv->hal, &buf, &inout_rd_len);

  /* Since we don't have error bits associated with receive, we set zero */

  *status = 0;

  return (int)buf;
}

/****************************************************************************
 * Name: esp_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *   Here it's employed to implement the TERMIOS ioctls and TIOCSERGSTRUCT.
 *
 * Input Parameters:
 *   filep         - Pointer to a file structure instance.
 *   cmd           - The ioctl command.
 *   arg           - The argument of the ioctl cmd.
 *
 * Returned Value:
 *   Returns a non-negative number on success; a negated errno value is
 *   returned on any failure (see comments ioctl() for a list of appropriate
 *   errno values).
 *
 ****************************************************************************/

static int esp_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  /* Get access to the internal instance of the driver through the file
   * pointer.
   */

#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode *inode = filep->f_inode;
  uart_dev_t *dev     = inode->i_private;
#endif
  int ret = OK;

  /* Run the requested ioctl command. */

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT

    /* Get the internal driver data structure for debug purposes */

    case TIOCSERGSTRUCT:
      {
         struct esp_uart_s *user = (struct esp_uart_s *)arg;
         if (user == NULL)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev->priv, sizeof(struct esp_uart_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS

    /* Fill a termios structure with the required information */

    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        struct esp_uart_s *priv  = (struct esp_uart_s *)dev->priv;
        if (termiosp == NULL)
          {
            ret = -EINVAL;
            break;
          }

        /* Return parity (0 = no parity, 1 = odd parity, 2 = even parity) */

        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                            ((priv->parity == 1) ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= (priv->stop_b2) ? CSTOPB : 0;

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        termiosp->c_cflag |= (priv->oflow) ? CCTS_OFLOW : 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        termiosp->c_cflag |= (priv->iflow) ? CRTS_IFLOW : 0;
#endif

        /* Set the baud rate in termiosp using the cfsetispeed interface */

        cfsetispeed(termiosp, priv->baud);

        /* Return number of bits */

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

            case 8:
            default:
              termiosp->c_cflag |= CS8;
              break;
          }
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        struct esp_uart_s *priv  = (struct esp_uart_s *)dev->priv;
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

        /* Get the target baud rate to change */

        baud = cfgetispeed(termiosp);

        /* Decode number of bits */

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

        /* Decode parity */

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        /* Decode stop bits */

        stop2 = (termiosp->c_cflag & CSTOPB) ? 1 : 0;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
        iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
        oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif

        /* Verify if all settings are valid before performing the changes */

        if (ret == OK)
          {
            /* Fill the private struct fields */

            priv->baud    = baud;
            priv->parity  = parity;
            priv->bits    = bits;
            priv->stop_b2 = stop2;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            priv->iflow   = iflow;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow   = oflow;
#endif

            /* Effect the changes immediately - note that we do not implement
             * TCSADRAIN or TCSAFLUSH, only TCSANOW option.
             * See nuttx/libs/libc/termios/lib_tcsetattr.c
             */

            esp_lowputc_disable_all_uart_int(priv, &current_int_sts);
            ret = esp_setup(dev);

            /* Restore the interrupt state */

            esp_lowputc_restore_all_uart_int(priv, &current_int_sts);
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
 * Name: esp_rxflowcontrol
 *
 * Description:
 *   Called when upper half RX buffer is full (or exceeds configured
 *   watermark levels if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data.
 *   NOTE: Espressif chips have a hardware RX FIFO threshold mechanism to
 *   control RTS line and to stop receiving data. This is very similar to the
 *   concept behind upper watermark level. The hardware threshold is used
 *   here to control the RTS line. When setting the threshold to zero, RTS
 *   will immediately be asserted. If nbuffered = 0 or the lower watermark is
 *   crossed and the serial driver decides to disable RX flow control, the
 *   threshold will be changed to UART_RX_FLOW_THRHD_VALUE, which is almost
 *   half the HW RX FIFO capacity. It keeps some space to keep the data
 *   received between the RTS assertion and the stop by the sender.
 *
 * Input Parameters:
 *   dev           - UART device instance
 *   nbuffered     - the number of characters currently buffered
 *                   (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *                   not defined the value will be 0 for an empty buffer or
 *                   the defined buffer size for a full buffer)
 *   upper         - true indicates the upper watermark was crossed where
 *                   false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool esp_rxflowcontrol(uart_dev_t *dev, unsigned int nbuffered,
                              bool upper)
{
  bool ret = false;
  struct esp_uart_s *priv = dev->priv;
  if (priv->iflow)
    {
      if (nbuffered == 0 || !upper)
        {
          /* Empty buffer, RTS should be de-asserted and logic in above
           * layers should re-enable RX interrupt.
           */

          esp_lowputc_set_iflow(priv, (uint8_t)(UART_RX_FIFO_SIZE / 2),
                                true);
          esp_rxint(dev, true);
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

          esp_lowputc_set_iflow(priv, 0 , true);
          esp_rxint(dev, false);
          ret = true;
        }
    }

  return ret;
}
#endif
#endif /* CONFIG_ESPRESSIF_UART */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup. This must be called
 *   before riscv_serialinit.
 *   NOTE: This function depends on GPIO pin configuration performed in
 *   in up_consoleinit() and main clock initialization performed in
 *   up_clkinitialize().
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
  /* NOTE: All GPIO configuration for the UARTs was performed in
   * esp_lowsetup.
   */

  /* Disable all UARTS interrupts */

#ifdef TTYS0_DEV
  esp_lowputc_disable_all_uart_int(TTYS0_DEV.priv, NULL);
#endif

#ifdef TTYS1_DEV
  esp_lowputc_disable_all_uart_int(TTYS1_DEV.priv, NULL);
#endif

  /* Configure console in early step.
   * Setup for other serials will be perfomed when the serial driver is
   * open.
   */

#ifdef CONSOLE_UART
  esp_setup(&CONSOLE_DEV);
#endif
}

#endif /* USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports. This assumes that
 *   riscv_earlyserialinit has been called previously.
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
 *   Provide priority, low-level access to support OS debug writes.
 *
 * Input Parameters:
 *   ch            - Character to be output to the console.
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller. A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef CONSOLE_UART
  uint32_t int_status;

  esp_lowputc_disable_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);

#ifdef CONSOLE_UART
  esp_lowputc_restore_all_uart_int(CONSOLE_DEV.priv, &int_status);
#endif
  return ch;
}

#else /* HAVE_UART_DEVICE */

/****************************************************************************
 * Name: riscv_earlyserialinit, riscv_serialinit, and up_putc
 *
 * Description:
 *   Stubs that may be needed. These stubs will be used if all UARTs are
 *   disabled. In that case, the logic in common/xtensa_initialize.c is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *   This may be a special case where the upper and lower half serial layers
 *   are added but other device is used as console.
 *   For more details, take a look at: nuttx/arch/xtensa/src/common/xtensa.h
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

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes.
 *
 * Input Parameters:
 *   ch            - Character to be output to the console.
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller. A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
#endif /* HAVE_SERIAL_CONSOLE */

  return ch;
}

#endif /* USE_SERIALDRIVER */
