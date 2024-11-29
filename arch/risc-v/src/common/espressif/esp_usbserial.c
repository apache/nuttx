/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_usbserial.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#  include <nuttx/fs/ioctl.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/serial/serial.h>
#include <arch/irq.h>

#include "riscv_internal.h"

#include "esp_config.h"
#include "esp_irq.h"

#include "hal/uart_hal.h"
#include "hal/usb_serial_jtag_ll.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* The hardware buffer has a fixed size of 64 bytes */

#define ESP_USBCDC_BUFFERSIZE 64

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_priv_s
{
  const uint8_t  source;        /* Source ID */
  const uint8_t  irq;           /* IRQ number assigned to the source */
  int            cpuint;        /* CPU interrupt assigned */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_interrupt(int irq, void *context, void *arg);

/* Serial driver methods */

static int  esp_setup(struct uart_dev_s *dev);
static void esp_shutdown(struct uart_dev_s *dev);
static int  esp_attach(struct uart_dev_s *dev);
static void esp_detach(struct uart_dev_s *dev);
static void esp_txint(struct uart_dev_s *dev, bool enable);
static void esp_rxint(struct uart_dev_s *dev, bool enable);
static bool esp_rxavailable(struct uart_dev_s *dev);
static bool esp_txready(struct uart_dev_s *dev);
static void esp_send(struct uart_dev_s *dev, int ch);
static int  esp_receive(struct uart_dev_s *dev, unsigned int *status);
static int  esp_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_rxbuffer[ESP_USBCDC_BUFFERSIZE];
static char g_txbuffer[ESP_USBCDC_BUFFERSIZE];

static struct esp_priv_s g_usbserial_priv =
{
  .source = USB_SERIAL_JTAG_INTR_SOURCE,
  .irq    = ESP_IRQ_USB_SERIAL_JTAG,
  .cpuint = -ENOMEM,
};

static struct uart_ops_s g_uart_ops =
{
  .setup       = esp_setup,
  .shutdown    = esp_shutdown,
  .attach      = esp_attach,
  .detach      = esp_detach,
  .txint       = esp_txint,
  .rxint       = esp_rxint,
  .rxavailable = esp_rxavailable,
  .txready     = esp_txready,
  .txempty     = NULL,
  .send        = esp_send,
  .receive     = esp_receive,
  .ioctl       = esp_ioctl,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

uart_dev_t g_uart_usbserial =
{
  .isconsole = true,
  .recv =
    {
      .size = ESP_USBCDC_BUFFERSIZE,
      .buffer = g_rxbuffer,
    },
  .xmit =
    {
      .size = ESP_USBCDC_BUFFERSIZE,
      .buffer = g_txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_usbserial_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_interrupt
 *
 * Description:
 *   This is the common UART interrupt handler. It will be invoked when an
 *   interrupt is received on the 'irq'. It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers. The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int esp_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uint32_t tx_mask = USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ST;
  uint32_t rx_mask = USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ST;
  uint32_t int_status = usb_serial_jtag_ll_get_intsts_mask();

  /* Send buffer has room and can accept new data. */

  if ((int_status & tx_mask) != 0)
    {
      usb_serial_jtag_ll_clr_intsts_mask(tx_mask);
      uart_xmitchars(dev);
    }

  /* Data from the host are available to read. */

  if ((int_status & rx_mask) != 0)
    {
      usb_serial_jtag_ll_clr_intsts_mask(rx_mask);
      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int esp_setup(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: esp_shutdown
 *
 * Description:
 *   This method is called when the serial port is closed.
 *
 ****************************************************************************/

static void esp_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: esp_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void esp_txint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      usb_serial_jtag_ll_ena_intr_mask(
        USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ENA);
    }
  else
    {
      usb_serial_jtag_ll_disable_intr_mask(
        USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ENA);
    }
}

/****************************************************************************
 * Name: esp_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void esp_rxint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      usb_serial_jtag_ll_ena_intr_mask(
        USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
    }
  else
    {
      usb_serial_jtag_ll_disable_intr_mask(
        USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
    }
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
 *   RX and TX interrupts are not enabled by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling). The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int esp_attach(struct uart_dev_s *dev)
{
  struct esp_priv_s *priv = dev->priv;
  int ret;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

  /* Try to attach the IRQ to a CPU int */

  priv->cpuint = esp_setup_irq(priv->source,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL);
  if (priv->cpuint < 0)
    {
      return priv->cpuint;
    }

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, esp_interrupt, dev);
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
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void esp_detach(struct uart_dev_s *dev)
{
  struct esp_priv_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
  esp_teardown_irq(priv->source, priv->cpuint);

  priv->cpuint = -ENOMEM;
}

/****************************************************************************
 * Name: esp_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool esp_rxavailable(struct uart_dev_s *dev)
{
  return (bool)usb_serial_jtag_ll_rxfifo_data_available();
}

/****************************************************************************
 * Name: esp_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool esp_txready(struct uart_dev_s *dev)
{
  return (bool)usb_serial_jtag_ll_txfifo_writable();
}

/****************************************************************************
 * Name: esp_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void esp_send(struct uart_dev_s *dev, int ch)
{
  /* Write the character to the buffer. */

  uint8_t buf[1] = {
    (uint8_t)ch
  };

  usb_serial_jtag_ll_write_txfifo(buf, sizeof(buf));

  /* Flush the character out. */

  usb_serial_jtag_ll_txfifo_flush();
}

/****************************************************************************
 * Name: esp32_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character.
 *
 ****************************************************************************/

static int esp_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint8_t buf[1] = {
    0
  };

  *status = 0;
  usb_serial_jtag_ll_read_rxfifo(buf, sizeof(buf));

  return (int)buf[0];
}

/****************************************************************************
 * Name: esp_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int esp_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
          }
        else
          {
            /* The USB Serial Console has fixed configuration of:
             *    9600 baudrate, no parity, 8 bits, 1 stopbit.
             */

            termiosp->c_cflag = CS8;
            cfsetispeed(termiosp, 9600);
          }
      }
      break;

    case TCSETS:
      ret = -ENOTTY;
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

/****************************************************************************
 * Name: esp_usbserial_write
 *
 * Description:
 *   Write one character through the USB serial. Used mainly for early
 *   debugging.
 *
 ****************************************************************************/

void esp_usbserial_write(char ch)
{
  while (!esp_txready(&g_uart_usbserial));

  esp_send(&g_uart_usbserial, ch);
}

