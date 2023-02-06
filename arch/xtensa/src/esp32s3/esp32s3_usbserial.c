/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_usbserial.c
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
#endif

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/tioctl.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_usb_serial_jtag.h"

#include "esp32s3_config.h"
#include "esp32s3_irq.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* The hardware buffer has a fixed size of 64 bytes */

#define ESP32S3_USBCDC_BUFFERSIZE 64

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_priv_s
{
  const uint8_t  periph;        /* peripheral ID */
  const uint8_t  irq;           /* IRQ number assigned to the peripheral */
  int            cpu;           /* CPU id */
  int            cpuint;        /* CPU interrupt assigned */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32s3_interrupt(int irq, void *context, void *arg);

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_rxbuffer[ESP32S3_USBCDC_BUFFERSIZE];
static char g_txbuffer[ESP32S3_USBCDC_BUFFERSIZE];

static struct esp32s3_priv_s g_usbserial_priv =
{
  .periph = ESP32S3_PERIPH_USB_DEVICE,
  .irq    = ESP32S3_IRQ_USB_DEVICE,
  .cpu    = 0,
  .cpuint = -ENOMEM,
};

static struct uart_ops_s g_uart_ops =
{
  .setup       = esp32s3_setup,
  .shutdown    = esp32s3_shutdown,
  .attach      = esp32s3_attach,
  .detach      = esp32s3_detach,
  .txint       = esp32s3_txint,
  .rxint       = esp32s3_rxint,
  .rxavailable = esp32s3_rxavailable,
  .txready     = esp32s3_txready,
  .txempty     = esp32s3_txempty,
  .send        = esp32s3_send,
  .receive     = esp32s3_receive,
  .ioctl       = esp32s3_ioctl,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

uart_dev_t g_uart_usbserial =
{
  .isconsole = true,
  .recv      =
    {
      .size    = ESP32S3_USBCDC_BUFFERSIZE,
      .buffer  = g_rxbuffer,
    },
  .xmit      =
    {
      .size    = ESP32S3_USBCDC_BUFFERSIZE,
      .buffer  = g_txbuffer,
    },
  .ops       = &g_uart_ops,
  .priv      = &g_usbserial_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_interrupt
 *
 * Description:
 *   This is the common UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int esp32s3_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uint32_t regval;

  regval = getreg32(USB_SERIAL_JTAG_INT_ST_REG);

  /* Send buffer has room and can accept new data. */

  if (regval & USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ST)
    {
      putreg32(USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_CLR,
               USB_SERIAL_JTAG_INT_CLR_REG);
      uart_xmitchars(dev);
    }

  /* Data from the host are available to read. */

  if (regval & USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ST)
    {
      putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
               USB_SERIAL_JTAG_INT_CLR_REG);
      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int esp32s3_setup(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: esp32s3_shutdown
 *
 * Description:
 *   This method is called when the serial port is closed.
 *
 ****************************************************************************/

static void esp32s3_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: esp32s3_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void esp32s3_txint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
                  USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ENA);
    }
  else
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG,
                  USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ENA, 0);
    }
}

/****************************************************************************
 * Name: esp32s3_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void esp32s3_rxint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
                  USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
    }
  else
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG,
                  USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA, 0);
    }
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
 *   RX and TX interrupts are not enabled by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int esp32s3_attach(struct uart_dev_s *dev)
{
  struct esp32s3_priv_s *priv = dev->priv;
  int ret;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

  /* Try to attach the IRQ to a CPU int */

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32s3_setup_irq(priv->cpu, priv->periph,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      return priv->cpuint;
    }

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, esp32s3_interrupt, dev);
  if (ret == OK)
    {
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
 ****************************************************************************/

static void esp32s3_detach(struct uart_dev_s *dev)
{
  struct esp32s3_priv_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
  esp32s3_teardown_irq(priv->cpu, priv->periph, priv->cpuint);

  priv->cpuint = -ENOMEM;
}

/****************************************************************************
 * Name: esp32s3_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool esp32s3_rxavailable(struct uart_dev_s *dev)
{
  uint32_t regval;

  regval = getreg32(USB_SERIAL_JTAG_EP1_CONF_REG);

  return regval & USB_SERIAL_JTAG_SERIAL_OUT_EP_DATA_AVAIL;
}

/****************************************************************************
 * Name: esp32s3_txempty
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool esp32s3_txempty(struct uart_dev_s *dev)
{
  uint32_t regval;

  regval = getreg32(USB_SERIAL_JTAG_JFIFO_ST_REG);

  return regval & USB_SERIAL_JTAG_OUT_FIFO_EMPTY;
}

/****************************************************************************
 * Name: esp32s3_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool esp32s3_txready(struct uart_dev_s *dev)
{
  uint32_t regval;

  regval = getreg32(USB_SERIAL_JTAG_EP1_CONF_REG);

  return regval & USB_SERIAL_JTAG_SERIAL_IN_EP_DATA_FREE;
}

/****************************************************************************
 * Name: esp32s3_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void esp32s3_send(struct uart_dev_s *dev, int ch)
{
  /* Write the character to the buffer. */

  putreg32(ch, USB_SERIAL_JTAG_EP1_REG);

  /* Flush the character out. */

  putreg32(USB_SERIAL_JTAG_WR_DONE, USB_SERIAL_JTAG_EP1_CONF_REG);
}

/****************************************************************************
 * Name: esp32s3_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character.
 *
 ****************************************************************************/

static int esp32s3_receive(struct uart_dev_s *dev, unsigned int *status)
{
  *status = 0;
  return getreg32(USB_SERIAL_JTAG_EP1_REG) & USB_SERIAL_JTAG_RDWR_BYTE;
}

/****************************************************************************
 * Name: esp32s3_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int esp32s3_ioctl(struct file *filep, int cmd, unsigned long arg)
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
 * Name: esp32s3_usbserial_write
 *
 * Description:
 *   Write one character through the USB serial.  Used mainly for early
 *   debugging.
 *
 ****************************************************************************/

void esp32s3_usbserial_write(char ch)
{
  while (!esp32s3_txready(&g_uart_usbserial));

  esp32s3_send(&g_uart_usbserial, ch);
}

