/****************************************************************************
 * arch/avr/src/avrdx/avrdx_serial.c
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
#include "avrdx_config.h"

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/kmalloc.h>
#include <avr/io.h>

#include <arch/board/board.h>

#include "avr_internal.h"
#include "avrdx.h"
#include "avrdx_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Some sanity checks *******************************************************/

/* DMA is not supported */

#  ifdef CONFIG_AVR_USART0
#    if defined(CONFIG_USART0_RXDMA) || defined(CONFIG_USART0_TXDMA)
#      error USART0 DMA is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART1
#    if defined(CONFIG_USART1_RXDMA) || defined(CONFIG_USART1_TXDMA)
#      error USART1 DMA is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART2
#    if defined(CONFIG_USART2_RXDMA) || defined(CONFIG_USART2_TXDMA)
#      error USART2 DMA is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART3
#    if defined(CONFIG_USART3_RXDMA) || defined(CONFIG_USART3_TXDMA)
#      error USART3 DMA is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART4
#    if defined(CONFIG_USART4_RXDMA) || defined(CONFIG_USART4_TXDMA)
#      error USART4 DMA is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART5
#    if defined(CONFIG_USART5_RXDMA) || defined(CONFIG_USART5_TXDMA)
#      error USART5 DMA is not supported
#    endif
#  endif

/* Flow control is not supported, at least not yet */

#  ifdef CONFIG_AVR_USART0
#    if defined(CONFIG_USART0_IFLOWCONTROL) || defined(CONFIG_USART0_OFLOWCONTROL)
#      error USART0 flow control is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART1
#    if defined(CONFIG_USART1_IFLOWCONTROL) || defined(CONFIG_USART1_OFLOWCONTROL)
#      error USART1 flow control is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART2
#    if defined(CONFIG_USART2_IFLOWCONTROL) || defined(CONFIG_USART2_OFLOWCONTROL)
#      error USART2 flow control is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART3
#    if defined(CONFIG_USART3_IFLOWCONTROL) || defined(CONFIG_USART3_OFLOWCONTROL)
#      error USART3 flow control is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART4
#    if defined(CONFIG_USART4_IFLOWCONTROL) || defined(CONFIG_USART4_OFLOWCONTROL)
#      error USART4 flow control is not supported
#    endif
#  endif
#  ifdef CONFIG_AVR_USART5
#    if defined(CONFIG_USART5_IFLOWCONTROL) || defined(CONFIG_USART5_OFLOWCONTROL)
#      error USART5 flow control is not supported
#    endif
#  endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  avrdx_usart_setup(struct uart_dev_s *dev);
static void avrdx_usart_shutdown(struct uart_dev_s *dev);
static int  avrdx_usart_attach(struct uart_dev_s *dev);
static void avrdx_usart_detach(struct uart_dev_s *dev);
static int  avrdx_usart_rxinterrupt(int irq, void *context, FAR void *arg);
static int  avrdx_usart_txinterrupt(int irq, void *context, FAR void *arg);
static int  avrdx_usart_receive(\
  struct uart_dev_s *dev,
  FAR unsigned int *status);
static void avrdx_usart_rxint(struct uart_dev_s *dev, bool enable);
static bool avrdx_usart_rxavailable(struct uart_dev_s *dev);
static void avrdx_usart_send(struct uart_dev_s *dev, int ch);
static void avrdx_usart_txint(struct uart_dev_s *dev, bool enable);
static bool avrdx_usart_txready(struct uart_dev_s *dev);
static bool avrdx_usart_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is holding RX interrupt numbers for all ports,
 * can be indexed by peripheral index instead of need
 * to switch() it
 */

static const IOBJ uint8_t avrdx_usart_rx_interrupts[] =
{
#  ifdef CONFIG_AVR_HAS_USART_2
  AVRDX_IRQ_USART0_RXC, AVRDX_IRQ_USART1_RXC, AVRDX_IRQ_USART2_RXC
#  endif
#  ifdef CONFIG_AVR_HAS_USART_4
  , AVRDX_IRQ_USART3_RXC, AVRDX_IRQ_USART4_RXC
#  endif
#  ifdef CONFIG_AVR_HAS_USART_5
  , AVRDX_IRQ_USART5_RXC
#  endif
};

/* Same thing for DRE interrupts */

static const IOBJ uint8_t avrdx_usart_dre_interrupts[] =
{
#  ifdef CONFIG_AVR_HAS_USART_2
  AVRDX_IRQ_USART0_DRE, AVRDX_IRQ_USART1_DRE, AVRDX_IRQ_USART2_DRE
#  endif
#  ifdef CONFIG_AVR_HAS_USART_4
  , AVRDX_IRQ_USART3_DRE, AVRDX_IRQ_USART4_DRE
#  endif
#  ifdef CONFIG_AVR_HAS_USART_5
  , AVRDX_IRQ_USART5_DRE
#  endif
};

/* USARTn operations - common for all ports */

static const struct uart_ops_s g_usart_ops =
{
  .setup          = avrdx_usart_setup,
  .shutdown       = avrdx_usart_shutdown,
  .attach         = avrdx_usart_attach,
  .detach         = avrdx_usart_detach,
  .ioctl          = 0,
  .receive        = avrdx_usart_receive,
  .rxint          = avrdx_usart_rxint,
  .rxavailable    = avrdx_usart_rxavailable,
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#  endif
#  ifdef CONFIG_SERIAL_TXDMA
  .dmasend        = 0,
#  endif
#  ifdef CONFIG_SERIAL_RXDMA
  .dmareceive     = 0,
  .dmarxfree      = 0,
#  endif
#  ifdef CONFIG_SERIAL_TXDMA
  .dmatxavail     = 0,
#  endif
  .send           = avrdx_usart_send,
  .txint          = avrdx_usart_txint,
  .txready        = avrdx_usart_txready,
  .txempty        = avrdx_usart_txempty,
  .release        = 0,
  .recvbuf        = 0,
  .sendbuf        = 0
};

/* USART device description structs. Pointers, to be allocated
 * as needed
 */

#  ifdef CONFIG_MCU_SERIAL
#    if defined(CONFIG_AVR_HAS_USART_5)
static uart_dev_t *g_usart_ports[6];
#    elif defined(CONFIG_AVR_HAS_USART_4)
static uart_dev_t *g_usart_ports[5];
#    elif defined(CONFIG_AVR_HAS_USART_2)
static uart_dev_t *g_usart_ports[3];
#    endif
#  endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_usart_restoreint
 *
 * Description:
 *   Restore USARTn interrupts
 *
 * Input Parameters:
 *   USARTn peripheral index and value to be restored
 *
 * Assumptions:
 *   Interrupts are disabled or nothing else will manipulate
 *   with CTRLA register
 *
 ****************************************************************************/

#  ifdef AVRDX_SERIAL_CONSOLE_USART_N

static void avrdx_usart_restoreint(uint8_t usart_n, uint8_t imr)
{
  avr_usart_t *usart;
  uint8_t regval;

  usart = &(AVRDX_USART(usart_n));

  regval = usart->CTRLA;
  regval &= ~(USART_RXCIE_bm | USART_TXCIE_bm | USART_DREIE_bm);
  imr    &=  (USART_RXCIE_bm | USART_TXCIE_bm | USART_DREIE_bm);
  regval |= imr;
  usart->CTRLA = regval;
}

#  endif

/****************************************************************************
 * Name: avrdx_usart_disableint
 *
 * Description:
 *   Disable USARTn interrupts
 *
 * Input Parameters:
 *   USARTn peripheral index and pointer to uint8_t which will receive
 *   current interrupt settings
 *
 * Assumptions:
 *   Interrupts are disabled or nothing else will manipulate
 *   with CTRLA register
 *
 ****************************************************************************/

static void avrdx_usart_disableint(uint8_t usart_n, uint8_t *imr)
{
  avr_usart_t *usart;
  uint8_t regval;

  usart = &(AVRDX_USART(usart_n));

  regval = usart->CTRLA;
  if (imr)
    {
      *imr = regval;
    }

  regval &= ~(USART_RXCIE_bm | USART_TXCIE_bm | USART_DREIE_bm);
  usart->CTRLA = regval;
}

/****************************************************************************
 * Name: avrdx_usart_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int avrdx_usart_setup(struct uart_dev_s *dev)
{
#  ifndef CONFIG_SUPPRESS_UART_CONFIG
  avrdx_usart_configure(AVRDX_USART_DEV_PRIV(dev));
#  endif

  return OK;
}

/****************************************************************************
 * Name: avrdx_usart_shutdown
 *
 * Description:
 *   Disable the USART. This method is called when the serial port is closed
 *
 ****************************************************************************/

static void avrdx_usart_shutdown(struct uart_dev_s *dev)
{
  /* Reset, disable interrupts, and disable Rx and Tx */

  avrdx_usart_reset(AVRDX_USART_DEV_PRIV(dev));
}

/****************************************************************************
 * Name: avrdx_usart_attach
 *
 * Description:
 *   Configure the USART to operate in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when the attach method is executed
 *   (unless the hardware supports multiple levels of interrupt enabling).
 *   They are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int avrdx_usart_attach(struct uart_dev_s *dev)
{
  /* Attach the USART0 IRQs:
   *
   * RX:  USART Receive Complete. Set when are unread data in the receive
   *      buffer and cleared when the receive buffer is empty.
   * DRE: USART Data Register Empty.  Indicates if the transmit buffer is
   *      ready to receive new data: The buffer is empty, and therefore ready
   *      to be written.
   */

  irq_attach(avrdx_usart_rx_interrupts[AVRDX_USART_DEV_PRIV(dev)->usart_n],
             avrdx_usart_rxinterrupt, dev);
  irq_attach(avrdx_usart_dre_interrupts[AVRDX_USART_DEV_PRIV(dev)->usart_n],
             avrdx_usart_txinterrupt, dev);

  return OK;
}

/****************************************************************************
 * Name: avrdx_usart_detach
 *
 * Description:
 *   Detach USART interrupts. This method is called when the serial port is
 *   closed - normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void avrdx_usart_detach(struct uart_dev_s *dev)
{
  avr_usart_t *usart;
  usart = &(AVRDX_USART(AVRDX_USART_DEV_PRIV(dev)->usart_n));

  /* Disable all USART0 interrupts */

  avrdx_usart_disableint(AVRDX_USART_DEV_PRIV(dev)->usart_n, 0);

  /* Detach the USART0 IRQs */

  irq_detach(avrdx_usart_rx_interrupts[AVRDX_USART_DEV_PRIV(dev)->usart_n]);
  irq_detach(avrdx_usart_dre_interrupts[AVRDX_USART_DEV_PRIV(dev)->usart_n]);

  /* Clear interrupt flags if any left */

  usart->STATUS = (USART_RXCIE_bm | USART_TXCIE_bm | USART_DREIE_bm);
}

/****************************************************************************
 * Name: avrdx_usart_rxinterrupt
 *
 * Description:
 *   This is the USART RX interrupt handler. It will be invoked when an
 *   RX interrupt is received. It will call uart_recvchars to perform the RX
 *   data transfers.
 *
 ****************************************************************************/

static int avrdx_usart_rxinterrupt(int irq, void *context, FAR void *arg)
{
  uart_recvchars((struct uart_dev_s *) arg);

  return OK;
}

/****************************************************************************
 * Name: avrdx_usart_txinterrupt
 *
 * Description:
 *   This is the USART TX interrupt handler. It will be invoked when an
 *   DRE interrupt is received. It will call uart_xmitchars to perform
 *   the TXdata transfers.
 *
 ****************************************************************************/

static int avrdx_usart_txinterrupt(int irq, void *context, FAR void *arg)
{
  uart_xmitchars((struct uart_dev_s *) arg);

  return OK;
}

/****************************************************************************
 * Name: avrdx_usart_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int avrdx_usart_receive(struct uart_dev_s *dev,
                               FAR unsigned int *status)
{
  avr_usart_t *usart;
  uint8_t temp;

  usart = &(AVRDX_USART(AVRDX_USART_DEV_PRIV(dev)->usart_n));

  /* Return status information
   * As of 2025 Apr 1st, caller of this method in drivers/serial/serial_io.c
   * does nothing with the returned status. That's why this just fills
   * some value from a register (which does hold the status data though)
   */

  temp = usart->RXDATAH;
  if (status)
    {
      *status = temp;
    }

  /* Then return the actual received byte */

  return usart->RXDATAL;
}

/****************************************************************************
 * Name: avrdx_usart_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void avrdx_usart_rxint(struct uart_dev_s *dev, bool enable)
{
  avr_usart_t *usart;

  usart = &(AVRDX_USART(AVRDX_USART_DEV_PRIV(dev)->usart_n));
  if (enable)
    {
#  ifndef CONFIG_SUPPRESS_SERIAL_INTS
      usart->CTRLA |= USART_RXCIE_bm;
#  endif
    }
  else
    {
      usart->CTRLA &= ~USART_RXCIE_bm;
    }
}

/****************************************************************************
 * Name: avrdx_usart_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool avrdx_usart_rxavailable(struct uart_dev_s *dev)
{
  avr_usart_t *usart;
  uint8_t temp;

  usart = &(AVRDX_USART(AVRDX_USART_DEV_PRIV(dev)->usart_n));

  /* Temporary variable to make nxstyle happy */

  temp = USART_RXCIF_bm;
  return ((usart->STATUS & temp) != 0);
}

/****************************************************************************
 * Name: avrdx_usart_send
 *
 * Description:
 *   This method will send one byte on the USART.
 *
 ****************************************************************************/

static void avrdx_usart_send(struct uart_dev_s *dev, int ch)
{
  avr_usart_t *usart;

  usart = &(AVRDX_USART(AVRDX_USART_DEV_PRIV(dev)->usart_n));

  usart->TXDATAL = ch;
}

/****************************************************************************
 * Name: avrdx_usart_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void avrdx_usart_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;
  avr_usart_t *usart;

  usart = &(AVRDX_USART(AVRDX_USART_DEV_PRIV(dev)->usart_n));

  flags = enter_critical_section();
  if (enable)
    {
#  ifndef CONFIG_SUPPRESS_SERIAL_INTS
      usart->CTRLA |= USART_DREIE_bm;

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#  endif
    }
  else
    {
      usart->CTRLA &= ~USART_DREIE_bm;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: avrdx_usart_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool avrdx_usart_txready(struct uart_dev_s *dev)
{
  avr_usart_t *usart;
  uint8_t temp;

  usart = &(AVRDX_USART(AVRDX_USART_DEV_PRIV(dev)->usart_n));

  temp = USART_DREIF_bm;
  return ((usart->STATUS & temp) != 0);
}

/****************************************************************************
 * Name: avrdx_usart_txempty
 *
 * Description:
 *   Return true if the tranmsit data register and shift register are both
 *   empty
 *
 ****************************************************************************/

static bool avrdx_usart_txempty(struct uart_dev_s *dev)
{
  avr_usart_t *usart;
  uint8_t temp;

  usart = &(AVRDX_USART(AVRDX_USART_DEV_PRIV(dev)->usart_n));

  temp = USART_TXCIF_bm;
  return ((usart->STATUS & temp) != 0);
}

/****************************************************************************
 * Name: avrdx_initialize_port
 *
 * Description:
 *   DRY method for USARTn initialization. Allocates data structures
 *   for USARTn peripheral and assigns into g_usart_ports array
 *
 * Input Parameters:
 *   USARTn peripheral index "n", buffer sizes
 *
 * Returned Value:
 *   Pointer to initialized uart_dev_t
 *
 ****************************************************************************/

static uart_dev_t *avrdx_initialize_port(uint8_t usart_n, \
    uint16_t rxbufsize, uint16_t txbufsize)
{
  uart_dev_t *usart_port;

  usart_port = (uart_dev_t *) kmm_zalloc(sizeof(uart_dev_t));

  if (!usart_port)
    {
      abort();
    }

  usart_port->ops = &g_usart_ops;

  usart_port->recv.buffer = (char *) kmm_zalloc(rxbufsize);
  if (!usart_port->recv.buffer)
    {
      abort();
    }

  usart_port->xmit.buffer = (char *) kmm_zalloc(txbufsize);
  if (!usart_port->xmit.buffer)
    {
      abort();
    }

  usart_port->priv = kmm_zalloc(sizeof(struct avrdx_uart_priv_s));
  if (!usart_port->priv)
    {
      abort();
    }

  usart_port->recv.size = rxbufsize;
  usart_port->xmit.size = txbufsize;
  ((struct avrdx_uart_priv_s *)usart_port->priv)->usart_n = usart_n;

  g_usart_ports[usart_n] = usart_port;

  return usart_port;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avr_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that avr_earlyserialinit was called previously.
 *
 ****************************************************************************/

void avr_serialinit(void)
{
  uart_dev_t *usart_port;

#  ifdef CONFIG_AVR_USART0
  usart_port = avrdx_initialize_port(0, \
      CONFIG_USART0_RXBUFSIZE, CONFIG_USART0_TXBUFSIZE);

  uart_register("/dev/ttyS0", usart_port);

#    ifdef CONFIG_USART0_SERIAL_CONSOLE
  usart_port->isconsole = true;
  uart_register("/dev/console", usart_port);
#    endif

#  endif

#  ifdef CONFIG_AVR_USART1
  usart_port = avrdx_initialize_port(1, \
      CONFIG_USART1_RXBUFSIZE, CONFIG_USART1_TXBUFSIZE);

  uart_register("/dev/ttyS1", usart_port);

#    ifdef CONFIG_USART1_SERIAL_CONSOLE
  usart_port->isconsole = true;
  uart_register("/dev/console", usart_port);
#    endif

#  endif

#  ifdef CONFIG_AVR_USART2
  usart_port = avrdx_initialize_port(2, \
      CONFIG_USART2_RXBUFSIZE, CONFIG_USART2_TXBUFSIZE);

  uart_register("/dev/ttyS2", usart_port);

#    ifdef CONFIG_USART2_SERIAL_CONSOLE
  usart_port->isconsole = true;
  uart_register("/dev/console", usart_port);
#    endif

#  endif

#  ifdef CONFIG_AVR_USART3
  usart_port = avrdx_initialize_port(3, \
      CONFIG_USART3_RXBUFSIZE, CONFIG_USART3_TXBUFSIZE);

  uart_register("/dev/ttyS3", usart_port);

#    ifdef CONFIG_USART3_SERIAL_CONSOLE
  usart_port->isconsole = true;
  uart_register("/dev/console", usart_port);
#    endif

#  endif

#  ifdef CONFIG_AVR_USART4
  usart_port = avrdx_initialize_port(4, \
      CONFIG_USART4_RXBUFSIZE, CONFIG_USART4_TXBUFSIZE);

  uart_register("/dev/ttyS4", usart_port);

#    ifdef CONFIG_USART4_SERIAL_CONSOLE
  usart_port->isconsole = true;
  uart_register("/dev/console", usart_port);
#    endif

#  endif

#  ifdef CONFIG_AVR_USART5
  usart_port = avrdx_initialize_port(5, \
      CONFIG_USART5_RXBUFSIZE, CONFIG_USART5_TXBUFSIZE);

  uart_register("/dev/ttyS5", usart_port);

#    ifdef CONFIG_USART5_SERIAL_CONSOLE
  usart_port->isconsole = true;
  uart_register("/dev/console", usart_port);
#    endif

#  endif
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
#  ifdef AVRDX_SERIAL_CONSOLE_USART_N
  uint8_t imr;

  avrdx_usart_disableint(AVRDX_SERIAL_CONSOLE_USART_N, &imr);
  avr_lowputc(ch);
  avrdx_usart_restoreint(AVRDX_SERIAL_CONSOLE_USART_N, imr);
#  endif
}

#else /* for ifdef USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
  avr_lowputc(ch);
}

#endif /* USE_SERIALDRIVER */
