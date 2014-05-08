/****************************************************************************
 * arch/arm/src/sama5/sam_dbgu.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/serial.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

#include "chip.h"
#include "chip/sam_dbgu.h"
#include "chip/sam_pinmap.h"
#include "sam_pio.h"
#include "sam_dbgu.h"

#ifdef CONFIG_SAMA5_DBGU

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef USE_SERIALDRIVER
struct dbgu_dev_s
{
  uint32_t sr;        /* Saved status bits */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  dbgu_setup(struct uart_dev_s *dev);

#ifdef USE_SERIALDRIVER
static void dbgu_shutdown(struct uart_dev_s *dev);
static int  dbgu_attach(struct uart_dev_s *dev);
static void dbgu_detach(struct uart_dev_s *dev);
static int  dbgu_interrupt(int irq, void *context);
static int  dbgu_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  dbgu_receive(struct uart_dev_s *dev, uint32_t *status);
static void dbgu_rxint(struct uart_dev_s *dev, bool enable);
static bool dbgu_rxavailable(struct uart_dev_s *dev);
static void dbgu_send(struct uart_dev_s *dev, int ch);
static void dbgu_txint(struct uart_dev_s *dev, bool enable);
static bool dbgu_txready(struct uart_dev_s *dev);
static bool dbgu_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = dbgu_setup,
  .shutdown       = dbgu_shutdown,
  .attach         = dbgu_attach,
  .detach         = dbgu_detach,
  .ioctl          = dbgu_ioctl,
  .receive        = dbgu_receive,
  .rxint          = dbgu_rxint,
  .rxavailable    = dbgu_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = dbgu_send,
  .txint          = dbgu_txint,
  .txready        = dbgu_txready,
  .txempty        = dbgu_txempty,
};

/* DBGU I/O buffers */

static char g_dbgu_rxbuffer[CONFIG_SAMA5_DBGU_RXBUFSIZE];
static char g_dbgu_txbuffer[CONFIG_SAMA5_DBGU_TXBUFSIZE];

/* This describes the private state of the DBGU port */

static struct dbgu_dev_s g_dbgu_priv;

/* This describes the state of the DBGU port as is visible from the upper
 * half serial driver.
 */

static uart_dev_t g_dbgu_port =
{
  .recv     =
  {
    .size   = CONFIG_SAMA5_DBGU_RXBUFSIZE,
    .buffer = g_dbgu_rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_SAMA5_DBGU_TXBUFSIZE,
    .buffer = g_dbgu_txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_dbgu_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dbgu_configure
 *
 * Description:
 *   Configure the DBGU baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#if !defined(CONFIG_SUPPRESS_UART_CONFIG) && !defined(CONFIG_SAMA5_DBGU_NOCONFIG)
static void dbgu_configure(void)
{
  uint32_t regval;

  /* Set up the mode register.  Start with normal DBGU mode and the MCK
   * as the timing source
   */

  regval = DBGU_MR_CHMODE_NORMAL;

  /* OR in settings for the selected parity */

#if CONFIG_SAMA5_DBGU_PARITY == 1
  regval |= DBGU_MR_PAR_ODD;
#elif CONFIG_SAMA5_DBGU_PARITY == 2
  regval |= DBGU_MR_PAR_EVEN;
#else
  regval |= DBGU_MR_PAR_NONE;
#endif

  /* And save the new mode register value */

  putreg32(regval, SAM_DBGU_MR);

  /* Configure the console baud.  NOTE: Oversampling by 8 is not supported.
   * This may limit BAUD rates for lower DBGU clocks.
   */

  regval  = (BOARD_MCK_FREQUENCY + (CONFIG_SAMA5_DBGU_BAUD << 3)) /
            (CONFIG_SAMA5_DBGU_BAUD << 4);
  putreg32(regval, SAM_DBGU_BRGR);

  /* Enable receiver & transmitter */

  putreg32((DBGU_CR_RXEN|DBGU_CR_TXEN), SAM_DBGU_CR);
}

#else
#  define dbgu_configure()
#endif /* !CONFIG_SUPPRESS_UART_CONFIG && !CONFIG_SAMA5_DBGU_NOCONFIG */

/****************************************************************************
 * Name: dbgu_setup
 *
 * Description:
 *   Configure the DBGU baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int dbgu_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* The shutdown method will put the DBGU in a known, disabled state */

  dbgu_shutdown(dev);

  /* Then configure the DBGU */

  dbgu_configure();
#endif
  return OK;
}

/****************************************************************************
 * Name: dbgu_shutdown
 *
 * Description:
 *   Disable the DBGU.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void dbgu_shutdown(struct uart_dev_s *dev)
{
#if !defined(CONFIG_SUPPRESS_UART_CONFIG) && !defined(CONFIG_SAMA5_DBGU_NOCONFIG)
  /* Disable all interrupts */

  putreg32(DBGU_INT_ALLINTS, SAM_DBGU_IDR);

#else
  irqstate_t flags;

  /* The following must be atomic */

  flags = irqsave();

  /* Reset and disable receiver and transmitter */

  putreg32((DBGU_CR_RSTRX|DBGU_CR_RSTTX|DBGU_CR_RXDIS|DBGU_CR_TXDIS), SAM_DBGU_CR);

  /* Disable all interrupts */

  putreg32(DBGU_INT_ALLINTS, SAM_DBGU_IDR);
  irqrestore(flags);
#endif
}

/****************************************************************************
 * Name: dbgu_attach
 *
 * Description:
 *   Configure the DBGU to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int dbgu_attach(struct uart_dev_s *dev)
{
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(SAM_IRQ_DBGU, dbgu_interrupt);
  if (ret == OK)
    {
       /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the DBGU
        */

       up_enable_irq(SAM_IRQ_DBGU);
    }

  return ret;
}

/****************************************************************************
 * Name: dbgu_detach
 *
 * Description:
 *   Detach DBGU interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void dbgu_detach(struct uart_dev_s *dev)
{
  up_disable_irq(SAM_IRQ_DBGU);
  irq_detach(SAM_IRQ_DBGU);
}

/****************************************************************************
 * Name: dbgu_interrupt
 *
 * Description:
 *   This is the DBGU interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int dbgu_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = &g_dbgu_port;
  struct dbgu_dev_s *priv = (struct dbgu_dev_s*)dev->priv;
  uint32_t           pending;
  uint32_t           imr;
  int                passes;
  bool               handled;

  /* Loop until there are no characters to be transferred or, until we have
   * been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the DBGU/DBGU status (we are only interested in the unmasked interrupts). */

      priv->sr = getreg32(SAM_DBGU_SR);  /* Save for error reporting */
      imr      = getreg32(SAM_DBGU_IMR); /* Interrupt mask */
      pending  = priv->sr & imr;         /* Mask out disabled interrupt sources */

      /* Handle an incoming, receive byte.  RXRDY: At least one complete character
       * has been received and US_RHR has not yet been read.
       */

      if ((pending & DBGU_INT_RXRDY) != 0)
        {
           /* Received data ready... process incoming bytes */

           uart_recvchars(dev);
           handled = true;
        }

      /* Handle outgoing, transmit bytes. XRDY: There is no character in the
       * US_THR.
       */

      if ((pending & DBGU_INT_TXRDY) != 0)
        {
           /* Transmit data register empty ... process outgoing bytes */

           uart_xmitchars(dev);
           handled = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: dbgu_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int dbgu_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct dbgu_dev_s *user = (struct dbgu_dev_s*)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct dbgu_dev_s));
           }
       }
       break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: dbgu_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the DBGU.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int dbgu_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct dbgu_dev_s *priv = (struct dbgu_dev_s*)dev->priv;

  /* Return the error information in the saved status */

  *status  = priv->sr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return (int)(getreg32(SAM_DBGU_RHR) & 0xff);
}

/****************************************************************************
 * Name: dbgu_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void dbgu_rxint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      putreg32(DBGU_INT_RXRDY, SAM_DBGU_IER);
#endif
    }
  else
    {
      putreg32(DBGU_INT_RXRDY, SAM_DBGU_IDR);
    }
}

/****************************************************************************
 * Name: dbgu_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool dbgu_rxavailable(struct uart_dev_s *dev)
{
  return ((getreg32(SAM_DBGU_SR) & DBGU_INT_RXRDY) != 0);
}

/****************************************************************************
 * Name: dbgu_send
 *
 * Description:
 *   This method will send one byte on the DBGU/DBGU
 *
 ****************************************************************************/

static void dbgu_send(struct uart_dev_s *dev, int ch)
{
  putreg32((uint32_t)ch, SAM_DBGU_THR);
}

/****************************************************************************
 * Name: dbgu_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void dbgu_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX holding register register
       * is empty
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      putreg32(DBGU_INT_TXRDY, SAM_DBGU_IER);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      putreg32(DBGU_INT_TXRDY, SAM_DBGU_IDR);
    }

  irqrestore(flags);
}

/****************************************************************************
 * Name: dbgu_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool dbgu_txready(struct uart_dev_s *dev)
{
  return ((getreg32(SAM_DBGU_SR) & DBGU_INT_TXRDY) != 0);
 }

/****************************************************************************
 * Name: dbgu_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool dbgu_txempty(struct uart_dev_s *dev)
{
  return ((getreg32(SAM_DBGU_SR) & DBGU_INT_TXEMPTY) != 0);
}

/****************************************************************************
 * Name: sam_dbgu_devinitialize
 *
 * Description:
 *   Performs the low level DBGU initialization early in debug so that the
 *   DBGU console will be available during bootup.  This must be called
 *   before getreg32it.
 *
 ****************************************************************************/

void sam_dbgu_devinitialize(void)
{
  /* Disable all DBGU interrupts */

  putreg32(DBGU_INT_ALLINTS, SAM_DBGU_IDR);

#ifdef CONFIG_SAMA5_DBGU_CONSOLE
  /* Configuration the DBGU as the the console */

  g_dbgu_port.isconsole = true;
  dbgu_configure();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dbgu_register
 *
 * Description:
 *   Register DBBU console and serial port.  This assumes that
 *   sam_dbgu_initialize() was called previously.
 *
 ****************************************************************************/

void sam_dbgu_register(void)
{
  /* Register the console */

#ifdef CONFIG_SAMA5_DBGU_CONSOLE
  (void)uart_register("/dev/console", &g_dbgu_port);
#endif

  /* Register the DBGU serial device */

  (void)uart_register("/dev/ttyDBGU", &g_dbgu_port);
}

#endif /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: sam_dbgu_initialize
 *
 * Description:
 *   Performs the low level DBGU initialization early in debug so that the
 *   DBGU console will be available during bootup.  This must be called
 *   before getreg32it.
 *
 ****************************************************************************/

void sam_dbgu_initialize(void)
{
  /* Initialize DBGU pins */

  (void)sam_configpio(PIO_DBGU_DRXD);
  (void)sam_configpio(PIO_DBGU_DTXD);

#if defined(USE_SERIALDRIVER)
  /* Initialize the DBGU device driver */

  sam_dbgu_devinitialize();
#elif defined(CONFIG_SAMA5_DBGU_CONSOLE)
  sam_dbgu_configure();
#endif
}

#endif /* CONFIG_SAMA5_DBGU */
