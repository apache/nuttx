/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_serial.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "lpc43_config.h"
#include "lpc43_serial.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase;  /* Base address of UART registers */
  uint32_t  basefreq;  /* Base frequency of input clock */
  uint32_t  baud;      /* Configured baud */
  uint32_t  ier;       /* Saved IER value */
  uint8_t   id;        /* ID=0,1,2,3 */
  uint8_t   irq;       /* IRQ associated with this UART */
  uint8_t   parity;    /* 0=none, 1=odd, 2=even */
  uint8_t   bits;      /* Number of bits (7 or 8) */
  bool      stopbits2; /* true: Configure with 2 stop bits instead of 1 */
#ifdef HAVE_RS485
  bool      dctrl;     /* Hardware RS485 direction control */
  bool      diroinv;   /* Direction pin polarity invert */
  bool      dtrdir;    /* DTR pin is the direction bit */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifdef HAVE_RS485
static inline int up_set_rs485_mode(struct up_dev_s *priv,
                                    const struct serial_rs485 *mode);

static inline int up_get_rs485_mode(struct up_dev_s *priv,
                                    struct serial_rs485 *mode);
#endif
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_LPC43_USART0
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef CONFIG_LPC43_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_LPC43_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef CONFIG_LPC43_USART3
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif

/* This describes the state of the LPC43xx uart0 port. */

#ifdef CONFIG_LPC43_USART0
static struct up_dev_s g_usart0priv =
{
  .uartbase       = LPC43_USART0_BASE,
  .basefreq       = BOARD_USART0_BASEFREQ,
  .baud           = CONFIG_USART0_BAUD,
  .id             = 0,
  .irq            = LPC43M4_IRQ_USART0,
  .parity         = CONFIG_USART0_PARITY,
  .bits           = CONFIG_USART0_BITS,
  .stopbits2      = CONFIG_USART0_2STOP,
#if defined(CONFIG_USART0_RS485MODE)
  .dctrl          = true,
#  if defined(CONFIG_USART0_RS485DIROIN)
  .diroinv        = true,
#  else
  .diroinv        = false,
#  endif
#elif defined(HAVE_RS485) /* RS485 supported, but not on USART0 */
  .dctrl          = false,
#endif
};

static uart_dev_t g_usart0port =
{
  .recv     =
  {
    .size   = CONFIG_USART0_RXBUFSIZE,
    .buffer = g_usart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART0_TXBUFSIZE,
    .buffer = g_usart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart0priv,
};
#endif

/* This describes the state of the LPC43xx uart1 port. */

#ifdef CONFIG_LPC43_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = LPC43_UART1_BASE,
  .basefreq       = BOARD_UART1_BASEFREQ,
  .baud           = CONFIG_UART1_BAUD,
  .id             = 1,
  .irq            = LPC43M4_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stopbits2      = CONFIG_UART1_2STOP,
#if defined(CONFIG_UART1_RS485MODE) && defined(CONFIG_UART1_RS485_DTRDIR)
  .dtrdir         = true,
#endif
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};
#endif

/* This describes the state of the LPC43xx uart1 port. */

#ifdef CONFIG_LPC43_USART2
static struct up_dev_s g_usart2priv =
{
  .uartbase       = LPC43_USART2_BASE,
  .basefreq       = BOARD_USART2_BASEFREQ,
  .baud           = CONFIG_USART2_BAUD,
  .id             = 2,
  .irq            = LPC43M4_IRQ_USART2,
  .parity         = CONFIG_USART2_PARITY,
  .bits           = CONFIG_USART2_BITS,
  .stopbits2      = CONFIG_USART2_2STOP,
#if defined(CONFIG_USART2_RS485MODE)
  .dctrl          = true,
#  if defined(CONFIG_USART2_RS485DIROIN)
  .diroinv        = true,
#  else
  .diroinv        = false,
#  endif
#elif defined(HAVE_RS485) /* RS485 supported, but not on USART2 */
  .dctrl          = false,
#endif
};

static uart_dev_t g_usart2port =
{
  .recv     =
  {
    .size   = CONFIG_USART2_RXBUFSIZE,
    .buffer = g_usart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART2_TXBUFSIZE,
    .buffer = g_usart2txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart2priv,
};
#endif

/* This describes the state of the LPC43xx uart1 port. */

#ifdef CONFIG_LPC43_USART3
static struct up_dev_s g_usart3priv =
{
  .uartbase       = LPC43_USART3_BASE,
  .basefreq       = BOARD_USART3_BASEFREQ,
  .baud           = CONFIG_USART3_BAUD,
  .id             = 3,
  .irq            = LPC43M4_IRQ_USART3,
  .parity         = CONFIG_USART3_PARITY,
  .bits           = CONFIG_USART3_BITS,
  .stopbits2      = CONFIG_USART3_2STOP,
#if defined(CONFIG_USART3_RS485MODE)
  .dctrl          = true,
#  if defined(CONFIG_USART3_RS485DIROIN)
  .diroinv        = true,
#  else
  .diroinv        = false,
#  endif
#elif defined(HAVE_RS485) /* RS485 supported, but not on USART3 */
  .dctrl          = false,
#endif
};

static uart_dev_t g_usart3port =
{
  .recv     =
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_usart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_usart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_usart3priv,
};
#endif

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#undef CONSOLE_DEV                          /* Assume no console */
#undef TTYS0_DEV                            /* Assume no ttyS0 */
#undef TTYS1_DEV                            /* Assume no ttyS1 */
#undef TTYS2_DEV                            /* Assume no ttyS2 */
#undef TTYS3_DEV                            /* Assume no ttyS3 */
#undef USART0_ASSIGNED                      /* USART0 has not been assigned */
#undef UART1_ASSIGNED                       /* UART1 has not been assigned */
#undef USART2_ASSIGNED                      /* USART2 has not been assigned */
#undef USART3_ASSIGNED                      /* USART3 has not been assigned */

/* Assign the console device and ttyS0 */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_usart0port    /* USART0=console */
#    define TTYS0_DEV       g_usart0port    /* USART0=ttyS0 */
#    define USART0_ASSIGNED 1
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port     /* UART1=console */
#    define TTYS0_DEV       g_uart1port     /* UART1=ttyS0 */
#    define UART1_ASSIGNED  1
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_usart2port    /* USART2=console */
#    define TTYS0_DEV       g_usart2port    /* USART2=ttyS0 */
#    define USART2_ASSIGNED 1
#  else /* elif defined(CONFIG_USART3_SERIAL_CONSOLE) */
#    define CONSOLE_DEV     g_usart3port    /* USART3=console */
#    define TTYS0_DEV       g_usart3port    /* USART3=ttyS0 */
#    define USART3_ASSIGNED 1
# endif
#else
/* No console, assign only ttyS0 */

#  if defined(CONFIG_LPC43_USART0)
#    define TTYS0_DEV       g_usart0port    /* USART0=ttyS0 */
#    define USART0_ASSIGNED 1
#  elif defined(CONFIG_LPC43_UART1)
#    define TTYS0_DEV       g_uart1port     /* UART1=ttyS0 */
#    define UART1_ASSIGNED  1
#  elif defined(CONFIG_LPC43_USART2)
#    define TTYS0_DEV       g_usart2port    /* USART2=ttyS0 */
#    define USART2_ASSIGNED 1
#  else /* elif defined(CONFIG_LPC43_USART3) */
#    define TTYS0_DEV       g_usart3port    /* USART3=ttyS0 */
#    define USART3_ASSIGNED 1
# endif
#endif

/* Assign ttyS1 */

#if defined(CONFIG_LPC43_USART0) && !defined(USART0_ASSIGNED)
#  define TTYS1_DEV         g_usart0port    /* USART0=ttyS1 */
#  define USART0_ASSIGNED 1
#elif defined(CONFIG_LPC43_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV         g_uart1port     /* UART1=ttyS1 */
#  define UART1_ASSIGNED 1
#elif defined(CONFIG_LPC43_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS1_DEV         g_usart2port    /* USART2=ttyS1 */
#  define USART2_ASSIGNED 1
#elif defined(CONFIG_LPC43_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS1_DEV         g_usart3port    /* USART3=ttyS1 */
#  define USART3_ASSIGNED 1
#endif

/* Assign ttyS2.  USART0 has already been assigned if it was configured. */

#if defined(CONFIG_LPC43_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV         g_uart1port     /* UART1=ttyS2 */
#  define UART1_ASSIGNED 1
#elif defined(CONFIG_LPC43_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS2_DEV         g_usart2port    /* USART2=ttyS2 */
#  define USART2_ASSIGNED 1
#elif defined(CONFIG_LPC43_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS2_DEV         g_usart3port    /* USART3=ttyS2 */
#  define USART3_ASSIGNED 1
#endif

/* Assign ttyS3.  USART0 and UART1 have already been assigned if they were
 * configured.
 */

#if defined(CONFIG_LPC43_USART2) && !defined(USART2_ASSIGNED)
#  define TTYS3_DEV         g_usart2port    /* USART2=ttyS3 */
#  define USART2_ASSIGNED 1
#elif defined(CONFIG_LPC43_USART3) && !defined(USART3_ASSIGNED)
#  define TTYS3_DEV         g_usart3port    /* USART3=ttyS3 */
#  define USART3_ASSIGNED 1
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *ier)
{
  if (ier)
    {
      *ier = priv->ier & UART_IER_ALLIE;
    }

  priv->ier &= ~UART_IER_ALLIE;
  up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
  up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);
}
#endif

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
  uint32_t lcr = up_serialin(priv, LPC43_UART_LCR_OFFSET);

  if (enable)
    {
      lcr |= UART_LCR_BRK;
    }
  else
    {
      lcr &= ~UART_LCR_BRK;
    }

  up_serialout(priv, LPC43_UART_LCR_OFFSET, lcr);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
#ifdef HAVE_RS485
  struct serial_rs485 rs485mode;
#endif
  uint32_t lcr;

  /* Clear fifos */

  up_serialout(priv, LPC43_UART_FCR_OFFSET,
               (UART_FCR_RXRST | UART_FCR_TXRST));

  /* Set trigger */

  up_serialout(priv, LPC43_UART_FCR_OFFSET,
               (UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_8));

  /* Set up the IER */

  priv->ier = up_serialin(priv, LPC43_UART_IER_OFFSET);

  /* Set up the LCR */

  lcr = 0;

  if (priv->bits == 7)
    {
      lcr |= UART_LCR_WLS_7BIT;
    }
  else
    {
      lcr |= UART_LCR_WLS_8BIT;
    }

  if (priv->stopbits2)
    {
      lcr |= UART_LCR_STOP;
    }

  if (priv->parity == 1)
    {
      lcr |= (UART_LCR_PE | UART_LCR_PS_ODD);
    }
  else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PE | UART_LCR_PS_EVEN);
    }

  /* Save the LCR */

  up_serialout(priv, LPC43_UART_LCR_OFFSET, lcr);

  /* Set the BAUD divisor */

  lpc43_setbaud(priv->uartbase, priv->basefreq, priv->baud);

  /* Configure the FIFOs */

  up_serialout(priv, LPC43_UART_FCR_OFFSET,
               (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST | UART_FCR_RXRST |
                UART_FCR_FIFOEN));

  /* Enable Auto-RTS and Auto-CS Flow Control in the Modem Control Register */

#ifdef CONFIG_UART1_FLOWCONTROL
  if (priv->id == 1)
    {
      up_serialout(priv, LPC43_UART_MCR_OFFSET,
                   (UART_MCR_RTSEN | UART_MCR_CTSEN));
    }
#endif

  /* Setup initial RS485 settings */

#ifdef HAVE_RS485
  if (priv->dctrl)
    {
      rs485mode.flags                 = SER_RS485_ENABLED;
      rs485mode.delay_rts_after_send  = 0;
      rs485mode.delay_rts_before_send = 0;

      if (priv->diroinv)
        {
          rs485mode.flags |= SER_RS485_RTS_ON_SEND;
        }

      up_set_rs485_mode(priv, &rs485mode);
    }
#endif

#endif
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable further interrupts from the U[S]ART */

  up_disableuartint(priv, NULL);

  /* Put the U[S]ART hardware back its reset state */

  switch (priv->id)
    {
#ifdef CONFIG_LPC43_USART0
      case 0:
        lpc43_usart0_reset();
        break;
#endif

#ifdef CONFIG_LPC43_UART1
      case 1:
        lpc43_uart1_reset();
        break;
#endif

#ifdef CONFIG_LPC43_USART2
      case 2:
        lpc43_usart2_reset();
        break;
#endif

#ifdef CONFIG_LPC43_USART3
      case 3:
        lpc43_usart3_reset();
        break;
#endif

      default:
        break;
    }
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just
 *   after the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint32_t           status;
  int                passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

       status = up_serialin(priv, LPC43_UART_IIR_OFFSET);

      /* The UART_IIR_INTSTATUS bit should be zero if there are pending
       * interrupts
       */

      if ((status & UART_IIR_INTSTATUS) != 0)
        {
          /* Break out of the loop when there is no longer a
           * pending interrupt
           */

          break;
        }

      /* Handle the interrupt by its interrupt ID field */

      switch (status & UART_IIR_INTID_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

          case UART_IIR_INTID_RDA:
          case UART_IIR_INTID_CTI:
            {
              uart_recvchars(dev);
              break;
            }

          /* Handle outgoing, transmit bytes */

          case UART_IIR_INTID_THRE:
            {
              uart_xmitchars(dev);
              break;
            }

          /* Just clear modem status interrupts (UART1 only) */

          case UART_IIR_INTID_MSI:
            {
              /* Read the modem status register (MSR) to clear */

              status = up_serialin(priv, LPC43_UART_MSR_OFFSET);
              _info("MSR: %02" PRIx32 "\n", status);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_INTID_RLS:
            {
              /* Read the line status register (LSR) to clear */

              status = up_serialin(priv, LPC43_UART_LSR_OFFSET);
              _info("LSR: %02" PRIx32 "\n", status);
              break;
            }

          /* There should be no other values */

          default:
            {
              _err("ERROR: Unexpected IIR: %02" PRIx32 "\n", status);
              break;
            }
        }
    }

    return OK;
}

/****************************************************************************
 * Name: up_set_rs485_mode
 *
 * Description:
 *   Handle LPC43xx USART0,2,3 RS485 mode set ioctl (TIOCSRS485) to enable
 *   and disable RS-485 mode.  This is part of the serial ioctl logic.
 *
 *   Supported and un-supported LPC43 RS-485 features:
 *
 *     RS-485/EIA-485 Normal Multidrop Mode (NMM) -- NOT supported
 *
 *       In this mode, an address is detected when a received byte causes
 *       the USART to set the parity error and generate an interrupt.  When
 *       the parity error interrupt will be generated and the processor can
 *       decide whether or not to disable the receiver.
 *
 *     RS-485/EIA-485 Auto Address Detection (AAD) mode -- NOT supported
 *
 *       In this mode, the receiver will compare any address byte received
 *       (parity = 1) to the 8-bit value programmed into the RS485ADRMATCH
 *       register.  When a matching address character is detected it will be
 *       pushed onto the RXFIFO along with the parity bit, and the receiver
 *       will be automatically enabled.
 *
 *       When an address byte which does not match the RS485ADRMATCH value
 *       is received, the receiver will be automatically disabled in
 *       hardware.
 *
 *     RS-485/EIA-485 Auto Direction Control -- Supported
 *
 *       Allow the transmitter to automatically control the state of the DIR
 *       pin as a direction control output signal.  The DIR pin will be
 *       asserted (driven LOW) when the CPU writes data into the TXFIFO. The
 *       pin will be de-asserted (driven HIGH) once the last bit of data has
 *       been transmitted.
 *
 *     RS485/EIA-485 driver delay time -- Supported
 *
 *       The driver delay time is the delay between the last stop bit leaving
 *       the TXFIFO and the de-assertion of the DIR pin. This delay time can
 *       be programmed in the 8-bit RS485DLY register. The delay time is in
 *       periods of the baud clock.
 *
 *     RS485/EIA-485 output inversion -- Supported
 *
 *       The polarity of the direction control signal on the DIR pin can be
 *       reversed by programming bit 5 in the RS485CTRL register.
 *
 ****************************************************************************/

#ifdef HAVE_RS485
static inline int up_set_rs485_mode(struct up_dev_s *priv,
                                    const struct serial_rs485 *mode)
{
  irqstate_t flags;
  uint32_t   regval;
  uint64_t   tmp;

  DEBUGASSERT(priv && mode);
  flags = enter_critical_section();

  /* Are we enabling or disabling RS-485 support? */

  if ((mode->flags & SER_RS485_ENABLED) == 0)
    {
      /* Disable all RS-485 features */

      up_serialout(priv, LPC43_UART_RS485CTRL_OFFSET, 0);
    }
  else
    {
      /* Set the RS-485/EIA-485 Control register:
       *
       *  NMMEN 0 = Normal Multidrop Mode (NMM) disabled
       *  RXDIS 0 = Receiver is not disabled
       *  AADEN 0 = Auto Address Detect (ADD) is disabled
       *  DCTRL 1 = Auto Direction Control is enabled
       *  OINV  ? = Value controlle by user mode settings
       *  SEL   ? = Value controlled by user mode settings
       */

      regval = UART_RS485CTRL_DCTRL;

      /* Logic levels are controlled by the SER_RS485_RTS_ON_SEND and
       * SER_RS485_RTS_AFTER_SEND bits in the mode flags.
       * SER_RS485_RTS_AFTER_SEND is ignored.
       *
       * By default, DIR will go logic low on send, but this can
       * be inverted.
       */

      if ((mode->flags & SER_RS485_RTS_ON_SEND) != 0)
        {
          regval |= UART_RS485CTRL_OINV;
        }

#ifdef BOARD_LPC43_UART1_DTRDIR
      if (priv->dtrdir)
        {
          /* If we are using DTR for direction then ensure the H/W is
           * configured correctly.
           */

          regval |= UART_RS485CTRL_SEL;
        }
#endif

      up_serialout(priv, LPC43_UART_RS485CTRL_OFFSET, regval);

      /* We only have control of the delay after send.  Time provided
       * is in milliseconds; this must be converted to the baud clock.
       * The baud clock should be 16 times the currently selected BAUD.
       *
       * Eg. Given BAUD=115,200, then a delay of n milliseconds would be:
       *
       *  115,200 * n / 1000 = 11525 clocks.
       *
       *    n=1: 115 (OK)
       *    n=2: 230 (OK)
       *    n>2: Out of range
       *
       * The valid range is 0 to 255 bit times.
       *
       * REVISIT:  Is this time in bit time or in terms of the baud clock?
       * The text says either interchange-ably.  Baud clock is 16 x BAUD
       * and a bit time is 1/BAUD.  The value range of values 0-255 suggests
       * BAUD bit times, not the baud clock.
       */

      if (mode->delay_rts_after_send > 0)
        {
          regval = 0;
        }
      else
        {
          tmp = ((priv->baud << 4) * mode->delay_rts_after_send) / 1000;
          if (tmp > 255)
            {
              regval = 255;
            }
          else
            {
              regval = (uint32_t)tmp;
            }
        }

      up_serialout(priv, LPC43_UART_RS485DLY_OFFSET, regval);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: up_get_rs485_mode
 *
 * Description:
 *   Handle LPC43xx USART0,2,3 RS485 mode get ioctl (TIOCGRS485) to get the
 *   current RS-485 mode.
 *
 ****************************************************************************/

#ifdef HAVE_RS485
static inline int up_get_rs485_mode(struct up_dev_s *priv,
                                    struct serial_rs485 *mode)
{
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(priv && mode);
  flags = enter_critical_section();

  /* Assume disabled */

  memset(mode, 0, sizeof(struct serial_rs485));

  /* If RS-485 mode is enabled, then the DCTRL will be set in the RS485CTRL
   * register.
   */

  regval = up_serialin(priv, LPC43_UART_RS485CTRL_OFFSET);
  if ((regval & UART_RS485CTRL_DCTRL) != 0)
    {
      /* RS-485 mode is enabled */

      mode->flags = SER_RS485_ENABLED;

      /* Check if DIR is inverted */

      if ((regval & UART_RS485CTRL_OINV) != 0)
        {
          mode->flags = (SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND);
        }
      else
        {
          mode->flags = SER_RS485_ENABLED;
        }

      /* We only have control of the delay after send.  Time must be
       * returned in milliseconds; this must be converted from the baud
       * clock. (The baud clock should be 16 times the currently
       * selected BAUD.)
       *
       *   msec = 1000 * dly / baud
       */

       regval = up_serialin(priv, LPC43_UART_RS485DLY_OFFSET);
       mode->delay_rts_after_send = (1000 * regval) / priv->baud;
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct up_dev_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Other termios fields are not yet returned.
         * Note that only cfsetospeed is not necessary because we have
         * knowledge that only one speed is supported.
         */

        cfsetispeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Handle other termios settings.
         * Note that only cfgetispeed is used because we have knowledge
         * that only one speed is supported.
         */

        priv->baud = cfgetispeed(termiosp);
        lpc43_setbaud(priv->uartbase, priv->basefreq, priv->baud);
      }
      break;
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = enter_critical_section();
        up_enablebreaks(priv, true);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = enter_critical_section();
        up_enablebreaks(priv, false);
        leave_critical_section(flags);
      }
      break;

#ifdef HAVE_RS485
    case TIOCSRS485:  /* Set RS485 mode, arg: pointer to struct serial_rs485 */
      {
        ret = up_set_rs485_mode(
          priv, (const struct serial_rs485 *)((uintptr_t)arg));
      }
      break;

    case TIOCGRS485:  /* Get RS485 mode, arg: pointer to struct serial_rs485 */
      {
        ret = up_get_rs485_mode(
          priv, (struct serial_rs485 *)((uintptr_t)arg));
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
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rbr;

  *status = up_serialin(priv, LPC43_UART_LSR_OFFSET);
  rbr     = up_serialin(priv, LPC43_UART_RBR_OFFSET);
  return rbr;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_RBRIE;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_RBRIE;
    }

  up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, LPC43_UART_LSR_OFFSET) & UART_LSR_RDR) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, LPC43_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_THREIE;
      up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_THREIE;
      up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, LPC43_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, LPC43_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by up_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
{
  /* Configure all UARTs (except the CONSOLE UART) and disable interrupts */

#ifdef CONFIG_LPC43_USART0
#ifndef CONFIG_USART0_SERIAL_CONSOLE
  lpc43_usart0_setup();
#endif
  up_disableuartint(&g_usart0priv, NULL);
#endif

#ifdef CONFIG_LPC43_UART1
#ifndef CONFIG_UART1_SERIAL_CONSOLE
  lpc43_uart1_setup();
#endif
  up_disableuartint(&g_uart1priv, NULL);
#endif

#ifdef CONFIG_LPC43_USART2
#ifndef CONFIG_USART2_SERIAL_CONSOLE
  lpc43_usart2_setup();
#endif
  up_disableuartint(&g_usart2priv, NULL);
#endif

#ifdef CONFIG_LPC43_USART3
#ifndef CONFIG_USART3_SERIAL_CONSOLE
  lpc43_usart3_setup();
#endif
  up_disableuartint(&g_usart3priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
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
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV);
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
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint32_t ier;
  up_disableuartint(priv, &ier);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#ifdef HAVE_SERIAL_CONSOLE
  up_restoreuartint(priv, ier);
#endif

  return ch;
}

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
#ifdef HAVE_UART
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
