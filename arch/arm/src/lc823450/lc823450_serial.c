/****************************************************************************
 * arch/arm/src/lc823450/lc823450_serial.c
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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "lc823450_dma.h"
#include "lc823450_serial.h"
#include "lc823450_syscontrol.h"
#include "lc823450_gpio.h"

#ifdef CONFIG_DEV_CONSOLE_SWITCH
int g_console_disable;
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1-7?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART0-5 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0port /* UART0 is console */
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port /* UART1 is console */
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart2port /* UART2 is console */
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_LC823450_UART0)
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_LC823450_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_LC823450_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-7 excluding the console UART. */

#if defined(CONFIG_LC823450_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_LC823450_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_LC823450_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART1-7. It can't be UART0 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-7 could also be the
 * console.
 */

#if defined(CONFIG_LC823450_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_LC823450_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART2-7. It can't be UART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART 2-7 could also be the console.
 */

#if defined(CONFIG_LC823450_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS3_DEV           g_uart2port /* UART2 is ttyS3 */
#  define UART2_ASSIGNED      1
#endif

#define CTL_CLK XT1OSC_CLK
#define UART_FIFO_SIZE 16

#if defined(CONFIG_HSUART)
#  define HS_DMAACT_STOP1     0
#  define HS_DMAACT_STOP2     1
#  define HS_DMAACT_ACT1      2
#  define HS_DMAACT_ACT2      3
#endif

#ifndef MIN
#  define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase; /* Base address of UART registers */
  uint32_t baud;     /* Configured baud */
  uint32_t im;       /* Saved IM value */
  uint8_t  irq;      /* IRQ associated with this UART */
  uint8_t  parity;   /* 0=none, 1=odd, 2=even */
  uint8_t  bits;     /* Number of bits (7 or 8) */
  bool    stopbits2; /* true: Configure with 2 stop bits instead of 1 */
  uint32_t rowe;     /* receive register over write error */
  uint32_t pe;       /* parity error */
  uint32_t fe;       /* framing error */
  uint32_t rxowe;    /* RX FIFO over write error */
  bool iflow;        /* input flow control (RTS) enabled */
  bool oflow;        /* output flow control (CTS) enabled */
#ifdef CONFIG_HSUART
  DMA_HANDLE       hrxdma;
  DMA_HANDLE       htxdma;
  sem_t rxdma_wait;
  sem_t rxpkt_wait;
  sem_t txdma_wait;
#endif /* CONFIG_HSUART */
  spinlock_t lock;
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
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);
#ifdef CONFIG_HSUART
static int  up_hs_attach(struct uart_dev_s *dev);
static void up_hs_detach(struct uart_dev_s *dev);
static void uart_dma_callback(DMA_HANDLE hdma, void *arg, int result);
static void uart_rxdma_callback(DMA_HANDLE hdma, void *arg, int result);
static void  up_hs_dmasetup(void);
static int up_hs_receive(struct uart_dev_s *dev, char *buf, int buflen);
static int up_hs_send(struct uart_dev_s *dev, const char *buf, int buflen);
#endif

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
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
#ifdef CONFIG_HSUART
  .hs_attach      = up_hs_attach,
  .hs_detach      = up_hs_detach,
  .hs_receive     = up_hs_receive,
  .hs_send        = up_hs_send,
#endif
};

/* I/O buffers */

#ifdef CONFIG_LC823450_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_LC823450_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_LC823450_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

/* This describes the state of the Stellaris uart0 port. */

#ifdef CONFIG_LC823450_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = LC823450_UART0_REGBASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = LC823450_IRQ_UART0,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
  .stopbits2      = CONFIG_UART0_2STOP,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif

/* This describes the state of the Stellaris uart1 port. */

#ifdef CONFIG_LC823450_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = LC823450_UART1_REGBASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = LC823450_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stopbits2      = CONFIG_UART1_2STOP,
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

/* This describes the state of the Stellaris uart2 port. */

#ifdef CONFIG_LC823450_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = LC823450_UART2_REGBASE,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = LC823450_IRQ_UART2,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
  .stopbits2      = CONFIG_UART2_2STOP,
};

static uart_dev_t g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_UART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};
#endif

#ifdef CONFIG_HSUART
static int hs_recstart;
static int hs_dmaact;
#endif

/****************************************************************************
 * Private Functions
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

static inline void up_serialout(struct up_dev_s *priv,
                                int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *im)
{
  /* Return the current interrupt mask value */

  if (im != NULL)
    {
      *im = priv->im;
    }

  /* Disable all interrupts */

  priv->im = 0;
  up_serialout(priv, UART_UIEN, 0);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t im)
{
  priv->im = im;
  up_serialout(priv, UART_UIEN, im);
}

/****************************************************************************
 * Name: up_waittxnotfull
 ****************************************************************************/

static inline void up_waittxnotfull(struct up_dev_s *priv)
{
  int tmp;

  /* Limit how long we will wait for the TX available condition */

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      /* Check Tx FIFO is full */

      if ((up_serialin(priv, UART_USR) & UART_USR_TXFULL) == 0)
        {
          /* The Tx FIFO is not full... return */

          break;
        }

      up_udelay(1);
    }

  /* If we get here, then the wait has timed out and the Tx FIFO remains
   * full.
   */
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t ctl;
  int min_diff = INT_MAX;
  int udiv = -1;
  int tmp_reg;
  int real_baud;
  int i;

  /* Note:  The logic here depends on the fact that that the UART module
   * was enabled and the GPIOs were configured in up_lowsetup().
   */

  /* Disable the UART by clearing the UARTEN bit in the UART CTL register */

  ctl = up_serialin(priv, UART_UCM);
  ctl &= ~(UART_UCM_RE | UART_UCM_TE);
  up_serialout(priv, UART_UCM, ctl);

  /* Calculate BAUD rate from the SYS clock:
   * REGVAL = 65536 - (ControllerCLK / (UDIV(7~15) + 1) * baudrate)
   */

  /* search best divider setting */

  for (i = 7; i <= 15; i++)
    {
      tmp_reg = 65536 - (CTL_CLK / ((i + 1)  * priv->baud));
      if (tmp_reg >= 65536)
        {
          continue;
        }

      real_baud = CTL_CLK / ((i + 1)  * (65536 - tmp_reg));
      if (min_diff > abs(real_baud - priv->baud))
        {
          min_diff = abs(real_baud - priv->baud);
          udiv = i;
        }
    }

  if (udiv < 0)
    {
      serr("ERROR: baud = %" PRId32 "\n", priv->baud);
      return -EINVAL;
    }

  tmp_reg = 65536 - (CTL_CLK / ((udiv + 1)  * priv->baud));
  real_baud = CTL_CLK / ((udiv + 1)  * (65536 - tmp_reg));

  up_serialout(priv, UART_UDIV, udiv);
  up_serialout(priv, UART_UBR, tmp_reg);

  /* Set up the UMD register */

  ctl = up_serialin(priv, UART_UMD);
  switch (priv->bits)
    {
      case 7:
        ctl &= ~UART_UMD_CL;
        break;

      case 8:
        ctl |= UART_UMD_CL;
        break;

      default:
        serr("ERROR: bits = %d\n", priv->bits);
        return -EINVAL;
    }

  ctl &= ~(UART_UMD_PS0 | UART_UMD_PS1);
  switch (priv->parity)
    {
      case 0: /* non */
        break;

      case 1: /* odd */
        ctl |= UART_UMD_PS0;
        break;

      case 2: /* even */
        ctl |= UART_UMD_PS1;
        break;

      default:
        serr("ERROR: bits = %d\n", priv->bits);
        return -EINVAL;
    }

  ctl &= ~UART_UMD_STL;
  if (priv->stopbits2)
    {
      ctl |= UART_UMD_STL;
    }

  if (priv->oflow)
    {
      ctl |= UART_UMD_CTSEN;
    }
  else
    {
      ctl &= ~UART_UMD_CTSEN;
    }

  if (priv->iflow)
    {
      ctl |= UART_UMD_RTSEN;
    }
  else
    {
      ctl &= ~UART_UMD_RTSEN;
    }

  up_serialout(priv, UART_UMD, ctl);

  /* Set the UART to interrupt whenever the TX FIFO is almost empty or when
   * any character is received.
   */

  ctl  = up_serialin(priv, UART_UIEN);
  ctl |= UART_UIEN_UARTRF_IEN
      | UART_UIEN_UARTTF_IEN
      | UART_UIEN_ROWE_IEN
      | UART_UIEN_PE_IEN
      | UART_UIEN_FE_IEN
      | UART_UIEN_RXOWE_IEN;
  up_serialout(priv, UART_UIEN, ctl);

  /* Enable the FIFOs */

  ctl = up_serialin(priv, UART_USFC);
  ctl |= (UART_USFC_RXFF_EN | UART_USFC_TXFF_EN);
  up_serialout(priv, UART_USFC, ctl);

  /* Enable Rx, Tx, and the UART */

  ctl = up_serialin(priv, UART_UCM);
  ctl |= (UART_UCM_RE | UART_UCM_TE);
  up_serialout(priv, UART_UCM, ctl);

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the the setup() method is called,
 *   however, the serial console may operate in a non-interrupt driven mode
 *   during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method
 *   (unless the hardware supports multiple levels of interrupt enabling).
 *   The RX and TX interrupts are not enabled until the txint() and rxint()
 *   methods are called.
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
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
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
 *   This is the UART interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the appropriate
 *   uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev;
  struct up_dev_s   *priv;
  uint32_t           mis;

  dev = (struct uart_dev_s *)arg;
  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Get the masked UART status and clear the pending interrupts. */

  mis = up_serialin(priv, UART_UISR);

  /* Handle incoming, receive bytes (with or without timeout) */

  /* Rx buffer not empty ... process incoming bytes */

  if (mis & UART_UISR_UARTRF)
    {
      uart_recvchars(dev);
    }

  if (mis & UART_UISR_ROWE)
    {
      priv->rowe++;
    }

  if (mis & UART_UISR_PE)
    {
      priv->pe++;
    }

  if (mis & UART_UISR_FE)
    {
      priv->fe++;
    }

  if (mis & UART_UISR_RXOWE)
    {
      priv->rxowe++;
    }

  /* Handle outgoing, transmit bytes */

  /* Tx FIFO not full ... process outgoing bytes */

  if (mis & UART_UISR_UARTTF)
    {
      uart_xmitchars(dev);
    }

  up_serialout(priv, UART_UISR, mis);

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/
#ifdef CONFIG_SERIAL_TERMIOS
static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int                ret   = OK;
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;

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

      case TCGETS:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* Note that since we only support 8/9 bit modes and
           * there is no way to report 9-bit mode, we always claim 8.
           */

          termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                              ((priv->parity == 1) ? PARODD : 0) |
                              ((priv->stopbits2) ? CSTOPB : 0) |
#ifdef CONFIG_SERIAL_OFLOWCONTROL
                              ((priv->oflow) ? CCTS_OFLOW : 0) |
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
                              ((priv->iflow) ? CRTS_IFLOW : 0) |
#endif
                              CS8;

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

          /* Perform some sanity checks before accepting any changes */

          if (termiosp->c_cflag & PARENB)
            {
              priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              priv->parity = 0;
            }

          priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif

          /* Note that since there is no way to request 9-bit mode
           * and no way to support 5/6/7-bit modes, we ignore them
           * all here.
           */

          /* Note that only cfgetispeed is used because we have knowledge
           * that only one speed is supported.
           */

          priv->baud = cfgetispeed(termiosp);

          up_setup(dev);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#else
static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}
#endif

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
  uint32_t rxd;

  /* Get the Rx byte + 4 bits of error information.  Return those in status */

  rxd     = up_serialin(priv, UART_USRF);
  *status = rxd;

  /* The lower 8bits of the Rx data is the actual received byte */

  return rxd & 0xff;
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
      /* Receive an interrupt when their is anything in the Rx FIFO (or an Rx
       * timeout occurs.
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_UIEN_UARTRF_IEN;
#endif
    }
  else
    {
      priv->im &= ~UART_UIEN_UARTRF_IEN;
    }

  up_serialout(priv, UART_UIEN, priv->im);
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

#ifdef CONFIG_DEV_CONSOLE_SWITCH
  if (g_console_disable && dev == &CONSOLE_DEV)
    {
      return false;
    }
#endif /* CONFIG_DEV_CONSOLE_SWITCH */

  return ((up_serialin(priv, UART_USR) & UART_USR_RXEMP) == 0);
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

#ifdef CONFIG_DEV_CONSOLE_SWITCH
  if (g_console_disable && dev == &CONSOLE_DEV)
    {
      return;
    }
#endif /* CONFIG_DEV_CONSOLE_SWITCH */

  up_serialout(priv, UART_USTF, (uint32_t)ch);
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

  flags = spin_lock_irqsave(&priv->lock);
  if (enable)
    {
      /* Set to receive an interrupt when the TX fifo is half emptied */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_UIEN_UARTTF_IEN;
      up_serialout(priv, UART_UIEN, priv->im);

      /* The serial driver wants an interrupt here, but will not get get
       * one unless we "prime the pump."  I believe that this is because
       * behave like a level interrupt and the Stellaris interrupts behave
       * (at least by default) like edge interrupts.
       *
       * In any event, faking a TX interrupt here solves the problem;
       * Call uart_xmitchars() just as would have been done if we received
       * the TX interrupt.
       */

      spin_unlock_irqrestore(&priv->lock, flags);
      uart_xmitchars(dev);
      flags = spin_lock_irqsave(&priv->lock);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->im &= ~UART_UIEN_UARTTF_IEN;
      up_serialout(priv, UART_UIEN, priv->im);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
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

#ifdef CONFIG_DEV_CONSOLE_SWITCH
  if (g_console_disable && dev == &CONSOLE_DEV)
    {
      return true;
    }
#endif /* CONFIG_DEV_CONSOLE_SWITCH */

#ifndef CONFIG_SMP
  return ((up_serialin(priv, UART_USR) & UART_USR_TXFULL) == 0);
#else
  return (UART_USFS_TXFF_LV(up_serialin(priv, UART_USFS)) <= 1);
#endif
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

#ifdef CONFIG_DEV_CONSOLE_SWITCH
  if (g_console_disable && dev == &CONSOLE_DEV)
    {
      return 0;
    }
#endif /* CONFIG_DEV_CONSOLE_SWITCH */

  return ((up_serialin(priv, UART_USR) & UART_USR_TXEMP) != 0);
}

#ifdef CONFIG_HSUART

/****************************************************************************
 * Name: up_hs_attach
 ****************************************************************************/

static int  up_hs_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  dev->recv.tail = 0;
  dev->recv.head = 0;
  hs_dmaact = HS_DMAACT_STOP1;

  up_hs_dmasetup();

  hs_recstart = 1;

  return OK;
}

/****************************************************************************
 * Name: up_hs_detach
 ****************************************************************************/

static void  up_hs_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  hs_recstart = 0;
  lc823450_dmastop(priv->htxdma);
  lc823450_dmastop(priv->hrxdma);
  hs_dmaact = 0;

  return;
}

/****************************************************************************
 * Name: uart_dma_callback
 ****************************************************************************/

static void uart_dma_callback(DMA_HANDLE hdma, void *arg, int result)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  sem_t *waitsem = &priv->txdma_wait;
  nxsem_post(waitsem);
  uart_datasent(dev);
}

/****************************************************************************
 * Name: uart_rxdma_callback
 ****************************************************************************/

static void uart_rxdma_callback(DMA_HANDLE hdma, void *arg, int result)
{
  if (hs_dmaact == HS_DMAACT_ACT1)
    {
      hs_dmaact = HS_DMAACT_STOP2;
    }
  else
    {
      hs_dmaact = HS_DMAACT_STOP1;
    }

  hsuart_wdtimer();
  up_hs_dmasetup();
}

/****************************************************************************
 * Name: up_hs_dmasetup
 ****************************************************************************/

static void  up_hs_dmasetup()
{
  irqstate_t flags;

  flags = enter_critical_section();

  switch (hs_dmaact)
    {
      case HS_DMAACT_ACT1:
      case HS_DMAACT_ACT2:
        break;

      case HS_DMAACT_STOP1:
        if (g_uart1port.recv.tail > 0 &&
            g_uart1port.recv.tail < CONFIG_UART1_RXBUFSIZE / 2)
          {
            break;
          }

        lc823450_dmasetup(g_uart1priv.hrxdma,
                          LC823450_DMA_SRCWIDTH_BYTE |
                          LC823450_DMA_DSTWIDTH_BYTE |
                          LC823450_DMA_DSTINC,
                          LC823450_UART1_REGBASE + UART_USRF,
                          (uint32_t)g_uart1rxbuffer,
                          CONFIG_UART1_RXBUFSIZE / 2);

        lc823450_dmastart(g_uart1priv.hrxdma, uart_rxdma_callback, NULL);
        hs_dmaact = HS_DMAACT_ACT1;
        break;

      case HS_DMAACT_STOP2:
        if (g_uart1port.recv.tail > CONFIG_UART1_RXBUFSIZE / 2)
          {
            break;
          }

        lc823450_dmasetup(g_uart1priv.hrxdma,
                          LC823450_DMA_SRCWIDTH_BYTE |
                          LC823450_DMA_DSTWIDTH_BYTE |
                          LC823450_DMA_DSTINC,
                          LC823450_UART1_REGBASE + UART_USRF,
                          (uint32_t)(g_uart1rxbuffer + CONFIG_UART1_RXBUFSIZE
                                     / 2),
                          CONFIG_UART1_RXBUFSIZE / 2);

        lc823450_dmastart(g_uart1priv.hrxdma, uart_rxdma_callback, NULL);
        hs_dmaact = HS_DMAACT_ACT2;
        break;
    };

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_hs_receive
 ****************************************************************************/

static int up_hs_receive(struct uart_dev_s *dev, char *buf, int buflen)
{
  int len = 0;
  int dmalen;
  irqstate_t flags;

  while ((dmalen = up_hsuart_get_rbufsize(dev)) == 0)
    ;
  len = MIN(dmalen, buflen);

  flags = enter_critical_section();
  if (len + dev->recv.tail > dev->recv.size)
    {
      int len2;

      len2 = dev->recv.size - dev->recv.tail;
      memcpy(buf, dev->recv.buffer + dev->recv.tail, len2);
      memcpy(buf + len2, dev->recv.buffer, len - len2);
      dev->recv.tail = len - len2;
    }
  else
    {
      memcpy(buf, dev->recv.buffer + dev->recv.tail, len);
      dev->recv.tail += len;
    }

  up_hs_dmasetup();
  leave_critical_section(flags);
  return len;
}

/****************************************************************************
 * Name: up_hs_send
 ****************************************************************************/

static int up_hs_send(struct uart_dev_s *dev, const char *buf, int buflen)
{
  int len;
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

retry:
  nxsem_wait(&priv->txdma_wait);

  /* If buflen <= FIFO space, write it by PIO. */

  if (buflen <=
    UART_FIFO_SIZE - UART_USFS_TXFF_LV(up_serialin(priv, UART_USFS)))
    {
      int i;
      for (i = 0; i < buflen; i++)
        up_serialout(priv, UART_USTF, buf[i]);
      nxsem_post(&priv->txdma_wait);
      return buflen;
    }

  len = MIN(buflen, LC823450_DMA_MAX_TRANSSIZE);
  len = MIN(len, dev->xmit.size);

  memcpy(dev->xmit.buffer, buf, len);

  lc823450_dmasetup(priv->htxdma,
                    LC823450_DMA_SRCWIDTH_BYTE |
                    LC823450_DMA_DSTWIDTH_BYTE |
                    LC823450_DMA_SRCINC,
                    (uint32_t)dev->xmit.buffer,
                    priv->uartbase + UART_USTF,
                    len);
  lc823450_dmastart(priv->htxdma, uart_dma_callback, dev);

  /* BT stack may not handle to short write */

  if (len != buflen)
    {
      buflen -= len;
      buf += len;
      goto retry;
    }

  return len;
}
#endif /* CONFIG_HSUART */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm_serialinit.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the UARTs was performed in
   * up_lowsetup
   */

  /* workaround: force clock enable */

#if defined(CONFIG_LC823450_UART0)
  modifyreg32(MCLKCNTAPB,
              0,
              MCLKCNTAPB_UART0_CLKEN | MCLKCNTAPB_UART0IF_CLKEN);
  modifyreg32(MRSTCNTAPB,
              0,
              MRSTCNTAPB_UART0_RSTB);
#endif
#if defined(CONFIG_LC823450_UART1)
  modifyreg32(MCLKCNTAPB,
              0,
              MCLKCNTAPB_UART1_CLKEN | MCLKCNTAPB_UART1IF_CLKEN);
  modifyreg32(MRSTCNTAPB,
              0,
              MRSTCNTAPB_UART1_RSTB);
#endif
#if defined(CONFIG_LC823450_UART2)
  modifyreg32(MCLKCNTAPB,
              0,
              MCLKCNTAPB_UART2_CLKEN | MCLKCNTAPB_UART2IF_CLKEN);
  modifyreg32(MRSTCNTAPB,
              0,
              MRSTCNTAPB_UART2_RSTB);
#endif

  /* Disable all UARTS */

  up_disableuartint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableuartint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableuartint(TTYS2_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

int hsuart_register(const char *path, uart_dev_t *dev);

void arm_serialinit(void)
{
  /* Register the console */

  uart_register("/dev/console", &CONSOLE_DEV);

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#ifdef CONFIG_HSUART
  nxsem_init(&g_uart1priv.txdma_wait, 0, 1);
  g_uart1priv.htxdma = lc823450_dmachannel(DMA_CHANNEL_UART1TX);
  lc823450_dmarequest(g_uart1priv.htxdma, DMA_REQUEST_UART1TX);

  nxsem_init(&g_uart1priv.rxdma_wait, 0, 0);
  g_uart1priv.hrxdma = lc823450_dmachannel(DMA_CHANNEL_UART1RX);
  lc823450_dmarequest(g_uart1priv.hrxdma, DMA_REQUEST_UART1RX);

  up_serialout(&g_uart1priv, UART_UDMA,
               UART_UDMA_RREQ_EN | UART_UDMA_TREQ_EN);
  hsuart_register("/dev/ttyHS1", &TTYS1_DEV);
#endif /* CONFIG_HSUART */
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
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
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint32_t im;

#ifdef CONFIG_DEV_CONSOLE_SWITCH
  if (g_console_disable)
    {
      return ch;
    }
#endif /* CONFIG_DEV_CONSOLE_SWITCH */

  up_disableuartint(priv, &im);
  up_waittxnotfull(priv);
  up_serialout(priv, UART_USTF, (uint32_t)ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxnotfull(priv);
      up_serialout(priv, UART_USTF, (uint32_t)'\r');
    }

  up_waittxnotfull(priv);
  up_restoreuartint(priv, im);
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
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */

#ifdef CONFIG_DEV_CONSOLE_SWITCH
void up_console_disable(int disable)
{
  uint32_t clkmask;
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  clkmask = MCLKCNTAPB_UART0_CLKEN | MCLKCNTAPB_UART0IF_CLKEN;
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  clkmask = MCLKCNTAPB_UART1_CLKEN | MCLKCNTAPB_UART1IF_CLKEN;
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  clkmask = MCLKCNTAPB_UART2_CLKEN | MCLKCNTAPB_UART2IF_CLKEN;
#else
  clkmask = 0;
#endif

  if (disable)
    {
      llinfo("disable console\n");
      g_console_disable = 1;
      modifyreg32(MCLKCNTAPB, clkmask, 0);

      /* remux to gpio */

      lc823450_gpio_mux(GPIO_PORT5 | GPIO_PIN6 | GPIO_PULLDOWN | GPIO_MUX0);
      lc823450_gpio_mux(GPIO_PORT5 | GPIO_PIN7 | GPIO_PULLDOWN | GPIO_MUX0);
    }
  else
    {
      llinfo("enable console\n");

      /* remux to uart */

      lc823450_gpio_mux(GPIO_PORT5 | GPIO_PIN6 | GPIO_PULLDOWN | GPIO_MUX3);
      lc823450_gpio_mux(GPIO_PORT5 | GPIO_PIN7 | GPIO_PULLDOWN | GPIO_MUX3);

      modifyreg32(MCLKCNTAPB, 0, clkmask);
      g_console_disable = 0;
    }
}
#endif /* CONFIG_DEV_CONSOLE_SWITCH */

#ifdef CONFIG_HSUART

/****************************************************************************
 * Name: hsuart_wdtimer
 ****************************************************************************/

void hsuart_wdtimer(void)
{
  int newhead = 0;

  if (!hs_recstart)
    {
      return;
    }

  switch (hs_dmaact)
    {
      case HS_DMAACT_ACT1:
        newhead = CONFIG_UART1_RXBUFSIZE /
          2 - lc823450_dmaremain(g_uart1priv.hrxdma);
        break;
      case HS_DMAACT_ACT2:
        newhead = CONFIG_UART1_RXBUFSIZE  -
          lc823450_dmaremain(g_uart1priv.hrxdma);
        break;
      case HS_DMAACT_STOP1:
        newhead = 0;
        break;
      case HS_DMAACT_STOP2:
        newhead = CONFIG_UART1_RXBUFSIZE / 2;
        break;
    };

  if (g_uart1port.recv.head != newhead)
    {
      if (newhead == CONFIG_UART1_RXBUFSIZE)
        {
          newhead = 0;
        }

      if (g_uart1port.recv.tail == CONFIG_UART1_RXBUFSIZE)
        {
          g_uart1port.recv.tail = 0;
        }

      g_uart1port.recv.head = newhead;
      uart_datareceived(&g_uart1port);
    }
}

/****************************************************************************
 * Name: up_hs_get_rbufsize
 ****************************************************************************/

int up_hsuart_get_rbufsize(struct uart_dev_s *dev)
{
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();
  if (dev->recv.tail <= dev->recv.head)
    {
      ret = dev->recv.head - dev->recv.tail;
    }
  else
    {
      ret = dev->recv.size - dev->recv.tail + dev->recv.head;
    }

  leave_critical_section(flags);
  return ret;
}

#endif /* CONFIG_HSUART */
