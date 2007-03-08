/************************************************************
 * up_serial.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs.h>
#include <arch/serial.h>
#include "c5471.h"
#include "os_internal.h"
#include "up_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

#define BASE_BAUD     115200

#if defined(CONFIG_UART_IRDA_HWFLOWCONTROL) || defined(CONFIG_UART_MODEM_HWFLOWCONTROL)
# define CONFIG_UART_HWFLOWCONTROL
#endif

/************************************************************
 * Private Types
 ************************************************************/

/* This is the internal structure for each serial port's state. */

struct uart_regs_s
{
  uint32  ier;
  uint32  lcr;
  uint32  fcr;
  uint32  efr;
  uint32  tcr;
};

struct uart_buffer_s
{
  int   head;
  int   tail;
  int   size;
  char *buffer;
};

struct up_dev_s
{
  int                  open_count;	/* The number of times
					* the device has been opened */
  unsigned int         uartbase;	/* Base address of UART
					 * registers */
  unsigned int         baud_base;	/* Base baud for conversions */
  unsigned int         baud;		/* Configured baud */
  ubyte                xmit_fifo_size;	/* Size of transmit FIFO */
  ubyte                irq;		/* IRQ associated with
					 * this UART */
  ubyte                parity;		/* 0=none, 1=odd, 2=even */
  ubyte                bits;		/* Number of bits (7 or 8) */
#ifdef CONFIG_UART_HWFLOWCONTROL
  boolean              flowcontrol;	/* TRUE: Hardware flow control
					 * is enabled. */
#endif
  boolean              stopbits2;	/* TRUE: Configure with 2
					 * stop bits instead of 1 */
  boolean              xmitwaiting;	/* TRUE: User is waiting
					 * for space in xmit.buffer */
  boolean              recvwaiting;	/* TRUE: User is waiting
					 * for space in recv.buffer */
  boolean              isconsole;       /* TRUE: This is the serial console */
  sem_t                closesem;	/* Looks out new opens while
					 * close is in progress */
  sem_t                xmitsem;		/* Used to wakeup user waiting
					 * for space in xmit.buffer */
  sem_t                recvsem;		/* Used to wakeup user waiting
					 * for sapce in recv.buffer */
  struct uart_buffer_s xmit;		/* Describes transmit buffer */
  struct uart_buffer_s recv;		/* Describes receive buffer */
  struct uart_regs_s   regs;		/* Shadow copy of readonly regs */
};
typedef struct up_dev_s up_dev_t;

/************************************************************
 * Private Function Prototypes
 ************************************************************/

static int     up_open(struct file *filep);
static int     up_close(struct file *filep);
static ssize_t up_read(struct file *filep, char *buffer, size_t buflen);
static ssize_t up_write(struct file *filep, const char *buffer, size_t buflen);
static int     up_ioctl(struct file *filep, int cmd, unsigned long arg);
static void    up_uartsetup(up_dev_t *dev);

/************************************************************
 * Private Variables
 ************************************************************/

struct file_operations g_serialops =
{
  .open  = up_open,
  .close = up_close,
  .read  = up_read,
  .write = up_write,
  .ioctl = up_ioctl,
};

/* This describes the state of the C5471 serial IRDA port. */

static up_dev_t g_irdaport =
{
  .xmit_fifo_size = UART_IRDA_XMIT_FIFO_SIZE,
  .baud_base      = BASE_BAUD,
  .irq            = C5471_IRQ_UART_IRDA,
  .uartbase       = UART_IRDA_BASE,
  .baud           = CONFIG_UART_IRDA_BAUD,
  .parity         = CONFIG_UART_IRDA_PARITY,
  .bits           = CONFIG_UART_IRDA_BITS,
#ifdef CONFIG_UART_IRDA_HWFLOWCONTROL
  .flowcontrol    = TRUE,
#endif
  .stopbits2      = CONFIG_UART_IRDA_2STOP,
};

/* This describes the state of the C5471 serial Modem port. */

static up_dev_t g_modemport =
{
  .xmit_fifo_size = UART_XMIT_FIFO_SIZE,
  .baud_base      = BASE_BAUD,
  .irq            = C5471_IRQ_UART,
  .uartbase       = UART_MODEM_BASE,
  .baud           = CONFIG_UART_MODEM_BAUD,
  .parity         = CONFIG_UART_MODEM_PARITY,
  .bits           = CONFIG_UART_MODEM_BITS,
#ifdef CONFIG_UART_MODEM_HWFLOWCONTROL
  .flowcontrol    = TRUE,
#endif
  .stopbits2      = CONFIG_UART_MODEM_2STOP,
};

/* I/O buffers */

static char g_irdarxbuffer[CONFIG_UART_IRDA_RXBUFSIZE];
static char g_irdatxbuffer[CONFIG_UART_IRDA_TXBUFSIZE];
static char g_modemrxbuffer[CONFIG_UART_IRDA_RXBUFSIZE];
static char g_modemtxbuffer[CONFIG_UART_MODEM_TXBUFSIZE];

/* Now, which one with be tty0/console and which tty1? */

#ifdef CONFIG_SERIAL_IRDA_CONSOLE
# define CONSOLE_DEV     g_irdaport
# define TTYS0_DEV       g_irdaport
# define TTYS0_RXBUFFER  g_irdarxbuffer
# define TTYS0_RXBUFSIZE CONFIG_UART_IRDA_RXBUFSIZE
# define TTYS0_TXBUFFER  g_irdatxbuffer
# define TTYS0_TXBUFSIZE CONFIG_UART_IRDA_TXBUFSIZE
# define TTYS1_DEV       g_modemport
# define TTYS1_RXBUFFER  g_modemrxbuffer
# define TTYS1_RXBUFSIZE CONFIG_UART_MODEM_RXBUFSIZE
# define TTYS1_TXBUFFER  g_modemtxbuffer
# define TTYS1_TXBUFSIZE CONFIG_UART_MODEM_TXBUFSIZE
#else
# define CONSOLE_DEV     g_modemport
# define TTYS0_DEV       g_modemport
# define TTYS0_RXBUFFER  g_modemrxbuffer
# define TTYS0_RXBUFSIZE CONFIG_UART_MODEM_RXBUFSIZE
# define TTYS0_TXBUFFER  g_modemtxbuffer
# define TTYS0_TXBUFSIZE CONFIG_UART_MODEM_TXBUFSIZE
# define TTYS1_DEV       g_irdaport
# define TTYS1_RXBUFFER  g_irdarxbuffer
# define TTYS1_RXBUFSIZE CONFIG_UART_IRDA_RXBUFSIZE
# define TTYS1_TXBUFFER  g_irdatxbuffer
# define TTYS1_TXBUFSIZE CONFIG_UART_IRDA_TXBUFSIZE
#endif

/************************************************************
 * Private Functions
 ************************************************************/

static inline uint32 up_inserial(up_dev_t *dev, uint32 offset)
{
  return getreg32(dev->uartbase + offset);
}

static inline void up_serialout(up_dev_t *dev, uint32 offset, uint32 value)
{
  putreg32(value, dev->uartbase + offset);
}

static inline void up_serialreset(up_dev_t *dev)
{
  /* Both the IrDA and MODEM UARTs support RESET and UART mode. */

  up_serialout(dev, UART_MDR_OFFS, MDR_RESET_MODE);
  up_delay(5);
  up_serialout(dev, UART_MDR_OFFS, MDR_UART_MODE);
  up_delay(5);
}

static inline void up_disabletxint(up_dev_t *dev)
{
  dev->regs.ier &= ~UART_IER_XmitInt;
  up_serialout(dev, UART_IER_OFFS, dev->regs.ier);
}

static inline void up_disablerxint(up_dev_t *dev)
{
  dev->regs.ier &= ~UART_IER_RecvInt;
  up_serialout(dev, UART_IER_OFFS, dev->regs.ier);
}

static inline void up_enabletxint(up_dev_t *dev)
{
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
  dev->regs.ier |= UART_IER_XmitInt;
  up_serialout(dev, UART_IER_OFFS, dev->regs.ier);
#endif
}

static inline void up_enablerxint(up_dev_t *dev)
{
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
  dev->regs.ier |= UART_IER_RecvInt;
  up_serialout(dev, UART_IER_OFFS, dev->regs.ier);
#endif
}

static inline void up_disableuartint(up_dev_t *dev, uint16 *ier)
{
  *ier = dev->regs.ier & UART_IER_AllInts;
  dev->regs.ier &= ~UART_IER_AllInts;
  up_serialout(dev, UART_IER_OFFS, dev->regs.ier);
}

static inline void up_restoreuartint(up_dev_t *dev, uint16 ier)
{
  dev->regs.ier |= ier & (UART_IER_RecvInt|UART_IER_XmitInt);
  up_serialout(dev, UART_IER_OFFS, dev->regs.ier);
}

static inline void up_clearfifos(up_dev_t *dev)
{
  up_serialout(dev, UART_EFR_OFFS,  0x0010);           /* Enable fifo control */
  up_serialout(dev, UART_TFCR_OFFS, 0);                /* Reset to 0 */
  up_serialout(dev, UART_RFCR_OFFS, UART_FCR_RX_CLR);  /* Clear RX fifo */
  up_serialout(dev, UART_TFCR_OFFS, UART_FCR_TX_CLR);  /* Clear TX fifo */
  up_serialout(dev, UART_TFCR_OFFS, UART_FCR_FIFO_EN); /* Enable RX/TX fifos */
}

static inline void up_disablebreaks(up_dev_t *dev)
{
  dev->regs.lcr &= ~UART_LCR_BOC;
  up_serialout(dev, UART_LCR_OFFS, dev->regs.lcr);
}

static inline void up_enablebreaks(up_dev_t *dev)
{
  dev->regs.lcr |= UART_LCR_BOC;
  up_serialout(dev, UART_LCR_OFFS, dev->regs.lcr);
}

static inline void up_setrate(up_dev_t *dev, unsigned int rate)
{
  uint32 div_bit_rate;

  switch (rate)
    {
    case 115200:
      div_bit_rate = BAUD_115200;
      break;
    case 57600:
      div_bit_rate = BAUD_57600;
      break;
    case 38400:
      div_bit_rate = BAUD_38400;
      break;
    case 19200:
      div_bit_rate = BAUD_19200;
      break;
    case 4800:
      div_bit_rate = BAUD_4800;
      break;
    case 2400:
      div_bit_rate = BAUD_2400;
      break;
    case 1200:
      div_bit_rate = BAUD_1200;
      break;
    case 9600:
    default:
      div_bit_rate = BAUD_9600;
      break;
    }

  up_serialout(dev, UART_DIV_BIT_RATE_OFFS, div_bit_rate);
}

static inline void up_setmode(up_dev_t *dev, uint16 mode)
{
  dev->regs.lcr &= 0xffffffe0;          /* clear original field, and... */
  dev->regs.lcr |= (uint32)mode; /* Set new bits in that field. */
  up_serialout(dev, UART_LCR_OFFS, dev->regs.lcr);
}

static inline void up_setrxtrigger(up_dev_t *dev, uint16 val)
{
  /* does val need to be shifted to the right spot? */
  dev->regs.fcr = (dev->regs.fcr & 0xffffff3f) | (val & 0xc0);
  up_serialout(dev, UART_RFCR_OFFS, dev->regs.fcr);
}

static inline void up_settxtrigger(up_dev_t *dev, uint16 val)
{
  /* does val need to be shifted to the right spot? */
  dev->regs.fcr = (dev->regs.fcr & 0xffffffcf) | (val & 0x30);
  up_serialout(dev, UART_RFCR_OFFS, dev->regs.fcr);
}

static inline void up_outserialchar(up_dev_t *dev, ubyte val)
{
  up_serialout(dev, UART_THR_OFFS, val);
}

static inline unsigned char up_inserialchar(up_dev_t *dev,
                                           uint16 *status)
{
  uint32 rhr;
  uint32 lsr;

  /* Next, construct a 16bit status word that
   * uses the high byte to hold the status bits
   * associated with framing,parity,break and
   * a low byte that holds error bits of LSR for
   * conditions such as overflow, etc.
   */

  rhr = up_inserial(dev, UART_RHR_OFFS);
  lsr = up_inserial(dev, UART_LSR_OFFS);

  *status = (uint16)((rhr & 0x0000ff00) | (lsr & 0x000000ff));

  return rhr & 0x000000ff;
}

static inline int up_errorcondition(uint16 status)
{
  return status & 0x0782;
}

static inline int up_parityerror(uint16 status)
{
  return status & 0x0100;
}

static inline int up_framingerror(uint16 status)
{
  return status & 0x0200;
}

static inline int up_overrunerror(uint16 status)
{
  return status & 0x0002;
}

static inline uint32 up_getintcause(up_dev_t *dev)
{
  return up_inserial(dev, UART_ISR_OFFS) & 0x0000003f;
}

static inline int up_xmitint(up_dev_t *dev,
                                                uint32 cause)
{
  return (cause & 0x00000002) == 0x00000002;
}

static inline int up_recvint(up_dev_t *dev, uint32 cause)
{
  return (cause & 0x0000000c) == 0x00000004;
}

static inline int up_recvtimeoutint(up_dev_t *dev, uint32 cause)
{
  return (cause & 0x0000000c) == 0x0000000c;
}

static inline ubyte up_txfifoempty(up_dev_t *dev)
{
  return (up_inserial(dev, UART_LSR_OFFS) & UART_LSR_TREF) != 0;
}

static inline ubyte up_txfifonotfull(up_dev_t *dev)
{
  return (up_inserial(dev, UART_SSR_OFFS) & UART_SSR_TXFULL) == 0;
}

static inline void up_waittxfifonotfull(up_dev_t *dev)
{
  int tmp;
  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if (up_txfifonotfull(dev))
        {
          break;
        }
    }
}

static inline ubyte up_rxfifonotempty(up_dev_t *dev)
{
  return up_inserial(dev, UART_LSR_OFFS) & UART_RX_FIFO_NOEMPTY;
}

static inline void serial_enable_hw_flow_control(up_dev_t *dev)
{
  /* Set the FIFO level triggers for flow control
   * Halt = 48 bytes, resume = 12 bytes
   */

  dev->regs.tcr = (dev->regs.tcr & 0xffffff00) | 0x0000003c;
  up_serialout(dev, UART_TCR_OFFS, dev->regs.tcr);

  /* Enable RTS/CTS flow control */

  dev->regs.efr |= 0x000000c0;
  up_serialout(dev, UART_EFR_OFFS, dev->regs.efr);
}

static inline void serial_disable_hw_flow_control(up_dev_t *dev)
{
  /* Disable RTS/CTS flow control */

  dev->regs.efr &= 0xffffff3f;
  up_serialout(dev, UART_EFR_OFFS, dev->regs.efr);
}

static inline void up_saveregisters(up_dev_t *dev)
{
  dev->regs.ier = up_inserial(dev, UART_IER_OFFS);
  dev->regs.lcr = up_inserial(dev, UART_LCR_OFFS);
#ifdef CONFIG_UART_HWFLOWCONTROL
  if (dev->flowcontrol)
    {
      dev->regs.efr = up_inserial(dev, UART_EFR_OFFS);
      dev->regs.tcr = up_inserial(dev, UART_TCR_OFFS);
    }
#endif
}

/************************************************************
 * up_takesem
 ************************************************************/

static void up_takesem(sem_t *sem)
{
  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

/************************************************************
 * Name: up_givesem
 ************************************************************/

static inline void up_givesem(sem_t *sem)
{
  (void)sem_post(sem);
}

/************************************************************
 * Name: up_receivechars
 ************************************************************/

/* Add chars to head of receive buffer.  up_read will take
 * characters from the tail of the buffer.
 */

static inline void up_recvchars(up_dev_t *dev)
{
  uint16 status;
  int nexthead = dev->recv.head + 1;

  if (nexthead >= dev->recv.size)
    {
      nexthead = 0;
    }

  while (nexthead != dev->recv.tail && up_rxfifonotempty(dev))
    {
      dev->recv.buffer[dev->recv.head] = up_inserialchar(dev, &status);

      dev->recv.head = nexthead;
      if (++nexthead >= dev->recv.size)
        {
           nexthead = 0;
        }

      if (dev->recvwaiting)
        {
          dev->recvwaiting = FALSE;
          up_givesem(&dev->recvsem);
        }
    }
}

/************************************************************
 * up_xmitchars
 ************************************************************/

/* Transmit characters from the tail of the xmit buffer while
 * up_write adds data to the head of the xmit buffer.
 */

static void up_xmitchars(up_dev_t *dev)
{
  /* Send while we still have data & room in the fifo */

  while (dev->xmit.head != dev->xmit.tail && up_txfifonotfull(dev))
    {
      up_outserialchar(dev, dev->xmit.buffer[dev->xmit.tail]);

      if (++(dev->xmit.tail) >= dev->xmit.size)
        {
          dev->xmit.tail = 0;
        }

      if (dev->xmitwaiting)
        {
          dev->xmitwaiting = FALSE;
          up_givesem(&dev->xmitsem);
        }
    }

  /* When all of the characters have been sent from the buffer
   * disable the TX interrupt.
   */

  if (dev->xmit.head == dev->xmit.tail)
    {
      up_disabletxint(dev);
    }
}

/************************************************************
 * Name: up_putxmitchar
 ************************************************************/

static void up_putxmitchar(up_dev_t *dev, int ch)
{
  int nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  for(;;)
    {
      if (nexthead != dev->xmit.tail)
        {
          dev->xmit.buffer[dev->xmit.head] = ch;
          dev->xmit.head = nexthead;
          return;
        }
      else
        {
          /* Transfer some characters with interrupts disabled */

          up_xmitchars(dev);

          /* If we unsuccessful in making room in the buffer.
           * then transmit the characters with interrupts
           * enabled and wait for result.
           */

          if (nexthead == dev->xmit.tail)
            {
              /* Still no space */

#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_SERIAL_INTS)
              up_waittxfifonotfull(dev);
#else
              dev->xmitwaiting = TRUE;

              /* Wait for some characters to be sent from the buffer
               * with the TX interrupt disabled.
               */

              up_enabletxint(dev);
              up_takesem(&dev->xmitsem);
              up_disabletxint(dev);
#endif
            }
        }
    }
}

/************************************************************
 * Name: up_interrupt
 ************************************************************/

/* This routine is used to handle the "top half" processing for the
 * serial driver.
 */

static int up_interrupt(int irq, void *context)
{
  up_dev_t         *dev;
  volatile uint32 cause;

  if (g_irdaport.irq == irq)
    {
      dev = &g_irdaport;
    }
  else if (g_modemport.irq == irq)
    {
      dev = &g_modemport;
    }
  else
    {
      PANIC(OSERR_INTERNAL);
    }

  cause = up_getintcause(dev);

  if (up_recvtimeoutint(dev, cause))
    {
      uint32 ier_val = 0;

      /* Is this an interrupt from the IrDA UART? */

      if (irq == C5471_IRQ_UART_IRDA)
         {
           /* Save the currently enabled IrDA UART interrupts
            * so that we can restore the IrDA interrupt state
            * below.
            */

           ier_val = up_inserial(dev, UART_IER_OFFS);

           /* Then disable all IrDA UART interrupts */

           up_serialout(dev, UART_IER_OFFS, 0);
         }

      /* Receive characters from the RX fifo */

      up_recvchars(dev);

      /* read UART_RHR to clear int condition
       * toss = up_inserialchar(dev,&status);
       */

      /* Is this an interrupt from the IrDA UART? */

      if (irq == C5471_IRQ_UART_IRDA)
         {
           /* Restore the IrDA UART interrupt enables */

           up_serialout(dev, UART_IER_OFFS, ier_val);
         }
    }
  else if (up_recvint(dev, cause))
    {
      up_recvchars(dev);
    }

  if (up_xmitint(dev, cause))
    {
      up_xmitchars(dev);
    }

  return OK;
}

/************************************************************
 * Name: up_uartsetup
 ************************************************************/

static void up_uartsetup(up_dev_t *dev)
{
#ifdef CONFIG_SUPPRESS_UART_CONFIG
  unsigned int cval;
  uint16  mrs;

  if (dev->bits == 7)
    {
      cval = UART_LCR_7bits;
    }
  else
    {
      cval = UART_LCR_8bits;
    }

  if (dev->stopbits2)
    {
      cval |= UART_LCR_2stop;
    }

  if (dev->parity == 1)   /* Odd parity */
    {
      cval |= (UART_LCR_ParEn|UART_LCR_ParOdd);
    }
  else if (dev->parity == 2)  /* Even parity */
    {
      cval |= (UART_LCR_ParEn|UART_LCR_ParEven);
    }

  up_serialreset(dev);
  up_saveregisters(dev);
  up_disableuartint(dev, &mrs);
  up_clearfifos(dev);
  up_disablebreaks(dev);
  up_settxtrigger(dev, UART_FCR_FTL);
  up_setrxtrigger(dev, UART_FCR_FTL);
  up_setrate(dev, dev->baud);
  up_setmode(dev, cval);

#ifdef CONFIG_UART_HWFLOWCONTROL
  if (dev->flowcontrol)
    {
      serial_enable_hw_flow_control(dev);
    }
  else
#endif
    {
      serial_disable_hw_flow_control(dev);
    }
#endif
}

/************************************************************
 * shutdown
 ************************************************************/

/* This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */

static void shutdown(up_dev_t * dev)
{
  irqstate_t flags;
  uint16 msr;

  /* Free the IRQ */

  flags = irqsave(); /* Disable interrupts */
  up_disable_irq(dev->irq);
  irq_detach(dev->irq);
  up_disableuartint(dev, &msr);
  irqrestore(flags);
}

/************************************************************
 * Name: up_write
 ************************************************************/

static ssize_t up_write(struct file *filep, const char *buffer, size_t buflen)
{
  struct inode *inode    = filep->f_inode;
  up_dev_t     *dev      = inode->i_private;
  ssize_t       ret      = buflen;

  /* Loop while we still have data to copy to the transmit buffer.
   * we add data to the head of the buffer; up_xmitchars takes the
   * data from the end of the buffer.
   */

  up_disabletxint(dev);
  for (; buflen; buflen--)
    {
      int ch = *buffer++;

      /* Put the character into the transmit buffer */

      up_putxmitchar(dev, ch);
 
     /* If this is the console, then we should replace LF with LF-CR */

      if (ch == '\n')
        {
          up_putxmitchar(dev, '\r');
        }
    }

  if (dev->xmit.head != dev->xmit.tail)
    {
      up_xmitchars(dev);
      if (dev->xmit.head != dev->xmit.tail)
        {
          up_enabletxint(dev);
        }
    }

  return ret;
}

/************************************************************
 * Name: up_read
 ************************************************************/

static ssize_t up_read(struct file *filep, char *buffer, size_t buflen)
{
  struct inode *inode = filep->f_inode;
  up_dev_t     *dev   = inode->i_private;
  ssize_t       ret   = buflen;

  /* Loop while we still have data to copy to the receive buffer.
   * we add data to the head of the buffer; up_xmitchars takes the
   * data from the end of the buffer.
   */

  up_disablerxint(dev);
  while (buflen)
    {
      if (dev->recv.head != dev->recv.tail)
        {
          *buffer++ = dev->recv.buffer[dev->recv.tail];
          buflen--;

          if (++dev->recv.tail >= dev->recv.size)
            {
              dev->recv.tail = 0;
            }
        }
      else
        {
          /* Wait for some characters to be sent from the buffer
           * with the TX interrupt disabled.
           */

          dev->recvwaiting = TRUE;
          up_enablerxint(dev);
          up_takesem(&dev->recvsem);
          up_disablerxint(dev);
        }
    }

  up_enablerxint(dev);

  return ret;
}

/************************************************************
 * Name: up_ioctl
 ************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  up_dev_t     *dev   = inode->i_private;
  int           ret   = OK;

  switch (cmd)
    {
    case TIOCSERGSTRUCT:
      {
         up_dev_t *user = (up_dev_t*)arg;
         if (!user)
           {
             *get_errno_ptr() = EINVAL;
             ret = ERROR;
           }
         else
           {
             memcpy(user, dev, sizeof(up_dev_t));
           }
       }
       break;

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = irqsave();
        up_enablebreaks(dev);
        irqrestore(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = irqsave();
        up_disablebreaks(dev);
        irqrestore(flags);
      }
      break;

    default:
      *get_errno_ptr() = ENOTTY;
      ret = ERROR;
      break;
    }

  return ret;
}

/************************************************************
 * Name: up_close
 *
 * Description:
 *   This routine is called when the serial port gets closed.
 *   It waits for the last remaining data to be sent.
 *
 ************************************************************/

static int up_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  up_dev_t     *dev   = inode->i_private;

  up_takesem(&dev->closesem);
  if (dev->open_count > 1)
    {
      dev->open_count--;
      up_givesem(&dev->closesem);
      return OK;
    }

  /* There are no more references to the port */

  dev->open_count = 0;

  /* Stop accepting input */

  up_disablerxint(dev);

  /* Now we wait for the transmit buffer to clear */

  while (dev->xmit.head != dev->xmit.tail)
    {
      usleep(500*1000);
    }

  /* And wait for the TX fifo to drain */

  while (!up_txfifoempty(dev))
    {
      usleep(500*1000);
    }

  /* That's it! */

  shutdown(dev);
  up_givesem(&dev->closesem);
  return OK;
 }

/************************************************************
 * Name: up_open
 *
 * Description:
 *   This routine is called whenever a serial port is opened.
 *
 ************************************************************/

static int up_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  up_dev_t     *dev   = inode->i_private;
  int           ret   = OK;

  /* If the port is the middle of closing, wait until the close
   * is finished
   */

  up_takesem(&dev->closesem);

  /* Start up serial port */

  if (++dev->open_count == 1)
    {
      irqstate_t flags = irqsave();

      /* If this is the console, then the UART has already
       * been initialized.
       */

      if (!dev->isconsole)
        {
          up_uartsetup(dev);
        }

      /* But, in any event, we do have to configure for
       * interrupt driven mode of operation.
       */

      /* Attache and enabled the IRQ */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      ret = irq_attach(dev->irq, up_interrupt);
      if (ret == OK)
#endif
        {
          /* Mark the io buffers empty */

          dev->xmit.head = 0;
          dev->xmit.tail = 0;
          dev->recv.head = 0;
          dev->recv.tail = 0;

          /* Finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
          up_enable_irq(dev->irq);
#endif
          up_enablerxint(dev);
        }
      irqrestore(flags);
    }

  up_givesem(&dev->closesem);
  return ret;
}

/************************************************************
 * Name: up_devinit
 ************************************************************/

static void up_devinit(up_dev_t *dev,
                       char *rxbuffer, int rxbufsize,
                       char *txbuffer, int txbufsize)
{
  /* Initialize fields in the dev structure that were not
   * statically initialized.
   */

  dev->xmit.size   = txbufsize;
  dev->xmit.buffer = txbuffer;
  dev->recv.size   = rxbufsize;
  dev->recv.buffer = rxbuffer;

  sem_init(&dev->closesem, 0, 1);
  sem_init(&dev->xmitsem, 0, 0);
  sem_init(&dev->recvsem, 0, 0);
}

/************************************************************
 * Public Funtions
 ************************************************************/

/************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in 
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_serialinit.
 *
 ************************************************************/

void up_earlyserialinit(void)
{
  /* Set up default port setings */

  up_devinit(&TTYS0_DEV,
             TTYS0_RXBUFFER, TTYS0_RXBUFSIZE,
             TTYS0_TXBUFFER, TTYS0_TXBUFSIZE);
  up_devinit(&TTYS1_DEV,
             TTYS1_RXBUFFER, TTYS1_RXBUFSIZE,
             TTYS1_TXBUFFER, TTYS1_TXBUFSIZE);

  /* Configure the console for use now */

  CONSOLE_DEV.isconsole = TRUE;
  up_uartsetup(&CONSOLE_DEV);
}

/************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ************************************************************/

void up_serialinit(void)
{
  (void)register_inode("/dev/console", &g_serialops, 0666, &CONSOLE_DEV);
  (void)register_inode("/dev/ttyS0", &g_serialops, 0666, &TTYS0_DEV);
  (void)register_inode("/dev/ttyS1", &g_serialops, 0666, &TTYS1_DEV);
}

/************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ************************************************************/

int up_putc(int ch)
{
  uint16  ier;

  up_disableuartint(&CONSOLE_DEV, &ier);
  up_waittxfifonotfull(&CONSOLE_DEV);
  up_outserialchar(&CONSOLE_DEV, ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxfifonotfull(&CONSOLE_DEV);
      up_outserialchar(&CONSOLE_DEV, '\r');
    }

  up_waittxfifonotfull(&CONSOLE_DEV);
  up_restoreuartint(&CONSOLE_DEV, ier);
  return ch;
}

