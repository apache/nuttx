/****************************************************************************
 * arch/arm64/src/imx8/imx8qm_serial.c
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
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>

#include "chip.h"
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_gic.h"

#include "imx8_serial.h"
#include "imx8_boot.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which UART with be tty0/console and which tty1-4?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART1-5 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV             g_uart1port /* UART1 is console */
#  define TTYS0_DEV               g_uart1port /* UART1 is ttyS0 */
#  define UART1_ASSIGNED          1
#endif

/* 32-bit global registers only for i.MX7ulp/MX8x
 * The driver only use the reset feature to reset HW.
 */
#define UART_GLOBAL                 0x8

/* 32-bit register definition */
#define UARTBAUD                    0x00
#define UARTSTAT                    0x04
#define UARTCTRL                    0x08
#define UARTDATA                    0x0C
#define UARTMATCH                   0x10
#define UARTMODIR                   0x14
#define UARTFIFO                    0x18
#define UARTWATER                   0x1c

#define UARTBAUD_MAEN1              0x80000000
#define UARTBAUD_MAEN2              0x40000000
#define UARTBAUD_M10                0x20000000
#define UARTBAUD_TDMAE              0x00800000
#define UARTBAUD_RDMAE              0x00200000
#define UARTBAUD_RIDMAE             0x00100000
#define UARTBAUD_MATCFG             0x00400000
#define UARTBAUD_BOTHEDGE           0x00020000
#define UARTBAUD_RESYNCDIS          0x00010000
#define UARTBAUD_LBKDIE             0x00008000
#define UARTBAUD_RXEDGIE            0x00004000
#define UARTBAUD_SBNS               0x00002000
#define UARTBAUD_SBR                0x00000000
#define UARTBAUD_SBR_MASK           0x1fff
#define UARTBAUD_OSR_MASK           0x1f
#define UARTBAUD_OSR_SHIFT          24

#define UARTSTAT_LBKDIF             0x80000000
#define UARTSTAT_RXEDGIF            0x40000000
#define UARTSTAT_MSBF               0x20000000
#define UARTSTAT_RXINV              0x10000000
#define UARTSTAT_RWUID              0x08000000
#define UARTSTAT_BRK13              0x04000000
#define UARTSTAT_LBKDE              0x02000000
#define UARTSTAT_RAF                0x01000000
#define UARTSTAT_TDRE               0x00800000
#define UARTSTAT_TC                 0x00400000
#define UARTSTAT_RDRF               0x00200000
#define UARTSTAT_IDLE               0x00100000
#define UARTSTAT_OR                 0x00080000
#define UARTSTAT_NF                 0x00040000
#define UARTSTAT_FE                 0x00020000
#define UARTSTAT_PE                 0x00010000
#define UARTSTAT_MA1F               0x00008000
#define UARTSTAT_M21F               0x00004000

#define UARTCTRL_R8T9               0x80000000
#define UARTCTRL_R9T8               0x40000000
#define UARTCTRL_TXDIR              0x20000000
#define UARTCTRL_TXINV              0x10000000
#define UARTCTRL_ORIE               0x08000000
#define UARTCTRL_NEIE               0x04000000
#define UARTCTRL_FEIE               0x02000000
#define UARTCTRL_PEIE               0x01000000
#define UARTCTRL_TIE                0x00800000
#define UARTCTRL_TCIE               0x00400000
#define UARTCTRL_RIE                0x00200000
#define UARTCTRL_ILIE               0x00100000
#define UARTCTRL_TE                 0x00080000
#define UARTCTRL_RE                 0x00040000
#define UARTCTRL_RWU                0x00020000
#define UARTCTRL_SBK                0x00010000
#define UARTCTRL_MA1IE              0x00008000
#define UARTCTRL_MA2IE              0x00004000
#define UARTCTRL_IDLECFG_OFF        0x8
#define UARTCTRL_LOOPS              0x00000080
#define UARTCTRL_DOZEEN             0x00000040
#define UARTCTRL_RSRC               0x00000020
#define UARTCTRL_M                  0x00000010
#define UARTCTRL_WAKE               0x00000008
#define UARTCTRL_ILT                0x00000004
#define UARTCTRL_PE                 0x00000002
#define UARTCTRL_PT                 0x00000001

#define UARTDATA_NOISY              0x00008000
#define UARTDATA_PARITYE            0x00004000
#define UARTDATA_FRETSC             0x00002000
#define UARTDATA_RXEMPT             0x00001000
#define UARTDATA_IDLINE             0x00000800
#define UARTDATA_MASK               0x3ff

#define UARTMODIR_IREN              0x00020000
#define UARTMODIR_RTSWATER_S        0x8
#define UARTMODIR_TXCTSSRC          0x00000020
#define UARTMODIR_TXCTSC            0x00000010
#define UARTMODIR_RXRTSE            0x00000008
#define UARTMODIR_TXRTSPOL          0x00000004
#define UARTMODIR_TXRTSE            0x00000002
#define UARTMODIR_TXCTSE            0x00000001

#define UARTFIFO_TXEMPT             0x00800000
#define UARTFIFO_RXEMPT             0x00400000
#define UARTFIFO_TXOF               0x00020000
#define UARTFIFO_RXUF               0x00010000
#define UARTFIFO_TXFLUSH            0x00008000
#define UARTFIFO_RXFLUSH            0x00004000
#define UARTFIFO_RXIDEN_MASK        0x7
#define UARTFIFO_RXIDEN_OFF         10
#define UARTFIFO_TXOFE              0x00000200
#define UARTFIFO_RXUFE              0x00000100
#define UARTFIFO_TXFE               0x00000080
#define UARTFIFO_FIFOSIZE_MASK      0x7
#define UARTFIFO_TXSIZE_OFF         4
#define UARTFIFO_RXFE               0x00000008
#define UARTFIFO_RXSIZE_OFF         0
#define UARTFIFO_DEPTH(x)  (0x1 << ((x) ? ((x) + 1) : 0))

#define UARTWATER_COUNT_MASK        0xff
#define UARTWATER_TXCNT_OFF         8
#define UARTWATER_RXCNT_OFF         24
#define UARTWATER_WATER_MASK        0xff
#define UARTWATER_TXWATER_OFF       0
#define UARTWATER_RXWATER_OFF       16

#define UART_GLOBAL_RST             0x2
#define RST_HW_MIN_MS               20
#define RST_HW_MAX_MS               40

#define UARTFIFO_RXIDEN_RDRF        0x3
#define UARTCTRL_IDLECFG            0x7

/* Rx DMA timeout in ms, which is used to calculate Rx ring buffer size */
#define DMA_RX_TIMEOUT              (10)
#define UART_AUTOSUSPEND_TIMEOUT    3000

/* IMX lpuart has four extra unused regs located at the beginning */
#define IMX_REG_OFF                 0x10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx8_uart_port_s
{
  unsigned long iobase;
  unsigned int  baud;         /* Configured baud */
  unsigned int  irq_num;
  unsigned int  txfifo_size;
  unsigned int  rxfifo_size;
  int           is_console;
};

static inline uint32_t imx8_read(struct imx8_uart_port_s *port,
                                 uint32_t off)
{
  return getreg32(port->iobase + off);
}

static inline void imx8_write(struct imx8_uart_port_s *port,
                              uint32_t val, uint32_t off)
{
  putreg32(val, port->iobase + off);
}

static int imx8_lpuart_stop_tx(struct imx8_uart_port_s *sport)
{
  unsigned long temp;

  temp  = imx8_read(sport, UARTCTRL);
  temp  &= ~(UARTCTRL_TIE | UARTCTRL_TCIE);
  imx8_write(sport, temp, UARTCTRL);

  return 0;
}

static int imx8_lpuart_stop_rx(struct imx8_uart_port_s *sport)
{
  unsigned long temp;

  temp = imx8_read(sport, UARTCTRL);
  imx8_write(sport, temp & ~UARTCTRL_RE, UARTCTRL);

  return 0;
}

static int imx8_lpuart_start_tx(struct imx8_uart_port_s *sport)
{
  unsigned long temp;

  temp = imx8_read(sport, UARTCTRL);
  imx8_write(sport, temp | UARTCTRL_TIE, UARTCTRL);

  return 0;
}

static int imx8_lpuart_start_rx(struct imx8_uart_port_s *sport)
{
  unsigned long temp;

  temp = imx8_read(sport, UARTCTRL);
  imx8_write(sport, temp | UARTCTRL_RE, UARTCTRL);

  return 0;
}

/****************************************************************************
 * Name: imx8_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void imx8_rxint(struct uart_dev_s *dev, bool enable)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;

  if (enable)
    {
      imx8_lpuart_start_rx(sport);
    }
  else
    {
      imx8_lpuart_stop_rx(sport);
    }
}

/****************************************************************************
 * Name: imx8_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void imx8_txint(struct uart_dev_s *dev, bool enable)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;

  if (enable)
    {
      imx8_lpuart_start_tx(sport);
    }
  else
    {
      imx8_lpuart_stop_tx(sport);
    }
}

/****************************************************************************
 * Name: imx8_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void imx8_send(struct uart_dev_s *dev, int ch)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;

  imx8_write(sport, ch, UARTDATA);
}

/****************************************************************************
 * Name: imx8_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int imx8_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;
  unsigned int             rx;
  unsigned int             sr;

  /* to clear the FE, OR, NF, FE, PE flags,
   * read STAT then read DATA reg
   */

  sr = imx8_read(sport, UARTSTAT);
  rx = imx8_read(sport, UARTDATA);

  *status = sr;

  return rx;
}

/****************************************************************************
 * Name: imx8_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool imx8_rxavailable(struct uart_dev_s *dev)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;
  unsigned long            sfifo = imx8_read(sport, UARTFIFO);

  if (sfifo & UARTFIFO_RXEMPT)
    {
      return false;
    }
  else
    {
      return true;
    }
}

/****************************************************************************
 * Name: imx8_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool imx8_txready(struct uart_dev_s *dev)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;
  unsigned long            txcnt;

  /* When TXFULL is set, there is no space in the Tx FIFO  */

  txcnt = imx8_read(sport, UARTWATER);
  txcnt = txcnt >> UARTWATER_TXCNT_OFF;
  txcnt &= UARTWATER_COUNT_MASK;

  if (txcnt < sport->txfifo_size)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: imx8_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool imx8_txempty(struct uart_dev_s *dev)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;
  unsigned long            stat    = imx8_read(sport, UARTSTAT);
  unsigned long            sfifo   = imx8_read(sport, UARTFIFO);

  if (stat & UARTSTAT_TC && sfifo & UARTFIFO_TXEMPT)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: imx8_irq_handler (and front-ends)
 *
 * Description:
 *   This is the common UART interrupt handler.  It should cal
 *   uart_transmitchars or uart_receivechar to perform the appropriate data
 *   transfers.
 *
 ****************************************************************************/

static int imx8_uart_irq_handler(int irq, void *context, void *arg)
{
  struct uart_dev_s       *dev = (struct uart_dev_s *)arg;
  struct imx8_uart_port_s *sport;
  unsigned long            sts;
  unsigned long            rxcount;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  sport = (struct imx8_uart_port_s *)dev->priv;

  sts     = imx8_read(sport, UARTSTAT);
  rxcount = imx8_read(sport, UARTWATER);
  rxcount = rxcount >> UARTWATER_RXCNT_OFF;

  if (sts & UARTSTAT_RDRF || rxcount > 0)
    {
      uart_recvchars(dev);
    }

  if (sts & UARTSTAT_TDRE)
    {
      uart_xmitchars(dev);
    }

  imx8_write(sport, sts, UARTSTAT);
  return OK;
}

static int imx8_lpuart_hw_reset(struct imx8_uart_port_s *sport)
{
  unsigned long global_addr;

  if (sport->is_console)
    {
      return 0;
    }

  global_addr = sport->iobase + UART_GLOBAL - IMX_REG_OFF;
  putreg32(UART_GLOBAL_RST, global_addr);

  /* arch_timer_delay(RST_HW_MAX_MS); */

  putreg32(0, global_addr);

  /* arch_timer_delay(RST_HW_MAX_MS); */

  return 0;
}

static int imx8_setup(struct uart_dev_s *dev)
{
  irqstate_t               i_flags;
  int                      ret;
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;

  i_flags = up_irq_save();

  sport->iobase += IMX_REG_OFF;

  ret = imx8_lpuart_hw_reset(sport);
  if (ret < 0)
    {
      serr("failed to reset hw: %d\n", ret);
    }

  up_irq_restore(i_flags);

  return ret;
}

void board_timerhook(void)
{
}

/****************************************************************************
 * Name: imx8_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void imx8_shutdown(struct uart_dev_s *dev)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;
  unsigned long            temp;
  irqstate_t               i_flags;

  i_flags = up_irq_save();

  /* clear statue */

  temp = imx8_read(sport, UARTSTAT);
  imx8_write(sport, temp, UARTSTAT);

  /* disable Rx/Tx DMA */

  temp = imx8_read(sport, UARTBAUD);
  temp &= ~(UARTBAUD_TDMAE | UARTBAUD_RDMAE | UARTBAUD_RIDMAE);
  imx8_write(sport, temp, UARTBAUD);

  /* disable Rx/Tx and interrupts */

  temp = imx8_read(sport, UARTCTRL);
  temp &= ~(UARTCTRL_TE | UARTCTRL_RE | UARTCTRL_TIE |
            UARTCTRL_TCIE | UARTCTRL_RIE | UARTCTRL_ILIE |
            UARTCTRL_LOOPS);
  imx8_write(sport, temp, UARTCTRL);
  imx8_write(sport, 0, UARTMODIR);

  up_irq_restore(i_flags);
}

static void imx8_setup_watermark(struct imx8_uart_port_s *sport)
{
  unsigned long val;
  unsigned long ctrl;
  unsigned long ctrl_saved;
  unsigned long rxiden_cnt;

  ctrl       = imx8_read(sport, UARTCTRL);
  ctrl_saved = ctrl;
  ctrl       &= ~(UARTCTRL_TIE | UARTCTRL_TCIE | UARTCTRL_TE |
                  UARTCTRL_RIE | UARTCTRL_RE);
  imx8_write(sport, ctrl, UARTCTRL);

  /* enable FIFO mode */

  val         = imx8_read(sport, UARTFIFO);
  val        |= UARTFIFO_TXFE | UARTFIFO_RXFE;
  val        |= UARTFIFO_TXFLUSH | UARTFIFO_RXFLUSH;
  val        &= ~(UARTFIFO_RXIDEN_MASK << UARTFIFO_RXIDEN_OFF);
  rxiden_cnt = 0;
  val        |= ((rxiden_cnt & UARTFIFO_RXIDEN_MASK) << UARTFIFO_RXIDEN_OFF);

  imx8_write(sport, val, UARTFIFO);

  /* set the watermark
   * rx_watermark = 1;
   */

  val = (1 << UARTWATER_RXWATER_OFF) | (0x0 << UARTWATER_TXWATER_OFF);
  imx8_write(sport, val, UARTWATER);

  /* Restore cr2 */

  imx8_write(sport, ctrl_saved, UARTCTRL);
}

static void imx8_setup_watermark_enable(struct imx8_uart_port_s *sport)
{
  uint32_t temp;

  imx8_setup_watermark(sport);

  temp = imx8_read(sport, UARTCTRL);
  temp |= UARTCTRL_RE | UARTCTRL_TE;
  temp |= UARTCTRL_IDLECFG << UARTCTRL_IDLECFG_OFF;
  imx8_write(sport, temp, UARTCTRL);
}

static void imx8_hw_disable(struct imx8_uart_port_s *sport)
{
  unsigned long temp;

  temp  = imx8_read(sport, UARTCTRL);
  temp  &= ~(UARTCTRL_RIE | UARTCTRL_ILIE | UARTCTRL_RE |
             UARTCTRL_TIE | UARTCTRL_TE);
  imx8_write(sport, temp, UARTCTRL);
}

static void imx8_configure(struct imx8_uart_port_s *sport)
{
  unsigned long temp;

  temp = imx8_read(sport, UARTCTRL);
  temp |= UARTCTRL_RIE | UARTCTRL_ILIE;
  temp |= UARTCTRL_TIE;
  imx8_write(sport, temp, UARTCTRL);
}

static void imx8_hw_setup(struct imx8_uart_port_s *sport)
{
  irqstate_t i_flags;

  i_flags = up_irq_save();

  imx8_hw_disable(sport);

  imx8_setup_watermark_enable(sport);
  imx8_configure(sport);
  up_enable_irq(sport->irq_num);

  up_irq_restore(i_flags);
}

/****************************************************************************
 * Name: imx8_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int imx8_attach(struct uart_dev_s *dev)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;
  int                      ret;
  unsigned long            temp;

  /* determine FIFO size */

  temp = imx8_read(sport, UARTFIFO);

  sport->txfifo_size = UARTFIFO_DEPTH(
    (temp >> UARTFIFO_TXSIZE_OFF) & UARTFIFO_FIFOSIZE_MASK);
  sport->rxfifo_size = UARTFIFO_DEPTH(
    (temp >> UARTFIFO_RXSIZE_OFF) & UARTFIFO_FIFOSIZE_MASK);

  ret = irq_attach(sport->irq_num, imx8_uart_irq_handler, dev);

  if (ret == OK)
    {
      imx8_hw_setup(sport);
    }
  else
    {
      sinfo("error ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: imx_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void imx8_detach(struct uart_dev_s *dev)
{
  struct imx8_uart_port_s *sport = (struct imx8_uart_port_s *)dev->priv;

  up_disable_irq(sport->irq_num);
  irq_detach(sport->irq_num);
}

/****************************************************************************
 * Name: imx8_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *   for current imx8qm configure,
 *
 ****************************************************************************/

static int imx8_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TIOCSERGSTRUCT) || defined(CONFIG_SERIAL_TERMIOS)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  irqstate_t        flags;
#endif
  int               ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
    {
      struct imx_uart_s *user = (struct imx_uart_s *)arg;
      if (!user)
        {
          ret = -EINVAL;
        }
      else
        {
          memcpy(user, dev, sizeof(struct imx_uart_s));
        }
    }
    break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
    {
      struct termios    *termiosp = (struct termios *)arg;
      struct imx_uart_s *priv     = (struct imx_uart_s *)dev->priv;

      if (!termiosp)
        {
          ret = -EINVAL;
          break;
        }

      /* Return parity */

      termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                         ((priv->parity == 1) ? PARODD : 0);

      /* Return stop bits */

      termiosp->c_cflag |= (priv->stopbits2) ? CSTOPB : 0;

      /* Return flow control */

#ifdef CONFIG_SERIAL_OFLOWCONTROL
      termiosp->c_cflag |= ((priv->oflow) ? CCTS_OFLOW : 0);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
      termiosp->c_cflag |= ((priv->iflow) ? CRTS_IFLOW : 0);
#endif

      /* Return baud */

      cfsetispeed(termiosp, priv->baud);

      /* Return number of bits */

      switch (priv->bits)
        {
        case 5:
        {
          termiosp->c_cflag |= CS5;
          break;
        }

        case 6:
        {
          termiosp->c_cflag |= CS6;
          break;
        }

        case 7:
        {
          termiosp->c_cflag |= CS7;
          break;
        }

        default:
        case 8:
        {
          termiosp->c_cflag |= CS8;
          break;
        }

#if defined(CS9)
        case 9:
        {
          termiosp->c_cflag |= CS9;
          break;
        }
#endif
        }
    }
    break;

    case TCSETS:
    {
      struct termios    *termiosp = (struct termios *)arg;
      struct imx_uart_s *priv     = (struct imx_uart_s *)dev->priv;
      uint32_t           baud;
      uint32_t           ie;
      uint8_t            parity;
      uint8_t            nbits;
      bool               stop2;

      if ((!termiosp)
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#endif
          )
        {
          ret = -EINVAL;
          break;
        }

      /* Decode baud. */

      ret  = OK;
      baud = cfgetispeed(termiosp);

      /* Decode number of bits */

      switch (termiosp->c_cflag & CSIZE)
        {
        case CS5:
        {
          nbits = 5;
          break;
        }

        case CS6:
        {
          nbits = 6;
          break;
        }

        case CS7:
        {
          nbits = 7;
          break;
        }

        case CS8:
        {
          nbits = 8;
          break;
        }

#if defined(CS9)
        case CS9:
        {
          nbits = 9;
          break;
        }

#endif
        default:
        {
          ret = -EINVAL;
          break;
        }
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

      stop2 = (termiosp->c_cflag & CSTOPB) != 0;

      /* Verify that all settings are valid before committing */

      if (ret == OK)
        {
          /* Commit */

          priv->baud      = baud;
          priv->parity    = parity;
          priv->bits      = nbits;
          priv->stopbits2 = stop2;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif

          /* effect the changes immediately - note that we do not
           * implement TCSADRAIN / TCSAFLUSH
           */

          flags = spin_lock_irqsave(NULL);
          imx8_disableuartint(priv, &ie);
          ret = imx8_setup(dev);

          /* Restore the interrupt state */

          imx_restoreuartint(priv, ie);
          priv->ie = ie;
          spin_unlock_irqrestore(NULL, flags);
        }
    }
    break;
#endif /* CONFIG_SERIAL_TERMIOS */

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
    default:
    {
      ret = -ENOTTY;
      break;
    }
    }

  return ret;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup       = imx8_setup,
  .shutdown    = imx8_shutdown,
  .attach      = imx8_attach,
  .detach      = imx8_detach,
  .ioctl       = imx8_ioctl,
  .receive     = imx8_receive,
  .rxint       = imx8_rxint,
  .rxavailable = imx8_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send        = imx8_send,
  .txint       = imx8_txint,
  .txready     = imx8_txready,
  .txempty     = imx8_txempty,
};

/* I/O buffers */

#ifdef CONFIG_IMX8_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

/* This describes the state of the IMX uart1 port. */

static struct imx8_uart_port_s  g_uart1priv =
{
  .iobase     = CONFIG_IMX8QM_UART1_BASE,
  .baud       = CONFIG_UART1_BAUD,
  .irq_num    = CONFIG_IMX8QM_UART1_IRQ,
  .is_console = 1,
};

static struct uart_dev_s g_uart1port =
{
  .recv =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart1priv,
};

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm64_serialinit.
 *
 ****************************************************************************/

void arm64_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function imx8_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */
#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  imx8_setup(&CONSOLE_DEV);
#endif
}

/* Used to assure mutually exclusive access up_putc() */

/* static sem_t g_putc_lock = SEM_INITIALIZER(1); */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
  struct uart_dev_s *dev = &CONSOLE_DEV;

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      imx8_send(dev, '\r');
    }

  imx8_send(dev, (uint8_t)ch);
  return ch;
}

/****************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that imx_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm64_serialinit(void)
{
  int ret;

  ret = uart_register("/dev/console", &CONSOLE_DEV);
  if (ret < 0)
    {
      sinfo("error at register dev/console, ret =%d\n", ret);
    }

  ret = uart_register("/dev/ttyS0", &TTYS0_DEV);

  if (ret < 0)
    {
      sinfo("error at register dev/ttyS0, ret =%d\n", ret);
    }
}

#endif /* USE_SERIALDRIVER */
