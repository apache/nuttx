/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_serial.c
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
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "rv32m1.h"
#include "rv32m1_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rv32m1_tty_s
{
  uart_dev_t * dev; /* TTY Device Reference */
  const int    idx; /* TTY Index */
};

#ifdef USE_SERIALDRIVER

#ifdef HAVE_UART 

#if defined(CONFIG_RV32M1_LPUART0)
#  define RV32M1_LPUART0_DEV     g_uart0dev
#endif

#if defined(CONFIG_RV32M1_LPUART1)
#  define RV32M1_LPUART1_DEV     g_uart1dev
#endif

#if defined(CONFIG_RV32M1_LPUART2)
#  define RV32M1_LPUART2_DEV     g_uart2dev
#endif

#if defined(CONFIG_RV32M1_LPUART3)
#  define RV32M1_LPUART3_DEV     g_uart3dev
#endif

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#if defined(CONFIG_LPUART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV       RV32M1_LPUART0_DEV
#  define SERIAL_CONSOLE    0
#elif defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV       RV32M1_LPUART1_DEV
#  define SERIAL_CONSOLE    1
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV       RV32M1_LPUART2_DEV
#  define SERIAL_CONSOLE    2
#elif defined(CONFIG_LPUART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV       RV32M1_LPUART3_DEV
#  define SERIAL_CONSOLE    3
#elif defined(HAVE_SERIAL_CONSOLE)
#  error "No Serial Consoles for RV32M1"
#endif

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONSOLE_DEV)
#  error "Serial Console Undefined for RV32M1"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  const uintptr_t uartbase; /* Base address of UART registers */
  const uintptr_t pcc;      /* Address of UART PCC clock gate */
  const uint32_t  tx_gpio;  /* LPUART TX GPIO pin configuration */
  const uint32_t  rx_gpio;  /* LPUART RX GPIO pin configuration */
  uint32_t  baud;           /* Configured baud */
  uint16_t  irq;            /* IRQ associated with this UART */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers */

static uint32_t up_getreg(struct up_dev_s *priv, int offset);
static void up_putreg(struct up_dev_s *priv, int offset, uint32_t value);
static void up_restoreuartint(struct up_dev_s *priv, uint32_t im);
static void up_disableuartint(struct up_dev_s *priv, uint32_t *im);
static void up_clkconfig(struct up_dev_s * priv);
static uint32_t up_clkfreq(struct up_dev_s * priv);

/* Serial driver methods */

static void up_set_format(struct uart_dev_s * dev);
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

/* LPUART Instances */

#ifdef CONFIG_RV32M1_LPUART0
static char g_uart0rxbuffer[CONFIG_LPUART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_LPUART0_TXBUFSIZE];

static struct up_dev_s g_uart0priv =
{
  .uartbase  = RV32M1_LPUART0_BASE,
  .pcc       = RV32M1_PCC_LPUART0,
  .tx_gpio   = GPIO_LPUART0_TX,
  .rx_gpio   = GPIO_LPUART0_RX,
  .baud      = CONFIG_LPUART0_BAUD,
  .irq       = RV32M1_IRQ_LPUART0,
};

static uart_dev_t g_uart0dev =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 0,
#endif

  .recv      =
  {
    .size    = CONFIG_LPUART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_LPUART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};
#endif

#ifdef CONFIG_RV32M1_LPUART1
static char g_uart1rxbuffer[CONFIG_LPUART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_LPUART1_TXBUFSIZE];

static struct up_dev_s g_uart1priv =
{
  .uartbase  = RV32M1_LPUART1_BASE,
  .pcc       = RV32M1_PCC_LPUART1,
  .tx_gpio   = GPIO_LPUART1_TX,
  .rx_gpio   = GPIO_LPUART1_RX,
  .baud      = CONFIG_LPUART1_BAUD,
  .irq       = RV32M1_IRQ_LPUART1,
};

static uart_dev_t g_uart1dev =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 1,
#endif

  .recv      =
  {
    .size    = CONFIG_LPUART1_RXBUFSIZE,
    .buffer  = g_uart1rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_LPUART1_TXBUFSIZE,
    .buffer  = g_uart1txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart1priv,
};
#endif

#ifdef CONFIG_RV32M1_LPUART2
static char g_uart2rxbuffer[CONFIG_LPUART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_LPUART2_TXBUFSIZE];

static struct up_dev_s g_uart2priv =
{
  .uartbase  = RV32M1_LPUART2_BASE,
  .pcc       = RV32M1_PCC_LPUART2,
  .tx_gpio   = GPIO_LPUART2_TX,
  .rx_gpio   = GPIO_LPUART2_RX,
  .baud      = CONFIG_LPUART2_BAUD,
  .irq       = RV32M1_IRQ_LPUART2,
};

static uart_dev_t g_uart2dev =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 2,
#endif

  .recv      =
  {
    .size    = CONFIG_LPUART2_RXBUFSIZE,
    .buffer  = g_uart2rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_LPUART2_TXBUFSIZE,
    .buffer  = g_uart2txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart2priv,
};
#endif

#ifdef CONFIG_RV32M1_LPUART3
static char g_uart3rxbuffer[CONFIG_LPUART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_LPUART3_TXBUFSIZE];

static struct up_dev_s g_uart3priv =
{
  .uartbase  = RV32M1_LPUART3_BASE,
  .pcc       = RV32M1_PCC_LPUART3,
  .tx_gpio   = GPIO_LPUART3_TX,
  .rx_gpio   = GPIO_LPUART3_RX,
  .baud      = CONFIG_LPUART3_BAUD,
  .irq       = RV32M1_IRQ_LPUART3,
};

static uart_dev_t g_uart3dev =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 3,
#endif

  .recv      =
  {
    .size    = CONFIG_LPUART3_RXBUFSIZE,
    .buffer  = g_uart3rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_LPUART3_TXBUFSIZE,
    .buffer  = g_uart3txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart3priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getreg
 ****************************************************************************/

static uint32_t up_getreg(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_putreg
 ****************************************************************************/

static void up_putreg(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static void up_restoreuartint(struct up_dev_s *priv, uint32_t im)
{
  irqstate_t flags = enter_critical_section();

  up_putreg(priv, RV32M1_LPUART_CTRL_OFFSET, im);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static void up_disableuartint(struct up_dev_s *priv, uint32_t *im)
{
  irqstate_t flags = enter_critical_section();
  uint32_t regval = up_getreg(priv, RV32M1_LPUART_CTRL_OFFSET);

  /* Return the current interrupt mask value */

  if (im)
    {
      *im = regval;
    }

  /* Disable all interrupts */

  regval &= ~(LPUART_CTRL_TCIE | LPUART_CTRL_TIE | LPUART_CTRL_RIE);
  up_putreg(priv, RV32M1_LPUART_CTRL_OFFSET, regval);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_clkconfig
 ****************************************************************************/

static void up_clkconfig(struct up_dev_s * priv)
{
  uint32_t regval;

  rv32m1_pcc_clock_disable(priv->pcc);

  regval = getreg32(priv->pcc);
  regval &= ~PCC_CLKCFG_PCS_MASK;
  regval |= PCC_CLKCFG_PCS_FIRC;
  putreg32(regval, priv->pcc);

  /* Open the uart pcc clock gate */

  rv32m1_pcc_clock_enable(priv->pcc);
}

/****************************************************************************
 * Name: up_clkfreq
 ****************************************************************************/

static uint32_t up_clkfreq(struct up_dev_s * priv)
{
  uint32_t regval;
  uint32_t pcs;

  regval = getreg32(priv->pcc);
  pcs = regval & PCC_CLKCFG_PCS_MASK;

  switch (pcs)
    {
      case PCC_CLKCFG_PCS_SOSC:
        return rv32m1_clockfreq(CLK_SOSCDIV2);

      case PCC_CLKCFG_PCS_SIRC:
        return rv32m1_clockfreq(CLK_SIRCDIV2);

      case PCC_CLKCFG_PCS_FIRC:
        return rv32m1_clockfreq(CLK_FIRCDIV2);

      case PCC_CLKCFG_PCS_LPFLL:
        return rv32m1_clockfreq(CLK_LPFLLDIV2);

      default:
        return 0u;
    }

  return 0u;
}

/****************************************************************************
 * Name: up_setup_format
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc.
 *
 ****************************************************************************/

static void up_set_format(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  uint32_t const freq = up_clkfreq(priv);

  uint32_t diff;
  uint16_t sbr;
  uint16_t osr;
  uint16_t tosr;

  uint32_t ctrl;
  uint32_t regval;

  /* 'This LPUART instantiation uses a slightly different baud rate
   * calculation. The idea is to use the best OSR (over-sampling rate)
   * possible. Note, OSR is typically hard-set to 16 in other LPUART
   * instantiations loop to find the best OSR value possible, one that
   * generates minimum baudDiff iterate through the rest of the supported
   * values of OSR.'
   * from rv32m1sdk.
   */

  diff = priv->baud;

  osr = 0;
  sbr = 0;

  for (tosr = 4; tosr <= 32; ++tosr)
    {
      uint32_t baud;
      uint32_t tdiff;

      /* Calculate the temporary sbr value */

      uint32_t tsbr = freq / (priv->baud * tosr);
      if (tsbr == 0)
        {
          tsbr = 1;
        }

      baud = freq / (tosr * tsbr);
      tdiff = baud - priv->baud;

      /* Select the better value between sbr and (sbr + 1) */

      baud = freq / (tosr * (tsbr + 1));

      if (tdiff > (priv->baud - baud))
        {
          /* Get the closer one. i.e. the minimum difference */

          tdiff = priv->baud - baud;
          tsbr++;
        }

      /* Pick up the best osr and sbr with the minimum diff */

      if (tdiff <= diff)
        {
          diff = tdiff;
          osr = tosr;
          sbr = tsbr;
        }
    }

  /* We don't check the baud error rate here even it is out of 3%
   * which will cause undesired performance.
   */

  regval = up_getreg(priv, RV32M1_LPUART_CTRL_OFFSET);
  ctrl = regval;

  /* Stop the Receiver and Transmitter before Baud rate update */

  regval &= ~(LPUART_CTRL_TE | LPUART_CTRL_RE);
  up_putreg(priv, RV32M1_LPUART_CTRL_OFFSET, regval);

  regval = up_getreg(priv, RV32M1_LPUART_BAUD_OFFSET);

  /* BOTHEDGE is required if osr is below 8 */

  if (osr > 3 && osr < 8)
    {
      regval |= LPUART_BAUD_BOTHEDGE;
    }

  /* Update OSR */

  regval &= ~LPUART_BAUD_OSR_MASK;
  regval |= ((osr - 1) << LPUART_BAUD_OSR_SHIFT) & LPUART_BAUD_OSR_MASK;

  /* Update SBR */

  regval &= ~LPUART_BAUD_SBR_MASK;
  regval |= (sbr << LPUART_BAUD_SBR_SHIFT) & LPUART_BAUD_SBR_MASK;

  /* Disable 10-bit Mode */

  regval &= ~LPUART_BAUD_M10;

  /* FIXME: Parity, 1-Wire and Stop bits are configuralbe,
   *        for the initial(started) version, they are fixed
   *        (hard coded): No Parity, Full-duplex, and 1 Stop bit.
   *        Fix the missing pieces later.
   */

  /* 1 Stop bit */

  regval &= ~LPUART_BAUD_SBNS_MASK;
  regval |= LPUART_BAUD_SBNS_1;

  /* Set Baud Register */

  up_putreg(priv, RV32M1_LPUART_BAUD_OFFSET, regval);

  regval = up_getreg(priv, RV32M1_LPUART_CTRL_OFFSET);

  /* No parity, 8-Bit mode */

  regval &= ~(LPUART_CTRL_PE | LPUART_CTRL_PT_MASK | LPUART_CTRL_M_MASK |
              LPUART_CTRL_M7 | LPUART_CTRL_IDLECFG_MASK |
              LPUART_CTRL_ILT);

  regval |= LPUART_CTRL_M8 | LPUART_CTRL_IDLECFG_1;

  regval = up_getreg(priv, RV32M1_LPUART_CTRL_OFFSET);
  if (ctrl & LPUART_CTRL_TE)
    {
      regval |= LPUART_CTRL_TE;
    }
  else
    {
      regval &= ~LPUART_CTRL_TE;
    }

  if (ctrl & LPUART_CTRL_RE)
    {
      regval |= LPUART_CTRL_RE;
    }
  else
    {
      regval &= ~LPUART_CTRL_RE;
    }

  /* Restore the Control Register */

  up_putreg(priv, RV32M1_LPUART_CTRL_OFFSET, regval);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t regval;

  /* Configure GPIO */

  rv32m1_gpio_config(priv->tx_gpio);
  rv32m1_gpio_config(priv->rx_gpio);

  /* Configure the clock source and open the clock gate */

  up_clkconfig(priv);

  /* Reset the Uart */

  regval = up_getreg(priv, RV32M1_LPUART_GLOBAL_OFFSET);
  regval |= LPUART_GLOBAL_RST;
  up_putreg(priv, RV32M1_LPUART_GLOBAL_OFFSET, regval);
  regval &= ~LPUART_GLOBAL_RST;
  up_putreg(priv, RV32M1_LPUART_GLOBAL_OFFSET, regval);

  /* Setup the Baudrate, Line */

  up_set_format(dev);

  /* Set Watermark Zero Level for this version  */

  regval = up_getreg(priv, RV32M1_LPUART_WATER_OFFSET);
  regval &= ~(LPUART_WATER_RXWATER_MASK | LPUART_WATER_TXWATER_MASK);
  up_putreg(priv, RV32M1_LPUART_WATER_OFFSET, regval);

  regval = up_getreg(priv, RV32M1_LPUART_FIFO_OFFSET);

  /* Enable RX and TX FIFO */

  regval |= LPUART_FIFO_TXFE | LPUART_FIFO_RXFE;
  up_putreg(priv, RV32M1_LPUART_FIFO_OFFSET, regval);

  /* Flush FIFO */

  regval |= LPUART_FIFO_TXFLUSH | LPUART_FIFO_RXFLUSH;
  up_putreg(priv, RV32M1_LPUART_FIFO_OFFSET, regval);

  /* Write '1' to Clear all Status Flags */

  regval = up_getreg(priv, RV32M1_LPUART_STAT_OFFSET);
  regval |= LPUART_STAT_LBKDIF | LPUART_STAT_RXEDGIF | LPUART_STAT_IDLE |
            LPUART_STAT_OR     | LPUART_STAT_NF      | LPUART_STAT_FE   |
            LPUART_STAT_PF     | LPUART_STAT_MA1F    | LPUART_STAT_MA2F ;

  /* LSB First */

  regval &= ~LPUART_STAT_MSBF;
  up_putreg(priv, RV32M1_LPUART_STAT_OFFSET, regval);

  /* Enable Receiver and Transmitter */

  regval = up_getreg(priv, RV32M1_LPUART_CTRL_OFFSET);
  regval |= LPUART_CTRL_TE | LPUART_CTRL_RE;

  up_putreg(priv, RV32M1_LPUART_CTRL_OFFSET, regval);

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
  uint32_t regval;

  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disableuartint(priv, NULL);

  /* Disable Transmitter and Receiver. */

  regval = up_getreg(priv, RV32M1_LPUART_CTRL_OFFSET);
  regval &= ~(LPUART_CTRL_TE | LPUART_CTRL_RE);
  up_putreg(priv, RV32M1_LPUART_CTRL_OFFSET, regval);

  /* Close the uart PCC clock gate */

  rv32m1_pcc_clock_disable(priv->pcc);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  uint32_t regval;

  regval = up_getreg(priv, RV32M1_LPUART_CTRL_OFFSET);
  regval |= LPUART_CTRL_TCIE | LPUART_CTRL_TIE | LPUART_CTRL_RIE ;
  up_putreg(priv, RV32M1_LPUART_CTRL_OFFSET, regval);

  ret = irq_attach(priv->irq, up_interrupt, dev);

  if (ret == OK)
    {
      /* Enable the interrupt */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

LOCATE_ITCM
static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint32_t           status;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Retrieve interrupt pending status */

  status = up_getreg(priv, RV32M1_LPUART_STAT_OFFSET);

  if (status & LPUART_STAT_RDRF)
    {
      /* Process incoming bytes */

      uart_recvchars(dev);
    }

  if (status & (LPUART_STAT_TDRE | LPUART_STAT_TC))
    {
      /* Process outgoing bytes */

      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
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

  /* Return status information */

  if (status)
    {
      *status = 0; /* We are not yet tracking serial errors */
    }

  /* Return cached data */

  return up_getreg(priv, RV32M1_LPUART_DATA_OFFSET) & 0xff;
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
  uint32_t regval;

  irqstate_t flags = enter_critical_section();

  regval = up_getreg(priv, RV32M1_LPUART_CTRL_OFFSET);

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      regval |= LPUART_CTRL_RIE;
#endif
    }
  else
    {
      regval &= ~LPUART_CTRL_RIE;
    }

  up_putreg(priv, RV32M1_LPUART_CTRL_OFFSET, regval);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return true is data is available in the receive data buffer */

  uint32_t fifo = up_getreg(priv, RV32M1_LPUART_FIFO_OFFSET);
  return (fifo & LPUART_FIFO_RXEMPT) == 0;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_putreg(priv, RV32M1_LPUART_DATA_OFFSET, (uint32_t)ch & 0x0ff);
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
  irqstate_t flags = enter_critical_section();

  uint32_t regval;

  regval = up_getreg(priv, RV32M1_LPUART_CTRL_OFFSET);

  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      regval |= LPUART_CTRL_TCIE | LPUART_CTRL_TIE;
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      regval &= ~(LPUART_CTRL_TCIE | LPUART_CTRL_TIE);
    }

  up_putreg(priv, RV32M1_LPUART_CTRL_OFFSET, regval);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  uint32_t txvol = (up_getreg(priv, RV32M1_LPUART_FIFO_OFFSET) &
                    LPUART_FIFO_TXFIFOSIZE_MASK) >>
                    LPUART_FIFO_TXFIFOSIZE_SHIFT ;

  uint32_t txcnt = (up_getreg(priv, RV32M1_LPUART_WATER_OFFSET) &
                    LPUART_WATER_TXCOUNT_MASK) >>
                    LPUART_WATER_TXCOUNT_SHIFT ;

  /* Return TRUE if the TX FIFO is not full */

  return txcnt < txvol;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return TRUE if the TX wartermak is pending */

  return (up_getreg(priv, RV32M1_LPUART_FIFO_OFFSET) & LPUART_FIFO_TXEMPT)
         != 0;
}
#endif /* HAVE_UART */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  /* Do nothing, we've set up the serial console.
   * The function must be provided to get rid of
   * linking problem when USE_EARLYSERIALINIT is
   * defined.
   */
}
#endif

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
  int i;
  int nuart;

  char devpath[16] = "/dev/ttyS";

  /* Device NO. */

  int devno = 1;

  const struct rv32m1_tty_s  ttydevs[] =
  {
#ifdef RV32M1_LPUART0_DEV
    {
      .dev = &RV32M1_LPUART0_DEV,
      .idx =  RV32M1_LPUART0_DEV.isconsole ? 0 : devno ++,
    },
#endif
#ifdef RV32M1_LPUART1_DEV
    {
      .dev = &RV32M1_LPUART1_DEV,
      .idx =  RV32M1_LPUART1_DEV.isconsole ? 0 : devno ++,
    },
#endif
#ifdef RV32M1_LPUART2_DEV
    {
      .dev = &RV32M1_LPUART2_DEV,
      .idx =  RV32M1_LPUART2_DEV.isconsole ? 0 : devno ++,
    },
#endif
#ifdef RV32M1_LPUART3_DEV
    {
      .dev = &RV32M1_LPUART3_DEV,
      .idx =  RV32M1_LPUART3_DEV.isconsole ? 0 : devno ++,
    },
#endif

  /* Place a dummy One as a Place holder to avoid uartdevs
   * to be empty when All above uart devices are undefined,
   * in which case a complier error will raise.
   */

    {
      .dev = NULL,
      .idx = -1,
    },
  };

  nuart = (int)(sizeof(ttydevs) / sizeof(ttydevs[0]));

  /* Register the console */

#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register All Uarts */

  for (i = 0; i < nuart; ++i)
    {
      const struct rv32m1_tty_s * tty = &ttydevs[i];

      if (!tty->dev)
        {
          continue;
        }

      /* OS (NuttX) is primordial and so many resources are uninitialized
       * while we are in 'riscv_serialinit'. The high level C lib functions
       * may not work well. Codes such as the following
       * 'snprintf(devpath, "/dev/ttyS%d\n", devno)...'
       * would not work as expected.
       *
       * It is ok to complete the device path manually.
       */

      devno = tty->idx;

      if (devno < 10)
        {
          devpath[9] = devno + '0';

          /* Terminate the String */

          devpath[10] = '\0';
        }
      else
        {
          /* There is one pre-condition that devno doesn't exceed 100 */

          int d = devno / 10;
          devpath[9] = d + '0';

          d = devno - d * 10;
          devpath[10] = d + '0';

          /* Terminate the String */

          devpath[11] = '\0';
        }

      uart_register(devpath, tty->dev);
    }
}

#endif /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef CONSOLE_DEV
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint32_t im;

  up_disableuartint(priv, &im);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
  up_restoreuartint(priv, im);
#endif
  return ch;
}

#ifdef HAVE_SERIAL_CONSOLE
/****************************************************************************
 * Name: rv32m1_console_uart_setup
 ****************************************************************************/

void rv32m1_console_uart_setup(void)
{
#ifdef CONSOLE_DEV
  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: rv32m1_console_uart_putc
 ****************************************************************************/

void rv32m1_console_uart_putc(char ch)
{
#ifdef CONSOLE_DEV
  while (!up_txready(&CONSOLE_DEV)) ;
  up_send(&CONSOLE_DEV, ch);
#endif
}
#endif /* HAVE_SERIAL_CONSOLE */
