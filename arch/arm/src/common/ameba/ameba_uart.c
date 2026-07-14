/****************************************************************************
 * arch/arm/src/common/ameba/ameba_uart.c
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

/* NuttX serial lower half for the Realtek Ameba high-speed UARTs (UART0 and
 * UART1; UART2 is shared with Bluetooth and is not wired here).  The
 * LOG-UART owns the console and /dev/ttyS0 (arch/.../ameba_loguart.c); this
 * driver registers the general-purpose UARTs from board bring-up, so they
 * appear as /dev/ttyS1 and up.
 *
 * The controller is programmed through the SDK fwlib UART API.  NuttX runs
 * on the KM4 core in the SECURE state, so the fwlib routines resolve to the
 * secure register aliases via TrustZone_IsSecure() and reach the UART, the
 * PINMUX pad mux and the pull control without stall.  Every symbol used here
 * (UART_Init, UART_SetBaud, UART_CharPut/Get, Pinmux_Config, PAD_PullCtrl,
 * RCC_PeriphClockCmd, ...) resolves to the on-chip ROM symbol table; the
 * fwlib data tables that ROM code indexes (UART_DEV_TABLE, APBPeriph_UARTx)
 * are compiled into libameba_fwlib.a (see AMEBA_FWLIB_SRCS).  To keep the
 * vendor headers out of the NuttX include world, the few fwlib symbols and
 * the UART_InitTypeDef layout used here are declared locally below rather
 * than pulled in from <ameba_uart.h>.
 *
 * Interrupt dispatch stays NuttX-native: each UART's NVIC vector is owned by
 * NuttX (irq_attach), and the ISR drains RX / refills TX through the stock
 * uart_recvchars()/uart_xmitchars() upper-half helpers.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/serial/serial.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include "arm_internal.h"
#include "ameba_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-controller register base, peripheral clock mask and NuttX IRQ number.
 * The driver source is shared by every Ameba ARM chip; each chip's
 * <arch/irq.h> (pulled in by <nuttx/irq.h>) names its own vectors, so map
 * them onto chip-agnostic aliases here.
 */

#if defined(CONFIG_ARCH_CHIP_RTL8721DX)
#  define AMEBA_IRQ_UART0       RTL8721DX_IRQ_UART0
#  define AMEBA_IRQ_UART1       RTL8721DX_IRQ_UART1
#else
#  error "Ameba UART: controller IRQ numbers not defined for this chip"
#endif

#define AMEBA_UART0_BASE        0x4100c000ul
#define AMEBA_UART1_BASE        0x4100d000ul

/* APBPeriph_UARTx / APBPeriph_UARTx_CLOCK (sysreg_lsys.h). */

#define AMEBA_APBPERIPH_UART0   (((uint32_t)1 << 30) | ((uint32_t)1 << 6))
#define AMEBA_APBPERIPH_UART1   (((uint32_t)1 << 30) | ((uint32_t)1 << 7))

/* Pin mux function (ameba_pinmux.h) and pull control (ameba_gpio.h).  RX is
 * held high while idle so a floating line is not read as a start bit.
 *
 * On this SoC (amebadplus) the pad mux is a crossbar: the generic
 * PINMUX_FUNCTION_UART code is not enough -- each pad must be routed with a
 * direction-specific function code so the crossbar knows which internal UART
 * signal to drive onto it (see the SDK's per-chip branch in the raw UART
 * example).  The codes are indexed by controller in g_uart_txfid/rxfid.
 */

#define AMEBA_PINMUX_UART0_TXD  19    /* PINMUX_FUNCTION_UART0_TXD          */
#define AMEBA_PINMUX_UART0_RXD  20    /* PINMUX_FUNCTION_UART0_RXD          */
#define AMEBA_PINMUX_UART1_TXD  23    /* PINMUX_FUNCTION_UART1_TXD          */
#define AMEBA_PINMUX_UART1_RXD  24    /* PINMUX_FUNCTION_UART1_RXD          */
#define AMEBA_GPIO_PUPD_UP      0x2   /* GPIO_PuPd_UP                       */

/* Mirror of the fwlib UART_InitTypeDef field values (ameba_uart.h). */

#define AMEBA_WLS_7BITS         0x0   /* RUART_WLS_7BITS                    */
#define AMEBA_WLS_8BITS         0x1   /* RUART_WLS_8BITS                    */

#define AMEBA_STOP_BIT_1        0x0   /* RUART_STOP_BIT_1                   */
#define AMEBA_STOP_BIT_2        0x1   /* RUART_STOP_BIT_2                   */

#define AMEBA_PARITY_DISABLE    0x0   /* RUART_PARITY_DISABLE               */
#define AMEBA_PARITY_ENABLE     0x1   /* RUART_PARITY_ENABLE                */

#define AMEBA_ODD_PARITY        0x0   /* RUART_ODD_PARITY                   */
#define AMEBA_EVEN_PARITY       0x1   /* RUART_EVEN_PARITY                  */

#define AMEBA_RX_FIFOTRIG_1     0x0   /* UART_RX_FIFOTRIG_LEVEL_1BYTES      */

/* Interrupt enable register (IER) bits. */

#define AMEBA_UART_INT_ERBI     ((uint32_t)1 << 0)  /* Rx data available    */
#define AMEBA_UART_INT_ETBEI    ((uint32_t)1 << 1)  /* Tx FIFO empty        */
#define AMEBA_UART_INT_ETOI     ((uint32_t)1 << 5)  /* Rx timeout           */

/* Line status register (LSR) bits. */

#define AMEBA_UART_LSR_TX_EMPTY ((uint32_t)1 << 5)  /* Tx FIFO empty        */

/* Second/third argument to fwlib "state" style APIs. */

#define AMEBA_DISABLE           0x0
#define AMEBA_ENABLE            0x1

/* Number of general-purpose controllers this driver can address. */

#define AMEBA_NUART             2

/* Local parity encoding (priv->parity). */

#define AMEBA_PARITY_NONE       0
#define AMEBA_PARITY_ODD        1
#define AMEBA_PARITY_EVEN       2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Layout-compatible mirror of the fwlib UART_InitTypeDef (all u32, same
 * order); passed by address to UART_Init().
 */

struct ameba_uart_init_s
{
  uint32_t dmamodectrl;        /* DmaModeCtrl     */
  uint32_t wordlen;            /* WordLen         */
  uint32_t stopbit;            /* StopBit         */
  uint32_t parity;             /* Parity          */
  uint32_t paritytype;         /* ParityType      */
  uint32_t stickparity;        /* StickParity     */
  uint32_t flowcontrol;        /* FlowControl     */
  uint32_t rxfifotriglevel;    /* RxFifoTrigLevel */
  uint32_t rxerreportctrl;     /* RxErReportCtrl  */
  uint32_t rxtimeoutcnt;       /* RxTimeOutCnt    */
};

struct ameba_uart_dev_s
{
  struct uart_dev_s dev;       /* Serial upper-half device (must be first) */
  uintptr_t base;              /* UART register base address */
  int       irq;               /* NuttX interrupt vector */
  uint32_t  clk;               /* APBPeriph mask for RCC_PeriphClockCmd() */
  uint32_t  baud;              /* Baud rate */
  uint8_t   txpin;             /* TX pad (AMEBA_PA()/AMEBA_PB() encoding) */
  uint8_t   rxpin;             /* RX pad (AMEBA_PA()/AMEBA_PB() encoding) */
  uint8_t   txfid;             /* Pin mux function code for the TX pad */
  uint8_t   rxfid;             /* Pin mux function code for the RX pad */
  uint8_t   bits;              /* Data bits (7 or 8) */
  uint8_t   parity;            /* AMEBA_PARITY_NONE / _ODD / _EVEN */
  bool      stop2;             /* True: 2 stop bits, false: 1 */
  bool      txint;             /* True while the TX interrupt is unmasked */
  spinlock_t lock;             /* Serializes IER updates vs the ISR */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK fwlib UART/pin/clock API (all resolve to the ROM symbol table). */

extern void RCC_PeriphClockCmd(uint32_t periph, uint32_t clock,
                               uint8_t newstate);
extern void Pinmux_Config(uint8_t pin, uint32_t func);
extern void PAD_PullCtrl(uint8_t pin, uint8_t pull);
extern void UART_DeInit(void *uartx);
extern void UART_StructInit(struct ameba_uart_init_s *init);
extern void UART_Init(void *uartx, struct ameba_uart_init_s *init);
extern void UART_SetBaud(void *uartx, uint32_t baud);
extern void UART_RxCmd(void *uartx, uint32_t newstate);
extern uint32_t UART_Writable(void *uartx);
extern uint32_t UART_Readable(void *uartx);
extern void UART_CharPut(void *uartx, uint8_t txdata);
extern void UART_CharGet(void *uartx, uint8_t *rxbyte);
extern void UART_INTConfig(void *uartx, uint32_t uart_it, uint32_t newstate);
extern uint32_t UART_LineStatusGet(void *uartx);

/* Serial lower-half operations. */

static int  ameba_uart_setup(struct uart_dev_s *dev);
static void ameba_uart_shutdown(struct uart_dev_s *dev);
static int  ameba_uart_attach(struct uart_dev_s *dev);
static void ameba_uart_detach(struct uart_dev_s *dev);
static int  ameba_uart_ioctl(struct file *filep, int cmd,
                             unsigned long arg);
static int  ameba_uart_receive(struct uart_dev_s *dev, unsigned int *status);
static void ameba_uart_rxint(struct uart_dev_s *dev, bool enable);
static bool ameba_uart_rxavailable(struct uart_dev_s *dev);
static void ameba_uart_send(struct uart_dev_s *dev, int ch);
static void ameba_uart_txint(struct uart_dev_s *dev, bool enable);
static bool ameba_uart_txready(struct uart_dev_s *dev);
static bool ameba_uart_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_ameba_uart_ops =
{
  .setup       = ameba_uart_setup,
  .shutdown    = ameba_uart_shutdown,
  .attach      = ameba_uart_attach,
  .detach      = ameba_uart_detach,
  .ioctl       = ameba_uart_ioctl,
  .receive     = ameba_uart_receive,
  .rxint       = ameba_uart_rxint,
  .rxavailable = ameba_uart_rxavailable,
  .send        = ameba_uart_send,
  .txint       = ameba_uart_txint,
  .txready     = ameba_uart_txready,
  .txempty     = ameba_uart_txempty,
};

/* Per-controller register base, peripheral clock and IRQ, indexed by the
 * AMEBA_UART0/AMEBA_UART1 controller number.
 */

static const uintptr_t g_uart_base[AMEBA_NUART] =
{
  AMEBA_UART0_BASE,
  AMEBA_UART1_BASE,
};

static const uint32_t g_uart_clk[AMEBA_NUART] =
{
  AMEBA_APBPERIPH_UART0,
  AMEBA_APBPERIPH_UART1,
};

static const int g_uart_irq[AMEBA_NUART] =
{
  AMEBA_IRQ_UART0,
  AMEBA_IRQ_UART1,
};

/* Direction-specific pad mux function codes, indexed by controller. */

static const uint8_t g_uart_txfid[AMEBA_NUART] =
{
  AMEBA_PINMUX_UART0_TXD,
  AMEBA_PINMUX_UART1_TXD,
};

static const uint8_t g_uart_rxfid[AMEBA_NUART] =
{
  AMEBA_PINMUX_UART0_RXD,
  AMEBA_PINMUX_UART1_RXD,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_uart_configure
 *
 * Description:
 *   Gate the peripheral clock, route the TX/RX pads to the UART function and
 *   program the line format and baud through fwlib UART_Init()/SetBaud(),
 *   then enable the receiver.  Called from setup() and on a termios change.
 *
 ****************************************************************************/

static void ameba_uart_configure(struct ameba_uart_dev_s *priv)
{
  struct ameba_uart_init_s init;
  void *uartx = (void *)priv->base;

  /* Gate on the UART peripheral clock (idempotent). */

  RCC_PeriphClockCmd(priv->clk, priv->clk, AMEBA_ENABLE);

  /* Route the pads to this controller's TX/RX signals through the crossbar;
   * hold RX high while idle.
   */

  Pinmux_Config(priv->txpin, priv->txfid);
  Pinmux_Config(priv->rxpin, priv->rxfid);
  PAD_PullCtrl(priv->rxpin, AMEBA_GPIO_PUPD_UP);

  /* Start from the fwlib defaults, then apply the requested line format. */

  UART_StructInit(&init);

  init.wordlen         = (priv->bits == 7) ?
                         AMEBA_WLS_7BITS : AMEBA_WLS_8BITS;
  init.stopbit         = priv->stop2 ? AMEBA_STOP_BIT_2 : AMEBA_STOP_BIT_1;
  init.flowcontrol     = AMEBA_DISABLE;
  init.rxfifotriglevel = AMEBA_RX_FIFOTRIG_1;

  if (priv->parity == AMEBA_PARITY_NONE)
    {
      init.parity     = AMEBA_PARITY_DISABLE;
    }
  else
    {
      init.parity     = AMEBA_PARITY_ENABLE;
      init.paritytype = (priv->parity == AMEBA_PARITY_ODD) ?
                        AMEBA_ODD_PARITY : AMEBA_EVEN_PARITY;
    }

  UART_Init(uartx, &init);
  UART_SetBaud(uartx, priv->baud);
  UART_RxCmd(uartx, AMEBA_ENABLE);
}

/****************************************************************************
 * Name: ameba_uart_interrupt
 *
 * Description:
 *   Common UART interrupt handler.  Drains the RX FIFO into the receive
 *   buffer and, while the TX interrupt is unmasked, refills the TX FIFO from
 *   the transmit buffer.
 *
 ****************************************************************************/

static int ameba_uart_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;
  void *uartx = (void *)priv->base;

  UNUSED(irq);
  UNUSED(context);

  /* Reading the line status register clears any latched RX error bits. */

  UART_LineStatusGet(uartx);

  if (UART_Readable(uartx))
    {
      uart_recvchars(dev);
    }

  if (priv->txint && UART_Writable(uartx))
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_uart_setup
 ****************************************************************************/

static int ameba_uart_setup(struct uart_dev_s *dev)
{
  ameba_uart_configure((struct ameba_uart_dev_s *)dev);
  return OK;
}

/****************************************************************************
 * Name: ameba_uart_shutdown
 ****************************************************************************/

static void ameba_uart_shutdown(struct uart_dev_s *dev)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;

  ameba_uart_rxint(dev, false);
  ameba_uart_txint(dev, false);
  UART_DeInit((void *)priv->base);
}

/****************************************************************************
 * Name: ameba_uart_attach
 ****************************************************************************/

static int ameba_uart_attach(struct uart_dev_s *dev)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;
  int ret;

  ret = irq_attach(priv->irq, ameba_uart_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: ameba_uart_detach
 ****************************************************************************/

static void ameba_uart_detach(struct uart_dev_s *dev)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: ameba_uart_ioctl
 ****************************************************************************/

static int ameba_uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;

  switch (cmd)
    {
      case TCGETS:
        {
          struct termios *termiosp = (struct termios *)(uintptr_t)arg;
          tcflag_t ccflag;

          if (termiosp == NULL)
            {
              return -EINVAL;
            }

          ccflag = (priv->bits == 7) ? CS7 : CS8;
          if (priv->parity != AMEBA_PARITY_NONE)
            {
              ccflag |= PARENB;
              if (priv->parity == AMEBA_PARITY_ODD)
                {
                  ccflag |= PARODD;
                }
            }

          if (priv->stop2)
            {
              ccflag |= CSTOPB;
            }

          termiosp->c_cflag = ccflag;
          cfsetispeed(termiosp, priv->baud);
          cfsetospeed(termiosp, priv->baud);
          return OK;
        }

      case TCSETS:
        {
          struct termios *termiosp = (struct termios *)(uintptr_t)arg;
          irqstate_t flags;

          if (termiosp == NULL)
            {
              return -EINVAL;
            }

          flags = spin_lock_irqsave(&priv->lock);

          priv->bits  = ((termiosp->c_cflag & CSIZE) == CS7) ? 7 : 8;
          priv->stop2 = (termiosp->c_cflag & CSTOPB) != 0;

          if ((termiosp->c_cflag & PARENB) == 0)
            {
              priv->parity = AMEBA_PARITY_NONE;
            }
          else if ((termiosp->c_cflag & PARODD) != 0)
            {
              priv->parity = AMEBA_PARITY_ODD;
            }
          else
            {
              priv->parity = AMEBA_PARITY_EVEN;
            }

          priv->baud = cfgetispeed(termiosp);

          ameba_uart_configure(priv);

          spin_unlock_irqrestore(&priv->lock, flags);
          return OK;
        }

      default:
        return -ENOTTY;
    }
#else
  UNUSED(filep);
  UNUSED(cmd);
  UNUSED(arg);
  return -ENOTTY;
#endif
}

/****************************************************************************
 * Name: ameba_uart_receive
 ****************************************************************************/

static int ameba_uart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;
  uint8_t rxbyte = 0;

  *status = 0;
  UART_CharGet((void *)priv->base, &rxbyte);
  return rxbyte;
}

/****************************************************************************
 * Name: ameba_uart_rxint
 ****************************************************************************/

static void ameba_uart_rxint(struct uart_dev_s *dev, bool enable)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  /* Enable both the RX-trigger and RX-timeout sources so that bytes still
   * held below the FIFO trigger level are delivered promptly.
   */

  UART_INTConfig((void *)priv->base,
                 AMEBA_UART_INT_ERBI | AMEBA_UART_INT_ETOI,
                 enable ? AMEBA_ENABLE : AMEBA_DISABLE);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: ameba_uart_rxavailable
 ****************************************************************************/

static bool ameba_uart_rxavailable(struct uart_dev_s *dev)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;

  return UART_Readable((void *)priv->base) != 0;
}

/****************************************************************************
 * Name: ameba_uart_send
 ****************************************************************************/

static void ameba_uart_send(struct uart_dev_s *dev, int ch)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;

  UART_CharPut((void *)priv->base, (uint8_t)ch);
}

/****************************************************************************
 * Name: ameba_uart_txint
 *
 * Description:
 *   Unmask/mask the TX-FIFO-empty interrupt.  Enabling it while the FIFO is
 *   already empty asserts the interrupt at once, so the ISR pumps the first
 *   bytes; uart_xmitchars() re-disables it once the transmit buffer drains.
 *
 ****************************************************************************/

static void ameba_uart_txint(struct uart_dev_s *dev, bool enable)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  priv->txint = enable;
  UART_INTConfig((void *)priv->base, AMEBA_UART_INT_ETBEI,
                 enable ? AMEBA_ENABLE : AMEBA_DISABLE);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: ameba_uart_txready
 ****************************************************************************/

static bool ameba_uart_txready(struct uart_dev_s *dev)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;

  return UART_Writable((void *)priv->base) != 0;
}

/****************************************************************************
 * Name: ameba_uart_txempty
 ****************************************************************************/

static bool ameba_uart_txempty(struct uart_dev_s *dev)
{
  struct ameba_uart_dev_s *priv = (struct ameba_uart_dev_s *)dev;

  return (UART_LineStatusGet((void *)priv->base) &
          AMEBA_UART_LSR_TX_EMPTY) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_uart_register
 *
 * Description:
 *   See ameba_uart.h.
 *
 ****************************************************************************/

int ameba_uart_register(const char *path, int uart, uint8_t txpin,
                        uint8_t rxpin, uint32_t baud)
{
  struct ameba_uart_dev_s *priv;
  char *rxbuffer;
  char *txbuffer;
  int ret;

  if (uart < 0 || uart >= AMEBA_NUART || path == NULL || baud == 0)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct ameba_uart_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  rxbuffer = kmm_malloc(CONFIG_AMEBA_UART_RXBUFSIZE);
  txbuffer = kmm_malloc(CONFIG_AMEBA_UART_TXBUFSIZE);
  if (rxbuffer == NULL || txbuffer == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  priv->base   = g_uart_base[uart];
  priv->clk    = g_uart_clk[uart];
  priv->irq    = g_uart_irq[uart];
  priv->txpin  = txpin;
  priv->rxpin  = rxpin;
  priv->txfid  = g_uart_txfid[uart];
  priv->rxfid  = g_uart_rxfid[uart];
  priv->baud   = baud;
  priv->bits   = 8;
  priv->parity = AMEBA_PARITY_NONE;
  priv->stop2  = false;
  spin_lock_init(&priv->lock);

  priv->dev.recv.size   = CONFIG_AMEBA_UART_RXBUFSIZE;
  priv->dev.recv.buffer = rxbuffer;
  priv->dev.xmit.size   = CONFIG_AMEBA_UART_TXBUFSIZE;
  priv->dev.xmit.buffer = txbuffer;
  priv->dev.ops         = &g_ameba_uart_ops;

  ret = uart_register(path, &priv->dev);
  if (ret < 0)
    {
      _err("ERROR: uart_register(%s) failed: %d\n", path, ret);
      goto errout;
    }

  return OK;

errout:
  kmm_free(rxbuffer);
  kmm_free(txbuffer);
  kmm_free(priv);
  return ret;
}
