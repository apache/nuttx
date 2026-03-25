/****************************************************************************
 * arch/arm/src/ht32f491x3/ht32f491x3_serial.c
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

#include <stdbool.h>
#include <errno.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>

#include "chip.h"
#include "arm_internal.h"

#include "ht32f491x3_config.h"
#include "ht32f491x3_lowputc.h"
#include "ht32f491x3_serial.h"

#include "hardware/ht32f491x3_crm.h"
#include "hardware/ht32f491x3_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_USART1_SERIAL_CONSOLE) && \
    defined(CONFIG_HT32F491X3_USART1_SERIALDRIVER)
#  define CONSOLE_USART 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && \
      defined(CONFIG_HT32F491X3_USART2_SERIALDRIVER)
#  define CONSOLE_USART 2
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && \
      defined(CONFIG_HT32F491X3_USART3_SERIALDRIVER)
#  define CONSOLE_USART 3
#else
#  define CONSOLE_USART 0
#endif

#define HT32_USART_CTRL1_USED_INTS \
  (HT32_USART_CTRL1_IDLEIEN | HT32_USART_CTRL1_RDBFIEN | \
   HT32_USART_CTRL1_TDCIEN | HT32_USART_CTRL1_TDBEIEN | \
   HT32_USART_CTRL1_PERRIEN)

#define HT32_USART_CTRL3_USED_INTS HT32_USART_CTRL3_ERRIEN

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase;
  uintptr_t apbreg;
  uint32_t apbmask;
  uint32_t apbclock;
  uint32_t baud;
  uint32_t ie;
  int irq;
  uint8_t parity;
  uint8_t bits;
  bool stopbits2;
  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int up_setup(FAR struct uart_dev_s *dev);
static void up_shutdown(FAR struct uart_dev_s *dev);
static int up_attach(FAR struct uart_dev_s *dev);
static void up_detach(FAR struct uart_dev_s *dev);
static int ht32_interrupt(int irq, FAR void *context, FAR void *arg);
static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int up_receive(FAR struct uart_dev_s *dev,
                      FAR unsigned int *status);
static void up_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(FAR struct uart_dev_s *dev);
static void up_send(FAR struct uart_dev_s *dev, int ch);
static void up_txint(FAR struct uart_dev_s *dev, bool enable);
static bool up_txready(FAR struct uart_dev_s *dev);
static bool up_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup       = up_setup,
  .shutdown    = up_shutdown,
  .attach      = up_attach,
  .detach      = up_detach,
  .ioctl       = up_ioctl,
  .receive     = up_receive,
  .rxint       = up_rxint,
  .rxavailable = up_rxavailable,
  .send        = up_send,
  .txint       = up_txint,
  .txready     = up_txready,
  .txempty     = up_txempty,
};

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART)

#ifdef CONFIG_HT32F491X3_USART1_SERIALDRIVER
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];

static struct up_dev_s g_usart1priv =
{
  .uartbase  = HT32_USART1_BASE,
  .apbreg    = HT32_CRM_APB2EN,
  .apbmask   = HT32_CRM_APB2EN_USART1EN,
  .apbclock  = HT32_PCLK2_FREQUENCY,
  .baud      = CONFIG_USART1_BAUD,
  .irq       = HT32_IRQ_USART1,
  .parity    = CONFIG_USART1_PARITY,
  .bits      = CONFIG_USART1_BITS,
  .stopbits2 = CONFIG_USART1_2STOP,
  .lock      = SP_UNLOCKED,
};

static uart_dev_t g_usart1port =
{
#if CONSOLE_USART == 1
  .isconsole = true,
#endif
  .recv      =
    {
      .size   = CONFIG_USART1_RXBUFSIZE,
      .buffer = g_usart1rxbuffer,
    },
  .xmit      =
    {
      .size   = CONFIG_USART1_TXBUFSIZE,
      .buffer = g_usart1txbuffer,
    },
  .ops       = &g_uart_ops,
  .priv      = &g_usart1priv,
};
#endif

#ifdef CONFIG_HT32F491X3_USART2_SERIALDRIVER
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];

static struct up_dev_s g_usart2priv =
{
  .uartbase  = HT32_USART2_BASE,
  .apbreg    = HT32_CRM_APB1EN,
  .apbmask   = HT32_CRM_APB1EN_USART2EN,
  .apbclock  = HT32_PCLK1_FREQUENCY,
  .baud      = CONFIG_USART2_BAUD,
  .irq       = HT32_IRQ_USART2,
  .parity    = CONFIG_USART2_PARITY,
  .bits      = CONFIG_USART2_BITS,
  .stopbits2 = CONFIG_USART2_2STOP,
  .lock      = SP_UNLOCKED,
};

static uart_dev_t g_usart2port =
{
#if CONSOLE_USART == 2
  .isconsole = true,
#endif
  .recv      =
    {
      .size   = CONFIG_USART2_RXBUFSIZE,
      .buffer = g_usart2rxbuffer,
    },
  .xmit      =
    {
      .size   = CONFIG_USART2_TXBUFSIZE,
      .buffer = g_usart2txbuffer,
    },
  .ops       = &g_uart_ops,
  .priv      = &g_usart2priv,
};
#endif

#ifdef CONFIG_HT32F491X3_USART3_SERIALDRIVER
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];

static struct up_dev_s g_usart3priv =
{
  .uartbase  = HT32_USART3_BASE,
  .apbreg    = HT32_CRM_APB1EN,
  .apbmask   = HT32_CRM_APB1EN_USART3EN,
  .apbclock  = HT32_PCLK1_FREQUENCY,
  .baud      = CONFIG_USART3_BAUD,
  .irq       = HT32_IRQ_USART3,
  .parity    = CONFIG_USART3_PARITY,
  .bits      = CONFIG_USART3_BITS,
  .stopbits2 = CONFIG_USART3_2STOP,
  .lock      = SP_UNLOCKED,
};

static uart_dev_t g_usart3port =
{
#if CONSOLE_USART == 3
  .isconsole = true,
#endif
  .recv      =
    {
      .size   = CONFIG_USART3_RXBUFSIZE,
      .buffer = g_usart3rxbuffer,
    },
  .xmit      =
    {
      .size   = CONFIG_USART3_TXBUFSIZE,
      .buffer = g_usart3txbuffer,
    },
  .ops       = &g_uart_ops,
  .priv      = &g_usart3priv,
};
#endif

static uart_dev_t *const g_uart_devs[] =
{
#ifdef CONFIG_HT32F491X3_USART1_SERIALDRIVER
  &g_usart1port,
#else
  NULL,
#endif
#ifdef CONFIG_HT32F491X3_USART2_SERIALDRIVER
  &g_usart2port,
#else
  NULL,
#endif
#ifdef CONFIG_HT32F491X3_USART3_SERIALDRIVER
  &g_usart3port,
#else
  NULL,
#endif
};

#define HT32_NUART_PORTS (sizeof(g_uart_devs) / sizeof(g_uart_devs[0]))

#if CONSOLE_USART == 1
#  define CONSOLE_DEVPTR (&g_usart1port)
#elif CONSOLE_USART == 2
#  define CONSOLE_DEVPTR (&g_usart2port)
#elif CONSOLE_USART == 3
#  define CONSOLE_DEVPTR (&g_usart3port)
#else
#  define CONSOLE_DEVPTR ((FAR uart_dev_t *)0)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t up_serialin(FAR struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

static inline void up_serialout(FAR struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

static uint32_t ht32f491x3_bauddiv(uint32_t clock, uint32_t baud)
{
  uint32_t div;

  div = (clock * 10u) / baud;
  return ((div % 10u) < 5u) ? (div / 10u) : (div / 10u + 1u);
}

static void up_setusartint_nolock(FAR struct up_dev_s *priv, uint32_t ie)
{
  uint32_t regval;

  priv->ie = ie;

  regval  = up_serialin(priv, HT32_USART_CTRL1_OFFSET);
  regval &= ~HT32_USART_CTRL1_USED_INTS;
  regval |= ie & HT32_USART_CTRL1_USED_INTS;
  up_serialout(priv, HT32_USART_CTRL1_OFFSET, regval);

  regval  = up_serialin(priv, HT32_USART_CTRL3_OFFSET);
  regval &= ~HT32_USART_CTRL3_USED_INTS;
  regval |= ie & HT32_USART_CTRL3_USED_INTS;
  up_serialout(priv, HT32_USART_CTRL3_OFFSET, regval);
}

static void up_restoreusartint(FAR struct up_dev_s *priv, uint32_t ie)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  up_setusartint_nolock(priv, ie);
  spin_unlock_irqrestore(&priv->lock, flags);
}

static void up_disableusartint(FAR struct up_dev_s *priv, FAR uint32_t *ie)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  if (ie != NULL)
    {
      uint32_t regval;

      regval  = up_serialin(priv, HT32_USART_CTRL1_OFFSET);
      *ie     = regval & HT32_USART_CTRL1_USED_INTS;
      regval  = up_serialin(priv, HT32_USART_CTRL3_OFFSET);
      *ie    |= regval & HT32_USART_CTRL3_USED_INTS;
    }

  up_setusartint_nolock(priv, 0);
  spin_unlock_irqrestore(&priv->lock, flags);
}

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void up_set_format(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  uint32_t regval;

  regval  = up_serialin(priv, HT32_USART_CTRL1_OFFSET);
  regval &= ~(HT32_USART_CTRL1_PEN | HT32_USART_CTRL1_PSEL |
              HT32_USART_CTRL1_DBN0 | HT32_USART_CTRL1_DBN1);

  if (priv->parity == 1)
    {
      regval |= HT32_USART_CTRL1_PEN;
    }
  else if (priv->parity == 2)
    {
      regval |= HT32_USART_CTRL1_PEN | HT32_USART_CTRL1_PSEL;
    }

  if (priv->bits == 9 || (priv->bits == 8 && priv->parity != 0))
    {
      regval |= HT32_USART_CTRL1_DBN0;
    }
  else if (priv->bits == 7)
    {
      regval |= HT32_USART_CTRL1_DBN1;
    }

  up_serialout(priv, HT32_USART_CTRL1_OFFSET, regval);

  regval  = up_serialin(priv, HT32_USART_CTRL2_OFFSET);
  regval &= ~HT32_USART_CTRL2_STOPBN_MASK;
  regval |= priv->stopbits2 ? HT32_USART_CTRL2_STOPBN_20
                            : HT32_USART_CTRL2_STOPBN_10;
  up_serialout(priv, HT32_USART_CTRL2_OFFSET, regval);

  regval  = up_serialin(priv, HT32_USART_CTRL3_OFFSET);
  regval &= ~(HT32_USART_CTRL3_RTSEN | HT32_USART_CTRL3_CTSEN |
              HT32_USART_CTRL3_RS485EN | HT32_USART_CTRL3_DEP);
  up_serialout(priv, HT32_USART_CTRL3_OFFSET, regval);

  up_serialout(priv, HT32_USART_BAUDR_OFFSET,
               ht32f491x3_bauddiv(priv->apbclock, priv->baud));
}
#endif

static int up_setup(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  uint32_t regval;

  modifyreg32(priv->apbreg, 0, priv->apbmask);
  ht32f491x3_usart_config(priv->uartbase);
  ht32f491x3_usart_pins(priv->uartbase);

  regval  = up_serialin(priv, HT32_USART_CTRL1_OFFSET);
  regval &= ~HT32_USART_CTRL1_UEN;
  up_serialout(priv, HT32_USART_CTRL1_OFFSET, regval);

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  up_set_format(dev);
#endif

  regval  = up_serialin(priv, HT32_USART_CTRL1_OFFSET);
  regval |= HT32_USART_CTRL1_UEN |
            HT32_USART_CTRL1_TEN |
            HT32_USART_CTRL1_REN;
  up_serialout(priv, HT32_USART_CTRL1_OFFSET, regval);

  return 0;
}

static void up_shutdown(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  uint32_t regval;

  up_disableusartint(priv, NULL);

  regval  = up_serialin(priv, HT32_USART_CTRL1_OFFSET);
  regval &= ~(HT32_USART_CTRL1_UEN |
              HT32_USART_CTRL1_TEN |
              HT32_USART_CTRL1_REN);
  up_serialout(priv, HT32_USART_CTRL1_OFFSET, regval);
}

static int up_attach(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  int ret;

  ret = irq_attach(priv->irq, ht32_interrupt, dev);
  if (ret < 0)
    {
      return ret;
    }

  up_enable_irq(priv->irq);
  return 0;
}

static void up_detach(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

static int ht32_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = (FAR struct uart_dev_s *)arg;
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  uint32_t status;

  UNUSED(irq);
  UNUSED(context);

  status = up_serialin(priv, HT32_USART_STS_OFFSET);

  if ((status & HT32_USART_STS_RDBF) != 0)
    {
      uart_recvchars(dev);
    }

  if ((status & HT32_USART_STS_TDBE) != 0 &&
      (priv->ie & HT32_USART_CTRL1_TDBEIEN) != 0)
    {
      uart_xmitchars(dev);
    }

  return 0;
}

static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  UNUSED(filep);
  UNUSED(cmd);
  UNUSED(arg);
  return -ENOTTY;
}

static int up_receive(FAR struct uart_dev_s *dev, FAR unsigned int *status)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  uint32_t regval;

  regval = up_serialin(priv, HT32_USART_STS_OFFSET);
  if (status != NULL)
    {
      *status = regval;
    }

  regval = up_serialin(priv, HT32_USART_RDR_OFFSET);
  return (int)(regval & (HT32_USART_DT_MASK >> HT32_USART_DT_SHIFT));
}

static void up_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  irqstate_t flags;
  uint32_t ie;

  flags = spin_lock_irqsave(&priv->lock);
  ie    = priv->ie;

  if (enable)
    {
      ie |= HT32_USART_CTRL1_RDBFIEN | HT32_USART_CTRL3_ERRIEN;
    }
  else
    {
      ie &= ~(HT32_USART_CTRL1_RDBFIEN | HT32_USART_CTRL3_ERRIEN);
    }

  up_setusartint_nolock(priv, ie);
  spin_unlock_irqrestore(&priv->lock, flags);
}

static bool up_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;

  return (up_serialin(priv, HT32_USART_STS_OFFSET) &
          HT32_USART_STS_RDBF) != 0;
}

static void up_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;

  up_serialout(priv, HT32_USART_TDR_OFFSET,
               (uint32_t)ch & (HT32_USART_DT_MASK >> HT32_USART_DT_SHIFT));
}

static void up_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  irqstate_t flags;
  uint32_t ie;

  flags = spin_lock_irqsave(&priv->lock);
  ie    = priv->ie;

  if (enable)
    {
      ie |= HT32_USART_CTRL1_TDBEIEN;
    }
  else
    {
      ie &= ~HT32_USART_CTRL1_TDBEIEN;
    }

  up_setusartint_nolock(priv, ie);
  spin_unlock_irqrestore(&priv->lock, flags);

  if (enable)
    {
      uart_xmitchars(dev);
    }
}

static bool up_txready(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;

  return (up_serialin(priv, HT32_USART_STS_OFFSET) &
          HT32_USART_STS_TDBE) != 0;
}

static bool up_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;

  return (up_serialin(priv, HT32_USART_STS_OFFSET) &
          HT32_USART_STS_TDC) != 0;
}

void arm_earlyserialinit(void)
{
  unsigned int i;

  for (i = 0; i < HT32_NUART_PORTS; i++)
    {
      if (g_uart_devs[i] != NULL)
        {
          up_disableusartint((FAR struct up_dev_s *)g_uart_devs[i]->priv,
                             NULL);
        }
    }

#if CONSOLE_USART > 0
  up_setup(CONSOLE_DEVPTR);
#endif
}

void arm_serialinit(void)
{
  char devname[] = "/dev/ttySx";
  unsigned int i;
  unsigned int minor = 0;

#if CONSOLE_USART > 0
  uart_register("/dev/console", CONSOLE_DEVPTR);
  uart_register("/dev/ttyS0", CONSOLE_DEVPTR);
  minor = 1;
#endif

  for (i = 0; i < HT32_NUART_PORTS; i++)
    {
      if (g_uart_devs[i] == NULL || g_uart_devs[i]->isconsole)
        {
          continue;
        }

      devname[9] = '0' + minor++;
      uart_register(devname, g_uart_devs[i]);
    }
}

#else

void arm_earlyserialinit(void)
{
}

void arm_serialinit(void)
{
}

#endif /* USE_SERIALDRIVER && HAVE_UART */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_CONSOLE
#  if defined(USE_SERIALDRIVER) && defined(HAVE_UART) && CONSOLE_USART > 0
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)CONSOLE_DEVPTR->priv;
  uint32_t ie;

  up_disableusartint(priv, &ie);
  arm_lowputc(ch);
  up_restoreusartint(priv, ie);
#  else
  arm_lowputc(ch);
#  endif
#endif
}
