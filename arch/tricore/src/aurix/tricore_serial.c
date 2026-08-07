/****************************************************************************
 * arch/tricore/src/aurix/tricore_serial.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/serial/serial.h>

#include "tricore_internal.h"
#include "tricore_asclin.h"

#ifndef CONFIG_TRICORE_ASCLINF_FREQUENCY
#  if defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
#    define CONFIG_TRICORE_ASCLINF_FREQUENCY  200000000
#  elif defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
#    define CONFIG_TRICORE_ASCLINF_FREQUENCY  100000000
#  endif
#endif

/* Default ASCLIN clock source selection */
#ifndef CONFIG_TRICORE_ASCLIN_CLK_SRC
#  define CONFIG_TRICORE_ASCLIN_CLK_SRC  ASCLIN_CSR_CLKSEL_FASCLINF
#endif

#ifndef CONFIG_TRICORE_ASCLIN_OVERSAMPLING
#  define CONFIG_TRICORE_ASCLIN_OVERSAMPLING  16u
#endif

#ifndef CONFIG_TRICORE_ASCLIN_SAMPLEPOINT
#  define CONFIG_TRICORE_ASCLIN_SAMPLEPOINT   8u
#endif

#ifndef CONFIG_TRICORE_ASCLIN_PRESCALER
#  define CONFIG_TRICORE_ASCLIN_PRESCALER     1u
#endif

#ifndef CONFIG_TRICORE_ASCLIN_MEDIAN_FILTER
#  define CONFIG_TRICORE_ASCLIN_MEDIAN_FILTER 1u
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ASCLIN_CLK_TIMEOUT          100000u

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_TRICORE_UART0)
#  define HAVE_SERIAL_CONSOLE       1
#  define CONSOLE_DEV               g_uart0port
#  define TTYS0_DEV                 g_uart0port
#  ifdef CONFIG_TRICORE_UART1
#    define TTYS1_DEV               g_uart1port
#  endif
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_TRICORE_UART1)
#  define HAVE_SERIAL_CONSOLE       1
#  define CONSOLE_DEV               g_uart1port
#  define TTYS0_DEV                 g_uart1port
#  ifdef CONFIG_TRICORE_UART0
#    define TTYS1_DEV               g_uart0port
#  endif
#elif defined(CONFIG_TRICORE_UART0)
#  define TTYS0_DEV                 g_uart0port
#  ifdef CONFIG_TRICORE_UART1
#    define TTYS1_DEV               g_uart1port
#  endif
#elif defined(CONFIG_TRICORE_UART1)
#  define TTYS0_DEV                 g_uart1port
#endif

struct aurix_uart_config_s
{
  uintptr_t base;
  uint32_t  fclk;
  uint32_t  baud;
  uint8_t   clk_src;
  uint8_t   oversampling;
  uint8_t   samplepoint;
  uint8_t   prescaler_val;
  uint8_t   median_filter;
  uint8_t   data_bits;
  uint8_t   stop_bits;
  uint8_t   parity;
  uint8_t   rxirq;
  uint8_t   txirq;
  uint8_t   erirq;
  uint8_t   rx_alti;
};

struct aurix_uart_priv_s
{
  struct aurix_uart_config_s *config;
  bool      txint_enabled;
  bool      rxint_enabled;
};

static int  aurix_setup(struct uart_dev_s *dev);
static void aurix_shutdown(struct uart_dev_s *dev);
static int  aurix_attach(struct uart_dev_s *dev);
static void aurix_detach(struct uart_dev_s *dev);
static int  aurix_interrupt(int irq, void *context, void *arg);
static int  aurix_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  aurix_receive(struct uart_dev_s *dev, unsigned int *status);
static void aurix_rxint(struct uart_dev_s *dev, bool enable);
static bool aurix_rxavailable(struct uart_dev_s *dev);
static void aurix_send(struct uart_dev_s *dev, int ch);
static void aurix_txint(struct uart_dev_s *dev, bool enable);
static bool aurix_txready(struct uart_dev_s *dev);
static bool aurix_txempty(struct uart_dev_s *dev);

static const struct uart_ops_s g_uart_ops =
{
  .setup          = aurix_setup,
  .shutdown       = aurix_shutdown,
  .attach         = aurix_attach,
  .detach         = aurix_detach,
  .ioctl          = aurix_ioctl,
  .receive        = aurix_receive,
  .rxint          = aurix_rxint,
  .rxavailable    = aurix_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = aurix_send,
  .txint          = aurix_txint,
  .txready        = aurix_txready,
  .txempty        = aurix_txempty,
};

#ifdef CONFIG_TRICORE_UART0

static struct aurix_uart_config_s g_uart0_config =
{
  .base           = TRICORE_ASCLIN_BASE(0),
  .fclk           = CONFIG_TRICORE_ASCLINF_FREQUENCY,
  .baud           = CONFIG_UART0_BAUD,
  .clk_src        = CONFIG_TRICORE_ASCLIN_CLK_SRC,
  .oversampling   = CONFIG_TRICORE_ASCLIN_OVERSAMPLING,
  .samplepoint    = CONFIG_TRICORE_ASCLIN_SAMPLEPOINT,
  .prescaler_val  = CONFIG_TRICORE_ASCLIN_PRESCALER,
  .median_filter  = CONFIG_TRICORE_ASCLIN_MEDIAN_FILTER,
  .data_bits      = CONFIG_UART0_BITS,
  .stop_bits      = CONFIG_UART0_2STOP ? 2 : 1,
#if defined(CONFIG_UART0_PARITY) && CONFIG_UART0_PARITY == 1
  .parity         = 2,
#elif defined(CONFIG_UART0_PARITY) && CONFIG_UART0_PARITY == 2
  .parity         = 1,
#else
  .parity         = 0,
#endif
  .rxirq          = CONFIG_TRICORE_UART0_RXIRQ,
  .txirq          = CONFIG_TRICORE_UART0_TXIRQ,
  .erirq          = CONFIG_TRICORE_UART0_ERIRQ,

  .rx_alti        = ASCLIN_IOCR_ALTI_A,
};

static struct aurix_uart_priv_s g_uart0_priv =
{
  .config         = &g_uart0_config,
  .txint_enabled  = false,
  .rxint_enabled  = false,
};

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static uart_dev_t g_uart0port =
{
#ifdef HAVE_SERIAL_CONSOLE
  .isconsole = true,
#endif
  .recv =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart0_priv,
};

#endif /* CONFIG_TRICORE_UART0 */

#ifdef CONFIG_TRICORE_UART1

static struct aurix_uart_config_s g_uart1_config =
{
  .base           = TRICORE_ASCLIN_BASE(1),
  .fclk           = CONFIG_TRICORE_ASCLINF_FREQUENCY,
  .baud           = CONFIG_UART1_BAUD,
  .clk_src        = CONFIG_TRICORE_ASCLIN_CLK_SRC,
  .oversampling   = CONFIG_TRICORE_ASCLIN_OVERSAMPLING,
  .samplepoint    = CONFIG_TRICORE_ASCLIN_SAMPLEPOINT,
  .prescaler_val  = CONFIG_TRICORE_ASCLIN_PRESCALER,
  .median_filter  = CONFIG_TRICORE_ASCLIN_MEDIAN_FILTER,
  .data_bits      = CONFIG_UART1_BITS,
  .stop_bits      = CONFIG_UART1_2STOP ? 2 : 1,
#if defined(CONFIG_UART1_PARITY) && CONFIG_UART1_PARITY == 1
  .parity         = 2,
#elif defined(CONFIG_UART1_PARITY) && CONFIG_UART1_PARITY == 2
  .parity         = 1,
#else
  .parity         = 0,
#endif
  .rxirq          = CONFIG_TRICORE_UART1_RXIRQ,
  .txirq          = CONFIG_TRICORE_UART1_TXIRQ,
  .erirq          = CONFIG_TRICORE_UART1_ERIRQ,

  .rx_alti        = ASCLIN_IOCR_ALTI_A,
};

static struct aurix_uart_priv_s g_uart1_priv =
{
  .config         = &g_uart1_config,
  .txint_enabled  = false,
  .rxint_enabled  = false,
};

static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static uart_dev_t g_uart1port =
{
#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(HAVE_SERIAL_CONSOLE)
  .isconsole = true,
#endif
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
  .priv = &g_uart1_priv,
};

#endif /* CONFIG_TRICORE_UART1 */

static inline uintptr_t aurix_serialreg(
    const struct aurix_uart_config_s *cfg, uint32_t offset)
{
  return cfg->base + offset;
}

static inline uint32_t aurix_serialin(
    const struct aurix_uart_config_s *cfg, uint32_t offset)
{
  return getreg32(aurix_serialreg(cfg, offset));
}

static inline void aurix_serialout(
    const struct aurix_uart_config_s *cfg, uint32_t offset, uint32_t value)
{
  putreg32(value, aurix_serialreg(cfg, offset));
}

static inline void aurix_serialmod(
    const struct aurix_uart_config_s *cfg, uint32_t offset,
    uint32_t setbits, uint32_t clrbits)
{
  modreg32(setbits, setbits | clrbits, aurix_serialreg(cfg, offset));
}

static int aurix_asclin_wait_clk(
    const struct aurix_uart_config_s *cfg, bool on)
{
  uint32_t timeout = ASCLIN_CLK_TIMEOUT;

  while (timeout--)
    {
      uint32_t csr = aurix_serialin(cfg, ASCLIN_CSR_OFFSET);
      bool con = !!(csr & ASCLIN_CSR_CON);
      if (con == on)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

static int aurix_asclin_set_clk(
    const struct aurix_uart_config_s *cfg, uint8_t clksel)
{
  aurix_serialout(cfg, ASCLIN_CSR_OFFSET, clksel);
  return aurix_asclin_wait_clk(cfg, clksel != ASCLIN_CSR_CLKSEL_NONE);
}

static void aurix_asclin_set_baudrate(
    const struct aurix_uart_config_s *cfg)
{
  uint32_t fovs = cfg->baud * cfg->oversampling;
  uint32_t fpd = cfg->fclk / cfg->prescaler_val;

  uint32_t n = fovs;
  uint32_t d = fpd;
  uint32_t h0 = 0;
  uint32_t h1 = 1;
  uint32_t k0 = 1;
  uint32_t k1 = 0;

  while (d != 0)
    {
      uint32_t a = n / d;
      uint32_t r = n - a * d;
      uint32_t h2 = a * h1 + h0;
      uint32_t k2 = a * k1 + k0;

      if (k2 > 4095u)
        {
          break;
        }

      h0 = h1;
      h1 = h2;
      k0 = k1;
      k1 = k2;

      n = d;
      d = r;
    }

  if (h1 == 0)
    {
      h1 = 1;
    }

  if (k1 == 0)
    {
      k1 = 1;
    }

  aurix_serialout(cfg, ASCLIN_BRG_OFFSET, ASCLIN_BRG_VAL(h1, k1));
}

static void aurix_asclin_set_bittime(
    const struct aurix_uart_config_s *cfg)
{
  uint32_t val = 0;

  val |= ((cfg->prescaler_val - 1u) & 0xfffu)
           << ASCLIN_BITCON_PRESCALER_SHIFT;

  val |= (((uint32_t)(cfg->oversampling - 1u)) & 0xfu)
           << ASCLIN_BITCON_OVERSAMPLING_SHIFT;

  val |= ((uint32_t)cfg->samplepoint & 0xfu)
           << ASCLIN_BITCON_SAMPLEPOINT_SHIFT;

  if (cfg->median_filter)
    {
      val |= ASCLIN_BITCON_SM;
    }

  aurix_serialout(cfg, ASCLIN_BITCON_OFFSET, val);
}

static void aurix_asclin_set_frame(
    const struct aurix_uart_config_s *cfg)
{
  uint32_t framecon;
  uint32_t datcon;
  uint32_t txfifocon;
  uint32_t rxfifocon;

  framecon = ASCLIN_FRAMECON_MODE_ASC;

  framecon |= ((uint32_t)cfg->stop_bits & 0x7u)
              << ASCLIN_FRAMECON_STOP_SHIFT;

  if (cfg->parity != 0)
    {
      framecon |= ASCLIN_FRAMECON_PEN;
      if (cfg->parity == 2)
        {
          framecon |= ASCLIN_FRAMECON_ODD;
        }
    }

  aurix_serialout(cfg, ASCLIN_FRAMECON_OFFSET, framecon);

  datcon = ASCLIN_DATCON_DATLEN(cfg->data_bits);
  aurix_serialout(cfg, ASCLIN_DATCON_OFFSET, datcon);

  txfifocon = ASCLIN_TXFIFOCON_FLUSH
            | ASCLIN_TXFIFOCON_ENO
            | ASCLIN_TXFIFOCON_FM_COMBINED
            | ASCLIN_TXFIFOCON_INW_1;

  aurix_serialout(cfg, ASCLIN_TXFIFOCON_OFFSET, txfifocon);

  rxfifocon = ASCLIN_RXFIFOCON_FLUSH
            | ASCLIN_RXFIFOCON_ENI
            | ASCLIN_RXFIFOCON_OUTW_1;

  aurix_serialout(cfg, ASCLIN_RXFIFOCON_OFFSET, rxfifocon);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int aurix_asclin_index(const struct aurix_uart_config_s *cfg)
{
  return (int)((cfg->base - TRICORE_ASCLIN_BASE(0)) / TRICORE_ASCLIN_STRIDE);
}

static int aurix_asclin_init(const struct aurix_uart_config_s *cfg)
{
  int ret;
  uint32_t val;

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
  aurix_cpu_endinit_enable(false);
#endif

  val = aurix_serialin(cfg, ASCLIN_CLC_OFFSET);
  val &= ~ASCLIN_CLC_DISR;
  aurix_serialout(cfg, ASCLIN_CLC_OFFSET, val);

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
  aurix_cpu_endinit_enable(true);
#endif

  uint32_t timeout = ASCLIN_CLK_TIMEOUT;

  while ((aurix_serialin(cfg, ASCLIN_CLC_OFFSET) & ASCLIN_CLC_DISS) &&
         timeout--);

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  board_aurix_setup_serial_pin(aurix_asclin_index(cfg));

  ret = aurix_asclin_set_clk(cfg, ASCLIN_CSR_CLKSEL_NONE);
  if (ret < 0)
    {
      return ret;
    }

  aurix_serialout(cfg, ASCLIN_FRAMECON_OFFSET, 0);

  aurix_asclin_set_bittime(cfg);
  aurix_asclin_set_baudrate(cfg);

  val = (uint32_t)cfg->rx_alti & ASCLIN_IOCR_ALTI_MASK;
  aurix_serialout(cfg, ASCLIN_IOCR_OFFSET, val);

  aurix_asclin_set_frame(cfg);

  aurix_serialout(cfg, ASCLIN_FLAGSENABLE_OFFSET, 0);
  aurix_serialout(cfg, ASCLIN_FLAGSCLEAR_OFFSET, ASCLIN_FLAGSCLEAR_ALL);

  val = ASCLIN_FLAGSENABLE_PEE
      | ASCLIN_FLAGSENABLE_FEE
      | ASCLIN_FLAGSENABLE_RFOE
      | ASCLIN_FLAGSENABLE_RFUE
      | ASCLIN_FLAGSENABLE_TFOE;
  aurix_serialout(cfg, ASCLIN_FLAGSENABLE_OFFSET, val);

  ret = aurix_asclin_set_clk(cfg, cfg->clk_src);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

static int aurix_setup(struct uart_dev_s *dev)
{
  struct aurix_uart_priv_s *priv = dev->priv;

  return aurix_asclin_init(priv->config);
}

static void aurix_shutdown(struct uart_dev_s *dev)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;

  aurix_serialout(cfg, ASCLIN_FLAGSENABLE_OFFSET, 0);

  aurix_asclin_set_clk(cfg, ASCLIN_CSR_CLKSEL_NONE);

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
  aurix_cpu_endinit_enable(false);
#endif

  aurix_serialmod(cfg, ASCLIN_CLC_OFFSET, ASCLIN_CLC_DISR, 0);

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
  aurix_cpu_endinit_enable(true);
#endif
}

static int aurix_attach(struct uart_dev_s *dev)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;
  int ret;

  ret = irq_attach(cfg->rxirq, aurix_interrupt, dev);
  if (ret != OK)
    {
      return ret;
    }

  ret = irq_attach(cfg->txirq, aurix_interrupt, dev);
  if (ret != OK)
    {
      irq_detach(cfg->rxirq);
      return ret;
    }

  ret = irq_attach(cfg->erirq, aurix_interrupt, dev);
  if (ret != OK)
    {
      irq_detach(cfg->rxirq);
      irq_detach(cfg->txirq);
      return ret;
    }

  up_enable_irq(cfg->rxirq);
  up_enable_irq(cfg->txirq);
  up_enable_irq(cfg->erirq);

  return OK;
}

static void aurix_detach(struct uart_dev_s *dev)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;

  up_disable_irq(cfg->rxirq);
  up_disable_irq(cfg->txirq);
  up_disable_irq(cfg->erirq);

  irq_detach(cfg->rxirq);
  irq_detach(cfg->txirq);
  irq_detach(cfg->erirq);
}

static int aurix_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;
  uint32_t flags;

  flags = aurix_serialin(cfg, ASCLIN_FLAGS_OFFSET);

  if (flags & ASCLIN_FLAGS_RFL)
    {
      aurix_serialout(cfg, ASCLIN_FLAGSCLEAR_OFFSET,
                       ASCLIN_FLAGSCLEAR_RFLC);

      uart_recvchars(dev);
    }

  if (flags & ASCLIN_FLAGS_TFL)
    {
      aurix_serialout(cfg, ASCLIN_FLAGSCLEAR_OFFSET,
                       ASCLIN_FLAGSCLEAR_TFLC);

      uart_xmitchars(dev);
    }

  if (flags & (ASCLIN_FLAGS_PE | ASCLIN_FLAGS_FE | ASCLIN_FLAGS_CE |
               ASCLIN_FLAGS_RFO | ASCLIN_FLAGS_RFU))
    {
      aurix_serialout(cfg, ASCLIN_FLAGSCLEAR_OFFSET,
                       ASCLIN_FLAGSCLEAR_ALL_ERRS);
    }

  return OK;
}

static int aurix_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct aurix_uart_priv_s *priv = dev->priv;
  struct aurix_uart_config_s *cfg =
    (struct aurix_uart_config_s *)priv->config;
  int ret = OK;

  switch (cmd)
    {
      case TCGETS:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              return -EINVAL;
            }

          memset(termiosp, 0, sizeof(struct termios));

          cfsetispeed(termiosp, cfg->baud);
          cfsetospeed(termiosp, cfg->baud);

          switch (cfg->data_bits)
            {
              case 5:
                termiosp->c_cflag |= CS5;
                break;
              case 6:
                termiosp->c_cflag |= CS6;
                break;
              case 7:
                termiosp->c_cflag |= CS7;
                break;
              default:
                termiosp->c_cflag |= CS8;
                break;
            }

          if (cfg->parity == 1)
            {
              termiosp->c_cflag |= PARENB;
            }
          else if (cfg->parity == 2)
            {
              termiosp->c_cflag |= PARENB | PARODD;
            }

          if (cfg->stop_bits >= 2)
            {
              termiosp->c_cflag |= CSTOPB;
            }
        }
        break;

      case TCSETS:
      case TCSETSW:
      case TCSETSF:
        {
          struct termios *termiosp = (struct termios *)arg;

          if (!termiosp)
            {
              return -EINVAL;
            }

          cfg->baud = cfgetospeed(termiosp);

          switch (termiosp->c_cflag & CSIZE)
            {
              case CS5:
                cfg->data_bits = 5;
                break;
              case CS6:
                cfg->data_bits = 6;
                break;
              case CS7:
                cfg->data_bits = 7;
                break;
              default:
                cfg->data_bits = 8;
                break;
            }

          if (termiosp->c_cflag & PARENB)
            {
              cfg->parity = (termiosp->c_cflag & PARODD) ? 2 : 1;
            }
          else
            {
              cfg->parity = 0;
            }

          cfg->stop_bits = (termiosp->c_cflag & CSTOPB) ? 2 : 1;

          aurix_shutdown(dev);
          aurix_setup(dev);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int aurix_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;
  uint32_t flags;
  int ch;

  ch = (int)(aurix_serialin(cfg, ASCLIN_RXDATA_OFFSET) & 0xff);

  flags = aurix_serialin(cfg, ASCLIN_FLAGS_OFFSET);
  *status = flags & (ASCLIN_FLAGS_PE | ASCLIN_FLAGS_FE |
                     ASCLIN_FLAGS_RFO | ASCLIN_FLAGS_BD);

  return ch;
}

static void aurix_rxint(struct uart_dev_s *dev, bool enable)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
      aurix_serialmod(cfg, ASCLIN_FLAGSENABLE_OFFSET,
                       ASCLIN_FLAGSENABLE_RFLE, 0);
    }
  else
    {
      aurix_serialmod(cfg, ASCLIN_FLAGSENABLE_OFFSET,
                       0, ASCLIN_FLAGSENABLE_RFLE);
    }

  priv->rxint_enabled = enable;
  leave_critical_section(flags);
}

static bool aurix_rxavailable(struct uart_dev_s *dev)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;
  uint32_t rxfifocon;

  rxfifocon = aurix_serialin(cfg, ASCLIN_RXFIFOCON_OFFSET);
  return ASCLIN_RX_FILL(rxfifocon) > 0;
}

static void aurix_send(struct uart_dev_s *dev, int ch)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;

  aurix_serialout(cfg, ASCLIN_TXDATA_OFFSET, (uint32_t)(uint8_t)ch);
}

static void aurix_txint(struct uart_dev_s *dev, bool enable)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
      aurix_serialmod(cfg, ASCLIN_FLAGSENABLE_OFFSET,
                       ASCLIN_FLAGSENABLE_TFLE, 0);

      uart_xmitchars(dev);
    }
  else
    {
      aurix_serialmod(cfg, ASCLIN_FLAGSENABLE_OFFSET,
                       0, ASCLIN_FLAGSENABLE_TFLE);
    }

  priv->txint_enabled = enable;
  leave_critical_section(flags);
}

static bool aurix_txready(struct uart_dev_s *dev)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;
  uint32_t txfifocon;

  txfifocon = aurix_serialin(cfg, ASCLIN_TXFIFOCON_OFFSET);
  return ASCLIN_TX_FILL(txfifocon) < ASCLIN_FIFO_DEPTH;
}

static bool aurix_txempty(struct uart_dev_s *dev)
{
  struct aurix_uart_priv_s *priv = dev->priv;
  const struct aurix_uart_config_s *cfg = priv->config;
  uint32_t txfifocon;

  txfifocon = aurix_serialin(cfg, ASCLIN_TXFIFOCON_OFFSET);
  return ASCLIN_TX_FILL(txfifocon) == 0;
}

static void aurix_lowputc(int ch)
{
#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_TRICORE_UART1)
  const struct aurix_uart_config_s *cfg = &g_uart1_config;
#elif defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_TRICORE_UART0)
  const struct aurix_uart_config_s *cfg = &g_uart0_config;
#elif defined(CONFIG_TRICORE_UART0)
  const struct aurix_uart_config_s *cfg = &g_uart0_config;
#elif defined(CONFIG_TRICORE_UART1)
  const struct aurix_uart_config_s *cfg = &g_uart1_config;
#else
  (void)ch;
  return;
#endif

#if defined(CONFIG_TRICORE_UART0) || defined(CONFIG_TRICORE_UART1)
  uint32_t txfifocon;

  do
    {
      txfifocon = aurix_serialin(cfg, ASCLIN_TXFIFOCON_OFFSET);
    }
  while (ASCLIN_TX_FILL(txfifocon) >= ASCLIN_FIFO_DEPTH);

  aurix_serialout(cfg, ASCLIN_TXDATA_OFFSET, (uint32_t)(uint8_t)ch);
#endif
}

#ifdef USE_SERIALDRIVER

#ifdef USE_EARLYSERIALINIT
void aurix_earlyserialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  aurix_setup(&CONSOLE_DEV);
#endif
}
#endif

void aurix_serialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}

#endif /* USE_SERIALDRIVER */

void up_putc(int ch)
{
  if (ch == '\n')
    {
      aurix_lowputc('\r');
    }

  aurix_lowputc(ch);
}

void up_lowputc(int ch)
{
  up_putc(ch);
}
