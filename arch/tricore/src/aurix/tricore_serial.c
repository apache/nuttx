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
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include <arch/chip/chip.h>

#include "tricore_internal.h"
#include "tricore_asclin.h"

#if defined(CONFIG_ARCH_CHIP_TC3XX)
#  include "hardware/tc3xx_port.h"
#  include "tc3xx_wdt.h"
#elif defined(CONFIG_ARCH_CHIP_TC4XX)
#  include "hardware/tc4xx_port.h"
#endif

#include <chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_TC3XX_UART0) || defined(CONFIG_TC4XX_UART0)
#  define HAVE_UART0 1
#endif

/* Bit timing: fPD is fASCLINF, 16 times oversampling, sample point 8 */

#define ASCLIN_PRESCALER           1
#define ASCLIN_OVERSAMPLING        16
#define ASCLIN_SAMPLEPOINT         8

/* Largest BRG numerator and denominator */

#define ASCLIN_BRG_MAX             0xfff

/* ASCLIN register of a UART */

#define ASCLIN_REG(priv, reg)      ((priv)->uartbase + ASCLIN_##reg##_OFFSET)

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(HAVE_UART0)
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port     /* UART0 is console */
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    define SERIAL_CONSOLE  1
#  else
#    error "I'm confused... Do we have a serial console or not?"
#  endif
#else
#  undef  CONSOLE_DEV                        /* No console */
#  undef  CONFIG_UART0_SERIAL_CONSOLE
#  if defined(HAVE_UART0)
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    define SERIAL_CONSOLE  1
#  else
#    undef  TTYS0_DEV
#  endif
#endif

#undef HAVE_UART_DEVICE
#if defined(HAVE_UART0)
#  define HAVE_UART_DEVICE 1
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of tricore_earlyserialinit(),
 * tricore_serialinit(), and up_putc().
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t      uartbase; /* Base address of UART registers */
  uint32_t       baud;     /* Configured baud */
  uint8_t        irq;      /* IRQ associated with this UART */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

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

/* I/O buffers */

#ifdef HAVE_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef HAVE_UART0

static struct up_dev_s g_uart0priv =
{
  .uartbase  = TRICORE_ASCLIN0_BASE,
  .baud      = CONFIG_UART0_BAUD,
  .irq       = TRICORE_UART_RX_IRQ,
};

static uart_dev_t g_uart0port =
{
#if SERIAL_CONSOLE == 1
  .isconsole = 1,
#endif
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: asclin_modify
 ****************************************************************************/

static void asclin_modify(uintptr_t reg, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(reg) & ~clear) | set, reg);
}

/****************************************************************************
 * Name: asclin_pin_config
 *
 * Description:
 *   Set up a UART pin: RX as input with pull-up, TX as push-pull alternate
 *   output 2, both with the speed 1 pad driver.  On TC3xx the mode is in
 *   IOCR and the pad driver in PDR, under the CPU ENDINIT; on TC4xx both
 *   are in the PADCFG DRVCFG register of the pin.
 *
 ****************************************************************************/

static void asclin_pin_config(uintptr_t port, int pin, bool tx)
{
#if defined(CONFIG_ARCH_CHIP_TC3XX)
  int shift = TC3XX_PORT_IOCR_SHIFT(pin);

  asclin_modify(TC3XX_PORT_IOCR(port, pin), TC3XX_PORT_IOCR_MASK << shift,
                (tx ? TC3XX_PORT_OUT_ALT2 : TC3XX_PORT_IN_PULLUP) << shift);

  shift = TC3XX_PORT_PDR_SHIFT(pin);
  tc3xx_wdt_cpu_endinit(false);
  asclin_modify(TC3XX_PORT_PDR(port, pin), TC3XX_PORT_PDR_MASK << shift,
                TC3XX_PORT_PAD_SPEED1 << shift);
  tc3xx_wdt_cpu_endinit(true);
#elif defined(CONFIG_ARCH_CHIP_TC4XX)
  uintptr_t drvcfg = TC4XX_PORT_DRVCFG(port, pin);

  asclin_modify(drvcfg, TC4XX_PORT_DRVCFG_MODE_MASK,
                tx ? TC4XX_PORT_OUT_ALT2 : TC4XX_PORT_IN_PULLUP);
  asclin_modify(drvcfg, TC4XX_PORT_DRVCFG_PAD_MASK, TC4XX_PORT_PAD_SPEED1);
#endif
}

/****************************************************************************
 * Name: asclin_set_clock
 *
 * Description:
 *   Select the ASCLIN clock and wait until it is on or off.
 *
 ****************************************************************************/

static void asclin_set_clock(struct up_dev_s *priv, uint32_t clksel)
{
  uint32_t on = clksel == ASCLIN_CSR_CLKSEL_NONE ? 0 : ASCLIN_CSR_CON;

  asclin_modify(ASCLIN_REG(priv, CSR), ASCLIN_CSR_CLKSEL_MASK, clksel);
  while ((getreg32(ASCLIN_REG(priv, CSR)) & ASCLIN_CSR_CON) != on);
}

/****************************************************************************
 * Name: asclin_set_bittime
 *
 * Description:
 *   Set the prescaler, the oversampling and the sample point, with three
 *   samples per bit.
 *
 ****************************************************************************/

static void asclin_set_bittime(struct up_dev_s *priv)
{
  asclin_modify(ASCLIN_REG(priv, BITCON),
                ASCLIN_BITCON_PRESCALER_MASK | ASCLIN_BITCON_OVS_MASK |
                ASCLIN_BITCON_SP_MASK | ASCLIN_BITCON_SM,
                (ASCLIN_PRESCALER - 1) |
                ((ASCLIN_OVERSAMPLING - 1) << ASCLIN_BITCON_OVS_SHIFT) |
                (ASCLIN_SAMPLEPOINT << ASCLIN_BITCON_SP_SHIFT) |
                ASCLIN_BITCON_SM);
}

/****************************************************************************
 * Name: asclin_set_baudrate
 *
 * Description:
 *   Set the fractional divider, which makes the oversampling clock
 *   fPD * NUMERATOR / DENOMINATOR.  The fraction is the last continued
 *   fraction convergent of (baud * oversampling) / fPD whose terms fit in
 *   the 12-bit BRG fields.
 *
 ****************************************************************************/

static void asclin_set_baudrate(struct up_dev_s *priv)
{
  uint32_t n = priv->baud * ASCLIN_OVERSAMPLING;
  uint32_t d = tricore_clock_asclinf() / ASCLIN_PRESCALER;
  uint32_t num0 = 0;
  uint32_t num1 = 1;
  uint32_t den0 = 1;
  uint32_t den1 = 0;

  while (d != 0)
    {
      uint32_t a = n / d;
      uint32_t r = n % d;
      uint32_t num2;
      uint32_t den2;

      if (a > (ASCLIN_BRG_MAX - num0) / num1 ||
          (den1 != 0 && a > (ASCLIN_BRG_MAX - den0) / den1))
        {
          break;
        }

      num2 = a * num1 + num0;
      den2 = a * den1 + den0;
      num0 = num1;
      num1 = num2;
      den0 = den1;
      den1 = den2;
      n = d;
      d = r;
    }

  asclin_modify(ASCLIN_REG(priv, BRG),
                ASCLIN_BRG_DENOMINATOR_MASK | ASCLIN_BRG_NUMERATOR_MASK,
                ((num1 != 0 ? num1 : 1) << ASCLIN_BRG_NUMERATOR_SHIFT) |
                (den1 != 0 ? den1 : 1));
}

/****************************************************************************
 * Name: asclin_init
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.  8N1, FIFO interrupts at
 *   one byte.
 *
 ****************************************************************************/

static void asclin_init(struct up_dev_s *priv)
{
  /* Enable the module, under the CPU ENDINIT on TC3xx */

#ifdef CONFIG_ARCH_CHIP_TC3XX
  tc3xx_wdt_cpu_endinit(false);
#endif
  asclin_modify(ASCLIN_REG(priv, CLC), ASCLIN_CLC_DISR, 0);
#ifdef CONFIG_ARCH_CHIP_TC3XX
  tc3xx_wdt_cpu_endinit(true);
#endif

  /* Clock off, initialise mode, bit timing and baud rate */

  asclin_set_clock(priv, ASCLIN_CSR_CLKSEL_NONE);
  asclin_modify(ASCLIN_REG(priv, FRAMECON), ASCLIN_FRAMECON_MODE_MASK,
                ASCLIN_FRAMECON_MODE_INIT);
  asclin_set_bittime(priv);
  asclin_set_baudrate(priv);

  /* No loop back, 8 data bits, no parity, 1 stop bit, LSB first, one byte
   * wide FIFO accesses, interrupts at the FIFO level, ASC mode
   */

  asclin_modify(ASCLIN_REG(priv, IOCR), ASCLIN_IOCR_LB, 0);
  asclin_modify(ASCLIN_REG(priv, FRAMECON),
                ASCLIN_FRAMECON_PEN | ASCLIN_FRAMECON_ODD |
                ASCLIN_FRAMECON_STOP_MASK | ASCLIN_FRAMECON_MSB |
                ASCLIN_FRAMECON_IDLE_MASK,
                1 << ASCLIN_FRAMECON_STOP_SHIFT);
  asclin_modify(ASCLIN_REG(priv, DATCON), ASCLIN_DATCON_DATLEN_MASK, 8 - 1);
  asclin_modify(ASCLIN_REG(priv, TXFIFOCON),
                ASCLIN_FIFOCON_W_MASK | ASCLIN_FIFOCON_INTLEVEL_MASK |
                ASCLIN_FIFOCON_FM_MASK, 1 << ASCLIN_FIFOCON_W_SHIFT);
  asclin_modify(ASCLIN_REG(priv, RXFIFOCON),
                ASCLIN_FIFOCON_W_MASK | ASCLIN_FIFOCON_INTLEVEL_MASK |
                ASCLIN_FIFOCON_FM_MASK, 1 << ASCLIN_FIFOCON_W_SHIFT);
  asclin_modify(ASCLIN_REG(priv, FRAMECON), ASCLIN_FRAMECON_MODE_MASK,
                ASCLIN_FRAMECON_MODE_ASC);

  /* Pins, RX on input A */

  asclin_pin_config(UART_PIN_PORT, UART_PIN_RX, false);
  asclin_modify(ASCLIN_REG(priv, IOCR), ASCLIN_IOCR_ALTI_MASK, 0);
  asclin_pin_config(UART_PIN_PORT, UART_PIN_TX, true);

  /* Clock on, error flags enabled, FIFOs enabled and flushed */

  asclin_set_clock(priv, ASCLIN_CSR_CLKSEL_FAST);
  putreg32(0, ASCLIN_REG(priv, FLAGSENABLE));
  putreg32(0xffffffff, ASCLIN_REG(priv, FLAGSCLEAR));
  asclin_modify(ASCLIN_REG(priv, FLAGSENABLE), 0,
                ASCLIN_FLAGS_PE | ASCLIN_FLAGS_FE | ASCLIN_FLAGS_RFO |
                ASCLIN_FLAGS_RFU | ASCLIN_FLAGS_TFO);
  asclin_modify(ASCLIN_REG(priv, RXFIFOCON), 0, ASCLIN_FIFOCON_EN);
  asclin_modify(ASCLIN_REG(priv, TXFIFOCON), 0, ASCLIN_FIFOCON_EN);
  asclin_modify(ASCLIN_REG(priv, RXFIFOCON), 0, ASCLIN_FIFOCON_FLUSH);
  asclin_modify(ASCLIN_REG(priv, TXFIFOCON), 0, ASCLIN_FIFOCON_FLUSH);
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
  asclin_init(dev->priv);

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
  struct up_dev_s *priv = dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);
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
  struct up_dev_s *priv = dev->priv;
  int ret;

  /* Initialize interrupt generation on the peripheral */

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
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = dev->priv;

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
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = arg;

  if (up_rxavailable(dev))
    {
      uart_recvchars(dev);
    }

  if (up_txready(dev))
    {
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
  struct up_dev_s *priv = dev->priv;

  return getreg32(ASCLIN_REG(priv, RXDATA));
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
  struct up_dev_s *priv = dev->priv;
  irqstate_t flags = enter_critical_section();

  asclin_modify(ASCLIN_REG(priv, FLAGSENABLE), ASCLIN_FLAGS_RFL,
                enable ? ASCLIN_FLAGS_RFL : 0);
  asclin_modify(ASCLIN_REG(priv, RXFIFOCON), ASCLIN_FIFOCON_EN,
                enable ? ASCLIN_FIFOCON_EN : 0);

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
  struct up_dev_s *priv = dev->priv;

  return (getreg32(ASCLIN_REG(priv, RXFIFOCON)) &
          ASCLIN_FIFOCON_FILL_MASK) != 0;
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
  struct up_dev_s *priv = dev->priv;

  /* Wait for FIFO */

  if (dev == &CONSOLE_DEV)
    {
      up_putc(ch);
      return;
    }

  while ((getreg32(ASCLIN_REG(priv, TXFIFOCON)) &
          ASCLIN_FIFOCON_FILL_MASK) != 0);

  putreg32(0xffffffff, ASCLIN_REG(priv, FLAGSCLEAR));
  putreg32(ch, ASCLIN_REG(priv, TXDATA));
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
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
      /* Enable the TX interrupt */

      uart_xmitchars(dev);
    }

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
  return true;
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
  struct up_dev_s *priv = dev->priv;

  /* Return true if the TX wartermak is pending */

  return (getreg32(ASCLIN_REG(priv, TXFIFOCON)) &
          ASCLIN_FIFOCON_FILL_MASK) != 0;
}

/****************************************************************************
 * Name: tricore_lowputc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void tricore_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  struct up_dev_s *priv = CONSOLE_DEV.priv;

  /* Wait for FIFO */

  while ((getreg32(ASCLIN_REG(priv, TXFIFOCON)) &
          ASCLIN_FIFOCON_FILL_MASK) != 0);

  putreg32(0xffffffff, ASCLIN_REG(priv, FLAGSCLEAR));
  putreg32(ch, ASCLIN_REG(priv, TXDATA));
#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: tricore_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before tricore_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void tricore_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: tricore_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that tricore_earlyserialinit was called previously.
 *
 ****************************************************************************/

void tricore_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
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
#ifdef HAVE_SERIAL_CONSOLE
  tricore_lowputc(ch);
#endif
}
#endif /* USE_SERIALDRIVER */
