/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_serial.c
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

#include "arm_internal.h"
#include "chip.h"
#include "hardware/lpc17_40_uart.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_serial.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART)

/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t baud;      /* Configured baud */
  uint32_t ier;       /* Saved IER value */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
#ifdef LPC176x
  uint8_t  cclkdiv;   /* Divisor needed to get PCLK from CCLK */
#endif
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
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

#ifdef CONFIG_LPC17_40_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_LPC17_40_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_LPC17_40_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_LPC17_40_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif

/* This describes the state of the LPC17xx/LPC40xx uart0 port. */

#ifdef CONFIG_LPC17_40_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = LPC17_40_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = LPC17_40_IRQ_UART0,
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

/* This describes the state of the LPC17xx/LPC40xx uart1 port. */

#ifdef CONFIG_LPC17_40_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = LPC17_40_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = LPC17_40_IRQ_UART1,
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

/* This describes the state of the LPC17xx/LPC40xx uart1 port. */

#ifdef CONFIG_LPC17_40_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = LPC17_40_UART2_BASE,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = LPC17_40_IRQ_UART2,
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

/* This describes the state of the LPC17xx/LPC40xx uart1 port. */

#ifdef CONFIG_LPC17_40_UART3
static struct up_dev_s g_uart3priv =
{
  .uartbase       = LPC17_40_UART3_BASE,
  .baud           = CONFIG_UART3_BAUD,
  .irq            = LPC17_40_IRQ_UART3,
  .parity         = CONFIG_UART3_PARITY,
  .bits           = CONFIG_UART3_BITS,
  .stopbits2      = CONFIG_UART3_2STOP,
};

static uart_dev_t g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_UART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart3priv,
};
#endif

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port     /* UART0=console */
#    define TTYS0_DEV       g_uart0port     /* UART0=ttyS0 */
#    ifdef CONFIG_LPC17_40_UART1
#      define TTYS1_DEV     g_uart1port     /* UART0=ttyS0;UART1=ttyS1 */
#      ifdef CONFIG_LPC17_40_UART2
#        define TTYS2_DEV   g_uart2port     /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2 */
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS3_DEV g_uart3port     /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#        else
#          undef TTYS3_DEV                  /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS2_DEV g_uart3port     /* UART0=ttyS0;UART1=ttyS1;UART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* UART0=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC17_40_UART2
#        define TTYS1_DEV   g_uart2port     /* UART0=ttyS0;UART2=ttyS1;No ttyS3 */
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS2_DEV g_uart3port     /* UART0=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* UART0=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      else
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS1_DEV g_uart3port     /* UART0=ttyS0;UART3=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                  /* UART0=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#          undef TTYS2_DEV                  /* No ttyS2 */
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port     /* UART1=console */
#    define TTYS0_DEV       g_uart1port     /* UART1=ttyS0 */
#    ifdef CONFIG_LPC17_40_UART0
#      define TTYS1_DEV     g_uart0port     /* UART1=ttyS0;UART0=ttyS1 */
#      ifdef CONFIG_LPC17_40_UART2
#        define TTYS2_DEV   g_uart2port     /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2 */
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS3_DEV g_uart3port     /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#        else
#          undef TTYS3_DEV                  /* UART1=ttyS0;UART0=ttyS1;UART2=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS2_DEV g_uart3port     /* UART1=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* UART1=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC17_40_UART2
#        define TTYS1_DEV   g_uart2port     /* UART1=ttyS0;UART2=ttyS1 */
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS2_DEV g_uart3port     /* UART1=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* UART1=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      else
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS1_DEV   g_uart3port   /* UART1=ttyS0;UART3=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                  /* UART1=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                    /* No ttyS2 */
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2port     /* UART2=console */
#    define TTYS0_DEV       g_uart2port     /* UART2=ttyS0 */
#    ifdef CONFIG_LPC17_40_UART2
#      define TTYS1_DEV     g_uart0port     /* UART2=ttyS0;UART0=ttyS1 */
#      ifdef CONFIG_LPC17_40_UART1
#        define TTYS2_DEV   g_uart1port     /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS3_DEV g_uart3port     /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS2;UART3=ttyS3 */
#        else
#          undef TTYS3_DEV                  /* UART2=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS2_DEV g_uart3port     /* UART2=ttyS0;UART0=ttyS1;UART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* UART2=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC17_40_UART1
#        define TTYS1_DEV   g_uart1port     /* UART2=ttyS0;UART1=ttyS1 */
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS2_DEV g_uart3port     /* UART2=ttyS0;UART1=ttyS1;UART3=ttyS2 */
#        else
#          undef TTYS2_DEV                  /* UART2=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      else
#        ifdef CONFIG_LPC17_40_UART3
#          define TTYS1_DEV g_uart3port     /* UART2=ttyS0;UART3=ttyS1;No ttyS3 */
#        else
#          undef TTYS1_DEV                  /* UART2=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                    /* No ttyS2 */
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart3port     /* UART3=console */
#    define TTYS0_DEV       g_uart3port     /* UART3=ttyS0 */
#    ifdef CONFIG_LPC17_40_UART0
#      define TTYS1_DEV     g_uart0port     /* UART3=ttyS0;UART0=ttyS1 */
#      ifdef CONFIG_LPC17_40_UART1
#        define TTYS2_DEV   g_uart1port     /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2 */
#        ifdef CONFIG_LPC17_40_UART2
#          define TTYS3_DEV g_uart2port     /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS2;UART2=ttyS3 */
#        else
#          undef TTYS3_DEV                  /* UART3=ttyS0;UART0=ttyS1;UART1=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_LPC17_40_UART2
#          define TTYS2_DEV g_uart2port     /* UART3=ttyS0;UART0=ttyS1;UART2=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* UART3=ttyS0;UART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#          undef TTYS3_DEV                  /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC17_40_UART1
#        define TTYS1_DEV   g_uart1port     /* UART3=ttyS0;UART1=ttyS1 */
#        ifdef CONFIG_LPC17_40_UART2
#          define TTYS2_DEV g_uart2port     /* UART3=ttyS0;UART1=ttyS1;UART2=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* UART3=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      else
#        ifdef CONFIG_LPC17_40_UART2
#          define TTYS1_DEV   g_uart2port   /* UART3=ttyS0;UART2=ttyS1;No ttyS3;No ttyS3 */
#          undef TTYS3_DEV                  /* UART3=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                  /* UART3=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                    /* No ttyS2 */
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    endif
#  endif
#else /* No console */
#  define TTYS0_DEV       g_uart0port       /* UART0=ttyS0 */
#  ifdef CONFIG_LPC17_40_UART1
#    define TTYS1_DEV     g_uart1port       /* UART0=ttyS0;UART1=ttyS1 */
#    ifdef CONFIG_LPC17_40_UART2
#      define TTYS2_DEV   g_uart2port       /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2 */
#      ifdef CONFIG_LPC17_40_UART3
#        define TTYS3_DEV g_uart3port       /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS2;UART3=ttyS3 */
#      else
#        undef TTYS3_DEV                    /* UART0=ttyS0;UART1=ttyS1;UART2=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC17_40_UART3
#        define TTYS2_DEV g_uart3port       /* UART0=ttyS0;UART1=ttyS1;UART3=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                    /* UART0=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                      /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_LPC17_40_UART2
#      define TTYS1_DEV   g_uart2port       /* UART0=ttyS0;UART2=ttyS1;No ttyS3 */
#      ifdef CONFIG_LPC17_40_UART3
#        define TTYS2_DEV g_uart3port       /* UART0=ttyS0;UART2=ttyS1;UART3=ttyS2;No ttyS3 */
#      else
#        undef TTYS2_DEV                    /* UART0=ttyS0;UART2=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                      /* No ttyS3 */
#    else
#      ifdef CONFIG_LPC17_40_UART3
#        define TTYS1_DEV g_uart3port       /* UART0=ttyS0;UART3=ttyS1;No ttyS2;No ttyS3 */
#      else
#        undef TTYS1_DEV                    /* UART0=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#      endif
#        undef TTYS2_DEV                    /* No ttyS2 */
#      undef TTYS3_DEV                      /* No ttyS3 */
#    endif
#  endif
#endif /* HAVE_CONSOLE */

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
  up_serialout(priv, LPC17_40_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

#ifdef HAVE_CONSOLE
static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
  up_serialout(priv, LPC17_40_UART_IER_OFFSET, priv->ier);
}
#endif

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
  uint32_t lcr = up_serialin(priv, LPC17_40_UART_LCR_OFFSET);

  if (enable)
    {
      lcr |= UART_LCR_BRK;
    }
  else
    {
      lcr &= ~UART_LCR_BRK;
    }

  up_serialout(priv, LPC17_40_UART_LCR_OFFSET, lcr);
}

#ifdef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
/****************************************************************************
 * Name: lpc17_40_setbaud
 *
 * Description:
 *   Configure the UART divisors to accomplish the desired BAUD given the
 *   UART base frequency.
 *
 *   This computationally intensive algorithm is based on the same logic
 *   used in the NXP sample code.
 *
 ****************************************************************************/

void up_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud)
{
  uint32_t lcr;      /* Line control register value */
  uint32_t dl;       /* Best DLM/DLL full value */
  uint32_t mul;      /* Best FDR MULVALL value */
  uint32_t divadd;   /* Best FDR DIVADDVAL value */
  uint32_t best;     /* Error value associated with best {dl, mul, divadd} */
  uint32_t cdl;      /* Candidate DLM/DLL full value */
  uint32_t cmul;     /* Candidate FDR MULVALL value */
  uint32_t cdivadd;  /* Candidate FDR DIVADDVAL value */
  uint32_t errval;   /* Error value associated with the candidate */

  /* The UART baud is given by:
   *
   * Fbaud =  Fbase * mul / (mul + divadd) / (16 * dl)
   * dl    =  Fbase * mul / (mul + divadd) / Fbaud / 16
   *       =  Fbase * mul / ((mul + divadd) * Fbaud * 16)
   *       = ((Fbase * mul) >> 4) / ((mul + divadd) * Fbaud)
   *
   * Where the  value of MULVAL and DIVADDVAL comply with:
   *
   *  0 < mul < 16
   *  0 <= divadd < mul
   */

  best   = UINT32_MAX;
  divadd = 0;
  mul    = 0;
  dl     = 0;

  /* Try each multiplier value in the valid range */

  for (cmul = 1 ; cmul < 16; cmul++)
    {
      /* Try each divider value in the valid range */

      for (cdivadd = 0 ; cdivadd < cmul ; cdivadd++)
        {
          /* Candidate:
           *   dl         = ((Fbase * mul) >> 4) / ((mul + cdivadd) * Fbaud)
           *   (dl << 32) = (Fbase << 28) * cmul / ((mul + cdivadd) * Fbaud)
           */

          uint64_t dl64 = ((uint64_t)basefreq << 28) * cmul /
                          ((cmul + cdivadd) * baud);

          /* The lower 32-bits of this value is the error */

          errval = (uint32_t)(dl64 & 0x00000000ffffffffull);

          /* The upper 32-bits is the candidate DL value */

          cdl = (uint32_t)(dl64 >> 32);

          /* Round up */

          if (errval > (1 << 31))
            {
              errval = -errval;
              cdl++;
            }

          /* Check if the resulting candidate DL value is within range */

          if (cdl < 1 || cdl > 65536)
            {
              /* No... try a different divadd value */

              continue;
            }

          /* Is this the best combination that we have seen so far? */

          if (errval < best)
            {
              /* Yes.. then the candidate is out best guess so far */

              best   = errval;
              dl     = cdl;
              divadd = cdivadd;
              mul    = cmul;

              /* If the new best guess is exact (within our precision), then
               * we are finished.
               */

              if (best == 0)
                {
                  break;
                }
            }
        }
    }

  DEBUGASSERT(dl > 0);

  /* Enter DLAB=1 */

  lcr = getreg32(uartbase + LPC17_40_UART_LCR_OFFSET);
  putreg32(lcr | UART_LCR_DLAB, uartbase + LPC17_40_UART_LCR_OFFSET);

  /* Save the divider values */

  putreg32(dl >> 8, uartbase + LPC17_40_UART_DLM_OFFSET);
  putreg32(dl & 0xff, uartbase + LPC17_40_UART_DLL_OFFSET);

  /* Clear DLAB */

  putreg32(lcr & ~UART_LCR_DLAB, uartbase + LPC17_40_UART_LCR_OFFSET);

  /* Then save the fractional divider values */

  putreg32((mul << UART_FDR_MULVAL_SHIFT) | \
           (divadd << UART_FDR_DIVADDVAL_SHIFT),
           uartbase + LPC17_40_UART_FDR_OFFSET);
}
#  ifdef LPC176x
static inline uint32_t lpc17_40_uartcclkdiv(uint32_t baud)
{
  /* If we're using the fractional divider, assume that the full PCLK speed
   * will be acceptable.
   */

  return SYSCON_PCLKSEL_CCLK;
}
#  endif
#else

/****************************************************************************
 * Name: lpc17_40_uartcclkdiv
 *
 * Description:
 *   Select a CCLK divider to produce the UART PCLK.  The strategy is to
 *   select the smallest divisor that results in an solution within range of
 *   the 16-bit DLM and DLL divisor:
 *
 *     PCLK = CCLK / divisor
 *     BAUD = PCLK / (16 * DL)
 *
 *
 *   For the LPC176x the PCLK is determined by the UART-specific divisor in
 *   PCLKSEL0 or PCLKSEL1:
 *
 *     PCLK = CCLK / divisor
 *
 *   For the LPC178x/40xx, the PCLK is determined by the global divisor
 *   setting in the PLKSEL register (and, in that case, this function is not
 *   needed).
 *
 *   NOTE:  This is an inline function.  If a typical optimization level is
 *   used and a constant is provided for the desired frequency, then most of
 *   the following logic will be optimized away.
 *
 ****************************************************************************/

#  ifdef LPC176x
static inline uint32_t lpc17_40_uartcclkdiv(uint32_t baud)
{
  /* Ignoring the fractional divider, the BAUD is given by:
   *
   *   BAUD = PCLK / (16 * DL), or
   *   DL   = PCLK / BAUD / 16
   *
   * Where for the LPC176x the PCLK is determined by the UART-specific
   * divisor in PCLKSEL0 or PCLKSEL1:
   *
   *   PCLK = CCLK / divisor
   *
   * And for the LPC178x/40xx, the PCLK is determined by the global divisor
   * setting in the PLKSEL register (and, in that case, this function is not
   * needed).
   */

  /* Calculate and optimal PCLKSEL0/1 divisor.
   * First, check divisor == 1.  This works if the upper limit is met:
   *
   *   DL < 0xffff, or
   *   PCLK / BAUD / 16 < 0xffff, or
   *   CCLK / BAUD / 16 < 0xffff, or
   *   CCLK < BAUD * 0xffff * 16
   *   BAUD > CCLK / 0xffff / 16
   *
   * And the lower limit is met (we can't allow DL to get very close to one).
   *
   *   DL >= MinDL
   *   CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 16 / MinDL
   */

  if (baud < (LPC17_40_CCLK / 16 / UART_MINDL))
    {
      return SYSCON_PCLKSEL_CCLK;
    }

  /* Check divisor == 2.  This works if:
   *
   *   2 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 8
   *
   * And
   *
   *   2 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 8 / MinDL
   */

  else if (baud < (LPC17_40_CCLK / 8 / UART_MINDL))
    {
      return SYSCON_PCLKSEL_CCLK2;
    }

  /* Check divisor == 4.  This works if:
   *
   *   4 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 4
   *
   * And
   *
   *   4 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 4 / MinDL
   */

  else if (baud < (LPC17_40_CCLK / 4 / UART_MINDL))
    {
      return SYSCON_PCLKSEL_CCLK4;
    }

  /* Check divisor == 8.  This works if:
   *
   *   8 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 2
   *
   * And
   *
   *   8 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 2 / MinDL
   */

  else /* if (baud < (LPC17_40_CCLK / 2 / UART_MINDL)) */
    {
      return SYSCON_PCLKSEL_CCLK8;
    }
}
#  endif /* LPC176x */
#endif /* CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER */

/****************************************************************************
 * Name: lpc17_40_uart0config, uart1config, uart2config, and uart3config
 *
 * Description:
 *   Configure the UART.  UART0/1/2/3 peripherals are configured using the
 *   following registers:
 *
 *   1. Power: In the PCONP register, set bits PCUART0/1/2/3.
 *      On reset, UART0 and UART 1 are enabled (PCUART0 = 1 and PCUART1 = 1)
 *      and UART2/3 are disabled (PCUART1 = 0 and PCUART3 = 0).
 *   2. Peripheral clock: In the PCLKSEL0 register, select PCLK_UART0 and
 *      PCLK_UART1; in the PCLKSEL1 register, select PCLK_UART2 and
 *      PCLK_UART3.
 *   3. Pins: Select UART pins through the PINSEL registers and pin modes
 *      through the PINMODE registers. UART receive pins should not have
 *      pull-down resistors enabled.
 *
 ****************************************************************************/

#if defined(CONFIG_LPC17_40_UART0) && !defined(CONFIG_UART0_SERIAL_CONSOLE)
static inline void lpc17_40_uart0config(void)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Step 1: Enable power on UART0 */

  flags   = enter_critical_section();
  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUART0;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Step 2: Enable clocking on UART */

#ifdef LPC176x
  regval = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_UART0_MASK;
  regval |= ((uint32_t)g_uart0priv.cclkdiv << SYSCON_PCLKSEL0_UART0_SHIFT);
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);
#endif

  /* Step 3: Configure I/O pins */

  lpc17_40_configgpio(GPIO_UART0_TXD);
  lpc17_40_configgpio(GPIO_UART0_RXD);
  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_LPC17_40_UART1
static inline void lpc17_40_uart1config(void)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Step 1: Enable power on UART1 */

  flags   = enter_critical_section();
  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUART1;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Step 2: Enable clocking on UART */

#ifdef LPC176x
  regval = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_UART1_MASK;
  regval |= ((uint32_t)g_uart1priv.cclkdiv << SYSCON_PCLKSEL0_UART1_SHIFT);
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);
#endif

  /* Step 3: Configure RS-485 control register */

#ifdef CONFIG_LPC17_40_UART1_RS485
  regval  = getreg32(LPC17_40_UART1_RS485CTRL);
  regval |= UART_RS485CTRL_DCTRL;
#if (CONFIG_LPC17_40_RS485_DIR_POLARITY == 1)
  regval |= UART_RS485CTRL_OINV;
#endif
#ifdef CONFIG_LPC17_40_UART1_RS485_DIR_DTR
  regval |= UART_RS485CTRL_SEL;
#endif
  putreg32(regval, LPC17_40_UART1_RS485CTRL);
#endif

  /* Step 4: Configure I/O pins */

  lpc17_40_configgpio(GPIO_UART1_TXD);
  lpc17_40_configgpio(GPIO_UART1_RXD);
#if defined(CONFIG_UART1_IFLOWCONTROL) || defined(CONFIG_UART1_OFLOWCONTROL)
  lpc17_40_configgpio(GPIO_UART1_CTS);
  lpc17_40_configgpio(GPIO_UART1_RTS);
  lpc17_40_configgpio(GPIO_UART1_DCD);
  lpc17_40_configgpio(GPIO_UART1_DSR);
  lpc17_40_configgpio(GPIO_UART1_DTR);
#ifdef CONFIG_LPC17_40_UART1_RINGINDICATOR
  lpc17_40_configgpio(GPIO_UART1_RI);
#endif
#endif

#ifdef CONFIG_LPC17_40_UART1_RS485
  lpc17_40_configgpio(GPIO_UART1_RS485_DIR);
#endif

  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_LPC17_40_UART2
static inline void lpc17_40_uart2config(void)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Step 1: Enable power on UART2 */

  flags   = enter_critical_section();
  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUART2;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Step 2: Enable clocking on UART */

#ifdef LPC176x
  regval = getreg32(LPC17_40_SYSCON_PCLKSEL1);
  regval &= ~SYSCON_PCLKSEL1_UART2_MASK;
  regval |= ((uint32_t)g_uart2priv.cclkdiv << SYSCON_PCLKSEL1_UART2_SHIFT);
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL1);
#endif

  /* Step 3: Configure I/O pins */

  lpc17_40_configgpio(GPIO_UART2_TXD);
  lpc17_40_configgpio(GPIO_UART2_RXD);
  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_LPC17_40_UART3
static inline void lpc17_40_uart3config(void)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Step 1: Enable power on UART3 */

  flags   = enter_critical_section();
  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUART3;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Step 2: Enable clocking on UART */

#ifdef LPC176x
  regval = getreg32(LPC17_40_SYSCON_PCLKSEL1);
  regval &= ~SYSCON_PCLKSEL1_UART3_MASK;
  regval |= ((uint32_t)g_uart3priv.cclkdiv << SYSCON_PCLKSEL1_UART3_SHIFT);
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL1);
#endif

  /* Step 3: Configure I/O pins */

  lpc17_40_configgpio(GPIO_UART3_TXD);
  lpc17_40_configgpio(GPIO_UART3_RXD);
  leave_critical_section(flags);
};
#endif

/****************************************************************************
 * Name: lpc17_40_uartdl
 *
 * Description:
 *   Select a divider to produce the BAUD from the UART PCLK.
 *
 *     BAUD = PCLK / (16 * DL), or
 *     DL   = PCLK / BAUD / 16
 *
 ****************************************************************************/

#ifndef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
#  ifdef LPC176x
static inline uint32_t lpc17_40_uartdl(uint32_t baud, uint8_t divcode)
{
  uint32_t num;

  switch (divcode)
    {
    case SYSCON_PCLKSEL_CCLK4:   /* PCLK_peripheral = CCLK/4 */
      num = (LPC17_40_CCLK / 4);
      break;

    case SYSCON_PCLKSEL_CCLK:    /* PCLK_peripheral = CCLK */
      num = LPC17_40_CCLK;
      break;

    case SYSCON_PCLKSEL_CCLK2:   /* PCLK_peripheral = CCLK/2 */
      num = (LPC17_40_CCLK / 2);
      break;

    case SYSCON_PCLKSEL_CCLK8:   /* PCLK_peripheral = CCLK/8 (except CAN1, CAN2, and CAN) */
    default:
      num = (LPC17_40_CCLK / 8);
      break;
    }

  return num / (baud << 4);
}
#else
static inline uint32_t lpc17_40_uartdl(uint32_t baud)
{
  return (uint32_t)BOARD_PCLK_FREQUENCY / (baud << 4);
}
#  endif
#endif

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
#  ifndef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
  uint16_t dl;
#  endif
  uint32_t lcr;

  /* Clear fifos */

  up_serialout(priv, LPC17_40_UART_FCR_OFFSET,
               (UART_FCR_RXRST | UART_FCR_TXRST));

  /* Set trigger */

  up_serialout(priv, LPC17_40_UART_FCR_OFFSET,
               (UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_8));

  /* Set up the IER */

  priv->ier = up_serialin(priv, LPC17_40_UART_IER_OFFSET);

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

#ifndef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
  /* Disable FDR (fractional divider),
   * ignored by baudrate calculation => has to be disabled
   */

  up_serialout(priv, LPC17_40_UART_FDR_OFFSET,
               (1 << UART_FDR_MULVAL_SHIFT) + \
               (0 << UART_FDR_DIVADDVAL_SHIFT));
#endif

  /* Enter DLAB=1 */

  up_serialout(priv, LPC17_40_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

  /* Set the BAUD divisor */

#ifdef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
  up_setbaud(priv->uartbase, LPC17_40_CCLK / priv->cclkdiv, priv->baud);
#else
#  ifdef LPC176x
  dl = lpc17_40_uartdl(priv->baud, priv->cclkdiv);
#  else
  dl = lpc17_40_uartdl(priv->baud);
#  endif
  up_serialout(priv, LPC17_40_UART_DLM_OFFSET, dl >> 8);
  up_serialout(priv, LPC17_40_UART_DLL_OFFSET, dl & 0xff);
#endif

  /* Clear DLAB */

  up_serialout(priv, LPC17_40_UART_LCR_OFFSET, lcr);

  /* Configure the FIFOs */

  up_serialout(priv, LPC17_40_UART_FCR_OFFSET,
               (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST | UART_FCR_RXRST |
                UART_FCR_FIFOEN));

  /* Enable Auto-RTS and Auto-CS Flow Control in the Modem Control Register */

#if defined(CONFIG_UART1_IFLOWCONTROL) || defined(CONFIG_UART1_OFLOWCONTROL)
  if (priv->uartbase == LPC17_40_UART1_BASE)
    {
#if defined(CONFIG_UART1_IFLOWCONTROL) && defined(CONFIG_UART1_OFLOWCONTROL)
      up_serialout(priv, LPC17_40_UART_MCR_OFFSET,
                   (UART_MCR_RTSEN | UART_MCR_CTSEN));
#elif defined(CONFIG_UART1_IFLOWCONTROL)
      up_serialout(priv, LPC17_40_UART_MCR_OFFSET, UART_MCR_RTSEN);
#else
      up_serialout(priv, LPC17_40_UART_MCR_OFFSET, UART_MCR_CTSEN);
#endif
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
  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
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
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
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

       status = up_serialin(priv, LPC17_40_UART_IIR_OFFSET);

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

              status = up_serialin(priv, LPC17_40_UART_MSR_OFFSET);
              _info("MSR: %02" PRIx32 "\n", status);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_INTID_RLS:
            {
              /* Read the line status register (LSR) to clear */

              status = up_serialin(priv, LPC17_40_UART_LSR_OFFSET);
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
         * Note that cfsetospeed is not necessary because we have
         * knowledge that only one speed is supported.
         * Both cfset(i|o)speed() translate to cfsetspeed.
         */

        cfsetispeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;
#  ifndef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
        uint32_t           lcr;  /* Holds current values of line control register */
        uint16_t           dl;   /* Divisor latch */
#  endif

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

        /* TODO: Re-calculate the optimal CCLK divisor for the new baud and
         * and reset the divider in the CLKSEL0/1 register.
         */

#  ifdef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
        up_setbaud(priv->uartbase, LPC17_40_CCLK / \
                   priv->cclkdiv, priv->baud);
#  else
#    if 0 /* ifdef LPC176x */
        priv->cclkdiv = lpc17_40_uartcclkdiv(priv->baud);
#    endif
        /* DLAB open latch
         * REVISIT: Shouldn't we just call up_setup() to do all of the
         *          following?
         */

        lcr = getreg32(priv->uartbase + LPC17_40_UART_LCR_OFFSET);
        up_serialout(priv, LPC17_40_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

        /* Set the BAUD divisor */

#    ifdef LPC176x
        dl = lpc17_40_uartdl(priv->baud, priv->cclkdiv);
#    else
        dl = lpc17_40_uartdl(priv->baud);
#    endif
        up_serialout(priv, LPC17_40_UART_DLM_OFFSET, dl >> 8);
        up_serialout(priv, LPC17_40_UART_DLL_OFFSET, dl & 0xff);

        /* Clear DLAB */

        up_serialout(priv, LPC17_40_UART_LCR_OFFSET, lcr);
#  endif
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

  *status = up_serialin(priv, LPC17_40_UART_LSR_OFFSET);
  rbr     = up_serialin(priv, LPC17_40_UART_RBR_OFFSET);
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

  up_serialout(priv, LPC17_40_UART_IER_OFFSET, priv->ier);
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
  return ((up_serialin(priv, LPC17_40_UART_LSR_OFFSET) & UART_LSR_RDR) != 0);
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
  up_serialout(priv, LPC17_40_UART_THR_OFFSET, (uint32_t)ch);
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
      up_serialout(priv, LPC17_40_UART_IER_OFFSET, priv->ier);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_THREIE;
      up_serialout(priv, LPC17_40_UART_IER_OFFSET, priv->ier);
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
  return ((up_serialin(priv, LPC17_40_UART_LSR_OFFSET) & \
          UART_LSR_THRE) != 0);
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
  return ((up_serialin(priv, LPC17_40_UART_LSR_OFFSET) & \
          UART_LSR_THRE) != 0);
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

#ifdef CONFIG_LPC17_40_UART0
#ifdef LPC176x
  g_uart0priv.cclkdiv = lpc17_40_uartcclkdiv(CONFIG_UART0_BAUD);
#endif
#ifndef CONFIG_UART0_SERIAL_CONSOLE
  lpc17_40_uart0config();
#endif
  up_disableuartint(&g_uart0priv, NULL);
#endif

#ifdef CONFIG_LPC17_40_UART1
#ifdef LPC176x
  g_uart1priv.cclkdiv = lpc17_40_uartcclkdiv(CONFIG_UART1_BAUD);
#endif
#ifndef CONFIG_UART1_SERIAL_CONSOLE
  lpc17_40_uart1config();
#else
#endif
  up_disableuartint(&g_uart1priv, NULL);
#endif

#ifdef CONFIG_LPC17_40_UART2
#ifdef LPC176x
  g_uart2priv.cclkdiv = lpc17_40_uartcclkdiv(CONFIG_UART2_BAUD);
#endif
#ifndef CONFIG_UART2_SERIAL_CONSOLE
  lpc17_40_uart2config();
#endif
  up_disableuartint(&g_uart2priv, NULL);
#endif

#ifdef CONFIG_LPC17_40_UART3
#ifdef LPC176x
  g_uart3priv.cclkdiv = lpc17_40_uartcclkdiv(CONFIG_UART3_BAUD);
#endif
#ifndef CONFIG_UART3_SERIAL_CONSOLE
  lpc17_40_uart3config();
#endif
  up_disableuartint(&g_uart3priv, NULL);
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
#ifdef HAVE_CONSOLE
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
#ifdef HAVE_CONSOLE
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
