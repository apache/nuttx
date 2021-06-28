/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_serial.c
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
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include "rx65n_macrodriver.h"
#include "arch/rx65n/iodefine.h"
#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "rx65n_definitions.h"
#include "rx65n_sci.h"
#include "arch/rx65n/irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

#ifdef USE_SERIALDRIVER

/* Which SCI with be tty0/console and which tty1? */

#ifdef CONFIG_RX65N_SCI0
#  define TTYS0_DEV     g_sci0port       /* SCI0 is tty0 */
#else
#  undef  TTYS0_DEV                      /* No tty0 */
#endif
#ifdef CONFIG_RX65N_SCI1
#  define TTYS1_DEV     g_sci1port       /* SCI1 is tty1 */
#else
#  undef  TTYS1_DEV                      /* No tty1 */
#endif
#ifdef CONFIG_RX65N_SCI2
#  define TTYS2_DEV     g_sci2port       /* SCI2 is tty2 */
#else
#  undef  TTYS2_DEV                      /* No tty2 */
#endif
#ifdef CONFIG_RX65N_SCI3
#  define TTYS3_DEV     g_sci3port       /* SCI3 is tty3 */
#else
#  undef  TTYS3_DEV                      /* No tty3 */
#endif
#ifdef CONFIG_RX65N_SCI4
#  define TTYS4_DEV     g_sci4port       /* SCI4 is tty4 */
#else
#  undef  TTYS4_DEV                      /* No tty4 */
#endif
#ifdef CONFIG_RX65N_SCI5
#  define TTYS5_DEV     g_sci5port       /* SCI5 is tty5 */
#else
#  undef  TTYS5_DEV                      /* No tty5 */
#endif
#ifdef CONFIG_RX65N_SCI6
#  define TTYS6_DEV     g_sci6port       /* SCI6 is tty6 */
#else
#  undef  TTYS6_DEV                      /* No tty6 */
#endif
#ifdef CONFIG_RX65N_SCI7
#  define TTYS7_DEV     g_sci7port       /* SCI7 is tty7 */
#else
#  undef  TTYS7_DEV                      /* No tty7 */
#endif
#ifdef CONFIG_RX65N_SCI8
#  define TTYS8_DEV     g_sci8port       /* SCI8 is tty8 */
#else
#  undef  TTYS8_DEV                      /* No tty8 */
#endif
#ifdef CONFIG_RX65N_SCI9
#  define TTYS9_DEV     g_sci9port       /* SCI9 is tty9 */
#else
#  undef  TTYS9_DEV                      /* No tty9 */
#endif
#ifdef CONFIG_RX65N_SCI10
#  define TTYS10_DEV    g_sci10port      /* SCI10 is tty10 */
#else
#  undef  TTYS10_DEV                     /* No tty10 */
#endif
#ifdef CONFIG_RX65N_SCI11
#  define TTYS11_DEV    g_sci11port      /* SCI11 is tty11 */
#else
#  undef  TTYS11_DEV                     /* No tty11 */
#endif
#ifdef CONFIG_RX65N_SCI12
#  define TTYS12_DEV    g_sci12port      /* SCI12 is tty12 */
#else
#  undef  TTYS12_DEV                     /* No tty12 */
#endif

#if   defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI0)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci0port /* SCI0 is console */
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI1)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci1port /* SCI1 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS1_DEV
#    define     TTYS1_DEV               g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV

#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI2)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci2port /* SCI2 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS2_DEV
#    define     TTYS2_DEV               g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI3)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci3port /* SCI3 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS3_DEV
#    define     TTYS3_DEV               g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI4)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci4port /* SCI4 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS4_DEV
#    define     TTYS4_DEV               g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI5)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci5port /* SCI5 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS5_DEV
#    define     TTYS5_DEV               g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI6)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci6port /* SCI6 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS6_DEV
#    define     TTYS6_DEV               g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI7)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci7port /* SCI7 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS7_DEV
#    define     TTYS7_DEV               g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI8)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci8port /* SCI8 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS8_DEV
#    define     TTYS8_DEV               g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI9)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci9port /* SCI9 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS9_DEV
#    define     TTYS9_DEV               g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI10_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI10)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci10port /* SCI10 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS10_DEV
#    define     TTYS10_DEV              g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI11_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI11)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci11port /* SCI11 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS11_DEV
#    define     TTYS11_DEV              g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#elif defined(CONFIG_SCI12_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI12)
#  define       HAVE_CONSOLE
#  define       CONSOLE_DEV             g_sci12port   /* SCI12 is console */
#  ifdef        TTYS0_DEV
#    undef      TTYS12_DEV
#    define     TTYS12_DEV              g_sci0port
#    undef      TTYS0_DEV
#  endif /* TTYS0_DEV */

#  define       TTYS0_DEV               CONSOLE_DEV
#else
#  undef        HAVE_CONSOLE
#  undef        CONSOLE_DEV     /* No console */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t scibase;      /* Base address of SCI registers */
  uint32_t baud;         /* Configured baud */
  volatile  uint8_t scr; /* Saved SCR value */
  volatile  uint8_t ssr; /* Saved SR value */
  uint8_t xmitirq;       /* Base IRQ associated with xmit IRQ */
  uint8_t recvirq;       /* Base IRQ associated with receive IRQ */
  uint8_t eriirq;
  uint8_t teiirq;
  uint32_t grpibase;
  uint32_t erimask;
  uint32_t teimask;
  uint8_t parity; /* 0=none, 1=odd, 2=even */
  uint8_t bits;   /* Number of bits (7 or 8) */
  bool stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_xmtinterrupt(int irq, void *context, FAR void *arg);
static int  up_rcvinterrupt(int irq, void *context, FAR void *arg);
static int  up_eriinterrupt(int irq, void *context, FAR void *arg);
static int  up_teiinterrupt(int irq, void *context, FAR void *arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_enable_irq(int irq);
void up_disable_irq(int irq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I/O buffers */

#ifdef CONFIG_RX65N_SCI0
  static char g_sci0rxbuffer[CONFIG_SCI0_RXBUFSIZE];
  static char g_sci0txbuffer[CONFIG_SCI0_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI1
  static char g_sci1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
  static char g_sci1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI2
  static char g_sci2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
  static char g_sci2txbuffer[CONFIG_SCI2_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI3
  static char g_sci3rxbuffer[CONFIG_SCI3_RXBUFSIZE];
  static char g_sci3txbuffer[CONFIG_SCI3_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI4
  static char g_sci4rxbuffer[CONFIG_SCI4_RXBUFSIZE];
  static char g_sci4txbuffer[CONFIG_SCI4_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI5
  static char g_sci5rxbuffer[CONFIG_SCI5_RXBUFSIZE];
  static char g_sci5txbuffer[CONFIG_SCI5_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI6
  static char g_sci6rxbuffer[CONFIG_SCI6_RXBUFSIZE];
  static char g_sci6txbuffer[CONFIG_SCI6_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI7
  static char g_sci7rxbuffer[CONFIG_SCI7_RXBUFSIZE];
  static char g_sci7txbuffer[CONFIG_SCI7_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI8
  static char g_sci8rxbuffer[CONFIG_SCI8_RXBUFSIZE];
  static char g_sci8txbuffer[CONFIG_SCI8_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI9
  static char g_sci9rxbuffer[CONFIG_SCI9_RXBUFSIZE];
  static char g_sci9txbuffer[CONFIG_SCI9_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI10
  static char g_sci10rxbuffer[CONFIG_SCI10_RXBUFSIZE];
  static char g_sci10txbuffer[CONFIG_SCI10_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI11
  static char g_sci11rxbuffer[CONFIG_SCI11_RXBUFSIZE];
  static char g_sci11txbuffer[CONFIG_SCI11_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI12
  static char g_sci12rxbuffer[CONFIG_SCI12_RXBUFSIZE];
  static char g_sci12txbuffer[CONFIG_SCI12_TXBUFSIZE];
#endif

struct uart_ops_s g_sci_ops =
{
  .setup       =  up_setup,
  .shutdown    =  up_shutdown,
  .attach      =  up_attach,
  .detach      =  up_detach,
  .receive     =  up_receive,
  .ioctl       =  up_ioctl,
  .rxint       =  up_rxint,
  .rxavailable =  up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send        =  up_send,
  .txint       =  up_txint,
  .txready     =  up_txready,
  .txempty     =  up_txready,
};

#ifdef CONFIG_RX65N_SCI0
static struct up_dev_s g_sci0priv =
{
  .scibase     =  RX65N_SCI0_BASE,
  .baud        =  CONFIG_SCI0_BAUD,
  .recvirq     =  RX65N_RXI0_IRQ,
  .xmitirq     =  RX65N_TXI0_IRQ,
  .eriirq      =  RX65N_ERI0_IRQ,
  .teiirq      =  RX65N_TEI0_IRQ,
  .grpibase    =  RX65N_GRPBL0_ADDR,
  .erimask     =  RX65N_GRPBL0_ERI0_MASK,
  .teimask     =  RX65N_GRPBL0_TEI0_MASK,
  .parity      =  CONFIG_SCI0_PARITY,
  .bits        =  CONFIG_SCI0_BITS,
  .stopbits2   =  CONFIG_SCI0_2STOP,
};

static uart_dev_t g_sci0port =
{
  .recv     =
    {
      .size   = CONFIG_SCI0_RXBUFSIZE,
      .buffer = g_sci0rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI0_TXBUFSIZE,
      .buffer = g_sci0txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci0priv,
};
#endif

#ifdef CONFIG_RX65N_SCI1
static struct up_dev_s g_sci1priv =
{
  .scibase   =  RX65N_SCI1_BASE,
  .baud      =  CONFIG_SCI1_BAUD,
  .recvirq   =  RX65N_RXI1_IRQ,
  .xmitirq   =  RX65N_TXI1_IRQ,
  .eriirq    =  RX65N_ERI1_IRQ,
  .teiirq    =  RX65N_TEI1_IRQ,
  .grpibase  =  RX65N_GRPBL0_ADDR,
  .erimask   =  RX65N_GRPBL0_ERI1_MASK,
  .teimask   =  RX65N_GRPBL0_TEI1_MASK,
  .parity    =  CONFIG_SCI1_PARITY,
  .bits      =  CONFIG_SCI1_BITS,
  .stopbits2 =  CONFIG_SCI1_2STOP,
};

static uart_dev_t g_sci1port =
{
  .recv     =
    {
      .size    = CONFIG_SCI1_RXBUFSIZE,
      .buffer  = g_sci1rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI1_TXBUFSIZE,
      .buffer = g_sci1txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci1priv,
};
#endif

#ifdef CONFIG_RX65N_SCI2
static struct up_dev_s g_sci2priv =
{
  .scibase      =  RX65N_SCI2_BASE,
  .baud         =  CONFIG_SCI2_BAUD,
  .recvirq      =  RX65N_RXI2_IRQ,
  .xmitirq      =  RX65N_TXI2_IRQ,
  .eriirq       =  RX65N_ERI2_IRQ,
  .teiirq       =  RX65N_TEI2_IRQ,
  .grpibase     =  RX65N_GRPBL0_ADDR,
  .erimask      =  RX65N_GRPBL0_ERI2_MASK,
  .teimask      =  RX65N_GRPBL0_TEI2_MASK,
  .parity       =  CONFIG_SCI2_PARITY,
  .bits         =  CONFIG_SCI2_BITS,
  .stopbits2    =  CONFIG_SCI2_2STOP,
};

static uart_dev_t g_sci2port =
{
  .recv     =
    {
      .size    = CONFIG_SCI2_RXBUFSIZE,
      .buffer  = g_sci2rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI2_TXBUFSIZE,
      .buffer = g_sci2txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci2priv,
};
#endif

#ifdef CONFIG_RX65N_SCI3
static struct up_dev_s g_sci3priv =
{
  .scibase    =  RX65N_SCI3_BASE,
  .baud       =  CONFIG_SCI3_BAUD,
  .recvirq    =  RX65N_RXI3_IRQ,
  .xmitirq    =  RX65N_TXI3_IRQ,
  .eriirq     =  RX65N_ERI3_IRQ,
  .teiirq     =  RX65N_TEI3_IRQ,
  .grpibase   =  RX65N_GRPBL0_ADDR,
  .erimask    =  RX65N_GRPBL0_ERI3_MASK,
  .teimask    =  RX65N_GRPBL0_TEI3_MASK,
  .parity     =  CONFIG_SCI3_PARITY,
  .bits       =  CONFIG_SCI3_BITS,
  .stopbits2  =  CONFIG_SCI3_2STOP,
};

static uart_dev_t g_sci3port =
{
  .recv     =
    {
      .size    = CONFIG_SCI3_RXBUFSIZE,
      .buffer  = g_sci3rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI3_TXBUFSIZE,
      .buffer = g_sci3txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci3priv,
};
#endif

#ifdef CONFIG_RX65N_SCI4
static struct up_dev_s g_sci4priv =
{
  .scibase   =  RX65N_SCI4_BASE,
  .baud      =  CONFIG_SCI4_BAUD,
  .recvirq   =  RX65N_RXI4_IRQ,
  .xmitirq   =  RX65N_TXI4_IRQ,
  .eriirq    =  RX65N_ERI4_IRQ,
  .teiirq    =  RX65N_TEI4_IRQ,
  .grpibase  =  RX65N_GRPBL0_ADDR,
  .erimask   =  RX65N_GRPBL0_ERI4_MASK,
  .teimask   =  RX65N_GRPBL0_TEI4_MASK,
  .parity    =  CONFIG_SCI4_PARITY,
  .bits      =  CONFIG_SCI4_BITS,
  .stopbits2 =  CONFIG_SCI4_2STOP,
};

static uart_dev_t g_sci4port =
{
  .recv     =
    {
      .size    = CONFIG_SCI4_RXBUFSIZE,
      .buffer  = g_sci4rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI4_TXBUFSIZE,
      .buffer = g_sci4txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci4priv,
};
#endif

#ifdef CONFIG_RX65N_SCI5
static struct up_dev_s g_sci5priv =
{
  .scibase    =  RX65N_SCI5_BASE,
  .baud       =  CONFIG_SCI5_BAUD,
  .recvirq    =  RX65N_RXI5_IRQ,
  .xmitirq    =  RX65N_TXI5_IRQ,
  .eriirq     =  RX65N_ERI5_IRQ,
  .teiirq     =  RX65N_TEI5_IRQ,
  .grpibase   =  RX65N_GRPBL0_ADDR,
  .erimask    =  RX65N_GRPBL0_ERI5_MASK,
  .teimask    =  RX65N_GRPBL0_TEI5_MASK,
  .parity     =  CONFIG_SCI5_PARITY,
  .bits       =  CONFIG_SCI5_BITS,
  .stopbits2  =  CONFIG_SCI5_2STOP,
};

static uart_dev_t g_sci5port =
{
  .recv     =
    {
      .size    = CONFIG_SCI5_RXBUFSIZE,
      .buffer  = g_sci5rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI5_TXBUFSIZE,
      .buffer = g_sci5txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci5priv,
};
#endif

#ifdef CONFIG_RX65N_SCI6
static struct up_dev_s g_sci6priv =
{
  .scibase   =  RX65N_SCI6_BASE,
  .baud      =  CONFIG_SCI6_BAUD,
  .recvirq   =  RX65N_RXI6_IRQ,
  .xmitirq   =  RX65N_TXI6_IRQ,
  .eriirq    =  RX65N_ERI6_IRQ,
  .teiirq    =  RX65N_TEI6_IRQ,
  .grpibase  =  RX65N_GRPBL0_ADDR,
  .erimask   =  RX65N_GRPBL0_ERI6_MASK,
  .teimask   =  RX65N_GRPBL0_TEI6_MASK,
  .parity    =  CONFIG_SCI6_PARITY,
  .bits      =  CONFIG_SCI6_BITS,
  .stopbits2 =  CONFIG_SCI6_2STOP,
};

static uart_dev_t g_sci6port =
{
  .recv     =
    {
      .size    = CONFIG_SCI6_RXBUFSIZE,
      .buffer  = g_sci6rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI6_TXBUFSIZE,
      .buffer = g_sci6txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci6priv,
};
#endif

#ifdef CONFIG_RX65N_SCI7
static struct up_dev_s g_sci7priv =
{
  .scibase   =  RX65N_SCI7_BASE,
  .baud      =  CONFIG_SCI7_BAUD,
  .recvirq   =  RX65N_RXI7_IRQ,
  .xmitirq   =  RX65N_TXI7_IRQ,
  .eriirq    =  RX65N_ERI7_IRQ,
  .teiirq    =  RX65N_TEI7_IRQ,
  .grpibase  =  RX65N_GRPBL0_ADDR,
  .erimask   =  RX65N_GRPBL0_ERI7_MASK,
  .teimask   =  RX65N_GRPBL0_TEI7_MASK,
  .parity    =  CONFIG_SCI7_PARITY,
  .bits      =  CONFIG_SCI7_BITS,
  .stopbits2 =  CONFIG_SCI7_2STOP,
};

static uart_dev_t g_sci7port =
{
  .recv     =
    {
      .size    = CONFIG_SCI7_RXBUFSIZE,
      .buffer  = g_sci7rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI7_TXBUFSIZE,
      .buffer = g_sci7txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci7priv,
};
#endif

#ifdef CONFIG_RX65N_SCI8
static struct up_dev_s g_sci8priv =
{
  .scibase    =  RX65N_SCI8_BASE,
  .baud       =  CONFIG_SCI8_BAUD,
  .recvirq    =  RX65N_RXI8_IRQ,
  .xmitirq    =  RX65N_TXI8_IRQ,
  .eriirq     =  RX65N_ERI8_IRQ,
  .teiirq     =  RX65N_TEI8_IRQ,
  .grpibase   =  RX65N_GRPBL1_ADDR,
  .erimask    =  RX65N_GRPBL1_ERI8_MASK,
  .teimask    =  RX65N_GRPBL1_TEI8_MASK,
  .parity     =  CONFIG_SCI8_PARITY,
  .bits       =  CONFIG_SCI8_BITS,
  .stopbits2  =  CONFIG_SCI8_2STOP,
};

static uart_dev_t g_sci8port =
{
  .recv     =
    {
      .size    = CONFIG_SCI8_RXBUFSIZE,
      .buffer  = g_sci8rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI8_TXBUFSIZE,
      .buffer = g_sci8txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci8priv,
};
#endif

#ifdef CONFIG_RX65N_SCI9
static struct up_dev_s g_sci9priv =
{
  .scibase     =  RX65N_SCI9_BASE,
  .baud        =  CONFIG_SCI9_BAUD,
  .recvirq     =  RX65N_RXI9_IRQ,
  .xmitirq     =  RX65N_TXI9_IRQ,
  .eriirq      =  RX65N_ERI9_IRQ,
  .teiirq      =  RX65N_TEI9_IRQ,
  .grpibase    =  RX65N_GRPBL1_ADDR,
  .erimask     =  RX65N_GRPBL1_ERI9_MASK,
  .teimask     =  RX65N_GRPBL1_TEI9_MASK,
  .parity      =  CONFIG_SCI9_PARITY,
  .bits        =  CONFIG_SCI9_BITS,
  .stopbits2   =  CONFIG_SCI9_2STOP,
};

static uart_dev_t g_sci9port =
{
  .recv     =
    {
      .size    = CONFIG_SCI9_RXBUFSIZE,
      .buffer  = g_sci9rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI9_TXBUFSIZE,
      .buffer = g_sci9txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci9priv,
};
#endif

#ifdef CONFIG_RX65N_SCI10
static struct up_dev_s g_sci10priv =
{
  .scibase   =  RX65N_SCI10_BASE,
  .baud      =  CONFIG_SCI10_BAUD,
  .recvirq   =  RX65N_RXI10_IRQ,
  .xmitirq   =  RX65N_TXI10_IRQ,
  .eriirq    =  RX65N_ERI10_IRQ,
  .teiirq    =  RX65N_TEI10_IRQ,
  .grpibase  =  RX65N_GRPAL0_ADDR,
  .erimask   =  RX65N_GRPAL0_ERI10_MASK,
  .teimask   =  RX65N_GRPAL0_TEI10_MASK,
  .parity    =  CONFIG_SCI10_PARITY,
  .bits      =  CONFIG_SCI10_BITS,
  .stopbits2 =  CONFIG_SCI10_2STOP,
};

static uart_dev_t g_sci10port =
{
  .recv     =
    {
      .size    = CONFIG_SCI10_RXBUFSIZE,
      .buffer  = g_sci10rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI10_TXBUFSIZE,
      .buffer = g_sci10txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci10priv,
};
#endif

#ifdef CONFIG_RX65N_SCI11
static struct up_dev_s g_sci11priv =
{
  .scibase    =  RX65N_SCI11_BASE,
  .baud       =  CONFIG_SCI11_BAUD,
  .recvirq    =  RX65N_RXI11_IRQ,
  .xmitirq    =  RX65N_TXI11_IRQ,
  .eriirq     =  RX65N_ERI11_IRQ,
  .teiirq     =  RX65N_TEI11_IRQ,
  .grpibase   =  RX65N_GRPAL0_ADDR,
  .erimask    =  RX65N_GRPAL0_ERI11_MASK,
  .teimask    =  RX65N_GRPAL0_TEI11_MASK,
  .parity     =  CONFIG_SCI11_PARITY,
  .bits       =  CONFIG_SCI11_BITS,
  .stopbits2  =  CONFIG_SCI11_2STOP,
};

static uart_dev_t g_sci11port =
{
  .recv     =
    {
      .size    = CONFIG_SCI11_RXBUFSIZE,
      .buffer  = g_sci11rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI11_TXBUFSIZE,
      .buffer = g_sci11txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci11priv,
};
#endif

#ifdef CONFIG_RX65N_SCI12
static struct up_dev_s g_sci12priv =
{
  .scibase   =  RX65N_SCI12_BASE,
  .baud      =  CONFIG_SCI12_BAUD,
  .recvirq   =  RX65N_RXI12_IRQ,
  .xmitirq   =  RX65N_TXI12_IRQ,
  .eriirq    =  RX65N_ERI12_IRQ,
  .teiirq    =  RX65N_TEI12_IRQ,
  .grpibase  =  RX65N_GRPBL0_ADDR,
  .erimask   =  RX65N_GRPBL0_ERI12_MASK,
  .teimask   =  RX65N_GRPBL0_TEI12_MASK,
  .parity    =  CONFIG_SCI12_PARITY,
  .bits      =  CONFIG_SCI12_BITS,
  .stopbits2 =  CONFIG_SCI12_2STOP,
};

static uart_dev_t g_sci12port =
{
  .recv     =
    {
      .size    = CONFIG_SCI12_RXBUFSIZE,
      .buffer  = g_sci12rxbuffer,
    },

  .xmit     =
    {
      .size   = CONFIG_SCI12_TXBUFSIZE,
      .buffer = g_sci12txbuffer,
    },

  .ops      = &g_sci_ops,
  .priv     = &g_sci12priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint8_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg8(priv->scibase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline
void up_serialout(struct up_dev_s *priv, int offset, uint8_t value)
{
  putreg8(value, priv->scibase + offset);
}

/****************************************************************************
 * Name: up_disablesciint
 ****************************************************************************/

static inline void up_disablesciint(struct up_dev_s *priv, uint8_t *scr)
{
  if (scr)
    {
      *scr = priv->scr;
    }

  /* The disable all interrupts */

  priv->scr &= ~RX_SCISCR_ALLINTS;
  up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
}

/****************************************************************************
 * Name: up_restoresciint
 ****************************************************************************/

static inline void up_restoresciint(struct up_dev_s *priv, uint8_t scr)
{
  /* Set the interrupt bits in the scr value */

  priv->scr  &= ~RX_SCISCR_ALLINTS;
  priv->scr |= (scr & RX_SCISCR_ALLINTS);
  up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/

#ifdef HAVE_CONSOLE
static inline void up_waittxready(struct up_dev_s *priv)
{
  int tmp;

  /* Limit how long we will wait for the TDR empty condition */

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      /* Check if the TDR is empty.  The TDR becomes empty when:  (1) the
       * the chip is reset or enters standby mode, (2) the TE bit in the SCR
       * is cleared, or (3) the current TDR contents are loaded in the TSR so
       * that new data can be written in the TDR.
       */

      if (0 != (up_serialin(priv, RX_SCI_SSR_OFFSET) & RX_SCISSR_TDRE))
        {
          /* The TDR is empty... return */

          break;
        }
    }
}
#endif

/****************************************************************************
 * Name: up_setbrr
 *
 * Description:
 *   Calculate the correct value for the BRR given the configured frequency
 *   and the desired BAUD settings.
 *
 ****************************************************************************/

static inline void up_setbrr(struct up_dev_s *priv, unsigned int baud)
{
  uint32_t        t_baud;
  uint32_t        t_brr1;
  uint32_t        brrdiv;
  uint32_t        t_pclk_divbrr;
  float           f_mddr;
  uint8_t         semr;
  uint8_t         mddr;
  uint8_t         brr;

  semr = up_serialin(priv, RX_SCI_SEMR_OFFSET);
  brrdiv = 32U;
  if (0U != (0x10 & semr))
    {
      brrdiv /= 2U;
    }

  if (0U != (0x40 & semr))
    {
      brrdiv /= 2U;
    }

  if ((RX65N_SCI10_BASE == priv->scibase)
      || (RX65N_SCI11_BASE == priv->scibase))
    {
      t_pclk_divbrr = RX_PCLKA / brrdiv;
    }
  else
    {
      t_pclk_divbrr = RX_PCLKB / brrdiv;
    }

  t_brr1 = t_pclk_divbrr / baud;
  t_baud = t_pclk_divbrr / t_brr1;
  while (t_baud < baud)
    {
      t_brr1--;
      t_baud = RX_PCLKB / brrdiv / t_brr1;
    }

  brr  = t_brr1 - 1;
  f_mddr = ((float)baud * 256.0f) / (float)t_baud + 0.5f;
  mddr = ((256.0f <= f_mddr) || (0.0f > f_mddr)) ? 0 : (uint8_t)f_mddr;
  if (0 < mddr)
    {
      /* BRME(0x04) = 1; */

       semr |= 0x04;
    }
  else
    {
      /* BRME(0x04) = 0; */

       semr &= 0xf1;
       mddr = 255U;
    }

  /* RXDE(0x80) = NFEN(0x20) = 1; */

  semr |= 0xa0;

  up_serialout(priv, RX_SCI_SEMR_OFFSET, semr);
  up_serialout(priv, RX_SCI_BRR_OFFSET, brr);
  up_serialout(priv, RX_SCI_MDDR_OFFSET, mddr);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_SCI_CONFIG
  struct up_dev_s         *priv;
  uint8_t         smr;

  priv = (struct up_dev_s *)dev->priv;

  /* Disable the transmitter and receiver */

  priv->scr  = up_serialin(priv, RX_SCI_SCR_OFFSET);
  priv->scr &= ~(RX_SCISCR_TE | RX_SCISCR_RE);
  priv->scr &= ~RX_SCISCR_CKEMASK;
  up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);

  /* Set communication to be asynchronous with the configured number of data
   * bits, parity, and stop bits.  Use the internal clock (undivided)
   */

  smr = 0;
  if (7 == priv->bits)
    {
      smr |= RX_SCISMR_CHR;
    }

  if (1 == priv->parity)
    {
      smr |= (RX_SCISMR_PE | RX_SCISMR_OE);
    }
  else if(2 == priv->parity)
    {
      smr |= RX_SCISMR_PE;
    }

  if (priv->stopbits2)
    {
      smr |= RX_SCISMR_STOP;
    }

  up_serialout(priv, RX_SCI_SMR_OFFSET, smr);

  /* Set the baud based on the configured console baud and configured
   * system clock.
   */

  up_setbrr(priv, priv->baud);

  /* Then enable the transmitter and receiver */

  priv->scr |= (RX_SCISCR_TE | RX_SCISCR_RE);
  up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
#endif
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv;

  priv = (struct up_dev_s *)dev->priv;
  up_disablesciint(priv, NULL);
  priv->scr  = up_serialin(priv, RX_SCI_SCR_OFFSET);
  priv->scr &= ~(RX_SCISCR_TE | RX_SCISCR_RE);
  up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the SCI to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method
 *   (unless the
 *   hardware supports multiple levels of interrupt enabling).
 *   The RX and TX
 *   interrupts are not enabled until the txint() and rxint()
 *   methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv;
  int             ret;
  priv = (struct up_dev_s *)dev->priv;

  /* Attach the RDR full IRQ (RXI) that is enabled by the RIE SCR bit */

  ret = irq_attach(priv->recvirq, up_rcvinterrupt, dev);
  if (OK != ret)
    {
      return ret;
    }

  /* Attach the TDR empty IRQ (TXI) enabled by the TIE SCR bit */

  ret = irq_attach(priv->xmitirq, up_xmtinterrupt, dev);
  if (OK != ret)
    {
      return ret;
    }

  /* Attach the ERI IRQ */

  ret = irq_attach(priv->eriirq, up_eriinterrupt, dev);
  if (OK != ret)
    {
      return ret;
    }

  /* Attach the TEI IRQ */

  ret = irq_attach(priv->teiirq, up_teiinterrupt, dev);
  if (OK == ret)
    {
#ifdef CONFIG_ARCH_IRQPRIO
      /* All SCI0 interrupts share the same prioritization */

      up_prioritize_irq(priv->recvirq, 7);  /* Set SCI priority midway */
      up_prioritize_irq(priv->xmitirq, 7);
#endif

      /* Return OK on success */

      return OK;
    }

  irq_detach(priv->recvirq);
  irq_detach(priv->xmitirq);
  irq_detach(priv->eriirq);
  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach SCI interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv;
  priv = (struct up_dev_s *)dev->priv;

  /* Disable all SCI interrupts */

  up_disablesciint(priv, NULL);

  /* Detach the SCI interrupts */

  up_disable_irq(priv->recvirq);
  up_disable_irq(priv->xmitirq);
  irq_detach(priv->recvirq);
  irq_detach(priv->xmitirq);
}

static int up_eriinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev;
  struct up_dev_s   *priv;
  uint32_t grpreg;
  dev = (struct uart_dev_s *)arg;
  DEBUGASSERT((NULL != dev) && (NULL != priv));
  priv = (struct up_dev_s *)dev->priv;
  grpreg = getreg32(priv->grpibase);
  if (grpreg | priv->erimask)
    {
      /* Get the current SCI status  */

      priv->ssr = up_serialin(priv, RX_SCI_SSR_OFFSET);

      /* Clear all read related events (probably already done in
       * up_receive))
       */

      priv->ssr &= ~(RX_SCISSR_ORER | RX_SCISSR_FER | RX_SCISSR_PER);
      up_serialout(priv, RX_SCI_SSR_OFFSET, priv->ssr);
    }

  return OK;
}

static int up_teiinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev;
  struct up_dev_s   *priv;
  uint32_t grpreg;

  dev = (struct uart_dev_s *)arg;
  DEBUGASSERT((NULL != dev) && (NULL != priv));
  priv = (struct up_dev_s *)dev->priv;
  grpreg = getreg32(priv->grpibase);
  if (grpreg | priv->teimask)
    {
      /* Get the current SCI status  */

    priv->ssr = up_serialin(priv, RX_SCI_SSR_OFFSET);

      /* Clear all read related events (probably already done in
       * up_receive))
       */

     priv->ssr &= ~(RX_SCISSR_TEND);
     up_serialout(priv, RX_SCI_SSR_OFFSET, priv->ssr);
    }

  return OK;
}

static int up_rcvinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev;
  dev = (struct uart_dev_s *)arg;
  DEBUGASSERT((NULL != dev));

  /* Handle receive-related events with RIE is enabled.  RIE is enabled at
   * times that driver is open EXCEPT when the driver is actively copying
   * data from the circular buffer.  In that case, the read events must
   * pend until RIE is set
   */

  uart_recvchars(dev);
  return OK;
}

/****************************************************************************
 * Name: up_xmtinterrupt
 *
 * Description:
 *   This is the SCI interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the appropriate
 *   up_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int  up_xmtinterrupt(int irq, void *context, FAR void *arg)
{
  struct uart_dev_s *dev;
  dev = (struct uart_dev_s *)arg;
  DEBUGASSERT((NULL != dev));

  /* Handle outgoing, transmit bytes (TDRE: Transmit Data Register Empty)
   * when TIE is enabled.  TIE is only enabled when the driver is waiting
   * with buffered data.  Since TDRE is usually true,
   */

  uart_xmitchars(dev);
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
    int                ret    = OK;

    switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
#error CONFIG_SERIAL_TERMIOS NOT IMPLEMENTED
#endif /* CONFIG_SERIAL_TERMIOS */
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
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv;
  uint8_t rdr;
  uint8_t ssr;
  priv = (struct up_dev_s *)dev->priv;

  /* Read the character from the RDR port */

  rdr  = up_serialin(priv, RX_SCI_RDR_OFFSET);

  /* Clear all read related status in  real ssr (so that when when
   * rx available is called again, it will return false.
   */

  ssr = up_serialin(priv, RX_SCI_SSR_OFFSET);
  ssr &= ~(RX_SCISSR_RDRF | RX_SCISSR_ORER |
           RX_SCISSR_FER  | RX_SCISSR_PER) ;
  up_serialout(priv, RX_SCI_SSR_OFFSET, ssr);

  /* For status, return SSR at the time that the interrupt was received */

  *status = (uint32_t)priv->ssr << 8 | rdr;

  /* Return the received character */

  return (int)rdr;
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
  struct up_dev_s *priv;
  irqstate_t flags;
  priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts to prevent asynchronous accesses */

  flags = enter_critical_section();

  /* Are we enabling or disabling? */

  if (enable)
    {
      /* Enable the RDR full interrupt */

     priv->scr |= RX_SCISCR_RIE;
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
         up_enable_irq(priv->recvirq);
#endif
    }
  else
    {
      /* Disable the RDR full interrupt */

     priv->scr &= ~RX_SCISCR_RIE;
     up_disable_irq(priv->recvirq);
    }

  /* Write the modified SCR value to hardware */

  up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the RDR is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv;

  /* Return true if the RDR full bit is set in the SSR */

  priv = (struct up_dev_s *)dev->priv;
  return (0 != (up_serialin(priv, RX_SCI_SSR_OFFSET) & RX_SCISSR_RDRF));
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s  *priv;
  uint8_t ssr;
  priv = (struct up_dev_s *)dev->priv;

  /* Write the data to the TDR */

  up_serialout(priv, RX_SCI_TDR_OFFSET, (uint8_t)ch);

  /* Clear the TDRE bit in the SSR */

  ssr  = up_serialin(priv, RX_SCI_SSR_OFFSET);
  ssr &= ~(RX_SCISSR_TDRE | RX_SCISSR_TEND | RX_SCISSR_MPBT);
  up_serialout(priv, RX_SCI_SSR_OFFSET, ssr);
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
  struct up_dev_s *priv;
  irqstate_t      flags;
  priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts to prevent asynchronous accesses */

  flags = enter_critical_section();

  /* Are we enabling or disabling? */

  if (enable)
    {
      /* Enable the TDR empty interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_enable_irq(priv->xmitirq);
#endif

      priv->scr |= RX_SCISCR_TIE;

      /* If the TDR is already empty, then don't wait for the interrupt */

      if (up_txready(dev))
        {
          /* Tx data register empty ... process outgoing bytes.  Note:
           * this could call up_txint to be called recursively.  However,
           * in this event, priv->scr should hold the correct value upon
           * return from uuart_xmitchars().
           */

          uart_xmitchars(dev);
        }
    }
  else
    {
      /* Disable the TDR empty interrupt */

      up_disable_irq(priv->xmitirq);
    }

  /* Write the modified SCR value to hardware */

  up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the TDR is empty
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv;
  priv = (struct up_dev_s *)dev->priv;
  return (0 != (up_serialin(priv, RX_SCI_SSR_OFFSET) & RX_SCISSR_TDRE));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyconsoleinit
 *
 * Description:
 *   Performs the low level SCI initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_consoleinit.
 *
 ****************************************************************************/

void    up_earlyconsoleinit(void)
{
  /* NOTE:  All GPIO configuration for the SCIs was performed in
   * up_lowsetup
   */

  /* Disable all SCIs */

#ifdef TTYS0_DEV
  up_disablesciint(TTYS0_DEV.priv, NULL);
#endif
#ifdef TTYS1_DEV
  up_disablesciint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disablesciint(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  up_disablesciint(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
  up_disablesciint(TTYS4_DEV.priv, NULL);
#endif
#ifdef TTYS5_DEV
  up_disablesciint(TTYS5_DEV.priv, NULL);
#endif
#ifdef TTYS6_DEV
  up_disablesciint(TTYS6_DEV.priv, NULL);
#endif
#ifdef TTYS7_DEV
  up_disablesciint(TTYS7_DEV.priv, NULL);
#endif
#ifdef TTYS8_DEV
  up_disablesciint(TTYS8_DEV.priv, NULL);
#endif
#ifdef TTYS9_DEV
  up_disablesciint(TTYS9_DEV.priv, NULL);
#endif
#ifdef TTYS10_DEV
  up_disablesciint(TTYS10_DEV.priv, NULL);
#endif
#ifdef TTYS11_DEV
  up_disablesciint(TTYS11_DEV.priv, NULL);
#endif
#ifdef TTYS12_DEV
  up_disablesciint(TTYS12_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyconsoleinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  /* Register all SCIs */

#ifdef CONFIG_RX65N_SCI0
  r_sci0_create();
  r_sci0_start();
#endif
#ifdef CONFIG_RX65N_SCI1
  r_sci1_create();
  r_sci1_start();
#endif
#ifdef CONFIG_RX65N_SCI2
  r_sci2_create();
  r_sci2_start();
#endif
#ifdef CONFIG_RX65N_SCI3
  r_sci3_create();
  r_sci3_start();
#endif
#ifdef CONFIG_RX65N_SCI4
  r_sci4_create();
  r_sci4_start();
#endif
#ifdef CONFIG_RX65N_SCI5
  r_sci5_create();
  r_sci5_start();
#endif
#ifdef CONFIG_RX65N_SCI6
  r_sci6_create();
  r_sci6_start();
#endif
#ifdef CONFIG_RX65N_SCI7
  r_sci7_create();
  r_sci7_start();
#endif
#ifdef CONFIG_RX65N_SCI8
  r_sci8_create();
  r_sci8_start();
#endif
#ifdef CONFIG_RX65N_SCI9
  r_sci9_create();
  r_sci9_start();
#endif
#ifdef CONFIG_RX65N_SCI10
  r_sci10_create();
  r_sci10_start();
#endif
#ifdef CONFIG_RX65N_SCI11
  r_sci11_create();
  r_sci11_start();
#endif
#ifdef CONFIG_RX65N_SCI12
  r_sci12_create();
  r_sci12_start();
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
#ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS4_DEV);
#endif
#ifdef TTYS5_DEV
  uart_register("/dev/ttyS5", &TTYS5_DEV);
#endif
#ifdef TTYS6_DEV
  uart_register("/dev/ttyS6", &TTYS6_DEV);
#endif
#ifdef TTYS7_DEV
  uart_register("/dev/ttyS7", &TTYS7_DEV);
#endif
#ifdef TTYS8_DEV
  uart_register("/dev/ttyS8", &TTYS8_DEV);
#endif
#ifdef TTYS9_DEV
  uart_register("/dev/ttyS9", &TTYS9_DEV);
#endif
#ifdef TTYS10_DEV
  uart_register("/dev/ttyS10", &TTYS10_DEV);
#endif
#ifdef TTYS11_DEV
  uart_register("/dev/ttyS11", &TTYS11_DEV);
#endif
#ifdef TTYS12_DEV
  uart_register("/dev/ttyS12", &TTYS12_DEV);
#endif

  /* Register the console */

#ifdef HAVE_CONSOLE
  up_setup(&CONSOLE_DEV);
  uart_register("/dev/console", &CONSOLE_DEV);
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
  struct up_dev_s *priv;
  uint8_t  scr;
  priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  up_disablesciint(priv, &scr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxready(priv);
      up_serialout(priv, RX_SCI_TDR_OFFSET, '\r');
    }

  up_waittxready(priv);
  up_serialout(priv, RX_SCI_TDR_OFFSET, (uint8_t)ch);
  up_waittxready(priv);
  up_restoresciint(priv, scr);
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
#ifdef HAVE_CONSOLE

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
