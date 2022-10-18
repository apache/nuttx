/****************************************************************************
 * arch/arm/src/tiva/common/tiva_hciuart.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
#include "chip.h"
#include "tiva_hciuart.h"
#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"
#include "tiva_periphrdy.h"
#include "tiva_gpio.h"
#include "hardware/tiva_pinmap.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All interrupts */

#define HCIUART_ALLINTS   (UART_IM_TXIM | UART_IM_RXIM | UART_IM_RTIM)
#define HCIUART_RXHANDLED (1 << 0)
#define HCIUART_TXHANDLED (1 << 1)

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_TIVA_PM_SERIAL_ACTIVITY)
#  define CONFIG_TIVA_PM_SERIAL_ACTIVITY 10
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure is the variable state of the HCI UART */

struct hciuart_state_s
{
  /* Registered Rx callback */

  btuart_rxcallback_t callback;      /* Rx callback function */
  void *arg;                         /* Rx callback argument */

  /* Rx/Tx circular buffer management */

  sem_t rxwait;                      /* Supports wait for more Rx data */
  sem_t txwait;                      /* Supports wait for space in Tx buffer */

  uint32_t baud;                     /* Current BAUD selection */
  volatile uint16_t rxhead;          /* Head and tail index of the Rx buffer */
  uint16_t rxtail;
  uint16_t txhead;                   /* Head and tail index of the Tx buffer */
  volatile uint16_t txtail;
  volatile bool rxwaiting;           /* A thread is waiting for more Rx data */
  volatile bool txwaiting;           /* A thread is waiting for space in the Tx buffer */
  uint32_t im;                       /* Saved IM value */
};

/* This structure is the constant configuration of the HCI UART */

struct hciuart_config_s
{
  struct btuart_lowerhalf_s lower;   /* Generic HCI-UART lower half */
  struct hciuart_state_s *state;     /* Reference to variable state */
  uint8_t *rxbuffer;                 /* Rx buffer start */
  uint8_t *txbuffer;                 /* Tx buffer start */
  uint16_t rxbufsize;                /* Size of the Rx buffer */
  uint16_t txbufsize;                /* Size of the tx buffer */

  uint8_t irq;                       /* IRQ associated with this UART */
  uint32_t baud;                     /* Configured baud */
  uint32_t id;                       /* UART identifier */
  uint32_t uartbase;                 /* Base address of UART registers */
  uint32_t tx_gpio;                  /* UART TX GPIO pin configuration */
  uint32_t rx_gpio;                  /* UART RX GPIO pin configuration */
  uint32_t cts_gpio;                 /* UART CTS GPIO pin configuration */
  uint32_t rts_gpio;                 /* UART RTS GPIO pin configuration */
  uint32_t shutd_gpio;               /* */

  uint8_t  parity;                   /* 0=none, 1=odd, 2=even */
  uint8_t  bits;                     /* Number of bits (7 or 8) */
  bool     stopbits2;                /* true: Configure 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t hciuart_getreg32(
              const struct hciuart_config_s *config, unsigned int offset);
static inline void hciuart_putreg32(const struct hciuart_config_s *config,
              unsigned int offset, uint32_t value);
static void hciuart_enableints(const struct hciuart_config_s *config,
              uint32_t intset);
static void hciuart_disableints(const struct hciuart_config_s *config,
              uint32_t intset);
static bool hciuart_isenabled(const struct hciuart_config_s *config,
              uint32_t intset);
static inline bool hciuart_rxenabled(const struct hciuart_config_s *config);

static uint16_t hciuart_rxinuse(const struct hciuart_config_s *config);
static ssize_t hciuart_copytorxbuffer(const struct hciuart_config_s *config);
static ssize_t hciuart_copyfromrxbuffer(
              const struct hciuart_config_s *config, uint8_t *dest,
              size_t destlen);
static ssize_t hciuart_copytotxfifo(const struct hciuart_config_s *config);
static void hciuart_line_configure(const struct hciuart_config_s *config);
static int  hciuart_configure(const struct hciuart_config_s *config);
static int  hciuart_interrupt(int irq, void *context, void *arg);

/* HCI-UART Lower-Half Methods */

static void hciuart_rxattach(const struct btuart_lowerhalf_s *lower,
              btuart_rxcallback_t callback, void *arg);
static void hciuart_rxenable(const struct btuart_lowerhalf_s *lower,
              bool enable);
static int hciuart_setbaud(const struct btuart_lowerhalf_s *lower,
              uint32_t baud);
static ssize_t hciuart_read(const struct btuart_lowerhalf_s *lower,
              void *buffer, size_t buflen);
static ssize_t hciuart_write(const struct btuart_lowerhalf_s *lower,
              const void *buffer, size_t buflen);
static ssize_t hciuart_rxdrain(const struct btuart_lowerhalf_s *lower);

#ifdef CONFIG_PM
static void hciuart_pm_notify(struct pm_callback_s *cb, int dowmin,
              enum pm_state_e pmstate);
static int  hciuart_pm_prepare(struct pm_callback_s *cb, int domain,
              enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This describes the state of the TIVA UART0 port. */

#ifdef CONFIG_TIVA_UART0_HCIUART
/* I/O buffers */

static uint8_t g_uart0_rxbuffer[CONFIG_TIVA_HCIUART0_RXBUFSIZE];
static uint8_t g_uart0_txbuffer[CONFIG_TIVA_HCIUART0_TXBUFSIZE];

/* HCI UART0 variable state information */

static struct hciuart_state_s g_hciuart0_state;

/* HCI UART0 constant configuration information */

static const struct hciuart_config_s g_hciuart0_config =
{
  .lower         =
    {
      .rxattach  = hciuart_rxattach,
      .rxenable  = hciuart_rxenable,
      .setbaud   = hciuart_setbaud,
      .read      = hciuart_read,
      .write     = hciuart_write,
      .rxdrain   = hciuart_rxdrain,
    },

  .state         = &g_hciuart0_state,

  .rxbuffer      = g_uart0_rxbuffer,
  .txbuffer      = g_uart0_txbuffer,
  .rxbufsize     = CONFIG_TIVA_HCIUART0_RXBUFSIZE,
  .txbufsize     = CONFIG_TIVA_HCIUART0_TXBUFSIZE,

  .irq           = TIVA_IRQ_UART0,
  .baud          = CONFIG_TIVA_HCIUART0_BAUD,
  .id            = 0,
  .uartbase      = TIVA_UART0_BASE,
  .tx_gpio       = GPIO_UART0_TX,
  .rx_gpio       = GPIO_UART0_RX,
  .cts_gpio      = UART0_GPIO_CTS,
  .rts_gpio      = UART0_GPIO_RTS,
  .shutd_gpio    = UART0_GPIO_NSHUTD
};
#endif

/* This describes the state of the TIVA UART1 port. */

#ifdef CONFIG_TIVA_UART1_HCIUART

/* I/O buffers */

static uint8_t g_uart1_rxbuffer[CONFIG_TIVA_HCIUART1_RXBUFSIZE];
static uint8_t g_uart1_txbuffer[CONFIG_TIVA_HCIUART1_TXBUFSIZE];

/* HCI UART1 variable state information */

static struct hciuart_state_s g_hciuart1_state;

/* HCI UART1 constant configuration information */

static const struct hciuart_config_s g_hciuart1_config =
{
  .lower         =
    {
      .rxattach  = hciuart_rxattach,
      .rxenable  = hciuart_rxenable,
      .setbaud   = hciuart_setbaud,
      .read      = hciuart_read,
      .write     = hciuart_write,
      .rxdrain   = hciuart_rxdrain,
    },

  .state         = &g_hciuart1_state,

  .rxbuffer      = g_uart1_rxbuffer,
  .txbuffer      = g_uart1_txbuffer,
  .rxbufsize     = CONFIG_TIVA_HCIUART1_RXBUFSIZE,
  .txbufsize     = CONFIG_TIVA_HCIUART1_TXBUFSIZE,

  .irq           = TIVA_IRQ_UART1,
  .baud          = CONFIG_TIVA_HCIUART1_BAUD,
  .id            = 1,
  .uartbase      = TIVA_UART1_BASE,
  .tx_gpio       = GPIO_UART1_TX,
  .rx_gpio       = GPIO_UART1_RX,
  .cts_gpio      = UART1_GPIO_CTS,
  .rts_gpio      = UART1_GPIO_RTS,
  .shutd_gpio    = UART1_GPIO_NSHUTD
};
#endif

/* This describes the state of the TIVA UART2 port. */

#ifdef CONFIG_TIVA_UART2_HCIUART

/* I/O buffers */

static uint8_t g_uart2_rxbuffer[CONFIG_TIVA_HCIUART2_RXBUFSIZE];
static uint8_t g_uart2_txbuffer[CONFIG_TIVA_HCIUART2_TXBUFSIZE];

/* HCI UART2 variable state information */

static struct hciuart_state_s g_hciuart2_state;

/* HCI UART2 constant configuration information */

static const struct hciuart_config_s g_hciuart2_config =
{
  .lower         =
    {
      .rxattach  = hciuart_rxattach,
      .rxenable  = hciuart_rxenable,
      .setbaud   = hciuart_setbaud,
      .read      = hciuart_read,
      .write     = hciuart_write,
      .rxdrain   = hciuart_rxdrain,
    },

  .state         = &g_hciuart2_state,

  .rxbuffer      = g_uart2_rxbuffer,
  .txbuffer      = g_uart2_txbuffer,
  .rxbufsize     = CONFIG_TIVA_HCIUART2_RXBUFSIZE,
  .txbufsize     = CONFIG_TIVA_HCIUART2_TXBUFSIZE,

  .irq           = TIVA_IRQ_UART2,
  .baud          = CONFIG_TIVA_HCIUART2_BAUD,
  .id            = 2,
  .uartbase      = TIVA_UART2_BASE,
  .tx_gpio       = GPIO_UART2_TX,
  .rx_gpio       = GPIO_UART2_RX,
  .cts_gpio      = UART2_GPIO_CTS,
  .rts_gpio      = UART2_GPIO_RTS,
  .shutd_gpio    = UART2_GPIO_NSHUTD
};
#endif

/* This describes the state of the TIVA UART3 port. */

#ifdef CONFIG_TIVA_UART3_HCIUART

/* I/O buffers */

static uint8_t g_uart3_rxbuffer[CONFIG_TIVA_HCIUART3_RXBUFSIZE];
static uint8_t g_uart3_txbuffer[CONFIG_TIVA_HCIUART3_TXBUFSIZE];

/* HCI UART3 variable state information */

static struct hciuart_state_s g_hciuart3_state;

/* HCI UART3 constant configuration information */

static const struct hciuart_config_s g_hciuart3_config =
{
  .lower         =
    {
      .rxattach  = hciuart_rxattach,
      .rxenable  = hciuart_rxenable,
      .setbaud   = hciuart_setbaud,
      .read      = hciuart_read,
      .write     = hciuart_write,
      .rxdrain   = hciuart_rxdrain,
    },

  .state         = &g_hciuart3_state,

  .rxbuffer      = g_uart3_rxbuffer,
  .txbuffer      = g_uart3_txbuffer,
  .rxbufsize     = CONFIG_TIVA_HCIUART3_RXBUFSIZE,
  .txbufsize     = CONFIG_TIVA_HCIUART3_TXBUFSIZE,

  .irq           = TIVA_IRQ_UART3,
  .baud          = CONFIG_TIVA_HCIUART3_BAUD,
  .id            = 3,
  .uartbase      = TIVA_UART3_BASE,
  .tx_gpio       = GPIO_UART3_TX,
  .rx_gpio       = GPIO_UART3_RX,
  .cts_gpio      = UART3_GPIO_CTS,
  .rts_gpio      = UART3_GPIO_RTS,
  .shutd_gpio    = UART3_GPIO_NSHUTD
};
#endif

/* This describes the state of the TIVA UART4 port. */

#ifdef CONFIG_TIVA_UART4_HCIUART

/* I/O buffers */

static uint8_t g_uart4_rxbuffer[CONFIG_TIVA_HCIUART4_RXBUFSIZE];
static uint8_t g_uart4_txbuffer[CONFIG_TIVA_HCIUART4_TXBUFSIZE];

/* HCI UART4 variable state information */

static struct hciuart_state_s g_hciuart4_state;

/* HCI UART4 constant configuration information */

static const struct hciuart_config_s g_hciuart4_config =
{
  .lower         =
    {
      .rxattach  = hciuart_rxattach,
      .rxenable  = hciuart_rxenable,
      .setbaud   = hciuart_setbaud,
      .read      = hciuart_read,
      .write     = hciuart_write,
      .rxdrain   = hciuart_rxdrain,
    },

  .state         = &g_hciuart4_state,

  .rxbuffer      = g_uart4_rxbuffer,
  .txbuffer      = g_uart4_txbuffer,
  .rxbufsize     = CONFIG_TIVA_HCIUART4_RXBUFSIZE,
  .txbufsize     = CONFIG_TIVA_HCIUART4_TXBUFSIZE,

  .irq           = TIVA_IRQ_UART4,
  .baud          = CONFIG_TIVA_HCIUART4_BAUD,
  .id            = 4,
  .uartbase      = TIVA_UART4_BASE,
  .tx_gpio       = GPIO_UART4_TX,
  .rx_gpio       = GPIO_UART4_RX,
  .cts_gpio      = UART4_GPIO_CTS,
  .rts_gpio      = UART4_GPIO_RTS,
  .shutd_gpio    = UART4_GPIO_NSHUTD
};
#endif

/* This describes the state of the TIVA UART5 port. */

#ifdef CONFIG_TIVA_UART5_HCIUART

/* I/O buffers */

static uint8_t g_uart5_rxbuffer[CONFIG_TIVA_HCIUART5_RXBUFSIZE];
static uint8_t g_uart5_txbuffer[CONFIG_TIVA_HCIUART5_TXBUFSIZE];

/* HCI UART5 variable state information */

static struct hciuart_state_s g_hciuart5_state;

/* HCI UART5 constant configuration information */

static const struct hciuart_config_s g_hciuart5_config =
{
  .lower         =
    {
      .rxattach  = hciuart_rxattach,
      .rxenable  = hciuart_rxenable,
      .setbaud   = hciuart_setbaud,
      .read      = hciuart_read,
      .write     = hciuart_write,
      .rxdrain   = hciuart_rxdrain,
    },

  .state         = &g_hciuart5_state,

  .rxbuffer      = g_uart5_rxbuffer,
  .txbuffer      = g_uart5_txbuffer,
  .rxbufsize     = CONFIG_TIVA_HCIUART5_RXBUFSIZE,
  .txbufsize     = CONFIG_TIVA_HCIUART5_TXBUFSIZE,

  .irq           = TIVA_IRQ_UART5,
  .baud          = CONFIG_TIVA_HCIUART5_BAUD,
  .id            = 5,
  .uartbase      = TIVA_UART5_BASE,
  .tx_gpio       = GPIO_UART5_TX,
  .rx_gpio       = GPIO_UART5_RX,
  .cts_gpio      = UART5_GPIO_CTS,
  .rts_gpio      = UART5_GPIO_RTS,
  .shutd_gpio    = UART5_GPIO_NSHUTD
};
#endif

/* This describes the state of the TIVA UART6 port. */

#ifdef CONFIG_TIVA_UART6_HCIUART

/* I/O buffers */

static uint8_t g_uart6_rxbuffer[CONFIG_TIVA_HCIUART6_RXBUFSIZE];
static uint8_t g_uart6_txbuffer[CONFIG_TIVA_HCIUART6_TXBUFSIZE];

/* HCI UART6 variable state information */

static struct hciuart_state_s g_hciuart6_state;

/* HCI UART6 constant configuration information */

static const struct hciuart_config_s g_hciuart6_config =
{
  .lower         =
    {
      .rxattach  = hciuart_rxattach,
      .rxenable  = hciuart_rxenable,
      .setbaud   = hciuart_setbaud,
      .read      = hciuart_read,
      .write     = hciuart_write,
      .rxdrain   = hciuart_rxdrain,
    },

  .state         = &g_hciuart6_state,

  .rxbuffer      = g_uart6_rxbuffer,
  .txbuffer      = g_uart6_txbuffer,
  .rxbufsize     = CONFIG_TIVA_HCIUART6_RXBUFSIZE,
  .txbufsize     = CONFIG_TIVA_HCIUART6_TXBUFSIZE,

  .irq           = TIVA_IRQ_UART6,
  .baud          = CONFIG_TIVA_HCIUART6_BAUD,
  .id            = 6,
  .uartbase      = TIVA_UART6_BASE,
  .tx_gpio       = GPIO_UART6_TX,
  .rx_gpio       = GPIO_UART6_RX,
  .cts_gpio      = UART6_GPIO_CTS,
  .rts_gpio      = UART6_GPIO_RTS,
  .shutd_gpio    = UART6_GPIO_NSHUTD
};
#endif

/* This describes the state of the TIVA UART7 port. */

#ifdef CONFIG_TIVA_UART7_HCIUART

/* I/O buffers */

static uint8_t g_uart7_rxbuffer[CONFIG_TIVA_HCIUART7_RXBUFSIZE];
static uint8_t g_uart7_txbuffer[CONFIG_TIVA_HCIUART7_TXBUFSIZE];

/* HCI UART7 variable state information */

static struct hciuart_state_s g_hciuart7_state;

/* HCI UART7 constant configuration information */

static const struct hciuart_config_s g_hciuart7_config =
{
  .lower         =
    {
      .rxattach  = hciuart_rxattach,
      .rxenable  = hciuart_rxenable,
      .setbaud   = hciuart_setbaud,
      .read      = hciuart_read,
      .write     = hciuart_write,
      .rxdrain   = hciuart_rxdrain,
    },

  .state         = &g_hciuart7_state,

  .rxbuffer      = g_uart7_rxbuffer,
  .txbuffer      = g_uart7_txbuffer,
  .rxbufsize     = CONFIG_TIVA_HCIUART7_RXBUFSIZE,
  .txbufsize     = CONFIG_TIVA_HCIUART7_TXBUFSIZE,

  .irq           = TIVA_IRQ_UART7,
  .baud          = CONFIG_TIVA_HCIUART7_BAUD,
  .id            = 7,
  .uartbase      = TIVA_UART7_BASE,
  .tx_gpio       = GPIO_UART7_TX,
  .rx_gpio       = GPIO_UART7_RX,
  .cts_gpio      = UART7_GPIO_CTS,
  .rts_gpio      = UART7_GPIO_RTS,
  .shutd_gpio    = UART7_GPIO_NSHUTD
};
#endif

/* This table lets us iterate over the configured UARTs */

static const struct hciuart_config_s * const g_hciuarts[] =
{
#ifdef CONFIG_TIVA_UART0_HCIUART
  &g_hciuart0_config,   /* HCI UART on TIVA UART0 */
#endif
#ifdef CONFIG_TIVA_UART1_HCIUART
  &g_hciuart1_config,   /* HCI UART on TIVA UART1 */
#endif
#ifdef CONFIG_TIVA_UART2_HCIUART
  &g_hciuart2_config,   /* HCI UART on TIVA UART2 */
#endif
#ifdef CONFIG_TIVA_UART3_HCIUART
  &g_hciuart3_config,   /* HCI UART on TIVA UART3 */
#endif
#ifdef CONFIG_TIVA_UART4_HCIUART
  &g_hciuart4_config,   /* HCI UART on TIVA UART4 */
#endif
#ifdef CONFIG_TIVA_UART5_HCIUART
  &g_hciuart5_config,   /* HCI UART on TIVA UART5 */
#endif
#ifdef CONFIG_TIVA_UART6_HCIUART
  &g_hciuart6_config,   /* HCI UART on TIVA UART6 */
#endif
#ifdef CONFIG_TIVA_UART7_HCIUART
  &g_hciuart7_config,    /* HCI UART on TIVA UART7 */
#endif
};

#ifdef CONFIG_PM
static  struct pm_callback_s g_serialcb =
{
  .notify  = hciuart_pm_notify,
  .prepare = hciuart_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hciuart_getreg32
 ****************************************************************************/

static inline uint32_t
  hciuart_getreg32(const struct hciuart_config_s *config,
                   unsigned int offset)
{
  return getreg32(config->uartbase + offset);
}

/****************************************************************************
 * Name: hciuart_putreg32
 ****************************************************************************/

static inline void hciuart_putreg32(const struct hciuart_config_s *config,
                                    unsigned int offset, uint32_t value)
{
  putreg32(value, config->uartbase + offset);
}

/****************************************************************************
 * Name: hciuart_enableints
 *
 * Description:
 *   Enable interrupts as specified by bits in the 'intset' argument
 *
 *   NOTE: This operation is not atomic.  This function should be called
 *   only from within a critical section.
 *
 ****************************************************************************/

static void hciuart_enableints(const struct hciuart_config_s *config,
                               uint32_t intset)
{
  config->state->im |= intset;

  hciuart_putreg32(config, TIVA_UART_IM_OFFSET, config->state->im);
}

/****************************************************************************
 * Name: hciuart_disableints
 *
 * Description:
 *   Disable interrupts as specified by bits in the 'intset' argument
 *
 *   NOTE: This operation is not atomic.  This function should be called
 *   only from within a critical section.
 *
 ****************************************************************************/

static void hciuart_disableints(const struct hciuart_config_s *config,
                                uint32_t intset)
{
  config->state->im &= ~intset;

  hciuart_putreg32(config, TIVA_UART_IM_OFFSET, config->state->im);
}

/****************************************************************************
 * Name: hciuart_isenabled
 *
 * Description:
 *   Return true if any any of the interrupts specified in the 'intset'
 *   argument are enabled.
 *
 ****************************************************************************/

static bool hciuart_isenabled(const struct hciuart_config_s *config,
                              uint32_t intset)
{
  if ((config->state->im & intset) != 0)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: hciuart_rxenabled
 *
 * Description:
 *   Check if Rx interrupts are enabled.
 *
 ****************************************************************************/

static inline bool hciuart_rxenabled(const struct hciuart_config_s *config)
{
  return hciuart_isenabled(config, UART_MIS_RXMIS | UART_MIS_RTMIS);
}

/****************************************************************************
 * Name: hciuart_rxinuse
 *
 * Description:
 *   Return the number of bytes in the Rx buffer
 *
 *   Example:  rxbufsize=4, rxhead = 0, rxtail = 2
 *
 *    +---+---+---+---+
 *    | X | X |   |   |  X = inuse
 *    +---+---+---+---+
 *      |       `- rxtail = 2
 *      `- rxhead = 0
 *
 *    inuse = 2 - 0 = 2
 *
 *   Example:  rxbufsize=4, rxhead = 2, rxtail = 0
 *
 *    +---+---+---+---+
 *    |   |   | X | X |  X = inuse
 *    +---+---+---+---+
 *      |       `- rxhead = 2
 *      `- rxtail = 0
 *
 *    inuse = (0 + 4) - 2 = 2
 *
 ****************************************************************************/

static uint16_t hciuart_rxinuse(const struct hciuart_config_s *config)
{
  struct hciuart_state_s *state;
  size_t inuse;

  DEBUGASSERT(config != NULL && config->state != NULL);
  state = config->state;

  /* Keep track of how much is discarded */

  if (state->rxtail >= state->rxhead)
    {
      inuse = state->rxtail - state->rxhead;
    }
  else
    {
      inuse = (state->rxtail + config->rxbufsize) - state->rxhead;
    }

  return inuse;
}

/****************************************************************************
 * Name: hciuart_copytorxbuffer
 *
 * Description:
 *   Copy data to the driver Rx buffer.
 *
 ****************************************************************************/

static ssize_t hciuart_copytorxbuffer(const struct hciuart_config_s *config)
{
  struct hciuart_state_s *state;
  ssize_t nbytes = 0;
  uint16_t rxhead;
  uint16_t rxtail;
  uint16_t rxnext;
  uint8_t rxbyte;

  /* Get a copy of the rxhead and rxtail indices of the Rx buffer */

  state  = config->state;
  rxhead = state->rxhead;
  rxtail = state->rxtail;

    {
      /* Is there data available in the Rx FIFO? */

      while ((hciuart_getreg32(config, TIVA_UART_FR_OFFSET) & UART_FR_RXFE)
             == 0)
        {
          /* Compare the Rx buffer head and tail indices.  If the
           * incremented tail index would make the Rx buffer appear empty,
           * then we must stop the copy.  If there is data pending in the Rx
           * FIFO, this could be very bad because a data overrun condition
           * is likely to* occur.
           */

          rxnext = rxtail + 1;
          if (rxnext >= config->rxbufsize)
            {
              rxnext = 0;
            }

          /* Would this make the Rx buffer appear full? */

          if (rxnext == rxhead)
            {
              /* Yes, stop the copy and update the indices */

              break;
            }

          /* Get a byte from the Rx FIFO buffer */

          rxbyte = hciuart_getreg32(config, TIVA_UART_DR_OFFSET) & 0xff;

          /* And add it to the tail of the Rx buffer */

          config->rxbuffer[rxtail] = rxbyte;
          rxtail = rxnext;
          nbytes++;
        }
    }

  /* Save the updated Rx buffer tail index */

  state->rxtail = rxtail;

  /* Notify any waiting threads that new Rx data is available */

  if (nbytes > 0 && state->rxwaiting)
    {
      state->rxwaiting = false;
      nxsem_post(&state->rxwait);
    }

  wlinfo("rxhead %u rxtail %u nbytes %ld\n", rxhead, rxtail, (long)nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: hciuart_copyfromrxbuffer
 *
 * Description:
 *   Copy data from the driver Rx buffer to the caller provided destination
 *   buffer.
 *
 ****************************************************************************/

static ssize_t
  hciuart_copyfromrxbuffer(const struct hciuart_config_s *config,
                           uint8_t *dest, size_t destlen)
{
  struct hciuart_state_s *state;
  ssize_t nbytes;
  uint16_t rxhead;
  uint16_t rxtail;
  uint8_t rxbyte;

  /* Get a copy of the rxhead and rxtail indices of the Rx buffer */

  state  = config->state;
  rxhead = state->rxhead;
  rxtail = state->rxtail;
  nbytes = 0;

  /* Is there data available in the Rx buffer?  Is there space in the user
   * buffer?
   */

  while (rxhead != rxtail && nbytes < destlen)
    {
      /* Get a byte from the head of the Rx buffer */

      rxbyte = config->rxbuffer[rxhead];

      /* And add it to the caller's buffer buffer */

      dest[nbytes] = rxbyte;

      /* Update indices and counts */

      nbytes++;

      if (++rxhead >= config->rxbufsize)
        {
          rxhead = 0;
        }
    }

  /* Save the updated Rx buffer head index */

  state->rxhead = rxhead;

  wlinfo("rxhead %u rxtail %u nbytes %ld\n", rxhead, rxtail, (long)nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: hciuart_copytotxfifo
 *
 * Description:
 *   Copy data from the Tx buffer to the Tx FIFO
 *
 ****************************************************************************/

static ssize_t hciuart_copytotxfifo(const struct hciuart_config_s *config)
{
  struct hciuart_state_s *state;
  ssize_t nbytes;
  uint16_t txhead;
  uint16_t txtail;
  uint8_t txbyte;

  /* Get a copy of the txhead and txtail indices of the Rx buffer */

  state  = config->state;
  txhead = state->txhead;
  txtail = state->txtail;
  nbytes = 0;

  /* Compare the Tx buffer head and tail indices.  If the Tx buffer is
   * empty, then we finished with the copy.
   */

  while (txhead != txtail)
    {
#ifdef CONFIG_TIVA_HCIUART_SW_TXFLOW
      if (tiva_gpioread(config->cts_gpio))
        {
          break;
        }
#endif

      /* Is the transmit data register empty?
       *
       * Transmit data register empty
       *   This bit is set by hardware when the content of the transmit
       *   register has been transferred into the shift register.
       */

      if ((hciuart_getreg32(config, TIVA_UART_FR_OFFSET) &
           UART_FR_TXFF) != 0)
        {
          break;
        }

      /* Get a byte from the head of the Tx buffer */

      txbyte = config->txbuffer[txhead];
      if (++txhead >= config->txbufsize)
        {
          txhead = 0;
        }

      /* And add it to the of the Tx FIFO */

      hciuart_putreg32(config, TIVA_UART_DR_OFFSET, (uint32_t)txbyte);
      nbytes++;
    }

  wlinfo("txhead %u txtail %u nbytes %ld\n", txhead, txtail, (long)nbytes);
  state->txhead = txhead;
  return nbytes;
}

/****************************************************************************
 * Name: hciuart_line_configure
 *
 * Description:
 *   Set the serial line format and speed.
 *
 *   Per "Specification of the Bluetooth System, Wireless connections made
 *   easy, Host Controller Interface [Transport Layer]", Volume 4, Revision
 *   1.2 or later, 1 January 2006, HCI UART transport uses these settings:
 *
 *     8 data bits, no parity, 1 stop, RTS/CTS flow control
 *
 *   BAUD and flow control response time are manufacturer specific.
 *
 ****************************************************************************/

static void hciuart_line_configure(const struct hciuart_config_s *config)
{
  uint32_t baud;

  uint32_t den;
  uint32_t brdi;
  uint32_t remainder;
  uint32_t divfrac;
  uint32_t lcrh;

  /* The current BAUD selection is part of the variable state data */

  DEBUGASSERT(config != NULL && config->state != NULL);
  baud = config->state->baud;

  wlinfo("baud %lu\n", (unsigned long)baud);

  den       = baud << 4;
  brdi      = SYSCLK_FREQUENCY / den;
  remainder = SYSCLK_FREQUENCY - den * brdi;
  divfrac   = ((remainder << 6) + (den >> 1)) / den;

  hciuart_putreg32(config, TIVA_UART_IBRD_OFFSET, brdi);
  hciuart_putreg32(config, TIVA_UART_FBRD_OFFSET, divfrac);

  /* Configure 8 data bits, No parity, 1 stop bit - required by HCI UART */

  lcrh = 0;
  lcrh |= UART_LCRH_WLEN_8BITS;
  hciuart_putreg32(config, TIVA_UART_LCRH_OFFSET, lcrh);
}

/****************************************************************************
 * Name: hciuart_configure
 *
 * Description:
 *   Configure the UART clocking, GPIO pins, baud, bits, parity, etc.
 *
 *   Per "Specification of the Bluetooth System, Wireless connections made
 *   easy, Host Controller Interface [Transport Layer]", Volume 4, Revision
 *   1.2 or later, 1 January 2006, HCI UART transport uses these settings:
 *
 *     8 data bits, no parity, 1 stop, RTS/CTS flow control
 *
 *   BAUD and flow control response time are manufacturer specific.
 *
 ****************************************************************************/

static int hciuart_configure(const struct hciuart_config_s *config)
{
  uint32_t lcrh;
  uint32_t ctl;

  /* Note: The logic here depends on the fact that that the UART module
   * was enabled in tiva_lowsetup().
   */

  wlinfo("config %p\n", config);

  /* Configure pins for UART use */

  tiva_configgpio(config->tx_gpio);
  tiva_configgpio(config->rx_gpio);

  tiva_configgpio(config->cts_gpio);
  tiva_configgpio(config->rts_gpio);

#ifdef CONFIG_BLUETOOTH_UART_CC2564
  tiva_configgpio(config->shutd_gpio);
#endif

  DEBUGASSERT(config->state != NULL);
  config->state->baud = config->baud;
  hciuart_line_configure(config);

  /* Enable the FIFOs */

  lcrh = hciuart_getreg32(config, TIVA_UART_LCRH_OFFSET);
  lcrh |= UART_LCRH_FEN;
  hciuart_putreg32(config, TIVA_UART_LCRH_OFFSET, lcrh);

  /* Enable Rx, Tx, and the UART */

  ctl = hciuart_getreg32(config, TIVA_UART_CTL_OFFSET);
  ctl |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
  hciuart_putreg32(config, TIVA_UART_CTL_OFFSET, ctl);

  /* Set up the cache IM value */

  config->state->im = hciuart_getreg32(config, TIVA_UART_IM_OFFSET);

  hciuart_putreg32(config, TIVA_UART_IFLS_OFFSET,
                   UART_IFLS_TXIFLSEL_18TH | UART_IFLS_RXIFLSEL_78TH);

  hciuart_putreg32(config, TIVA_UART_IM_OFFSET, UART_IM_RXIM | UART_IM_RTIM);

  /* Enable bluetooth module */

#ifdef CONFIG_BLUETOOTH_UART_CC2564
  tiva_gpiowrite(config->shutd_gpio, true);
#endif

#ifdef CONFIG_TIVA_HCIUART_SW_TXFLOW
  while (tiva_gpioread(config->cts_gpio))
    {
    }
#endif

#ifdef CONFIG_TIVA_HCIUART_SW_RXFLOW
  /* Disable Rx flow control, i.e, assert RTS (active low). */

  tiva_gpiowrite(config->rts_gpio, false);
#endif

  return OK;
}

/****************************************************************************
 * Name: hciuart_interrupt
 *
 * Description:
 *   This is the UART interrupt callback.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call hciuart_copytorxbuffer
 *   or hciuart_copytotxfifo to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate btuart_lowerhalf_s structure in order to call these
 *   functions.
 *
 ****************************************************************************/

static int hciuart_interrupt(int irq, void *context, void *arg)
{
  const struct hciuart_config_s *config =
    (const struct hciuart_config_s *)arg;
  struct hciuart_state_s *state;
  uint32_t status;
  uint8_t handled;
  int  passes;

  DEBUGASSERT(config != NULL && config->state != NULL);
  state = config->state;

  /* Report serial activity to the power management logic */

#if defined(CONFIG_PM) && CONFIG_TIVA_PM_SERIAL_ACTIVITY > 0
  pm_activity(PM_IDLE_DOMAIN, CONFIG_TIVA_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = (HCIUART_RXHANDLED | HCIUART_TXHANDLED);
  for (passes = 0; passes < 256 && handled != 0; passes++)
    {
      handled = 0;

      /* Get the masked UART status word. */

      status = hciuart_getreg32(config, TIVA_UART_MIS_OFFSET);
      hciuart_putreg32(config, TIVA_UART_ICR_OFFSET, status);

      /* Handle incoming, receive bytes (non-DMA only) */

      if ((status & (UART_MIS_RXMIS | UART_MIS_RTMIS)) != 0 &&
          hciuart_rxenabled(config))
        {
          ssize_t nbytes;
#ifdef CONFIG_TIVA_HCIUART_SW_RXFLOW
          /* Set CTS high if FIFO 7/8 full is triggered */

          if ((status & (UART_MIS_RXMIS)) != 0)
            {
              tiva_gpiowrite(config->rts_gpio, true);
            }
#endif

          /* Received data ready... copy data from the Rx FIFO to the Rx
           * buffer.
           */

          nbytes = hciuart_copytorxbuffer(config);
          UNUSED(nbytes);

#ifdef CONFIG_TIVA_HCIUART_SW_RXFLOW
          /* Assert CTS because we now have space for more data */

          if ((status & (UART_MIS_RXMIS)) != 0)
            {
              tiva_gpiowrite(config->rts_gpio, false);
            }
#endif

          /* Is there anything in the Rx buffer?  Has the user registered an
           * Rx callback function?
           */

          if (state->rxhead != state->rxtail && state->callback != NULL)
            {
              state->callback(&config->lower, state->arg);
              handled = HCIUART_RXHANDLED;
            }
        }

      /* Handle outgoing, transmit bytes
       *
       * Transmit data register empty
       *   This bit is set by hardware when the content of the transmit data
       *   register has been transferred into the shift register.
       */

      if ((status & UART_MIS_TXMIS) != 0)
        {
          ssize_t nbytes;
          uint8_t txhandled;

          /* Transmit data register empty ... copy data from the Tx buffer
           * to the Tx FIFO.
           */

          nbytes = hciuart_copytotxfifo(config);
          UNUSED(nbytes);

          /* If the Tx buffer is now empty, then disable further Tx
           * interrupts.  Tx interrupts will only be enabled in the
           * following circumstances:
           *
           * 1. The user is waiting in hciuart_write() for space to become
           *    available in the Tx FIFO.
           * 2. The full, outgoing message has been placed into the Tx buffer
           *    by hciuart_write().
           *
           * In either case, no more Tx interrupts will be needed until more
           * data is added to the Tx buffer.
           */

          txhandled = HCIUART_TXHANDLED;
          if (state->txhead == state->txtail)
            {
              /* Disable Tx interrupts and treat the event as unhandled in
               * order to terminate looping.
               */

              hciuart_disableints(config, UART_IM_TXIM);
              txhandled = 0;
            }

          /* This copy will free up space in the Tx FIFO.  Wake up any
           * threads that may have been waiting for space in the Tx
           * buffer.
           */

          if (state->txwaiting)
            {
              state->txwaiting = false;
              nxsem_post(&state->txwait);
            }

          handled |= txhandled;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: hciuart_rxattach
 *
 * Description:
 *   Attach/detach the upper half Rx callback.
 *
 *   rxattach() allows the upper half logic to attach a callback function
 *   that will be used to inform the upper half that an Rx frame is
 *   available.  This callback will, most likely, be invoked in the
 *   context of an interrupt callback.  The receive() method should then
 *   be invoked in order to receive the obtain the Rx frame data.
 *
 ****************************************************************************/

static void hciuart_rxattach(const struct btuart_lowerhalf_s *lower,
                             btuart_rxcallback_t callback, void *arg)
{
  const struct hciuart_config_s *config =
    (const struct hciuart_config_s *)lower;
  struct hciuart_state_s *state;
  irqstate_t flags;

  wlinfo("config %p callback %p arg %p\n", config, callback, arg);

  DEBUGASSERT(config != NULL && config->state != NULL);
  state = config->state;

  /* If the callback is NULL, then we are detaching */

  flags = spin_lock_irqsave(NULL);
  if (callback == NULL)
    {
      uint32_t intset;

      /* Disable Rx callbacks and detach the Rx callback */

      intset = UART_IM_RXIM | UART_IM_RTIM;
      hciuart_disableints(config, intset);

      state->callback = NULL;
      state->arg      = NULL;
    }

  /* Otherwise, we are attaching */

  else
    {
      state->arg      = arg;
      state->callback = callback;
    }

  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: hciuart_rxenable
 *
 * Description:
 *   Enable/disable RX callbacks from the HCI UART.
 *
 *   hciuart_rxenable() may be used to enable or disable callback events.
 *   This probably translates to enabling and disabled Rx interrupts at
 *   the UART.  NOTE:  Rx event notification should be done sparingly:
 *   Rx data overrun may occur when Rx events are disabled!
 *
 ****************************************************************************/

static void hciuart_rxenable(const struct btuart_lowerhalf_s *lower,
                             bool enable)
{
  const struct hciuart_config_s *config =
    (const struct hciuart_config_s *)lower;

  DEBUGASSERT(config != NULL && config->state != NULL);

    {
      uint32_t intset;
      irqstate_t flags;

      flags = spin_lock_irqsave(NULL);
      if (enable)
        {
          /* Receive an interrupt when their is anything in the Rx data
           * register (or an Rx timeout occurs).
           */

          intset = UART_IM_RXIM | UART_IM_RTIM;
          hciuart_enableints(config, intset);
        }
      else
        {
          intset = UART_IM_RXIM | UART_IM_RTIM;
          hciuart_disableints(config, intset);
        }

      spin_unlock_irqrestore(NULL, flags);
    }
}

/****************************************************************************
 * Name: hciuart_setbaud
 *
 * Description:
 *   The HCI UART comes up with some initial BAUD rate.  Some support
 *   auto-BAUD detection, some support writing a configuration file to
 *   select the initial BAUD.  The simplest strategy, however, is simply
 *   to use the HCI UART's default initial BAUD to perform the basic
 *   bring up, then send a vendor-specific command to increase the HCI
 *   UARTs BAUD.  This method then may be used to adjust the lower half
 *   driver to the new HCI UART BAUD.
 *
 ****************************************************************************/

static int hciuart_setbaud(const struct btuart_lowerhalf_s *lower,
                           uint32_t baud)
{
  const struct hciuart_config_s *config =
    (const struct hciuart_config_s *)lower;

  DEBUGASSERT(config != NULL && config->state != NULL);

  config->state->baud = baud;
  hciuart_line_configure(config);
  return OK;
}

/****************************************************************************
 * Name: hciuart_read
 *
 * Description:
 *   Read UART data.
 *
 *   hciuart_read() after receipt of a callback notifying the upper half of
 *   the availability of Rx frame, the upper half may call the receive()
 *   method in order to obtain the buffered Rx frame data.
 *
 ****************************************************************************/

static ssize_t hciuart_read(const struct btuart_lowerhalf_s *lower,
                            void *buffer, size_t buflen)
{
  const struct hciuart_config_s *config =
    (const struct hciuart_config_s *)lower;
  struct hciuart_state_s *state;
  uint8_t *dest;
  size_t remaining;
  ssize_t ntotal;
  ssize_t nbytes;
  bool rxenable;
  int ret;

  wlinfo("config %p buffer %p buflen %lu\n",
         config, buffer, (unsigned long)buflen);

  /* NOTE:  This assumes that the caller has exclusive access to the Rx
   * buffer, i.e., one lower half instance can server only one upper half!
   */

  DEBUGASSERT(config != NULL && config->state != NULL);
  state = config->state;

  /* Read any pending data to the Rx buffer */

  nbytes = hciuart_copytorxbuffer(config);
  UNUSED(nbytes);

  /* Loop copying data to the user buffer while the Rx buffer is not empty
   * and the callers buffer is not full.
   */

  dest      = (uint8_t *)buffer;
  remaining = buflen;
  ntotal    = 0;

  rxenable  = hciuart_rxenabled(config);
  hciuart_rxenable(lower, false);

  while (state->rxtail != state->rxhead && ntotal < buflen)
    {
      nbytes = hciuart_copyfromrxbuffer(config, dest, remaining);
      if (nbytes <= 0)
        {
          DEBUGASSERT(nbytes == 0);

          /* If no data has been received, then we must wait for the arrival
           * of new Rx data and try again.
           */

          if (ntotal == 0)
            {
              DEBUGASSERT(!state->rxwaiting);
              state->rxwaiting = true;
              do
                {
                  ret = nxsem_wait_uninterruptible(&state->rxwait);
                  if (ret < 0)
                    {
                      ntotal = (ssize_t)ret;
                      break;
                    }
                }
              while (state->rxwaiting);
            }

          /* Otherwise, this must be the end of the packet.  Just break out
           * and return what we have.
           */

          else
            {
              break;
            }
        }
      else
        {
          /* More data has been copied.  Update pointers, counts, and
           * indices.
           */

          ntotal    += nbytes;
          dest      += nbytes;
          remaining -= nbytes;

          /* Read any additional pending data into the Rx buffer that may
           * have accumulated while we were copying.
           */

          nbytes = hciuart_copytorxbuffer(config);
          if (nbytes < 0)
            {
              /* An error occurred.. this should not really happen */

              return nbytes;
            }

          /* Otherwise, continue looping */
        }
    }

  hciuart_rxenable(lower, rxenable);
  return ntotal;
}

/****************************************************************************
 * Name: hciuart_write
 *
 * Description:
 *   Write UART data.
 *
 *   hciuart_write() will add the outgoing frame to the Tx buffer and will
 *   return immediately.  This function may block only in the event that
 *   there is insufficient buffer space to hold the Tx frame data.  In that
 *   case the lower half will block until there is sufficient to buffer
 *   the entire outgoing packet.
 *
 ****************************************************************************/

static ssize_t hciuart_write(const struct btuart_lowerhalf_s *lower,
                             const void *buffer, size_t buflen)
{
  const struct hciuart_config_s *config =
    (const struct hciuart_config_s *)lower;
  struct hciuart_state_s *state;
  const uint8_t *src;
  ssize_t nbytes = 0;
  uint16_t txhead;
  uint16_t txtail;
  uint16_t txnext;
  ssize_t ntotal;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(config != NULL && config->state != NULL);
  state = config->state;

  /* NOTE:  This assumes that the caller has exclusive access to the Tx
   * buffer, i.e., one lower half instance can serve only one upper half!
   */

  /* Make sure that the Tx Interrupts are disabled. */

  flags = spin_lock_irqsave(NULL);
  hciuart_disableints(config, UART_IM_TXIM);
  spin_unlock_irqrestore(NULL, flags);

  /* Loop until all of the user data have been moved to the Tx buffer */

  src    = buffer;
  ntotal = 0;

  while (ntotal < (ssize_t)buflen)
    {
      /* Copy bytes to the tail of the Tx buffer */

      /* Get a copy of the rxhead and rxtail indices of the Tx buffer */

      txhead = state->txhead;
      txtail = state->txtail;

      txnext = txtail + 1;
      if (txnext >= config->txbufsize)
        {
          txnext = 0;
        }

      /* Is there space available in the Tx buffer? Do have more bytes to
       * copy?
       */

      while (txhead != txnext && ntotal < (ssize_t)buflen)
        {
          /* Yes.. copy one byte to the Tx buffer */

          config->txbuffer[txtail] = *src++;

          txtail = txnext;
          if (++txnext >= config->txbufsize)
            {
              txnext = 0;
            }

          ntotal++;
        }

      /* Save the updated Tx buffer tail index */

      state->txtail = txtail;

      /* Copy bytes from the Tx buffer to the Tx FIFO */

      nbytes = hciuart_copytotxfifo(config);

      /* If nothing could be copied to the Tx FIFO and we still have user
       * data that we have not added to the Tx buffer, then we must wait for
       * space in the Tx* buffer then try again.
       */

      if (nbytes <= 0 && ntotal < buflen)
        {
          DEBUGASSERT(nbytes == 0);

          /* Enable the Tx interrupt and wait for space open up in the Tx
           * buffer.
           */

          flags = enter_critical_section();
          hciuart_enableints(config, UART_IM_TXIM);

          DEBUGASSERT(!state->txwaiting);
          state->txwaiting = true;
          do
            {
              ret = nxsem_wait_uninterruptible(&state->txwait);
              if (ret < 0)
                {
                  if (ntotal == 0)
                    {
                      ntotal = (ssize_t)ret;
                    }

                  break;
                }
            }
          while (state->txwaiting);

          /* Disable Tx interrupts again */

          hciuart_disableints(config, UART_IM_TXIM);
          leave_critical_section(flags);
        }
    }

  /* If the Tx buffer is not empty, then exit with the Tx interrupts
   * enabled.
   */

  if (state->txhead != state->txtail)
    {
      flags = spin_lock_irqsave(NULL);
      hciuart_enableints(config, UART_IM_TXIM);
      spin_unlock_irqrestore(NULL, flags);
    }

  return ntotal;
}

/****************************************************************************
 * Name: hciuart_rxdrain
 *
 * Description:
 *   Flush/drain all buffered RX data
 *
 ****************************************************************************/

static ssize_t hciuart_rxdrain(const struct btuart_lowerhalf_s *lower)
{
  const struct hciuart_config_s *config =
    (const struct hciuart_config_s *)lower;
  struct hciuart_state_s *state;
  size_t ntotal;
  ssize_t nbytes;
  bool rxenable;

  DEBUGASSERT(config != NULL && config->state != NULL);
  state = config->state;

  /* Read any pending data to the Rx buffer */

  nbytes = hciuart_copytorxbuffer(config);
  UNUSED(nbytes);

  /* Loop discarding in the Rx buffer until the Rx buffer is empty */

  ntotal    = 0;

  rxenable  = hciuart_rxenabled(config);
  hciuart_rxenable(lower, false);

  while (state->rxtail != state->rxhead)
    {
      /* Keep track of how much is discarded */

      ntotal += hciuart_rxinuse(config);

      /* Discard the data in the Rx buffer */

      state->rxhead = 0;
      state->rxtail = 0;

      /* Read any additional pending data into the Rx buffer that may
       * have accumulated while we were discarding.
       */

      nbytes = hciuart_copytorxbuffer(config);
      UNUSED(nbytes);
    }

  hciuart_rxenable(lower, rxenable);
  return ntotal;
}

/****************************************************************************
 * Name: hciuart_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void hciuart_pm_notify(struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Logic for PM_NORMAL goes here */
        }
        break;

      case(PM_IDLE):
        {
          /* Logic for PM_IDLE goes here */
        }
        break;

      case(PM_STANDBY):
        {
          /* Logic for PM_STANDBY goes here */
        }
        break;

      case(PM_SLEEP):
        {
          /* Logic for PM_SLEEP goes here */
        }
        break;

      default:

        /* Should not get here */

        break;
    }
}
#endif

/****************************************************************************
 * Name: hciuart_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int hciuart_pm_prepare(struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hciuart_instantiate
 *
 * Description:
 *   Obtain an instance of the HCI UART interface for the specified HCI UART
 *   This assumes that hciuart_initialize was called previously.
 *
 * Input Parameters:
 *   uart - Identifies the HCI UART to be configured
 *
 * Returned Value:
 *   On success, a reference to the HCI UART lower driver for the associated
 *   UART
 *
 ****************************************************************************/

const struct btuart_lowerhalf_s *
hciuart_instantiate(enum hciuart_devno_e uart)
{
  const struct hciuart_config_s *config;
#ifdef CONFIG_PM
  int ret;
#endif

  wlinfo("Instantiating HCIUART%d\n", (int)uart + 1);
  DEBUGASSERT((int)uart >= 0 && (int)uart < 8);

  /* Check if this uart is available in the configuration */

  config = g_hciuarts[(int)uart];
  if (config == NULL)
    {
      wlerr("ERROR: UART%d not configured\n", uart + 1);
      return NULL;
    }

  /* Register to receive power management callbacks */

#ifdef CONFIG_PM
  ret = pm_register(&g_serialcb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

  /* Configure and enable the UART */

  hciuart_configure(config);
  return &config->lower;
}

/****************************************************************************
 * Name: hciuart_initialize
 *
 * Description:
 *   Performs the low-level, one-time UART initialization.  This must be
 *   called before hciuart_instantiate.
 *
 ****************************************************************************/

void hciuart_initialize(void)
{
  const struct hciuart_config_s *config;
  struct hciuart_state_s *state;
  int ret;
  int i;

  /* Configure all UARTs */

  for (i = 0; i < sizeof(g_hciuarts) / sizeof(g_hciuarts[0]); i++)
    {
      config = g_hciuarts[i];
      if (config != NULL)
        {
          state = config->state;

          /* Enable UART clock */

          tiva_uart_enableclk(config->id);
          tiva_uart_enablepwr(config->id);

          /* Disable UART interrupts */

          hciuart_disableints(config, HCIUART_ALLINTS);

          /* Initialize signalling semaphores */

          nxsem_init(&state->rxwait, 0, 0);
          nxsem_init(&state->txwait, 0, 0);

          /* Attach and enable the HCI UART IRQ */

          ret = irq_attach(config->irq, hciuart_interrupt, (void *)config);
          if (ret == OK)
            {
              /* Enable the interrupt (RX and TX interrupts are still
               * disabled in the UART)
               */

              up_enable_irq(config->irq);
            }
        }
    }
}
