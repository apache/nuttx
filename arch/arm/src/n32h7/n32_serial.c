/****************************************************************************
 * arch/arm/src/n32h7/n32_serial.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>
#include <nuttx/power/pm.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include "arm_internal.h"
#include "chip.h"
#include "n32_gpio.h"
#include "hardware/n32h7_uart.h"
#include "hardware/n32h76x_pinmap.h"
#include "n32_rcc.h"
#include "n32_dma.h"
#include "n32_uart.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifdef HAVE_UART

/* DMA Support */
#ifdef CONFIG_ARCH_DMA
#  define SERIAL_HAVE_DMA 1
#else
#  undef CONFIG_USART1_RXDMA
#  undef CONFIG_USART1_TXDMA
#  undef CONFIG_USART2_RXDMA
#  undef CONFIG_USART2_TXDMA
#  undef CONFIG_USART3_RXDMA
#  undef CONFIG_USART3_TXDMA
#  undef CONFIG_USART4_RXDMA
#  undef CONFIG_USART4_TXDMA
#  undef CONFIG_USART5_RXDMA
#  undef CONFIG_USART5_TXDMA
#  undef CONFIG_USART6_RXDMA
#  undef CONFIG_USART6_TXDMA
#  undef CONFIG_USART7_RXDMA
#  undef CONFIG_USART7_TXDMA
#  undef CONFIG_USART8_RXDMA
#  undef CONFIG_USART8_TXDMA
#  undef CONFIG_UART9_RXDMA
#  undef CONFIG_UART9_TXDMA
#  undef CONFIG_UART10_RXDMA
#  undef CONFIG_UART10_TXDMA
#  undef CONFIG_UART11_RXDMA
#  undef CONFIG_UART11_TXDMA
#  undef CONFIG_UART12_RXDMA
#  undef CONFIG_UART12_TXDMA
#  undef CONFIG_UART13_RXDMA
#  undef CONFIG_UART13_TXDMA
#  undef CONFIG_UART14_RXDMA
#  undef CONFIG_UART14_TXDMA
#  undef CONFIG_UART15_RXDMA
#  undef CONFIG_UART15_TXDMA
#endif

/* Detect which UARTs have DMA enabled */
#undef SERIAL_HAVE_RXDMA
#if defined(CONFIG_USART1_RXDMA) || defined(CONFIG_USART2_RXDMA) || \
    defined(CONFIG_USART3_RXDMA) || defined(CONFIG_USART4_RXDMA) || \
    defined(CONFIG_USART5_RXDMA) || defined(CONFIG_USART6_RXDMA) || \
    defined(CONFIG_USART7_RXDMA) || defined(CONFIG_USART8_RXDMA) || \
    defined(CONFIG_UART9_RXDMA)  || defined(CONFIG_UART10_RXDMA) || \
    defined(CONFIG_UART11_RXDMA) || defined(CONFIG_UART12_RXDMA) || \
    defined(CONFIG_UART13_RXDMA) || defined(CONFIG_UART14_RXDMA) || \
    defined(CONFIG_UART15_RXDMA)
#  define SERIAL_HAVE_RXDMA 1
n32_dmacfg_t rxcfg =
{
    .src_addr    = (uint32_t)NULL,
    .dst_addr    = (uint32_t)NULL,
    .block_size  = 0,
    .ctrl        = (DMA_CHCTRL_INTEN                    |
                    DMA_CHCTRL_DTW(DMA_WIDTH_8BITS)     |
                    DMA_CHCTRL_STW(DMA_WIDTH_8BITS)     |
                    DMA_CHCTRL_DINC(DMA_ADDR_INC)       |
                    DMA_CHCTRL_SINC(DMA_ADDR_NOCHANGE)  |
                    DMA_CHCTRL_DSTMSIZE(DMA_BURST_1) |
                    DMA_CHCTRL_SRCMSIZE(DMA_BURST_1) |
                    DMA_CHCTRL_SMS(DMA_MS_1) |
                    DMA_CHCTRL_DMS(DMA_MS_1) |
                    DMA_CHCTRL_TTFC(DMA_TTFC_PER_TO_MEM_DMA)),
    .sg_cfg      = 0,
    .ds_cfg      = 0,
    .src_hs_if   = 0,
    .dst_hs_if   = 0,
    .priority    = 7,
    .src_hs_mode = 0,   /* HW handshake */
    .dst_hs_mode = 0,
    .link_list   = NULL,
};
#endif

#undef SERIAL_HAVE_TXDMA
#if defined(CONFIG_USART1_TXDMA) || defined(CONFIG_USART2_TXDMA) || \
    defined(CONFIG_USART3_TXDMA) || defined(CONFIG_USART4_TXDMA) || \
    defined(CONFIG_USART5_TXDMA) || defined(CONFIG_USART6_TXDMA) || \
    defined(CONFIG_USART7_TXDMA) || defined(CONFIG_USART8_TXDMA) || \
    defined(CONFIG_UART9_TXDMA)  || defined(CONFIG_UART10_TXDMA) || \
    defined(CONFIG_UART11_TXDMA) || defined(CONFIG_UART12_TXDMA) || \
    defined(CONFIG_UART13_TXDMA) || defined(CONFIG_UART14_TXDMA) || \
    defined(CONFIG_UART15_TXDMA)
#  define SERIAL_HAVE_TXDMA 1
n32_dmacfg_t txcfg =
{
    .src_addr    = (uint32_t)NULL,
    .dst_addr    = (uint32_t)NULL,
    .block_size  = 0,
    .ctrl        = (DMA_CHCTRL_INTEN                    |
                    DMA_CHCTRL_DTW(DMA_WIDTH_8BITS)     |
                    DMA_CHCTRL_STW(DMA_WIDTH_8BITS)     |
                    DMA_CHCTRL_DINC(DMA_ADDR_NOCHANGE)  |
                    DMA_CHCTRL_SINC(DMA_ADDR_INC)       |
                    DMA_CHCTRL_DSTMSIZE(DMA_BURST_1) |
                    DMA_CHCTRL_SRCMSIZE(DMA_BURST_1) |
                    DMA_CHCTRL_SMS(DMA_MS_1) |
                    DMA_CHCTRL_DMS(DMA_MS_1) |
                    DMA_CHCTRL_TTFC(DMA_TTFC_MEM_TO_PER_DMA)),
    .sg_cfg      = 0,
    .ds_cfg      = 0,
    .src_hs_if   = 0,
    .dst_hs_if   = 0,
    .priority    = 7,
    .src_hs_mode = 0,   /* HW handshake */
    .dst_hs_mode = 0,
    .link_list   = NULL,
};

#endif

#undef SERIAL_HAVE_RXDMA_TXDMA
#if defined(SERIAL_HAVE_RXDMA) && defined(SERIAL_HAVE_TXDMA)
#  define SERIAL_HAVE_RXDMA_TXDMA 1
#endif

/* Interrupt masks for up_setusartint */
#define USART_CTRL1_USED_INTS \
    (USART_CTRL1_RXDNEIEN | USART_CTRL1_TXDEIEN | USART_CTRL1_IDLEIEN | \
     USART_CTRL1_PEIEN | USART_CTRL1_TXCIEN)
#define USART_CTRL3_USED_INTS \
    (USART_CTRL3_CTSIEN | USART_CTRL3_ERRIEN)

/* Power management activity count */
#ifndef CONFIG_N32H7_PM_SERIAL_ACTIVITY
#  define CONFIG_N32H7_PM_SERIAL_ACTIVITY 10
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  struct uart_dev_s dev;       /* Generic UART device */
  uint16_t          ie;        /* Saved interrupt enable bits */
  uint16_t          sr;        /* Saved status (for ISR) */
  bool              initialized;

#ifdef CONFIG_PM
  bool              suspended; /* UART suspended for PM */
  uint16_t          suspended_ie;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
  uint8_t           parity;
  uint8_t           bits;
  bool              stopbits2;
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool              iflow;
#  endif
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool              oflow;
#  endif
  uint32_t          baud;
#else
  const uint8_t     parity;
  const uint8_t     bits;
  const bool        stopbits2;
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
  const bool        iflow;
#  endif
#  ifdef CONFIG_SERIAL_OFLOWCONTROL
  const bool        oflow;
#  endif
  const uint32_t    baud;
#endif

  const uint8_t     irq;
  const uint32_t    apbclock;
  const uint32_t    usartbase;
  const uint32_t    tx_gpio;
  const uint32_t    rx_gpio;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  const uint32_t    rts_gpio;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t    cts_gpio;
#endif

#ifdef SERIAL_HAVE_TXDMA
  const unsigned int txdma_channel;
  DMA_HANDLE         txdma;
#endif
#ifdef SERIAL_HAVE_RXDMA
  const unsigned int rxdma_channel;
  DMA_HANDLE         rxdma;
#  ifdef CONFIG_PM
  bool               rxdmasusp;
#  endif
#endif

  spinlock_t         lock;
};

#ifdef CONFIG_PM
struct pm_config_s
{
  struct pm_callback_s pm_cb;
  bool serial_suspended;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void up_set_format(struct uart_dev_s *dev);
static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);

#if !defined(SERIAL_HAVE_ONLY_RXDMA)
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
#endif
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev, unsigned int nbuffered,
                             bool upper);
#endif
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_send(struct uart_dev_s *dev);
static void up_dma_txint(struct uart_dev_s *dev, bool enable);
static void up_dma_txavailable(struct uart_dev_s *dev);
static void up_dma_txcallback(DMA_HANDLE handle, uint8_t status, void *arg);
#endif

#ifdef SERIAL_HAVE_RXDMA
static void up_dma_receive(struct uart_dev_s *dev);
static void up_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool up_dma_rxavailable(struct uart_dev_s *dev);
static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status, void *arg);
#endif

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int  up_dma_setup(struct uart_dev_s *dev);
static void up_dma_shutdown(struct uart_dev_s *dev);
#endif

#ifdef CONFIG_PM
static void up_pm_setsuspend(bool suspend);
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Ops tables: Different combinations of DMA usage */
#ifdef SERIAL_HAVE_RXDMA
static const struct uart_ops_s g_uart_rxdma_ops =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .dmareceive     = up_dma_receive,
  .rxint          = up_dma_rxint,
  .rxavailable    = up_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};
#endif

#ifdef SERIAL_HAVE_TXDMA
static const struct uart_ops_s g_uart_txdma_ops =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_dma_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
  .dmatxavail     = up_dma_txavailable,
  .dmasend        = up_dma_send,
};
#endif

#ifdef SERIAL_HAVE_RXDMA_TXDMA
static const struct uart_ops_s g_uart_rxtxdma_ops =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .dmareceive     = up_dma_receive,
  .rxint          = up_dma_rxint,
  .rxavailable    = up_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_dma_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
  .dmatxavail     = up_dma_txavailable,
  .dmasend        = up_dma_send,
};
#endif

/* Non-DMA ops */

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
  .rxflowcontrol  = up_rxflowcontrol,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* --------------------------------------------------------------------------
 * OPS macro definitions (choose correct ops based on DMA config)
 * --------------------------------------------------------------------------
 */
#if defined(CONFIG_USART1_RXDMA) && defined(CONFIG_USART1_TXDMA)
#  define UART_USART1_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_USART1_RXDMA)
#  define UART_USART1_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_USART1_TXDMA)
#  define UART_USART1_OPS &g_uart_txdma_ops
#else
#  define UART_USART1_OPS &g_uart_ops
#endif
#if defined(CONFIG_USART2_RXDMA) && defined(CONFIG_USART2_TXDMA)
#  define UART_USART2_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_USART2_RXDMA)
#  define UART_USART2_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_USART2_TXDMA)
#  define UART_USART2_OPS &g_uart_txdma_ops
#else
#  define UART_USART2_OPS &g_uart_ops
#endif
#if defined(CONFIG_USART3_RXDMA) && defined(CONFIG_USART3_TXDMA)
#  define UART_USART3_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_USART3_RXDMA)
#  define UART_USART3_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_USART3_TXDMA)
#  define UART_USART3_OPS &g_uart_txdma_ops
#else
#  define UART_USART3_OPS &g_uart_ops
#endif
#if defined(CONFIG_USART4_RXDMA) && defined(CONFIG_USART4_TXDMA)
#  define UART_USART4_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_USART4_RXDMA)
#  define UART_USART4_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_USART4_TXDMA)
#  define UART_USART4_OPS &g_uart_txdma_ops
#else
#  define UART_USART4_OPS &g_uart_ops
#endif
#if defined(CONFIG_USART5_RXDMA) && defined(CONFIG_USART5_TXDMA)
#  define UART_USART5_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_USART5_RXDMA)
#  define UART_USART5_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_USART5_TXDMA)
#  define UART_USART5_OPS &g_uart_txdma_ops
#else
#  define UART_USART5_OPS &g_uart_ops
#endif
#if defined(CONFIG_USART6_RXDMA) && defined(CONFIG_USART6_TXDMA)
#  define UART_USART6_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_USART6_RXDMA)
#  define UART_USART6_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_USART6_TXDMA)
#  define UART_USART6_OPS &g_uart_txdma_ops
#else
#  define UART_USART6_OPS &g_uart_ops
#endif
#if defined(CONFIG_USART7_RXDMA) && defined(CONFIG_USART7_TXDMA)
#  define UART_USART7_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_USART7_RXDMA)
#  define UART_USART7_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_USART7_TXDMA)
#  define UART_USART7_OPS &g_uart_txdma_ops
#else
#  define UART_USART7_OPS &g_uart_ops
#endif
#if defined(CONFIG_USART8_RXDMA) && defined(CONFIG_USART8_TXDMA)
#  define UART_USART8_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_USART8_RXDMA)
#  define UART_USART8_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_USART8_TXDMA)
#  define UART_USART8_OPS &g_uart_txdma_ops
#else
#  define UART_USART8_OPS &g_uart_ops
#endif
#if defined(CONFIG_UART9_RXDMA) && defined(CONFIG_UART9_TXDMA)
#  define UART_UART9_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_UART9_RXDMA)
#  define UART_UART9_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_UART9_TXDMA)
#  define UART_UART9_OPS &g_uart_txdma_ops
#else
#  define UART_UART9_OPS &g_uart_ops
#endif
#if defined(CONFIG_UART10_RXDMA) && defined(CONFIG_UART10_TXDMA)
#  define UART_UART10_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_UART10_RXDMA)
#  define UART_UART10_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_UART10_TXDMA)
#  define UART_UART10_OPS &g_uart_txdma_ops
#else
#  define UART_UART10_OPS &g_uart_ops
#endif
#if defined(CONFIG_UART11_RXDMA) && defined(CONFIG_UART11_TXDMA)
#  define UART_UART11_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_UART11_RXDMA)
#  define UART_UART11_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_UART11_TXDMA)
#  define UART_UART11_OPS &g_uart_txdma_ops
#else
#  define UART_UART11_OPS &g_uart_ops
#endif
#if defined(CONFIG_UART12_RXDMA) && defined(CONFIG_UART12_TXDMA)
#  define UART_UART12_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_UART12_RXDMA)
#  define UART_UART12_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_UART12_TXDMA)
#  define UART_UART12_OPS &g_uart_txdma_ops
#else
#  define UART_UART12_OPS &g_uart_ops
#endif
#if defined(CONFIG_UART13_RXDMA) && defined(CONFIG_UART13_TXDMA)
#  define UART_UART13_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_UART13_RXDMA)
#  define UART_UART13_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_UART13_TXDMA)
#  define UART_UART13_OPS &g_uart_txdma_ops
#else
#  define UART_UART13_OPS &g_uart_ops
#endif
#if defined(CONFIG_UART14_RXDMA) && defined(CONFIG_UART14_TXDMA)
#  define UART_UART14_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_UART14_RXDMA)
#  define UART_UART14_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_UART14_TXDMA)
#  define UART_UART14_OPS &g_uart_txdma_ops
#else
#  define UART_UART14_OPS &g_uart_ops
#endif
#if defined(CONFIG_UART15_RXDMA) && defined(CONFIG_UART15_TXDMA)
#  define UART_UART15_OPS &g_uart_rxtxdma_ops
#elif defined(CONFIG_UART15_RXDMA)
#  define UART_UART15_OPS &g_uart_rxdma_ops
#elif defined(CONFIG_UART15_TXDMA)
#  define UART_UART15_OPS &g_uart_txdma_ops
#else
#  define UART_UART15_OPS &g_uart_ops
#endif

/* --------------------------------------------------------------------------
 * Instance data (generated by script)
 * --------------------------------------------------------------------------
 */
#ifdef CONFIG_N32H7_USART1
static char locate_data(".axi_ram") g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_usart1priv =
{
  .dev =
  {
#if CONSOLE_UART == 1
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_usart1rxbuffer),
      .buffer = g_usart1rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_usart1txbuffer),
      .buffer = g_usart1txbuffer
    },
    .ops = UART_USART1_OPS,
    .priv = &g_usart1priv,
  },
  .irq           = N32_IRQ_USART1,
  .parity        = CONFIG_USART1_PARITY,
  .bits          = CONFIG_USART1_BITS,
  .stopbits2     = CONFIG_USART1_2STOP,
  .baud          = CONFIG_USART1_BAUD,
  .apbclock      = N32_AHB_FREQUENCY,
  .usartbase     = N32_USART1_BASE,
  .tx_gpio       = GPIO_USART1_TX,
  .rx_gpio       = GPIO_USART1_RX,
#ifdef CONFIG_USART1_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_USART1_RTS,
#endif
#ifdef CONFIG_USART1_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_USART1_CTS,
#endif
#ifdef CONFIG_USART1_TXDMA
  .txdma_channel = DMAMAP_USART1_TX,
#endif
#ifdef CONFIG_USART1_RXDMA
  .rxdma_channel = DMAMAP_USART1_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_USART1 */

#ifdef CONFIG_N32H7_USART2
static char locate_data(".axi_ram") g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_usart2priv =
{
  .dev =
  {
#if CONSOLE_UART == 2
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_usart2rxbuffer),
      .buffer = g_usart2rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_usart2txbuffer),
      .buffer = g_usart2txbuffer
    },
    .ops = UART_USART2_OPS,
    .priv = &g_usart2priv,
  },
  .irq           = N32_IRQ_USART2,
  .parity        = CONFIG_USART2_PARITY,
  .bits          = CONFIG_USART2_BITS,
  .stopbits2     = CONFIG_USART2_2STOP,
  .baud          = CONFIG_USART2_BAUD,
  .apbclock      = N32_AHB_FREQUENCY,
  .usartbase     = N32_USART2_BASE,
  .tx_gpio       = GPIO_USART2_TX,
  .rx_gpio       = GPIO_USART2_RX,
#ifdef CONFIG_USART2_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_USART2_RTS,
#endif
#ifdef CONFIG_USART2_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_USART2_CTS,
#endif
#ifdef CONFIG_USART2_TXDMA
  .txdma_channel = DMAMAP_USART2_TX,
#endif
#ifdef CONFIG_USART2_RXDMA
  .rxdma_channel = DMAMAP_USART2_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_USART2 */

#ifdef CONFIG_N32H7_USART3
static char locate_data(".axi_ram") g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_usart3priv =
{
  .dev =
  {
#if CONSOLE_UART == 3
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_usart3rxbuffer),
      .buffer = g_usart3rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_usart3txbuffer),
      .buffer = g_usart3txbuffer
    },
    .ops = UART_USART3_OPS,
    .priv = &g_usart3priv,
  },
  .irq           = N32_IRQ_USART3,
  .parity        = CONFIG_USART3_PARITY,
  .bits          = CONFIG_USART3_BITS,
  .stopbits2     = CONFIG_USART3_2STOP,
  .baud          = CONFIG_USART3_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_USART3_BASE,
  .tx_gpio       = GPIO_USART3_TX,
  .rx_gpio       = GPIO_USART3_RX,
#ifdef CONFIG_USART3_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_USART3_RTS,
#endif
#ifdef CONFIG_USART3_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_USART3_CTS,
#endif
#ifdef CONFIG_USART3_TXDMA
  .txdma_channel = DMAMAP_USART3_TX,
#endif
#ifdef CONFIG_USART3_RXDMA
  .rxdma_channel = DMAMAP_USART3_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_USART3 */

#ifdef CONFIG_N32H7_USART4
static char locate_data(".axi_ram") g_usart4rxbuffer[CONFIG_USART4_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_usart4txbuffer[CONFIG_USART4_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_usart4priv =
{
  .dev =
  {
#if CONSOLE_UART == 4
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_usart4rxbuffer),
      .buffer = g_usart4rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_usart4txbuffer),
      .buffer = g_usart4txbuffer
    },
    .ops = UART_USART4_OPS,
    .priv = &g_usart4priv,
  },
  .irq           = N32_IRQ_USART4,
  .parity        = CONFIG_USART4_PARITY,
  .bits          = CONFIG_USART4_BITS,
  .stopbits2     = CONFIG_USART4_2STOP,
  .baud          = CONFIG_USART4_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_USART4_BASE,
  .tx_gpio       = GPIO_USART4_TX,
  .rx_gpio       = GPIO_USART4_RX,
#ifdef CONFIG_USART4_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_USART4_RTS,
#endif
#ifdef CONFIG_USART4_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_USART4_CTS,
#endif
#ifdef CONFIG_USART4_TXDMA
  .txdma_channel = DMAMAP_USART4_TX,
#endif
#ifdef CONFIG_USART4_RXDMA
  .rxdma_channel = DMAMAP_USART4_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_USART4 */

#ifdef CONFIG_N32H7_USART5
static char locate_data(".axi_ram") g_usart5rxbuffer[CONFIG_USART5_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_usart5txbuffer[CONFIG_USART5_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_usart5priv =
{
  .dev =
  {
#if CONSOLE_UART == 5
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_usart5rxbuffer),
      .buffer = g_usart5rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_usart5txbuffer),
      .buffer = g_usart5txbuffer
    },
    .ops = UART_USART5_OPS,
    .priv = &g_usart5priv,
  },
  .irq           = N32_IRQ_USART5,
  .parity        = CONFIG_USART5_PARITY,
  .bits          = CONFIG_USART5_BITS,
  .stopbits2     = CONFIG_USART5_2STOP,
  .baud          = CONFIG_USART5_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_USART5_BASE,
  .tx_gpio       = GPIO_USART5_TX,
  .rx_gpio       = GPIO_USART5_RX,
#ifdef CONFIG_USART5_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_USART5_RTS,
#endif
#ifdef CONFIG_USART5_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_USART5_CTS,
#endif
#ifdef CONFIG_USART5_TXDMA
  .txdma_channel = DMAMAP_USART5_TX,
#endif
#ifdef CONFIG_USART5_RXDMA
  .rxdma_channel = DMAMAP_USART5_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_USART5 */

#ifdef CONFIG_N32H7_USART6
static char locate_data(".axi_ram") g_usart6rxbuffer[CONFIG_USART6_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_usart6txbuffer[CONFIG_USART6_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_usart6priv =
{
  .dev =
  {
#if CONSOLE_UART == 6
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_usart6rxbuffer),
      .buffer = g_usart6rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_usart6txbuffer),
      .buffer = g_usart6txbuffer
    },
    .ops = UART_USART6_OPS,
    .priv = &g_usart6priv,
  },
  .irq           = N32_IRQ_USART6,
  .parity        = CONFIG_USART6_PARITY,
  .bits          = CONFIG_USART6_BITS,
  .stopbits2     = CONFIG_USART6_2STOP,
  .baud          = CONFIG_USART6_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_USART6_BASE,
  .tx_gpio       = GPIO_USART6_TX,
  .rx_gpio       = GPIO_USART6_RX,
#ifdef CONFIG_USART6_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_USART6_RTS,
#endif
#ifdef CONFIG_USART6_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_USART6_CTS,
#endif
#ifdef CONFIG_USART6_TXDMA
  .txdma_channel = DMAMAP_USART6_TX,
#endif
#ifdef CONFIG_USART6_RXDMA
  .rxdma_channel = DMAMAP_USART6_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_USART6 */

#ifdef CONFIG_N32H7_USART7
static char locate_data(".axi_ram") g_usart7rxbuffer[CONFIG_USART7_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_usart7txbuffer[CONFIG_USART7_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_usart7priv =
{
  .dev =
  {
#if CONSOLE_UART == 7
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_usart7rxbuffer),
      .buffer = g_usart7rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_usart7txbuffer),
      .buffer = g_usart7txbuffer
    },
    .ops = UART_USART7_OPS,
    .priv = &g_usart7priv,
  },
  .irq           = N32_IRQ_USART7,
  .parity        = CONFIG_USART7_PARITY,
  .bits          = CONFIG_USART7_BITS,
  .stopbits2     = CONFIG_USART7_2STOP,
  .baud          = CONFIG_USART7_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_USART7_BASE,
  .tx_gpio       = GPIO_USART7_TX,
  .rx_gpio       = GPIO_USART7_RX,
#ifdef CONFIG_USART7_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_USART7_RTS,
#endif
#ifdef CONFIG_USART7_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_USART7_CTS,
#endif
#ifdef CONFIG_USART7_TXDMA
  .txdma_channel = DMAMAP_USART7_TX,
#endif
#ifdef CONFIG_USART7_RXDMA
  .rxdma_channel = DMAMAP_USART7_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_USART7 */

#ifdef CONFIG_N32H7_USART8
static char locate_data(".axi_ram") g_usart8rxbuffer[CONFIG_USART8_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_usart8txbuffer[CONFIG_USART8_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_usart8priv =
{
  .dev =
  {
#if CONSOLE_UART == 8
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_usart8rxbuffer),
      .buffer = g_usart8rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_usart8txbuffer),
      .buffer = g_usart8txbuffer
    },
    .ops = UART_USART8_OPS,
    .priv = &g_usart8priv,
  },
  .irq           = N32_IRQ_USART8,
  .parity        = CONFIG_USART8_PARITY,
  .bits          = CONFIG_USART8_BITS,
  .stopbits2     = CONFIG_USART8_2STOP,
  .baud          = CONFIG_USART8_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_USART8_BASE,
  .tx_gpio       = GPIO_USART8_TX,
  .rx_gpio       = GPIO_USART8_RX,
#ifdef CONFIG_USART8_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_USART8_RTS,
#endif
#ifdef CONFIG_USART8_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_USART8_CTS,
#endif
#ifdef CONFIG_USART8_TXDMA
  .txdma_channel = DMAMAP_USART8_TX,
#endif
#ifdef CONFIG_USART8_RXDMA
  .rxdma_channel = DMAMAP_USART8_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_USART8 */

#ifdef CONFIG_N32H7_UART9
static char locate_data(".axi_ram") g_uart9rxbuffer[CONFIG_UART9_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_uart9txbuffer[CONFIG_UART9_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_uart9priv =
{
  .dev =
  {
#if CONSOLE_UART == 9
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_uart9rxbuffer),
      .buffer = g_uart9rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_uart9txbuffer),
      .buffer = g_uart9txbuffer
    },
    .ops = UART_UART9_OPS,
    .priv = &g_uart9priv,
  },
  .irq           = N32_IRQ_UART9,
  .parity        = CONFIG_UART9_PARITY,
  .bits          = CONFIG_UART9_BITS,
  .stopbits2     = CONFIG_UART9_2STOP,
  .baud          = CONFIG_UART9_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_UART9_BASE,
  .tx_gpio       = GPIO_UART9_TX,
  .rx_gpio       = GPIO_UART9_RX,
#ifdef CONFIG_UART9_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_UART9_RTS,
#endif
#ifdef CONFIG_UART9_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_UART9_CTS,
#endif
#ifdef CONFIG_UART9_TXDMA
  .txdma_channel = DMAMAP_UART9_TX,
#endif
#ifdef CONFIG_UART9_RXDMA
  .rxdma_channel = DMAMAP_UART9_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_UART9 */

#ifdef CONFIG_N32H7_UART10
static char locate_data(".axi_ram") g_uart10rxbuffer[CONFIG_UART10_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_uart10txbuffer[CONFIG_UART10_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_uart10priv =
{
  .dev =
  {
#if CONSOLE_UART == 10
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_uart10rxbuffer),
      .buffer = g_uart10rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_uart10txbuffer),
      .buffer = g_uart10txbuffer
    },
    .ops = UART_UART10_OPS,
    .priv = &g_uart10priv,
  },
  .irq           = N32_IRQ_UART10,
  .parity        = CONFIG_UART10_PARITY,
  .bits          = CONFIG_UART10_BITS,
  .stopbits2     = CONFIG_UART10_2STOP,
  .baud          = CONFIG_UART10_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_UART10_BASE,
  .tx_gpio       = GPIO_UART10_TX,
  .rx_gpio       = GPIO_UART10_RX,
#ifdef CONFIG_UART10_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_UART10_RTS,
#endif
#ifdef CONFIG_UART10_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_UART10_CTS,
#endif
#ifdef CONFIG_UART10_TXDMA
  .txdma_channel = DMAMAP_UART10_TX,
#endif
#ifdef CONFIG_UART10_RXDMA
  .rxdma_channel = DMAMAP_UART10_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_UART10 */

#ifdef CONFIG_N32H7_UART11
static char locate_data(".axi_ram") g_uart11rxbuffer[CONFIG_UART11_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_uart11txbuffer[CONFIG_UART11_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_uart11priv =
{
  .dev =
  {
#if CONSOLE_UART == 11
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_uart11rxbuffer),
      .buffer = g_uart11rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_uart11txbuffer),
      .buffer = g_uart11txbuffer
    },
    .ops = UART_UART11_OPS,
    .priv = &g_uart11priv,
  },
  .irq           = N32_IRQ_UART11,
  .parity        = CONFIG_UART11_PARITY,
  .bits          = CONFIG_UART11_BITS,
  .stopbits2     = CONFIG_UART11_2STOP,
  .baud          = CONFIG_UART11_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_UART11_BASE,
  .tx_gpio       = GPIO_UART11_TX,
  .rx_gpio       = GPIO_UART11_RX,
#ifdef CONFIG_UART11_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_UART11_RTS,
#endif
#ifdef CONFIG_UART11_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_UART11_CTS,
#endif
#ifdef CONFIG_UART11_TXDMA
  .txdma_channel = DMAMAP_UART11_TX,
#endif
#ifdef CONFIG_UART11_RXDMA
  .rxdma_channel = DMAMAP_UART11_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_UART11 */

#ifdef CONFIG_N32H7_UART12
static char locate_data(".axi_ram") g_uart12rxbuffer[CONFIG_UART12_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_uart12txbuffer[CONFIG_UART12_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_uart12priv =
{
  .dev =
  {
#if CONSOLE_UART == 12
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_uart12rxbuffer),
      .buffer = g_uart12rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_uart12txbuffer),
      .buffer = g_uart12txbuffer
    },
    .ops = UART_UART12_OPS,
    .priv = &g_uart12priv,
  },
  .irq           = N32_IRQ_UART12,
  .parity        = CONFIG_UART12_PARITY,
  .bits          = CONFIG_UART12_BITS,
  .stopbits2     = CONFIG_UART12_2STOP,
  .baud          = CONFIG_UART12_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_UART12_BASE,
  .tx_gpio       = GPIO_UART12_TX,
  .rx_gpio       = GPIO_UART12_RX,
#ifdef CONFIG_UART12_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_UART12_RTS,
#endif
#ifdef CONFIG_UART12_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_UART12_CTS,
#endif
#ifdef CONFIG_UART12_TXDMA
  .txdma_channel = DMAMAP_UART12_TX,
#endif
#ifdef CONFIG_UART12_RXDMA
  .rxdma_channel = DMAMAP_UART12_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_UART12 */

#ifdef CONFIG_N32H7_UART13
static char locate_data(".axi_ram") g_uart13rxbuffer[CONFIG_UART13_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_uart13txbuffer[CONFIG_UART13_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_uart13priv =
{
  .dev =
  {
#if CONSOLE_UART == 13
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_uart13rxbuffer),
      .buffer = g_uart13rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_uart13txbuffer),
      .buffer = g_uart13txbuffer
    },
    .ops = UART_UART13_OPS,
    .priv = &g_uart13priv,
  },
  .irq           = N32_IRQ_UART13,
  .parity        = CONFIG_UART13_PARITY,
  .bits          = CONFIG_UART13_BITS,
  .stopbits2     = CONFIG_UART13_2STOP,
  .baud          = CONFIG_UART13_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_UART13_BASE,
  .tx_gpio       = GPIO_UART13_TX,
  .rx_gpio       = GPIO_UART13_RX,
#ifdef CONFIG_UART13_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_UART13_RTS,
#endif
#ifdef CONFIG_UART13_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_UART13_CTS,
#endif
#ifdef CONFIG_UART13_TXDMA
  .txdma_channel = DMAMAP_UART13_TX,
#endif
#ifdef CONFIG_UART13_RXDMA
  .rxdma_channel = DMAMAP_UART13_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_UART13 */

#ifdef CONFIG_N32H7_UART14
static char locate_data(".axi_ram") g_uart14rxbuffer[CONFIG_UART14_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_uart14txbuffer[CONFIG_UART14_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_uart14priv =
{
  .dev =
  {
#if CONSOLE_UART == 14
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_uart14rxbuffer),
      .buffer = g_uart14rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_uart14txbuffer),
      .buffer = g_uart14txbuffer
    },
    .ops = UART_UART14_OPS,
    .priv = &g_uart14priv,
  },
  .irq           = N32_IRQ_UART14,
  .parity        = CONFIG_UART14_PARITY,
  .bits          = CONFIG_UART14_BITS,
  .stopbits2     = CONFIG_UART14_2STOP,
  .baud          = CONFIG_UART14_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_UART14_BASE,
  .tx_gpio       = GPIO_UART14_TX,
  .rx_gpio       = GPIO_UART14_RX,
#ifdef CONFIG_UART14_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_UART14_RTS,
#endif
#ifdef CONFIG_UART14_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_UART14_CTS,
#endif
#ifdef CONFIG_UART14_TXDMA
  .txdma_channel = DMAMAP_UART14_TX,
#endif
#ifdef CONFIG_UART14_RXDMA
  .rxdma_channel = DMAMAP_UART14_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_UART14 */

#ifdef CONFIG_N32H7_UART15
static char locate_data(".axi_ram") g_uart15rxbuffer[CONFIG_UART15_RXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);
static char locate_data(".axi_ram") g_uart15txbuffer[CONFIG_UART15_TXBUFSIZE]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static struct up_dev_s g_uart15priv =
{
  .dev =
  {
#if CONSOLE_UART == 15
    .isconsole = true,
#endif
    .recv =
    {
      .size = sizeof(g_uart15rxbuffer),
      .buffer = g_uart15rxbuffer
    },
    .xmit =
    {
      .size = sizeof(g_uart15txbuffer),
      .buffer = g_uart15txbuffer
    },
    .ops = UART_UART15_OPS,
    .priv = &g_uart15priv,
  },
  .irq           = N32_IRQ_UART15,
  .parity        = CONFIG_UART15_PARITY,
  .bits          = CONFIG_UART15_BITS,
  .stopbits2     = CONFIG_UART15_2STOP,
  .baud          = CONFIG_UART15_BAUD,
  .apbclock      = N32_APB_FREQUENCY,
  .usartbase     = N32_UART15_BASE,
  .tx_gpio       = GPIO_UART15_TX,
  .rx_gpio       = GPIO_UART15_RX,
#ifdef CONFIG_UART15_IFLOWCONTROL
  .iflow         = true,
  .rts_gpio      = GPIO_UART15_RTS,
#endif
#ifdef CONFIG_UART15_OFLOWCONTROL
  .oflow         = true,
  .cts_gpio      = GPIO_UART15_CTS,
#endif
#ifdef CONFIG_UART15_TXDMA
  .txdma_channel = DMAMAP_UART15_TX,
#endif
#ifdef CONFIG_UART15_RXDMA
  .rxdma_channel = DMAMAP_UART15_RX,
#endif
  .lock = SP_UNLOCKED,
};
#endif /* CONFIG_N32H7_UART15 */

/* Device pointer array */

static struct up_dev_s * const g_uart_devs[] =
{
#ifdef CONFIG_N32H7_USART1
  [0] = &g_usart1priv,
#endif
#ifdef CONFIG_N32H7_USART2
  [1] = &g_usart2priv,
#endif
#ifdef CONFIG_N32H7_USART3
  [2] = &g_usart3priv,
#endif
#ifdef CONFIG_N32H7_USART4
  [3] = &g_usart4priv,
#endif
#ifdef CONFIG_N32H7_USART5
  [4] = &g_usart5priv,
#endif
#ifdef CONFIG_N32H7_USART6
  [5] = &g_usart6priv,
#endif
#ifdef CONFIG_N32H7_USART7
  [6] = &g_usart7priv,
#endif
#ifdef CONFIG_N32H7_USART8
  [7] = &g_usart8priv,
#endif
#ifdef CONFIG_N32H7_UART9
  [8] = &g_uart9priv,
#endif
#ifdef CONFIG_N32H7_UART10
  [9] = &g_uart10priv,
#endif
#ifdef CONFIG_N32H7_UART11
  [10] = &g_uart11priv,
#endif
#ifdef CONFIG_N32H7_UART12
  [11] = &g_uart12priv,
#endif
#ifdef CONFIG_N32H7_UART13
  [12] = &g_uart13priv,
#endif
#ifdef CONFIG_N32H7_UART14
  [13] = &g_uart14priv,
#endif
#ifdef CONFIG_N32H7_UART15
  [14] = &g_uart15priv,
#endif
};

#ifdef CONFIG_PM
static struct pm_config_s g_serialpm =
{
  .pm_cb.notify     = up_pm_notify,
  .pm_cb.prepare    = up_pm_prepare,
  .serial_suspended = false
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->usartbase + offset);
}

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_setusartint
 ****************************************************************************/

static inline void up_setusartint(struct up_dev_s *priv, uint16_t ie)
{
  uint32_t cr1;
  uint32_t cr3;
  irqstate_t flags = spin_lock_irqsave(&priv->lock);

  priv->ie = ie;

  cr1 = up_serialin(priv, N32_USART_CTRL1_OFFSET);
  cr1 &= ~USART_CTRL1_USED_INTS;
  cr1 |= (ie & USART_CTRL1_USED_INTS);
  up_serialout(priv, N32_USART_CTRL1_OFFSET, cr1);

  cr3 = up_serialin(priv, N32_USART_CTRL3_OFFSET);
  cr3 &= ~USART_CTRL3_USED_INTS;
  cr3 |= (ie & USART_CTRL3_USED_INTS);
  up_serialout(priv, N32_USART_CTRL3_OFFSET, cr3);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_disableusartint
 ****************************************************************************/

static void up_disableusartint(struct up_dev_s *priv, uint16_t *ie)
{
  irqstate_t flags = spin_lock_irqsave(&priv->lock);

  if (ie)
    {
      uint32_t cr1 = up_serialin(priv, N32_USART_CTRL1_OFFSET);
      uint32_t cr3 = up_serialin(priv, N32_USART_CTRL3_OFFSET);

      *ie = (cr1 & USART_CTRL1_USED_INTS) | (cr3 & USART_CTRL3_USED_INTS);
    }

  up_setusartint(priv, 0);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: up_set_format
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void up_set_format(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t cr1;
  uint32_t cr2;
  uint32_t cr3;
  uint32_t brcf;
  uint32_t usartdiv;
  irqstate_t flags = enter_critical_section();

  /* Disable UE */

  cr1 = up_serialin(priv, N32_USART_CTRL1_OFFSET);
  cr1 &= ~USART_CTRL1_UEN;
  up_serialout(priv, N32_USART_CTRL1_OFFSET, cr1);

  /* Calculate baud rate */

  bool over8 = false;
  uint32_t tmp = (25 * (priv->apbclock / 2)) / priv->baud;

  if (tmp < 100) /* Empirical threshold for using 8x oversampling */
    {
      over8 = true;
      usartdiv = (tmp & 0xfff0) | ((tmp & 0x000f) >> 1);
    }
  else
    {
      usartdiv = (25 * (priv->apbclock / 4)) / priv->baud;
    }

  /* Convert to 12.4 format (integer part 12 bits, fractional 4 bits) */

  brcf = (usartdiv / 100) << 4;
  tmp = usartdiv - ((brcf >> 4) * 100);
  if (over8)
    {
      tmp = ((((tmp * 8) + 50) / 100)) & ((uint8_t)0x0f);
      if (tmp == 0x08)
        {
          brcf  = brcf + 0x10;
        }
      else
        {
          brcf |= tmp;
        }
    }
  else
    {
      /* Oversampling mode is 16 Samples */

      brcf += ((((tmp * 16) + 50) / 100)) & ((uint8_t)0x1f);
    }

  up_serialout(priv, N32_USART_BRCF_OFFSET, brcf);

  /* Oversampling mode */

  cr1 = up_serialin(priv, N32_USART_CTRL1_OFFSET);
  if (over8)
    {
      cr1 |= USART_CTRL1_OSPM;
    }
  else
    {
      cr1 &= ~USART_CTRL1_OSPM;
    }

  /* Word length and parity */

  cr1 &= ~(USART_CTRL1_WL | USART_CTRL1_PCEN | USART_CTRL1_PSEL);
  if (priv->bits == 8 && priv->parity != 0)
    {
      cr1 |= USART_CTRL1_WL;
    }

  if (priv->parity == 1) /* Odd parity */
    {
      cr1 |= (USART_CTRL1_PCEN | USART_CTRL1_PSEL);
    }
  else if (priv->parity == 2) /* Even parity */
    {
      cr1 |= USART_CTRL1_PCEN;
    }

  /* Stop bits */

  cr2 = up_serialin(priv, N32_USART_CTRL2_OFFSET);
  cr2 &= ~USART_CTRL2_STPB_MASK;
  if (priv->stopbits2)
    {
      cr2 |= USART_CTRL2_STPB(2); /* 2 stop bits */
    }
  else
    {
      cr2 |= USART_CTRL2_STPB(0); /* 1 stop bit */
    }

  up_serialout(priv, N32_USART_CTRL2_OFFSET, cr2);

  /* Hardware flow control (RTSE/CTSE) */

  cr3 = up_serialin(priv, N32_USART_CTRL3_OFFSET);
  cr3 &= ~(USART_CTRL3_CTSEN | USART_CTRL3_RTSEN);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      cr3 |= USART_CTRL3_RTSEN;
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->oflow)
    {
      cr3 |= USART_CTRL3_CTSEN;
    }
#endif

  up_serialout(priv, N32_USART_CTRL3_OFFSET, cr3);

  /* Re-enable UE */

  cr1 |= USART_CTRL1_UEN;
  up_serialout(priv, N32_USART_CTRL1_OFFSET, cr1);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: up_set_apb_clock
 ****************************************************************************/

static void up_set_apb_clock(struct up_dev_s *priv, bool on)
{
  uint32_t regaddr = 0;
  uint32_t en_bit = 0;

  switch (priv->usartbase)
    {
  #ifdef CONFIG_N32H7_USART1
      case N32_USART1_BASE:
        regaddr = N32_RCC_APB1EN3;
        en_bit  = RCC_APB1EN3_M7USART1EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_USART2
      case N32_USART2_BASE:
        regaddr = N32_RCC_APB1EN3;
        en_bit  = RCC_APB1EN3_M7USART2EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_USART3
      case N32_USART3_BASE:
        regaddr = N32_RCC_APB1EN3;
        en_bit  = RCC_APB1EN3_M7USART3EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_USART4
      case N32_USART4_BASE:
        regaddr = N32_RCC_APB1EN3;
        en_bit  = RCC_APB1EN3_M7USART4EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_USART5
      case N32_USART5_BASE:
        regaddr = N32_RCC_APB2EN3;
        en_bit  = RCC_APB2EN3_M7USART5EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_USART6
      case N32_USART6_BASE:
        regaddr = N32_RCC_APB2EN3;
        en_bit  = RCC_APB2EN3_M7USART6EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_USART7
      case N32_USART7_BASE:
        regaddr = N32_RCC_APB2EN3;
        en_bit  = RCC_APB2EN3_M7USART7EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_USART8
      case N32_USART8_BASE:
        regaddr = N32_RCC_APB2EN3;
        en_bit  = RCC_APB2EN3_M7USART8EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_UART9
      case N32_UART9_BASE:
        regaddr = N32_RCC_APB1EN3;
        en_bit  = RCC_APB1EN3_M7UART9EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_UART10
      case N32_UART10_BASE:
        regaddr = N32_RCC_APB1EN3;
        en_bit  = RCC_APB1EN3_M7UART10EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_UART11
      case N32_UART11_BASE:
        regaddr = N32_RCC_APB1EN3;
        en_bit  = RCC_APB1EN3_M7UART11EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_UART12
      case N32_UART12_BASE:
        regaddr = N32_RCC_APB1EN3;
        en_bit  = RCC_APB1EN3_M7UART12EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_UART13
      case N32_UART13_BASE:
        regaddr = N32_RCC_APB2EN3;
        en_bit  = RCC_APB2EN3_M7UART13EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_UART14
      case N32_UART14_BASE:
        regaddr = N32_RCC_APB2EN3;
        en_bit  = RCC_APB2EN3_M7UART14EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_UART15
      case N32_UART15_BASE:
        regaddr = N32_RCC_APB2EN3;
        en_bit  = RCC_APB2EN3_M7UART15EN;
        break;
  #endif
      default:
        return;
    }

  if (on)
    {
      modifyreg32(regaddr, 0, en_bit);
    }
  else
    {
      modifyreg32(regaddr, en_bit, 0);
    }
}

/****************************************************************************
 * Name: up_setup (non-DMA)
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t cr1;
  uint32_t cr2;
  uint32_t cr3;

  /* Enable clock */

  up_set_apb_clock(priv, true);

  /* Configure GPIOs */

  n32_configgpio(priv->tx_gpio);
  n32_configgpio(priv->rx_gpio);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio)
    {
      n32_configgpio(priv->rts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio)
    {
      n32_configgpio(priv->cts_gpio);
    }
#endif

  /* Disable USART for configuration */

  cr1 = up_serialin(priv, N32_USART_CTRL1_OFFSET);
  cr1 &= ~USART_CTRL1_UEN;
  up_serialout(priv, N32_USART_CTRL1_OFFSET, cr1);

  /* Configure CR2 (stop bits) */

  cr2 = up_serialin(priv, N32_USART_CTRL2_OFFSET);
  cr2 &= ~(USART_CTRL2_STPB_MASK | USART_CTRL2_CLKEN | USART_CTRL2_LINMEN);
  if (priv->stopbits2)
    {
      cr2 |= USART_CTRL2_STPB(2);
    }

  up_serialout(priv, N32_USART_CTRL2_OFFSET, cr2);

  /* Configure CR3 (flow control, DMA, etc.) */

  cr3 = up_serialin(priv, N32_USART_CTRL3_OFFSET);
  cr3 &= ~(USART_CTRL3_CTSEN | USART_CTRL3_RTSEN |
           USART_CTRL3_DMARXEN | USART_CTRL3_DMATXEN |
           USART_CTRL3_HDMEN | USART_CTRL3_SCMEN | USART_CTRL3_IRDAMEN);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      cr3 |= USART_CTRL3_RTSEN;
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->oflow)
    {
      cr3 |= USART_CTRL3_CTSEN;
    }
#endif

  up_serialout(priv, N32_USART_CTRL3_OFFSET, cr3);

  /* Call up_set_format to set baud rate, word length, parity, etc. */

  up_set_format(dev);

  /* Disable FIFO */

  uint32_t fifo = up_serialin(priv, N32_USART_FIFO_OFFSET);

  fifo &= ~USART_FIFO_EN;
  up_serialout(priv, N32_USART_FIFO_OFFSET, fifo);

  /* Enable USART (UE, TE, RE) */

  cr1 = up_serialin(priv, N32_USART_CTRL1_OFFSET);
  cr1 |= (USART_CTRL1_UEN | USART_CTRL1_TXEN | USART_CTRL1_RXEN);
  up_serialout(priv, N32_USART_CTRL1_OFFSET, cr1);

  priv->initialized = true;
  return OK;
}

/****************************************************************************
 * Name: up_shutdown (non-DMA)
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  priv->initialized = false;

  /* Disable interrupts */

  up_disableusartint(priv, NULL);

  /* Disable USART */

  uint32_t cr1 = up_serialin(priv, N32_USART_CTRL1_OFFSET);

  cr1 &= ~(USART_CTRL1_UEN | USART_CTRL1_TXEN | USART_CTRL1_RXEN);
  up_serialout(priv, N32_USART_CTRL1_OFFSET, cr1);

  /* Disable clock */

  up_set_apb_clock(priv, false);

  /* Release GPIOs (optional) */

  n32_unconfiggpio(priv->tx_gpio);
  n32_unconfiggpio(priv->rx_gpio);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio)
    {
      n32_unconfiggpio(priv->rts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio)
    {
      n32_unconfiggpio(priv->cts_gpio);
    }
#endif
}

/****************************************************************************
 * Name: up_dma_setup / up_dma_shutdown
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int up_dma_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Perform basic setup first */

  ret = up_setup(dev);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure TX DMA */

#ifdef SERIAL_HAVE_TXDMA
  if (priv->txdma_channel != 0)
    {
      priv->txdma = n32_dmachannel(priv->txdma_channel);
      if (!priv->txdma)
        {
          return -ENODEV;
        }

      /* Enable USART DMA transmission */

      uint32_t cr3 = up_serialin(priv, N32_USART_CTRL3_OFFSET);

      cr3 |= USART_CTRL3_DMATXEN;
      up_serialout(priv, N32_USART_CTRL3_OFFSET, cr3);
    }
#endif

  /* Configure RX DMA */

#ifdef SERIAL_HAVE_RXDMA
  if (priv->rxdma_channel != 0)
    {
      uart_recvchars_dma(&priv->dev);
      priv->rxdma = n32_dmachannel(priv->rxdma_channel);
      if (!priv->rxdma)
        {
          return -ENODEV;
        }

      /* Configure DMA for circular reception */

      rxcfg.src_addr = priv->usartbase + N32_USART_DAT_OFFSET;
      rxcfg.dst_addr = (uint32_t)priv->dev.dmarx.buffer;
      rxcfg.block_size = priv->dev.dmarx.length;
      rxcfg.src_hs_if = n32_getchanel(priv->rxdma);
      rxcfg.dst_hs_if = n32_getchanel(priv->rxdma);

      n32_dmasetup(priv->rxdma, &rxcfg);
      n32_dmastart(priv->rxdma, up_dma_rxcallback, priv);

      /* Enable USART DMA reception */

      uint32_t cr3 = up_serialin(priv, N32_USART_CTRL3_OFFSET);

      cr3 |= USART_CTRL3_DMARXEN;
      up_serialout(priv, N32_USART_CTRL3_OFFSET, cr3);
    }
#endif

  return OK;
}

static void up_dma_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Stop DMA */

#ifdef SERIAL_HAVE_RXDMA
  if (priv->rxdma)
    {
      n32_dmastop(priv->rxdma);
      n32_dmafree(priv->rxdma);
      priv->rxdma = NULL;
    }
#endif

#ifdef SERIAL_HAVE_TXDMA
  if (priv->txdma)
    {
      n32_dmastop(priv->txdma);
      n32_dmafree(priv->txdma);
      priv->txdma = NULL;
    }
#endif

  up_shutdown(dev);
}
#endif

/****************************************************************************
 * Name: up_attach / up_detach
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret = irq_attach(priv->irq, up_interrupt, priv);

  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct up_dev_s *priv = (struct up_dev_s *)arg;
  uint32_t sts = up_serialin(priv, N32_USART_STS_OFFSET);

  /* Report activity to PM */

#ifdef CONFIG_PM
  pm_activity(PM_IDLE_DOMAIN, CONFIG_N32H7_PM_SERIAL_ACTIVITY);
#endif

  /* Handle errors (ORE, FE, NE) */

  if (sts & (USART_STS_OREF | USART_STS_FEF | USART_STS_NEF))
    {
      /* Clear error by reading STS then DAT */

      uint32_t dat = up_serialin(priv, N32_USART_DAT_OFFSET);

      UNUSED(dat);
    }

  /* Handle IDLE interrupt (DMA reception complete) */

  if ((sts & USART_STS_IDLEF) && (priv->rxdma) &&
      (priv->ie & USART_CTRL1_IDLEIEN))
    {
      if (up_dma_rxavailable(&priv->dev))
        {
          /* Clear error by reading STS then DAT */

          uint32_t dat = up_serialin(priv, N32_USART_DAT_OFFSET);

          UNUSED(dat);

          n32_dmastop(priv->rxdma);
          priv->dev.dmarx.nbytes = n32_getcount(priv->rxdma);
          uart_recvchars_done(&priv->dev);
          uart_recvchars_dma(&priv->dev);

          rxcfg.src_addr = priv->usartbase + N32_USART_DAT_OFFSET;
          rxcfg.dst_addr = (uint32_t)priv->dev.dmarx.buffer;
          rxcfg.block_size = priv->dev.dmarx.length;
          rxcfg.src_hs_if = n32_getchanel(priv->rxdma);
          rxcfg.dst_hs_if = n32_getchanel(priv->rxdma);

          n32_dmasetup(priv->rxdma, &rxcfg);
          n32_dmastart(priv->rxdma, up_dma_rxcallback, priv);

          return OK;
        }
    }

  /* Non-DMA reception */

  if ((sts & USART_STS_RXDNE) && (priv->ie & USART_CTRL1_RXDNEIEN))
    {
      uart_recvchars(&priv->dev);
    }

  /* Transmit by interrupt */

  if ((sts & USART_STS_TXDE) && (priv->ie & USART_CTRL1_TXDEIEN))
    {
      uart_xmitchars(&priv->dev);
    }

  return OK;
}

/****************************************************************************
 * Name: up_receive (non-DMA)
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t dat = up_serialin(priv, N32_USART_DAT_OFFSET);

  *status = 0;
  return dat & 0xff;
}
#endif

/****************************************************************************
 * Name: up_rxint
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;
  uint16_t ie;

  /* USART receive interrupts:
   *
   * Enable             Status          Meaning                  Usage
   * ------------------ --------------- ------------------------ ----------
   * USART_CR1_IDLEIE   USART_ISR_IDLE  Idle Line Detected       (not used)
   * USART_CR1_RXNEIE   USART_ISR_RXNE  Received Data Ready
   *                                      to be Read
   * "              "   USART_ISR_ORE   Overrun Error Detected
   * USART_CR1_PEIE     USART_ISR_PE    Parity Error
   *
   * USART_CR2_LBDIE    USART_ISR_LBD   Break Flag               (not used)
   * USART_CR3_EIE      USART_ISR_FE    Framing Error
   * "           "      USART_ISR_NF    Noise Error
   * "           "      USART_ISR_ORE   Overrun Error Detected
   */

  flags = enter_critical_section();
  ie = priv->ie;
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register
       * (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
#ifdef CONFIG_USART_ERRINTS
      ie |= (USART_CTRL1_RXDNEIEN | USART_CTRL1_PEIEN | USART_CTRL3_ERRIEN);
#else
      ie |= USART_CTRL1_RXDNEIEN;
#endif
#endif
    }
  else
    {
      ie &= ~(USART_CTRL1_RXDNEIEN | USART_CTRL1_PEIEN | USART_CTRL3_ERRIEN);
    }

  /* Then set the new interrupt state */

  up_setusartint(priv, ie);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return (up_serialin(priv, N32_USART_STS_OFFSET) & USART_STS_RXDNE) != 0;
}

/****************************************************************************
 * Name: up_rxflowcontrol
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev, unsigned int nbuffered,
                             bool upper)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  if (priv->iflow)
    {
      /* Simple flow control: disable RX interrupt when buffer full */

      if (upper)
        {
          uart_disablerxint(dev);
          return true;
        }
      else
        {
          uart_enablerxint(dev);
        }
    }

  return false;
}
#endif

/****************************************************************************
 * Name: up_send
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  up_serialout(priv, N32_USART_DAT_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  /* USART transmit interrupts:
   *
   * Enable             Status      Meaning                 Usage
   * --------------- -------------- --------------------- ----------
   * USART_CR1_TCIE  USART_ISR_TC   Transmission Complete (used only
   *                                                       for RS-485)
   * USART_CR1_TXEIE USART_ISR_TXE  Transmit Data
   *                                   Register Empty
   * USART_CR3_CTSIE USART_ISR_CTS  CTS flag               (not used)
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uint16_t ie = priv->ie | USART_CTRL1_TXDEIEN;

#  ifdef CONFIG_STM32_SERIALBRK_BSDCOMPAT
      if (priv->ie & USART_CR1_IE_BREAK_INPROGRESS)
        {
          leave_critical_section(flags);
          return;
        }
#  endif

      up_setusartint(priv, ie);
#else
      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      up_setusartint(priv, priv->ie & ~USART_CTRL1_TXDEIEN);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready / up_txempty
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return (up_serialin(priv, N32_USART_STS_OFFSET) & USART_STS_TXDE) != 0;
}

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return (up_serialin(priv, N32_USART_STS_OFFSET) & USART_STS_TXC) != 0;
}

/****************************************************************************
 * Name: up_dma_rxcallback
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct up_dev_s *priv = arg;

  uart_recvchars_dma(&priv->dev);
}
#endif

/****************************************************************************
 * Name: up_dma_receive / up_dma_rxint / up_dma_rxavailable
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void up_dma_receive(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t nextrx = n32_getcount(priv->rxdma);

  /* Check if more data is available */

  if (nextrx != 0)
    {
#ifdef CONFIG_ARMV7M_DCACHE
      /* If the data cache is enabled, then we will also need to manage
       * cache coherency.  Are any bytes available in the currently coherent
       * region of the data cache?
       */

      uintptr_t addr;

      /* Invalidate the DMA buffer range */

      addr = (uintptr_t)&dev->recv.buffer[dev->recv.tail];
      up_invalidate_dcache(addr, addr + nextrx);
#endif
    }
}

static void up_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  /* Check if DMA handle is valid */

  if (!priv->rxdma)
    {
      return;
    }

  flags = enter_critical_section();
  if (enable)
    {
      uint16_t ie = priv->ie | USART_CTRL1_IDLEIEN;

      up_setusartint(priv, ie);
    }
  else
    {
      uint16_t ie = priv->ie & ~USART_CTRL1_IDLEIEN;

      up_setusartint(priv, ie);
    }

  leave_critical_section(flags);
}

static bool up_dma_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return n32_getcount(priv->rxdma) != 0;
}
#endif

/****************************************************************************
 * Name: up_dma_txcallback / up_dma_txavailable / up_dma_send / up_dma_txint
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void up_dma_txcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct up_dev_s *priv = arg;

  if (status == DMA_INT_TFR)
    {
      priv->dev.dmatx.nbytes += priv->dev.dmatx.length;
      if (priv->dev.dmatx.nlength)
        {
          /* Set up DMA on next buffer */

          txcfg.src_addr = (uint32_t)priv->dev.dmatx.nbuffer;
          txcfg.dst_addr = (uint32_t)priv->usartbase | N32_USART_DAT_OFFSET;
          txcfg.block_size = priv->dev.dmatx.nlength;
          txcfg.src_hs_if  = n32_getchanel(priv->txdma);
          txcfg.dst_hs_if  = n32_getchanel(priv->txdma);
          n32_dmasetup(priv->txdma, &txcfg);

          /* Set length for the next completion */

          priv->dev.dmatx.length  = priv->dev.dmatx.nlength;
          priv->dev.dmatx.nlength = 0;

          /* Start transmission with the callback on DMA completion */

          n32_dmastart(priv->txdma, up_dma_txcallback, priv);

          return;
        }
    }
  else if(status == DMA_INT_ERROR)
    {
      PANIC();
    }

  /* Adjust the pointers */

  uart_xmitchars_done(&priv->dev);

  /* Send more if available */

  up_dma_txavailable(&priv->dev);
}

static void up_dma_txavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags = enter_critical_section();

  if (priv->dev.dmatx.length == 0 && priv->dev.dmatx.nlength == 0)
    {
      uart_xmitchars_dma(dev);
    }

  leave_critical_section(flags);
}

static void up_dma_send(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* We need to stop DMA before reconfiguration */

  n32_dmastop(priv->txdma);

  /* Reset the number sent */

  dev->dmatx.nbytes = 0;

  /* Flush the contents of the TX buffer into physical memory */

  up_clean_dcache((uintptr_t)dev->dmatx.buffer,
                  (uintptr_t)dev->dmatx.buffer + dev->dmatx.length);

  /* Is this a split transfer */

  if (dev->dmatx.nbuffer)
    {
      /* Flush the contents of the next TX buffer into physical memory */

      up_clean_dcache((uintptr_t)dev->dmatx.nbuffer,
                      (uintptr_t)dev->dmatx.nbuffer + dev->dmatx.nlength);
    }

  /* Configure TX DMA */

  txcfg.src_addr   = (uint32_t)priv->dev.dmatx.buffer;
  txcfg.dst_addr   = (uint32_t)priv->usartbase | N32_USART_DAT_OFFSET;
  txcfg.block_size = priv->dev.dmatx.length;
  txcfg.src_hs_if  = n32_getchanel(priv->txdma);
  txcfg.dst_hs_if  = n32_getchanel(priv->txdma);

  n32_dmasetup(priv->txdma, &txcfg);
  n32_dmastart(priv->txdma, up_dma_txcallback, priv);
}

static void up_dma_txint(struct uart_dev_s *dev, bool enable)
{
  /* DMA TX doesn't use interrupts, driven by DMA callback */
}
#endif

/****************************************************************************
 * Name: up_ioctl
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret = OK;
#endif

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
              memcpy(user, priv, sizeof(struct up_dev_s));
            }
        }
        break;
  #endif

  #ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct termios *t = (struct termios *)arg;

          if (!t)
            {
              ret = -EINVAL;
            }

          t->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                      ((priv->parity == 1) ? PARODD : 0) |
                      ((priv->stopbits2) ? CSTOPB : 0) |
  #ifdef CONFIG_SERIAL_OFLOWCONTROL
                      ((priv->oflow) ? CCTS_OFLOW : 0) |
  #endif
  #ifdef CONFIG_SERIAL_IFLOWCONTROL
                      ((priv->iflow) ? CRTS_IFLOW : 0) |
  #endif
                      ((priv->bits == 8 && priv->parity != 0) ? CS7 : CS8);
          cfsetispeed(t, priv->baud);
          cfsetospeed(t, priv->baud);
        }
        break;

      case TCSETS:
        {
          struct termios *t = (struct termios *)arg;

          if (!t)
            {
              return -EINVAL;
            }

          if ((t->c_cflag & (CS8 | CS7)) == 0)
            {
              return -EINVAL;
            }

          if ((t->c_cflag & CSIZE) == CS7 && (t->c_cflag & PARENB) == 0)
            {
              return -EINVAL;
            }

          priv->parity = (t->c_cflag & PARENB) ?
                        ((t->c_cflag & PARODD) ? 1 : 2) : 0;
          priv->stopbits2 = (t->c_cflag & CSTOPB) != 0;
  #ifdef CONFIG_SERIAL_OFLOWCONTROL
          priv->oflow = (t->c_cflag & CCTS_OFLOW) != 0;
  #endif
  #ifdef CONFIG_SERIAL_IFLOWCONTROL
          priv->iflow = (t->c_cflag & CRTS_IFLOW) != 0;
  #endif
          priv->baud = cfgetispeed(t);
          up_set_format(dev);
        }

        break;
  #endif

      case TIOCSSWAP:
        {
          uint32_t cr1 = up_serialin(priv, N32_USART_CTRL1_OFFSET);
          uint32_t ue = cr1 & USART_CTRL1_UEN;

          cr1 &= ~USART_CTRL1_UEN;
          up_serialout(priv, N32_USART_CTRL1_OFFSET, cr1);
          if (arg)
            {
              cr1 |= USART_CTRL1_SWAP;
            }
          else
            {
              cr1 &= ~USART_CTRL1_SWAP;
            }

          cr1 |= ue;
          up_serialout(priv, N32_USART_CTRL1_OFFSET, cr1);
        }

        break;

      case TIOCSBRK:
        {
          uint32_t cr1 = up_serialin(priv, N32_USART_CTRL1_OFFSET);

          cr1 |= USART_CTRL1_SDBRK;
          up_serialout(priv, N32_USART_CTRL1_OFFSET, cr1);

          /* Wait for break to be sent (hardware clears it automatically) */

          while (up_serialin(priv, N32_USART_CTRL1_OFFSET) &
                USART_CTRL1_SDBRK);
        }

        break;
      case TIOCCBRK:

        /* No operation needed */

        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_pm_* (Power Management)
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_setsuspend(bool suspend)
{
  /* Iterate all UARTs to suspend/resume */

  for (int i = 0; i < sizeof(g_uart_devs) / sizeof(g_uart_devs[0]); i++)
    {
      struct up_dev_s *priv = g_uart_devs[i];

      if (!priv || !priv->initialized)
        {
          continue;
        }

      if (suspend)
        {
          /* Save and disable interrupts */

          up_disableusartint(priv, &priv->suspended_ie);
          priv->suspended = true;
        }
      else
        {
          up_setusartint(priv, priv->suspended_ie);
          priv->suspended = false;
        }
    }
}

static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  if (pmstate == PM_NORMAL || pmstate == PM_IDLE)
    {
      up_pm_setsuspend(false);
    }
  else
    {
      up_pm_setsuspend(true);
    }
}

static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* Check if there is pending data, prevent low-power if so */

  if (pmstate == PM_SLEEP || pmstate == PM_STANDBY)
    {
      for (int i = 0; i < sizeof(g_uart_devs) / sizeof(g_uart_devs[0]); i++)
        {
          struct up_dev_s *priv = g_uart_devs[i];

          if (!priv || !priv->initialized)
            {
              continue;
            }

          if (priv->dev.xmit.head != priv->dev.xmit.tail)
            {
              return -EBUSY;
            }

          if (priv->dev.recv.head != priv->dev.recv.tail)
            {
              return -EBUSY;
            }
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_serial_get_uart
 ****************************************************************************/

uart_dev_t *n32_serial_get_uart(int uart_num)
{
  int idx = uart_num - 1;

  if (idx < 0 || idx >= (int)(sizeof(g_uart_devs) / sizeof(g_uart_devs[0])))
    {
      return NULL;
    }

  struct up_dev_s *priv = g_uart_devs[idx];

  if (!priv || !priv->initialized)
    {
      return NULL;
    }

  return &priv->dev;
}

/****************************************************************************
 * Name: arm_earlyserialinit
 ****************************************************************************/

void arm_earlyserialinit(void)
{
#ifdef HAVE_UART
  /* Disable all UART interrupts */

  for (int i = 0; i < sizeof(g_uart_devs) / sizeof(g_uart_devs[0]); i++)
    {
      if (g_uart_devs[i])
        {
          up_disableusartint(g_uart_devs[i], NULL);
        }
    }
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef HAVE_UART
  char devname[32];
  int minor = 0;

  UNUSED(g_uart_rxdma_ops);
  UNUSED(g_uart_txdma_ops);
  UNUSED(g_uart_rxtxdma_ops);
  UNUSED(g_uart_ops);

  /* Configure console */

#if CONSOLE_UART > 0
#if defined(SERIAL_HAVE_TXDMA) || defined(SERIAL_HAVE_RXDMA)
  up_dma_setup(&g_uart_devs[CONSOLE_UART - 1]->dev);
#else
  up_setup(&g_uart_devs[CONSOLE_UART - 1]->dev);
#endif
#endif

#ifdef CONFIG_PM
  pm_register(&g_serialpm.pm_cb);
#endif

  /* Register console */

#if CONSOLE_UART > 0
  uart_register("/dev/console", &g_uart_devs[CONSOLE_UART - 1]->dev);

  /* Register as ttyS0 first, skip console in later loop */

  uart_register("/dev/ttyS0", &g_uart_devs[CONSOLE_UART - 1]->dev);
  minor = 1;
#endif

  /* Register remaining UARTs */

  for (int i = 0; i < sizeof(g_uart_devs) / sizeof(g_uart_devs[0]); i++)
    {
      if (!g_uart_devs[i])
        {
          continue;
        }

      if (g_uart_devs[i]->dev.isconsole)
        {
          continue;
        }

      snprintf(devname, sizeof(devname), "/dev/ttyS%d", minor++);
      uart_register(devname, &g_uart_devs[i]->dev);
    }
#endif
}

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

void up_putc(int ch)
{
#if CONSOLE_UART > 0
  struct up_dev_s *priv = g_uart_devs[CONSOLE_UART - 1];
  uint16_t ie;

  up_disableusartint(priv, &ie);
  arm_lowputc(ch);
  up_setusartint(priv, ie);
#endif
}

#endif /* HAVE_UART */
