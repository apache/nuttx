/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_serial.c
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

#include <errno.h>
#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/compiler.h>
#include <nuttx/board.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>
#include <arch/board/board.h>

#include <assert.h>
#include <debug.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "arm_internal.h"

#include "hardware/tlsr82_gpio.h"
#include "hardware/tlsr82_register.h"
#include "tlsr82_spi_console.h"
#include "tlsr82_gpio.h"

#include "hardware/tlsr82_uart.h"
#include "hardware/tlsr82_dma.h"
#include "hardware/tlsr82_clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_RAMCODE               locate_code(".ram_code")

#define CONSOLE_PORT               0
#define CONSOLE_PRIV               g_uart0priv
#define CONSOLE_DEV                g_uart0_dev

#define UART0_PIN_TX_MUX           GPIO_AF_MUX1
#define UART0_PIN_RX_MUX           GPIO_AF_MUX1

#define UART0_PIN_TX               GPIO_PIN_PB1
#define UART0_PIN_RX               GPIO_PIN_PB0

#define UART0_TX_BUF_SIZE          CONFIG_TLSR82_UART0_TX_BUF_SIZE
#define UART0_RX_BUF_SIZE          CONFIG_TLSR82_UART0_RX_BUF_SIZE

#define UART0_TXDMA_BUF_SIZE       CONFIG_TLSR82_UART0_TXDMA_BUF_SIZE
#define UART0_RXDMA_BUF_SIZE       CONFIG_TLSR82_UART0_RXDMA_BUF_SIZE

#define DMA_HEAD_LEN               4

#ifdef CONFIG_SERIAL_TXDMA
#  if UART0_TXDMA_BUF_SIZE > UINT8_MAX
#    error "UART0 Tx dma buffer size must less than 255"
#  endif

#  if (UART0_TXDMA_BUF_SIZE % 16) != 0
#    error "UART0 Tx dma buffer size must be multiple of 16"
#  endif

#  if UART0_TXDMA_BUF_SIZE < UART0_TX_BUF_SIZE
#    warning "UART0 Tx dma buffer size better larger than tx buffer size"
#  endif
#endif

#ifdef CONFIG_SERIAL_RXDMA
#  if UART0_RXDMA_BUF_SIZE > UINT8_MAX
#    error "UART0 Rx dma buffer size must less than 255"
#  endif

#  if (UART0_RXDMA_BUF_SIZE % 16) != 0
#    error "UART0 Rx dma buffer size must be multiple of 16"
#  endif

#  if UART0_RXDMA_BUF_SIZE < UART0_RX_BUF_SIZE
#    warning "UART0 Rx dma buffer size better larger than rx buffer size"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  uint32_t baud;
  uint32_t port;
  uint32_t irq;
  uint32_t txpin;
  uint32_t rxpin;
  uint32_t parity;
  uint32_t stopbits;
  bool     txdma;
  bool     rxdma;
#ifdef CONFIG_SERIAL_TXDMA
  sem_t   *txdmasem;
  char    *txdmabuf;
  uint32_t txdmasize;
#endif
#ifdef CONFIG_SERIAL_RXDMA
  char    *rxdmabuf;
  uint32_t rxdmasize;
  uint32_t rxdmaindex;
  uint32_t rxdmalen;
#endif
} uart_priv_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Uart interrupt handler */

static int UART_RAMCODE tlsr82_interrupt(int irq, void *context, void *arg);

#if defined(CONFIG_SERIAL_RXDMA) || defined(CONFIG_SERIAL_TXDMA)
static int UART_RAMCODE tlsr82_dma_interrupt(int irq, void *context,
                                             void *arg);
#endif

/* Uart operation function */

static int  tlsr82_uart_setup(struct uart_dev_s *dev);
static void tlsr82_uart_shutdown(struct uart_dev_s *dev);
static int  tlsr82_uart_attach(struct uart_dev_s *dev);
static void tlsr82_uart_detach(struct uart_dev_s *dev);
static int  tlsr82_uart_ioctl(struct file *filep, int cmd,
                              unsigned long arg);
static void tlsr82_uart_rxint(struct uart_dev_s *dev, bool enable);
static void tlsr82_uart_txint(uart_dev_t *dev, bool enable);
static int  UART_RAMCODE tlsr82_uart_receive(struct uart_dev_s *dev,
                                             unsigned int * status);
static bool UART_RAMCODE tlsr82_uart_rxavailable(struct uart_dev_s *dev);
static void UART_RAMCODE tlsr82_uart_send(struct uart_dev_s *dev, int ch);
static bool UART_RAMCODE tlsr82_uart_txready(struct uart_dev_s *dev);
static bool UART_RAMCODE tlsr82_uart_txempty(struct uart_dev_s *dev);

#ifdef CONFIG_SERIAL_RXDMA
static void tlsr82_uart_dma_rxint(struct uart_dev_s *dev, bool enable);
static int  UART_RAMCODE tlsr82_uart_dma_receive(struct uart_dev_s *dev,
                                                 unsigned int * status);
static bool UART_RAMCODE tlsr82_uart_dma_rxavail(struct uart_dev_s *dev);
#endif

#ifdef CONFIG_SERIAL_TXDMA
static void tlsr82_uart_dma_send(struct uart_dev_s *dev);
static void tlsr82_uart_dma_txavail(struct uart_dev_s *dev);
static void tlsr82_uart_dma_txint(struct uart_dev_s *dev, bool enable);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t uart_rxindex;
static uint8_t uart_txindex;

/* UART operations */

#if !defined(CONFIG_TLSR82_UART0_RXDMA) && \
    !defined(CONFIG_TLSR82_UART0_TXDMA)
static const struct uart_ops_s g_uart_ops =
{
  .setup       = tlsr82_uart_setup,
  .shutdown    = tlsr82_uart_shutdown,
  .attach      = tlsr82_uart_attach,
  .detach      = tlsr82_uart_detach,
  .ioctl       = tlsr82_uart_ioctl,
  .receive     = tlsr82_uart_receive,
  .rxint       = tlsr82_uart_rxint,
  .rxavailable = tlsr82_uart_rxavailable,
  .send        = tlsr82_uart_send,
  .txint       = tlsr82_uart_txint,
  .txready     = tlsr82_uart_txready,
  .txempty     = tlsr82_uart_txempty,
};
#elif defined(CONFIG_TLSR82_UART0_RXDMA) && \
      !defined(CONFIG_TLSR82_UART0_TXDMA)
static const struct uart_ops_s g_uart_rxdma_ops =
{
  .setup       = tlsr82_uart_setup,
  .shutdown    = tlsr82_uart_shutdown,
  .attach      = tlsr82_uart_attach,
  .detach      = tlsr82_uart_detach,
  .ioctl       = tlsr82_uart_ioctl,
  .receive     = tlsr82_uart_dma_receive,
  .rxint       = tlsr82_uart_dma_rxint,
  .rxavailable = tlsr82_uart_dma_rxavail,
  .send        = tlsr82_uart_send,
  .txint       = tlsr82_uart_txint,
  .txready     = tlsr82_uart_txready,
  .txempty     = tlsr82_uart_txempty,
};
#elif !defined(CONFIG_TLSR82_UART0_RXDMA) && \
      defined(CONFIG_TLSR82_UART0_TXDMA)
static const struct uart_ops_s g_uart_txdma_ops =
{
  .setup       = tlsr82_uart_setup,
  .shutdown    = tlsr82_uart_shutdown,
  .attach      = tlsr82_uart_attach,
  .detach      = tlsr82_uart_detach,
  .ioctl       = tlsr82_uart_ioctl,
  .receive     = tlsr82_uart_receive,
  .rxint       = tlsr82_uart_rxint,
  .rxavailable = tlsr82_uart_rxavailable,
  .send        = tlsr82_uart_send,
  .txint       = tlsr82_uart_dma_txint,
  .txready     = tlsr82_uart_txready,
  .txempty     = tlsr82_uart_txempty,
  .dmatxavail  = tlsr82_uart_dma_txavail,
  .dmasend     = tlsr82_uart_dma_send,
};
#else
static const struct uart_ops_s g_uart_rxtxdma_ops =
{
  .setup       = tlsr82_uart_setup,
  .shutdown    = tlsr82_uart_shutdown,
  .attach      = tlsr82_uart_attach,
  .detach      = tlsr82_uart_detach,
  .ioctl       = tlsr82_uart_ioctl,
  .receive     = tlsr82_uart_dma_receive,
  .rxint       = tlsr82_uart_dma_rxint,
  .rxavailable = tlsr82_uart_dma_rxavail,
  .send        = tlsr82_uart_send,
  .txint       = tlsr82_uart_dma_txint,
  .txready     = tlsr82_uart_txready,
  .txempty     = tlsr82_uart_txempty,
  .dmatxavail  = tlsr82_uart_dma_txavail,
  .dmasend     = tlsr82_uart_dma_send,
};
#endif

static char g_uart0rxbuffer[UART0_RX_BUF_SIZE];
static char g_uart0txbuffer[UART0_TX_BUF_SIZE];

#ifdef CONFIG_TLSR82_UART0_TXDMA
static char g_uart0txdmabuf[UART0_TXDMA_BUF_SIZE + DMA_HEAD_LEN] \
aligned_data(4);
static sem_t g_uart0txdmasem = SEM_INITIALIZER(1);
#endif

#ifdef CONFIG_TLSR82_UART0_RXDMA
static char g_uart0rxdmabuf[UART0_RXDMA_BUF_SIZE + DMA_HEAD_LEN] \
aligned_data(4);
#endif

/* Private UART config and state */

static uart_priv_t g_uart0priv =
{
  .baud       = 115200,
  .port       = 0,
  .irq        = NR_UART_IRQ,
  .parity     = 0,
  .stopbits   = 0,
  .txpin      = UART0_PIN_TX,
  .rxpin      = UART0_PIN_RX,
#ifdef CONFIG_TLSR82_UART0_TXDMA
  .txdma      = true,
  .txdmasem   = &g_uart0txdmasem,
  .txdmabuf   = g_uart0txdmabuf,
  .txdmasize  = UART0_TXDMA_BUF_SIZE,
#else
  .txdma      = false,
#endif
#ifdef CONFIG_TLSR82_UART0_RXDMA
  .rxdma      = true,
  .rxdmabuf   = g_uart0rxdmabuf,
  .rxdmasize  = UART0_RXDMA_BUF_SIZE,
  .rxdmalen   = 0,
  .rxdmaindex = 0,
#else
  .rxdma      = false,
#endif
};

/* UART device structure */

static uart_dev_t g_uart0_dev =
{
  .recv =
    {
      .size   = sizeof(g_uart0rxbuffer),
      .buffer = g_uart0rxbuffer,
    },
  .xmit =
    {
      .size   = sizeof(g_uart0txbuffer),
      .buffer = g_uart0txbuffer,
    },
#if defined(CONFIG_TLSR82_UART0_TXDMA) && defined(CONFIG_TLSR82_UART0_RXDMA)
  .ops  = &g_uart_rxtxdma_ops,
#elif defined(CONFIG_TLSR82_UART0_TXDMA) && !defined(CONFIG_TLSR82_UART0_RXDMA)
  .ops  = &g_uart_txdma_ops,
#elif !defined(CONFIG_TLSR82_UART0_TXDMA) && defined(CONFIG_TLSR82_UART0_RXDMA)
  .ops  = &g_uart_rxdma_ops,
#else
  .ops  = &g_uart_ops,
#endif
  .priv = &g_uart0priv,
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_reset
 *
 * Description:
 *   Reset the uart hardware, the software pointer must be set to zero
 *   (function uart_clr_rx_index() must be called).
 *
 * Parameters:
 *   void
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static inline void uart_reset(void)
{
  RESET_RST0_REG |= RESET_RST0_UART;
  RESET_RST0_REG &= ~RESET_RST0_UART;
}

/****************************************************************************
 * Name: uart_reset
 *
 * Description:
 *   Clear the uart receive software pointer, this function must be called
 *   after wakeup from power-saving mode or reset uart.
 *
 * Parameters:
 *   uart_num  - the uart hardware index
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static inline void uart_clr_rx_index(int uart_num)
{
  uart_rxindex = 0;
}

/****************************************************************************
 * Name: uart_get_rxfifo_num
 *
 * Description:
 *   Get the recieved data numbers in the rx fifo.
 *
 * Parameters:
 *   uart_num  - the uart hardware index
 *
 * Returned Values:
 *   number
 *
 ****************************************************************************/

static inline uint8_t uart_get_rxfifo_num(int uart_num)
{
  return UART_GET_RX_BUF_CNT();
}

/****************************************************************************
 * Name: uart_get_txfifo_num
 *
 * Description:
 *   Get the transimit data numbers in the tx fifo.
 *
 * Parameters:
 *   uart_num  - the uart hardware index
 *
 * Returned Values:
 *   number
 *
 ****************************************************************************/

static inline uint8_t uart_get_txfifo_num(int uart_num)
{
  return UART_GET_TX_BUF_CNT();
}

/****************************************************************************
 * Name: uart_get_irq_status
 *
 * Description:
 *   Get the uart interrupt status
 *
 * Parameters:
 *   uart_num  - the uart hardware index
 *   flags     - interrupt bit mask: UART_IRQ_TXDONE
 *                                   UART_IRQ_TXBUF
 *                                   UART_IRQ_RXDONE
 *                                   UART_IRQ_RXBUF
 *
 * Returned Values:
 *   interrupt status: 0, interrupt not occured
 *                     1, interrupt occured
 *
 ****************************************************************************/

static inline uint8_t uart_get_irq_status(int uart_num, uint8_t flags)
{
  return (UART_IRQ_REG & flags);
}

/****************************************************************************
 * Name: uart_irq_clr
 *
 * Description:
 *   Clear the uart interrupt status
 *
 * Parameters:
 *   uart_num  - the uart hardware index
 *   flag      - interrupt bit mask: UART_IRQ_CLR_RX
 *                                   UART_IRQ_CLR_TX
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static inline void uart_irq_clr(int uart_num, uint8_t flag)
{
  BM_SET(UART_IRQ_REG, flag);
}

/****************************************************************************
 * Name: uart_irq_rx_enable
 *
 * Description:
 *   Enable/Disable the uart rx interrupt
 *
 * Parameters:
 *   enable  - true : enable the rx interrupt
 *             false: disable the rx interrupt
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static inline void uart_irq_rx_enable(bool enable)
{
  if (enable)
    {
      UART_CTRL0_REG |= UART_CTRL0_RX_IRQ_EN;
    }
  else
    {
      UART_CTRL0_REG &= ~UART_CTRL0_RX_IRQ_EN;
    }
}

/****************************************************************************
 * Name: uart_irq_tx_enable
 *
 * Description:
 *   Enable/Disable the uart tx interrupt
 *
 * Parameters:
 *   enable  - true : enable the tx interrupt
 *             false: disable the tx interrupt
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static inline void uart_irq_tx_enable(bool enable)
{
  if (enable)
    {
      UART_CTRL0_REG |= UART_CTRL0_TX_IRQ_EN;
    }
  else
    {
      UART_CTRL0_REG &= ~UART_CTRL0_TX_IRQ_EN;
    }
}

/****************************************************************************
 * Name: uart_txbufirq_isenable
 *
 * Description:
 *   Check weather the uart tx buffer interrupt is enable or not
 *
 * Parameters:
 *   uart_num  - the uart hardware index
 *
 * Returned Values:
 *   interrupt is enable or not: 1, enable
 *                               0, disable
 *
 ****************************************************************************/

static inline int uart_txbufirq_isenable(int uart_num)
{
  return (UART_CTRL0_REG & UART_CTRL0_TX_IRQ_EN);
}

/****************************************************************************
 * Name: uart_rxbufirq_isenable
 *
 * Description:
 *   Check weather the uart rx buffer interrupt is enable or not
 *
 * Parameters:
 *   uart_num  - the uart hardware index
 *
 * Returned Values:
 *   interrupt is enable or not: 1, enable
 *                               0, disable
 *
 ****************************************************************************/

static inline int uart_rxbufirq_isenable(int uart_num)
{
  return (UART_CTRL0_REG & UART_CTRL0_RX_IRQ_EN);
}

/****************************************************************************
 * Name: uart_rxbufirq_isenable
 *
 * Description:
 *   Check weather the uart tx done interrupt is enable or not,
 *   uart rx done interrupt is used for uart tx dma mode.
 *
 * Parameters:
 *   port  - the uart hardware index
 *
 * Returned Values:
 *   interrupt is enable or not: 1, enable
 *                               0, disable
 *
 ****************************************************************************/

static inline bool uart_txdoneirq_isenable(int port)
{
  return (UART_RXTIMEOUT1_REG & UART_RXTIMEOUT1_MASK_TXDONE);
}

/****************************************************************************
 * Name: uart_read_byte
 *
 * Description:
 *   Read a byte data from uart received buffer
 *
 * Parameters:
 *   uart_num  - the uart hardware index
 *
 * Returned Values:
 *   received char
 *
 ****************************************************************************/

static inline uint8_t uart_read_byte(int uart_num)
{
  uint8_t data = UART_BUF(uart_rxindex);

  uart_rxindex++;

  /* Cycle the four register 0x90 0x91 0x92 0x93 */

  uart_rxindex &= 0x03;
  return data;
}

/****************************************************************************
 * Name: uart_send_byte
 *
 * Description:
 *   Send a char by uart transimit buffer
 *
 * Parameters:
 *   uart_num  - the uart hardware index
 *   byte      - the char want to send
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static inline void uart_send_byte(uint8_t byte)
{
  irqstate_t flags;

  for (; ; )
    {
      while (UART_GET_TX_BUF_CNT() > 7);

      flags = enter_critical_section();

      if (UART_GET_TX_BUF_CNT() <= 7)
        {
          UART_BUF(uart_txindex) = byte;

          /* Cycle the four register 0x90 0x91 0x92 0x93 */

          uart_txindex++;
          uart_txindex &= 0x03;

          leave_critical_section(flags);
          return;
        }

      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: uart_dma_send
 *
 * Description:
 *   Send a buffer by dma, this function only allowed to be called when the
 *   uart dma is in idle state.
 *
 * Parameters:
 *   priv - the uart driver private struct
 *   buf  - the data buffer pointer
 *   len  - the data length
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TXDMA
static void uart_dma_send(uart_priv_t *priv, char *buf, size_t len)
{
  /* Get the real data pointer */

  char *data = &priv->txdmabuf[DMA_HEAD_LEN];

  /* Buffer is too large, assert here */

  if (len + DMA_HEAD_LEN > priv->txdmasize)
    {
      DEBUGASSERT(false);
      return;
    }

  /* Copy the driver buffer to dma buffer, this is necessary because
   * the first 4 bytes of the buffer must be the transfered size, which
   * is telin dma peripheral required.
   * NOTE: the memcpy better be ramcode.
   */

  memcpy(data, buf, len);

  /* Set the dma transfer length */

  priv->txdmabuf[0] = len;
  priv->txdmabuf[1] = 0;
  priv->txdmabuf[2] = 0;
  priv->txdmabuf[3] = 0;

  /* Set the dma buffer address, the high 8 byte is not need because
   * telink chip not use.
   */

  DMA_UART_TX_ADDR_REG =
    (uint16_t)((uint32_t)priv->txdmabuf & 0x0000ffff);
  DMA_UART_TX_ADDRHI_REG =
    (uint8_t)(((uint32_t)priv->txdmabuf >> 16) & 0x000000ff);

  /* Set uart tx channel be ready for transfer */

  DMA_TX_RDY0_REG |= DMA_CHAN_UART_TX;
}
#endif

/****************************************************************************
 * Name: uart_irq_trilevel
 *
 * Description:
 *   Set the receive and transimit interrupt trigger threshold
 *
 * Parameters:
 *   rxlevel  - receive interrupt triiger threshold
 *   txlevel  - transimit interrupt triiger threshold
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static inline void uart_irq_trilevel(uint8_t rxlevel, uint8_t txlevel)
{
  UART_CTRL3_REG = rxlevel | (txlevel << 4);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_gpio_config
 *
 * Description:
 *   Config tx_pin and rx_pin as uart usage.
 *
 * Parameters:
 *   tx_pin  - uart transimit pin config
 *   rx_pin  - uart receive pin config
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static void uart_gpio_config(uint32_t tx_pin, uint32_t rx_pin)
{
  /* When the pad is configured with mux input and a pull-up resistor is
   * required, tlsr82_gpio_input_ctrl() needs to be placed before
   * tlsr82_gpioconfig, otherwise, the mux pad will may misread the short
   * low-level timing.
   */

  tlsr82_gpio_input_ctrl(tx_pin, true);
  tlsr82_gpio_input_ctrl(rx_pin, true);

  /* The pullup setting must before uart gpio config, cause it will lead
   * to ERR data to uart RX buffer (PM_PIN_PULLUP_1M or PM_PIN_PULLUP_10K),
   * then set Tx pin and RX pin mux function
   */

  tlsr82_gpioconfig(tx_pin | UART0_PIN_TX_MUX | GPIO_PUPD_PU10K);
  tlsr82_gpioconfig(rx_pin | UART0_PIN_RX_MUX | GPIO_PUPD_PU10K);
}

/****************************************************************************
 * Name: uart_baudrate_config
 *
 * Description:
 *   Config the uart baudrate
 *
 * Parameters:
 *   baudrate  - excepted baudrate
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static void uart_baudrate_config(uint32_t baudrate)
{
  int i;
  int j;
  int bwpc = 0;
  int clkdiv;
  int clkdivp1_arr[2];
  int tmp;
  int min;

  /* Caculate the uart clkdiv and bit width
   *     baudrate = CPU_CLK / ((clkdiv + 1) * (bwpc + 1)), 3 <= bwpc <= 15
   */

  min = INT32_MAX;
  for (i = 3; i <= 15; i++)
    {
      /* Consider the round up and round down clkdiv value */

      clkdivp1_arr[0] = CPU_CLK / (baudrate * (i + 1));
      clkdivp1_arr[1] = clkdivp1_arr[0] + 1;
      for (j = 0; j < 2; j++)
        {
          if (clkdivp1_arr[j] == 0)
            {
              continue;
            }

          tmp = CPU_CLK / clkdivp1_arr[j];
          tmp = abs(tmp / (i + 1) - baudrate);

          /* Get the clkdiv and bwpc that make the smallest difference
           * between expected baudrate and real baudrate, mealwhile the
           * clkdiv should be smaller as soon as possible to get a larger
           * uart clock, whick leads more precise baudrate.
           */

          if ((tmp == min && i > bwpc) || (tmp < min))
            {
              min    = tmp;
              bwpc   = i;
              clkdiv = clkdivp1_arr[j] - 1;
            }
        }
    }

  /* Set the bwpc (first clear) */

  BM_CLR(UART_CTRL0_REG, UART_CTRL0_BWPC);
  BM_SET(UART_CTRL0_REG, bwpc << UART_CTRL0_BWPC_SHIFT);

  /* Set the Clock Divider and Enable clock */

  UART_CLK_REG = (clkdiv & UART_CLK_DIV) | UART_CLK_EN;

  /* Set Rx timeout numbers, one byte includes 12 bits at most */

  UART_RXTIMEOUT0_REG = (bwpc + 1) * 12;
  BM_CLR(UART_RXTIMEOUT1_REG, UART_RXTIMEOUT1_SEL);
  BM_SET(UART_RXTIMEOUT1_REG, 2 << UART_RXTIMEOUT1_SEL_SHIFT);
}

/****************************************************************************
 * Name: uart_parity_config
 *
 * Description:
 *   Config the uart parity
 *
 * Parameters:
 *   parity  - 0                  , no parity
 *             odd number         , odd parity
 *             even number (not 0), even parity
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static void uart_parity_config(int parity)
{
  if (parity)
    {
      BM_SET(UART_CTRL1_REG, UART_CTRL1_PARITY_EN);
      if (parity % 2 == 1)
        {
          /* Odd parity */

          BM_CLR(UART_CTRL1_REG, UART_CTRL1_PARITY_SEL);
        }
      else
        {
          /* Even parity */

          BM_SET(UART_CTRL1_REG, UART_CTRL1_PARITY_SEL);
        }
    }
  else
    {
      BM_CLR(UART_CTRL1_REG, UART_CTRL1_PARITY_EN);
    }
}

/****************************************************************************
 * Name: uart_stopbits_config
 *
 * Description:
 *   Config the uart stop bits
 *
 * Parameters:
 *   stopbits  - 0, one stop bit
 *             - 1, one and a halp stop bit
 *             - 2, two stop bits
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static void uart_stopbits_config(int stopbits)
{
  BM_CLR(UART_CTRL1_REG, UART_CTRL1_STOPBIT);
  BM_SET(UART_CTRL1_REG, stopbits << UART_CTRL1_STOPBIT_SHIFT);
}

/****************************************************************************
 * Name: uart_dma_config
 *
 * Description:
 *   Disable the dma function for uart
 *
 * Parameters:
 *   void
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

static void uart_dma_config(uart_priv_t *priv)
{
  /* Disable dma interrupt */

  up_disable_irq(NR_DMA_IRQ);

#ifdef CONFIG_SERIAL_TXDMA
  if (priv->txdma)
    {
      /* Enable the dma function in uart */

      BM_SET(UART_CTRL0_REG, UART_CTRL0_TX_DMA_EN);

      /* Enable the uart tx dma channel in dma */

      BM_SET(DMA_IRQ_EN_REG, DMA_CHAN_UART_TX);
    }
  else
#endif
    {
      BM_CLR(UART_CTRL0_REG, UART_CTRL0_TX_DMA_EN);
      BM_CLR(DMA_IRQ_EN_REG, DMA_CHAN_UART_TX);
    }

#ifdef CONFIG_SERIAL_RXDMA
  if (priv->rxdma)
    {
      /* Enable the dma function in uart */

      BM_SET(UART_CTRL0_REG, UART_CTRL0_RX_DMA_EN);

      /* Enable the uart rx dma channel in dma */

      BM_SET(DMA_IRQ_EN_REG, DMA_CHAN_UART_RX);

      /* Set the rx dma buffer, this configuration must be earlier than
       * uart configuration, such as BaudRate, Parity etc.
       */

      DMA_UART_RX_ADDR_REG =
        (uint16_t)((uint32_t)priv->rxdmabuf & 0x0000ffff);
      DMA_UART_RX_ADDRHI_REG =
        (uint8_t)(((uint32_t)priv->rxdmabuf >> 16) & 0x000000ff);
      DMA_UART_RX_SIZE_REG = priv->rxdmasize / 16;

      /* Set dma mode to 0x01 for receive */

      DMA_UART_RX_MODE_REG |= DMA_MODE_WR_MEM;
    }
  else
#endif
    {
      BM_CLR(UART_CTRL0_REG, UART_CTRL0_RX_DMA_EN);
      BM_CLR(DMA_IRQ_EN_REG, DMA_CHAN_UART_RX);
    }

  /* Disable uart dma interrupt, interrupt will be enabled when attach
   * the interrupt.
   */

  BM_CLR(DMA_IRQ_MASK_REG, DMA_CHAN_UART_RX | DMA_CHAN_UART_TX);
}

/****************************************************************************
 * Name: tlsr82_uart_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int tlsr82_uart_setup(struct uart_dev_s *dev)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

  /* Uart Dma config, follow the telink sdk, the dma receive buffer
   * configuration must be earlier than uart configuration, so put
   * dma_config earlier than uart config.
   */

  uart_dma_config(priv);

  /* Uart Gpio config */

  uart_gpio_config(priv->txpin, priv->rxpin);

  /* Reset the uart */

  uart_reset();

  /* Uart communication parameters config
   * TODO: unity below functions to uart_format_config()
   */

  uart_baudrate_config(priv->baud);
  uart_parity_config(priv->parity);
  uart_stopbits_config(priv->stopbits);

  /* Diable uart rx_buff and tx_buff irq */

  uart_irq_rx_enable(false);
  uart_irq_tx_enable(false);

  /* Rx trigger level = 1 indicates one byte will cause interrupt
   * Tx trigger level = 0 indicates tx buff empty will cause interrupt
   */

  uart_irq_trilevel(1, 0);

  /* open uart_error_mask, when stop bit error or parity error,
   * it will enter error_interrupt
   */

  /* uart_mask_error_irq_enable(); */

  return 0;
}

/****************************************************************************
 * Name: tlsr82_uart_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void tlsr82_uart_shutdown(struct uart_dev_s *dev)
{
  UNUSED(dev);
}

/****************************************************************************
 * Name: tlsr82_uart_interrupt
 *
 * Description:
 *   This is the UART status interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int UART_RAMCODE tlsr82_interrupt(int irq, void *context, void *arg)
{
  UNUSED(irq);
  UNUSED(context);

  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uart_priv_t *     priv = (uart_priv_t *)dev->priv;

  if ((UART_BUF_CNT1_REG & UART_BUF_CNT1_RX_ERR))
    {
      /* It will clear rx_fifo and rx_err_irq, rx_buff_irq, so it won't enter
       * rx_buff interrupt.
       */

      uart_irq_clr(priv->port, UART_IRQ_CLR_RX);

      /* uart_reset() clear hardware pointer, and clear software pointer */

      uart_reset();
      uart_clr_rx_index(priv->port);
    }

#ifdef CONFIG_SERIAL_TXDMA
  if (priv->txdma && uart_get_irq_status(priv->port, UART_IRQ_TXDONE) &&
      uart_txdoneirq_isenable(priv->port))
    {
      /* Clear txdone interrupt */

      UART_STATE_REG = BIT(7);

      /* Adjust the send bytes */

      dev->dmatx.nbytes += dev->dmatx.length;
      if (dev->dmatx.nlength)
        {
          /* Dma send */

          uart_dma_send(priv, dev->dmatx.nbuffer, dev->dmatx.nlength);

          /* Set length for next next completion */

          dev->dmatx.length  = dev->dmatx.nlength;
          dev->dmatx.nlength = 0;
        }
      else
        {
          /* No data to send, call uart_xmitchars_done() */

          uart_xmitchars_done(dev);

          /* Post the tx dma sem, the next dma transfer can start */

          nxsem_post(priv->txdmasem);
        }
    }
#else
  if (uart_get_irq_status(priv->port, UART_IRQ_TXBUF) &&
      uart_txbufirq_isenable(priv->port))
    {
      uart_xmitchars(dev);
    }
#endif

  if (uart_get_irq_status(priv->port, UART_IRQ_RXBUF) &&
      uart_rxbufirq_isenable(priv->port))
    {
      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: tlsr82_dma_interrupt
 *
 * Description:
 *   This is the UART dma interrupt handler. When the dma transfer finish,
 *   this interrupt will occur. In this interrupt, we should call
 *   uart_recvchars() to receive the char from the dma buffer.
 *
 ****************************************************************************/

#if defined(CONFIG_SERIAL_RXDMA)
static int UART_RAMCODE tlsr82_dma_interrupt(int irq, void *context,
                                             void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uart_priv_t *     priv = (uart_priv_t *)dev->priv;

  /* Check the uart dma rx interrupt status */

  if ((DMA_IRQ_STA_REG & DMA_CHAN_UART_RX) &&
      (DMA_IRQ_MASK_REG & DMA_CHAN_UART_RX))
    {
      /* Write 1 to clear the interrupt status */

      DMA_IRQ_STA_REG |= DMA_CHAN_UART_RX;

      /* The first 4 bytes in dma rx buffer indicates the received
       * data number, the real data start at DMA_HEAD_LEN
       * set rxdmalen be the total valid length (Include DMA HEADER)
       * set rxdmaindex be DMA_HEAD_LEN for skipping the DMA HEADER
       */

      priv->rxdmalen   = priv->rxdmabuf[0] + DMA_HEAD_LEN;
      priv->rxdmaindex = DMA_HEAD_LEN;

      /* Call uart_recvchars */

      uart_recvchars(dev);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: tlsr82_uart_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).
 *   The RX and TX interrupts are not enabled until the txint() and rxint()
 *   methods are called.
 *
 ****************************************************************************/

static int tlsr82_uart_attach(struct uart_dev_s *dev)
{
  int ret;
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

#ifdef CONFIG_SERIAL_RXDMA
  /* If the serial rx dma is enable, the dma interrupt must be used for
   * received.
   */

  ret = irq_attach(NR_DMA_IRQ, tlsr82_dma_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the rx dma interrupt, and Write 1 to clear the dma
       * interrupt status
       */

      DMA_IRQ_MASK_REG |= DMA_CHAN_UART_RX;
      DMA_IRQ_STA_REG  |= DMA_CHAN_UART_RX;

      up_enable_irq(NR_DMA_IRQ);
    }
#endif

  ret = irq_attach(priv->irq, tlsr82_interrupt, dev);
  if (ret == OK)
    {
      uart_irq_clr(priv->port, UART_IRQ_CLR_RX);
      uart_clr_rx_index(priv->port);

#ifndef CONFIG_SERIAL_RXDMA
      /* Enable the rx fifo not empty irq, but when serial rx dma
       * is enable, rx fito not empty interrupt is not used, so
       * disable it.
       */

      uart_irq_rx_enable(true);
#endif

#ifdef CONFIG_SERIAL_TXDMA
      /* Enable the tx dma irq and clear the txdone status */

      UART_RXTIMEOUT1_REG |= UART_RXTIMEOUT1_MASK_TXDONE;
      UART_STATE_REG       = BIT(7);
#endif

      up_enable_irq(NR_UART_IRQ);
    }

  return ret;
}

/****************************************************************************
 * Name: tlsr82_uart_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void tlsr82_uart_detach(struct uart_dev_s *dev)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach from the interrupt(s) */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: tlsr82_uart_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int tlsr82_uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  UNUSED(filep);
  UNUSED(cmd);
  UNUSED(arg);

  return -ENOTTY;
}

/****************************************************************************
 * Name: tlsr82_uart_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int UART_RAMCODE tlsr82_uart_receive(struct uart_dev_s *dev,
                                            unsigned int *status)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

  /* Return receiver control information */

  if (status)
    {
      *status = 0x00;
    }

  /* Then return the actual received data. */

  return uart_read_byte(priv->port);
}

/****************************************************************************
 * Name: tlsr82_uart_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void tlsr82_uart_rxint(struct uart_dev_s *dev, bool enable)
{
  uart_irq_rx_enable(enable);
}

/****************************************************************************
 * Name: tlsr82_uart_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool UART_RAMCODE tlsr82_uart_rxavailable(struct uart_dev_s *dev)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

  /* Return true if the receive buffer/fifo is not "empty." */

  return 0 != uart_get_rxfifo_num(priv->port);
}

/****************************************************************************
 * Name: tlsr82_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void tlsr82_uart_txint(uart_dev_t *dev, bool enable)
{
  uart_irq_tx_enable(enable);
}

/****************************************************************************
 * Name: tlsr82_uart_dma_rxint
 *
 * Description:
 *   Control the uart rx interrupt in UART RX DMA mode, but here nothing
 *   to be done.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_RXDMA
static void tlsr82_uart_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  /* Nothing */
}
#endif

/****************************************************************************
 * Name: tlsr82_uart_dma_receive
 *
 * Description:
 *   Receive the data from uart rx dma buffer, and increase the rx dma index
 *
 * Parameters:
 *   dev    - uart driver device pointer
 *   status - driver not used
 *
 * Returned Values:
 *   received char
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_RXDMA
static int  UART_RAMCODE tlsr82_uart_dma_receive(struct uart_dev_s *dev,
                                                 unsigned int * status)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;
  int ch;

  ch = (int)priv->rxdmabuf[priv->rxdmaindex];
  priv->rxdmaindex++;

  return ch;
}
#endif

/****************************************************************************
 * Name: tlsr82_uart_dma_rxavail
 *
 * Description:
 *   Receive the data from uart rx dma buffer, and increase the rx dma index
 *
 * Parameters:
 *   dev    - uart driver device pointer
 *
 * Returned Values:
 *   true : uart rx available
 *   false: uart rx not available
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_RXDMA
static bool UART_RAMCODE tlsr82_uart_dma_rxavail(struct uart_dev_s *dev)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

  return priv->rxdmaindex < priv->rxdmalen;
}
#endif

/****************************************************************************
 * Name: tlsr82_uart_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void UART_RAMCODE tlsr82_uart_send(struct uart_dev_s *dev, int ch)
{
  uart_send_byte((uint8_t)ch);
}

/****************************************************************************
 * Name: tlsr82_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool UART_RAMCODE tlsr82_uart_txready(struct uart_dev_s *dev)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

  /* Return true if the transmit FIFO is "not full." */

  return uart_get_txfifo_num(priv->port) < 8;
}

/****************************************************************************
 * Name: tlsr82_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool UART_RAMCODE tlsr82_uart_txempty(struct uart_dev_s *dev)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

  /* Return true if the transmit FIFO is "empty." */

  return 0 == uart_get_txfifo_num(priv->port);
}

/****************************************************************************
 * Name: tlsr82_uart_dma_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts from the UART.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TXDMA
static void tlsr82_uart_dma_txint(struct uart_dev_s *dev, bool enable)
{
  /* Nothing to do. */

  /* In case of DMA transfer we do not want to make use of UART interrupts.
   * Instead, we use DMA interrupts that are activated once during boot
   * sequence. Furthermore we can use up_dma_txcallback() to handle staff at
   * half DMA transfer or after transfer completion (depending configuration,
   * see stm32_dmastart(...) ).
   */
}
#endif

/****************************************************************************
 * Name: tlsr82_uart_dma_send
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TXDMA
static void tlsr82_uart_dma_send(struct uart_dev_s *dev)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

  uart_dma_send(priv, dev->dmatx.buffer, dev->dmatx.length);
}
#endif

/****************************************************************************
 * Name: tlsr82_uart_dma_txavail
 *
 * Description:
 *   Informs DMA that Tx data is available and is ready for transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_TXDMA
static void tlsr82_uart_dma_txavail(struct uart_dev_s *dev)
{
  uart_priv_t *priv = (uart_priv_t *)dev->priv;

  if (dev->xmit.head == dev->xmit.tail)
    {
      /* No data to transfer. */

      return;
    }

  /* Wait for the previous dma transfer finish */

  nxsem_wait(priv->txdmasem);

  uart_xmitchars_dma(dev);
}
#endif

/****************************************************************************
 * Name: tlsr82_uart_lowputc
 *
 * Description:
 *   Direct put the char into the uart tx fifo
 *
 * Parameters:
 *   ch - output char
 *
 * Return Value:
 *   Output char.
 *
 ****************************************************************************/

static inline int tlsr82_uart_lowputc(int ch)
{
  /* Wait until the tx fifo is not full */

  while (uart_get_txfifo_num(CONSOLE_PORT) > 7);

  tlsr82_uart_send(&CONSOLE_DEV, ch);

  return ch;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ****************************************************************************/

#ifndef CONFIG_TLSR82_SPI_SYSLOG
int up_putc(int ch)
{
  if (ch == '\n')
    {
      /* Add CR */

      tlsr82_uart_lowputc('\r');
    }

  tlsr82_uart_lowputc(ch);

  return 0;
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
  /* Register the serial console */

#ifndef CONFIG_TLSR82_SPI_CONSOLE
  CONSOLE_DEV.isconsole = true;
  uart_register("/dev/console", &CONSOLE_DEV);
#else
  spi_console_init();
#endif /* CONFIG_TLSR82_SPI_CONSOLE */

  /* Register the serial driver */

  uart_register("/dev/ttyS0", &g_uart0_dev);
}

/****************************************************************************
 * Name: tlsr82_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm_serialinit.
 *
 ****************************************************************************/

void tlsr82_earlyserialinit(void)
{
  tlsr82_uart_setup(&CONSOLE_DEV);
}
