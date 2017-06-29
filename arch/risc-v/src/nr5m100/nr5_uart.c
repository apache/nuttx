/************************************************************************************
 * arch/risc-v/src/nr5m100/nr5_uart.c
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "nr5.h"

#define MAKE_UINT32(a,b,c,d)  (((a) << 24) | ((b) << 16) | ((c) << 8) | d)

struct nr5_uart_buffer_s
{
  uint16_t    head;
  uint16_t    tail;
  uint16_t    size;
  char      * buffer;
};

struct nr5_uart_regs_s
{
   uint32_t* pBaud;        // Data status port
   uint32_t* pStat;        // Data status port
   uint8_t*  pTx;          // Data TX port
   uint8_t*  pRx;          // Data RX port
   uint32_t* pIntCtrl;     // Interrupt enable control
   int       rx_irq;       // IRQ number
   int       tx_irq;       // IRQ number
};

struct nr5_uart_s
{
  volatile struct nr5_uart_regs_s * regs;
  struct nr5_uart_buffer_s        * tx_buf;
  struct nr5_uart_buffer_s        * rx_buf;
};

/*
==============================================================================
Static global pointers to access the hardware
==============================================================================
*/

#ifdef CONFIG_NR5_HAVE_UART1
static char g_uart1_rx_buf[CONFIG_NR5_UART_RX_BUF_SIZE];
static char g_uart1_tx_buf[CONFIG_NR5_UART_TX_BUF_SIZE];

static struct nr5_uart_buffer_s g_nr5_uart1_rx_buf =
{
  .head = 0,
  .tail = 0,
  .size = CONFIG_NR5_UART_RX_BUF_SIZE,
  .buffer = g_uart1_rx_buf,
};

static struct nr5_uart_buffer_s g_nr5_uart1_tx_buf =
{
  .head = 0,
  .tail = 0,
  .size = CONFIG_NR5_UART_TX_BUF_SIZE,
  .buffer = g_uart1_tx_buf,
};

static volatile struct nr5_uart_regs_s g_nr5_uart1_regs =
{
  .pBaud    = (uint32_t *) NR5_UART1_BAUD_RATE_REG,
  .pStat    = (uint32_t *) NR5_UART1_STATUS_REG,
  .pRx      = (uint8_t *)  NR5_UART1_RX_REG,
  .pTx      = (uint8_t *)  NR5_UART1_TX_REG,
  .pIntCtrl = (uint32_t *) NR5_UART1_CTRL_REG,
  .rx_irq   = NR5_IRQ_UART1_RX,
  .tx_irq   = NR5_IRQ_UART1_TX,
};

static struct nr5_uart_s g_nr5_uart1 =
{
  .regs = &g_nr5_uart1_regs,
  .rx_buf = &g_nr5_uart1_rx_buf,
  .tx_buf = &g_nr5_uart1_tx_buf,
};
#endif

/*
==============================================================================
ISR for NanoRisc5 UART RX availalbe.
==============================================================================
*/
int nr5_uart_rx_isr(int irq_num, void *context)
{
  struct nr5_uart_s *dev = NULL;
  char rxdata;

#ifdef CONFIG_NR5_HAVE_UART1
  if (irq_num == g_nr5_uart1_regs.rx_irq)
    {
      dev = &g_nr5_uart1;
    }
#endif

  /* Process the data */

  if (dev != NULL)
    {
      /* Read the RX byte */

      rxdata = *dev->regs->pRx;
      *dev->regs->pTx = rxdata;

      dev->rx_buf->buffer[dev->rx_buf->head++] = rxdata;
      if (dev->rx_buf->head == dev->rx_buf->size)
        dev->rx_buf->head = 0;
    }

  return 0;
}

/*
==============================================================================
Routine to initialize the HAL layer.  Must be called prior to any other
HAL function.
==============================================================================
*/
void nr5_uart_init(int uart)
{
  volatile struct nr5_uart_s *dev = NULL;
  uint32_t cmpval = MAKE_UINT32('F', 'P', 'G', 'A');

  switch (uart)
  {
#ifdef CONFIG_NR5_HAVE_UART1
    case 1:
      dev = &g_nr5_uart1;
#endif
  }

  // If a device was selected above, then initilize it
  //
  if (dev != NULL)
    {
      /* Attache the ISR and enable the IRQ with the EPIC */

      //irq_attach(dev->regs->rx_irq, &nr5_uart_rx_isr, NULL);
      //up_enable_irq(dev->regs->rx_irq);

      // Set the baud rate

      if (up_getimpid() == cmpval)
        {
          *dev->regs->pBaud = 0x0d;
        }

      /* Now enable the RX IRQ in the UART peripheral */

      //*dev->regs->pIntCtrl = NR5_UART_CTRL_ENABLE_RX_IRQ;
    }
}

/*
==============================================================================
Routine to get RX byte from console UART
==============================================================================
*/
uint8_t nr5_uart_get_rx()
{
  uint8_t rxdata = 0;

  up_disableints();
  if (g_nr5_uart1.rx_buf->head != g_nr5_uart1.rx_buf->tail)
    {
      struct nr5_uart_buffer_s *pBuf = g_nr5_uart1.rx_buf;

      rxdata = pBuf->buffer[pBuf->tail++];
      if (pBuf->tail == pBuf->size)
        pBuf->tail = 0;
    }
  up_enableints();
  return rxdata;
}

/*
==============================================================================
Routine to test if RX byte available at console UART
==============================================================================
*/
int nr5_uart_test_rx_avail()
{
  struct nr5_uart_buffer_s *pBuf = g_nr5_uart1.rx_buf;
  int  avail;

  up_disableints();
  avail= !(pBuf->head == pBuf->tail);
  up_enableints();

  /* If no RX data available then halt the processor until an interrupt */
  if (!avail)
     __asm__ volatile ("wfi");

  return avail;
}

/*
==============================================================================
Routine to test if RX byte available at console UART
==============================================================================
*/
int nr5_uart_test_tx_empty()
{
   return *g_nr5_uart1.regs->pStat & NR5_UART_STATUS_TX_EMPTY;
}

/*
==============================================================================
Routine to send TX byte to console UART
==============================================================================
*/
void nr5_uart_put_tx(uint8_t ch)
{
   // Wait for TX to be empty
   while (!(*g_nr5_uart1.regs->pStat & NR5_UART_STATUS_TX_EMPTY))
      ;

   // Write to TX
   *g_nr5_uart1.regs->pTx = ch;
}

