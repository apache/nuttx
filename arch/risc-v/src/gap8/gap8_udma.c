/****************************************************************************
 * arch/risc-v/src/gapuino/gap8_udma.c
 * uDMA driver for GAP8
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ****************************************************************************/

/****************************************************************************
 *  GAP8 features a simple uDMA subsystem. Peripherals including UART, SPI,
 *  I2C, I2S, CPI, LVDS, Hyperbus, have config registers memory-mapped, but
 *  not data registers.  The only way to send or receive data is using the
 *  uDMA. Those peripherals share the same uDMA ISR.
 *
 *  Note that uDMA can only recognize L2 RAM. So data must not be stored at
 *  L1, which means that if you link the stack at L1, you cannot use local
 *  buffers as data src or destination.
 *
 *  As for the UART driver, the SOC_EU may fire a redundant IRQ even if the
 *  uDMA hasn't finished its job. So I spin on TX channel and bypass on RX
 *  channel, if the IRQ is redundant.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gap8_udma.h"
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CHECK_CHANNEL_ID(INSTANCE) \
  if ((INSTANCE) == NULL || \
      (INSTANCE)->id >= GAP8_UDMA_NR_CHANNELS) \
    { \
      return ERROR; \
    }

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* uDMA peripheral instances
 * The peripheral driver instantiate it and register through _init()
 */

static struct gap8_udma_peripheral *_peripherals[GAP8_UDMA_NR_CHANNELS] =
                                                                          {
                                                                            0
                                                                          };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void _dma_txstart(struct gap8_udma_peripheral *the_peri)
{
  the_peri->regs->TX_SADDR = (uint32_t)the_peri->tx.buff;
  the_peri->regs->TX_SIZE  = (uint32_t)the_peri->tx.block_size;
  the_peri->regs->TX_CFG = UDMA_CFG_EN(1);
}

static void _dma_rxstart(struct gap8_udma_peripheral *the_peri)
{
  the_peri->regs->RX_SADDR = (uint32_t)the_peri->rx.buff;
  the_peri->regs->RX_SIZE  = (uint32_t)the_peri->rx.block_size;
  the_peri->regs->RX_CFG = UDMA_CFG_EN(1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gap8_udma_init
 *
 * Description:
 *   Initialize (and enable) a udma peripheral.
 *
 * Input:
 *   instance: pointer to a peripheral instance connected to uDMA
 *
 ****************************************************************************/

int gap8_udma_init(struct gap8_udma_peripheral *instance)
{
  uint32_t id;

  CHECK_CHANNEL_ID(instance)

  id = instance->id;
  _peripherals[id] = instance;

  /* Enable clock gating */

  UDMA_GC->CG |= (1L << id);

  return OK;
}

/****************************************************************************
 * Name: gap8_udma_deinit
 *
 * Description:
 *   Deinit a udma peripheral
 *
 ****************************************************************************/

int gap8_udma_deinit(struct gap8_udma_peripheral *instance)
{
  uint32_t id;

  CHECK_CHANNEL_ID(instance)

  id = instance->id;
  _peripherals[id] = NULL;

  /* Disable clock gating */

  UDMA_GC->CG &= ~(1L << id);

  return OK;
}

/****************************************************************************
 * Name: gap8_udma_tx_setirq
 *
 * Description:
 *   Enable or disable the tx interrupt.
 *
 ****************************************************************************/

int gap8_udma_tx_setirq(struct gap8_udma_peripheral *instance, bool enable)
{
  CHECK_CHANNEL_ID(instance)

  /* The irq enable bit happened to be 2*id + 1 */

  if (enable)
    {
      up_enable_event(1 + (instance->id << 1));
    }
  else
    {
      up_disable_event(1 + (instance->id << 1));
#if 0
      instance->regs->TX_CFG = UDMA_CFG_CLR(1);
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: gap8_udma_rx_setirq
 *
 * Description:
 *   Enable or disable the rx interrupt.
 *
 ****************************************************************************/

int gap8_udma_rx_setirq(struct gap8_udma_peripheral *instance, bool enable)
{
  CHECK_CHANNEL_ID(instance)

  /* The irq enable bit happened to be 2*id */

  if (enable)
    {
      up_enable_event(instance->id << 1);
    }
  else
    {
      up_disable_event(instance->id << 1);
#if 0
      instance->regs->RX_CFG = UDMA_CFG_CLR(1);
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: gap8_udma_tx_start
 *
 * Description:
 *   Send size * count bytes non-blocking.
 *
 *   This function may be called from ISR, so it cannot be blocked. The
 *   caller should manage the muxing.
 *
 ****************************************************************************/

int gap8_udma_tx_start(struct gap8_udma_peripheral *instance,
                       uint8_t *buff, uint32_t size, int count)
{
  CHECK_CHANNEL_ID(instance)

  instance->tx.buff = buff;
  instance->tx.block_size = size;
  instance->tx.block_count = count;

  _dma_txstart(instance);

  return OK;
}

/****************************************************************************
 * Name: gap8_udma_rx_start
 *
 * Description:
 *   Receive size * count bytes
 *
 *   This function may be called from ISR, so it cannot be blocked. The
 *   caller should manage the muxing.
 *
 ****************************************************************************/

int gap8_udma_rx_start(struct gap8_udma_peripheral *instance,
                       uint8_t *buff, uint32_t size, int count)
{
  CHECK_CHANNEL_ID(instance)

  instance->rx.buff = buff;
  instance->rx.block_size = size;
  instance->rx.block_count = count;

  _dma_rxstart(instance);

  return OK;
}

/****************************************************************************
 * Name: gap8_udma_tx_poll
 *
 * Description:
 *   Return OK if the buffer is not in the tx pending list.
 *
 ****************************************************************************/

int gap8_udma_tx_poll(struct gap8_udma_peripheral *instance)
{
  CHECK_CHANNEL_ID(instance)

  return instance->tx.block_count <= 0 ? OK : ERROR;
}

/****************************************************************************
 * Name: gap8_udma_rx_poll
 *
 * Description:
 *   Return OK if the buffer is not in the rx pending list.
 *
 ****************************************************************************/

int gap8_udma_rx_poll(struct gap8_udma_peripheral *instance)
{
  CHECK_CHANNEL_ID(instance)

  return instance->rx.block_count <= 0 ? OK : ERROR;
}

/****************************************************************************
 * Name: gap8_udma_doirq
 *
 * Description:
 *   uDMA ISR
 *
 ****************************************************************************/

int gap8_udma_doirq(int irq, void *context, FAR void *arg)
{
  struct gap8_udma_peripheral *the_peripheral;
  uint32_t event = SOC_EVENTS->CURRENT_EVENT & 0xff;

  if (event > GAP8_UDMA_MAX_EVENT)
    {
      /* Bypass */

      return OK;
    }

  /* Peripheral id happened to be half of event... */

  the_peripheral = _peripherals[event >> 1];
  if (the_peripheral == NULL)
    {
      return OK;
    }

  if (event & 0x1)
    {
      /* Tx channel */

      if (the_peripheral->tx.block_count > 1)
        {
          the_peripheral->tx.block_count--;
          the_peripheral->tx.buff += the_peripheral->tx.block_size;
          _dma_txstart(the_peripheral);
        }
      else
        {
          /* The buffer is exhausted. Forward to peripheral's driver */

          while (the_peripheral->regs->TX_SIZE != 0)
            {
              /* I don't know why but I have to spin here. SOC_EU would
               * fire the IRQ even if uDMA hasn't finished the job.
               * If I bypassed it, I would lose the finish event...
               */
            }

          the_peripheral->tx.block_count = 0;
          if (the_peripheral->on_tx)
            {
              the_peripheral->on_tx(the_peripheral->tx_arg);
            }
        }
    }
  else
    {
      /* Rx channel */

      if (the_peripheral->rx.block_count > 1)
        {
          the_peripheral->rx.block_count--;
          the_peripheral->rx.buff += the_peripheral->rx.block_size;
          _dma_rxstart(the_peripheral);
        }
      else if (!(the_peripheral->regs->RX_CFG & UDMA_CFG_CLR(1)))
        {
          /* The buffer is exhausted. Forward to peripheral's driver
           *
           * Again I don't know why but I have to test the PENDING bit of
           * the uDMA, so as to avoid the redundant IRQ...
           */

          the_peripheral->rx.block_count = 0;
          if (the_peripheral->on_rx)
            {
              the_peripheral->on_rx(the_peripheral->rx_arg);
            }
        }
    }

  return OK;
}
