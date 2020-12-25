/************************************************************************************
 * arch/risc-v/src/gap8/gap8_udma.h
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
 ************************************************************************************/

/************************************************************************************
 *  GAP8 features a simple uDMA subsystem. Peripherals including UART, SPI, I2C, I2S,
 *  CPI, LVDS, Hyperbus, have config registers memory-mapped, but not data registers.
 *  The only way to send or receive data is using the uDMA. These peripherals share
 *  the same uDMA ISR.
 *
 *  uDMA subsystem drivers are object oriented to some extend. Peripherals inherit
 *  the udma class, which handles all the data exchange stuff.
 ************************************************************************************/

#ifndef _ARCH_RISCV_SRC_GAP8_UDMA_H
#define _ARCH_RISCV_SRC_GAP8_UDMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdbool.h>

#include "gap8.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* uDMA channel ID */

#define GAP8_UDMA_ID_LVDS   0
#define GAP8_UDMA_ID_SPIM0  1
#define GAP8_UDMA_ID_SPIM1  2
#define GAP8_UDMA_ID_HYPER  3
#define GAP8_UDMA_ID_UART   4
#define GAP8_UDMA_ID_I2C0   5
#define GAP8_UDMA_ID_I2C1   6
#define GAP8_UDMA_ID_TCDM   7   /* l2 to fc-l1 */
#define GAP8_UDMA_ID_I2S    8
#define GAP8_UDMA_ID_CPI    9

/* Total udma channels */

#define GAP8_UDMA_NR_CHANNELS  10

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* Software abstraction for uDMA */

/* One round of data exchange on one channel gathered into linked list because
 * threads would request for data exchange simultaneously.
 * Private for udma driver.
 */

struct __udma_queue
{
  uint8_t *buff;         /* Memory address. either TX or RX  */
  uint32_t block_size;   /* Size of a data block in bytes    */
  int  block_count;      /* Number of blocks to send or recv */
};

/* This is the base class of uDMA subsystem. Peripherals connected to uDMA
 * should inherited this class.
 */

struct gap8_udma_peripheral
{
  /* Public */

  udma_reg_t *regs;             /* Hardware config regs */
  uint32_t id;                  /* GAP8_UDMA_ID_x */
  void (*on_tx)(void *arg);     /* tx callback */
  void (*on_rx)(void *arg);     /* rx callback */
  void *tx_arg;                 /* tx argument */
  void *rx_arg;                 /* rx argument */
  uint16_t is_tx_continous;
  uint16_t is_rx_continous;

  /* Private */

  struct __udma_queue tx;        /* TX queue */
  struct __udma_queue rx;        /* RX queue */
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: gap8_udma_init
 *
 * Description:
 *   Initialize (and enable) a udma peripheral.
 *
 * Input:
 *   instance: pointer to a peripheral instance connected to uDMA
 *
 ************************************************************************************/

int gap8_udma_init(struct gap8_udma_peripheral *instance);

/************************************************************************************
 * Name: gap8_udma_deinit
 *
 * Description:
 *   Deinit a udma peripheral
 *
 ************************************************************************************/

int gap8_udma_deinit(struct gap8_udma_peripheral *instance);

/************************************************************************************
 * Name: gap8_udma_tx_setirq
 *
 * Description:
 *   Enable or disable the tx interrupt.
 *
 ************************************************************************************/

int gap8_udma_tx_setirq(struct gap8_udma_peripheral *instance, bool enable);

/************************************************************************************
 * Name: gap8_udma_rx_setirq
 *
 * Description:
 *   Enable or disable the rx interrupt.
 *
 ************************************************************************************/

int gap8_udma_rx_setirq(struct gap8_udma_peripheral *instance, bool enable);

/************************************************************************************
 * Name: gap8_udma_tx_start
 *
 * Description:
 *   Send size * count bytes non-blocking.
 *
 * Return ERROR if unable to send. The caller should poll on execution, or register
 * a on_tx to get the signal.
 *
 ************************************************************************************/

int gap8_udma_tx_start(struct gap8_udma_peripheral *instance,
                   uint8_t *buff, uint32_t size, int count);

/************************************************************************************
 * Name: gap8_udma_rx_start
 *
 * Description:
 *   Receive size * count bytes
 *
 * Return ERROR if unable to send. The caller should poll on execution, or register
 * a on_rx to get the signal.
 *
 ************************************************************************************/

int gap8_udma_rx_start(struct gap8_udma_peripheral *instance,
                   uint8_t *buff, uint32_t size, int count);

/************************************************************************************
 * Name: gap8_udma_tx_poll
 *
 * Description:
 *   Return OK if tx finished.
 *
 ************************************************************************************/

int gap8_udma_tx_poll(struct gap8_udma_peripheral *instance);

/************************************************************************************
 * Name: gap8_udma_rx_poll
 *
 * Description:
 *   Return OK if rx finished.
 *
 ************************************************************************************/

int gap8_udma_rx_poll(struct gap8_udma_peripheral *instance);

/************************************************************************************
 * Name: gap8_udma_doirq
 *
 * Description:
 *   uDMA ISR
 *
 ************************************************************************************/

int gap8_udma_doirq(int irq, void *context, FAR void *arg);

#endif /* _ARCH_RISCV_SRC_GAP8_UDMA_H */
