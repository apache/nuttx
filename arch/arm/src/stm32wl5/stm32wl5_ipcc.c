/****************************************************************************
 * arch/arm/src/stm32wl5/stm32wl5_ipcc.c
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
#include <nuttx/ipcc.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <arm_internal.h>
#include <assert.h>
#include <debug.h>

#include <arch/stm32wl5/stm32wl5xxx_cpu1_irq.h>
#include "hardware/stm32wl5_ipcc.h"
#include "stm32wl5.h"
#include "stm32wl5_ipcc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes tx or rx of single channel in memory */

struct stm32wl5_ipcc_chan_mem_s
{
  unsigned len; /* Number of valid bytes in data[] */
  char data[];  /* Data in IPCC memory */
};

/* Internal stm32wl5 ipcc structure describing channel state. */

struct stm32wl5_ipcc_s
{
  /* Pointer to API connecting upper and lower half of the driver */

  struct ipcc_lower_s *ipcc;

  /* Physical memory address where second CPU will write data for us,
   * we will be reading from this memory.
   */

  char *rxmem;

  /* Maximum length of data that rxmem can hold. It is size of the
   * reserved space for rxmem minus sizeof(stm32wl5_ipcc_chan_mem_s.len)
   */

  unsigned rxlen;

  /* Number of bytes copied from IPCC memory to buffer. Can be less than
   * stm32wl5_ipcc_chan_mem_s.len after copy operation when buffer is full.
   * Value can persist between multiple ISR and stm32wl5_ipcc_buffer_data()
   * calls, until all data from IPCC memory is successfully buffered.
   *
   * When unbuffered version is used, this holds number of bytes already
   * read from IPCC memory. Consecutive call to read() will read from
   * this position.
   */

  unsigned rxcopied;

  /* Physical memory address where we will write data for the second
   * CPU to read.
   */

  char *txmem;

  /* Maximum length of data that txmem can hold. It is size of the
   * reserved space for txmem minus sizeof(stm32wl5_ipcc_chan_mem_s.len)
   */

  unsigned txlen;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t stm32wl5_ipcc_read(struct ipcc_lower_s *ipcc,
                                  char *buffer, size_t buflen);
static ssize_t stm32wl5_ipcc_write(struct ipcc_lower_s *ipcc,
                                   const char *buffer, size_t buflen);
#ifdef CONFIG_IPCC_BUFFERED
static ssize_t stm32wl5_ipcc_buffer_data(struct ipcc_lower_s *ipcc,
                                         struct circbuf_s *rxbuf);
static ssize_t stm32wl5_ipcc_copy_to_buffer(int chan,
                                            struct circbuf_s *rxbuf);
#endif
static int     stm32wl5_ipcc_rx_isr(int irq, void *context, void *arg);
static int     stm32wl5_ipcc_tx_isr(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct stm32wl5_ipcc_s g_ipccpriv[IPCC_NCHAN] =
{
  /* Channel 1 is always enabled when IPCC is enabled */

    {
      .rxmem = (char *)(IPCC_CHAN1_START),
      .rxlen = IPCC_CHAN1_RX_SIZE - sizeof(unsigned),
      .txmem = (char *)(IPCC_CHAN1_START + IPCC_CHAN1_RX_SIZE),
      .txlen = IPCC_CHAN1_TX_SIZE - sizeof(unsigned)
    }

#if IPCC_CHAN2
    ,
    {
      .rxmem = (char *)(IPCC_CHAN2_START),
      .rxlen = IPCC_CHAN2_RX_SIZE - sizeof(unsigned),
      .txmem = (char *)(IPCC_CHAN2_START + IPCC_CHAN2_RX_SIZE),
      .txlen = IPCC_CHAN2_TX_SIZE - sizeof(unsigned)
    }
#endif

#if IPCC_CHAN3
    ,
    {
      .rxmem = (char *)(IPCC_CHAN3_START),
      .rxlen = IPCC_CHAN3_RX_SIZE - sizeof(unsigned),
      .txmem = (char *)(IPCC_CHAN3_START + IPCC_CHAN3_RX_SIZE),
      .txlen = IPCC_CHAN3_TX_SIZE - sizeof(unsigned)
    }
#endif

#if IPCC_CHAN4
    ,
    {
      .rxmem = (char *)(IPCC_CHAN4_START),
      .rxlen = IPCC_CHAN4_RX_SIZE - sizeof(unsigned),
      .txmem = (char *)(IPCC_CHAN4_START + IPCC_CHAN4_RX_SIZE),
      .txlen = IPCC_CHAN4_TX_SIZE - sizeof(unsigned)
    }
#endif

#if IPCC_CHAN5
    ,
    {
      .rxmem = (char *)(IPCC_CHAN5_START),
      .rxlen = IPCC_CHAN5_RX_SIZE - sizeof(unsigned),
      .txmem = (char *)(IPCC_CHAN5_START + IPCC_CHAN5_RX_SIZE),
      .txlen = IPCC_CHAN5_TX_SIZE - sizeof(unsigned)
    }
#endif

#if IPCC_CHAN6
    ,
    {
      .rxmem = (char *)(IPCC_CHAN6_START),
      .rxlen = IPCC_CHAN6_RX_SIZE - sizeof(unsigned),
      .txmem = (char *)(IPCC_CHAN6_START + IPCC_CHAN6_RX_SIZE),
      .txlen = IPCC_CHAN6_TX_SIZE - sizeof(unsigned)
    }
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wl5_ipcc_tx_isr
 *
 * Description:
 *   IPCC TX interrupt service routine. This interrupt is called when
 *   second CPU read all data from IPCC TX memory and we are free to
 *   write new data to that memory.
 *
 *   For buffered IPCC, we will immediately write data from buffer to
 *   IPCC memory and notify second CPU. Also blocked writer will be
 *   notified that there is free space on tx buffer.
 *
 *   For unbuffered IPCC, we won't copy anything but only notify blocked
 *   writers that IPCC TX memory is free.
 *
 * Input Parameters:
 *   irq - Number of IRQ that generated interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg - user data (not used by us)
 *
 * Returned Value:
 *   Always OK
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

static int stm32wl5_ipcc_tx_isr(int irq, void *context, void *arg)
{
  int chan;
  uint32_t mr;
  uint32_t sr;
  uint32_t status;
  struct stm32wl5_ipcc_s *priv;
#ifdef CONFIG_IPCC_BUFFERED
  size_t nwritten;
  struct stm32wl5_ipcc_chan_mem_s *txmem;
#endif

  UNUSED(context);
  UNUSED(arg);
  UNUSED(irq);

  mr = getreg32(STM32WL5_IPCC_C1MR) >> STM32WL5_IPCC_TX_SHIFT;
  sr = getreg32(STM32WL5_IPCC_C1TOC2SR);

  /* Consider only channels that have tx memory free and are unmasked */

  status = sr | mr;

  /* Check which channels have data ready to read */

  for (chan = 0; chan != IPCC_NCHAN; chan++)
    {
      if (status & (1 << chan))
        {
          /* Transmit on channel is either masked or TX memory is not
           * ready to write (second CPU is still read data from it,
           * CHnF == 1).
           */

          continue;
        }

      /* Get internal data structure for current chan */

      priv = &g_ipccpriv[chan];

#ifdef CONFIG_IPCC_BUFFERED
      txmem = (struct stm32wl5_ipcc_chan_mem_s *)priv->txmem;

      /* Copy as much as we can into IPCC memory, circbuf won't copy
       * more than there is in the buffer.
       */

      nwritten = circbuf_read(&priv->ipcc->txbuf, txmem->data, priv->txlen);

      /* Did we write anything to TX memory? */

      if (nwritten)
        {
          /* Yes, tell another CPU that data is available to read */

          txmem->len = nwritten;
          modifyreg32(STM32WL5_IPCC_C1SCR, 0, STM32WL5_IPCC_SCR_CHNS(chan));
        }

      if (circbuf_used(&priv->ipcc->txbuf) == 0)
        {
          /* Mask tx free interrupt - if tx buffer is empty and we
           * did not write anything and we don't mask interrupt, we
           * will be constantly interrupted by tx free irq.
           */

          modifyreg32(STM32WL5_IPCC_C1MR, 0, STM32WL5_IPCC_MR_CHNFM(chan));
        }
#else /* CONFIG_IPCC_BUFFERED */
      /* In unbuffered operations we never write anything to IPCC
       * memory from interrupt context, so we have to mask interrupts,
       * or else we will constantly get TX interrupts
       */

      modifyreg32(STM32WL5_IPCC_C1MR, 0, STM32WL5_IPCC_MR_CHNFM(chan));
#endif /* CONFIG_IPCC_BUFFERED */
      /* Wake up all blocked writers that there is free space available
       * in IPCC memory (or txbuffer) to write.
       */

      ipcc_txfree_notify(priv->ipcc->upper);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32wl5_ipcc_write
 *
 * Description:
 *   Function writes buffer to IPCC memory that will be later read by
 *   second CPU.
 *
 * Input Parameters:
 *   ipcc - ipcc channel instance to write to
 *   buffer - data to write to IPCC memory
 *   buflen - number of bytes requested to write
 *
 * Returned Value:
 *   Number of bytes that has been successfully written, or 0 when no
 *   bytes could be written for any reason.
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

static ssize_t stm32wl5_ipcc_write(struct ipcc_lower_s *ipcc,
                                   const char *buffer, size_t buflen)
{
  size_t to_copy;
  struct stm32wl5_ipcc_s *priv;
  struct stm32wl5_ipcc_chan_mem_s *txmem;
  uint32_t sr;

  sr = getreg32(STM32WL5_IPCC_C1TOC2SR);

  if ((sr & (1 << ipcc->chan)))
    {
      /* CHnF == 1 means that channel is occupied and second CPU can read
       * data from it. In any case second CPU did not yet read all data
       * and we should not write to that memory. Unmask TX interrupt
       * so we are notified when we can write to memory.
       */

      modifyreg32(STM32WL5_IPCC_C1MR, STM32WL5_IPCC_MR_CHNFM(ipcc->chan), 0);
      return 0;
    }

  priv = &g_ipccpriv[ipcc->chan];
  txmem = (struct stm32wl5_ipcc_chan_mem_s *)priv->txmem;

  /* Disable TX interrupt since we will modify shared data */

  up_disable_irq(STM32WL5_IRQ_IPCC_C1_TX_IT);

  /* Copy as much as we can into IPCC memory */

  to_copy = buflen > priv->txlen ? priv->txlen : buflen;
  memcpy(txmem->data, buffer, to_copy);
  txmem->len = to_copy;

  /* Tell another CPU that data is available to read */

  modifyreg32(STM32WL5_IPCC_C1SCR, 0, STM32WL5_IPCC_SCR_CHNS(ipcc->chan));

  /* Reenable interrupts */

  modifyreg32(STM32WL5_IPCC_C1MR, STM32WL5_IPCC_MR_CHNFM(ipcc->chan), 0);
  up_enable_irq(STM32WL5_IRQ_IPCC_C1_TX_IT);

  /* Return number of successfully copied bytes to IPCC memory */

  return to_copy;
}

/****************************************************************************
 * Name: stm32wl5_ipcc_rx_isr
 *
 * Description:
 *   Interrupt service routine - this function is called when another CPU
 *   has copied data to IPCC memory. Function will wake up blocked readers.
 *
 *   If buffering is enabled, function will also try to copy all data from
 *   IPCC memory to buffer. If buffer gets full, we will copy as many bytes
 *   as we can fit into buffer, internally save how many bytes we managed
 *   to copy and set overflow flag to 1.
 *
 * Input Parameters:
 *   irq - Number of IRQ that generated interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg - user data (not used by us)
 *
 * Returned Value:
 *   Always OK
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

static int stm32wl5_ipcc_rx_isr(int irq, void *context, void *arg)
{
  int chan;
  uint32_t mr;
  uint32_t sr;
  uint32_t status;
  struct stm32wl5_ipcc_s *priv;
#ifdef CONFIG_IPCC_BUFFERED
  ssize_t nread;
#endif

  UNUSED(context);
  UNUSED(arg);
  UNUSED(irq);

  mr = getreg32(STM32WL5_IPCC_C1MR);
  sr = getreg32(STM32WL5_IPCC_C2TOC1SR);

  /* Consider only channels that have data in rx memory and are unmasked */

  status = sr & ~mr;

  /* Check which channels have data ready to read */

  for (chan = 0; chan != IPCC_NCHAN; chan++)
    {
      if (!(status & (1 << chan)))
        {
          /* Receive on channel is either masked or there is no data
           * ready for us to read (CHnF == 0)
           */

          continue;
        }

      /* Get internal data structure for current chan */

      priv = &g_ipccpriv[chan];

#ifdef CONFIG_IPCC_BUFFERED
      nread = stm32wl5_ipcc_copy_to_buffer(chan, &priv->ipcc->rxbuf);

      if (nread)
#endif /* CONFIG_IPCC_BUFFERED */
        {
          /* Wake up all blocked readers that there is data
           * available to read
           */

          ipcc_rxfree_notify(priv->ipcc->upper);
        }

#ifndef CONFIG_IPCC_BUFFERED
      /* In unbuffered mode, we never read anything in interrupt, so
       * we have to mask rxirq so we don't get that irq again.
       */

      modifyreg32(STM32WL5_IPCC_C1MR, 0, STM32WL5_IPCC_MR_CHNOM(chan));
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: stm32wl5_ipcc_read
 *
 * Description:
 *   Function will copy requests number of bytes to buffer. If there is not
 *   enough data in IPCC memory, less bytes than requests will be copied.
 *   Buflen does not have to be bigger than IPCC memory - function can be
 *   called multiple times and only new data will be transfered. If we don't
 *   have control over IPCC memory (CHnF is 0 - second CPU is writing data
 *   to memory) then it's assumed no data is there to read and 0 is returned.
 *
 * Input Parameters:
 *   ipcc - ipcc channel struct
 *   buffer - location where data shall be copied
 *   buflen - size of buffer and number of requested bytes to copy
 *
 * Returned Value:
 *   Number of bytes that have been successfully copied to buffer (which
 *   may be less than buflen), or 0 when there was no data to be copied.
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

static ssize_t stm32wl5_ipcc_read(struct ipcc_lower_s *ipcc,
                                  char *buffer, size_t buflen)
{
  size_t to_copy;
  uint32_t sr;
  struct stm32wl5_ipcc_s *priv;
  struct stm32wl5_ipcc_chan_mem_s *rxmem;

  sr = getreg32(STM32WL5_IPCC_C2TOC1SR);

  if (!(sr & (1 << ipcc->chan)))
    {
      /* CHnF == 0 means that channel is free and second CPU can write
       * data to it. In any case data is not yet ready to read - so no
       * data can be read.
       */

      return 0;
    }

  priv = &g_ipccpriv[ipcc->chan];
  rxmem = (struct stm32wl5_ipcc_chan_mem_s *)priv->rxmem;

  /* Disable RX interrupt since we will modify shared data */

  up_disable_irq(STM32WL5_IRQ_IPCC_C1_RX_IT);

  /* This function may be called multiple times to get only part
   * of data from IPCC memory, ie. There are 8 bytes of data in
   * IPCC memory and upper half calls this function 4 times, each
   * time only reading 2 bytes. Check how many bytes we can copy
   */

  to_copy = rxmem->len - priv->rxcopied;
  to_copy = to_copy > buflen ? buflen : to_copy;

  /* Copy as much data to upper half as possible */

  memcpy(buffer, rxmem->data + priv->rxcopied, to_copy);
  priv->rxcopied += to_copy;
  if (priv->rxcopied == priv->rxlen)
    {
      /* We have copied all data from IPCC memory */

      priv->rxcopied = 0;

      /* Tell another CPU that IPCC rx buffer is free to be populated */

      modifyreg32(STM32WL5_IPCC_C1SCR, 0,
                  STM32WL5_IPCC_SCR_CHNC(ipcc->chan));

      /* Unmask RX interrupt to know when second CPU sends us a message */

      modifyreg32(STM32WL5_IPCC_C1MR, STM32WL5_IPCC_MR_CHNOM(ipcc->chan), 0);
    }

  /* Reenable interrupt */

  up_enable_irq(STM32WL5_IRQ_IPCC_C1_RX_IT);

  return to_copy;
}

/****************************************************************************
 * Name: stm32wl5_ipcc_copy_to_buffer
 *
 * Description:
 *   Copies as much bytes from channel as possible to rxbuf circ buffer.
 *
 * Input Parameters:
 *   chan - channel number to get data from
 *   rxbuf - circural buffer to copy data to
 *
 * Returned Value:
 *   Number of bytes copied to rxbuf
 *
 * Assumptions/Limitations:
 *   This is helper function, it does not perform any locking. It may
 *   be called from interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_IPCC_BUFFERED
static ssize_t stm32wl5_ipcc_copy_to_buffer(int chan,
                                            struct circbuf_s *rxbuf)
{
  size_t to_copy;
  size_t rxbuf_space;
  struct stm32wl5_ipcc_s *priv;
  struct stm32wl5_ipcc_chan_mem_s *rxmem;
  uint32_t sr;

  sr = getreg32(STM32WL5_IPCC_C2TOC1SR);

  if (!(sr & (1 << chan)))
    {
      /* CHnF == 0 means that channel is free and second CPU can write
       * data to it. In any case data is not yet ready to read - so no
       * data can be read.
       */

      return 0;
    }

  priv = &g_ipccpriv[chan];
  rxmem = (struct stm32wl5_ipcc_chan_mem_s *)priv->rxmem;

  /* If buffer is full, it's possible we did not copy everything from
   * IPCC memory to buffer in previous interrupt. Then when another
   * channel triggers interrupt we may only need to copy what is left
   * in IPCC memory to buffer.
   */

  to_copy = rxmem->len - priv->rxcopied;
  rxbuf_space = circbuf_space(rxbuf);

  if (to_copy > rxbuf_space)
    {
      /* we can't fit all data into buffer, copy as much as
       * possible and set overflow flag to 1, to tell upper
       * half that there is still data in IPCC memory.
       */

      to_copy = rxbuf_space;
      priv->ipcc->overflow = 1;

      /* Also disable RX interrupt. If data is still in the ipcc
       * memory and we don't set rxbuffer as free, we will
       * immediately get another RX interrupt once we leave
       * this one.
       */

      modifyreg32(STM32WL5_IPCC_C1MR, 0, STM32WL5_IPCC_MR_CHNOM(chan));
    }

  /* Buffer data. This function cannot really fail us if we
   * pass valid input parameters.
   */

  circbuf_write(&priv->ipcc->rxbuf, rxmem->data + priv->rxcopied, to_copy);

  /* Increment number of bytes that has been buffered */

  if ((priv->rxcopied += to_copy) == rxmem->len)
    {
      /* We have buffered all data from IPCC memory */

      priv->rxcopied = 0;
      priv->ipcc->overflow = 0;

      /* Tell another CPU that IPCC rx buffer is free to be populated */

      modifyreg32(STM32WL5_IPCC_C1SCR, 0, STM32WL5_IPCC_SCR_CHNC(chan));

      /* Unmask RX interrupt to know when second CPU sends us a message */

      modifyreg32(STM32WL5_IPCC_C1MR, STM32WL5_IPCC_MR_CHNOM(chan), 0);
    }

  return to_copy;
}
#endif

/****************************************************************************
 * Name: stm32wl5_ipcc_buffer_data
 *
 * Description:
 *   Copies as many bytes as possible from ipcc channel to rxbuf.
 *
 * Input Parameters:
 *   ipcc - ipcc channel to copy data from
 *   rxbuf - circural buffer to copy data to
 *
 * Returned Value:
 *   Number of successfully buffered bytes.
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

#ifdef CONFIG_IPCC_BUFFERED
static ssize_t stm32wl5_ipcc_buffer_data(struct ipcc_lower_s *ipcc,
                                         struct circbuf_s *rxbuf)
{
  int ret;

  /* Disable RX interrupt since we will modify shared data */

  up_disable_irq(STM32WL5_IRQ_IPCC_C1_RX_IT);

  /* Copy data to buffer */

  ret = stm32wl5_ipcc_copy_to_buffer(ipcc->chan, rxbuf);

  /* Reenable interrupt */

  up_enable_irq(STM32WL5_IRQ_IPCC_C1_RX_IT);

  /* Return number of bytes that were successfully buffered */

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32wl5_ipcc_write_notify
 *
 * Description:
 *   This function is called when there is new data on circ buffer.
 *   All copy operations from buffer to ipcc memory is done in interrupt,
 *   so we simply unmask the interrupt. Interrupt will arrive once ipcc
 *   TX memory is free to be written.
 *
 * Input Parameters:
 *   ipcc - ipcc channel
 *
 * Returned Value:
 *   Always OK
 *
 ****************************************************************************/

#ifdef CONFIG_IPCC_BUFFERED
static ssize_t stm32wl5_ipcc_write_notify(struct ipcc_lower_s *ipcc)
{
  modifyreg32(STM32WL5_IPCC_C1MR, STM32WL5_IPCC_MR_CHNFM(ipcc->chan), 0);
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32wl5_ipcc_cleanup
 *
 * Description:
 *   Cleans up resources initialized by stm32wl5_ipcc_init(). This will
 *   free() ipcc pointer!
 *
 * Input Parameters:
 *   ipcc - ipcc channel to cleanup
 *
 * Returned Value:
 *   Always OK
 *
 ****************************************************************************/

static int stm32wl5_ipcc_cleanup(FAR struct ipcc_lower_s *ipcc)
{
  DEBUGASSERT(ipcc);
  DEBUGASSERT(ipcc->chan <= IPCC_NCHAN);

  /* Mask interrupts for given channel */

  modifyreg32(STM32WL5_IPCC_C1MR, 1, STM32WL5_IPCC_MR_CHNFM(ipcc->chan));
  modifyreg32(STM32WL5_IPCC_C1MR, 1, STM32WL5_IPCC_MR_CHNOM(ipcc->chan));

  /* Free allocated ipcc memory */

  kmm_free(ipcc);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wl5_ipcc_init
 *
 * Description:
 *   Function initializes runtime options for IPCC. This function is called
 *   by upper half of the IPCC driver from ipcc_register(). This function
 *   is called once for every registered channel - so it can be called
 *   multiple times with different input parameters during system bring up.
 *
 * Input Parameters:
 *   chan - channel to initialize
 *
 * Returned Value:
 *   Structure to link lower and upper halfs of the driver, or NULL on
 *   initialization failure.
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

struct ipcc_lower_s *stm32wl5_ipcc_init(int chan)
{
  int ret;
  static int ipcc_fti;
  struct ipcc_lower_s *ipcc;

  DEBUGASSERT(ipcc);
  DEBUGASSERT(chan <= IPCC_NCHAN);

  if ((ipcc = kmm_zalloc(sizeof(*ipcc))) == NULL)
    {
      return NULL;
    }

  /* Link internal stm32wl5 ipcc struct with character device driver
   * via ipcc_lower struct.
   */

  g_ipccpriv[chan].ipcc = ipcc;

  /* Give upper half driver pointers to mcu specific functions that
   * upper half needs to call to work properly.
   */

  ipcc->ops.read = stm32wl5_ipcc_read;
  ipcc->ops.write = stm32wl5_ipcc_write;
  ipcc->ops.cleanup = stm32wl5_ipcc_cleanup;
#ifdef CONFIG_IPCC_BUFFERED
  ipcc->ops.buffer_data = stm32wl5_ipcc_buffer_data;
  ipcc->ops.write_notify = stm32wl5_ipcc_write_notify;
#endif

  ipcc->chan = chan;

  /* Unmask channel interrupt */

  modifyreg32(STM32WL5_IPCC_C1MR, STM32WL5_IPCC_MR_CHNFM(chan), 0);
  modifyreg32(STM32WL5_IPCC_C1MR, STM32WL5_IPCC_MR_CHNOM(chan), 0);

  if (ipcc_fti)
    {
      return ipcc;
    }

  /* For the first time initialization we also need to attach rx/tx
   * interrupt functions
   */

  ret = irq_attach(STM32WL5_IRQ_IPCC_C1_RX_IT, stm32wl5_ipcc_rx_isr, NULL);
  if (ret)
    {
      kmm_free(ipcc);
      return NULL;
    }

  ret = irq_attach(STM32WL5_IRQ_IPCC_C1_TX_IT, stm32wl5_ipcc_tx_isr, NULL);
  if (ret)
    {
      kmm_free(ipcc);
      return NULL;
    }

  /* Enable interrupts when:
   * - we receive data from CPU2
   * - CPU2 has read message from us and TX memory is free to be used again
   */

  putreg32(STM32WL5_IPCC_CR_RXOIE | STM32WL5_IPCC_CR_TXFIE,
           STM32WL5_IPCC_C1CR);

  up_enable_irq(STM32WL5_IRQ_IPCC_C1_RX_IT);
  up_enable_irq(STM32WL5_IRQ_IPCC_C1_TX_IT);

  ipcc_fti = 1;

  return ipcc;
}
