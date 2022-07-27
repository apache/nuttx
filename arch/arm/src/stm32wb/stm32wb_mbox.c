/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_mbox.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include "arm_internal.h"
#include "stm32wb_ipcc.h"
#include "stm32wb_mbox.h"
#include "hardware/stm32wb_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mailbox shared buffer base address.  Normally it is located at
 * the beginning of SRAM2a.
 */

#define STM32WB_MBOX_SHARED_BASE      STM32WB_SRAM2A_BASE

/* Mailbox shared buffer fields */

#define stm32wb_mbox_shared \
  (*(struct stm32wb_mbox_shared_buffer_s *)STM32WB_MBOX_SHARED_BASE)

#define stm32wb_mbox_ref_table        (stm32wb_mbox_shared.ref_table)
#define stm32wb_mbox_di_table         (stm32wb_mbox_shared.dev_info_table)
#define stm32wb_mbox_sys_table        (stm32wb_mbox_shared.sys_table)
#define stm32wb_mbox_mm_table         (stm32wb_mbox_shared.mm_table)
#define stm32wb_mbox_ble_table        (stm32wb_mbox_shared.ble_table)

/* Mailbox buffer sizes */

#define STM32WB_MBOX_CS_BUF_SIZE      16
#define STM32WB_MBOX_CMDPKT_BUF_SIZE  268
#define STM32WB_MBOX_ACLPKT_BUF_SIZE  264

#define STM32WB_MBOX_RX_BUF_SIZE \
  (CONFIG_STM32WB_MBOX_RX_EVT_QUEUE_LEN * STM32WB_MBOX_CMDPKT_BUF_SIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Mailbox shared buffer structures */

begin_packed_struct struct stm32wb_mbox_safe_boot_info_table_s
{
  uint32_t  version;
} end_packed_struct;

begin_packed_struct struct stm32wb_mbox_fus_info_table_s
{
  uint32_t  version;
  uint32_t  memory_size;
  uint32_t  fus_info;
} end_packed_struct;

begin_packed_struct struct stm32wb_mbox_wireless_fw_info_table_s
{
  uint32_t  version;
  uint32_t  memory_size;
  uint32_t  info_stack;
  uint32_t  reserved;
} end_packed_struct;

begin_packed_struct struct stm32wb_mbox_device_info_table_s
{
  struct stm32wb_mbox_safe_boot_info_table_s    safe_boot_info_table;
  struct stm32wb_mbox_fus_info_table_s          fus_info_table;
  struct stm32wb_mbox_wireless_fw_info_table_s  wireless_fw_info_table;
} end_packed_struct;

begin_packed_struct struct stm32wb_mbox_ble_table_s
{
  void      *cmd_buffer;
  void      *cs_buffer;
  void      *evt_queue;
  void      *acl_buffer;
} end_packed_struct;

begin_packed_struct struct stm32wb_mbox_sys_table_s
{
  void      *cmd_buffer;
  void      *evt_queue;
} end_packed_struct;

begin_packed_struct struct stm32wb_mbox_mem_manager_table_s
{
  void      *ble_spare_buffer;
  void      *sys_spare_buffer;
  void      *evtpool_buffer;
  uint32_t  evtpool_size;
  void      *evtfree_buffer;
  void      *traces_evtpool_buffer;
  uint32_t  traces_evtpool_size;
} end_packed_struct;

begin_packed_struct struct stm32wb_mbox_ref_table_s
{
  struct stm32wb_mbox_device_info_table_s   *dev_info_table;
  struct stm32wb_mbox_ble_table_s           *ble_table;
  void                                      *thread_table;
  struct stm32wb_mbox_sys_table_s           *sys_table;
  struct stm32wb_mbox_mem_manager_table_s   *mm_table;
  void                                      *traces_table;
  void                                      *mac_802_15_4_table;
  void                                      *zigbee_table;
  void                                      *ble_lld_test_table;
  void                                      *ble_lld_table;
} end_packed_struct;

/* Mailbox shared buffer memory layout structure */

struct stm32wb_mbox_shared_buffer_s
{
  aligned_data(4) struct stm32wb_mbox_ref_table_s         ref_table;

  aligned_data(4) struct stm32wb_mbox_device_info_table_s dev_info_table;
  aligned_data(4) struct stm32wb_mbox_ble_table_s         ble_table;
  aligned_data(4) struct stm32wb_mbox_sys_table_s         sys_table;
  aligned_data(4) struct stm32wb_mbox_mem_manager_table_s mm_table;

  aligned_data(4) stm32wb_mbox_list_t  evtfree_buffer;
#ifdef CONFIG_STM32WB_BLE
  aligned_data(4) stm32wb_mbox_list_t  ble_evt_queue;
#endif
  aligned_data(4) stm32wb_mbox_list_t  sys_evt_queue;

#ifdef CONFIG_STM32WB_BLE
  aligned_data(4) uint8_t ble_cs_buffer[STM32WB_MBOX_CS_BUF_SIZE];
#endif
  aligned_data(4) uint8_t evtpool_buffer[STM32WB_MBOX_RX_BUF_SIZE];
  aligned_data(4) uint8_t sys_cmd_buffer[STM32WB_MBOX_CMDPKT_BUF_SIZE];
  aligned_data(4) uint8_t sys_spare_buffer[STM32WB_MBOX_CMDPKT_BUF_SIZE];
#ifdef CONFIG_STM32WB_BLE
  aligned_data(4) uint8_t ble_spare_buffer[STM32WB_MBOX_CMDPKT_BUF_SIZE];
  aligned_data(4) uint8_t ble_cmd_buffer[STM32WB_MBOX_CMDPKT_BUF_SIZE];
  aligned_data(4) uint8_t ble_acl_buffer[STM32WB_MBOX_ACLPKT_BUF_SIZE];
#endif
};

/* Mailbox channel data type */

struct stm32wb_mbox_channel_s
{
  uint8_t                     ch_num;
  stm32wb_mbox_list_t         msg_buf_queue;
  struct stm32wb_mbox_cmd_s   *msg_buf;
  bool                        ack_ready;
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static void stm32wb_ipcc_rxoisr(int irq, uint32_t *regs, void *arg);
static void stm32wb_ipcc_txfisr(int irq, uint32_t *regs, void *arg);

static void stm32wb_mbox_rxworker(void *arg);
static void stm32wb_mbox_txworker(void *arg);

static void stm32wb_mbox_eventfree(stm32wb_mbox_list_t *evt);
static void stm32wb_mbox_acksyscmd(void);

static int stm32wb_mbox_txdata(struct stm32wb_mbox_channel_s *chan,
                               uint8_t type, void *data, size_t len);
static bool stm32wb_mbox_txnext(struct stm32wb_mbox_channel_s *chan);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct work_s g_rx_evt_work;
static struct work_s g_tx_cmd_work;

static stm32wb_mbox_list_t g_rx_evt_queue;
static stm32wb_mbox_list_t g_tx_evtfree_queue;
static uint8_t g_free_buffers[CONFIG_STM32WB_MBOX_TX_CMD_QUEUE_LEN]
                             [STM32WB_MBOX_CMDPKT_BUF_SIZE];
static stm32wb_mbox_list_t g_free_buffers_pool;

static struct stm32wb_mbox_channel_s g_syscmd_channel;
#ifdef CONFIG_STM32WB_BLE
static struct stm32wb_mbox_channel_s g_blecmd_channel;
static struct stm32wb_mbox_channel_s g_bleacl_channel;
#endif

static stm32wb_mbox_evt_handler_t receive_evt_handler;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_ipcc_rxoisr
 *
 * Description:
 *   RX channel occupied interrupt handler (communication data posted
 *   by sending processor).
 *
 ****************************************************************************/

static void stm32wb_ipcc_rxoisr(int irq, uint32_t *regs, void *arg)
{
  uint32_t clrmask = 0;

  /* Pull events from system channel into processing queue */

  if (stm32wb_ipcc_rxactive(STM32WB_MBOX_SYSEVT_CHANNEL))
    {
      stm32wb_mbox_list_moveall(&stm32wb_mbox_shared.sys_evt_queue,
                                &g_rx_evt_queue);

      clrmask |= IPCC_C1SCR_CLR_BIT(STM32WB_MBOX_SYSEVT_CHANNEL);
    }

#ifdef CONFIG_STM32WB_BLE

  /* Pull events from BLE channel into processing queue */

  if (stm32wb_ipcc_rxactive(STM32WB_MBOX_BLEEVT_CHANNEL))
    {
      stm32wb_mbox_list_moveall(&stm32wb_mbox_shared.ble_evt_queue,
                                &g_rx_evt_queue);

      clrmask |= IPCC_C1SCR_CLR_BIT(STM32WB_MBOX_BLEEVT_CHANNEL);
    }
#endif

  /* Process collected events in the work queue */

  if (work_available(&g_rx_evt_work))
    {
      work_queue(HPWORK, &g_rx_evt_work, stm32wb_mbox_rxworker, NULL, 0);
    }

  /* Clear active statuses */

  putreg32(clrmask, STM32WB_IPCC_C1SCR);
}

/****************************************************************************
 * Name: stm32wb_ipcc_txfisr
 *
 * Description:
 *   TX channel free interrupt handler (communication data retrieved
 *   by receiving processor).
 *
 ****************************************************************************/

static void stm32wb_ipcc_txfisr(int irq, uint32_t *regs, void *arg)
{
  uint32_t c1mr = getreg32(STM32WB_IPCC_C1MR);
  uint32_t txfsrc;

  /* TXF interrupt can be triggered by not masked channels and active status
   * of the source channel is cleared by CPU2. So we ignore masked and active
   * channels and rise other C1MR bits to highlight needed channels.
   */

  txfsrc = ~(c1mr | (getreg32(STM32WB_IPCC_C1TOC2SR) << IPCC_C1MR_FM_SHIFT))
           & IPCC_C1MR_FM_MASK;

  /* Check if the release channel triggered the interrupt */

  if (txfsrc & IPCC_C1MR_FM_BIT(STM32WB_MBOX_EVT_RELEASE_CHANNEL))
    {
      /* Move all released events (if any) into transmission mailbox */

      if (!stm32wb_mbox_list_is_empty(&g_tx_evtfree_queue))
        {
          stm32wb_mbox_list_moveall(&g_tx_evtfree_queue,
                                    &stm32wb_mbox_shared.evtfree_buffer);

          /* Start release channel transmission */

          stm32wb_ipcc_settxactive(STM32WB_MBOX_EVT_RELEASE_CHANNEL);
        }
    }

  /* Check other channels, except the release channel */

  if (txfsrc & ~IPCC_C1MR_FM_BIT(STM32WB_MBOX_EVT_RELEASE_CHANNEL))
    {
      /* Check if the system channel triggered the interrupt */

      if (txfsrc & IPCC_C1MR_FM_BIT(STM32WB_MBOX_SYSCMD_CHANNEL))
        {
          /* System channel works in 'half-duplex' mode and acks
           * immediately on each command before TXF, so it needs
           * to process ack response before sending next command.
           */

          g_syscmd_channel.ack_ready = true;
        }

      /* Continue command processing in a work queue */

      if (work_available(&g_tx_cmd_work))
        {
          work_queue(HPWORK, &g_tx_cmd_work, stm32wb_mbox_txworker, NULL, 0);
        }
    }

  /* Mask triggered channels */

  putreg32(c1mr | txfsrc, STM32WB_IPCC_C1MR);
}

/****************************************************************************
 * Name: stm32wb_mbox_txworker
 ****************************************************************************/

static void stm32wb_mbox_txworker(void *arg)
{
  bool handled;

  /* Process all queued commands if a dedicated channel is free */

  do
    {
      handled = false;

      if (!stm32wb_ipcc_txactive(STM32WB_MBOX_SYSCMD_CHANNEL))
        {
          /* Process ack response before send new command */

          if (g_syscmd_channel.ack_ready)
            {
              stm32wb_mbox_acksyscmd();
              g_syscmd_channel.ack_ready = false;
            }

          handled = stm32wb_mbox_txnext(&g_syscmd_channel);
        }

#ifdef CONFIG_STM32WB_BLE
      if (!stm32wb_ipcc_txactive(STM32WB_MBOX_BLECMD_CHANNEL))
        {
          handled |= stm32wb_mbox_txnext(&g_blecmd_channel);
        }

      if (!stm32wb_ipcc_txactive(STM32WB_MBOX_BLEACL_CHANNEL))
        {
          handled |= stm32wb_mbox_txnext(&g_bleacl_channel);
        }
#endif
    }
  while (handled);
}

/****************************************************************************
 * Name: stm32wb_mbox_rxworker
 ****************************************************************************/

static void stm32wb_mbox_rxworker(void *arg)
{
  stm32wb_mbox_list_t *evt;
  irqstate_t flags;

  while (1)
    {
      flags = enter_critical_section();

      /* Pull an event from the queue */

      evt = stm32wb_mbox_list_remove_head(&g_rx_evt_queue);

      leave_critical_section(flags);

      if (evt == NULL)
        {
          break;
        }

      /* Pass event to a callback function without a list header */

      receive_evt_handler((struct stm32wb_mbox_evt_s *)(evt + 1));

      /* Free completed event.  Released event needs to return to CPU2
       * via release channel.
       */

      stm32wb_mbox_eventfree((stm32wb_mbox_list_t *)evt);
    }
}

/****************************************************************************
 * Name: stm32wb_mbox_txdata
 *
 * Description:
 *   Send data over specified mailbox channel if possible.  If the
 *   channel is busy then add a prepared packet into awaiting queue.
 *
 ****************************************************************************/

static int stm32wb_mbox_txdata(struct stm32wb_mbox_channel_s *chan,
                               uint8_t type, void *data, size_t len)
{
  irqstate_t flags;
  struct stm32wb_mbox_cmd_s *pkt_buf;

  flags = enter_critical_section();

  /* To start transmission the channel needs to be free, there should be
   * none of other waiting commands and none of unprocessed ack responses.
   */

  if (stm32wb_mbox_list_is_empty(&chan->msg_buf_queue) &&
      !stm32wb_ipcc_txactive(chan->ch_num) && !chan->ack_ready)
    {
      /* Channel is ready, copy command into transmission buffer */

      pkt_buf = chan->msg_buf;
    }
  else
    {
      /* Otherwise get temp buffer for command */

      pkt_buf = (struct stm32wb_mbox_cmd_s *)
                stm32wb_mbox_list_remove_head(&g_free_buffers_pool);
    }

  leave_critical_section(flags);

  if (pkt_buf == NULL)
    {
      return -ENOBUFS;
    }

  pkt_buf->type = type;
  memcpy(&pkt_buf->cmd_hdr, data, len);

  if (pkt_buf == chan->msg_buf)
    {
      /* Command is ready in mailbox buffer, start transmission now */

      stm32wb_ipcc_settxactive(chan->ch_num);

      if (!stm32wb_mbox_list_is_empty(&chan->msg_buf_queue) ||
          chan->ch_num == STM32WB_MBOX_SYSCMD_CHANNEL)
        {
          /* There are more commands awaiting, so unmask interrupt to get
           * notified when channel gets ready to process a next one.
           * And the system channel needs to check ack on completion.
           */

          stm32wb_ipcc_unmasktxf(chan->ch_num);
        }
    }
  else
    {
      /* Command is in temp buffer, push it into queue */

      flags = enter_critical_section();
      stm32wb_mbox_list_add_tail(&chan->msg_buf_queue, &pkt_buf->list_hdr);
      leave_critical_section(flags);

      /* Unmask interrupt to get notified when channel gets free */

      stm32wb_ipcc_unmasktxf(chan->ch_num);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32wb_mbox_txnext
 *
 * Description:
 *   Send next command from the queue.
 *
 ****************************************************************************/

static bool stm32wb_mbox_txnext(struct stm32wb_mbox_channel_s *chan)
{
  struct stm32wb_mbox_cmd_s *pkt_buf;

  pkt_buf = (struct stm32wb_mbox_cmd_s *)
            stm32wb_mbox_list_remove_head(&chan->msg_buf_queue);

  if (pkt_buf != NULL)
    {
      chan->msg_buf->type = pkt_buf->type;

      if (chan->ch_num == STM32WB_MBOX_BLEACL_CHANNEL)
        {
          memcpy(chan->msg_buf, &pkt_buf->acl_hdr,
                 pkt_buf->acl_hdr.len);
        }
      else
        {
          memcpy(chan->msg_buf, &pkt_buf->cmd_hdr,
                 pkt_buf->cmd_hdr.param_len);
        }

      /* Start transmission */

      stm32wb_ipcc_settxactive(chan->ch_num);

      if (!stm32wb_mbox_list_is_empty(&chan->msg_buf_queue))
        {
          /* Unmask TXF interrupt to get notified when completed */

          stm32wb_ipcc_unmasktxf(chan->ch_num);
        }

      /* Put back to pool the freed command buffer */

      stm32wb_mbox_list_add_tail(&g_free_buffers_pool, &pkt_buf->list_hdr);
    }

  return pkt_buf != NULL;
}

/****************************************************************************
 * Name: stm32wb_mbox_eventfree
 *
 * Description:
 *   Free handled mailbox event.
 *
 ****************************************************************************/

static void stm32wb_mbox_eventfree(stm32wb_mbox_list_t *evt)
{
  irqstate_t flags;

  flags = enter_critical_section();

  /* Collect releasing events in the global list */

  stm32wb_mbox_list_add_tail(&g_tx_evtfree_queue, evt);

  /* Check if release channel is ready to process now */

  if (!stm32wb_ipcc_txactive(STM32WB_MBOX_EVT_RELEASE_CHANNEL))
    {
      /* Move all collected events into transmission queue */

      stm32wb_mbox_list_moveall(&g_tx_evtfree_queue,
                                &stm32wb_mbox_shared.evtfree_buffer);

      /* Start transmission */

      stm32wb_ipcc_settxactive(STM32WB_MBOX_EVT_RELEASE_CHANNEL);
    }
  else
    {
      /* Unmask interrupt to get notified when channel gets free */

      stm32wb_ipcc_unmasktxf(STM32WB_MBOX_EVT_RELEASE_CHANNEL);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32wb_mbox_acksyscmd
 *
 * Description:
 *   Send ACK response event for completed system command.
 *
 ****************************************************************************/

static void stm32wb_mbox_acksyscmd(void)
{
  struct stm32wb_mbox_evt_s *evt;

  /* System command ACK response is placed at the same address as the
   * processed command but without a list header.
   */

  evt = (struct stm32wb_mbox_evt_s *)(&g_syscmd_channel.msg_buf);
  evt->type = STM32WB_MBOX_SYSACK;

  receive_evt_handler(evt);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_mboxinitialize
 *
 * Description:
 *   Initialize mailbox driver memory.
 *
 * Input Parameters:
 *   evt_handler - the function to call on event receive
 *
 ****************************************************************************/

void stm32wb_mboxinitialize(stm32wb_mbox_evt_handler_t evt_handler)
{
  int i;

  /* Init mailbox shared data */

  stm32wb_mbox_list_initialize(&stm32wb_mbox_shared.sys_evt_queue);
  stm32wb_mbox_list_initialize(&stm32wb_mbox_shared.evtfree_buffer);
#ifdef CONFIG_STM32WB_BLE
  stm32wb_mbox_list_initialize(&stm32wb_mbox_shared.ble_evt_queue);
#endif

  stm32wb_mbox_ref_table.dev_info_table = &stm32wb_mbox_di_table;
  stm32wb_mbox_ref_table.ble_table      = &stm32wb_mbox_ble_table;
  stm32wb_mbox_ref_table.sys_table      = &stm32wb_mbox_sys_table;
  stm32wb_mbox_ref_table.mm_table       = &stm32wb_mbox_mm_table;

  stm32wb_mbox_sys_table.cmd_buffer = &stm32wb_mbox_shared.sys_cmd_buffer;
  stm32wb_mbox_sys_table.evt_queue  = &stm32wb_mbox_shared.sys_evt_queue;

  stm32wb_mbox_mm_table.evtpool_buffer   = &stm32wb_mbox_shared
                                            .evtpool_buffer;
  stm32wb_mbox_mm_table.evtpool_size     = sizeof(stm32wb_mbox_shared
                                                  .evtpool_buffer);
  stm32wb_mbox_mm_table.evtfree_buffer   = &stm32wb_mbox_shared
                                            .evtfree_buffer;
  stm32wb_mbox_mm_table.sys_spare_buffer = &stm32wb_mbox_shared
                                            .sys_spare_buffer;
#ifdef CONFIG_STM32WB_BLE
  stm32wb_mbox_mm_table.ble_spare_buffer = &stm32wb_mbox_shared
                                            .ble_spare_buffer;
#endif

#ifdef CONFIG_STM32WB_BLE
  stm32wb_mbox_ble_table.cmd_buffer = &stm32wb_mbox_shared.ble_cmd_buffer;
  stm32wb_mbox_ble_table.acl_buffer = &stm32wb_mbox_shared.ble_acl_buffer;
  stm32wb_mbox_ble_table.cs_buffer  = &stm32wb_mbox_shared.ble_cs_buffer;
  stm32wb_mbox_ble_table.evt_queue  = &stm32wb_mbox_shared.ble_evt_queue;
#endif

  /* Init system channel data */

  g_syscmd_channel.ch_num =  STM32WB_MBOX_SYSCMD_CHANNEL;
  g_syscmd_channel.msg_buf = (struct stm32wb_mbox_cmd_s *)
                              stm32wb_mbox_shared.sys_cmd_buffer;
  stm32wb_mbox_list_initialize(&g_syscmd_channel.msg_buf_queue);

#ifdef CONFIG_STM32WB_BLE
  /* Init BLE command channel data */

  g_blecmd_channel.ch_num =  STM32WB_MBOX_BLECMD_CHANNEL;
  g_blecmd_channel.msg_buf = (struct stm32wb_mbox_cmd_s *)
                             stm32wb_mbox_shared.ble_cmd_buffer;
  stm32wb_mbox_list_initialize(&g_blecmd_channel.msg_buf_queue);

  /* Init BLE ACL channel data */

  g_bleacl_channel.ch_num =  STM32WB_MBOX_BLEACL_CHANNEL;
  g_bleacl_channel.msg_buf = (struct stm32wb_mbox_cmd_s *)
                             stm32wb_mbox_shared.ble_cmd_buffer;
  stm32wb_mbox_list_initialize(&g_bleacl_channel.msg_buf_queue);
#endif

  /* Init local (not shared) queues */

  stm32wb_mbox_list_initialize(&g_rx_evt_queue);
  stm32wb_mbox_list_initialize(&g_tx_evtfree_queue);

  stm32wb_mbox_list_initialize(&g_free_buffers_pool);
  for (i = 0; i < CONFIG_STM32WB_MBOX_TX_CMD_QUEUE_LEN; i++)
    {
      stm32wb_mbox_list_add_tail(&g_free_buffers_pool,
                                 (stm32wb_mbox_list_t *)g_free_buffers[i]);
    }

  /* Set event receive function */

  receive_evt_handler = evt_handler;
}

/****************************************************************************
 * Name: stm32wb_mboxenable
 *
 * Description:
 *   Enable mailbox hardware and start communication.  The CPU2 responses
 *   with C2Ready event on success.
 *
 ****************************************************************************/

void stm32wb_mboxenable(void)
{
  uint32_t regval;

  /* Setup RXO and TXF interrupts */

  irq_attach(STM32WB_IRQ_IPCCRX, (xcpt_t)stm32wb_ipcc_rxoisr, NULL);
  up_enable_irq(STM32WB_IRQ_IPCCRX);

  irq_attach(STM32WB_IRQ_IPCCTX, (xcpt_t)stm32wb_ipcc_txfisr, NULL);
  up_enable_irq(STM32WB_IRQ_IPCCTX);

  regval = getreg32(STM32WB_IPCC_C1CR);
  regval |= IPCC_C1CR_RXOIE | IPCC_C1CR_TXFIE;
  putreg32(regval, STM32WB_IPCC_C1CR);

  /* Unmask system channel RXO interrupt.  Once CPU2 started we expect
   * to receive C2READY event via system channel.
   */

  stm32wb_ipcc_unmaskrxo(STM32WB_MBOX_SYSEVT_CHANNEL);

  /* Enable IPCC hardware and bootup CPU2 */

  stm32wb_ipccenable();
}

/****************************************************************************
 * Name: stm32wb_mbox_syscmd
 *
 * Description:
 *   Send command over mailbox system channel.  Command data must be
 *   prepended with HCI header.
 *
 ****************************************************************************/

int stm32wb_mbox_syscmd(void *data, size_t len)
{
  return stm32wb_mbox_txdata(&g_syscmd_channel, STM32WB_MBOX_SYSCMD,
                             data, len);
}

#ifdef CONFIG_STM32WB_BLE
/****************************************************************************
 * Name: stm32wb_mbox_blecmd
 *
 * Description:
 *   Send command over mailbox BLE channel.  Command data must be
 *   prepended with HCI header.
 *
 ****************************************************************************/

int stm32wb_mbox_blecmd(void *data, size_t len)
{
  return stm32wb_mbox_txdata(&g_blecmd_channel, STM32WB_MBOX_HCICMD,
                             data, len);
}

/****************************************************************************
 * Name: stm32wb_mbox_bleacl
 *
 * Description:
 *   Send BLE ACL data over mailbox BLE ACL channel.  Data must be
 *   prepended with HCI ACL header.
 *
 ****************************************************************************/

int stm32wb_mbox_bleacl(void *data, size_t len)
{
  return stm32wb_mbox_txdata(&g_bleacl_channel, STM32WB_MBOX_HCIACL,
                             data, len);
}

/****************************************************************************
 * Name: stm32wb_mbox_bleinit
 *
 * Description:
 *   Initialize and start BLE subsystem with provided configuration params.
 *
 ****************************************************************************/

void stm32wb_mbox_bleinit(struct stm32wb_shci_ble_init_cfg_s *params)
{
  struct bt_hci_cmd_hdr_s *cmd;

  /* Just borrow a temporary free buffer for command data */

  cmd = (struct bt_hci_cmd_hdr_s *)stm32wb_mbox_shared.sys_spare_buffer;

  /* Prepare command data */

  cmd->opcode = STM32WB_SHCI_BLE_INIT;
  cmd->param_len = sizeof(*cmd);
  memcpy(cmd + 1, params, sizeof(*params));

  /* Send BLE init command to CPU2 */

  stm32wb_mbox_syscmd(cmd, sizeof(*cmd) + sizeof(*params));

  /* Unmask BLE event channel RXO interrupt */

  stm32wb_ipcc_unmaskrxo(STM32WB_MBOX_BLEEVT_CHANNEL);
}
#endif /* CONFIG_STM32WB_BLE */
