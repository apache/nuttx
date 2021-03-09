/****************************************************************************
 * arch/arm/src/nrf52/nrf52_sdc.c
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
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/armv7-m/nvicpri.h>
#include <arch/nrf52/nrf52_irq.h>
#include <nuttx/wqueue.h>

#include "arm_internal.h"
#include "ram_vectors.h"
#include "arm_arch.h"

#include <mpsl.h>
#include <sdc.h>
#include <sdc_hci.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SDC_SLAVE_COUNT) && \
    CONFIG_SDC_SLAVE_COUNT > CONFIG_BLUETOOTH_MAX_CONN
#  error "Cannot support more BLE slave roles than connections"
#endif

#define SDC_MASTER_COUNT (CONFIG_BLUETOOTH_MAX_CONN - \
                          CONFIG_NRF52_SDC_SLAVE_COUNT)

/* Todo: check central/peripheral against master/slave count */

#define MASTER_MEM_SIZE  (SDC_MEM_PER_MASTER_LINK( \
                          SDC_DEFAULT_TX_PACKET_SIZE, \
                          SDC_DEFAULT_RX_PACKET_SIZE, \
                          SDC_DEFAULT_TX_PACKET_COUNT, \
                          SDC_DEFAULT_RX_PACKET_COUNT) \
                          + SDC_MEM_MASTER_LINKS_SHARED)

#define SLAVE_MEM_SIZE (SDC_MEM_PER_SLAVE_LINK( \
                        SDC_DEFAULT_TX_PACKET_SIZE, \
                        SDC_DEFAULT_RX_PACKET_SIZE, \
                        SDC_DEFAULT_TX_PACKET_COUNT, \
                        SDC_DEFAULT_RX_PACKET_COUNT) \
                        + SDC_MEM_SLAVE_LINKS_SHARED)

#define MEMPOOL_SIZE  ((CONFIG_NRF52_SDC_SLAVE_COUNT * SLAVE_MEM_SIZE) + \
                       (SDC_MASTER_COUNT * MASTER_MEM_SIZE))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_sdc_dev_s
{
  uint8_t mempool[MEMPOOL_SIZE];
  uint8_t msg_buffer[HCI_MSG_BUFFER_MAX_SIZE];

  sem_t exclsem;
  struct work_s work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mpsl_assert_handler(const char *const file, const uint32_t line);
static void sdc_fault_handler(const char *file, const uint32_t line);

static int bt_open(FAR const struct bt_driver_s *btdev);
static int bt_hci_send(FAR const struct bt_driver_s *btdev,
                       FAR struct bt_buf_s *buf);

static void on_hci(void);
static void on_hci_worker(void *arg);

static void low_prio_worker(void *arg);

static int swi_isr(int irq, FAR void *context, FAR void *arg);
static int power_clock_isr(int irq, FAR void *context, FAR void *arg);

static void rng_handler(void);
static void rtc0_handler(void);
static void timer0_handler(void);
static void radio_handler(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct bt_driver_s g_bt_driver =
{
  .head_reserve = 0,
  .open         = bt_open,
  .send         = bt_hci_send
};

static const mpsl_clock_lfclk_cfg_t g_clock_config =
{
  .source                   = MPSL_CLOCK_LF_SRC_XTAL,
  .rc_ctiv                  = 0,
  .rc_temp_ctiv             = 0,
  .accuracy_ppm             = CONFIG_NRF52_SDC_CLOCK_ACCURACY,
  .skip_wait_lfclk_started  = false
};

static struct nrf52_sdc_dev_s g_sdc_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_open
 ****************************************************************************/

static int bt_open(FAR const struct bt_driver_s *btdev)
{
  return 0;
}

/****************************************************************************
 * Name: bt_open
 ****************************************************************************/

static int bt_hci_send(FAR const struct bt_driver_s *btdev,
                       FAR struct bt_buf_s *buf)
{
  int ret = OK;

  /* Pass HCI CMD/DATA to SDC */

  if (buf->type == BT_CMD)
    {
      struct bt_hci_cmd_hdr_s *cmd = (struct bt_hci_cmd_hdr_s *)buf->data;

      wlinfo("passing CMD %d to softdevice\n", cmd->opcode);

      /* Ensure non-concurrent access to SDC operations */

      nxsem_wait_uninterruptible(&g_sdc_dev.exclsem);

      if (sdc_hci_cmd_put(buf->data) < 0)
        {
          wlerr("sdc_hci_cmd_put() failed\n");
          ret = -EIO;
        }

      nxsem_post(&g_sdc_dev.exclsem);

      work_queue(LPWORK, &g_sdc_dev.work, on_hci_worker, NULL, 0);
    }
  else if (buf->type == BT_ACL_OUT)
    {
      wlinfo("passing ACL to softdevice\n");

      /* Ensure non-concurrent access to SDC operations */

      nxsem_wait_uninterruptible(&g_sdc_dev.exclsem);

      if (sdc_hci_data_put(buf->data) < 0)
        {
          wlerr("sdc_hci_data_put() failed\n");
          ret = -EIO;
        }

      nxsem_post(&g_sdc_dev.exclsem);

      work_queue(LPWORK, &g_sdc_dev.work, on_hci_worker, NULL, 0);
    }

  if (ret < 0)
    {
      wlerr("bt_hci_send() failed: %d\n", ret);
      return ret;
    }
  else
    {
      return buf->len;
    }
}

/****************************************************************************
 * Name: sdc_fault_handler
 ****************************************************************************/

static void sdc_fault_handler(const char *file, const uint32_t line)
{
  _alert("SoftDevice Controller Fault\n");
  up_assert(file, line);
}

/****************************************************************************
 * Name: mpsl_assert_handler
 ****************************************************************************/

static void mpsl_assert_handler(const char *const file, const uint32_t line)
{
  _alert("MPSL assertion failed\n");
  up_assert(file, line);
}

/****************************************************************************
 * Name: low_prio_worker
 ****************************************************************************/

static void low_prio_worker(void *arg)
{
  /* Invoke MPSL low priority process handler. This will call on_hci()
   * internally when required.
   */

  nxsem_wait_uninterruptible(&g_sdc_dev.exclsem);
  mpsl_low_priority_process();
  nxsem_post(&g_sdc_dev.exclsem);
}

/****************************************************************************
 * Name: on_hci_worker
 ****************************************************************************/

static void on_hci_worker(void *arg)
{
  /* We use this worker to force a call to on_hci() right after sending
   * an HCI command as MPSL/SDC does not always signal the low priority
   * worker
   */

  nxsem_wait_uninterruptible(&g_sdc_dev.exclsem);
  on_hci();
  nxsem_post(&g_sdc_dev.exclsem);
}

/****************************************************************************
 * Name: on_hci
 ****************************************************************************/

static void on_hci(void)
{
  struct bt_buf_s *outbuf;
  size_t len;
  int ret;
  bool check_again = true;

  while (check_again)
    {
      check_again = false;

      /* Check for EVT by trying to get pending data into a generic
       * buffer and then create an actual bt_buf_s, depending on msg length
       */

      ret = sdc_hci_evt_get(g_sdc_dev.msg_buffer);

      if (ret == 0)
        {
          struct bt_hci_evt_hdr_s *hdr =
              (struct bt_hci_evt_hdr_s *)g_sdc_dev.msg_buffer;

          len = sizeof(*hdr) + hdr->len;

#ifdef CONFIG_DEBUG_WIRELESS_INFO
          if (hdr->evt == BT_HCI_EVT_CMD_COMPLETE)
            {
              struct hci_evt_cmd_complete_s *cmd_complete =
                  (struct hci_evt_cmd_complete_s *)
                      (g_sdc_dev.msg_buffer + sizeof(*hdr));
              uint8_t *status = (uint8_t *)cmd_complete + 1;

              wlinfo("received CMD_COMPLETE from softdevice "
                     "(opcode: %d, status: 0x%x)\n",
                     cmd_complete->opcode, *status);
            }
          else
            {
              wlinfo("received HCI EVT from softdevice "
                     "(evt: %d, len: %zu)\n", hdr->evt, len);
            }
#endif

          outbuf = bt_buf_alloc(BT_EVT, NULL, BLUETOOTH_H4_HDRLEN);
          bt_buf_extend(outbuf, len);

          memcpy(outbuf->data, g_sdc_dev.msg_buffer, len);

          bt_hci_receive(outbuf);

          check_again = true;
        }

      /* Same for ACL */

      ret = sdc_hci_data_get(g_sdc_dev.msg_buffer);

      if (ret == 0)
        {
          struct bt_hci_acl_hdr_s *hdr =
              (struct bt_hci_acl_hdr_s *)g_sdc_dev.msg_buffer;

          wlinfo("received HCI ACL from softdevice (handle: %d)\n",
                 hdr->handle);

          len = sizeof(*hdr) + hdr->len;

          outbuf = bt_buf_alloc(BT_ACL_IN, NULL, BLUETOOTH_H4_HDRLEN);
          bt_buf_extend(outbuf, len);

          memcpy(outbuf->data, g_sdc_dev.msg_buffer, len);

          bt_hci_receive(outbuf);

          check_again = true;
        }
    }
}

/****************************************************************************
 * Name: swi_isr
 ****************************************************************************/

static int swi_isr(int irq, FAR void *context, FAR void *arg)
{
  work_queue(LPWORK, &g_sdc_dev.work, low_prio_worker, NULL, 0);

  return 0;
}

/****************************************************************************
 * Name: rng_handler
 ****************************************************************************/

static void rng_handler(void)
{
  sdc_RNG_IRQHandler();
}

/****************************************************************************
 * Name: power_clock_isr
 ****************************************************************************/

static int power_clock_isr(int irq, FAR void *context, FAR void *arg)
{
  MPSL_IRQ_CLOCK_Handler();

  return 0;
}

/****************************************************************************
 * Name: rtc0_handler
 ****************************************************************************/

static void rtc0_handler(void)
{
  MPSL_IRQ_RTC0_Handler();
}

/****************************************************************************
 * Name: timer0_handler
 ****************************************************************************/

static void timer0_handler(void)
{
  MPSL_IRQ_TIMER0_Handler();
}

/****************************************************************************
 * Name: radio_handler
 ****************************************************************************/

static void radio_handler(void)
{
  MPSL_IRQ_RADIO_Handler();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nrf52_sdc_initialize(void)
{
  int ret;
  int32_t required_memory;
  sdc_cfg_t cfg;

  /* Initialize device data */

  memset(&g_sdc_dev, 0, sizeof(g_sdc_dev));
  nxsem_init(&g_sdc_dev.exclsem, 0, 1);

  /* Register interrupt handler for normal-priority events. SWI5 will be
   * used by MPSL to delegate low-priority work
   */

  irq_attach(NRF52_IRQ_SWI5_EGU5, swi_isr, NULL);
  irq_attach(NRF52_IRQ_POWER_CLOCK, power_clock_isr, NULL);

  up_enable_irq(NRF52_IRQ_SWI5_EGU5);
  up_enable_irq(NRF52_IRQ_POWER_CLOCK);

  up_prioritize_irq(NRF52_IRQ_SWI5_EGU5, NVIC_SYSH_PRIORITY_DEFAULT);
  up_prioritize_irq(NRF52_IRQ_POWER_CLOCK, NVIC_SYSH_PRIORITY_DEFAULT);

  /* Use zero-latency interrupt for RNG as we're expected to not add any
   * processing to the ISR
   */

  arm_ramvec_attach(NRF52_IRQ_RNG, rng_handler);
  up_prioritize_irq(NRF52_IRQ_RNG, NVIC_SYSH_MAXNORMAL_PRIORITY);
  up_enable_irq(NRF52_IRQ_RNG);

  /* Register high-priority interrupts for specific peripherals */

  arm_ramvec_attach(NRF52_IRQ_RTC0, rtc0_handler);
  arm_ramvec_attach(NRF52_IRQ_TIMER0, timer0_handler);
  arm_ramvec_attach(NRF52_IRQ_RADIO, radio_handler);

  up_prioritize_irq(NRF52_IRQ_RTC0, MPSL_HIGH_IRQ_PRIORITY);
  up_prioritize_irq(NRF52_IRQ_TIMER0, MPSL_HIGH_IRQ_PRIORITY);
  up_prioritize_irq(NRF52_IRQ_RADIO, MPSL_HIGH_IRQ_PRIORITY);

  up_enable_irq(NRF52_IRQ_RTC0);
  up_enable_irq(NRF52_IRQ_TIMER0);
  up_enable_irq(NRF52_IRQ_RADIO);

  /* TODO: how do WFI again after high priority interrupt wakes MCU up? */

  /* Initialize MPSL */

  ret = mpsl_init(&g_clock_config, NRF52_IRQ_SWI5_EGU5 - NRF52_IRQ_EXTINT,
                  &mpsl_assert_handler);

  if (ret < 0)
    {
      wlerr("mpsl init failed: %d\n", ret);
      return ret;
    }

  /* Initialize SDC */

  ret = sdc_init(&sdc_fault_handler);

  if (ret < 0)
    {
      wlerr("mpsl init failed: %d\n", ret);
      return ret;
    }

  /* Set some parameters */

  cfg.master_count.count = SDC_MASTER_COUNT;
  ret = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
                    SDC_CFG_TYPE_MASTER_COUNT, &cfg);

  if (ret < 0)
    {
      wlerr("Failed to set master role count: %d\n", ret);
      return ret;
    }

  cfg.slave_count.count = CONFIG_NRF52_SDC_SLAVE_COUNT;
  ret = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
                    SDC_CFG_TYPE_SLAVE_COUNT, &cfg);

  if (ret < 0)
    {
      wlerr("Failed to set slave role count: %d\n", ret);
      return ret;
    }

  cfg.buffer_cfg.rx_packet_size = SDC_DEFAULT_RX_PACKET_SIZE;
  cfg.buffer_cfg.tx_packet_size = SDC_DEFAULT_TX_PACKET_SIZE;
  cfg.buffer_cfg.rx_packet_count = SDC_DEFAULT_RX_PACKET_COUNT;
  cfg.buffer_cfg.tx_packet_count = SDC_DEFAULT_TX_PACKET_COUNT;

  required_memory =
      sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
                  SDC_CFG_TYPE_BUFFER_CFG, &cfg);

  if (required_memory < 0)
    {
      wlerr("Failed to set packet size/count: %ld\n", required_memory);
      return ret;
    }

  /* Verify we have enough memory for our configuration */

  ASSERT(required_memory <= sizeof(g_sdc_dev.mempool));

  /* Turn on specific features */

#ifdef CONFIG_NRF52_SDC_ADVERTISING
  ret = sdc_support_adv();

  if (ret < 0)
    {
      wlerr("Could not enable advertising feature: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_NRF52_SDC_SCANNING
  ret = sdc_support_scan();

  if (ret < 0)
    {
      wlerr("Could not enable scanning feature: %d\n", ret);
      return ret;
    }
#endif

#if SDC_MASTER_COUNT > 0
  ret = sdc_support_master();

  if (ret < 0)
    {
      wlerr("Could not enable master feature: %d\n", ret);
      return ret;
    }
#endif

#if CONFIG_NRF52_SDC_SLAVE_COUNT > 0
  ret = sdc_support_slave();

  if (ret < 0)
    {
      wlerr("Could not enable slave feature: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_NRF52_SDC_DLE
  ret = sdc_support_dle();

  if (ret < 0)
    {
      wlerr("Could not enable DLE feature: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_NRF52_SDC_LE_2M_PHY
  ret = sdc_support_le_2m_phy();

  if (ret < 0)
    {
      wlerr("Could not enable 2M PHY feature: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_NRF52_SDC_LE_CODED_PHY
  ret = sdc_support_le_coded_phy();

  if (ret < 0)
    {
      wlerr("Could not enable Coded PHY feature: %d\n", ret);
      return ret;
    }
#endif

  /* Finally enable SoftDevice Controller */

  ret = sdc_enable(on_hci, g_sdc_dev.mempool);

  if (ret < 0)
    {
      wlerr("SoftDevice controller enable failed: %d\n", ret);
      return ret;
    }

  /* Register network device */

  ret = bt_netdev_register(&g_bt_driver);

  return ret;
}
