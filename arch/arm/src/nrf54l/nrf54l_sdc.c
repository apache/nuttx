/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_sdc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/debug.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <arch/armv8-m/nvicpri.h>
#include <nuttx/wqueue.h>

#include "arm_internal.h"
#include "ram_vectors.h"

#include "hardware/nrf54l_ficr.h"
#include "hardware/nrf54l_rramc.h"
#include "hardware/nrf54l_power.h"
#include "hardware/nrf54l_clock.h"
#include "nrf54l_grtc.h"
#include "nrf54l_sdc.h"

#include <mpsl.h>
#include <sdc.h>
#include <sdc_hci.h>
#include <sdc_soc.h>
#include <sdc_hci_cmd_le.h>
#include <sdc_hci_cmd_controller_baseband.h>
#include <sdc_hci_vs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Connections configuration ************************************************/

#if NRF54L_SDC_CENTRAL_COUNT < 0
#  error "Cannot support more BLE peripheral roles than connections"
#endif

/* Memory configuration *****************************************************/

/* Observer configuration */

#ifdef CONFIG_NRF54L_SDC_SCANNING
#  if CONFIG_NRF54L_SDC_SCAN_BUFFER_COUNT < 2
#    error The minimum allowed number of scan buffers is 2.
#  endif
#  define SCAN_MEM_COUNT (CONFIG_NRF54L_SDC_SCAN_BUFFER_COUNT)
#elif NRF54L_SDC_CENTRAL_COUNT > 0
#  define SCAN_MEM_COUNT (3)
#else
#  define SCAN_MEM_COUNT (0)
#endif

/* Broadcaster configuration */

#ifdef CONFIG_NRF54L_SDC_ADVERTISING
/* Advertising extensions not supported for now */

#  define ADV_SET_COUNT (1)
#  define ADV_BUF_SIZE  (SDC_DEFAULT_ADV_BUF_SIZE)
#else
#  define ADV_SET_COUNT (0)
#  define ADV_BUF_SIZE  (0)
#endif

/* BT address configuration *************************************************/

#if (CONFIG_NRF54L_SDC_PUB_ADDR > 0) ||          \
  defined(CONFIG_NRF54L_SDC_FICR_STATIC_ADDR)
#  define HAVE_BTADDR_CONFIGURE
#endif

/* Calls to MPSL ************************************************************/

#define MPSL_IRQ_CLOCK_HANDLER  MPSL_IRQ_CLOCK_Handler
#define MPSL_IRQ_RTC0_HANDLER   MPSL_IRQ_RTC0_Handler
#define MPSL_IRQ_TIMER0_HANDLER MPSL_IRQ_TIMER0_Handler
#define MPSL_IRQ_RADIO_HANDLER  MPSL_IRQ_RADIO_Handler

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf54l_sdc_dev_s
{
  uint8_t *mempool;  /* Must be 8 bytes aligned */

  mutex_t lock;
  struct work_s work;
  struct work_s hci_work;
  bool opened;
  bool initialized;
  bool initializing;
  bool mpsl_initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mpsl_assert_handler(const char *const file, const uint32_t line);
static void sdc_fault_handler(const char *file, const uint32_t line);

static int bt_open(struct bt_driver_s *btdev);
static int bt_hci_send(struct bt_driver_s *btdev,
                       enum bt_buf_type_e type,
                       void *data, size_t len);

static void on_hci(void);
static void on_hci_worker(void *arg);

static void low_prio_worker(void *arg);

static int swi_isr(int irq, void *context, void *arg);
static int power_clock_isr(int irq, void *context, void *arg);

static void rtc0_handler(void);
static void timer0_handler(void);
static void radio_handler(void);
static int32_t nrf54l_sdc_lfclk_request(void);
static int32_t nrf54l_sdc_lfclk_release(void);
static void nrf54l_sdc_hfclk_request(void);
static void nrf54l_sdc_hfclk_release(void);
static bool nrf54l_sdc_hfclk_running(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_driver_s g_bt_driver =
{
  .head_reserve = 0,
  .open         = bt_open,
  .send         = bt_hci_send
};

static const mpsl_clock_lfclk_ctrl_source_t g_lfclk_source =
{
  .lfclk_wait               = nrf54l_sdc_lfclk_request,
  .lfclk_request            = nrf54l_sdc_lfclk_request,
  .lfclk_release            = nrf54l_sdc_lfclk_release,
  .accuracy_ppm             = CONFIG_NRF54L_SDC_CLOCK_ACCURACY,
  .skip_wait_lfclk_started  = false
};

static const mpsl_clock_hfclk_ctrl_source_t g_hfclk_source =
{
  .hfclk_request    = nrf54l_sdc_hfclk_request,
  .hfclk_release    = nrf54l_sdc_hfclk_release,
  .hfclk_is_running = nrf54l_sdc_hfclk_running,
  .startup_time_us  = 1
};

static uint32_t g_rram_lowpower;
static bool g_constlat;
static bool g_low_latency;

static struct nrf54l_sdc_dev_s g_sdc_dev =
{
  .lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_sdc_lfclk_request
 ****************************************************************************/

static int32_t nrf54l_sdc_lfclk_request(void)
{
  /* Startup keeps synthesized LFCLK running for both GRTC and MPSL. */

  return (getreg32(NRF54L_CLOCK_LFCLK_STAT) & CLOCK_LFCLK_STAT_STATE) != 0 ?
         OK : -EIO;
}

/****************************************************************************
 * Name: nrf54l_sdc_lfclk_release
 ****************************************************************************/

static int32_t nrf54l_sdc_lfclk_release(void)
{
  /* GRTC still uses this clock when the controller releases its request. */

  return OK;
}

/****************************************************************************
 * Name: nrf54l_sdc_hfclk_request
 ****************************************************************************/

static void nrf54l_sdc_hfclk_request(void)
{
  DEBUGASSERT(nrf54l_sdc_hfclk_running());
}

/****************************************************************************
 * Name: nrf54l_sdc_hfclk_release
 ****************************************************************************/

static void nrf54l_sdc_hfclk_release(void)
{
  /* The CPU and synthesized LFCLK retain the startup HFXO request. */
}

/****************************************************************************
 * Name: nrf54l_sdc_hfclk_running
 ****************************************************************************/

static bool nrf54l_sdc_hfclk_running(void)
{
  return (getreg32(NRF54L_CLOCK_XO_STAT) & CLOCK_XO_STAT_STATE) != 0;
}

/****************************************************************************
 * Name: bt_open
 ****************************************************************************/

static int bt_open(struct bt_driver_s *btdev)
{
  g_sdc_dev.opened = true;
  on_hci();
  return 0;
}

/****************************************************************************
 * Name: bt_hci_send
 ****************************************************************************/

static int bt_hci_send(struct bt_driver_s *btdev,
                       enum bt_buf_type_e type,
                       void *data, size_t len)
{
  const uint8_t *packet = data;
  uint8_t event[HCI_EVENT_PACKET_MAX_SIZE];
  size_t eventlen = 0;
  int ret = -EIO;

  if (data == NULL ||
      (type == BT_CMD && (len < 3 || len != 3 + packet[2] ||
                         len > HCI_CMD_PACKET_MAX_SIZE)) ||
      (type == BT_ACL_OUT &&
       (len < 4 || len > HCI_DATA_PACKET_MAX_SIZE ||
        len != 4 + (packet[2] | (packet[3] << 8)))))
    {
      return -EINVAL;
    }

  /* Pass HCI CMD/DATA to SDC */

  if (type == BT_CMD || type == BT_ACL_OUT)
    {
      wlinfo("passing type %s to softdevice\n",
             (type == BT_CMD) ? "CMD" : "ACL");

      /* Ensure non-concurrent access to SDC operations */

      ret = nxmutex_lock(&g_sdc_dev.lock);
      if (ret < 0)
        {
          return ret;
        }

      if (type == BT_CMD)
        {
          eventlen = nrf54l_sdc_command(data, event);
          ret = OK;
        }
      else
        {
          ret = sdc_hci_data_put(data);
        }

      nxmutex_unlock(&g_sdc_dev.lock);
      if (eventlen > 0)
        {
          bt_netdev_receive(&g_bt_driver, BT_EVT, event, eventlen);
        }

      if (ret >= 0)
        {
          ret = len;

          work_queue(LPWORK, &g_sdc_dev.hci_work, on_hci_worker, NULL, 0);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sdc_fault_handler
 ****************************************************************************/

static void sdc_fault_handler(const char *file, const uint32_t line)
{
  __assert(file, line, "SoftDevice Controller Fault");
}

/****************************************************************************
 * Name: mpsl_assert_handler
 ****************************************************************************/

static void mpsl_assert_handler(const char *const file, const uint32_t line)
{
  __assert(file, line, "MPSL assertion failed");
}

/****************************************************************************
 * Name: low_prio_worker
 ****************************************************************************/

static void low_prio_worker(void *arg)
{
  /* Invoke MPSL low priority process handler. This will call on_hci()
   * internally when required.
   */

  nxmutex_lock(&g_sdc_dev.lock);
  if (g_sdc_dev.mpsl_initialized)
    {
      mpsl_low_priority_process();
    }

  nxmutex_unlock(&g_sdc_dev.lock);
}

/****************************************************************************
 * Name: on_hci_worker
 ****************************************************************************/

static void on_hci_worker(void *arg)
{
  uint8_t msg_buffer[HCI_MSG_BUFFER_MAX_SIZE];
  sdc_hci_msg_type_t type;
  bool check_again;
  size_t len;
  int ret;

  do
    {
      check_again = false;

      /* Check for EVT by trying to get pending data into a generic
       * buffer and then create an actual bt_buf_s, depending on msg length
       */

      nxmutex_lock(&g_sdc_dev.lock);
      if (!g_sdc_dev.opened)
        {
          nxmutex_unlock(&g_sdc_dev.lock);
          return;
        }

      ret = sdc_hci_get(msg_buffer, &type);
      nxmutex_unlock(&g_sdc_dev.lock);

      if (ret == 0)
        {
          if (type == SDC_HCI_MSG_TYPE_EVT)
            {
              struct bt_hci_evt_hdr_s *hdr =
                (struct bt_hci_evt_hdr_s *)msg_buffer;

              len = sizeof(*hdr) + hdr->len;

#ifdef CONFIG_DEBUG_WIRELESS_INFO
              if (hdr->evt == BT_HCI_EVT_CMD_COMPLETE)
                {
                  struct hci_evt_cmd_complete_s *cmd_complete =
                    (struct hci_evt_cmd_complete_s *)
                    (msg_buffer + sizeof(*hdr));
                  uint8_t *status = (uint8_t *)cmd_complete + 3;

                  wlinfo("received CMD_COMPLETE from softdevice "
                         "(opcode: 0x%x, status: 0x%x)\n",
                         cmd_complete->opcode, *status);
                }
              else
                {
                  wlinfo("received HCI EVT from softdevice "
                         "(evt: %d, len: %zu)\n", hdr->evt, len);
                }
#endif

              bt_netdev_receive(&g_bt_driver, BT_EVT,
                                msg_buffer, len);
              check_again = true;
            }

          if (type == SDC_HCI_MSG_TYPE_DATA)
            {
              struct bt_hci_acl_hdr_s *hdr =
                (struct bt_hci_acl_hdr_s *)msg_buffer;

              wlinfo("received HCI ACL from softdevice (handle: %d)\n",
                     hdr->handle);

              len = sizeof(*hdr) + hdr->len;

              bt_netdev_receive(&g_bt_driver, BT_ACL_IN,
                                msg_buffer, len);
              check_again = true;
            }
        }
    }
  while (check_again);
}

/****************************************************************************
 * Name: on_hci
 *
 * Description:
 *   SDC message callback.
 *
 ****************************************************************************/

static void on_hci(void)
{
  work_queue(LPWORK, &g_sdc_dev.hci_work, on_hci_worker, NULL, 0);
}

/****************************************************************************
 * Name: swi_isr
 ****************************************************************************/

static int swi_isr(int irq, void *context, void *arg)
{
  work_queue(LPWORK, &g_sdc_dev.work, low_prio_worker, NULL, 0);

  return 0;
}

/****************************************************************************
 * Name: power_clock_isr
 ****************************************************************************/

static int power_clock_isr(int irq, void *context, void *arg)
{
  MPSL_IRQ_CLOCK_HANDLER();

  return 0;
}

/****************************************************************************
 * Name: rtc0_handler
 ****************************************************************************/

static void rtc0_handler(void)
{
  MPSL_IRQ_RTC0_HANDLER();
}

/****************************************************************************
 * Name: timer0_handler
 ****************************************************************************/

static void timer0_handler(void)
{
  MPSL_IRQ_TIMER0_HANDLER();
}

/****************************************************************************
 * Name: radio_handler
 ****************************************************************************/

static void radio_handler(void)
{
  MPSL_IRQ_RADIO_HANDLER();
}

#ifdef HAVE_BTADDR_CONFIGURE
/****************************************************************************
 * Name: nrf54l_sdc_btaddr_configure
 ****************************************************************************/

static int nrf54l_sdc_btaddr_configure(void)
{
#if CONFIG_NRF54L_SDC_PUB_ADDR > 0
  sdc_hci_cmd_vs_zephyr_write_bd_addr_t pub_addr;
#endif
#ifdef CONFIG_NRF54L_SDC_FICR_STATIC_ADDR
  sdc_hci_cmd_le_set_random_address_t   rand_addr;
  uint32_t                              addr[2];
  uint32_t                              addrtype = 0;
#endif
  int                                   ret      = OK;

#ifdef CONFIG_NRF54L_SDC_FICR_STATIC_ADDR
  /* Get device address type */

  addrtype = getreg32(NRF54L_FICR_DEVICEADDRTYPE);

  /* Get device addr from FICR */

  addr[0] = getreg32(NRF54L_FICR_DEVICEADDR(0));
  addr[1] = getreg32(NRF54L_FICR_DEVICEADDR(1));

  if ((addrtype & 0x01) == FICR_DEVICEADDRTYPE_RANDOM)
    {
      /* Configure static random address */

      memcpy(&rand_addr.random_address[0], &addr[0], 4);
      memcpy(&rand_addr.random_address[4], &addr[1], 2);

      /* The two most significant bits of the address shall be set */

      rand_addr.random_address[5] |= 0xc0;

      ret = sdc_hci_cmd_le_set_random_address(&rand_addr);
      if (ret != 0)
        {
          wlerr("sdc_hci_cmd_le_set_random_address failed: %d\n", ret);
          ret = -EIO;
          goto errout;
        }
    }
  else
    {
      wlerr("Static random address not available\n");
      ret = -EINVAL;
      goto errout;
    }
#endif

#if CONFIG_NRF54L_SDC_PUB_ADDR > 0
  /* Configure public address if available */

  pub_addr.bd_addr[0] = (CONFIG_NRF54L_SDC_PUB_ADDR >> (8 * 5)) & 0xff;
  pub_addr.bd_addr[1] = (CONFIG_NRF54L_SDC_PUB_ADDR >> (8 * 4)) & 0xff;
  pub_addr.bd_addr[2] = (CONFIG_NRF54L_SDC_PUB_ADDR >> (8 * 3)) & 0xff;
  pub_addr.bd_addr[3] = (CONFIG_NRF54L_SDC_PUB_ADDR >> (8 * 2)) & 0xff;
  pub_addr.bd_addr[4] = (CONFIG_NRF54L_SDC_PUB_ADDR >> (8 * 1)) & 0xff;
  pub_addr.bd_addr[5] = (CONFIG_NRF54L_SDC_PUB_ADDR >> (8 * 0)) & 0xff;

  ret = sdc_hci_cmd_vs_zephyr_write_bd_addr(&pub_addr);
  if (ret != 0)
    {
      wlerr("sdc_hci_cmd_vs_zephyr_write_bd_addr failed: %d\n", ret);
      ret = -EIO;
      goto errout;
    }
#endif

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: nrf54l_rand_poll
 ****************************************************************************/

static void nrf54l_rand_poll(uint8_t *p_buff, uint8_t length)
{
  arc4random_buf(p_buff, length);
}

/****************************************************************************
 * Name: nrf54l_configure_features
 ****************************************************************************/

static int nrf54l_configure_features(void)
{
  /* Turn on specific features */

#ifdef CONFIG_NRF54L_SDC_ADVERTISING
  sdc_support_adv();
#endif

#if defined(CONFIG_NRF54L_SDC_SCANNING) && NRF54L_SDC_CENTRAL_COUNT == 0
  sdc_support_scan();
#endif

#if NRF54L_SDC_CENTRAL_COUNT > 0
  sdc_support_central();
#endif

#if CONFIG_NRF54L_SDC_PERIPHERAL_COUNT > 0
  sdc_support_peripheral();
#endif

#ifdef CONFIG_NRF54L_SDC_DLE
#  if NRF54L_SDC_CENTRAL_COUNT > 0
  sdc_support_dle_central();
#  endif

#  if CONFIG_NRF54L_SDC_PERIPHERAL_COUNT > 0
  sdc_support_dle_peripheral();
#  endif
#endif

#ifdef CONFIG_NRF54L_SDC_LE_2M_PHY
  sdc_support_le_2m_phy();
#endif

#ifdef CONFIG_NRF54L_SDC_LE_CODED_PHY
  sdc_support_le_coded_phy();
#endif

#if defined(CONFIG_NRF54L_SDC_LE_2M_PHY) || \
    defined(CONFIG_NRF54L_SDC_LE_CODED_PHY)
#  if NRF54L_SDC_CENTRAL_COUNT > 0
  sdc_support_phy_update_central();
#  endif
#  if CONFIG_NRF54L_SDC_PERIPHERAL_COUNT > 0
  sdc_support_phy_update_peripheral();
#  endif
#endif

  return OK;
}

/****************************************************************************
 * Name: nrf54l_configure_memory
 ****************************************************************************/

static int nrf54l_configure_memory(void)
{
  int32_t   required_memory = 0;
  int       ret             = OK;
  sdc_cfg_t cfg;

  /* Configure SoftDevice memory.
   * sdc_support_*() calls must precede sdc_cfg_set() and sdc_enable().
   */

  /* Configure scanner memory */

#if SCAN_MEM_COUNT > 0
  cfg.scan_buffer_cfg.count = SCAN_MEM_COUNT;
  ret = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
                    SDC_CFG_TYPE_SCAN_BUFFER_CFG, &cfg);
  if (ret < 0)
    {
      wlerr("Failed to set scan count: %d\n", ret);
      goto errout;
    }
#endif

  /* Configure advertisers memory */

  cfg.adv_count.count = ADV_SET_COUNT;
  ret = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
                    SDC_CFG_TYPE_ADV_COUNT, &cfg);
  if (ret < 0)
    {
      wlerr("Failed to set advertising count: %d\n", ret);
      goto errout;
    }

#if ADV_SET_COUNT > 0
  cfg.adv_buffer_cfg.max_adv_data = ADV_BUF_SIZE;
  ret = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
                    SDC_CFG_TYPE_ADV_BUFFER_CFG, &cfg);
  if (ret < 0)
    {
      wlerr("Failed to set advertising buffer: %d\n", ret);
      goto errout;
    }
#endif

  /* Configure central connections memory */

  cfg.central_count.count = NRF54L_SDC_CENTRAL_COUNT;
  ret = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
                    SDC_CFG_TYPE_CENTRAL_COUNT, &cfg);
  if (ret < 0)
    {
      wlerr("Failed to set central role count: %d\n", ret);
      goto errout;
    }

  /* Configure peripheral connections memory */

  cfg.peripheral_count.count = CONFIG_NRF54L_SDC_PERIPHERAL_COUNT;
  ret = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
                    SDC_CFG_TYPE_PERIPHERAL_COUNT, &cfg);
  if (ret < 0)
    {
      wlerr("Failed to set peripheral role count: %d\n", ret);
      goto errout;
    }

  /* Configure buffers memory and get the final required memory */

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
      ret = required_memory;
      goto errout;
    }

  /* Use the exact size returned by this version of the controller. */

  g_sdc_dev.mempool = kmm_memalign(8, required_memory);
  ret = OK;
  if (g_sdc_dev.mempool == NULL)
    {
      ret = -ENOMEM;
    }
  else
    {
      memset(g_sdc_dev.mempool, 0, required_memory);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_configure_rand
 ****************************************************************************/

static int nrf54l_configure_rand(void)
{
  sdc_rand_source_t rand_func;
  int               ret = OK;

  /* Enable rand source */

  rand_func.rand_poll = nrf54l_rand_poll;

  ret = sdc_rand_source_register(&rand_func);
  if (ret < 0)
    {
      wlerr("sdc_rand_source_register failed %d\n", ret);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_sdc_reset
 *
 * Description:
 *   Reset the controller and restore the configured identity address.
 *   Called with the controller mutex held; returns an HCI status byte.
 *
 ****************************************************************************/

uint8_t nrf54l_sdc_reset(void)
{
  uint8_t status = sdc_hci_cmd_cb_reset();

#ifdef HAVE_BTADDR_CONFIGURE
  if (status == 0 && nrf54l_sdc_btaddr_configure() < 0)
    {
      status = 0x1f; /* Unspecified Error */
    }
#endif

  return status;
}

/****************************************************************************
 * Name: mpsl_low_latency_acquire_callback
 ****************************************************************************/

void mpsl_low_latency_acquire_callback(void)
{
  /* MPSL can coalesce adjacent windows and omit intermediate releases. */

  if (!g_low_latency)
    {
      g_rram_lowpower = getreg32(NRF54L_RRAMC_POWER_LOWPOWERCONFIG);
      g_constlat = (getreg32(NRF54L_POWER_CONSTLATSTAT) &
                    POWER_CONSTLATSTAT_STATUS) != 0;
      g_low_latency = true;
    }

  putreg32(POWER_TASKS_CONSTLAT, NRF54L_POWER_TASKS_CONSTLAT);
  putreg32((g_rram_lowpower & ~RRAMC_LOWPOWER_MODE_MASK) |
           RRAMC_LOWPOWER_STANDBY, NRF54L_RRAMC_POWER_LOWPOWERCONFIG);
}

/****************************************************************************
 * Name: mpsl_low_latency_release_callback
 ****************************************************************************/

void mpsl_low_latency_release_callback(void)
{
  if (g_low_latency)
    {
      putreg32(g_rram_lowpower, NRF54L_RRAMC_POWER_LOWPOWERCONFIG);
      if (!g_constlat)
        {
          putreg32(POWER_TASKS_LOWPWR, NRF54L_POWER_TASKS_LOWPWR);
        }

      g_low_latency = false;
    }
}

/****************************************************************************
 * Name: nrf54l_sdc_initialize
 ****************************************************************************/

int nrf54l_sdc_initialize(void)
{
#ifndef CONFIG_NRF54L_SYSTIMER_GRTC
  struct nrf54l_grtc_dev_s *grtc;
#endif
  int ret = OK;

  ret = nxmutex_lock(&g_sdc_dev.lock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_sdc_dev.initialized || g_sdc_dev.initializing)
    {
      ret = g_sdc_dev.initialized ? OK : -EBUSY;
      nxmutex_unlock(&g_sdc_dev.lock);
      return ret;
    }

  g_sdc_dev.initializing = true;

  ret = mpsl_clock_ctrl_source_register(&g_lfclk_source, &g_hfclk_source);
  if (ret < 0)
    {
      wlerr("mpsl_clock_ctrl_source_register failed: %d\n", ret);
      g_sdc_dev.initializing = false;
      nxmutex_unlock(&g_sdc_dev.lock);
      return ret;
    }

#ifndef CONFIG_NRF54L_SYSTIMER_GRTC
  grtc = nrf54l_grtc_init(0);
  if (grtc == NULL)
    {
      ret = -EBUSY;
      goto errout_clocks;
    }

  ret = NRF54L_GRTC_START(grtc);
  if (ret < 0)
    {
      goto errout_grtc;
    }
#endif

  /* Register interrupt handler for normal-priority events. SWI3 will be
   * used by MPSL to delegate low-priority work
   */

  ret = irq_attach(NRF54L_IRQ_SWI03, swi_isr, NULL);
  if (ret < 0)
    {
      goto errout_grtc;
    }

  ret = irq_attach(NRF54L_IRQ_CLOCK_POWER, power_clock_isr, NULL);
  if (ret < 0)
    {
      goto errout_swi;
    }

  up_enable_irq(NRF54L_IRQ_SWI03);
  up_enable_irq(NRF54L_IRQ_CLOCK_POWER);

  up_prioritize_irq(NRF54L_IRQ_SWI03, NVIC_SYSH_PRIORITY_DEFAULT);
  up_prioritize_irq(NRF54L_IRQ_CLOCK_POWER, NVIC_SYSH_PRIORITY_DEFAULT);

  /* Register high-priority interrupts for specific peripherals */

  ret = arm_ramvec_attach(NRF54L_IRQ_GRTC_3, rtc0_handler);
  if (ret < 0)
    {
      goto errout_power;
    }

  ret = arm_ramvec_attach(NRF54L_IRQ_TIMER10, timer0_handler);
  if (ret < 0)
    {
      goto errout_rtc;
    }

  ret = arm_ramvec_attach(NRF54L_IRQ_RADIO_0, radio_handler);
  if (ret < 0)
    {
      goto errout_timer;
    }

  up_prioritize_irq(NRF54L_IRQ_GRTC_3, MPSL_HIGH_IRQ_PRIORITY);
  up_prioritize_irq(NRF54L_IRQ_TIMER10, MPSL_HIGH_IRQ_PRIORITY);
  up_prioritize_irq(NRF54L_IRQ_RADIO_0, MPSL_HIGH_IRQ_PRIORITY);

  up_enable_irq(NRF54L_IRQ_GRTC_3);
  up_enable_irq(NRF54L_IRQ_TIMER10);
  up_enable_irq(NRF54L_IRQ_RADIO_0);

  /* Initialize MPSL */

  ret = mpsl_init(NULL, NRF54L_IRQ_SWI03 - NRF54L_IRQ_EXTINT,
                  &mpsl_assert_handler);
  if (ret < 0)
    {
      wlerr("mpsl_init failed: %d\n", ret);
      goto errout_radio;
    }

  g_sdc_dev.mpsl_initialized = true;

  /* Initialize SDC */

  ret = sdc_init(&sdc_fault_handler);
  if (ret < 0)
    {
      wlerr("sdc_init failed: %d\n", ret);
      goto errout_mpsl;
    }

  /* Configure SoftDevice random sources */

  ret = nrf54l_configure_rand();
  if (ret < 0)
    {
      wlerr("nrf54l_configure_rand failed: %d\n", ret);
      goto errout_mpsl;
    }

  /* Configure SoftDevice features */

  ret = nrf54l_configure_features();
  if (ret < 0)
    {
      wlerr("nrf54l_configure_features failed: %d\n", ret);
      goto errout_mpsl;
    }

  /* Configure SoftDevice memory */

  ret = nrf54l_configure_memory();
  if (ret < 0)
    {
      wlerr("nrf54l_configure_memory failed: %d\n", ret);
      goto errout_mpsl;
    }

  /* Finally enable SoftDevice Controller */

  ret = sdc_enable(on_hci, g_sdc_dev.mempool);
  if (ret < 0)
    {
      wlerr("sdc_enable failed: %d\n", ret);
      goto errout_mem;
    }

#ifdef HAVE_BTADDR_CONFIGURE
  ret = nrf54l_sdc_btaddr_configure();
  if (ret < 0)
    {
      wlerr("Could not configure BT addr: %d\n", ret);
      goto errout_sdc;
    }
#endif

  /* Registration can start the host, which calls back into this driver. */

  nxmutex_unlock(&g_sdc_dev.lock);
  ret = bt_driver_register(&g_bt_driver);
  nxmutex_lock(&g_sdc_dev.lock);
  if (ret < 0)
    {
      wlerr("bt_driver_register error: %d\n", ret);
      goto errout_sdc;
    }

  g_sdc_dev.initialized = true;
  g_sdc_dev.initializing = false;
  nxmutex_unlock(&g_sdc_dev.lock);
  return OK;

errout_sdc:
  sdc_disable();
  g_sdc_dev.opened = false;
  work_cancel(LPWORK, &g_sdc_dev.work);
  work_cancel(LPWORK, &g_sdc_dev.hci_work);

errout_mem:
  kmm_free(g_sdc_dev.mempool);
  g_sdc_dev.mempool = NULL;

errout_mpsl:
  mpsl_uninit();
  g_sdc_dev.mpsl_initialized = false;

errout_radio:
  up_disable_irq(NRF54L_IRQ_RADIO_0);
  up_disable_irq(NRF54L_IRQ_TIMER10);
  up_disable_irq(NRF54L_IRQ_GRTC_3);
  arm_ramvec_attach(NRF54L_IRQ_RADIO_0, NULL);

errout_timer:
  arm_ramvec_attach(NRF54L_IRQ_TIMER10, NULL);

errout_rtc:
  arm_ramvec_attach(NRF54L_IRQ_GRTC_3, NULL);

errout_power:
  up_disable_irq(NRF54L_IRQ_CLOCK_POWER);
  up_disable_irq(NRF54L_IRQ_SWI03);
  irq_detach(NRF54L_IRQ_CLOCK_POWER);

errout_swi:
  irq_detach(NRF54L_IRQ_SWI03);

errout_grtc:
#ifndef CONFIG_NRF54L_SYSTIMER_GRTC
  nrf54l_grtc_deinit(grtc);

errout_clocks:
#endif
  mpsl_clock_ctrl_source_unregister();
  g_sdc_dev.initializing = false;
  nxmutex_unlock(&g_sdc_dev.lock);
  return ret;
}
