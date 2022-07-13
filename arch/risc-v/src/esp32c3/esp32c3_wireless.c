/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_wireless.c
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
#include <nuttx/kmalloc.h>

#include <semaphore.h>
#include <debug.h>

#include "riscv_internal.h"
#include "hardware/esp32c3_system.h"
#include "hardware/esp32c3_soc.h"
#include "hardware/esp32c3_syscon.h"
#include "hardware/esp32c3_efuse.h"
#include "esp32c3_wireless.h"
#include "esp32c3.h"
#include "esp32c3_irq.h"
#include "esp32c3_attr.h"
#include "esp32c3_wireless.h"
#include "espidf_wifi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAC_ADDR0_REG EFUSE_RD_MAC_SPI_SYS_0_REG
#define MAC_ADDR1_REG EFUSE_RD_MAC_SPI_SYS_1_REG

/* Software Interrupt */

#define SWI_IRQ       ESP32C3_IRQ_FROM_CPU_INT0
#define SWI_PERIPH    ESP32C3_PERIPH_FROM_CPU_INT0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ESP32-C3 Wireless Private Data */

struct esp32c3_wl_priv_s
{
  volatile int ref;               /* Reference count */

  int cpuint;                     /* CPU interrupt assigned to SWI */

  struct list_node sc_list;       /* Semaphore cache list */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void phy_digital_regs_store(void);
static inline void phy_digital_regs_load(void);
static void esp32c3_phy_enable_clock(void);
static void esp32c3_phy_disable_clock(void);

/****************************************************************************
 * Extern Functions declaration
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_BLE
extern void coex_pti_v2(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Wi-Fi sleep private data */

static uint32_t g_phy_clk_en_cnt;

/* Reference count of enabling PHY */

static uint8_t g_phy_access_ref;

/* Memory to store PHY digital registers */

static uint32_t *g_phy_digital_regs_mem = NULL;

/* Indicate PHY is calibrated or not */

static bool g_is_phy_calibrated = false;

static struct esp32c3_wl_priv_s g_esp32c3_wl_priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_digital_regs_store
 *
 * Description:
 *    Store  PHY digital registers.
 *
 ****************************************************************************/

static inline void phy_digital_regs_store(void)
{
  if (g_phy_digital_regs_mem == NULL)
    {
      g_phy_digital_regs_mem = (uint32_t *)
                    kmm_malloc(SOC_PHY_DIG_REGS_MEM_SIZE);
    }

  DEBUGASSERT(g_phy_digital_regs_mem != NULL);

  phy_dig_reg_backup(true, g_phy_digital_regs_mem);
}

/****************************************************************************
 * Name: phy_digital_regs_load
 *
 * Description:
 *   Load  PHY digital registers.
 *
 ****************************************************************************/

static inline void phy_digital_regs_load(void)
{
  if (g_phy_digital_regs_mem != NULL)
    {
      phy_dig_reg_backup(false, g_phy_digital_regs_mem);
    }
}

/****************************************************************************
 * Name: esp32c3_phy_enable_clock
 *
 * Description:
 *   Enable PHY hardware clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32c3_phy_enable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_phy_clk_en_cnt == 0)
    {
      modifyreg32(SYSTEM_WIFI_CLK_EN_REG, 0,
                  SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M);
    }

  g_phy_clk_en_cnt++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32c3_phy_disable_clock
 *
 * Description:
 *   Disable PHY hardware clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32c3_phy_disable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_phy_clk_en_cnt)
    {
      g_phy_clk_en_cnt--;
      if (!g_phy_clk_en_cnt)
        {
          modifyreg32(SYSTEM_WIFI_CLK_EN_REG,
                      SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M,
                      0);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32c3_wl_swi_irq
 *
 * Description:
 *   Wireless software interrupt callback function.
 *
 * Parameters:
 *   cpuint  - CPU interrupt index
 *   context - Context data from the ISR
 *   arg     - NULL
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int esp32c3_wl_swi_irq(int irq, void *context, void *arg)
{
  int i;
  int ret;
  struct esp32c3_wl_semcache_s *sc;
  struct esp32c3_wl_semcache_s *tmp;
  struct esp32c3_wl_priv_s *priv = &g_esp32c3_wl_priv;

  putreg32(0, SYSTEM_CPU_INTR_FROM_CPU_0_REG);

  list_for_every_entry_safe(&priv->sc_list, sc, tmp,
                            struct esp32c3_wl_semcache_s, node)
    {
      for (i = 0; i < sc->count; i++)
        {
          ret = nxsem_post(sc->sem);
          if (ret < 0)
            {
              wlerr("ERROR: Failed to post sem ret=%d\n", ret);
            }
        }

      sc->count = 0;
      list_delete(&sc->node);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_phy_disable
 *
 * Description:
 *   Deinitialize PHY hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_phy_disable(void)
{
  irqstate_t flags;
  flags = enter_critical_section();

  g_phy_access_ref--;

  if (g_phy_access_ref == 0)
    {
      /* Store  PHY digital register. */

      phy_digital_regs_store();

      /* Disable PHY and RF. */

      phy_close_rf();

      phy_xpd_tsens();

      /* Disable Wi-Fi/BT common peripheral clock.
       * Do not disable clock for hardware RNG.
       */

      esp32c3_phy_disable_clock();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32c3_phy_enable
 *
 * Description:
 *   Initialize PHY hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_phy_enable(void)
{
  static bool debug = false;
  irqstate_t flags;
  esp_phy_calibration_data_t *cal_data;
  char *phy_version = get_phy_version_str();
  if (debug == false)
    {
      debug = true;
      wlinfo("phy_version %s\n", phy_version);
    }

  cal_data = kmm_zalloc(sizeof(esp_phy_calibration_data_t));
  if (!cal_data)
    {
      wlerr("ERROR: Failed to kmm_zalloc");
      DEBUGPANIC();
    }

  flags = enter_critical_section();

  if (g_phy_access_ref == 0)
    {
      esp32c3_phy_enable_clock();
      if (g_is_phy_calibrated == false)
        {
          register_chipv7_phy(&phy_init_data, cal_data, PHY_RF_CAL_FULL);
          g_is_phy_calibrated = true;
        }
      else
        {
          phy_wakeup_init();
          phy_digital_regs_load();
        }

#ifdef CONFIG_ESP32C3_BLE
      coex_pti_v2();
#endif
    }

  g_phy_access_ref++;
  leave_critical_section(flags);
  kmm_free(cal_data);
}

/****************************************************************************
 * Name: esp32c3_wl_init_semcache
 *
 * Description:
 *   Initialize semaphore cache.
 *
 * Parameters:
 *   sc  - Semaphore cache data pointer
 *   sem - Semaphore data pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_wl_init_semcache(struct esp32c3_wl_semcache_s *sc,
                              sem_t *sem)
{
  sc->sem   = sem;
  sc->count = 0;
  list_initialize(&sc->node);
}

/****************************************************************************
 * Name: esp32c3_wl_post_semcache
 *
 * Description:
 *   Store posting semaphore action into semaphore cache.
 *
 * Parameters:
 *   sc  - Semaphore cache data pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void IRAM_ATTR esp32c3_wl_post_semcache(
  struct esp32c3_wl_semcache_s *sc)
{
  struct esp32c3_wl_priv_s *priv = &g_esp32c3_wl_priv;

  if (!sc->count)
    {
      list_add_tail(&priv->sc_list, &sc->node);
    }

  sc->count++;

  putreg32(SYSTEM_CPU_INTR_FROM_CPU_0_M, SYSTEM_CPU_INTR_FROM_CPU_0_REG);
}

/****************************************************************************
 * Name: esp32c3_wl_init
 *
 * Description:
 *   Initialize ESP32-C3 wireless common components for both BT and Wi-Fi.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int esp32c3_wl_init(void)
{
  int ret;
  irqstate_t flags;
  struct esp32c3_wl_priv_s *priv = &g_esp32c3_wl_priv;

  flags = enter_critical_section();
  if (priv->ref != 0)
    {
      priv->ref++;
      leave_critical_section(flags);
      return OK;
    }

  priv->cpuint = esp32c3_request_irq(SWI_PERIPH,
                                     ESP32C3_INT_PRIO_DEF,
                                     ESP32C3_INT_LEVEL);

  ret = irq_attach(SWI_IRQ, esp32c3_wl_swi_irq, NULL);
  if (ret < 0)
    {
      esp32c3_free_cpuint(SWI_PERIPH);
      leave_critical_section(flags);
      wlerr("ERROR: Failed to attach IRQ ret=%d\n", ret);

      return ret;
    }

  list_initialize(&priv->sc_list);

  up_enable_irq(priv->cpuint);

  priv->ref++;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: esp32c3_wl_deinit
 *
 * Description:
 *   De-initialize ESP32-C3 wireless common components.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int esp32c3_wl_deinit(void)
{
  irqstate_t flags;
  struct esp32c3_wl_priv_s *priv = &g_esp32c3_wl_priv;

  flags = enter_critical_section();
  if (priv->ref == 0)
    {
      leave_critical_section(flags);
      return OK;
    }

  up_disable_irq(priv->cpuint);
  irq_detach(SWI_IRQ);
  esp32c3_free_cpuint(SWI_PERIPH);

  priv->ref--;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: esp_read_mac
 *
 * Description:
 *   Read MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *   type - MAC address type
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_read_mac(uint8_t *mac, esp_mac_type_t type)
{
  uint32_t regval[2];
  uint8_t tmp;
  uint8_t *data = (uint8_t *)regval;
  int i;

  if (type > ESP_MAC_BT)
    {
      wlerr("ERROR: Input type is error=%d\n", type);
      return -1;
    }

  regval[0] = getreg32(MAC_ADDR0_REG);
  regval[1] = getreg32(MAC_ADDR1_REG);

  for (i = 0; i < MAC_LEN; i++)
    {
      mac[i] = data[5 - i];
    }

  if (type == ESP_MAC_WIFI_SOFTAP)
    {
      tmp = mac[0];
      for (i = 0; i < 64; i++)
        {
          mac[0] = tmp | 0x02;
          mac[0] ^= i << 2;

          if (mac[0] != tmp)
            {
              break;
            }
        }

      if (i >= 64)
        {
          wlerr("ERROR: Failed to generate softAP MAC\n");
          return -1;
        }
    }

  if (type == ESP_MAC_BT)
    {
      tmp = mac[0];
      for (i = 0; i < 64; i++)
        {
          mac[0] = tmp | 0x02;
          mac[0] ^= i << 2;

          if (mac[0] != tmp)
            {
              break;
            }
        }

      mac[5] += 1;
    }

  return 0;
}
