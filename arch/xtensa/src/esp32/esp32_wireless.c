/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wireless.c
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

#include <debug.h>
#include <assert.h>

#include "xtensa.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_dport.h"
#include "esp32_wireless.h"
#include "esp_phy_init.h"
#include "phy_init_data.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void phy_digital_regs_store(void);
static inline void phy_digital_regs_load(void);

/****************************************************************************
 * Extern Functions declaration
 ****************************************************************************/

extern void coex_bt_high_prio(void);
extern void phy_wakeup_init(void);
extern void phy_close_rf(void);
extern void phy_dig_reg_backup(bool init, uint32_t *regs);
extern int  register_chipv7_phy(const esp_phy_init_data_t *init_data,
                                esp_phy_calibration_data_t *cal_data,
                                esp_phy_calibration_mode_t cal_mode);

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
 * Name: esp32_phy_enable_clock
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

void esp32_phy_enable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_phy_clk_en_cnt == 0)
    {
      modifyreg32(DPORT_WIFI_CLK_EN_REG, 0,
                  DPORT_WIFI_CLK_WIFI_BT_COMMON_M);
    }

  g_phy_clk_en_cnt++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32_phy_disable_clock
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

void esp32_phy_disable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_phy_clk_en_cnt > 0)
    {
      g_phy_clk_en_cnt--;
      if (g_phy_clk_en_cnt == 0)
        {
          modifyreg32(DPORT_WIFI_CLK_EN_REG,
                      DPORT_WIFI_CLK_WIFI_BT_COMMON_M,
                      0);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_phy_disable
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

void esp32_phy_disable(void)
{
  irqstate_t flags;
  flags = enter_critical_section();

  g_phy_access_ref--;

  if (g_phy_access_ref == 0)
    {
      /* Disable PHY and RF. */

      phy_close_rf();

      /* Disable Wi-Fi/BT common peripheral clock.
       * Do not disable clock for hardware RNG.
       */

      esp32_phy_disable_clock();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32_phy_enable
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

void esp32_phy_enable(void)
{
  static bool debug = false;
  irqstate_t flags;
  esp_phy_calibration_data_t *cal_data;
  if (debug == false)
    {
      char *phy_version = get_phy_version_str();
      wlinfo("phy_version %s\n", phy_version);
      debug = true;
    }

  cal_data = kmm_zalloc(sizeof(esp_phy_calibration_data_t));
  if (!cal_data)
    {
      wlerr("ERROR: Failed to allocate PHY calibration data buffer.");
      abort();
    }

  flags = enter_critical_section();

  if (g_phy_access_ref == 0)
    {
      esp32_phy_enable_clock();
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

#ifdef CONFIG_ESP32_BLE
      coex_bt_high_prio();
#endif
    }

  g_phy_access_ref++;
  leave_critical_section(flags);
  kmm_free(cal_data);
}
