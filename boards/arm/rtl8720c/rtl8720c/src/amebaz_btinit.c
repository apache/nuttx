/****************************************************************************
 * boards/arm/ameba/amebaZ/src/amebaz_btinit.c
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

#include <unistd.h>

#include <sys/types.h>

#include <nuttx/wireless/bluetooth/bt_uart.h>
#include <nuttx/wireless/bluetooth/bt_uart_shim.h>

#include "arm_arch.h"

#include "amebaz.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_PHY_EFUSE_BASE         (0x100)
#define BT_PHY_EFUSE_IQK_xx(a)    (a[3] | a[4] << 8)
#define BT_PHY_EFUSE_IQK_yy(a)    (a[5] | a[6] << 8)
#define BT_PHY_EFUSE_QDAC(a)      (a[12])
#define BT_PHY_EFUSE_IDAC(a)      (a[13])
#define BT_PHY_EFUSE_QDAC2(a)     (a[14])
#define BT_PHY_EFUSE_IDAC2(a)     (a[15])

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amebaz_bt_controller_reset
 ****************************************************************************/

static void amebaz_bt_controller_reset(void)
{
  modifyreg32(0x40000244, 3 << 8, 3 << 8);
  usleep(5 * 1000);
  modifyreg32(0x40000214, 3 << 24, 0);
  usleep(50 * 1000);
  modifyreg32(0x40000214, 3 << 24, 3 << 24);
  usleep(50 * 1000);

#if 0
  extern uint32_t bt_flatk_8710c(uint16_t txgain_flatk);
  extern uint32_t bt_lok_write(uint16_t idac, uint16_t qdac, uint16_t idac2, uint16_t qdac2);
  extern uint32_t bt_dck_write(uint8_t q_dck, uint8_t i_dck);
  extern uint32_t hal_efuse_read(uint16_t addr, uint8_t *pdata, uint8_t l25out_voltage);
  uint8_t hci_tp_phy_efuse[16];
  int i;

  bt_flatk_8710c(0xfff0);

  for (i = 0; i < 16; i++)
    {
      hal_efuse_read(BT_PHY_EFUSE_BASE + i, hci_tp_phy_efuse + i, 0);
    }

  if (hci_tp_phy_efuse[0] != 0)
    {
      bt_dck_write(BT_PHY_EFUSE_QDAC2(hci_tp_phy_efuse),
                   BT_PHY_EFUSE_IDAC2(hci_tp_phy_efuse));
    }

  bt_lok_write(BT_PHY_EFUSE_IDAC(hci_tp_phy_efuse),
               BT_PHY_EFUSE_QDAC(hci_tp_phy_efuse),
               BT_PHY_EFUSE_IDAC2(hci_tp_phy_efuse),
               BT_PHY_EFUSE_QDAC2(hci_tp_phy_efuse));
#endif
}

/****************************************************************************
 * Name: amebaz_bt_stack_initialize
 ****************************************************************************/

#if defined(CONFIG_BLUETOOTH_UART_OTHER)
static int bt_uart_init(void)
{
  struct btuart_lowerhalf_s *lower;

  lower = bt_uart_shim_getdevice(CONFIG_AMEBA_HCI_PROXY_DEV_NAME);

  if (lower)
    {
      return btuart_register(lower);
    }

  return -1;
}
#endif

/****************************************************************************
 * Name: amebaz_bt_initialize
 ****************************************************************************/

int amebaz_bt_initialize(void)
{
  extern void rtlk_bt_set_gnt_bt(int type);
  extern void rtlk_bt_pta_init(void);
  int ret;

  rtlk_bt_set_gnt_bt(1);

  amebaz_bt_controller_reset();

  ret = amebaz_bt_hci_uart_register(CONFIG_AMEBA_HCI_PROXY_DEV_NAME);
  if (ret < 0)
    return ret;

#if defined(CONFIG_BLUETOOTH_UART_OTHER) || defined(CONFIG_BT)
  usleep(10 * 1000);
#if defined(CONFIG_BT)
  extern int bt_uart_init(void);
  typedef void (*bt_ready_cb_t)(int err);
  extern int bt_enable(bt_ready_cb_t cb);
  bt_uart_init();
  bt_enable(NULL);
#else
  bt_uart_init();
#endif
#endif

  usleep(10 * 1000);
  rtlk_bt_pta_init();

  return OK;
}
