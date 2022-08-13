/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_mbox.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_STM32WB_MBOX_H
#define __ARCH_ARM_SRC_STM32WB_STM32WB_MBOX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/wireless/bluetooth/bt_hci.h>

#include "stm32wb_mbox_list.h"
#include "stm32wb_mbox_shci.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Mailbox channels */

#define STM32WB_MBOX_BLEEVT_CHANNEL       1
#define STM32WB_MBOX_BLECMD_CHANNEL       1
#define STM32WB_MBOX_SYSEVT_CHANNEL       2
#define STM32WB_MBOX_SYSCMD_CHANNEL       2
#define STM32WB_MBOX_EVT_RELEASE_CHANNEL  4
#define STM32WB_MBOX_BLEACL_CHANNEL       6

/* Mailbox packet types */

#define STM32WB_MBOX_HCICMD               0x01
#define STM32WB_MBOX_HCIACL               0x02
#define STM32WB_MBOX_HCIEVT               0x04
#define STM32WB_MBOX_SYSCMD               0x10
#define STM32WB_MBOX_SYSEVT               0x12
#define STM32WB_MBOX_SYSACK               0xe0

/* Mailbox configuration helpers */

#define STM32WB_MBOX_BLE_ATT_DEFAULT_MTU  23
#define STM32WB_MBOX_C2_MEM_BLOCK_SZ      32

#define DIV_UP(a, b)    (((a) + (b) - 1) / (b))

#define STM32WB_MBOX_DEFAULT_BLE_PREP_WRITE_NUM(max_mtu) \
  (DIV_UP((max_mtu), STM32WB_MBOX_BLE_ATT_DEFAULT_MTU - 5) * 2)

#define STM32WB_MBOX_DEFAULT_C2_MEM_BLOCK_NUM(max_mtu, max_conn, pw) \
  ((pw) + ((max_conn) + 1) * (DIV_UP((max_mtu) + 4, STM32WB_MBOX_C2_MEM_BLOCK_SZ) + 2))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Mailbox data transfer packets */

begin_packed_struct struct stm32wb_mbox_evt_s
{
  uint8_t                       type;
  union
    {
      struct bt_hci_evt_hdr_s   evt_hdr;
      struct bt_hci_acl_hdr_s   acl_hdr;
    };
} end_packed_struct;

begin_packed_struct struct stm32wb_mbox_cmd_s
{
  stm32wb_mbox_list_t           list_hdr;
  uint8_t                       type;
  union
  {
    struct bt_hci_cmd_hdr_s     cmd_hdr;
    struct bt_hci_acl_hdr_s     acl_hdr;
  };
} end_packed_struct;

/* Mailbox receive event handler type */

typedef int (*stm32wb_mbox_evt_handler_t)(struct stm32wb_mbox_evt_s *);

/****************************************************************************
 * Public Function Prototypes
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

void stm32wb_mboxinitialize(stm32wb_mbox_evt_handler_t evt_handler);

/****************************************************************************
 * Name: stm32wb_mboxenable
 *
 * Description:
 *   Enable mailbox hardware and start communication.  The CPU2 responses
 *   with C2Ready event on success.
 *
 ****************************************************************************/

void stm32wb_mboxenable(void);

/****************************************************************************
 * Name: stm32wb_mbox_syscmd
 *
 * Description:
 *   Send command over mailbox system channel.  Command data must be
 *   prepended with HCI header.
 *
 ****************************************************************************/

int stm32wb_mbox_syscmd(void *data, size_t len);

/****************************************************************************
 * Name: stm32wb_mbox_blecmd
 *
 * Description:
 *   Send command over mailbox BLE channel.  Command data must be
 *   prepended with HCI header.
 *
 ****************************************************************************/

int stm32wb_mbox_blecmd(void *data, size_t len);

/****************************************************************************
 * Name: stm32wb_mbox_bleacl
 *
 * Description:
 *   Send BLE ACL data over mailbox BLE ACL channel.  Data must be
 *   prepended with HCI ACL header.
 *
 ****************************************************************************/

int stm32wb_mbox_bleacl(void *data, size_t len);

/****************************************************************************
 * Name: stm32wb_mbox_bleinit
 *
 * Description:
 *   Initialize and start BLE subsystem with provided configuration params.
 *
 ****************************************************************************/

void stm32wb_mbox_bleinit(struct stm32wb_shci_ble_init_cfg_s *params);

#endif /* __ARCH_ARM_SRC_STM32WB_STM32WB_MBOX_H */
