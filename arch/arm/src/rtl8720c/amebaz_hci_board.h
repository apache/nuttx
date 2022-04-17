/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_hci_board.h
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

#ifndef __ARCH_ARM_SRC_RTL8720C_AMEBA_HCI_BOARD_H
#define __ARCH_ARM_SRC_RTL8720C_AMEBA_HCI_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HCI_START_IQK
#define AMEBAZ_COMMAND_FRAGMENT_SIZE    (252)
#define AMEBAZ_COMMAND_DONE             (0)
#define AMEBAZ_COMMAND_VALID            (1)
#define BT_DEFAUT_LMP_SUBVER            0x8710

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int hci_check_iqk(void);
int hci_start_iqk(void);
int hci_set_init_config_mac(uint8_t *addr, uint8_t diffvalue);
int hci_get_baudrate(uint32_t *bt_baudrate,
                     uint32_t *uart_baudrate);
int hci_find_fw_patch(uint8_t chipid);
int hci_get_efuse_iqk_data(uint8_t *data);
int hci_board_init(void);
int hci_board_init_done(void);

#endif /* __ARCH_ARM_SRC_RTL8720C_AMEBA_HCI_BOARD_H */
