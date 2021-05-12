/****************************************************************************
 * arch/arm/src/ameba/ameba_hci_board.h
 *
 *   Copyright (C) 2021 Xiaomi Inc. All rights reserved.
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
 ****************************************************************************/

#ifndef __INCLUDE_AMEBA_HCI_BOARD_H
#define __INCLUDE_AMEBA_HCI_BOARD_H

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

int hci_set_init_config_mac(FAR uint8_t* addr, uint8_t diffvalue);
int hci_get_baudrate(FAR uint32_t *bt_baudrate, FAR uint32_t *uart_baudrate);
int hci_find_fw_patch(uint8_t chipid);
int hci_get_efuse_iqk_data(uint8_t *data);

int hci_board_init(void);
int hci_board_init_done(void);

#endif /* __INCLUDE_AMEBA_FLASH_H */

