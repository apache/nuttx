/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_w1_wlan.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_W1_WLAN_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_W1_WLAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Interface of Microchip's pic32mzw1.a.  All of it is [EX]: see the tags
 * in pic32mz_w1_wlan.c.
 */

/* Completion callback of the crypto services (DRV_PIC32MZW_CRYPTO_CB) */

typedef CODE void (*wlan_crypto_cb_t)(int result, uintptr_t context);

struct pic32mz_wlan_init_s
{
  int alarm_1ms;
  int alarm_max;
};

struct pic32mz_wlan_pktmem_pri_s
{
  uint16_t num_resvd;
  uint16_t num_thresh;
  uint16_t num_allocd;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Packet buffer accounting per priority, owned by the library */

extern struct pic32mz_wlan_pktmem_pri_s g_pktmem_pri[5];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Library entry points */

bool wdrv_pic32mzw_init(FAR struct pic32mz_wlan_init_s *init);
void wdrv_pic32mzw_user_main(void);
void wdrv_pic32mzw_process_cfg_message(FAR uint8_t *msg);
void wdrv_pic32mzw_mac_controller_task(void);
void wdrv_pic32mzw_wlan_send_packet(FAR uint8_t *buf, uint16_t len,
                                    uint32_t tos, uint8_t offset);
void wdrv_pic32mzw_mac_isr(unsigned int vector);
void wdrv_pic32mzw_timer_tick_isr(unsigned int param);
void wdrv_pic32mzw_smc_isr(unsigned int param);

/* Callbacks the library expects from the host */

FAR void *DRV_PIC32MZW_MemAlloc(uint16_t size);
int8_t DRV_PIC32MZW_MemAddUsers(FAR void *buf, int count);
int8_t DRV_PIC32MZW_MemFree(FAR void *buf);
FAR void *DRV_PIC32MZW_PacketMemAlloc(uint16_t size, int prio);
void DRV_PIC32MZW_PacketMemFree(FAR void *buf);
void DRV_PIC32MZW_WIDRxQueuePush(FAR void *buf);

/* Output functions of the library, renamed (see Make.defs) */

int pic32mzw1_printf(FAR const IPTR char *fmt, ...) printf_like(1, 2);
int pic32mzw1_printf_npux(FAR const IPTR char *fmt, ...) printf_like(1, 2);
int pic32mzw1_puts(FAR const char *str);
int pic32mzw1_putchar(int c);

/* NuttX interface */

int pic32mz_wlan_initialize(void);

/* Run a crypto completion callback later from the WLAN thread */

void pic32mz_wlan_crypto_defer(wlan_crypto_cb_t cb, int result,
                               uintptr_t context);

#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_W1_WLAN_H */
