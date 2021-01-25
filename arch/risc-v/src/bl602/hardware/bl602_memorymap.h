/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_MEMORYMAP_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL602_CLIC_CTRL_BASE  0x02000000  /* clic */
#define BL602_CLIC_HART0_BASE 0x02800000  /* clic hart0 */
#define BL602_GLB_BASE        0x40000000  /* glb */
#define BL602_RF_BASE         0x40001000  /* rf */
#define BL602_GPIP_BASE       0x40002000  /* gpip */
#define BL602_SEC_DBG_BASE    0x40003000  /* sec_dbg */
#define BL602_SEC_ENG_BASE    0x40004000  /* sec_eng */
#define BL602_TZC_SEC_BASE    0x40005000  /* tzc_sec */
#define BL602_TZC_NSEC_BASE   0x40006000  /* tzc_nsec */
#define BL602_EF_BASE         0x40007000  /* ef_data_0 / ef_data_1 /ef_ctrl */
#define BL602_CCI_BASE        0x40008000  /* cci */
#define BL602_L1C_BASE        0x40009000  /* l1c */
#define BL602_UART0_BASE      0x4000a000  /* uart0 */
#define BL602_UART1_BASE      0x4000a100  /* uart1 */
#define BL602_SPI_BASE        0x4000a200  /* spi */
#define BL602_I2C_BASE        0x4000a300  /* i2c */
#define BL602_PWM_BASE        0x4000a400  /* pwm */
#define BL602_TIMER_BASE      0x4000a500  /* timer */
#define BL602_IR_BASE         0x4000a600  /* ir */
#define BL602_CKS_BASE        0x4000a000  /* cks (overlaps with UART0?) */
#define BL602_SF_BASE         0x4000b000  /* sf */
#define BL602_DMA_BASE        0x4000c000  /* dma */
#define BL602_PDS_BASE        0x4000e000  /* pds */
#define BL602_HBN_BASE        0x4000f000  /* hbn */
#define BL602_AON_BASE        0x4000f800  /* aon */

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_MEMORYMAP_H */
