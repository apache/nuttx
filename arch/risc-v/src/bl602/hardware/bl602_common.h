/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_common.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_COMMON_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL602_FLASH_XIP_BASE        (0x23000000)
#define BL602_FLASH_XIP_END         (0x23000000 + 16 * 1024 * 1024)
#define BL602_FLASH_XIP_REMAP0_BASE (0x33000000)
#define BL602_FLASH_XIP_REMAP0_END  (0x33000000 + 16 * 1024 * 1024)
#define BL602_FLASH_XIP_REMAP1_BASE (0x43000000)
#define BL602_FLASH_XIP_REMAP1_END  (0x43000000 + 16 * 1024 * 1024)
#define BL602_FLASH_XIP_REMAP2_BASE (0x53000000)
#define BL602_FLASH_XIP_REMAP2_END  (0x53000000 + 16 * 1024 * 1024)

#define BL602_WRAM_BASE        (0x42020000)
#define BL602_WRAM_END         (0x42020000 + 176 * 1024)
#define BL602_WRAM_REMAP0_BASE (0x22020000)
#define BL602_WRAM_REMAP0_END  (0x22020000 + 176 * 1024)
#define BL602_WRAM_REMAP1_BASE (0x32020000)

#define BL602_WRAM_REMAP1_END  (0x32020000 + 176 * 1024)
#define BL602_WRAM_REMAP2_BASE (0x52020000)
#define BL602_WRAM_REMAP2_END  (0x52020000 + 176 * 1024)

#define BL602_TCM_BASE        (0x22008000)
#define BL602_TCM_END         (0x22008000 + (96 + 176) * 1024)
#define BL602_TCM_REMAP0_BASE (0x32008000)
#define BL602_TCM_REMAP0_END  (0x32008000 + (96 + 176) * 1024)
#define BL602_TCM_REMAP1_BASE (0x42008000)
#define BL602_TCM_REMAP1_END  (0x42008000 + (96 + 176) * 1024)
#define BL602_TCM_REMAP2_BASE (0x52008000)
#define BL602_TCM_REMAP2_END  (0x52008000 + (96 + 176) * 1024)

/* BL602 peripherals base address */

#define GLB_BASE (0x40000000)
#define RF_BASE  (0x40001000)

/* AUX module base address */

#define GPIP_BASE (0x40002000)

/* Security Debug module base address */

#define SEC_DBG_BASE (0x40003000)

/* Security Engine module base address */

#define SEC_ENG_BASE (0x40004000)

/* Trustzone control security base address */

#define TZC_SEC_BASE (0x40005000)

/* Trustzone control none-security base address */

#define TZC_NSEC_BASE (0x40006000)
#define EF_DATA_BASE  (0x40007000)
#define EF_CTRL_BASE  (0x40007000)
#define CCI_BASE      (0x40008000)

/* L1 cache config base address */

#define L1C_BASE         (0x40009000)
#define UART0_BASE       (0x4000A000)
#define UART1_BASE       (0x4000A100)
#define SPI_BASE         (0x4000A200)
#define I2C_BASE         (0x4000A300)
#define PWM_BASE         (0x4000A400)
#define TIMER_BASE       (0x4000A500)
#define IR_BASE          (0x4000A600)
#define SF_CTRL_BASE     (0x4000B000)
#define SF_CTRL_BUF_BASE (0x4000B700)
#define DMA_BASE         (0x4000C000)
#define SDU_BASE         (0x4000D000)

/* Power down sleep module base address */

#define PDS_BASE (0x4000E000)

/* Hibernate module base address */

#define HBN_BASE (0x4000F000)

/* Always on module base address */

#define AON_BASE     (0x4000F000)
#define HBN_RAM_BASE (0x40010000)

/* Delay for a while */

#define BL_DRV_DUMMY() \
  { \
    __asm volatile("nop"); \
    __asm volatile("nop"); \
    __asm volatile("nop"); \
    __asm volatile("nop"); \
  }

#define BL_AHB_SLAVE1_GLB                0x00
#define BL_AHB_SLAVE1_RF                 0x01
#define BL_AHB_SLAVE1_GPIP_PHY_AGC       0x02
#define BL_AHB_SLAVE1_SEC_DBG            0x03
#define BL_AHB_SLAVE1_SEC                0x04
#define BL_AHB_SLAVE1_TZ1                0x05
#define BL_AHB_SLAVE1_TZ2                0x06
#define BL_AHB_SLAVE1_EFUSE              0x07
#define BL_AHB_SLAVE1_CCI                0x08
#define BL_AHB_SLAVE1_L1C                0x09
#define BL_AHB_SLAVE1_RSVD0A             0x0a
#define BL_AHB_SLAVE1_SFC                0x0b
#define BL_AHB_SLAVE1_DMA                0x0c
#define BL_AHB_SLAVE1_SDU                0x0d
#define BL_AHB_SLAVE1_PDS_HBN_AON_HBNRAM 0x0e
#define BL_AHB_SLAVE1_RSVD0F             0x0f
#define BL_AHB_SLAVE1_UART0              0x10
#define BL_AHB_SLAVE1_UART1              0x11
#define BL_AHB_SLAVE1_SPI                0x12
#define BL_AHB_SLAVE1_I2C                0x13
#define BL_AHB_SLAVE1_PWM                0x14
#define BL_AHB_SLAVE1_TMR                0x15
#define BL_AHB_SLAVE1_IRR                0x16
#define BL_AHB_SLAVE1_CKS                0x17
#define BL_AHB_SLAVE1_MAX                0x18

/* Std driver attribute macro */

#define ATTR_CLOCK_SECTION       __attribute__((section(".sclock_rlt_code")))
#define ATTR_CLOCK_CONST_SECTION __attribute__((section(".sclock_rlt_const")))
#define ATTR_TCM_SECTION         __attribute__((section(".tcm_code")))
#define ATTR_TCM_CONST_SECTION   __attribute__((section(".tcm_const")))
#define ATTR_DTCM_SECTION        __attribute__((section(".tcm_data")))
#define ATTR_HSRAM_SECTION       __attribute__((section(".hsram_code")))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_COMMON_H */
