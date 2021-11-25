/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_MEMORYMAP_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define MPFS_CLINT_BASE                (0x02000000UL)
#define MPFS_PDMA_BASE                 (0x03000000UL)
#define MPFS_PLIC_BASE                 (0x0C000000UL)

#define MPFS_UART0_LO_BASE             (0x20000000UL)
#define MPFS_WDOG0_LO_BASE             (0x20001000UL)
#define MPFS_SYSREG_BASE               (0x20002000UL)
#define MPFS_SYSREGSCB_BASE            (0x20003000UL)
#define MPFS_AXISW_BASE                (0x20004000UL)
#define MPFS_MPUCFG_BASE               (0x20005000UL)
#define MPFS_FMETER_BASE               (0x20006000UL)
#define MPFS_CFG_DDR_SGMII_PHY_BASE    (0x20007000UL)
#define MPFS_EMMC_SD_BASE              (0x20008000UL)
#define MPFS_DDRCFG_BASE               (0x20080000UL)
#define MPFS_UART1_LO_BASE             (0x20100000UL)
#define MPFS_WDOG1_LO_BASE             (0x20101000UL)
#define MPFS_UART2_LO_BASE             (0x20102000UL)
#define MPFS_WDOG2_LO_BASE             (0x20103000UL)
#define MPFS_UART3_LO_BASE             (0x20104000UL)
#define MPFS_WDOG3_LO_BASE             (0x20105000UL)
#define MPFS_UART4_LO_BASE             (0x20106000UL)
#define MPFS_WDOG4_LO_BASE             (0x20107000UL)
#define MPFS_SPI0_LO_BASE              (0x20108000UL)
#define MPFS_SPI1_LO_BASE              (0x20109000UL)
#define MPFS_I2C0_LO_BASE              (0x2010A000UL)
#define MPFS_I2C1_LO_BASE              (0x2010B000UL)
#define MPFS_CAN0_LO_BASE              (0x2010C000UL)
#define MPFS_CAN1_LO_BASE              (0x2010D000UL)
#define MPFS_GEM0_LO_BASE              (0x20110000UL)
#define MPFS_GEM1_LO_BASE              (0x20112000UL)
#define MPFS_GPIO0_LO_BASE             (0x20120000UL)
#define MPFS_GPIO1_LO_BASE             (0x20121000UL)
#define MPFS_GPIO2_LO_BASE             (0x20122000UL)
#define MPFS_MSRTC_LO_BASE             (0x20124000UL)
#define MPFS_MSTIMER_LO_BASE           (0x20125000UL)
#define MPFS_H2FINT_LO_BASE            (0x20126000UL)
#define MPFS_CRYPTO_BASE               (0x20127000UL)
#define MPFS_ENVMCFG_BASE              (0x20200000UL)
#define MPFS_USB_BASE                  (0x20201000UL)
#define MPFS_QSPIXIP_BASE              (0x21000000UL)
#define MPFS_ATHENA_BASE               (0x22000000UL)
#define MPFS_TRACE_BASE                (0x23000000UL)

/* These are high base */

#define MPFS_UART0_HI_BASE             (0x28000000UL)
#define MPFS_WDOG0_HI_BASE             (0x28001000UL)
#define MPFS_UART1_HI_BASE             (0x28100000UL)
#define MPFS_WDOG1_HI_BASE             (0x28101000UL)
#define MPFS_UART2_HI_BASE             (0x28102000UL)
#define MPFS_WDOG2_HI_BASE             (0x28103000UL)
#define MPFS_UART3_HI_BASE             (0x28104000UL)
#define MPFS_WDOG3_HI_BASE             (0x28105000UL)
#define MPFS_UART4_HI_BASE             (0x28106000UL)
#define MPFS_WDOG4_HI_BASE             (0x28107000UL)
#define MPFS_SPI0_HI_BASE              (0x28108000UL)
#define MPFS_SPI1_HI_BASE              (0x28109000UL)
#define MPFS_I2C0_HI_BASE              (0x2810A000UL)
#define MPFS_I2C1_HI_BASE              (0x2810B000UL)
#define MPFS_CAN0_HI_BASE              (0x2810C000UL)
#define MPFS_CAN1_HI_BASE              (0x2810D000UL)
#define MPFS_GEM0_HI_BASE              (0x28110000UL)
#define MPFS_GEM1_HI_BASE              (0x28112000UL)
#define MPFS_GPIO0_HI_BASE             (0x28120000UL)
#define MPFS_GPIO1_HI_BASE             (0x28121000UL)
#define MPFS_GPIO2_HI_BASE             (0x28122000UL)
#define MPFS_MSRTC_HI_BASE             (0x28124000UL)
#define MPFS_MSTIMER_HI_BASE           (0x28125000UL)
#define MPFS_H2FINT_HI_BASE            (0x28126000UL)

#define MPFS_IOSCB_SGMII_LANE01_BASE   (0x36500000UL)
#define MPFS_IOSCB_SGMII_LANE23_BASE   (0x36510000UL)
#define MPFS_IOSCBCFG_BASE             (0x37080000UL)
#define MPFS_IOSCB_MSS_PLL_BASE        (0x3e001000UL)
#define MPFS_IOSCB_MSS_MUX_BASE        (0x3e002000UL)
#define MPFS_IOSCB_DDR_PLL_BASE        (0x3e010000UL)
#define MOFS_IOSCB_BANK_DDR_BASE       (0x3E020000UL)
#define MPFS_IOSCB_IO_CALIB_DDR_BASE   (0x3E040000UL)
#define MPFS_IOSCB_SGMII_PLL_BASE      (0x3e080000UL)
#define MPFS_IOSCB_DLL_SGMII_BASE      (0x3e100000UL)
#define MPFS_IOSCB_SGMII_MUX_BASE      (0x3e200000UL)
#define MPFS_IOSCB_BANK_SGMII_BASE     (0x3e400000UL)
#define MPFS_IOSCB_IO_CALIB_SGMII_BASE (0x3e800000UL)

/* TODO: How to select if peripheral is on HI base address kconfig?
 *
 * On PolarFire SoC an AXI switch forms a bus matrix interconnect among
 * multiple masters and multiple slaves. Five RISC-V CPUs connect to the
 * Master ports M10 to M14 of the AXI switch. By default, all the APB
 * peripherals are accessible on AXI-Slave 5 of the AXI switch via the AXI to
 * AHB and AHB to APB bridges (referred as main APB bus).
 * However, to support logical separation in the Asymmetric Multi-Processing
 * (AMP) mode of operation, the APB peripherals can alternatively be accessed
 *  on the AXI-Slave 6 via the AXI to AHB and AHB to APB bridges
 *  (referred as the AMP APB bus).
 */

#define MPFS_UART0_BASE MPFS_UART0_LO_BASE
#define MPFS_UART1_BASE MPFS_UART1_LO_BASE
#define MPFS_UART2_BASE MPFS_UART2_LO_BASE
#define MPFS_UART3_BASE MPFS_UART3_LO_BASE
#define MPFS_UART4_BASE MPFS_UART4_LO_BASE

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_MEMORYMAP_H */
