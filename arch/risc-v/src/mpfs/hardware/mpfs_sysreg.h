/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_sysreg.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_SYSREG_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_SYSREG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define MPFS_SYSREG_TEMP0_OFFSET                       0x0000 /* Register for software use */
#define MPFS_SYSREG_TEMP1_OFFSET                       0x0004 /* Register for software use */
#define MPFS_SYSREG_CLOCK_CONFIG_CR_OFFSET             0x0008 /* Master clock configuration */
#define MPFS_SYSREG_RTC_CLOCK_CR_OFFSET                0x000C /* RTC clock divider */
#define MPFS_SYSREG_FABRIC_RESET_CR_OFFSET             0x0010 /* Fabric Reset mask */
#define MPFS_SYSREG_BOOT_FAIL_CR_OFFSET                0x0014 /* */
#define MPFS_SYSREG_MSS_RESET_CR_OFFSET                0x0018 /* write 0xDEAD to reset */
#define MPFS_SYSREG_CONFIG_LOCK_CR_OFFSET              0x001C /* Configuration lock */
#define MPFS_SYSREG_RESET_SR_OFFSET                    0x0020 /* reset reason */
#define MPFS_SYSREG_DEVICE_STATUS_OFFSET               0x0024 /* device status */
#define MPFS_SYSREG_MSS_BUILD_OFFSET                   0x0028 /* MSS Build Info */
#define MPFS_SYSREG_FAB_INTEN_U54_1_OFFSET             0x0040 /* U54-1 Fabric interrupt enable */
#define MPFS_SYSREG_FAB_INTEN_U54_2_OFFSET             0x0044 /* U54-2 Fabric interrupt enable */
#define MPFS_SYSREG_FAB_INTEN_U54_3_OFFSET             0x0048 /* U54-3 Fabric interrupt enable */
#define MPFS_SYSREG_FAB_INTEN_U54_4_OFFSET             0x004C /* U54-4 Fabric interrupt enable */
#define MPFS_SYSREG_FAB_INTEN_MISC_OFFSET              0x0050 /* Allow Eth direct irq routing to U54 CPUs */
#define MPFS_SYSREG_GPIO_INTERRUPT_FAB_CR_OFFSET       0x0054 /* Switches GPIO interrupt from PAD to Fabric GPIO */
#define MPFS_SYSREG_APBBUS_CR_OFFSET                   0x0080 /* AMP Mode peripheral mapping */
#define MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET             0x0084 /* Enables the clock to the MSS peripheral */
#define MPFS_SYSREG_SOFT_RESET_CR_OFFSET               0x0088 /* Holds the MSS peripherals in reset */
#define MPFS_SYSREG_AHBAXI_CR_OFFSET                   0x008C /* AXI-AHB bridges transfers */
#define MPFS_SYSREG_AHBAPB_CR_OFFSET                   0x0090 /* Configures the two AHB-APB bridges on S5 and S6 */
#define MPFS_SYSREG_DFIAPB_CR_OFFSET                   0x0098 /* MSS Corner APB interface controls */
#define MPFS_SYSREG_GPIO_CR_OFFSET                     0x009C /* GPIO Blocks reset control */
#define MPFS_SYSREG_MAC0_CR_OFFSET                     0x00A4 /* MAC0 configuration */
#define MPFS_SYSREG_MAC1_CR_OFFSET                     0x00A8 /* MAC1 configuration */
#define MPFS_SYSREG_USB_CR_OFFSET                      0x00AC /* USB Configuration */
#define MPFS_SYSREG_MESH_CR_OFFSET                     0x00B0 /* Crypto Mesh control and status */
#define MPFS_SYSREG_MESH_SEED_CR_OFFSET                0x00B4 /* Crypto mesh seed and update rate */
#define MPFS_SYSREG_ENVM_CR_OFFSET                     0x00B8 /* ENVM AHB Controller setup */
#define MPFS_SYSREG_RESERVED_BC_OFFSET                 0x00BC /* Reserved */
#define MPFS_SYSREG_QOS_PERIPHERAL_CR_OFFSET           0x00C0 /* QOS Athena USB & MMC Configuration */
#define MPFS_SYSREG_QOS_CPLEXIO_CR_OFFSET              0x00C4 /* QOS Configuration Coreplex */
#define MPFS_SYSREG_QOS_CPLEXDDR_CR_OFFSET             0x00C8 /* QOS configuration DDRC */
#define MPFS_SYSREG_MPU_VIOLATION_SR_OFFSET            0x00F0 /* Indicates that a master caused a MPU violation */
#define MPFS_SYSREG_MPU_VIOLATION_INTEN_CR_OFFSET      0x00F4 /* Enables interrupts on MPU violations */
#define MPFS_SYSREG_SW_FAIL_ADDR0_CR_OFFSET            0x00F8 /* AXI switch decode fail */
#define MPFS_SYSREG_SW_FAIL_ADDR1_CR_OFFSET            0x00FC /* AXI switch decode fail */
#define MPFS_SYSREG_EDAC_SR_OFFSET                     0x0100 /* Set when an ECC event happens */
#define MPFS_SYSREG_EDAC_INTEN_CR_OFFSET               0x0104 /* Enables ECC interrupt on event */
#define MPFS_SYSREG_EDAC_CNT_MMC_OFFSET                0x0108 /* Count off single bit errors */
#define MPFS_SYSREG_EDAC_CNT_DDRC_OFFSET               0x010C /* Count off single bit errors */
#define MPFS_SYSREG_EDAC_CNT_MAC0_OFFSET               0x0110 /* Count off single bit errors */
#define MPFS_SYSREG_EDAC_CNT_MAC1_OFFSET               0x0114 /* Count off single bit errors */
#define MPFS_SYSREG_EDAC_CNT_USB_OFFSET                0x0118 /* Count off single bit errors */
#define MPFS_SYSREG_EDAC_CNT_CAN0_OFFSET               0x011C /* Count off single bit errors */
#define MPFS_SYSREG_EDAC_CNT_CAN1_OFFSET               0x0120 /* Count off single bit errors */
#define MPFS_SYSREG_EDAC_INJECT_CR_OFFSET              0x0124 /* Will Corrupt write data to rams */
#define MPFS_SYSREG_MAINTENANCE_INTEN_CR_OFFSET        0x0140 /* Maintenance Interrupt Enable */
#define MPFS_SYSREG_PLL_STATUS_INTEN_CR_OFFSET         0x0144 /* PLL Status interrupt enables */
#define MPFS_SYSREG_MAINTENANCE_INT_SR_OFFSET          0x0148 /* Maintenance interrupt fault and status events*/
#define MPFS_SYSREG_PLL_STATUS_SR_OFFSET               0x014C /* PLL interrupt register */
#define MPFS_SYSREG_CFM_TIMER_CR_OFFSET                0x0150 /* Enable to CFM Timer */
#define MPFS_SYSREG_MISC_SR_OFFSET                     0x0154 /* Miscellaneous Register */
#define MPFS_SYSREG_DLL_STATUS_CR_OFFSET               0x0158 /* DLL Interrupt enables */
#define MPFS_SYSREG_DLL_STATUS_SR_OFFSET               0x015C /* DLL interrupt status */
#define MPFS_SYSREG_RAM_LIGHTSLEEP_CR_OFFSET           0x0168 /* Puts all the RAMS in that block into low leakage mode */
#define MPFS_SYSREG_RAM_DEEPSLEEP_CR_OFFSET            0x016C /* Puts all the RAMS in that block into deep sleep mode */
#define MPFS_SYSREG_RAM_SHUTDOWN_CR_OFFSET             0x0170 /* Puts all the RAMS in that block into shut down mode */
#define MPFS_SYSREG_L2_SHUTDOWN_CR_OFFSET              0x0174 /* Allows each bank of the L2 Cache to be powered down */
#define MPFS_SYSREG_IOMUX0_CR_OFFSET                   0x0200 /* Is peripheral connected to the Fabric or IOMUX structure */
#define MPFS_SYSREG_IOMUX1_CR_OFFSET                   0x0204 /* Configures the IO Mux structure for each IO pad */
#define MPFS_SYSREG_IOMUX2_CR_OFFSET                   0x0208 /* Configures the IO Mux structure for each IO pad */
#define MPFS_SYSREG_IOMUX3_CR_OFFSET                   0x020C /* Configures the IO Mux structure for each IO pad */
#define MPFS_SYSREG_IOMUX4_CR_OFFSET                   0x0210 /* Configures the IO Mux structure for each IO pad */
#define MPFS_SYSREG_IOMUX5_CR_OFFSET                   0x0214 /* Configures the IO Mux structure for each IO pad */
#define MPFS_SYSREG_IOMUX6_CR_OFFSET                   0x0218 /* Inverter MMC/SD Voltage select lines to the IOMUX structure */
#define MPFS_SYSREG_MSSIO_BANK4_CFG_CR                 0x0230 /* Configures the MSSIO block */
#define MPFS_SYSREG_MSSIO_BANK4_IO_CFG_0_1_CR_OFFSET   0x0234 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK4_IO_CFG_2_3_CR_OFFSET   0x0238 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK4_IO_CFG_4_5_CR_OFFSET   0x023C /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK4_IO_CFG_6_7_CR_OFFSET   0x0240 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK4_IO_CFG_8_9_CR_OFFSET   0x0244 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK4_IO_CFG_10_11_CR_OFFSET 0x0248 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK4_IO_CFG_12_13_CR_OFFSET 0x024C /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_CFG_CR                 0x0250 /* Configures the MSSIO block */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_0_1_CR_OFFSET   0x0254 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_2_3_CR_OFFSET   0x0258 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_4_5_CR_OFFSET   0x025C /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_6_7_CR_OFFSET   0x0260 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_8_9_CR_OFFSET   0x0264 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_10_11_CR_OFFSET 0x0268 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_12_13_CR_OFFSET 0x026C /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_14_15_CR_OFFSET 0x0270 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_16_17_CR_OFFSET 0x0274 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_18_19_CR_OFFSET 0x0278 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_20_21_CR_OFFSET 0x027C /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSSIO_BANK2_IO_CFG_22_23_CR_OFFSET 0x0280 /* IO electrical configuration for MSSIO pad */
#define MPFS_SYSREG_MSS_SPARE0_CR_OFFSET               0x02A8 /* Sets H2F [31:0] Spares out signals */
#define MPFS_SYSREG_MSS_SPARE1_CR_OFFSET               0x02AC /* Sets H2F [37:32] Spares out signals */
#define MPFS_SYSREG_MSS_SPARE0_SR_OFFSET               0x02B0 /* Read H2F [31:0] Spares out signals */
#define MPFS_SYSREG_MSS_SPARE1_SR_OFFSET               0x02B4 /* Read H2F [37:32] Spares out signals */
#define MPFS_SYSREG_MSS_SPARE2_SR_OFFSET               0x02B8 /* Read F2H [31:0] Spares in1 signals */
#define MPFS_SYSREG_MSS_SPARE3_SR_OFFSET               0x02BC /* Read F2H [37:32] Spares in1 signals */
#define MPFS_SYSREG_MSS_SPARE4_SR_OFFSET               0x02C0 /* Read F2H [31:0] Spares in2 signals */
#define MPFS_SYSREG_MSS_SPARE5_SR_OFFSET               0x02C4 /* Read F2H [37:32] Spares in2 signals */
#define MPFS_SYSREG_SPARE_REGISTER_RW_OFFSET           0x02D0 /* Register for ECO usage */
#define MPFS_SYSREG_SPARE_REGISTER_W1P_OFFSET          0x02D4 /* Register for ECO usage */
#define MPFS_SYSREG_SPARE_REGISTER_RO_OFFSET           0x02D8 /* Register for ECO usage */
#define MPFS_SYSREG_SPARE_PERIM_RW_OFFSET              0x02DC /* Spare signal back to G5C */
#define MPFS_SYSREG_SPARE_FIC_OFFSET                   0x02E0 /* Unused FIC resets */

/* Register bit field definitions *******************************************/

/* Master clock configuration */

#define SYSREG_CLOCK_CONFIG_CR_DIVIDER_SHIFT     (0) /* Bits: 0-5: master synchronous clock divider */
#define SYSREG_CLOCK_CONFIG_CR_DIVIDER_MASK      (0x3f << SYSREG_CLOCK_CONFIG_CR_DIVIDER_SHIFT)
#define SYSREG_CLOCK_CONFIG_CR_ENABLE_1MHZ_SHIFT (8) /* Bit: 8: enable the 1mHz (2MHz) on-chip oscillator */
#define SYSREG_CLOCK_CONFIG_CR_ENABLE_1MHZ_MASK  (1 << SYSREG_CLOCK_CONFIG_CR_ENABLE_1MHZ_SHIFT)
#define SYSREG_CLOCK_CONFIG_CR_ENABLE_1MHZ       (1 << SYSREG_CLOCK_CONFIG_CR_ENABLE_1MHZ_SHIFT)

/* RTC clock divider */

#define SYSREG_RTC_CLOCK_CR_PERIOD_SHIFT         (0) /* Bits: 0-11: period */
#define SYSREG_RTC_CLOCK_CR_PERIOD_MASK          (0xfff << SYSREG_CLOCK_CONFIG_CR_DIVIDER_SHIFT)
#define SYSREG_RTC_CLOCK_CR_ENABLE_SHIFT         (16) /* Bit: 16: RTC Clock enable */
#define SYSREG_RTC_CLOCK_CR_ENABLE_MASK          (1 << SYSREG_CLOCK_CONFIG_CR_DIVIDER_SHIFT)
#define SYSREG_RTC_CLOCK_CR_ENABLE               (1 << SYSREG_CLOCK_CONFIG_CR_DIVIDER_SHIFT)

/* TODO:
 * FABRIC_RESET_CR
 * BOOT_FAIL_CR
 * MSS_RESET_CR
 * CONFIG_LOCK_CR
 * RESET_SR
 * DEVICE_STATUS
 * MSS_BUILD
 * FAB_INTEN_U54_1
 * FAB_INTEN_U54_2
 * FAB_INTEN_U54_3
 * FAB_INTEN_U54_4
 * FAB_INTEN_MISC
 * GPIO_INTERRUPT_FAB_CR
 * APBBUS_CR
 */

/* Enables the clock to the MSS peripheral */

#define SYSREG_SUBBLK_CLOCK_CR_SHIFT   (0) /* Bit: 0-29: Clock enable (17 is reserved) */
#define SYSREG_SUBBLK_CLOCK_CR_MASK    (0x3ffdffff << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_ENVM    (0 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_MAC0    (1 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_MAC1    (2 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_MMC     (3 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_TIMER   (4 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_MMUART0 (5 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_MMUART1 (6 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_MMUART2 (7 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_MMUART3 (8 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_MMUART4 (9 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_SPI0    (10 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_SPI1    (11 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_I2C0    (12 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_I2C1    (13 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_CAN0    (14 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_CAN1    (15 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_USB     (16 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_RTC     (18 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_QSPI    (19 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_GPIO0   (20 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_GPIO1   (21 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_GPIO2   (22 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_DDRC    (23 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_FIC0    (24 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_FIC1    (25 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_FIC2    (26 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_FIC3    (27 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_ATHENA  (28 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SUBBLK_CLOCK_CR_CFM     (29 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)

/* Holds the MSS peripherals in reset */

#define SYSREG_SOFT_RESET_CR_SHIFT     (0) /* Bit: 0-30: Holds the MSS peripherals in reset */
#define SYSREG_SOFT_RESET_CR_MASK      (0x7fffffff << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_ENVM      (0 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_MAC0      (1 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_MAC1      (2 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_MMC       (3 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_TIMER     (4 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_MMUART0   (5 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_MMUART1   (6 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_MMUART2   (7 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_MMUART3   (8 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_MMUART4   (9 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_SPI0      (10 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_SPI1      (11 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_I2C0      (12 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_I2C1      (13 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_CAN0      (14 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_CAN1      (15 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_USB       (16 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_FPGA      (17 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_RTC       (18 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_QSPI      (19 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_GPIO0     (20 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_GPIO1     (21 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_GPIO2     (22 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_DDRC      (23 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_FIC0      (24 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_FIC1      (25 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_FIC2      (26 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_FIC3      (27 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_ATHENA    (28 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_CFM       (29 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)
#define SYSREG_SOFT_RESET_CR_SGMII     (30 << SYSREG_SUBBLK_CLOCK_CR_SHIFT)

/* TODO:
 * AHBAXI_CR
 * AHBAPB_CR
 * DFIAPB_CR
 * GPIO_CR
 * MAC0_CR
 * MAC1_CR
 * USB_CR
 * MESH_CR
 * MESH_SEED_CR
 * ENVM_CR
 * QOS_PERIPHERAL_CR
 * QOS_CPLEXIO_CR
 * QOS_CPLEXDDR_CR
 * MPU_VIOLATION_SR
 * MPU_VIOLATION_INTEN_CR
 * SW_FAIL_ADDR0_CR
 * SW_FAIL_ADDR1_CR
 * EDAC_SR
 * EDAC_INTEN_CR
 * EDAC_CNT_MMC
 * EDAC_CNT_DDRC
 * EDAC_CNT_MAC0
 * EDAC_CNT_MAC1
 * EDAC_CNT_USB
 * EDAC_CNT_CAN0
 * EDAC_CNT_CAN1
 * EDAC_INJECT_CR
 * MAINTENANCE_INTEN_CR
 * PLL_STATUS_INTEN_CR
 * MAINTENANCE_INT_SR
 * PLL_STATUS_SR
 * CFM_TIMER_CR
 * MISC_SR
 * PLL_STATUS_CR
 * DLL_STATUS_SR
 * RAM_LIGHTSLEEP_CR
 * RAM_DEEPSLEEP_CR
 * RAM_SHUTDOWN_CR
 * L2_SHUTDOWN_CR
 * IOMUX0_CR
 * IOMUX1_CR
 * IOMUX2_CR
 * IOMUX3_CR
 * IOMUX4_CR
 * IOMUX5_CR
 * IOMUX6_CR
 * MSSIO_BANK4_IO_CFG_0_1_CR
 * MSSIO_BANK4_IO_CFG_2_3_CR
 * MSSIO_BANK4_IO_CFG_4_5_CR
 * MSSIO_BANK4_IO_CFG_6_7_CR
 * MSSIO_BANK4_IO_CFG_8_9_CR
 * MSSIO_BANK4_IO_CFG_10_11_CR
 * MSSIO_BANK4_IO_CFG_12_13_CR
 * MSSIO_BANK2_IO_CFG_0_1_CR
 * MSSIO_BANK2_IO_CFG_2_3_CR
 * MSSIO_BANK2_IO_CFG_4_5_CR
 * MSSIO_BANK2_IO_CFG_6_7_CR
 * MSSIO_BANK2_IO_CFG_8_9_CR
 * MSSIO_BANK2_IO_CFG_10_11_CR
 * MSSIO_BANK2_IO_CFG_12_13_CR
 * MSSIO_BANK2_IO_CFG_14_15_CR
 * MSSIO_BANK2_IO_CFG_16_17_CR
 * MSSIO_BANK2_IO_CFG_18_19_CR
 * MSSIO_BANK2_IO_CFG_20_21_CR
 * MSSIO_BANK2_IO_CFG_22_23_CR
 * MSS_SPARE0_CR
 * MSS_SPARE1_CR
 * MSS_SPARE0_SR
 * MSS_SPARE1_SR
 * MSS_SPARE2_SR
 * MSS_SPARE3_SR
 * MSS_SPARE4_SR
 * MSS_SPARE5_SR
 * SPARE_PERIM_RW
 */

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_SYSREG_H */
