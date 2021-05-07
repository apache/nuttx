/****************************************************************************
 * arch/arm/src/am335x/hardware/am3358_memorymap.h
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

#ifndef __ARCH_ARM_SRC_AM335X_HARDWARE_AM3358_MEMORYMAP_H
#define __ARCH_ARM_SRC_AM335X_HARDWARE_AM3358_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/am335x/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Decimal configuration values may exceed 2Gb and, hence, overflow to
 * negative values unless we force them to unsigned long:
 */

#define __CONCAT(a,b) a ## b
#define MKULONG(a) __CONCAT(a,ul)

/* AM335X physical section base addresses (aligned to 1MB boundaries) */

#define AM335X_GPMC_PSECTION            0x00000000 /* External Memory   : 0x0000_0000-0x1FFF_FFFF : 512MB 8-/16-bit External Memory (Ex/R/W) */
#define AM335X_BROM_PSECTION            0x40000000 /* Boot ROM          : 0x4000_0000-0x4002_BFFF : 128KB + 48KB 32-bit Ex/R – Public */
#define AM335X_ISRAM_PSECTION           0x40200000 /* SRAM internal     : 0x4020_0000-0x402F_FFFF : 961KB Reserved + 63KB 32-bit Ex/R/W  */
#define AM335X_OCMC0_PSECTION           0x40300000 /* L3 OCMC0          : 0x4030_0000-0x4030_FFFF : 64KB 32-bit Ex/R/W OCMC SRAM */
#define AM335X_PERIPH_PSECTION          0x44000000 /* Peripherals       : 0x4400_0000-0x7FFF_FFFF : The region between OCMC and DDR */
#define AM335X_DDR_PSECTION             0x80000000 /* EMIF0 SDRAM       : 0x8000_0000-0xBFFF_FFFF : 1GB 8-/16-bit External Memory (Ex/R/W) */

/* AM335X offsets from the BRROM section base address */

#define AM335X_BROM_OFFSET              0x00020000 /* 0x4002_0000-0x4002_BFFF 48KB ??? TODO: clear out is offset 0x20000 or 0 */

/* AM335X offsets from the internal memory section base address */

#define AM335X_ISRAM_OFFSET             0x000F0400 /* 0x402F_0400-0x402F_FFFF 63KB Internal SRAM 32-bit Ex/R/W */

/* AM335X offsets from the L3 OCMC0 memory section base address */

#define AM335X_OCMC0_OFFSET             0x00000000 /* 0x4030_0000-0x4030_FFFF 64KB 32-bit Ex/R/W OCMC SRAM */

/* AM335X offsets from the peripheral section base address */

#define AM335X_L3_FAST_CFG_OFFSET       0x00000000 /* 0x4400_0000-0x443F_FFFF 4MB L3Fast configuration registers */
#define AM335X_L3_SLOW_CFG_OFFSET       0x00800000 /* 0x4480_0000-0x44BF_FFFF 4MB L3Slow configuration registers */
#define AM335X_L4_WKUP_CFG_OFFSET       0x00C00000 /* 0x4400_0000-0x44C0_1FFF 8KB L4_WKUP configuration */
#define AM335X_CM_PER_OFFSET            0x00E00000 /* 0x44E0_0000-0x44E0_3FFF 1KB Clock Module Peripheral Registers */
#define AM335X_CM_WKUP_OFFSET           0x00E00400 /* 0x44E0_0400-0x44E0_04FF 256 Bytes Clock Module Wakeup Registers */
#define AM335X_CM_DPLL_OFFSET           0x00E00500 /* 0x44E0_0500-0x44E0_05FF 256 Bytes Clock Module PLL Registers */
#define AM335X_CM_MPU_OFFSET            0x00E00600 /* 0x44E0_0600-0x44E0_06FF 256 Bytes Clock Module MPU Registers */
#define AM335X_CM_DEVICE_OFFSET         0x00E00700 /* 0x44E0_0700-0x44E0_07FF 256 Bytes Clock Module Device Registers */
#define AM335X_CM_RTC_OFFSET            0x00E00800 /* 0x44E0_0800-0x44E0_08FF 256 Bytes Clock Module RTC Registers */
#define AM335X_CM_GFX_OFFSET            0x00E00900 /* 0x44E0_0900-0x44E0_09FF 256 Bytes Clock Module Graphics Controller Registers */
#define AM335X_CM_CEFUSE_OFFSET         0x00E00A00 /* 0x44E0_0A00-0x44E0_0AFF 256 Bytes Clock Module Efuse Registers */
#define AM335X_PRM_IRQ_OFFSET           0x00E00B00 /* 0x44E0_0B00-0x44E0_0BFF 256 Bytes Power Reset Module Interrupt Registers */
#define AM335X_PRM_PER_OFFSET           0x00E00C00 /* 0x44E0_0C00-0x44E0_0CFF 256 Bytes Power Reset Module Peripheral Registers */
#define AM335X_PRM_WKUP_OFFSET          0x00E00D00 /* 0x44E0_0D00-0x44E0_0DFF 256 Bytes Power Reset Module Wakeup Registers */
#define AM335X_PRM_MPU_OFFSET           0x00E00E00 /* 0x44E0_0E00-0x44E0_0EFF 256 Bytes Power Reset Module MPU Registers */
#define AM335X_PRM_DEV_OFFSET           0x00E00F00 /* 0x44E0_0F00-0x44E0_0FFF 256 Bytes Power Reset Module Device Registers */
#define AM335X_PRM_RTC_OFFSET           0x00E01000 /* 0x44E0_1000-0x44E0_10FF 256 Bytes Power Reset Module RTC Registers */
#define AM335X_PRM_GFX_OFFSET           0x00E01100 /* 0x44E0_1100-0x44E0_11FF 256 Bytes Power Reset Module Graphics Controller Registers */
#define AM335X_PRM_CEFUSE_OFFSET        0x00E01200 /* 0x44E0_1200-0x44E0_12FF 256 Bytes Power Reset Module Efuse Registers */
#define AM335X_DMTIMER0_OFFSET          0x00E05000 /* 0x44E0_5000-0x44E0_5FFF 4KB DMTimer0 Registers */
#define AM335X_GPIO0_OFFSET             0x00E07000 /* 0x44E0_7000-0x44E0_7FFF 4KB GPIO Registers */
#define AM335X_UART0_OFFSET             0x00E09000 /* 0x44E0_9000-0x44E0_9FFF 4KB UART Registers */
#define AM335X_I2C0_OFFSET              0x00E0B000 /* 0x44E0_B000-0x44E0_BFFF 4KB I2C Registers */
#define AM335X_ADC_TSC_OFFSET           0x00E0D000 /* 0x44E0_D000-0x44E0_EFFF 8KB ADC_TSC Registers */
#define AM335X_CONTROL_MODULE_OFFSET    0x00E10000 /* 0x44E1_0000-0x44E1_1FFF 8KB Control Module Registers */
#define AM335X_DDR_PHY_OFFSET           0x00E12000 /* 0x44E1_2000-0x44E1_23FF 1KB DDR2/3/mDDR PHY Registers */
#define AM335X_DMTIMER1_1MS_OFFSET      0x00E31000 /* 0x44E3_1000-0x44E3_1FFF 4KB DMTimer1 1ms Registers (Accurate 1ms timer) */
#define AM335X_WDT1_OFFSET              0x00E35000 /* 0x44E3_5000-0x44E3_5FFF 4KB Watchdog Timer Registers */
#define AM335X_SMART_REFLEX0_OFFSET     0x00E37000 /* 0x44E3_7000-0x44E3_7FFF 4KB L3 Registers */
#define AM335X_SMART_REFLEX1_OFFSET     0x00E39000 /* 0x44E3_9000-0x44E3_9FFF 4KB L3 Registers */
#define AM335X_RTCSS_OFFSET             0x00E3E000 /* 0x44E3_E000-0x44E3_EFFF 4KB RTC Registers */
#define AM335X_DEBUG_SS_OFFSET          0x00E40000 /* 0x44E4_0000-0x44E7_FFFF 256KB Debug Register */

#define AM335X_MCASP0_DATA_OFFSET       0x02000000 /* 0x4600_0000-0x463F_FFFF 4MB McASP0 Data Registers */
#define AM335X_MCASP1_DATA_OFFSET       0x02400000 /* 0x4640_0000-0x467F_FFFF 4MB McASP1 Data Registers */

#define AM335X_USBSS_OFFSET             0x03400000 /* 0x4740_0000-0x4740_0FFF USB Subsystem Registers */
#define AM335X_USB0_OFFSET              0x03401000 /* 0x4740_1000-0x4740_12FF USB0 Controller Registers */
#define AM335X_USB0_PHY_OFFSET          0x03401300 /* 0x4740_1300-0x4740_13FF USB0 PHY Registers */
#define AM335X_USB0_CORE_OFFSET         0x03401400 /* 0x4740_1400-0x4740_17FF USB0 Core Registers */
#define AM335X_USB1_OFFSET              0x03401800 /* 0x4740_1800-0x4740_1AFF USB1 Controller Registers */
#define AM335X_USB1_PHY_OFFSET          0x03401B00 /* 0x4740_1B00-0x4740_1BFF USB1 PHY Registers */
#define AM335X_USB1_CORE_OFFSET         0x03401C00 /* 0x4740_1C00-0x4740_1FFF USB1 Core Registers */
#define AM335X_USB_DMA_OFFSET           0x03402000 /* 0x4740_2000-0x4740_2FFF USB CPPI DMA Controller Registers */
#define AM335X_USB_DMA_SCHED_OFFSET     0x03403000 /* 0x4740_3000-0x4740_3FFF USB CPPI DMA Scheduler Registers */
#define AM335X_USB_QUEUE_MGR_OFFSET     0x03404000 /* 0x4740_4000-0x4740_7FFF USB Queue Manager Registers */

#define AM335X_MMCHS2_OFFSET            0x03810000 /* 0x4781_0000-0x4781_FFFF 64KB MMCHS2 */

#define AM335X_UART1_OFFSET             0x04022000 /* 0x4802_2000-0x4802_2FFF 4KB UART1 Registers */
#define AM335X_UART2_OFFSET             0x04024000 /* 0x4802_4000-0x4802_4FFF 4KB UART2 Registers */
#define AM335X_I2C1_OFFSET              0x0402A000 /* 0x4802_A000-0x4802_AFFF 4KB I2C1 Registers */
#define AM335X_MCSPI0_OFFSET            0x04030000 /* 0x4803_0000-0x4803_0FFF 4KB McSPI0 Registers */
#define AM335X_MCASP0_OFFSET            0x04038000 /* 0x4803_8000-0x4803_9FFF 8KB McASP0 CFG Registers */
#define AM335X_MCASP1_OFFSET            0x0403C000 /* 0x4803_C000-0x4803_DFFF 8KB McASP1 CFG Registers */
#define AM335X_DMTIMER2_OFFSET          0x04040000 /* 0x4804_0000-0x4804_0FFF 4KB DMTimer2 Registers */
#define AM335X_DMTIMER3_OFFSET          0x04042000 /* 0x4804_2000-0x4804_2FFF 4KB DMTimer3 Registers */
#define AM335X_DMTIMER4_OFFSET          0x04044000 /* 0x4804_4000-0x4804_4FFF 4KB DMTimer4 Registers */
#define AM335X_DMTIMER5_OFFSET          0x04046000 /* 0x4804_6000-0x4804_6FFF 4KB DMTimer5 Registers */
#define AM335X_DMTIMER6_OFFSET          0x04048000 /* 0x4804_8000-0x4804_8FFF 4KB DMTimer6 Registers */
#define AM335X_DMTIMER7_OFFSET          0x0404A000 /* 0x4804_A000-0x4804_AFFF 4KB DMTimer7 Registers */
#define AM335X_GPIO1_OFFSET             0x0404C000 /* 0x4804_C000-0x4804_CFFF 4KB GPIO1 Registers */
#define AM335X_MMCHS0_OFFSET            0x04060000 /* 0x4806_0000-0x4806_0FFF 4KB MMCHS0 Registers */
#define AM335X_ELM_OFFSET               0x04080000 /* 0x4808_0000-0x4808_FFFF 64KB ELM Registers */
#define AM335X_MAIBOX_OFFSET            0x040C8000 /* 0x480C_8000-0x480C_8FFF 4KB Mailbox Registers */
#define AM335X_SPINLOCK_OFFSET          0x040CA000 /* 0x480C_A000-0x480C_AFFF 4KB Spinlock Registers */

#define AM335X_OCP_WATCHPOINT_OFFSET    0x0418C000 /* 0x4818_C000-0x4818_CFFF 4KB OCP Watchpoint Registers */
#define AM335X_I2C2_OFFSET              0x0419C000 /* 0x4819_C000-0x4819_CFFF 4KB I2C2 Registers */
#define AM335X_MCSPI1_OFFSET            0x041A0000 /* 0x481A_0000-0x481A_0FFF 4KB McSPI1 Registers */
#define AM335X_UART3_OFFSET             0x041A6000 /* 0x481A_6000-0x481A_6FFF 4KB UART3 Registers */
#define AM335X_UART4_OFFSET             0x041A8000 /* 0x481A_8000-0x481A_8FFF 4KB UART4 Registers */
#define AM335X_UART5_OFFSET             0x041AA000 /* 0x481A_A000-0x481A_AFFF 4KB UART5 Registers */
#define AM335X_GPIO2_OFFSET             0x041AC000 /* 0x481A_C000-0x481A_CFFF 4KB GPIO2 Registers */
#define AM335X_GPIO3_OFFSET             0x041AE000 /* 0x481A_E000-0x481A_EFFF 4KB GPIO3 Registers */
#define AM335X_DCAN0_OFFSET             0x041CC000 /* 0x481C_C000-0x481C_DFFF 8KB DCAN0 Registers */
#define AM335X_DCAN1_OFFSET             0x041D0000 /* 0x481D_0000-0x481D_1FFF 8KB DCAN1 Registers */
#define AM335X_MMC1_OFFSET              0x041D8000 /* 0x481D_8000-0x481D_8FFF 4KB MMC1 Registers */

#define AM335X_INTC_OFFSET              0x04200000 /* 0x4820_0000-0x4820_0FFF 4KB Interrupt Controller Registers */
#define AM335X_MPUSS_OFFSET             0x04240000 /* 0x4824_0000-0x4824_0FFF 4KB Host ARM non-shared device mapping */

#define AM335X_PWMSS0_OFFSET            0x04300000 /* 0x4830_0000-0x4830_00FF XXKB PWM Subsystem 0 Configuration Registers */
#define AM335X_ECAP0_OFFSET             0x04300100 /* 0x4830_0100-0x4830_017F XXKB PWMSS eCAP0 Registers */
#define AM335X_EQEP0_OFFSET             0x04300180 /* 0x4830_0180-0x4830_01FF XXKB PWMSS eQEP0 Registers */
#define AM335X_EPWM0_OFFSET             0x04300200 /* 0x4830_0200-0x4830_025F XXKB PWMSS ePWM0 Registers */
#define AM335X_PWMSS1_OFFSET            0x04302000 /* 0x4830_2000-0x4830_20FF XXKB PWM Subsystem 1 Configuration Registers */
#define AM335X_ECAP1_OFFSET             0x04302100 /* 0x4830_2100-0x4830_217F XXKB PWMSS eCAP1 Registers */
#define AM335X_EQEP1_OFFSET             0x04302180 /* 0x4830_2180-0x4830_21FF XXKB PWMSS eQEP1 Registers */
#define AM335X_EPWM1_OFFSET             0x04302200 /* 0x4830_2200-0x4830_225F XXKB PWMSS ePWM1 Registers */
#define AM335X_PWMSS2_OFFSET            0x04304000 /* 0x4830_4000-0x4830_40FF XXKB PWM Subsystem 2 Configuration Registers */
#define AM335X_ECAP2_OFFSET             0x04304100 /* 0x4830_4100-0x4830_417F XXKB PWMSS eCAP2 Registers */
#define AM335X_EQEP2_OFFSET             0x04304180 /* 0x4830_4180-0x4830_41FF XXKB PWMSS eQEP2 Registers */
#define AM335X_EPWM2_OFFSET             0x04304200 /* 0x4830_4200-0x4830_425F XXKB PWMSS ePWM2 Registers */
#define AM335X_LCD_OFFSET               0x0430E000 /* 0x4830_E000-0x4830_EFFF 4KB LCD Registers */

#define AM335X_EDMA3CC_OFFSET           0x05000000 /* 0x4900_0000-0x490F_FFFF 1MB EDMA3 Channel Controller Registers */
#define AM335X_EDMA3TC0_OFFSET          0x05800000 /* 0x4980_0000-0x498F_FFFF 1MB EDMA3 Transfer Controller 0 Registers */
#define AM335X_EDMA3TC1_OFFSET          0x05900000 /* 0x4990_0000-0x499F_FFFF 1MB EDMA3 Transfer Controller 1 Registers */
#define AM335X_EDMA3TC2_OFFSET          0x05A00000 /* 0x49A0_0000-0x49AF_FFFF 1MB EDMA3 Transfer Controller 2 Registers */

#define AM335X_L4_FAST_CONFIG_OFFSET    0x06000000 /* 0x4A00_0000-0x4A00_1FFF 8KB L4_FAST configuration */

#define AM335X_CPSW_SS_OFFSET           0x06100000 /* 0x4A10_0000-0x4A10_7FFF 32KB Ethernet Switch Subsystem */
#define AM335X_CPSW_PORT_OFFSET         0x06100100 /* 0x4A10_0100-0x4A10_07FF Ethernet Switch Port Control */
#define AM335X_CPSW_CPDMA_OFFSET        0x06100800 /* 0x4A10_0800-0x4A10_08FF CPPI DMA Controller Module */
#define AM335X_CPSW_STATS_OFFSET        0x06100900 /* 0x4A10_0900-0x4A10_09FF Ethernet Statistics */
#define AM335X_CPSW_STATERAM_OFFSET     0x06100A00 /* 0x4A10_0A00-0x4A10_0BFF CPPI DMA State RAM */
#define AM335X_CPSW_CPTS_OFFSET         0x06100C00 /* 0x4A10_0C00-0x4A10_0CFF Ethernet Time Sync Module */
#define AM335X_CPSW_ALE_OFFSET          0x06100D00 /* 0x4A10_0D00-0x4A10_0D7F Ethernet Address Lookup Engine */
#define AM335X_CPSW_SL1_OFFSET          0x06100D80 /* 0x4A10_0D80-0x4A10_0DBF Ethernet Sliver for Port 1 */
#define AM335X_CPSW_SL2_OFFSET          0x06100DC0 /* 0x4A10_0DC0-0x4A10_0DFF Ethernet Sliver for Port 2 */
#define AM335X_MDIO_OFFSET              0x06101000 /* 0x4A10_1000-0x4A10_10FF Ethernet MDIO Controller */
#define AM335X_CPSW_WR_OFFSET           0x06101200 /* 0x4A10_1200-0x4A10_1FFF Ethernet Subsystem Wrapper for RMII/RGMII */
#define AM335X_CPPI_RAM_OFFSET          0x06102000 /* 0x4A10_2000-0x4A10_3FFF Communications Port Programming Interface RAM */

#define AM335X_PRU_ICSS_OFFSET          0x06300000 /* 0x4A30_0000-0x4A37_FFFF 512KB PRU-ICSS Instruction/Data/Control Space */

#define AM335X_DEBUG_SS_DRM_OFFSET      0x07160000 /* 0x4B16_0000-0x4B16_0FFF 4KB Debug Subsystem: Debug Resource Manager */
#define AM335X_DEBUG_SS_ETB_OFFSET      0x07162000 /* 0x4B16_2000-0x4B16_2FFF 4KB Debug Subsystem: Embedded Trace Buffer */

#define AM335X_EMIF0_OFFSET             0x08000000 /* 0x4C00_0000-0x4CFF_FFFF 16MB EMIF0 Configuration registers */

#define AM335X_GPMC_OFFSET              0x0C000000 /* 0x5000_0000-0x50FF_FFFF 16MB GPMC Configuration registers */

#define AM335X_ADC_TSC_DMA_OFFSET       0x10C00000 /* 0x54C0_0000-0x54FF_FFFF 4MB ADC_TSC DMA Port */

#define AM335X_SGX530_OFFSET            0x12000000 /* 0x5600_0000-0x56FF_FFFF 16MB SGX530 Slave Port */

/* AM335X internal memory physical base addresses */

#define AM335X_ISRAM_PADDR              (AM335X_ISRAM_PSECTION+AM335X_ISRAM_OFFSET)

/* AM335X L3 OCMC0 memory physical base addresses */

#define AM335X_OCMC0_PADDR              (AM335X_OCMC0_PSECTION+AM335X_OCMC0_OFFSET)

/* Sizes of memory regions in bytes.
 *
 * These sizes exclude the undefined addresses at the end of the memory
 * region.  The implemented sizes of the external memory regions are
 * not known apriori and must be specified with configuration settings.
 */

#define AM335X_GPMC_SIZE                0x20000000 /* GPMC (External Memory) 0x0000_0000-0x1FFF_FFFF 512MB */
#define AM335X_BROM_SIZE                0x0002c000 /* BROM                   0x4000_0000-0x4002_BFFF 176KB */
#define AM335X_ISRAM_SIZE               0x00100000 /* SRAM internal          0x4020_0000 0x402F_FFFF 1MB */
#define AM335X_OCMC0_SIZE               0x00010000 /* L3 OCMC0               0x4030_0000 0x4030_FFFF 64KB */
#define AM335X_PERIPH_SIZE              0x3c000000 /* Peripherals            0x4400_0000-0x7FFF_FFFF 960MB */

/* Force configured sizes that might exceed 1GB to be unsigned long */

#define AM335X_DDR_MAPOFFSET            MKULONG(CONFIG_AM335X_DDR_MAPOFFSET)
#define AM335X_DDR_MAPSIZE              MKULONG(CONFIG_AM335X_DDR_MAPSIZE)
#define AM335X_DDR_HEAP_OFFSET          MKULONG(CONFIG_AM335X_DDR_HEAP_OFFSET)
#define AM335X_DDR_HEAP_SIZE            MKULONG(CONFIG_AM335X_DDR_HEAP_SIZE)

/* Convert size in bytes to number of sections (in Mb). */

#define _NSECTIONS(b)                   (((b)+0x000fffff) >> 20)

/* Sizes of memory regions in sections.
 *
 * The boot logic in AM335X_boot.c, will select 1Mb level 1 MMU mappings to
 * span the entire physical address space.  The definitions below specify
 * the number of 1Mb entries that are required to span a particular address
 * region.
 *
 * NOTE: the size of the mapped SDRAM region depends on the configured size
 * of DRAM, not on the size of the address space assigned to DRAM.
 */

#define AM335X_GPMC_NSECTIONS           _NSECTIONS(AM335X_GPMC_SIZE)
#define AM335X_BROM_NSECTIONS           _NSECTIONS(AM335X_BROM_SIZE)
#define AM335X_ISRAM_NSECTIONS          _NSECTIONS(AM335X_ISRAM_SIZE)
#define AM335X_OCMC0_NSECTIONS          _NSECTIONS(AM335X_OCMC0_SIZE)
#define AM335X_PERIPH_NSECTIONS         _NSECTIONS(AM335X_PERIPH_SIZE)
#define AM335X_DDR_NSECTIONS            _NSECTIONS(AM335X_DDR_MAPSIZE)

/* Section MMU Flags */

#define AM335X_GPMC_MMUFLAGS            MMU_MEMFLAGS
#define AM335X_BROM_MMUFLAGS            MMU_ROMFLAGS
#define AM335X_ISRAM_MMUFLAGS           MMU_MEMFLAGS
#define AM335X_OCMC0_MMUFLAGS           MMU_MEMFLAGS
#define AM335X_PERIPH_MMUFLAGS          MMU_IOFLAGS
#define AM335X_DDR_MMUFLAGS             MMU_MEMFLAGS

/* AM335X Virtual (mapped) Memory Map
 *
 * board_memorymap.h contains special mappings that are needed when a ROM
 * memory map is used.  It is included in this odd location because it
 * depends on some the virtual address definitions provided above.
 */

#include <arch/board/board_memorymap.h>

/* AM335X Virtual (mapped) Memory Map.  These are the mappings that will
 * be created if the page table lies in RAM.  If the platform has another,
 * read-only, pre-initialized page table (perhaps in ROM), then the board.h
 * file must provide these definitions.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE

/* The default mappings are a simple 1-to-1 mapping */

#define AM335X_GPMC_VSECTION            0x00000000 /* External Memory   : 0x0000_0000-0x1FFF_FFFF : 512MB 8-/16-bit External Memory (Ex/R/W) */
#define AM335X_BROM_VSECTION            0x40000000 /* Boot ROM          : 0x4000_0000-0x4002_BFFF : 128KB + 48KB 32-bit Ex/R – Public */
#define AM335X_ISRAM_VSECTION           0x40200000 /* SRAM internal     : 0x4020_0000-0x402F_FFFF : 961KB Reserved + 63KB 32-bit Ex/R/W  */
#define AM335X_OCMC0_VSECTION           0x40300000 /* L3 OCMC0          : 0x4030_0000-0x4030_FFFF : 64KB 32-bit Ex/R/W OCMC SRAM */
#define AM335X_PERIPH_VSECTION          0x44000000 /* Peripherals       : 0x4400_0000-0x7FFF_FFFF : The region between OCMC and DDR */
#define AM335X_DDR_VSECTION             0x80000000 /* EMIF0 SDRAM       : 0x8000_0000-0xBFFF_FFFF : 1GB 8-/16-bit External Memory (Ex/R/W) */

#endif

/* AM335X internal memory virtual base addresses */

#define AM335X_ISRAM_VADDR              (AM335X_ISRAM_VSECTION+AM335X_ISRAM_OFFSET)

/* AM335X L3 OCMC0 memory virtual base addresses */

#define AM335X_OCMC0_VADDR              (AM335X_OCMC0_VSECTION+AM335X_OCMC0_OFFSET)

/* Peripheral virtual base addresses */

#define AM335X_L3_FAST_CFG_VADDR        (AM335X_PERIPH_VSECTION+AM335X_L3_FAST_CFG_OFFSET)
#define AM335X_L3_SLOW_CFG_VADDR        (AM335X_PERIPH_VSECTION+AM335X_L3_SLOW_CFG_OFFSET)
#define AM335X_L4_WKUP_CFG_VADDR        (AM335X_PERIPH_VSECTION+AM335X_L4_WKUP_CFG_OFFSET)
#define AM335X_CM_PER_VADDR             (AM335X_PERIPH_VSECTION+AM335X_CM_PER_OFFSET)
#define AM335X_CM_WKUP_VADDR            (AM335X_PERIPH_VSECTION+AM335X_CM_WKUP_OFFSET)
#define AM335X_CM_DPLL_VADDR            (AM335X_PERIPH_VSECTION+AM335X_CM_DPLL_OFFSET)
#define AM335X_CM_MPU_VADDR             (AM335X_PERIPH_VSECTION+AM335X_CM_MPU_OFFSET)
#define AM335X_CM_DEVICE_VADDR          (AM335X_PERIPH_VSECTION+AM335X_CM_DEVICE_OFFSET)
#define AM335X_CM_RTC_VADDR             (AM335X_PERIPH_VSECTION+AM335X_CM_RTC_OFFSET)
#define AM335X_CM_GFX_VADDR             (AM335X_PERIPH_VSECTION+AM335X_CM_GFX_OFFSET)
#define AM335X_CM_CEFUSE_VADDR          (AM335X_PERIPH_VSECTION+AM335X_CM_CEFUSE_OFFSET)
#define AM335X_PRM_IRQ_VADDR            (AM335X_PERIPH_VSECTION+AM335X_PRM_IRQ_OFFSET)
#define AM335X_PRM_PER_VADDR            (AM335X_PERIPH_VSECTION+AM335X_PRM_PER_OFFSET)
#define AM335X_PRM_WKUP_VADDR           (AM335X_PERIPH_VSECTION+AM335X_PRM_WKUP_OFFSET)
#define AM335X_PRM_MPU_VADDR            (AM335X_PERIPH_VSECTION+AM335X_PRM_MPU_OFFSET)
#define AM335X_PRM_DEV_VADDR            (AM335X_PERIPH_VSECTION+AM335X_PRM_DEV_OFFSET)
#define AM335X_PRM_RTC_VADDR            (AM335X_PERIPH_VSECTION+AM335X_PRM_RTC_OFFSET)
#define AM335X_PRM_GFX_VADDR            (AM335X_PERIPH_VSECTION+AM335X_PRM_GFX_OFFSET)
#define AM335X_PRM_CEFUSE_VADDR         (AM335X_PERIPH_VSECTION+AM335X_PRM_CEFUSE_OFFSET)
#define AM335X_DMTIMER0_VADDR           (AM335X_PERIPH_VSECTION+AM335X_DMTIMER0_OFFSET)
#define AM335X_GPIO0_VADDR              (AM335X_PERIPH_VSECTION+AM335X_GPIO0_OFFSET)
#define AM335X_UART0_VADDR              (AM335X_PERIPH_VSECTION+AM335X_UART0_OFFSET)
#define AM335X_I2C0_VADDR               (AM335X_PERIPH_VSECTION+AM335X_I2C0_OFFSET)
#define AM335X_ADC_TSC_VADDR            (AM335X_PERIPH_VSECTION+AM335X_ADC_TSC_OFFSET)
#define AM335X_CONTROL_MODULE_VADDR     (AM335X_PERIPH_VSECTION+AM335X_CONTROL_MODULE_OFFSET)
#define AM335X_DDR_PHY_VADDR            (AM335X_PERIPH_VSECTION+AM335X_DDR_PHY_OFFSET)
#define AM335X_DMTIMER1_1MS_VADDR       (AM335X_PERIPH_VSECTION+AM335X_DMTIMER1_1MS_OFFSET)
#define AM335X_WDT1_VADDR               (AM335X_PERIPH_VSECTION+AM335X_WDT1_OFFSET)
#define AM335X_SMART_REFLEX0_VADDR      (AM335X_PERIPH_VSECTION+AM335X_SMART_REFLEX0_OFFSET)
#define AM335X_SMART_REFLEX1_VADDR      (AM335X_PERIPH_VSECTION+AM335X_SMART_REFLEX1_OFFSET)
#define AM335X_RTCSS_VADDR              (AM335X_PERIPH_VSECTION+AM335X_RTCSS_OFFSET)
#define AM335X_DEBUG_SS_VADDR           (AM335X_PERIPH_VSECTION+AM335X_DEBUG_SS_OFFSET)

#define AM335X_MCASP0_DATA_VADDR        (AM335X_PERIPH_VSECTION+AM335X_MCASP0_DATA_OFFSET)
#define AM335X_MCASP1_DATA_VADDR        (AM335X_PERIPH_VSECTION+AM335X_MCASP1_DATA_OFFSET)

#define AM335X_USBSS_VADDR              (AM335X_PERIPH_VSECTION+AM335X_USBSS_OFFSET)
#define AM335X_USB0_VADDR               (AM335X_PERIPH_VSECTION+AM335X_USB0_OFFSET)
#define AM335X_USB0_PHY_VADDR           (AM335X_PERIPH_VSECTION+AM335X_USB0_PHY_OFFSET)
#define AM335X_USB0_CORE_VADDR          (AM335X_PERIPH_VSECTION+AM335X_USB0_CORE_OFFSET)
#define AM335X_USB1_VADDR               (AM335X_PERIPH_VSECTION+AM335X_USB1_OFFSET)
#define AM335X_USB1_PHY_VADDR           (AM335X_PERIPH_VSECTION+AM335X_USB1_PHY_OFFSET)
#define AM335X_USB1_CORE_VADDR          (AM335X_PERIPH_VSECTION+AM335X_USB1_CORE_OFFSET)
#define AM335X_USB_DMA_VADDR            (AM335X_PERIPH_VSECTION+AM335X_USB_DMA_OFFSET)
#define AM335X_USB_DMA_SCHED_VADDR      (AM335X_PERIPH_VSECTION+AM335X_USB_DMA_SCHED_OFFSET)
#define AM335X_USB_QUEUE_MGR_VADDR      (AM335X_PERIPH_VSECTION+AM335X_USB_QUEUE_MGR_OFFSET)

#define AM335X_MMCHS2_VADDR             (AM335X_PERIPH_VSECTION+AM335X_MMCHS2_OFFSET)

#define AM335X_UART1_VADDR              (AM335X_PERIPH_VSECTION+AM335X_UART1_OFFSET)
#define AM335X_UART2_VADDR              (AM335X_PERIPH_VSECTION+AM335X_UART2_OFFSET)
#define AM335X_I2C1_VADDR               (AM335X_PERIPH_VSECTION+AM335X_I2C1_OFFSET)
#define AM335X_MCSPI0_VADDR             (AM335X_PERIPH_VSECTION+AM335X_MCSPI0_OFFSET)
#define AM335X_MCASP0_VADDR             (AM335X_PERIPH_VSECTION+AM335X_MCASP0_OFFSET)
#define AM335X_MCASP1_VADDR             (AM335X_PERIPH_VSECTION+AM335X_MCASP1_OFFSET)
#define AM335X_DMTIMER2_VADDR           (AM335X_PERIPH_VSECTION+AM335X_DMTIMER2_OFFSET)
#define AM335X_DMTIMER3_VADDR           (AM335X_PERIPH_VSECTION+AM335X_DMTIMER3_OFFSET)
#define AM335X_DMTIMER4_VADDR           (AM335X_PERIPH_VSECTION+AM335X_DMTIMER4_OFFSET)
#define AM335X_DMTIMER5_VADDR           (AM335X_PERIPH_VSECTION+AM335X_DMTIMER5_OFFSET)
#define AM335X_DMTIMER6_VADDR           (AM335X_PERIPH_VSECTION+AM335X_DMTIMER6_OFFSET)
#define AM335X_DMTIMER7_VADDR           (AM335X_PERIPH_VSECTION+AM335X_DMTIMER7_OFFSET)
#define AM335X_GPIO1_VADDR              (AM335X_PERIPH_VSECTION+AM335X_GPIO1_OFFSET)
#define AM335X_MMCHS0_VADDR             (AM335X_PERIPH_VSECTION+AM335X_MMCHS0_OFFSET)
#define AM335X_ELM_VADDR                (AM335X_PERIPH_VSECTION+AM335X_ELM_OFFSET)
#define AM335X_MAIBOX_VADDR             (AM335X_PERIPH_VSECTION+AM335X_MAIBOX_OFFSET)
#define AM335X_SPINLOCK_VADDR           (AM335X_PERIPH_VSECTION+AM335X_SPINLOCK_OFFSET)

#define AM335X_OCP_WATCHPOINT_VADDR     (AM335X_PERIPH_VSECTION+AM335X_OCP_WATCHPOINT_OFFSET)
#define AM335X_I2C2_VADDR               (AM335X_PERIPH_VSECTION+AM335X_I2C2_OFFSET)
#define AM335X_MCSPI1_VADDR             (AM335X_PERIPH_VSECTION+AM335X_MCSPI1_OFFSET)
#define AM335X_UART3_VADDR              (AM335X_PERIPH_VSECTION+AM335X_UART3_OFFSET)
#define AM335X_UART4_VADDR              (AM335X_PERIPH_VSECTION+AM335X_UART4_OFFSET)
#define AM335X_UART5_VADDR              (AM335X_PERIPH_VSECTION+AM335X_UART5_OFFSET)
#define AM335X_GPIO2_VADDR              (AM335X_PERIPH_VSECTION+AM335X_GPIO2_OFFSET)
#define AM335X_GPIO3_VADDR              (AM335X_PERIPH_VSECTION+AM335X_GPIO3_OFFSET)
#define AM335X_DCAN0_VADDR              (AM335X_PERIPH_VSECTION+AM335X_DCAN0_OFFSET)
#define AM335X_DCAN1_VADDR              (AM335X_PERIPH_VSECTION+AM335X_DCAN1_OFFSET)
#define AM335X_MMC1_VADDR               (AM335X_PERIPH_VSECTION+AM335X_MMC1_OFFSET)

#define AM335X_INTC_VADDR               (AM335X_PERIPH_VSECTION+AM335X_INTC_OFFSET)
#define AM335X_MPUSS_VADDR              (AM335X_PERIPH_VSECTION+AM335X_MPUSS_OFFSET)

#define AM335X_PWMSS0_VADDR             (AM335X_PERIPH_VSECTION+AM335X_PWMSS0_OFFSET)
#define AM335X_ECAP0_VADDR              (AM335X_PERIPH_VSECTION+AM335X_ECAP0_OFFSET)
#define AM335X_EQEP0_VADDR              (AM335X_PERIPH_VSECTION+AM335X_EQEP0_OFFSET)
#define AM335X_EPWM0_VADDR              (AM335X_PERIPH_VSECTION+AM335X_EPWM0_OFFSET)
#define AM335X_PWMSS1_VADDR             (AM335X_PERIPH_VSECTION+AM335X_PWMSS1_OFFSET)
#define AM335X_ECAP1_VADDR              (AM335X_PERIPH_VSECTION+AM335X_ECAP1_OFFSET)
#define AM335X_EQEP1_VADDR              (AM335X_PERIPH_VSECTION+AM335X_EQEP1_OFFSET)
#define AM335X_EPWM1_VADDR              (AM335X_PERIPH_VSECTION+AM335X_EPWM1_OFFSET)
#define AM335X_PWMSS2_VADDR             (AM335X_PERIPH_VSECTION+AM335X_PWMSS2_OFFSET)
#define AM335X_ECAP2_VADDR              (AM335X_PERIPH_VSECTION+AM335X_ECAP2_OFFSET)
#define AM335X_EQEP2_VADDR              (AM335X_PERIPH_VSECTION+AM335X_EQEP2_OFFSET)
#define AM335X_EPWM2_VADDR              (AM335X_PERIPH_VSECTION+AM335X_EPWM2_OFFSET)
#define AM335X_LCD_VADDR                (AM335X_PERIPH_VSECTION+AM335X_LCD_OFFSET)

#define AM335X_EDMA3CC_VADDR            (AM335X_PERIPH_VSECTION+AM335X_EDMA3CC_OFFSET)
#define AM335X_EDMA3TC0_VADDR           (AM335X_PERIPH_VSECTION+AM335X_EDMA3TC0_OFFSET)
#define AM335X_EDMA3TC1_VADDR           (AM335X_PERIPH_VSECTION+AM335X_EDMA3TC1_OFFSET)
#define AM335X_EDMA3TC2_VADDR           (AM335X_PERIPH_VSECTION+AM335X_EDMA3TC2_OFFSET)

#define AM335X_L4_FAST_CONFIG_VADDR     (AM335X_PERIPH_VSECTION+AM335X_L4_FAST_CONFIG_OFFSET)

#define AM335X_CPSW_SS_VADDR            (AM335X_PERIPH_VSECTION+AM335X_CPSW_SS_OFFSET)
#define AM335X_CPSW_PORT_VADDR          (AM335X_PERIPH_VSECTION+AM335X_CPSW_PORT_OFFSET)
#define AM335X_CPSW_CPDMA_VADDR         (AM335X_PERIPH_VSECTION+AM335X_CPSW_CPDMA_OFFSET)
#define AM335X_CPSW_STATS_VADDR         (AM335X_PERIPH_VSECTION+AM335X_CPSW_STATS_OFFSET)
#define AM335X_CPSW_STATERAM_VADDR      (AM335X_PERIPH_VSECTION+AM335X_CPSW_STATERAM_OFFSET)
#define AM335X_CPSW_CPTS_VADDR          (AM335X_PERIPH_VSECTION+AM335X_CPSW_CPTS_OFFSET)
#define AM335X_CPSW_ALE_VADDR           (AM335X_PERIPH_VSECTION+AM335X_CPSW_ALE_OFFSET)
#define AM335X_CPSW_SL1_VADDR           (AM335X_PERIPH_VSECTION+AM335X_CPSW_SL1_OFFSET)
#define AM335X_CPSW_SL2_VADDR           (AM335X_PERIPH_VSECTION+AM335X_CPSW_SL2_OFFSET)
#define AM335X_MDIO_VADDR               (AM335X_PERIPH_VSECTION+AM335X_MDIO_OFFSET)
#define AM335X_CPSW_WR_VADDR            (AM335X_PERIPH_VSECTION+AM335X_CPSW_WR_OFFSET)
#define AM335X_CPPI_RAM_VADDR           (AM335X_PERIPH_VSECTION+AM335X_CPPI_RAM_OFFSET)

#define AM335X_PRU_ICSS_VADDR           (AM335X_PERIPH_VSECTION+AM335X_PRU_ICSS_OFFSET)

#define AM335X_DEBUG_SS_DRM_VADDR       (AM335X_PERIPH_VSECTION+AM335X_DEBUG_SS_DRM_OFFSET)
#define AM335X_DEBUG_SS_ETB_VADDR       (AM335X_PERIPH_VSECTION+AM335X_DEBUG_SS_ETB_OFFSET)

#define AM335X_EMIF0_VADDR              (AM335X_PERIPH_VSECTION+AM335X_EMIF0_OFFSET)

#define AM335X_GPMC_VADDR               (AM335X_PERIPH_VSECTION+AM335X_GPMC_OFFSET)

#define AM335X_ADC_TSC_DMA_VADDR        (AM335X_PERIPH_VSECTION+AM335X_ADC_TSC_DMA_OFFSET)

#define AM335X_SGX530_VADDR             (AM335X_PERIPH_VSECTION+AM335X_SGX530_OFFSET)

/* Offset DDR address */

#define AM335X_DDR_MAPPADDR             (AM335X_DDR_PSECTION+AM335X_DDR_MAPOFFSET)
#define AM335X_DDR_MAPVADDR             (AM335X_DDR_VSECTION+AM335X_DDR_MAPOFFSET)

/* NuttX virtual base address
 *
 * The boot logic will create a temporarily mapping based on where NuttX is
 * executing in memory.  In this case, NuttX will be running from either
 * internal SRAM or external SDRAM.
 *
 * Setup the RAM region as the NUTTX .txt, .bss, and .data region.
 */

#define NUTTX_TEXT_VADDR     (CONFIG_RAM_VSTART & 0xfff00000)
#define NUTTX_TEXT_PADDR     (CONFIG_RAM_START & 0xfff00000)
#define NUTTX_TEXT_PEND      ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#define NUTTX_TEXT_SIZE      (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

/* MMU Page Table
 *
 * Determine the address of the MMU page table.  Regardless of the memory
 * configuration, we will keep the page table in the AM335X's internal SRAM.
 */

#if defined(PGTABLE_BASE_PADDR) || defined(PGTABLE_BASE_VADDR)

  /* Sanity check.. if one is undefined, both should be undefined */

#  if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)
#    error "Only one of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is defined"
#  endif

  /* A sanity check, if the configuration says that the page table is
   * read-only and pre-initialized (maybe ROM), then it should have also
   * defined both of the page table base addresses.
   */

#  ifdef CONFIG_ARCH_ROMPGTABLE
#    error "CONFIG_ARCH_ROMPGTABLE defined; PGTABLE_BASE_P/VADDR not defined"
#  endif

#else /* PGTABLE_BASE_PADDR || PGTABLE_BASE_VADDR */

  /* If CONFIG_PAGING is selected, then parts of the 1-to-1 virtual memory
   * map probably do not apply because paging logic will probably partition
   * the SRAM section differently.  In particular, if the page table is
   * located at the end of SRAM, then the virtual page table address defined
   * below will probably be in error.  In that case PGTABLE_BASE_VADDR is
   * defined in the file mmu.h
   *
   * We must declare the page table at the bottom or at the top of internal
   * SRAM.  We pick the bottom of internal SRAM *unless* there are vectors
   * in the way at that position.
   */

#  if defined(CONFIG_ARCH_LOWVECTORS)
  /* In this case, page table must lie at the top 16Kb of OCMC0 RAM. */

#    define PGTABLE_BASE_PADDR    (AM335X_OCMC0_PADDR + AM335X_OCMC0_SIZE - PGTABLE_SIZE)
#    define PGTABLE_BASE_VADDR    (AM335X_OCMC0_VADDR + AM335X_OCMC0_SIZE - PGTABLE_SIZE)
#    define PGTABLE_IN_HIGHSRAM   1

  /* We will force the IDLE stack to precede the page table */

#    define IDLE_STACK_PBASE      (PGTABLE_BASE_PADDR - CONFIG_IDLETHREAD_STACKSIZE)
#    define IDLE_STACK_VBASE      (PGTABLE_BASE_VADDR - CONFIG_IDLETHREAD_STACKSIZE)

#  else /* CONFIG_ARCH_LOWVECTORS */

  /* Otherwise, the vectors lie at another location (perhaps in NOR FLASH,
   * perhaps elsewhere in OCMC0 RAM).  The page table will then be positioned
   * at the first 16Kb of OCMC0 RAM.
   */

#    define PGTABLE_BASE_PADDR    AM335X_OCMC0_PADDR
#    define PGTABLE_BASE_VADDR    AM335X_OCMC0_VADDR
#    define PGTABLE_IN_LOWSRAM    1

  /* We will force the IDLE stack to follow the page table */

#    define IDLE_STACK_PBASE      (PGTABLE_BASE_PADDR + PGTABLE_SIZE)
#    define IDLE_STACK_VBASE      (PGTABLE_BASE_VADDR + PGTABLE_SIZE)

#  endif /* CONFIG_ARCH_LOWVECTORS */

  /* Note that the page table does not lie in the same address space as does
   * the mapped RAM in either case.  So we will need to create a special
   * mapping for the page table at boot time.
   */

#  define ARMV7A_PGTABLE_MAPPING 1

#endif /* PGTABLE_BASE_PADDR || PGTABLE_BASE_VADDR */

/* Level 2 Page table start addresses.
 *
 * 16Kb of memory is reserved hold the page table for the virtual mappings.
 *  A portion of this table is not accessible in the virtual address space
 * (for normal operation).
 */

#if !defined(CONFIG_ARCH_LOWVECTORS)
/* Vector L2 page table offset/size */

#  define VECTOR_L2_OFFSET        0x000000400
#  define VECTOR_L2_SIZE          0x000000bfc

/* Vector L2 page table base addresses */

#  define VECTOR_L2_PBASE         (PGTABLE_BASE_PADDR + VECTOR_L2_OFFSET)
#  define VECTOR_L2_VBASE         (PGTABLE_BASE_VADDR + VECTOR_L2_OFFSET)

/* Vector L2 page table end addresses */

#  define VECTOR_L2_END_PADDR     (VECTOR_L2_PBASE + VECTOR_L2_SIZE)
#  define VECTOR_L2_END_VADDR     (VECTOR_L2_VBASE + VECTOR_L2_SIZE)

#endif /* !CONFIG_ARCH_LOWVECTORS */

/* Paging L2 page table offset/size */

#define PGTABLE_L2_START_PADDR    (AM335X_DDR_PSECTION + AM335X_DDR_MAPOFFSET + AM335X_DDR_MAPSIZE)
#define PGTABLE_BROM_OFFSET       0x3ffc

#define PGTABLE_L2_OFFSET         ((PGTABLE_L2_START_PADDR >> 18) & ~3)
#define PGTABLE_L2_SIZE           (PGTABLE_BROM_OFFSET - PGTABLE_L2_OFFSET)

/* Paging L2 page table base addresses
 *
 * NOTE: If CONFIG_PAGING is defined, mmu.h will re-assign the virtual
 * address of the page table.
 */

#define PGTABLE_L2_PBASE          (PGTABLE_BASE_PADDR + PGTABLE_L2_OFFSET)
#define PGTABLE_L2_VBASE          (PGTABLE_BASE_VADDR + PGTABLE_L2_OFFSET)

/* Paging L2 page table end addresses */

#define PGTABLE_L2_END_PADDR      (PGTABLE_L2_PBASE + PGTABLE_L2_SIZE)
#define PGTABLE_L2_END_VADDR      (PGTABLE_L2_VBASE + PGTABLE_L2_SIZE)

/* Base address of the interrupt vector table.
 *
 *   AM335X_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
 *   AM335X_VECTOR_VSRAM - Virtual address of vector table in SRAM
 *   AM335X_VECTOR_VADDR - Virtual address of vector table
 *                         (0x00000000 or 0xffff0000)
 */

#define VECTOR_TABLE_SIZE         0x00010000

/* REVISIT: These definitions are not used:  The vector table is at some
 * arbitrary (but aligned) position in RAM or NOR FLASH and is positioned
 * using the VBAR register. For AM335X at start of OCMC0 RAM by default.
 */

#ifdef CONFIG_ARCH_LOWVECTORS  /* Vectors located at 0x0000:0000  */

#  define AM335X_VECTOR_PADDR     AM335X_OCMC0_PADDR
#  define AM335X_VECTOR_VSRAM     AM335X_OCMC0_VADDR
#  define AM335X_VECTOR_VADDR     0x00000000

#else  /* Vectors located at 0xffff:0000 -- this probably does not work */

#  define AM335X_VECTOR_PADDR     (AM335X_OCMC0_PADDR + AM335X_OCMC0_SIZE - VECTOR_TABLE_SIZE)
#  define AM335X_VECTOR_VSRAM     (AM335X_OCMC0_VADDR + AM335X_OCMC0_SIZE - VECTOR_TABLE_SIZE)
#  define AM335X_VECTOR_VADDR     0xffff0000

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_AM335X_HARDWARE_AM3358_MEMORYMAP_H */
