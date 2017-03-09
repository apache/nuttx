/************************************************************************************
 * arch/arm/src/imx6/chip/imx_memorymap.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "i.MX 6Dual/6Quad ApplicationsProcessor Reference Manual," Document Number
 *   IMX6DQRM, Rev. 3, 07/2015, FreeScale.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMX6_CHIP_IMX_MEMORYMAP_H
#define __ARCH_ARM_SRC_IMX6_CHIP_IMX_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/imx6/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Decimal configuration values may exceed 2Gb and, hence, overflow to negative
 * values unless we force them to unsigned long:
 */

#define __CONCAT(a,b) a ## b
#define MKULONG(a) __CONCAT(a,ul)

/* Overview *****************************************************************
 *
 *  i.MX6 Physical (unmapped) Memory Map
 *  - i.MX6 System 1MB PSECTIONS
 *  - i.MX6 DMA PSECTION Offsets
 *  - i.MX6 OCRAM PSECTION Offsets
 *  - i.MX6 ARM MP PSECTION Offsets
 *  - i.MX6 PCIE PSECTION Offsets
 *  - i.MX6 AIPS-1 PSECTION Offsets
 *  - i.MX6 AIPS-2 PSECTION Offsets
 *  - i.MX6 DAP AIPS-2 PSECTION Offsets
 *  - i.MX6 SATA PSECTION Offsets
 *  - i.MX6 DMA Physical Base Addresses
 *  - i.MX6 OCRAM Physical Base Addresses
 *  - i.MX6 ARM MP Physical Base Addresses
 *  - i.MX6 PCIE Physical Base Addresses
 *  - i.MX6 AIPS-1 Physical Base Addresses
 *  - i.MX6 AIPS-2 Physical Base Addresses
 *  - i.MX6 DAP AIPS-2 Physical Base Addresses
 *  - i.MX6 SATA Physical Base Addresses
 *  Sizes of memory regions in bytes
 *  Sizes of memory regions in sections
 *  Section MMU Flags
 *  i.MX6 Virtual (mapped) Memory Map
 *  - i.MX6 DMA Virtual Base Addresses
 *  - i.MX6 OCRAM Virtual Base Addresses
 *  - i.MX6 ARM MP Virtual Base Addresses
 *  - i.MX6 PCIE Virtual Base Addresses
 *  - i.MX6 AIPS-1 Virtual Base Addresses
 *  - i.MX6 AIPS-2 Virtual Base Addresses
 *  - i.MX6 DAP AIPS-2 Virtual Base Addresses
 *  - i.MX6 SATA Virtual Base Addresses
 *  - NuttX vitual base address
 *  MMU Page Table Location
 *  Page table start addresses
 *  Base address of the interrupt vector table
 *
 ****************************************************************************/

/* i.MX6 Physical (unmapped) Memory Map *************************************/
/* i.MX6 System PSECTIONS */

#define IMX_ROMCP_PSECTION       0x00000000  /* 00000000-00017fff  96 KB Boot ROM (ROMCP) */
                                             /* 00018000-000fffff 928 KB Reserved */
#define IMX_DMA_PSECTION         0x00100000  /* 00100000-001fffff   1 MB See offsets below */
#define IMX_GPV2_PSECTION        0x00200000  /* 00200000-002fffff   1 MB GPV_2 PL301 (per1) configuration port */
#define IMX_GPV3_PSECTION        0x00300000  /* 00300000-003fffff   1 MB GPV_3 PL301 (per2) configuration port */
                                             /* 00400000-007fffff   4 MB Reserved */
#define IMX_GPV4_PSECTION        0x00800000  /* 00800000-008fffff   1 MB GPV_4 PL301 (fast3) configuration port */
#define IMX_OCRAM_PSECTION       0x00900000  /* 00900000-009fffff   1 MB OCRAM */
#define IMX_ARMMP_PSECTION       0x00a00000  /* 00a00000-00afffff   8 KB ARM MP */
#define IMX_GPV0PL301_PSECTION   0x00b00000  /* 00b00000-00bfffff   1 MB GPV0 PL301 (fast2) configuration port */
#define IMX_GPV1PL301_PSECTION   0x00c00000  /* 00c00000-00cfffff   1 MB GPV1 PL301 (fast1) configuration port */
                                             /* 00d00000-00ffffff 3072 KB Reserved */
#define IMX_PCIE_PSECTION        0x01000000  /* 01000000-01ffffff  16 MB PCIe */
#define IMX_AIPS1_PSECTION       0x02000000  /* 02000000-020fffff   1 MB Peripheral IPs via AIPS-1 */
#define IMX_AIPS2_PSECTION       0x02100000  /* 02100000-021fffff   1 MB Peripheral IPs via AIPS-2 */
#define IMX_SATA_PSECTION        0x02200000  /* 02200000-0220bfff  48 KB SATA */
                                             /* 0220c000-023fffff   2 MB Reserved */
#define IMX_IPU1_PSECTION        0x02600000  /* 02600000-029fffff   4 MB IPU-1 */
#define IMX_IPU2_PSECTION        0x02a00000  /* 02a00000-02dfffff   4 MB IPU-2 */
#define IMX_EIM_PSECTION         0x08000000  /* 08000000-0fffffff 128 MB EIM - (NOR/SRAM) */
#define IMX_MMDCDDR_PSECTION     0x10000000  /* 10000000-ffffffff 3840 MB MMDC-DDR Controller */
                                             /* 10000000-7fffffff 1792 MB */

/* By default, NuttX uses a 1-1 memory mapping.  So the unused, reserved
 * address in the top-level memory map are candidates for other mapping uses:
 *
 *  00018000-000fffff Reserved -- Not used
 *  00400000-007fffff Reserved -- Not used
 *  00d00000-00ffffff Reserved -- Not used
 *  0220c000-023fffff Reserved -- Not used
 *  80000000-efffffff Reserved -- Level 2 page table (See below)
 */

/* i.MX6 DMA PSECTION Offsets */

#define IMX_CAAMRAM_OFFSET       0x00000000  /* 00000000-00003fff  16 KB CAAM (16K secure RAM) */
                                             /* 00004000-0000ffff  48 KB Reserved */
#define IMX_APBHDMA_OFFSET       0x00010000  /* 00010000-00011fff   8 KB APBH-Bridge-DMA */
#define IMX_GPMI_OFFSET          0x00012000  /* 00012000-00013fff   8 KB GPMI */
#define IMX_BCH_OFFSET           0x00014000  /* 00014000-00017fff  16 KB BCH */
                                             /* 00018000-0001ffff  32 KB Reserved */
#define IMX_HDMI_OFFSET          0x00020000  /* 00020000-00028fff  36 KB HDMI */
                                             /* 00029000-0002ffff  28 KB Reserved */
#define IMX_GPU3D_OFFSET         0x00030000  /* 00030000-00033fff  16 KB GPU 3D */
#define IMX_GPU2D_OFFSET         0x00034000  /* 00034000-00037fff  16 KB GPU 2D (GC320) */
#define IMX_DTCP_OFFSET          0x00038000  /* 00038000-0003bfff  16 KB DTCP */
                                             /* 0003c000-000fffff 784 KB Reserved */
/* i.MX6 OCRAM PSECTION Offsets */

#define IMX_OCRAM_OFFSET         0x00000000  /* 00000000-0003ffff  0.25 MB OCRAM 256 KB */
#define IMX_OCRAMALIAS_OFFSET    0x00040000  /* 00040000-000fffff  0.75 MB OCRAM aliased */

/* i.MX6 ARM MP PSECTION Offsets */

#define IMX_MPSCU_OFFSET         0x00000000  /* 00000000-000000fc  0.25 KB SCU registers */
#define IMX_MPICC_OFFSET         0x00000100  /* 00000100-000001ff  0.25 KB Interrupt controller interfaces */
#define IMX_MPGTM_OFFSET         0x00000200  /* 00000200-000002ff  0.25 KB Global timer */
                                             /* 00000300-000005ff        Reserved */
#define IMX_MPPTM_OFFSET         0x00000600  /* 00000600-000006ff  0.25 KB Private timers and watchdogs */
                                             /* 00000700-00000fff        Reserved */
#define IMX_MPICD_OFFSET         0x00001000  /* 00001000-00001fff   4 KB Interrupt distributor */
#define IMX_PL310_OFFSET         0x00002000  /* 00002000-00002fff   4 KB PL310 (L2 Cache controller) */
                                             /* 00003000-000fffff 1012 KB Reserved */

/* i.MX6 PCIE PSECTION Offsets */

#define IMX_PCIE_OFFSET          0x00000000  /* 00000000-00ffbfff 16368 KB PCIe */
#define IMX_PCIEREGS_OFFSET      0x00ffc000  /* 00ffc000-00ffffff  16 KB PCIe registers */

/* i.MX6 AIPS-1 PSECTION Offsets */

                                             /* 00000000 00003fff Reserved for SDMA internal registers 16 KB */
#define IMX_SPDIF_OFFSET         0x00004000  /* 00004000 00007fff SPDIF 16 KB */
#define IMX_ECSPI1_OFFSET        0x00008000  /* 00008000 0000bfff eCSPI1 16 KB */
#define IMX_ECSPI2_OFFSET        0x0000c000  /* 0000c000 0000ffff eCSPI2 16KB */
#define IMX_ECSPI3_OFFSET        0x00010000  /* 00010000 00013fff eCSPI3 16 KB */
#define IMX_ECSPI4_OFFSET        0x00014000  /* 00014000 00017fff eCSPI4 16 KB */
#define IMX_ECSPI5_OFFSET        0x00018000  /* 00018000 0001bfff eCSPI5 16 KB */
                                             /* 0001c000 0001ffff Reserved for SDMA internal registers 16 KB */
#define IMX_UART1_OFFSET         0x00020000  /* 00020000 00023fff UART1 16 KB */
#define IMX_ESAI_OFFSET          0x00024000  /* 00024000 00027fff ESAI 16 KB */
#define IMX_SSI1_OFFSET          0x00028000  /* 00028000 0002bfff SSI1 16 KB */
#define IMX_SSI2_OFFSET          0x0002c000  /* 0002c000 0002ffff SSI2 16 KB */
#define IMX_SSI3_OFFSET          0x00030000  /* 00030000 00033fff SSI3 16 KB */
#define IMX_ASRC_OFFSET          0x00034000  /* 00034000 00037fff ASRC 16 KB */
                                             /* 00038000 0003bfff Reserved for SDMA internal registers 16 KB */
#define IMX_SPBA_OFFSET          0x0003c000  /* 0003c000 0003ffff SPBA 16 KB */
#define IMX_VPU_OFFSET           0x00040000  /* 00040000 0007bfff AIPS-1 VPU 240 KB */
#define IMX_APIS1CFG_OFFSET      0x0007c000  /* 0007c000 0007ffff AIPS-1 Configuration 16 KB */
#define IMX_PWM1_OFFSET          0x00080000  /* 00080000 00083fff PWM1 16 KB */
#define IMX_PWM2_OFFSET          0x00084000  /* 00084000 00087fff PWM2 16 KB */
#define IMX_PWM3_OFFSET          0x00088000  /* 00088000 0008bfff PWM2 16 KB */
#define IMX_PWM4_OFFSET          0x0008c000  /* 0008c000 0008ffff PWM4 16 KB */
#define IMX_CAN1_OFFSET          0x00090000  /* 00090000 00093fff CAN1 16 KB */
#define IMX_CAN2_OFFSET          0x00094000  /* 00094000 00097fff CAN2 16 KB */
#define IMX_GPT_OFFSET           0x00098000  /* 00098000 0009bfff GPT 16 KB */
#define IMX_GPIO_OFFSET(n)       (0x0009c000 + ((n) << 14)) /* n=0..6 */
#define IMX_GPIO1_OFFSET         0x0009c000  /* 0009c000 0009ffff GPIO1 16 KB */
#define IMX_GPIO2_OFFSET         0x000a0000  /* 000a0000 000a3fff GPIO2 16 KB */
#define IMX_GPIO3_OFFSET         0x000a4000  /* 000a4000 000a7fff GPIO3 16 KB */
#define IMX_GPIO4_OFFSET         0x000a8000  /* 000a8000 000abfff GPIO4 16 KB */
#define IMX_GPIO5_OFFSET         0x000ac000  /* 000ac000 000affff GPIO5 16 KB */
#define IMX_GPIO6_OFFSET         0x000b0000  /* 000b0000 000b3fff GPIO6 16 KB */
#define IMX_GPIO7_OFFSET         0x000b4000  /* 000b4000 000b7fff GPIO7 16 KB */
#define IMX_KPP_OFFSET           0x000b8000  /* 000b8000 000bbfff KPP 16 KB */
#define IMX_WDOG1_OFFSET         0x000bc000  /* 000bc000 000bffff WDOG1 16 KB */
#define IMX_WDOG2_OFFSET         0x000c0000  /* 000c0000 000c3fff WDOG2 16 KB */
#define IMX_CCM_OFFSET           0x000c4000  /* 000c4000 000c7fff CCM 16 KB */
#define IMX_ANALOG_OFFSET        0x000c8000  /* 000c8000 000c8fff ANALOG 4 KB */
#define IMX_USBPHY1_OFFSET       0x000c9000  /* 000c9000 000c9fff USBPHY1 4 KB */
#define IMX_USBPHY2_OFFSET       0x000ca000  /* 000ca000 000cafff USBPHY2 4 KB */
                                             /* 000cb000 000cbfff Reserved 4 KB */
#define IMX_SNVSHP_OFFSET        0x000cc000  /* 000cc000 000cffff SNVSHP 16 KB */
#define IMX_EPIT1_OFFSET         0x000d0000  /* 000d0000 000d3fff EPIT1 16 KB */
#define IMX_EPIT2_OFFSET         0x000d4000  /* 000d4000 000d7fff EPIT2 16 KB */
#define IMX_SRC_OFFSET           0x000d8000  /* 000d8000 000dbfff SRC 16 KB */
#define IMX_GPC_OFFSET           0x000dc000  /* 000dc000 000dd25f GPC 608 B */
#define IMX_PGCPU_OFFSET         0x000dc260  /* 000dc260 000dd27f PGCPU 32 B */
                                             /* 000dc280 000dd29f Reserved 32 B */
#define IMX_PGCARM_OFFSET        0x000dc2a0  /* 000dc2a0 000dd2bf PGCARM 32 B */
                                             /* 000dc2C0 000dffff Reserved 15680 B */
#define IMX_IOMUXC_OFFSET        0x000e0000  /* 000e0000 000e3fff IOMUXC 16 KB */
#define IMX_DCIC1_OFFSET         0x000e4000  /* 000e4000 000e7fff DCIC1 16 KB */
#define IMX_DCIC2_OFFSET         0x000e8000  /* 000e8000 000ebfff DCIC2 16 KB */
#define IMX_SDMA_OFFSET          0x000ec000  /* 000ec000 000effff SDMA 16 KB */
                                             /* 000f0000 000f3fff Reserved 16 KB */
                                             /* 000f4000 000f7fff Reserved 16 KB */
                                             /* 000f8000 000fbfff Reserved 16 KB */
                                             /* 000fc000 000fffff AIPS-1 Reserved 16 KB */
/* i.MX6 AIPS-2 PSECTION Offsets */

#define IMX_CAAM_OFFSET          0x00100000  /* 00100000 0210ffff CAAM 64 KB */
                                             /* 00110000 0213ffff Reserved 192 KB */
#define IMX_DAP_OFFSET           0x00140000  /* 00140000 00160fff ARM Cortex A9 MPCore / DAP 132 KB (See below) */
                                             /* 00161000 0017bfff ARM Cortex A9 MPCore - Reserved 108 KB */
#define IMX_AIPS2CGF_OFFSET      0x0017c000  /* 0017c000 0017ffff AIPS-2 configuration 16 KB */
                                             /* 00180000 00183fff Reserved 16 KB */
#define IMX_USBOH3_OFFSET        0x00184000  /* 00184000 00187fff USBOH3 (USB) 16 KB */
#define IMX_ENET_OFFSET          0x00188000  /* 00188000 0018bfff ENET 16 KB */
#define IMX_MLB150_OFFSET        0x0018c000  /* 0018c000 0018ffff MLB150 16 KB */
#define IMX_USDHC1_OFFSET        0x00190000  /* 00190000 00193fff uSDHC1 16 KB */
#define IMX_USDHC2_OFFSET        0x00194000  /* 00194000 00197fff uSDHC2 16 KB */
#define IMX_USDHC3_OFFSET        0x00198000  /* 00198000 0019bfff uSDHC3 16 KB */
#define IMX_USDHC4_OFFSET        0x0019c000  /* 0019c000 0019ffff uSDHC4 16 KB */
#define IMX_I2C1_OFFSET          0x001a0000  /* 001a0000 001a3fff I2C1 16 KB */
#define IMX_I2C2_OFFSET          0x001a4000  /* 001a4000 001a7fff I2C2 16 KB */
#define IMX_I2C3_OFFSET          0x001a8000  /* 001a8000 001abfff I2C3 16 KB */
#define IMX_ROMCP_OFFSET         0x001ac000  /* 001ac000 001affff ROMCP 16 KB */
#define IMX_MMDC_OFFSET          0x001b0000  /* 001b0000 001b3fff MMDC 16 KB */
#define IMX_MMDCP1_OFFSET        0x001b4000  /* 001b4000 001b7fff MMDC (port 1) 16 KB */
#define IMX_EIM_OFFSET           0x001b8000  /* 001b8000 001bbfff EIM 16 KB */
#define IMX_OCOTPCTRL_OFFSET     0x001bc000  /* 001bc000 001bffff OCOTPCTRL 16 KB */
#define IMX_CSU_OFFSET           0x001c0000  /* 001c0000 001c3fff CSU 16 KB */
                                             /* 001c4000 001c7fff Reserved */
                                             /* 001c8000 Reserved */
                                             /* 001cc000 Reserved */
#define IMX_TZASC1_OFFSET        0x001d0000  /* 001d0000 001d3fff TZASC1 16 KB */
#define IMX_TZASC2_OFFSET        0x001d4000  /* 001d4000 001d7fff TZASC2 16 KB */
#define IMX_AUDMUX_OFFSET        0x001d8000  /* 001d8000 001dbfff AUDMUX 16 KB */
#define IMX_MIPICSI_OFFSET       0x001dc000  /* 001dc000 001dffff MIPI (CSI port) 16 KB */
#define IMX_MIPIDSI_OFFSET       0x001e0000  /* 001e0000 001e3fff MIPI (DSI port) 16 KB */
#define IMX_VDOA_OFFSET          0x001e4000  /* 001e4000 001e7fff VDOA 16 KB */
#define IMX_UART2_OFFSET         0x001e8000  /* 001e8000 001ebfff UART2 16 KB */
#define IMX_UART3_OFFSET         0x001ec000  /* 001ec000 001effff UART3 16 KB */
#define IMX_UART4_OFFSET         0x001f0000  /* 001f0000 001f3fff UART4 16 KB */
#define IMX_UART5_OFFSET         0x001f4000  /* 001f4000 001f7fff UART5 16 KB */
                                             /* 001f8000 001fbfff Reserved 16 KB */

/* i.MX6 DAP AIPS-2 PSECTION Offsets */

#define IMX_DAPROM_OFFSET        0x00140000  /* 00140000 00140fff 4 KB DAP ROM Table */
#define IMX_ETB_OFFSET           0x00141000  /* 00141000 00141fff 4 KB ETB */
#define IMX_EXTCTI_OFFSET        0x00142000  /* 00142000 00142fff 4 KB ext. CTI */
#define IMX_TPIU_OFFSET          0x00143000  /* 00143000 00143fff 4 KB TPIU */
#define IMX_FUNNEL_OFFSET        0x00144000  /* 00144000 00144fff 4 KB FUNNEL */
                                             /* 00145000 0014efff 40 KB Reserved */
#define IMX_CA9INTEG_OFFSET      0x0014f000  /* 0014f000 0014ffff 4 KB CA9-INTEG */
#define IMX_CPUDBG_OFFSET(n)     (0x00150000 + ((n) << 13))
#define IMX_CPUPMU_OFFSET(n)     (0x00151000 + ((n) << 13))
#define IMX_CPU0DBG_OFFSET       0x00150000  /* 00150000 00150fff 4 KB CPU0 Debug I/F */
#define IMX_CPU0PMU_OFFSET       0x00151000  /* 00151000 00151fff 4 KB CPU0 PMU */
#define IMX_CPU1DBG_OFFSET       0x00152000  /* 00152000 00152fff 4 KB CPU1 Debug I/F */
#define IMX_CPU1PMC_OFFSET       0x00153000  /* 00153000 00153fff 4 KB CPU1 PMU */
#define IMX_CPU2DBG_OFFSET       0x00154000  /* 00154000 00154fff 4 KB CPU2 Debug I/F */
#define IMX_CPU2PMU_OFFSET       0x00155000  /* 00155000 00155fff 4 KB CPU2 PMU */
#define IMX_CPU3DBG_OFFSET       0x00156000  /* 00156000 00156fff 4 KB CPU3 Debug I/F */
#define IMX_CPU3PMU_OFFSET       0x00157000  /* 00157000 00157fff 4 KB CPU3 PMU */
#define IMX_CTI_OFFSET(n)        (0x00158000 + ((n) << 12))
#define IMX_CTI0_OFFSET          0x00158000  /* 00158000 00158fff 4 KB CTI0 */
#define IMX_CTI1_OFFSET          0x00159000  /* 00159000 00159fff 4 KB CTI1 */
#define IMX_CTI2_OFFSET          0x0015a000  /* 0015a000 0015afff 4 KB CTI2 */
#define IMX_CTI3_OFFSET          0x0015b000  /* 0015b000 0015bfff 4 KB CTI3 */
#define IMX_PTM_OFFSET(n)        (0x0015c000 + ((n) << 12))
#define IMX_PTM0_OFFSET          0x0015c000  /* 0015c000 0015cfff 4 KB PTM0 */
#define IMX_PTM1_OFFSET          0x0015d000  /* 0015d000 0015dfff 4 KB PTM1 */
#define IMX_PTM2_OFFSET          0x0015e000  /* 0015e000 0015efff 4 KB PTM2 */
#define IMX_PTM3_OFFSET          0x0015f000  /* 0015f000 0015ffff 4 KB PTM3 */
#define IMX_PLATCTRL_OFFSET      0x00160000  /* 00160000 00160fff 4 KB Platform Control */

/* i.MX6 SATA PSECTION Offsets */

#define IMX_SATA_OFFSET          0x00200000  /* 00200000-02203fff  16 KB SATA */
#define IMX_OPENVG_OFFSET        0x00204000  /* 00204000-02207fff  16 KB OpenVG (GC355) */
#define IMX_MIPIHSI_OFFSET       0x00208000  /* 00208000-0220bfff  16 KB MIPIHSI */
                                             /* 0020c000-000fffff   2 MB Reserved */
/* i.MX6 DMA Physical Base Addresses */

#define IMX_CAAMRAM_PBASE        (IMX_DMA_PSECTION+IMX_CAAMRAM_OFFSET)
#define IMX_APBHDMA_PBASE        (IMX_DMA_PSECTION+IMX_APBHDMA_OFFSET)
#define IMX_GPMI_PBASE           (IMX_DMA_PSECTION+IMX_GPMI_OFFSET)
#define IMX_BCH_PBASE            (IMX_DMA_PSECTION+IMX_BCH_OFFSET)
#define IMX_HDMI_PBASE           (IMX_DMA_PSECTION+IMX_HDMI_OFFSET)
#define IMX_GPU3D_PBASE          (IMX_DMA_PSECTION+IMX_GPU3D_OFFSET)
#define IMX_GPU2D_PBASE          (IMX_DMA_PSECTION+IMX_GPU2D_OFFSET)
#define IMX_DTCP_PBASE           (IMX_DMA_PSECTION+IMX_DTCP_OFFSET)

/* i.MX6 OCRAM Physical Base Addresses */

#define IMX_OCRAM_PBASE          (IMX_OCRAM_PSECTION+IMX_OCRAM_OFFSET)
#define IMX_OCRAMALIAS_PBASE     (IMX_OCRAM_PSECTION+IMX_OCRAMALIAS_OFFSET)

/* i.MX6 ARM MP Physical Base Addresses */

#define IMX_MPSCU_PBASE          (IMX_ARMMP_PSECTION+IMX_MPSCU_OFFSET)
#define IMX_MPICC_PBASE          (IMX_ARMMP_PSECTION+IMX_MPICC_OFFSET)
#define IMX_MPGTM_PBASE          (IMX_ARMMP_PSECTION+IMX_MPGTM_OFFSET)
#define IMX_MPPTM_PBASE          (IMX_ARMMP_PSECTION+IMX_MPPTM_OFFSET)
#define IMX_MPICD_PBASE          (IMX_ARMMP_PSECTION+IMX_MPICD_OFFSET)
#define IMX_PL310_PBASE          (IMX_ARMMP_PSECTION+IMX_PL310_OFFSET)

/* i.MX6 PCIE Physical Base Addresses */

#define IMX_PCIE_PBASE           (IMX_PCIE_PSECTION+IMX_PCIE_OFFSET)
#define IMX_PCIEREGS_PBASE       (IMX_PCIE_PSECTION+IMX_PCIEREGS_OFFSET)

/* i.MX6 AIPS-1 Physical Base Addresses */

#define IMX_SPDIF_PBASE          (IMX_AIPS1_PSECTION+IMX_SPDIF_OFFSET)
#define IMX_ECSPI1_PBASE         (IMX_AIPS1_PSECTION+IMX_ECSPI1_OFFSET)
#define IMX_ECSPI2_PBASE         (IMX_AIPS1_PSECTION+IMX_ECSPI2_OFFSET)
#define IMX_ECSPI3_PBASE         (IMX_AIPS1_PSECTION+IMX_ECSPI3_OFFSET)
#define IMX_ECSPI4_PBASE         (IMX_AIPS1_PSECTION+IMX_ECSPI4_OFFSET)
#define IMX_ECSPI5_PBASE         (IMX_AIPS1_PSECTION+IMX_ECSPI5_OFFSET)
#define IMX_UART1_PBASE          (IMX_AIPS1_PSECTION+IMX_UART1_OFFSET)
#define IMX_ESAI_PBASE           (IMX_AIPS1_PSECTION+IMX_ESAI_OFFSET)
#define IMX_SSI1_PBASE           (IMX_AIPS1_PSECTION+IMX_SSI1_OFFSET)
#define IMX_SSI2_PBASE           (IMX_AIPS1_PSECTION+IMX_SSI2_OFFSET)
#define IMX_SSI3_PBASE           (IMX_AIPS1_PSECTION+IMX_SSI3_OFFSET)
#define IMX_ASRC_PBASE           (IMX_AIPS1_PSECTION+IMX_ASRC_OFFSET)
#define IMX_SPBA_PBASE           (IMX_AIPS1_PSECTION+IMX_SPBA_OFFSET)
#define IMX_VPU_PBASE            (IMX_AIPS1_PSECTION+IMX_VPU_OFFSET)
#define IMX_APIS1CFG_PBASE       (IMX_AIPS1_PSECTION+IMX_APIS1CFG_OFFSET)
#define IMX_PWM1_PBASE           (IMX_AIPS1_PSECTION+IMX_PWM1_OFFSET)
#define IMX_PWM2_PBASE           (IMX_AIPS1_PSECTION+IMX_PWM2_OFFSET)
#define IMX_PWM3_PBASE           (IMX_AIPS1_PSECTION+IMX_PWM3_OFFSET)
#define IMX_PWM4_PBASE           (IMX_AIPS1_PSECTION+IMX_PWM4_OFFSET)
#define IMX_CAN1_PBASE           (IMX_AIPS1_PSECTION+IMX_CAN1_OFFSET)
#define IMX_CAN2_PBASE           (IMX_AIPS1_PSECTION+IMX_CAN2_OFFSET)
#define IMX_GPT_PBASE            (IMX_AIPS1_PSECTION+IMX_GPT_OFFSET)
#define IMX_GPIO_PBASE(n)        (IMX_AIPS1_PSECTION+IMX_GPIO_OFFSET(n))
#define IMX_GPIO1_PBASE          (IMX_AIPS1_PSECTION+IMX_GPIO1_OFFSET)
#define IMX_GPIO2_PBASE          (IMX_AIPS1_PSECTION+IMX_GPIO2_OFFSET)
#define IMX_GPIO3_PBASE          (IMX_AIPS1_PSECTION+IMX_GPIO3_OFFSET)
#define IMX_GPIO4_PBASE          (IMX_AIPS1_PSECTION+IMX_GPIO4_OFFSET)
#define IMX_GPIO5_PBASE          (IMX_AIPS1_PSECTION+IMX_GPIO5_OFFSET)
#define IMX_GPIO6_PBASE          (IMX_AIPS1_PSECTION+IMX_GPIO6_OFFSET)
#define IMX_GPIO7_PBASE          (IMX_AIPS1_PSECTION+IMX_GPIO7_OFFSET)
#define IMX_KPP_PBASE            (IMX_AIPS1_PSECTION+IMX_KPP_OFFSET)
#define IMX_WDOG1_PBASE          (IMX_AIPS1_PSECTION+IMX_WDOG1_OFFSET)
#define IMX_WDOG2_PBASE          (IMX_AIPS1_PSECTION+IMX_WDOG2_OFFSET)
#define IMX_CCM_PBASE            (IMX_AIPS1_PSECTION+IMX_CCM_OFFSET)
#define IMX_ANALOG_PBASE         (IMX_AIPS1_PSECTION+IMX_ANALOG_OFFSET)
#define IMX_USBPHY1_PBASE        (IMX_AIPS1_PSECTION+IMX_USBPHY1_OFFSET)
#define IMX_USBPHY2_PBASE        (IMX_AIPS1_PSECTION+IMX_USBPHY2_OFFSET)
#define IMX_SNVSHP_PBASE         (IMX_AIPS1_PSECTION+IMX_SNVSHP_OFFSET)
#define IMX_EPIT1_PBASE          (IMX_AIPS1_PSECTION+IMX_EPIT1_OFFSET)
#define IMX_EPIT2_PBASE          (IMX_AIPS1_PSECTION+IMX_EPIT2_OFFSET)
#define IMX_SRC_PBASE            (IMX_AIPS1_PSECTION+IMX_SRC_OFFSET)
#define IMX_GPC_PBASE            (IMX_AIPS1_PSECTION+IMX_GPC_OFFSET)
#define IMX_PGCPU_PBASE          (IMX_AIPS1_PSECTION+IMX_PGCPU_OFFSET)
#define IMX_PGCARM_PBASE         (IMX_AIPS1_PSECTION+IMX_PGCARM_OFFSET)
#define IMX_IOMUXC_PBASE         (IMX_AIPS1_PSECTION+IMX_IOMUXC_OFFSET)
#define IMX_DCIC1_PBASE          (IMX_AIPS1_PSECTION+IMX_DCIC1_OFFSET)
#define IMX_DCIC2_PBASE          (IMX_AIPS1_PSECTION+IMX_DCIC2_OFFSET)
#define IMX_SDMA_PBASE           (IMX_AIPS1_PSECTION+IMX_SDMA_OFFSET)

/* i.MX6 AIPS-2 Physical Base Addresses */

#define IMX_CAAM_PBASE           (IMX_AIPS2_PSECTION+IMX_CAAM_OFFSET)
#define IMX_DAP_PBASE            (IMX_AIPS2_PSECTION+IMX_DAP_OFFSET)
#define IMX_AIPS2CGF_PBASE       (IMX_AIPS2_PSECTION+IMX_AIPS2CGF_OFFSET)
#define IMX_USBOH3_PBASE         (IMX_AIPS2_PSECTION+IMX_USBOH3_OFFSET)
#define IMX_ENET_PBASE           (IMX_AIPS2_PSECTION+IMX_ENET_OFFSET)
#define IMX_MLB150_PBASE         (IMX_AIPS2_PSECTION+IMX_MLB150_OFFSET)
#define IMX_USDHC1_PBASE         (IMX_AIPS2_PSECTION+IMX_USDHC1_OFFSET)
#define IMX_USDHC2_PBASE         (IMX_AIPS2_PSECTION+IMX_USDHC2_OFFSET)
#define IMX_USDHC3_PBASE         (IMX_AIPS2_PSECTION+IMX_USDHC3_OFFSET)
#define IMX_USDHC4_PBASE         (IMX_AIPS2_PSECTION+IMX_USDHC4_OFFSET)
#define IMX_I2C1_PBASE           (IMX_AIPS2_PSECTION+IMX_I2C1_OFFSET)
#define IMX_I2C2_PBASE           (IMX_AIPS2_PSECTION+IMX_I2C2_OFFSET)
#define IMX_I2C3_PBASE           (IMX_AIPS2_PSECTION+IMX_I2C3_OFFSET)
#define IMX_ROMCP_PBASE          (IMX_AIPS2_PSECTION+IMX_ROMCP_OFFSET)
#define IMX_MMDC_PBASE           (IMX_AIPS2_PSECTION+IMX_MMDC_OFFSET)
#define IMX_MMDCP1_PBASE         (IMX_AIPS2_PSECTION+IMX_MMDCP1_OFFSET)
#define IMX_EIM_PBASE            (IMX_AIPS2_PSECTION+IMX_EIM_OFFSET)
#define IMX_OCOTPCTRL_PBASE      (IMX_AIPS2_PSECTION+IMX_OCOTPCTRL_OFFSET)
#define IMX_TZASC1_PBASE         (IMX_AIPS2_PSECTION+IMX_TZASC1_OFFSET)
#define IMX_TZASC2_PBASE         (IMX_AIPS2_PSECTION+IMX_TZASC2_OFFSET)
#define IMX_AUDMUX_PBASE         (IMX_AIPS2_PSECTION+IMX_AUDMUX_OFFSET)
#define IMX_MIPICSI_PBASE        (IMX_AIPS2_PSECTION+IMX_MIPICSI_OFFSET)
#define IMX_MIPIDSI_PBASE        (IMX_AIPS2_PSECTION+IMX_MIPIDSI_OFFSET)
#define IMX_VDOA_PBASE           (IMX_AIPS2_PSECTION+IMX_VDOA_OFFSET)
#define IMX_UART2_PBASE          (IMX_AIPS2_PSECTION+IMX_UART2_OFFSET)
#define IMX_UART3_PBASE          (IMX_AIPS2_PSECTION+IMX_UART3_OFFSET)
#define IMX_UART4_PBASE          (IMX_AIPS2_PSECTION+IMX_UART4_OFFSET)
#define IMX_UART5_PBASE          (IMX_AIPS2_PSECTION+IMX_UART5_OFFSET)

/* i.MX6 DAP AIPS-2 Physical Base Addresses */

#define IMX_DAPROM_PBASE         (IMX_AIPS2_PSECTION+IMX_DAPROM_OFFSET)
#define IMX_ETB_PBASE            (IMX_AIPS2_PSECTION+IMX_ETB_OFFSET)
#define IMX_EXTCTI_PBASE         (IMX_AIPS2_PSECTION+IMX_EXTCTI_OFFSET)
#define IMX_TPIU_PBASE           (IMX_AIPS2_PSECTION+IMX_TPIU_OFFSET)
#define IMX_FUNNEL_PBASE         (IMX_AIPS2_PSECTION+IMX_FUNNEL_OFFSET)
#define IMX_CA9INTEG_PBASE       (IMX_AIPS2_PSECTION+IMX_CA9INTEG_OFFSET)
#define IMX_CPUDBG_PBASE(n)      (IMX_AIPS2_PSECTION+IMX_CPUDBG_OFFSET(n))
#define IMX_CPUPMU_PBASE(n)      (IMX_AIPS2_PSECTION+IMX_CPUPMU_OFFSET(n))
#define IMX_CPU0DBG_PBASE        (IMX_AIPS2_PSECTION+IMX_CPU0DBG_OFFSET)
#define IMX_CPU0PMU_PBASE        (IMX_AIPS2_PSECTION+IMX_CPU0PMU_OFFSET)
#define IMX_CPU1DBG_PBASE        (IMX_AIPS2_PSECTION+IMX_CPU1DBG_OFFSET)
#define IMX_CPU1PMC_PBASE        (IMX_AIPS2_PSECTION+IMX_CPU1PMC_OFFSET)
#define IMX_CPU2DBG_PBASE        (IMX_AIPS2_PSECTION+IMX_CPU2DBG_OFFSET)
#define IMX_CPU2PMU_PBASE        (IMX_AIPS2_PSECTION+IMX_CPU2PMU_OFFSET)
#define IMX_CPU3DBG_PBASE        (IMX_AIPS2_PSECTION+IMX_CPU3DBG_OFFSET)
#define IMX_CPU3PMU_PBASE        (IMX_AIPS2_PSECTION+IMX_CPU3PMU_OFFSET)
#define IMX_CTI_PBASE(n)         (IMX_AIPS2_PSECTION+IMX_CTI_OFFSET(n))
#define IMX_CTI0_PBASE           (IMX_AIPS2_PSECTION+IMX_CTI0_OFFSET)
#define IMX_CTI1_PBASE           (IMX_AIPS2_PSECTION+IMX_CTI1_OFFSET)
#define IMX_CTI2_PBASE           (IMX_AIPS2_PSECTION+IMX_CTI2_OFFSET)
#define IMX_CTI3_PBASE           (IMX_AIPS2_PSECTION+IMX_CTI3_OFFSET)
#define IMX_PTM_PBASE(n)         (IMX_AIPS2_PSECTION+IMX_PTM_OFFSET(n))
#define IMX_PTM0_PBASE           (IMX_AIPS2_PSECTION+IMX_PTM0_OFFSET)
#define IMX_PTM1_PBASE           (IMX_AIPS2_PSECTION+IMX_PTM1_OFFSET)
#define IMX_PTM2_PBASE           (IMX_AIPS2_PSECTION+IMX_PTM2_OFFSET)
#define IMX_PTM3_PBASE           (IMX_AIPS2_PSECTION+IMX_PTM3_OFFSET)
#define IMX_PLATCTRL_PBASE       (IMX_AIPS2_PSECTION+IMX_PLATCTRL_OFFSET)

/* i.MX6 SATA Physical Base Addresses */

#define IMX_SATA_PBASE           (IMX_SATA_PSECTION+IMX_SATA_OFFSET)
#define IMX_OPENVG_PBASE         (IMX_SATA_PSECTION+IMX_OPENVG_OFFSET)
#define IMX_MIPIHSI_PBASE        (IMX_SATA_PSECTION+IMX_MIPIHSI_OFFSET)

/* Sizes of memory regions in bytes.
 *
 * These sizes exclude the undefined addresses at the end of the memory
 * region.  The implemented sizes of the EBI CS0-3 and DDRCS regions
 * are not known apriori and must be specified with configuration settings.
 */

#define IMX_ROMCP_SECSIZE          (96*1024) /* 00000000-00017fff  96 KB Boot ROM (ROMCP) */
                                             /* 00018000-000fffff 928 KB Reserved */
#define IMX_DMA_SECSIZE          (1024*1024) /* 00100000-001fffff   1 MB See offsets below */
#define IMX_GPV2_SECSIZE         (1024*1024) /* 00200000-002fffff   1 MB GPV_2 PL301 (per1) configuration port */
#define IMX_GPV3_SECSIZE         (1024*1024) /* 00300000-003fffff   1 MB GPV_3 PL301 (per2) configuration port */
                                             /* 00400000-007fffff   4 MB Reserved */
#define IMX_GPV4_SECSIZE         (1024*1024) /* 00800000-008fffff   1 MB GPV_4 PL301 (fast3) configuration port */
#define IMX_OCRAM_SECSIZE        (1024*1024) /* 00900000-009fffff   1 MB OCRAM section size */
#define IMX_ARMMP_SECSIZE           (8*1024) /* 00a00000-00afffff   8 KB ARM MP */
#define IMX_GPV0PL301_SECSIZE    (1024*1024) /* 00b00000-00bfffff   1 MB GPV0 PL301 (fast2) configuration port */
#define IMX_GPV1PL301_SECSIZE    (1024*1024) /* 00c00000-00cfffff   1 MB GPV1 PL301 (fast1) configuration port */
                                             /* 00d00000-00ffffff 3072 KB Reserved */
#define IMX_PCIE_SECSIZE      (16*1024*1024) /* 01000000-01ffffff  16 MB PCIe */
#define IMX_AIPS1_SECSIZE        (1024*1024) /* 02000000-020fffff   1 MB Peripheral IPs via AIPS-1 */
#define IMX_AIPS2_SECSIZE        (1024*1024) /* 02100000-021fffff   1 MB Peripheral IPs via AIPS-2 */
#define IMX_SATA_SECSIZE           (48*1024) /* 02200000-0220bfff  48 KB SATA */
                                             /* 0220c000-023fffff   2 MB Reserved */
#define IMX_IPU1_SECSIZE       (4*1024*1024) /* 02600000-029fffff   4 MB IPU-1 */
#define IMX_IPU2_SECSIZE       (4*1024*1024) /* 02a00000-02dfffff   4 MB IPU-2 */
#define IMX_EIM_SECSIZE     MKULONG(CONFIG_IMX_EIM_SIZE) /* 08000000-0fffffff 128 MB EIM - (NOR/SRAM) */
#define IMX_MMDCDDR_SECSIZE MKULONG(CONFIG_IMX_DDR_SIZE) /* 10000000-ffffffff 3840 MB MMDC-DDR Controller */
                                             /* 10000000-7fffffff 1792 MB */

/* Convert size in bytes to number of sections (in Mb). */

#define _NSECTIONS(b)            (((b)+0x000fffff) >> 20)

/* Sizes of memory regions in sections.
 *
 * The boot logic in sam_boot.c, will select 1Mb level 1 MMU mappings to
 * span the entire physical address space.  The definitions below specify
 * the number of 1Mb entries that are required to span a particular address
 * region.
 */

#define IMX_ROMCP_NSECTIONS      _NSECTIONS(IMX_ROMCP_SECSIZE)
#define IMX_DMA_NSECTIONS        _NSECTIONS(IMX_DMA_SECSIZE)
#define IMX_GPV2_NSECTIONS       _NSECTIONS(IMX_GPV2_SECSIZE)
#define IMX_GPV3_NSECTIONS       _NSECTIONS(IMX_GPV3_SECSIZE)
#define IMX_GPV4_NSECTIONS       _NSECTIONS(IMX_GPV4_SECSIZE)
#define IMX_OCRAM_NSECTIONS      _NSECTIONS(IMX_OCRAM_SECSIZE)
#define IMX_ARMMP_NSECTIONS      _NSECTIONS(IMX_ARMMP_SECSIZE)
#define IMX_GPV0PL301_NSECTIONS  _NSECTIONS(IMX_GPV0PL301_SECSIZE)
#define IMX_GPV1PL301_NSECTIONS  _NSECTIONS(IMX_GPV1PL301_SECSIZE)
#define IMX_PCIE_NSECTIONS       _NSECTIONS(IMX_PCIE_SECSIZE)
#define IMX_AIPS1_NSECTIONS      _NSECTIONS(IMX_AIPS1_SECSIZE)
#define IMX_AIPS2_NSECTIONS      _NSECTIONS(IMX_AIPS2_SECSIZE)
#define IMX_SATA_NSECTIONS       _NSECTIONS(IMX_SATA_SECSIZE)
#define IMX_IPU1_NSECTIONS       _NSECTIONS(IMX_IPU1_SECSIZE)
#define IMX_IPU2_NSECTIONS       _NSECTIONS(IMX_IPU2_SECSIZE)
#define IMX_EIM_NSECTIONS        _NSECTIONS(IMX_EIM_SECSIZE)
#define IMX_MMDCDDR_NSECTIONS    _NSECTIONS(IMX_MMDCDDR_SECSIZE)

/* Section MMU Flags
 *
 * SDRAM is a special case because it requires non-cached access of its
 * initial configuration, then cached access thereafter.
 */

#define IMX_ROMCP_MMUFLAGS       MMU_ROMFLAGS
#define IMX_DMA_MMUFLAGS         MMU_IOFLAGS
#define IMX_GPV2_MMUFLAGS        MMU_IOFLAGS
#define IMX_GPV3_MMUFLAGS        MMU_IOFLAGS
#define IMX_GPV4_MMUFLAGS        MMU_IOFLAGS
#define IMX_OCRAM_MMUFLAGS       MMU_MEMFLAGS
#define IMX_ARMMP_MMUFLAGS       MMU_IOFLAGS
#define IMX_GPV0PL301_MMUFLAGS   MMU_IOFLAGS
#define IMX_GPV1PL301_MMUFLAGS   MMU_IOFLAGS
#define IMX_PCIE_MMUFLAGS        MMU_IOFLAGS
#define IMX_AIPS1_MMUFLAGS       MMU_IOFLAGS
#define IMX_AIPS2_MMUFLAGS       MMU_IOFLAGS
#define IMX_SATA_MMUFLAGS        MMU_IOFLAGS
#define IMX_IPU1_MMUFLAGS        MMU_IOFLAGS
#define IMX_IPU2_MMUFLAGS        MMU_IOFLAGS
#define IMX_EIM_MMUFLAGS         MMU_ROMFLAGS /* REVISIT */
#define IMX_MMDCDDR_MMUFLAGS     MMU_MEMFLAGS

/* i.MX6 Virtual (mapped) Memory Map
 *
 * board_memorymap.h contains special mappings that are needed when a ROM
 * memory map is used.  It is included in this odd location becaue it depends
 * on some the virtual address definitions provided above.
 */

#include <arch/board/board_memorymap.h>

/* i.MX6 Virtual (mapped) Memory Map.  These are the mappings that will
 * be created if the page table lies in RAM.  If the platform has another,
 * read-only, pre-initialized page table (perhaps in ROM), then the board.h
 * file must provide these definitions.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE

/* The default mappings are a simple 1-to-1 mapping */

#  define IMX_ROMCP_VSECTION     IMX_ROMCP_PSECTION        /*  96 KB Boot ROM (ROMCP) */
#  define IMX_DMA_VSECTION       IMX_DMA_PSECTION          /*   1 MB See offsets below */
#  define IMX_GPV2_VSECTION      IMX_GPV2_PSECTION         /*   1 MB GPV_2 PL301 (per1) configuration port */
#  define IMX_GPV3_VSECTION      IMX_GPV3_PSECTION         /*   1 MB GPV_3 PL301 (per2) configuration port */
#  define IMX_GPV4_VSECTION      IMX_GPV4_PSECTION         /*   1 MB GPV_4 PL301 (fast3) configuration port */
#  define IMX_OCRAM_VSECTION     IMX_OCRAM_PSECTION        /*   1 MB OCRAM */
#  define IMX_ARMMP_VSECTION     IMX_ARMMP_PSECTION        /*   8 KB ARM MP */
#  define IMX_GPV0PL301_VSECTION IMX_GPV0PL301_PSECTION    /*   1 MB GPV0 PL301 (fast2) configuration port */
#  define IMX_GPV1PL301_VSECTION IMX_GPV1PL301_PSECTION    /*   1 MB GPV1 PL301 (fast1) configuration port */
#  define IMX_PCIE_VSECTION      IMX_PCIE_PSECTION         /*  16 MB PCIe */
#  define IMX_AIPS1_VSECTION     IMX_AIPS1_PSECTION        /*   1 MB Peripheral IPs via AIPS-1 */
#  define IMX_AIPS2_VSECTION     IMX_AIPS2_PSECTION        /*   1 MB Peripheral IPs via AIPS-2 */
#  define IMX_SATA_VSECTION      IMX_SATA_PSECTION         /*  48 KB SATA */
#  define IMX_IPU1_VSECTION      IMX_IPU1_PSECTION         /*   4 MB IPU-1 */
#  define IMX_IPU2_VSECTION      IMX_IPU2_PSECTION         /*   4 MB IPU-2 */
#  define IMX_EIM_VSECTION       IMX_EIM_PSECTION          /* 128 MB EIM - (NOR/SRAM) */
#  define IMX_MMDCDDR_VSECTION   IMX_MMDCDDR_PSECTION      /* 3840 MB MMDC-DDR Controller */
#endif /* CONFIG_ARCH_ROMPGTABLE */

/* i.MX6 DMA Virtual Base Addresses */

#define IMX_CAAMRAM_VBASE        (IMX_DMA_VSECTION+IMX_CAAMRAM_OFFSET)
#define IMX_APBHDMA_VBASE        (IMX_DMA_VSECTION+IMX_APBHDMA_OFFSET)
#define IMX_GPMI_VBASE           (IMX_DMA_VSECTION+IMX_GPMI_OFFSET)
#define IMX_BCH_VBASE            (IMX_DMA_VSECTION+IMX_BCH_OFFSET)
#define IMX_HDMI_VBASE           (IMX_DMA_VSECTION+IMX_HDMI_OFFSET)
#define IMX_GPU3D_VBASE          (IMX_DMA_VSECTION+IMX_GPU3D_OFFSET)
#define IMX_GPU2D_VBASE          (IMX_DMA_VSECTION+IMX_GPU2D_OFFSET)
#define IMX_DTCP_VBASE           (IMX_DMA_VSECTION+IMX_DTCP_OFFSET)

/* i.MX6 OCRAM Virtual Base Addresses */

#define IMX_OCRAM_VBASE          (IMX_OCRAM_VSECTION+IMX_OCRAM_OFFSET)
#define IMX_OCRAMALIAS_VBASE     (IMX_OCRAM_VSECTION+IMX_OCRAMALIAS_OFFSET)

/* i.MX6 ARM MP Virtual Base Addresses */

#define IMX_MPSCU_VBASE          (IMX_ARMMP_VSECTION+IMX_MPSCU_OFFSET)
#define IMX_MPICC_VBASE          (IMX_ARMMP_VSECTION+IMX_MPICC_OFFSET)
#define IMX_MPGTM_VBASE          (IMX_ARMMP_VSECTION+IMX_MPGTM_OFFSET)
#define IMX_MPPTM_VBASE          (IMX_ARMMP_VSECTION+IMX_MPPTM_OFFSET)
#define IMX_MPICD_VBASE          (IMX_ARMMP_VSECTION+IMX_MPICD_OFFSET)
#define IMX_PL310_VBASE          (IMX_ARMMP_VSECTION+IMX_PL310_OFFSET)

/* i.MX6 PCIE Virtual Base Addresses */

#define IMX_PCIE_VBASE           (IMX_PCIE_VSECTION+IMX_PCIE_OFFSET)
#define IMX_PCIEREGS_VBASE       (IMX_PCIE_VSECTION+IMX_PCIEREGS_OFFSET)

/* i.MX6 AIPS-1 Virtual Base Addresses */

#define IMX_SPDIF_VBASE          (IMX_AIPS1_VSECTION+IMX_SPDIF_OFFSET)
#define IMX_ECSPI1_VBASE         (IMX_AIPS1_VSECTION+IMX_ECSPI1_OFFSET)
#define IMX_ECSPI2_VBASE         (IMX_AIPS1_VSECTION+IMX_ECSPI2_OFFSET)
#define IMX_ECSPI3_VBASE         (IMX_AIPS1_VSECTION+IMX_ECSPI3_OFFSET)
#define IMX_ECSPI4_VBASE         (IMX_AIPS1_VSECTION+IMX_ECSPI4_OFFSET)
#define IMX_ECSPI5_VBASE         (IMX_AIPS1_VSECTION+IMX_ECSPI5_OFFSET)
#define IMX_UART1_VBASE          (IMX_AIPS1_VSECTION+IMX_UART1_OFFSET)
#define IMX_ESAI_VBASE           (IMX_AIPS1_VSECTION+IMX_ESAI_OFFSET)
#define IMX_SSI1_VBASE           (IMX_AIPS1_VSECTION+IMX_SSI1_OFFSET)
#define IMX_SSI2_VBASE           (IMX_AIPS1_VSECTION+IMX_SSI2_OFFSET)
#define IMX_SSI3_VBASE           (IMX_AIPS1_VSECTION+IMX_SSI3_OFFSET)
#define IMX_ASRC_VBASE           (IMX_AIPS1_VSECTION+IMX_ASRC_OFFSET)
#define IMX_SPBA_VBASE           (IMX_AIPS1_VSECTION+IMX_SPBA_OFFSET)
#define IMX_VPU_VBASE            (IMX_AIPS1_VSECTION+IMX_VPU_OFFSET)
#define IMX_APIS1CFG_VBASE       (IMX_AIPS1_VSECTION+IMX_APIS1CFG_OFFSET)
#define IMX_PWM1_VBASE           (IMX_AIPS1_VSECTION+IMX_PWM1_OFFSET)
#define IMX_PWM2_VBASE           (IMX_AIPS1_VSECTION+IMX_PWM2_OFFSET)
#define IMX_PWM3_VBASE           (IMX_AIPS1_VSECTION+IMX_PWM3_OFFSET)
#define IMX_PWM4_VBASE           (IMX_AIPS1_VSECTION+IMX_PWM4_OFFSET)
#define IMX_CAN1_VBASE           (IMX_AIPS1_VSECTION+IMX_CAN1_OFFSET)
#define IMX_CAN2_VBASE           (IMX_AIPS1_VSECTION+IMX_CAN2_OFFSET)
#define IMX_GPT_VBASE            (IMX_AIPS1_VSECTION+IMX_GPT_OFFSET)
#define IMX_GPIO_VBASE(n)        (IMX_AIPS1_VSECTION+IMX_GPIO_OFFSET(n))
#define IMX_GPIO1_VBASE          (IMX_AIPS1_VSECTION+IMX_GPIO1_OFFSET)
#define IMX_GPIO2_VBASE          (IMX_AIPS1_VSECTION+IMX_GPIO2_OFFSET)
#define IMX_GPIO3_VBASE          (IMX_AIPS1_VSECTION+IMX_GPIO3_OFFSET)
#define IMX_GPIO4_VBASE          (IMX_AIPS1_VSECTION+IMX_GPIO4_OFFSET)
#define IMX_GPIO5_VBASE          (IMX_AIPS1_VSECTION+IMX_GPIO5_OFFSET)
#define IMX_GPIO6_VBASE          (IMX_AIPS1_VSECTION+IMX_GPIO6_OFFSET)
#define IMX_GPIO7_VBASE          (IMX_AIPS1_VSECTION+IMX_GPIO7_OFFSET)
#define IMX_KPP_VBASE            (IMX_AIPS1_VSECTION+IMX_KPP_OFFSET)
#define IMX_WDOG1_VBASE          (IMX_AIPS1_VSECTION+IMX_WDOG1_OFFSET)
#define IMX_WDOG2_VBASE          (IMX_AIPS1_VSECTION+IMX_WDOG2_OFFSET)
#define IMX_CCM_VBASE            (IMX_AIPS1_VSECTION+IMX_CCM_OFFSET)
#define IMX_ANALOG_VBASE         (IMX_AIPS1_VSECTION+IMX_ANALOG_OFFSET)
#define IMX_USBPHY1_VBASE        (IMX_AIPS1_VSECTION+IMX_USBPHY1_OFFSET)
#define IMX_USBPHY2_VBASE        (IMX_AIPS1_VSECTION+IMX_USBPHY2_OFFSET)
#define IMX_SNVSHP_VBASE         (IMX_AIPS1_VSECTION+IMX_SNVSHP_OFFSET)
#define IMX_EPIT1_VBASE          (IMX_AIPS1_VSECTION+IMX_EPIT1_OFFSET)
#define IMX_EPIT2_VBASE          (IMX_AIPS1_VSECTION+IMX_EPIT2_OFFSET)
#define IMX_SRC_VBASE            (IMX_AIPS1_VSECTION+IMX_SRC_OFFSET)
#define IMX_GPC_VBASE            (IMX_AIPS1_VSECTION+IMX_GPC_OFFSET)
#define IMX_PGCPU_VBASE          (IMX_AIPS1_VSECTION+IMX_PGCPU_OFFSET)
#define IMX_PGCARM_VBASE         (IMX_AIPS1_VSECTION+IMX_PGCARM_OFFSET)
#define IMX_IOMUXC_VBASE         (IMX_AIPS1_VSECTION+IMX_IOMUXC_OFFSET)
#define IMX_DCIC1_VBASE          (IMX_AIPS1_VSECTION+IMX_DCIC1_OFFSET)
#define IMX_DCIC2_VBASE          (IMX_AIPS1_VSECTION+IMX_DCIC2_OFFSET)
#define IMX_SDMA_VBASE           (IMX_AIPS1_VSECTION+IMX_SDMA_OFFSET)

/* i.MX6 AIPS-2 Virtual Base Addresses */

#define IMX_CAAM_VBASE           (IMX_AIPS2_VSECTION+IMX_CAAM_OFFSET)
#define IMX_DAP_VBASE            (IMX_AIPS2_VSECTION+IMX_DAP_OFFSET)
#define IMX_AIPS2CGF_VBASE       (IMX_AIPS2_VSECTION+IMX_AIPS2CGF_OFFSET)
#define IMX_USBOH3_VBASE         (IMX_AIPS2_VSECTION+IMX_USBOH3_OFFSET)
#define IMX_ENET_VBASE           (IMX_AIPS2_VSECTION+IMX_ENET_OFFSET)
#define IMX_MLB150_VBASE         (IMX_AIPS2_VSECTION+IMX_MLB150_OFFSET)
#define IMX_USDHC1_VBASE         (IMX_AIPS2_VSECTION+IMX_USDHC1_OFFSET)
#define IMX_USDHC2_VBASE         (IMX_AIPS2_VSECTION+IMX_USDHC2_OFFSET)
#define IMX_USDHC3_VBASE         (IMX_AIPS2_VSECTION+IMX_USDHC3_OFFSET)
#define IMX_USDHC4_VBASE         (IMX_AIPS2_VSECTION+IMX_USDHC4_OFFSET)
#define IMX_I2C1_VBASE           (IMX_AIPS2_VSECTION+IMX_I2C1_OFFSET)
#define IMX_I2C2_VBASE           (IMX_AIPS2_VSECTION+IMX_I2C2_OFFSET)
#define IMX_I2C3_VBASE           (IMX_AIPS2_VSECTION+IMX_I2C3_OFFSET)
#define IMX_ROMCP_VBASE          (IMX_AIPS2_VSECTION+IMX_ROMCP_OFFSET)
#define IMX_MMDC_VBASE           (IMX_AIPS2_VSECTION+IMX_MMDC_OFFSET)
#define IMX_MMDCP1_VBASE         (IMX_AIPS2_VSECTION+IMX_MMDCP1_OFFSET)
#define IMX_EIM_VBASE            (IMX_AIPS2_VSECTION+IMX_EIM_OFFSET)
#define IMX_OCOTPCTRL_VBASE      (IMX_AIPS2_VSECTION+IMX_OCOTPCTRL_OFFSET)
#define IMX_TZASC1_VBASE         (IMX_AIPS2_VSECTION+IMX_TZASC1_OFFSET)
#define IMX_TZASC2_VBASE         (IMX_AIPS2_VSECTION+IMX_TZASC2_OFFSET)
#define IMX_AUDMUX_VBASE         (IMX_AIPS2_VSECTION+IMX_AUDMUX_OFFSET)
#define IMX_MIPICSI_VBASE        (IMX_AIPS2_VSECTION+IMX_MIPICSI_OFFSET)
#define IMX_MIPIDSI_VBASE        (IMX_AIPS2_VSECTION+IMX_MIPIDSI_OFFSET)
#define IMX_VDOA_VBASE           (IMX_AIPS2_VSECTION+IMX_VDOA_OFFSET)
#define IMX_UART2_VBASE          (IMX_AIPS2_VSECTION+IMX_UART2_OFFSET)
#define IMX_UART3_VBASE          (IMX_AIPS2_VSECTION+IMX_UART3_OFFSET)
#define IMX_UART4_VBASE          (IMX_AIPS2_VSECTION+IMX_UART4_OFFSET)
#define IMX_UART5_VBASE          (IMX_AIPS2_VSECTION+IMX_UART5_OFFSET)

/* i.MX6 DAP AIPS-2 Virtual Base Addresses */

#define IMX_DAPROM_VBASE         (IMX_AIPS2_VSECTION+IMX_DAPROM_OFFSET)
#define IMX_ETB_VBASE            (IMX_AIPS2_VSECTION+IMX_ETB_OFFSET)
#define IMX_EXTCTI_VBASE         (IMX_AIPS2_VSECTION+IMX_EXTCTI_OFFSET)
#define IMX_TPIU_VBASE           (IMX_AIPS2_VSECTION+IMX_TPIU_OFFSET)
#define IMX_FUNNEL_VBASE         (IMX_AIPS2_VSECTION+IMX_FUNNEL_OFFSET)
#define IMX_CA9INTEG_VBASE       (IMX_AIPS2_VSECTION+IMX_CA9INTEG_OFFSET)
#define IMX_CPUDBG_VBASE(n)      (IMX_AIPS2_VSECTION+IMX_CPUDBG_OFFSET(n))
#define IMX_CPUPMU_VBASE(n)      (IMX_AIPS2_VSECTION+IMX_CPUPMU_OFFSET(n))
#define IMX_CPU0DBG_VBASE        (IMX_AIPS2_VSECTION+IMX_CPU0DBG_OFFSET)
#define IMX_CPU0PMU_VBASE        (IMX_AIPS2_VSECTION+IMX_CPU0PMU_OFFSET)
#define IMX_CPU1DBG_VBASE        (IMX_AIPS2_VSECTION+IMX_CPU1DBG_OFFSET)
#define IMX_CPU1PMC_VBASE        (IMX_AIPS2_VSECTION+IMX_CPU1PMC_OFFSET)
#define IMX_CPU2DBG_VBASE        (IMX_AIPS2_VSECTION+IMX_CPU2DBG_OFFSET)
#define IMX_CPU2PMU_VBASE        (IMX_AIPS2_VSECTION+IMX_CPU2PMU_OFFSET)
#define IMX_CPU3DBG_VBASE        (IMX_AIPS2_VSECTION+IMX_CPU3DBG_OFFSET)
#define IMX_CPU3PMU_VBASE        (IMX_AIPS2_VSECTION+IMX_CPU3PMU_OFFSET)
#define IMX_CTI_VBASE(n)         (IMX_AIPS2_VSECTION+IMX_CTI_OFFSET(n))
#define IMX_CTI0_VBASE           (IMX_AIPS2_VSECTION+IMX_CTI0_OFFSET)
#define IMX_CTI1_VBASE           (IMX_AIPS2_VSECTION+IMX_CTI1_OFFSET)
#define IMX_CTI2_VBASE           (IMX_AIPS2_VSECTION+IMX_CTI2_OFFSET)
#define IMX_CTI3_VBASE           (IMX_AIPS2_VSECTION+IMX_CTI3_OFFSET)
#define IMX_PTM_VBASE(n)         (IMX_AIPS2_VSECTION+IMX_PTM_OFFSET(n))
#define IMX_PTM0_VBASE           (IMX_AIPS2_VSECTION+IMX_PTM0_OFFSET)
#define IMX_PTM1_VBASE           (IMX_AIPS2_VSECTION+IMX_PTM1_OFFSET)
#define IMX_PTM2_VBASE           (IMX_AIPS2_VSECTION+IMX_PTM2_OFFSET)
#define IMX_PTM3_VBASE           (IMX_AIPS2_VSECTION+IMX_PTM3_OFFSET)
#define IMX_PLATCTRL_VBASE       (IMX_AIPS2_VSECTION+IMX_PLATCTRL_OFFSET)

/* i.MX6 SATA Virtual Base Addresses */

#define IMX_SATA_VBASE           (IMX_SATA_VSECTION+IMX_SATA_OFFSET)
#define IMX_OPENVG_VBASE         (IMX_SATA_VSECTION+IMX_OPENVG_OFFSET)
#define IMX_MIPIHSI_VBASE        (IMX_SATA_VSECTION+IMX_MIPIHSI_OFFSET)

/* NuttX virtual base address
 *
 * The boot logic will create a temporarily mapping based on where NuttX is
 * executing in memory.  In this case, NuttX could be running from EIM NOR
 * FLASH, EIM SRAM, or MMDC SDRAM. If we are running from FLASH, then we
 * must have a separate mapping for the non-contiguous RAM region.
 */

#ifdef CONFIG_IMX6_BOOT_NOR

/* Some sanity checks.  If we are running from FLASH, then one of the
 * external chip selects must be configured to boot from NOR flash.
 * And, if so, then its size must agree with the configured size.
 */

#  if defined(CONFIG_IMX_EIM) && defined(CONFIG_IMX_EIM_NOR)
#    if CONFIG_IMX_EIM_SIZE != CONFIG_FLASH_SIZE
#      error EIM FLASH size disagreement
#    endif
#  else
#    error CONFIG_IMX6_BOOT_NOR=y, but no bootable NOR flash defined
#  endif

  /* Set up the NOR FLASH region as the NUTTX .text region */

#  define NUTTX_TEXT_VADDR       (CONFIG_FLASH_VSTART & 0xfff00000)
#  define NUTTX_TEXT_PADDR       (CONFIG_FLASH_START & 0xfff00000)
#  define NUTTX_TEXT_PEND        ((CONFIG_FLASH_END + 0x000fffff) & 0xfff00000)
#  define NUTTX_TEXT_SIZE        (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

  /* The "primary" RAM is the SDRAM or SRAM used for .bss and .data */

#  define NUTTX_RAM_VADDR        (CONFIG_RAM_VSTART & 0xfff00000)
#  define NUTTX_RAM_PADDR        (CONFIG_RAM_START & 0xfff00000)
#  define NUTTX_RAM_PEND         ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#  define NUTTX_RAM_SIZE         (NUTTX_RAM_PEND - NUTTX_RAM_PADDR)

#else /* CONFIG_IMX6_BOOT_NOR */
  /* Must be CONFIG_IMX6_BOOT_OCRAM || CONFIG_IMX6_BOOT_SDRAM || CONFIG_IMX6_BOOT_SRAM */

  /* Otherwise we are running from some kind of RAM (OCRAM, SRAM, or SDRAM).
   * Setup the RAM region as the NUTTX .txt, .bss, and .data region.
   */

#  define NUTTX_TEXT_VADDR       (CONFIG_RAM_VSTART & 0xfff00000)
#  define NUTTX_TEXT_PADDR       (CONFIG_RAM_START & 0xfff00000)
#  define NUTTX_TEXT_PEND        ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#  define NUTTX_TEXT_SIZE        (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

#endif /* CONFIG_IMX6_BOOT_NOR */

/* MMU Page Table Location
 *
 * Determine the address of the MMU page table.  We will always attempt to
 * use the bottom 16KB of RAM (SRAM or DRAM) for the page table, but there
 * are a few conditions that affect this:
 *
 * If CONFIG_ARCH_ROMPGTABLE, then the page table resides in ROM and we will
 * not use any page table in RAM, and in that case the user must sepcify the
 * address of the page table explicitly by defining PGTABLE_BASE_VADDR and
 * PGTABLE_BASE_PADDR in the board.h file.
 */

#undef PGTABLE_IN_HIGHSRAM
#undef PGTABLE_IN_LOWSRAM
#undef ARMV7A_PGTABLE_MAPPING /* We do not remap the page table */

/* Check if the user has configured the page table address */

#if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)

  /* Sanity check.. if one is undefined, both should be undefined */

#  if defined(PGTABLE_BASE_PADDR) || defined(PGTABLE_BASE_VADDR)
#    error "Only one of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is defined"
#  endif

  /* A sanity check, if the configuration says that the page table is read-only
   * and pre-initialized (maybe ROM), then it should have also defined both of
   * the page table base addresses.
   */

#  ifdef CONFIG_ARCH_ROMPGTABLE
#    error "CONFIG_ARCH_ROMPGTABLE defined; PGTABLE_BASE_P/VADDR not defined"
#  endif

  /* We must declare the page table at the bottom or at the top of OCRAM. */
  /* Yes.. do the vectors lie in low memory? */

#  ifdef CONFIG_ARCH_LOWVECTORS

  /* In this case, page table must lie at the top 16Kb of OCRAM. */

#    define PGTABLE_BASE_PADDR    (IMX_OCRAM_PBASE + IMX_OCRAM_SIZE - PGTABLE_SIZE)
#    define PGTABLE_BASE_VADDR    (IMX_OCRAM_VBASE + IMX_OCRAM_SIZE - PGTABLE_SIZE)
#    define PGTABLE_IN_HIGHSRAM   1

  /* We will force the IDLE stack to precede the page table */

#    define IDLE_STACK_PBASE      (PGTABLE_BASE_PADDR - CONFIG_IDLETHREAD_STACKSIZE)
#    define IDLE_STACK_VBASE      (PGTABLE_BASE_VADDR - CONFIG_IDLETHREAD_STACKSIZE)

#  else /* CONFIG_ARCH_LOWVECTORS */

  /* Otherwise, the vectors lie at another location (perhaps in NOR FLASH,
   * perhaps elsewhere in OCRAM).  The page table will then be positioned
   * at the first 16Kb of SRAM.
   */

#    define PGTABLE_BASE_PADDR    IMX_OCRAM_PBASE
#    define PGTABLE_BASE_VADDR    IMX_OCRAM_VBASE
#    define PGTABLE_IN_LOWSRAM    1

   /* We will force the IDLE stack to follow the page table */

#    define IDLE_STACK_PBASE      (PGTABLE_BASE_PADDR + PGTABLE_SIZE)
#    define IDLE_STACK_VBASE      (PGTABLE_BASE_VADDR + PGTABLE_SIZE)

#  endif /* CONFIG_ARCH_LOWVECTORS */

  /* In either case, the page table lies in OCRAM.  If OCRAM is not the
   * primary RAM region, then we will need to set-up a special mapping for
   * the page table at boot time.
   */

#  if defined(CONFIG_BOOT_RUNFROMFLASH)
  /* If we are running from FLASH, then the primary memory region is
   * given by NUTTX_RAM_PADDR.
   */

#    if NUTTX_RAM_PADDR != SAM_OCRAM_PSECTION
#      define ARMV7A_PGTABLE_MAPPING 1
#    endif

/* Otherwise, we are running from RAM and that RAM is also the primary
 * RAM.  If that is not OCRAM, then we will need to create a mapping
 * for the OCRAM at start-up.
 */

#  elif !defined(CONFIG_IMX6_BOOT_OCRAM)
#    define ARMV7A_PGTABLE_MAPPING 1
#  endif

#else /* !PGTABLE_BASE_PADDR || !PGTABLE_BASE_VADDR */

  /* Sanity check.. if one is defined, both should be defined */

#  if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)
#    error "One of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is undefined"
#  endif

  /* The page table then lies at the beginning of the OSSRAM and
   * the IDLE stack follows immediately.
   */

#    define PGTABLE_BASE_PADDR    IMX_OCRAM_PBASE
#    define PGTABLE_BASE_VADDR    IMX_OCRAM_VBASE
#    define PGTABLE_IN_LOWSRAM    1

   /* We will force the IDLE stack to follow the page table */

#    define IDLE_STACK_PBASE      (PGTABLE_BASE_PADDR + PGTABLE_SIZE)
#    define IDLE_STACK_VBASE      (PGTABLE_BASE_VADDR + PGTABLE_SIZE)

#endif /* !PGTABLE_BASE_PADDR || !PGTABLE_BASE_VADDR */

/* Level 2 Page table start addresses.
 *
 * The maximum size of the L1 page table is:
 *
 *  (4GB address range / 1 MB per section ) * 4 bytes per entry = 16KB
 *
 * The maximum size of the L2 page table is:
 *
 *  (4GB address range / 4 KB per page ) * 4 bytes per entry = 4MB
 *
 * 16KB of memory is reserved hold the page table for the virtual mappings.  A
 * portion of this table is not accessible in the virtual address space (for
 * normal operation with a one-to-one address mapping).   There is this large
 * hole in the physcal address space for which there will never be level 1
 * mappings:
 *
 *   0x80000000-0xefffffff: Undefined (1.75 GB)
 *
 * That is the offset where the main L2 page tables will be positioned.  This
 * corresponds to page table offsets 0x00002000 up to 0x00003c00.  That
 * is 1792 entries, each mapping 4KB of address for a total of 7MB of virtual
 * address space)
 *
 * Up to two L2 page tables may be used:
 *
 * 1) One mapping the vector table.  However, L2 page tables must be aligned
 *    to 1KB address boundaries, so the minimum L2 page table size is then
 *    1KB, mapping up a full megabyte of virtual address space.
 *
 *    This L2 page table is only allocated if CONFIG_ARCH_LOWVECTORS is *not*
 *    defined.  The i.MX6 boot-up logic will map the beginning of the boot
 *    memory to address 0x0000:0000 using both the MMU and the AXI matrix
 *    REMAP register.  So no L2 page table is required.
 *
 * 2) If on-demand paging is supported (CONFIG_PAGING=y), than an additional
 *    L2 page table is needed.  This page table will use the remainder of
 *    the address space.
 */

#ifndef CONFIG_ARCH_LOWVECTORS
  /* Memory map
   * VIRTUAL ADDRESS RANGE L1 PG TABLE L2 PG TABLE  DESCRIPTION
   * START      END        OFFSET      SIZE
   * ---------- ---------- ------------ ----------------------------
   * 0x80000000 0x803fffff 0x000002000 0x000000400  Vectors (1MiB)
   * 0x80100000 0x806fffff 0x000002400 0x000001800  Paging  (6MiB)
   */

  /* Vector L2 page table offset/size */

#  define VECTOR_L2_OFFSET        0x000002000
#  define VECTOR_L2_SIZE          0x000000400

  /* Vector L2 page table base addresses */

#  define VECTOR_L2_PBASE         (PGTABLE_BASE_PADDR + VECTOR_L2_OFFSET)
#  define VECTOR_L2_VBASE         (PGTABLE_BASE_VADDR + VECTOR_L2_OFFSET)

  /* Vector L2 page table end addresses */

#  define VECTOR_L2_END_PADDR     (VECTOR_L2_PBASE + VECTOR_L2_SIZE)
#  define VECTOR_L2_END_VADDR     (VECTOR_L2_VBASE + VECTOR_L2_SIZE)

  /* Paging L2 page table offset/size */

#  define PGTABLE_L2_OFFSET       0x000002400
#  define PGTABLE_L2_SIZE         0x000001800

#else
  /* Memory map
   * VIRTUAL ADDRESS RANGE L1 PG TABLE L2 PG TABLE  DESCRIPTION
   * START      END        OFFSET      SIZE
   * ---------- ---------- ------------ ----------------------------
   * 0x80000000 0x806fffff 0x000002000 0x000001c00  Paging  (7MiB)
   */

  /* Paging L2 page table offset/size */

#  define PGTABLE_L2_OFFSET       0x000002000
#  define PGTABLE_L2_SIZE         0x000001c00

#endif

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
 *   IMX_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
 *   IMX_VECTOR_VSRAM - Virtual address of vector table in SRAM
 *   IMX_VECTOR_VADDR - Virtual address of vector table (0x00000000 or 0xffff0000)
 */

#define VECTOR_TABLE_SIZE         0x00010000

/* REVISIT: These definitions are not used:  The vector table is at some
 * arbitrary (but aligned) position in RAM or NOR FLASH and is positioned
 * using the VBAR register.
 */

#ifdef CONFIG_ARCH_LOWVECTORS  /* Vectors located at 0x0000:0000  */
/* Vectors will always lie at the beginning of OCRAM
 *
 * OCRAM Memory Map:
 * ---------- ---------- ---------------------------
 * START      END        CONTENT
 * ---------- ---------- ---------------------------
 * 0x00000000 0x00010000 Vectors (VECTOR_TABLE_SIZE)
 * 0x00010000 0x0003c000 Unused
 * 0x0003c000 0x00004000 Page table (PGTABLE_SIZE)
 */

#  define IMX_VECTOR_PADDR        IMX_OCRAM_PBASE
#  define IMX_VECTOR_VSRAM        IMX_OCRAM_VBASE
#  define IMX_VECTOR_VADDR        0x00000000

#else  /* Vectors located at 0xffff:0000 -- this probably does not work */
/* OCRAM Memory Map:
 * ---------- ---------- ---------------------------
 * START      END        CONTENT
 * ---------- ---------- ---------------------------
 * 0x00000000 0x00004000 Page table (PGTABLE_SIZE)
 * 0x00004000 0x00030000 Unused
 * 0x00030000 0x00010000 Vectors (VECTOR_TABLE_SIZE)
 */

#  define IMX_VECTOR_PADDR        (IMX_OCRAM_PBASE + IMX_OCRAM_SIZE - VECTOR_TABLE_SIZE)
#  define IMX_VECTOR_VSRAM        (IMX_OCRAM_VBASE + IMX_OCRAM_SIZE - VECTOR_TABLE_SIZE)
#  define IMX_VECTOR_VADDR        0xffff0000

#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX6_CHIP_IMX_MEMORYMAP_H */
