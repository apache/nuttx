/****************************************************************************
 * arch/arm64/include/zynq-mpsoc/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM64_INCLUDE_ZYNQ_MPSOC_IRQ_H
#define __ARCH_ARM64_INCLUDE_ZYNQ_MPSOC_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Rockchip ZYNQ_MPSOC Interrupts */

#define NR_IRQS                        166 /* Total number of interrupts    */

#define ZYNQ_MPSOC_IRQ_RPU0_PERF_MON   40  /*  RPU0 performance monitor IRQ */
#define ZYNQ_MPSOC_IRQ_RPU1_PERF_MON   41  /*  RPU1 performance monitorIRQ  */
#define ZYNQ_MPSOC_IRQ_OCM_ECC_ERROR   42  /*  OCM CE or UE ECC error IRQ   */
#define ZYNQ_MPSOC_IRQ_LPD_APB_ERROR   43  /*  APB slave port error IRQ     */
#define ZYNQ_MPSOC_IRQ_RPU0_ECC_ERROR  44  /*  RPU0 ECC error IRQ           */
#define ZYNQ_MPSOC_IRQ_RPU1_ECC_ERROR  45  /*  RPU0 ECC error IRQ           */
#define ZYNQ_MPSOC_IRQ_NAND            46  /*  NAND controller IRQ          */
#define ZYNQ_MPSOC_IRQ_QSPI            47  /*  QSPI controller IRQ          */
#define ZYNQ_MPSOC_IRQ_GPIO            48  /*  GPIO controller IRQ          */
#define ZYNQ_MPSOC_IRQ_I2C0            49  /*  I2C0 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_I2C1            50  /*  I2C1 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_SPI0            51  /*  SPI0 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_SPI1            52  /*  SPI1 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_UART0           53  /*  UART0 controller IRQ         */
#define ZYNQ_MPSOC_IRQ_UART1           54  /*  UART1 controller IRQ         */
#define ZYNQ_MPSOC_IRQ_CAN0            55  /*  CAN0 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_CAN1            56  /*  CAN1 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_LPD_APM         57  /*  LPD and OCM APM IRQ          */
#define ZYNQ_MPSOC_IRQ_RTC_ALARM       58  /*  RTC alarm IRQ                */
#define ZYNQ_MPSOC_IRQ_RTC_SECONDS     59  /*  RTC sceond IRQ               */
#define ZYNQ_MPSOC_IRQ_CLKMON          60  /*  LPD clock test IRQ           */
#define ZYNQ_MPSOC_IRQ_IPI_CH7         61  /*  IPI channel 7 IRQ            */
#define ZYNQ_MPSOC_IRQ_IPI_CH8         62  /*  IPI channel 8 IRQ            */
#define ZYNQ_MPSOC_IRQ_IPI_CH9         63  /*  IPI channel 9 IRQ            */
#define ZYNQ_MPSOC_IRQ_IPI_CH10        64  /*  IPI channel 10 IRQ           */
#define ZYNQ_MPSOC_IRQ_IPI_CH2         65  /*  IPI channel 2 IRQ            */
#define ZYNQ_MPSOC_IRQ_IPI_CH1         66  /*  IPI channel 1 IRQ            */
#define ZYNQ_MPSOC_IRQ_IPI_CH0         67  /*  IPI channel 0 IRQ            */
#define ZYNQ_MPSOC_IRQ_TTC0            68  /*  TTC0 timer IRQ               */
#define ZYNQ_MPSOC_IRQ_TTC1            71  /*  TTC1 timer IRQ               */
#define ZYNQ_MPSOC_IRQ_TTC2            74  /*  TTC2 timer IRQ               */
#define ZYNQ_MPSOC_IRQ_TTC3            77  /*  TTC3 timer IRQ               */
#define ZYNQ_MPSOC_IRQ_SDIO0           80  /*  SDIO0 controller IRQ         */
#define ZYNQ_MPSOC_IRQ_SDIO1           81  /*  SDIO1 controller IRQ         */
#define ZYNQ_MPSOC_IRQ_SDIO0_WAKEUP    82  /*  SDIO0 controller wakeup IRQ  */
#define ZYNQ_MPSOC_IRQ_SDIO1_WAKEUP    83  /*  SDIO1 controller wakeup IRQ  */
#define ZYNQ_MPSOC_IRQ_LPD_SWDT        84  /*  LPD watch dog IRQ            */
#define ZYNQ_MPSOC_IRQ_CSU_SWDT        85  /*  CSU watch dog IRQ            */
#define ZYNQ_MPSOC_IRQ_LPD_ATB         86  /*  LPD ATB IRQ                  */
#define ZYNQ_MPSOC_IRQ_AIB             87  /*  AIB IRQ                      */
#define ZYNQ_MPSOC_IRQ_SYSMON          88  /*  interrupt monitor IRQ        */
#define ZYNQ_MPSOC_IRQ_GEM0            89  /*  GEM0 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_GEM0_WAKEUP     90  /*  GEM0 controller wakeup IRQ   */
#define ZYNQ_MPSOC_IRQ_GEM1            91  /*  GEM1 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_GEM1_WAKEUP     92  /*  GEM1 controller wakeup IRQ   */
#define ZYNQ_MPSOC_IRQ_GEM2            93  /*  GEM2 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_GEM2_WAKEUP     94  /*  GEM2 controller wakeup IRQ   */
#define ZYNQ_MPSOC_IRQ_GEM3            95  /*  GEM3 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_GEM3_WAKEUP     96  /*  GEM3 controller wakeup IRQ   */
#define ZYNQ_MPSOC_IRQ_USB0_ENDPOINT   97  /*  USB0 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_USB0_OTG        101 /*  USB0 OTG controller IRQ      */
#define ZYNQ_MPSOC_IRQ_USB1_ENDPOINT   102 /*  USB1 controller IRQ          */
#define ZYNQ_MPSOC_IRQ_USB1_OTG        106 /*  USB1 OTG controller IRQ      */
#define ZYNQ_MPSOC_IRQ_USB0_WAKEUP     107 /*  USB0 controller wakeup IRQ   */
#define ZYNQ_MPSOC_IRQ_USB1_WAKEUP     108 /*  USB1 controller wakeup IRQ   */
#define ZYNQ_MPSOC_IRQ_LPD_DMA         109 /*  LPD DMA controller IRQ       */
#define ZYNQ_MPSOC_IRQ_CSU             117 /*  config and security IRQ      */
#define ZYNQ_MPSOC_IRQ_CSU_DMA         118 /*  CSU DMA controller IRQ       */
#define ZYNQ_MPSOC_IRQ_EFUSE           119 /*  eFuse controller IRQ         */

#define ZYNQ_MPSOC_IRQ_PCIE_MSI0       146 /*  PCIe MSI interrupt 0-31      */
#define ZYNQ_MPSOC_IRQ_PCIE_MSI1       147 /*  PCIe MSI interrupt 32-63     */
#define ZYNQ_MPSOC_IRQ_PCIE_INTX       148 /*  PCIe INTxIRQ                 */
#define ZYNQ_MPSOC_IRQ_PCIE_DMA        149 /*  PCIe DMA controller IRQ      */
#define ZYNQ_MPSOC_IRQ_PCIE_MSC        150 /*  PCIe MSC controller IRQ      */
#define ZYNQ_MPSOC_IRQ_DISPLAY         151 /*  display controller IRQ       */
#define ZYNQ_MPSOC_IRQ_FPD_APB         152
#define ZYNQ_MPSOC_IRQ_FPD_ATB         153
#define ZYNQ_MPSOC_IRQ_DPDMA           154 /*  display DMA controller IRQ   */
#define ZYNQ_MPSOC_IRQ_FPD_APM         155
#define ZYNQ_MPSOC_IRQ_FPD_DMA         156
#define ZYNQ_MPSOC_IRQ_GPU             164 /*  GPU controller IRQ           */
#define ZYNQ_MPSOC_IRQ_SATA            165 /*  SATA controller IRQ          */

#endif /* __ARCH_ARM64_INCLUDE_ZYNQ_MPSOC_IRQ_H */
