/****************************************************************************
 * arch/arm64/include/bcm2711/irq.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_IRQ_H
#define __ARCH_ARM64_SRC_BCM2711_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NR_IRQS 216
#define MPID_TO_CORE(mpid) (((mpid) >> MPIDR_AFF0_SHIFT) & MPIDR_AFFLVL_MASK)

/* ARMC interrupts */

#define BCM_IRQ_ARMC_BASE 64
#define BCM_IRQ_ARMC(n) (BCM_IRQ_ARMC_BASE + n)

#define BCM_IRQ_ARMC_TIMER BCM_IRQ_ARMC(0)      /* Timer */
#define BCM_IRQ_ARMC_MAILBOX BCM_IRQ_ARMC(1)    /* Mailbox */
#define BCM_IRQ_ARMC_DOORBELL0 BCM_IRQ_ARMC(2)  /* Doorbell 0 */
#define BCM_IRQ_ARMC_DOORBELL1 BCM_IRQ_ARMC(3)  /* Doorbell 1 */
#define BCM_IRQ_ARMC_VPU0HALT BCM_IRQ_ARMC(4)   /* VPU 0 halted */
#define BCM_IRQ_ARMC_VPU1HALT BCM_IRQ_ARMC(5)   /* VPU 1 halted */
#define BCM_IRQ_ARMC_ARMADDRERR BCM_IRQ_ARMC(6) /* ARM address error */
#define BCM_IRQ_ARMC_ARMAXIERR BCM_IRQ_ARMC(7)  /* ARM AXI error */
#define BCM_IRQ_ARMC_SWI0 BCM_IRQ_ARMC(8)       /* Software interrupt 0 */
#define BCM_IRQ_ARMC_SWI1 BCM_IRQ_ARMC(9)       /* Software interrupt 1 */
#define BCM_IRQ_ARMC_SWI2 BCM_IRQ_ARMC(10)      /* Software interrupt 2 */
#define BCM_IRQ_ARMC_SWI3 BCM_IRQ_ARMC(11)      /* Software interrupt 3 */
#define BCM_IRQ_ARMC_SWI4 BCM_IRQ_ARMC(12)      /* Software interrupt 4 */
#define BCM_IRQ_ARMC_SWI5 BCM_IRQ_ARMC(13)      /* Software interrupt 5 */
#define BCM_IRQ_ARMC_SWI6 BCM_IRQ_ARMC(14)      /* Software interrupt 6 */
#define BCM_IRQ_ARMC_SWI7 BCM_IRQ_ARMC(15)      /* Software interrupt 7 */

/* VideoCore interrupts */

#define BCM_IRQ_VC_BASE 96
#define BCM_IRQ_VC(n) (BCM_IRQ_VC_BASE + n)

#define BCM_IRQ_VC_TIMER0 BCM_IRQ_VC(0)
#define BCM_IRQ_VC_TIMER1 BCM_IRQ_VC(1)
#define BCM_IRQ_VC_TIMER2 BCM_IRQ_VC(2)
#define BCM_IRQ_VC_TIMER3 BCM_IRQ_VC(3)
#define BCM_IRQ_VC_H2640 BCM_IRQ_VC(4)
#define BCM_IRQ_VC_H2641 BCM_IRQ_VC(5)
#define BCM_IRQ_VC_H2642 BCM_IRQ_VC(6)
#define BCM_IRQ_VC_JPEG BCM_IRQ_VC(7)
#define BCM_IRQ_VC_ISP BCM_IRQ_VC(8)
#define BCM_IRQ_VC_USB BCM_IRQ_VC(9)
#define BCM_IRQ_VC_V3D BCM_IRQ_VC(10)
#define BCM_IRQ_VC_TRANSPOSE BCM_IRQ_VC(11)
#define BCM_IRQ_VC_MCSYNC0 BCM_IRQ_VC(12)
#define BCM_IRQ_VC_MCSYNC1 BCM_IRQ_VC(13)
#define BCM_IRQ_VC_MCSYNC2 BCM_IRQ_VC(14)
#define BCM_IRQ_VC_MCSYNC3 BCM_IRQ_VC(15)
#define BCM_IRQ_VC_DMA0 BCM_IRQ_VC(16)
#define BCM_IRQ_VC_DMA1 BCM_IRQ_VC(17)
#define BCM_IRQ_VC_DMA2 BCM_IRQ_VC(18)
#define BCM_IRQ_VC_DMA3 BCM_IRQ_VC(19)
#define BCM_IRQ_VC_DMA4 BCM_IRQ_VC(20)
#define BCM_IRQ_VC_DMA5 BCM_IRQ_VC(21)
#define BCM_IRQ_VC_DMA6 BCM_IRQ_VC(22)
#define BCM_IRQ_VC_DMA7N8 BCM_IRQ_VC(23)
#define BCM_IRQ_VC_DMA9N10 BCM_IRQ_VC(24)
#define BCM_IRQ_VC_DMA11 BCM_IRQ_VC(25)
#define BCM_IRQ_VC_DMA12 BCM_IRQ_VC(26)
#define BCM_IRQ_VC_DMA13 BCM_IRQ_VC(27)
#define BCM_IRQ_VC_DMA14 BCM_IRQ_VC(28)
#define BCM_IRQ_VC_AUX BCM_IRQ_VC(29)
#define BCM_IRQ_VC_ARM BCM_IRQ_VC(30)
#define BCM_IRQ_VC_DMA15 BCM_IRQ_VC(31)
#define BCM_IRQ_VC_HDMICEC BCM_IRQ_VC(32)
#define BCM_IRQ_VC_HVS BCM_IRQ_VC(33)
#define BCM_IRQ_VC_RPIVID BCM_IRQ_VC(34)
#define BCM_IRQ_VC_SDC BCM_IRQ_VC(35)
#define BCM_IRQ_VC_DSI0 BCM_IRQ_VC(36)
#define BCM_IRQ_VC_PIXVLV2 BCM_IRQ_VC(37)
#define BCM_IRQ_VC_CAM0 BCM_IRQ_VC(38)
#define BCM_IRQ_VC_CAM1 BCM_IRQ_VC(39)
#define BCM_IRQ_VC_HDMI0 BCM_IRQ_VC(40)
#define BCM_IRQ_VC_HDMI1 BCM_IRQ_VC(41)
#define BCM_IRQ_VC_PIXVLV3 BCM_IRQ_VC(42)
#define BCM_IRQ_VC_SPIBSCSLV BCM_IRQ_VC(43)
#define BCM_IRQ_VC_DSI1 BCM_IRQ_VC(44)
#define BCM_IRQ_VC_PXLVLV0 BCM_IRQ_VC(45)
#define BCM_IRQ_VC_PXLVLV1N4 BCM_IRQ_VC(46)
#define BCM_IRQ_VC_CPR BCM_IRQ_VC(47)
#define BCM_IRQ_VC_SMI BCM_IRQ_VC(48)
#define BCM_IRQ_VC_GPIO0 BCM_IRQ_VC(49)
#define BCM_IRQ_VC_GPIO1 BCM_IRQ_VC(50)
#define BCM_IRQ_VC_GPIO2 BCM_IRQ_VC(51)
#define BCM_IRQ_VC_GPIO3 BCM_IRQ_VC(52)
#define BCM_IRQ_VC_I2C BCM_IRQ_VC(53)
#define BCM_IRQ_VC_SPI BCM_IRQ_VC(54)
#define BCM_IRQ_VC_PCMI2S BCM_IRQ_VC(55)
#define BCM_IRQ_VC_SDHOST BCM_IRQ_VC(56)
#define BCM_IRQ_VC_PL011UART BCM_IRQ_VC(57)
#define BCM_IRQ_VC_ETHPCIE BCM_IRQ_VC(58)
#define BCM_IRQ_VC_VEC BCM_IRQ_VC(59)
#define BCM_IRQ_VC_CPG BCM_IRQ_VC(60)
#define BCM_IRQ_VC_RNG BCM_IRQ_VC(61)
#define BCM_IRQ_VC_EMMC BCM_IRQ_VC(62)
#define BCM_IRQ_VC_ETHPCIESEC BCM_IRQ_VC(63)

/* TODO: what about PACTL_CS address section 6.2.4? */

/* ETH_PCIe interrupts */

#define BCM_IRQ_ETH_BASE 160
#define BCM_IRQ_ETH(n) (BCM_IRQ_ETH_BASE + n)

#define BCM_IRQ_ETH_AVS BCM_IRQ_ETH(9)
#define BCM_IRQ_ETH_PCIE0_INTA BCM_IRQ_ETH(15)
#define BCM_IRQ_ETH_PCIE0_INTB BCM_IRQ_ETH(16)
#define BCM_IRQ_ETH_PCIE0_INTC BCM_IRQ_ETH(17)
#define BCM_IRQ_ETH_PCIE0_INTD BCM_IRQ_ETH(18)
#define BCM_IRQ_ETH_PCIE0_MSI BCM_IRQ_ETH(20)
#define BCM_IRQ_ETH_GENET0_A BCM_IRQ_ETH(29)
#define BCM_IRQ_ETH_GENET0_B BCM_IRQ_ETH(30)
#define BCM_IRQ_ETH_USB0_XHCI0 BCM_IRQ_ETH(48)

#define BCM_IRQ_ARMLOCAL_IRQ_SOURCEN_MAILBOX_CORE2_IRQ (1 << 5)
#define BCM_IRQ_ARMLOCAL_IRQ_SOURCEN_MAILBOX_CORE3_IRQ (1 << 4)
#define BCM_IRQ_ARMLOCAL_IRQ_SOURCEN_CNT_V_IRQ (1 << 3)

#endif // __ARCH_ARM64_SRC_BCM2711_IRQ_H
