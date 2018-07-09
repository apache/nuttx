/****************************************************************************
 * arch/arm/include/imx6/irq.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* This file should never be included directed but, rather, only indirectly
 & through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_IMX6_IRQ_H
#define __ARCH_ARM_INCLUDE_IMX6_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Global Interrupt Controller (GIC) collects up to 128 interrupt
 * requests from all i.MX 6Dual/6Quad sources and provides an interface to
 * each of the CPU cores.
 *
 * The first 32 interrupts are used for interrupts that are private to the
 * CPUs interface. interrupts besides the private CPU are also hooked up to
 * the GIC in the same order.
 *
 * Each interrupt can be configured as a normal or a secure interrupt.
 * Software force registers and software priority masking are also
 * supported. The following table describes the ARM interrupt sources.
 */

/* Private Peripheral Interrupts (PPI) **************************************/
/* Each Cortex-A9 processor has private interrupts, ID0-ID15, that can only
 * be triggered by software. These interrupts are aliased so that there is
 * no requirement for a requesting Cortex-A9 processor to determine its own
 * CPU ID when it deals with SGIs. The priority of an SGI depends on the
 * value set by the receiving Cortex-A9 processor in the banked SGI priority
 * registers, not the priority set by the sending Cortex-A9 processor.
 */

#define IMX_IRQ_SGI0              0  /* Software Generated Interrupt (SGI) 0 */
#define IMX_IRQ_SGI1              1  /* Software Generated Interrupt (SGI) 1 */
#define IMX_IRQ_SGI2              2  /* Software Generated Interrupt (SGI) 2 */
#define IMX_IRQ_SGI3              3  /* Software Generated Interrupt (SGI) 3 */
#define IMX_IRQ_SGI4              4  /* Software Generated Interrupt (SGI) 4 */
#define IMX_IRQ_SGI5              5  /* Software Generated Interrupt (SGI) 5 */
#define IMX_IRQ_SGI6              6  /* Software Generated Interrupt (SGI) 6 */
#define IMX_IRQ_SGI7              7  /* Software Generated Interrupt (SGI) 7 */
#define IMX_IRQ_SGI8              8  /* Software Generated Interrupt (SGI) 8 */
#define IMX_IRQ_SGI9              9  /* Software Generated Interrupt (SGI) 9 */
#define IMX_IRQ_SGI10            10  /* Software Generated Interrupt (SGI) 10 */
#define IMX_IRQ_SGI11            11  /* Software Generated Interrupt (SGI) 11 */
#define IMX_IRQ_SGI12            12  /* Software Generated Interrupt (SGI) 12 */
#define IMX_IRQ_SGI13            13  /* Software Generated Interrupt (SGI) 13 */
#define IMX_IRQ_SGI14            14  /* Software Generated Interrupt (SGI) 14 */
#define IMX_IRQ_SGI15            15  /* Software Generated Interrupt (SGI) 15 */

#define IMX_IRQ_GTM              27  /* Global Timer (GTM) PPI(0) */
#define IMX_IRQ_FIQ              28  /* Fast Interrupt Request (nFIQ) PPI(1) */
#define IMX_IRQ_PTM              29  /* Private Timer (PTM) PPI(2) */
#define IMX_IRQ_WDT              30  /* Watchdog Timer (WDT) PPI(3) */
#define IMX_IRQ_IRQ              31  /* Interrupt Request (nIRQ) PPI(4) */

/* Shared Peripheral Interrupts (SPI) ***************************************/

#define IMX_IRQ_IOMUXC           32  /* General Purpose Register 1 from IOMUXC */
#define IMX_IRQ_DAP              33  /* Debug Access Port interrupt request */
#define IMX_IRQ_SDMA             34  /* SDMA interrupt request from all channels */
#define IMX_IRQ_VPU              35  /* JPEG codec interrupt request */
#define IMX_IRQ_SNVS             36  /* PMIC power off request */
#define IMX_IRQ_IPU              37  /* IPU error interrupt request */
#define IMX_IRQ_IPU1             38  /* IPU1 sync interrupt request */
#define IMX_IRQ_IPU2             39  /* IPU2 error interrupt request */
#define IMX_IRQ_IPU2             40  /* IPU2 sync interrupt request */
#define IMX_IRQ_GPU3D            41  /* GPU3D interrupt request */
#define IMX_IRQ_R2D              42  /* GPU2D R2D GPU2D general interrupt request */
#define IMX_IRQ_V2D              43  /* GPU2D V2D GPU2D(OpenVG) general interrupt request */
#define IMX_IRQ_VPU              44  /* VPU interrupt request */
#define IMX_IRQ_APBHDMA          45  /* APBH-Bridge-DMA channels 0-3 interrupts */
#define IMX_IRQ_EIM              46  /* EIM interrupt request */
#define IMX_IRQ_BCH              47  /* BCH operation complete interrupt */
#define IMX_IRQ_GPMI             48  /* GPMI operation timeout error interrupt */
#define IMX_IRQ_DTCP             49  /* DTCP interrupt request */
#define IMX_IRQ_VDOA             50  /* VDOA interrupt requests */
#define IMX_IRQ_SNVS             51  /* SRTC consolidated interrupt */
#define IMX_IRQ_SNVS             52  /* SRTC security interrupt */
#define IMX_IRQ_CSU              53  /* CSU interrupt request 1 */
#define IMX_IRQ_USDHC1           54  /* uSDHC1 interrupt request */
#define IMX_IRQ_USDHC2           55  /* uSDHC2 interrupt request */
#define IMX_IRQ_USDHC3           56  /* uSDHC3 interrupt request */
#define IMX_IRQ_USDHC4           57  /* uSDHC4 interrupt request */
#define IMX_IRQ_UART1            58  /* UART1 interrupt request */
#define IMX_IRQ_UART2            59  /* UART2 interrupt request */
#define IMX_IRQ_UART3            60  /* UART3 interrupt request */
#define IMX_IRQ_UART4            61  /* UART4 interrupt request */
#define IMX_IRQ_UART5            62  /* UART5 interrupt request */
#define IMX_IRQ_ECSPI1           63  /* eCSPI1 interrupt request */
#define IMX_IRQ_ECSPI2           64  /* eCSPI2 interrupt request */
#define IMX_IRQ_ECSPI3           65  /* eCSPI3 interrupt request */
#define IMX_IRQ_ECSPI4           66  /* eCSPI4 interrupt request */
#define IMX_IRQ_ECSPI5           67  /* eCSPI5 interrupt request */
#define IMX_IRQ_I2C1             68  /* I2C1 interrupt request */
#define IMX_IRQ_I2C2             69  /* I2C2 interrupt request */
#define IMX_IRQ_I2C3             70  /* I2C3 interrupt request */
#define IMX_IRQ_SATA             71  /* SATA interrupt request */
#define IMX_IRQ_USBHOST1         72  /* USB Host 1 interrupt request */
#define IMX_IRQ_USBHOST2         73  /* USB Host 2 interrupt request */
#define IMX_IRQ_USBHOST3         74  /* USB Host 3 interrupt request */
#define IMX_IRQ_USBOTG           75  /* USB OTG interrupt request */
#define IMX_IRQ_USBPHY0          76  /* UTMI0 interrupt request */
#define IMX_IRQ_USBPHY1          77  /* UTMI1 interrupt request */
#define IMX_IRQ_SSI1             78  /* SSI1 interrupt request */
#define IMX_IRQ_SSI2             79  /* SSI2 interrupt request */
#define IMX_IRQ_SSI3             80  /* SSI3 interrupt request */
#define IMX_IRQ_TEMP             81  /* Temperature Sensor interrupt request */
#define IMX_IRQ_ASRC             82  /* ASRC interrupt request */
#define IMX_IRQ_ESAI             83  /* ESAI interrupt request */
#define IMX_IRQ_SPDIF            84  /* SPDIF interrupt */
#define IMX_IRQ_MLB150ERR        85  /* MLB error interrupt request */
#define IMX_IRQ_PMUANREG         86  /* Brown out of analog regulators occurred */
#define IMX_IRQ_GPT              87  /* GPT interrupt lines */
#define IMX_IRQ_EPIT1            88  /* EPIT1 output compare interrupt */
#define IMX_IRQ_EPIT2            89  /* EPIT2 output compare interrupt */
#define IMX_IRQ_GPIO1_INT7       90  /* INT7 interrupt request */
#define IMX_IRQ_GPIO1_INT6       91  /* INT6 interrupt request */
#define IMX_IRQ_GPIO1_INT5       92  /* INT5 interrupt request */
#define IMX_IRQ_GPIO1_INT4       93  /* INT4 interrupt request */
#define IMX_IRQ_GPIO1_INT3       94  /* INT3 interrupt request */
#define IMX_IRQ_GPIO1_INT2       95  /* INT2 interrupt request */
#define IMX_IRQ_GPIO1_INT1       96  /* INT1 interrupt request */
#define IMX_IRQ_GPIO1_INT0       97  /* INT0 interrupt request */
#define IMX_IRQ_GPIO1_INT_0_15   98  /* GPIO1 signals 0-15 */
#define IMX_IRQ_GPIO1_INT_16_31  99  /* GPIO1 signals 16-31 */
#define IMX_IRQ_GPIO2_INT_0_15  100  /* GPIO2 signals 0-15 */
#define IMX_IRQ_GPIO2_INT_16_31 101  /* GPIO2 signals 16-31 */
#define IMX_IRQ_GPIO3_INT_0_15  102  /* GPIO3 signals 0-15 */
#define IMX_IRQ_GPIO3_INT_16_31 103  /* GPIO3 signals 16-31 */
#define IMX_IRQ_GPIO4_INT_0_15  104  /* GPIO4 signals 0-15 */
#define IMX_IRQ_GPIO4_INT_16_31 105  /* GPIO4 signals 16-31 */
#define IMX_IRQ_GPIO5_INT_0_15  106  /* GPIO5 signals 0-15 */
#define IMX_IRQ_GPIO5_INT_16_31 107  /* GPIO5 signals 16-31 */
#define IMX_IRQ_GPIO6_INT_0_15  108  /* GPIO6 signals 0-15 */
#define IMX_IRQ_GPIO6_INT_16_31 109  /* GPIO6 signals 16-31 */
#define IMX_IRQ_GPIO7_INT_0_15  110  /* GPIO7 signals 0-15 */
#define IMX_IRQ_GPIO7_INT_16_31 111  /* GPIO7 signals 16-31 */
#define IMX_IRQ_WDOG1           112  /* WDOG1 timer reset interrupt request */
#define IMX_IRQ_WDOG2           113  /* WDOG2 timer reset interrupt request */
#define IMX_IRQ_KPP             114  /* Key Pad interrupt request */
#define IMX_IRQ_PWM1            115  /* PWM1 interrupts */
#define IMX_IRQ_PWM2            116  /* PWM2 interrupts */
#define IMX_IRQ_PWM3            117  /* PWM3 interrupts */
#define IMX_IRQ_PWM4            118  /* PWM4 interrupts */
#define IMX_IRQ_CCM1            119  /* CCM interrupt request 1 */
#define IMX_IRQ_CCM2            120  /* CCM interrupt request 2 */
#define IMX_IRQ_GPC             121  /* GPC interrupt request 1 */
#define IMX_IRQ_RESERVED122     122  /* Reserved */
#define IMX_IRQ_SRC             123  /* SRC interrupt request */
#define IMX_IRQ_CPUL2           124  /* L2 interrupt request */
#define IMX_IRQ_CPUPAR          125  /* Parity Check error interrupt request */
#define IMX_IRQ_CPUPERF         126  /* Performance Unit interrupt */
#define IMX_IRQ_CPUCTI          127  /* CTI trigger outputs interrupt */
#define IMX_IRQ_SRC             128  /* CPU wdog interrupts (4x) out of SRC */
#define IMX_IRQ_RESERVED129     129  /* Reserved */
#define IMX_IRQ_RESERVED130     130  /* Reserved */
#define IMX_IRQ_RESERVED131     131  /* Reserved */
#define IMX_IRQ_MIPICSI1        132  /* CSI interrupt request 1 */
#define IMX_IRQ_MIPICSI2        133  /* CSI interrupt request 2 */
#define IMX_IRQ_MIPIDSI         134  /* DSI interrupt request */
#define IMX_IRQ_MIPIHSI         135  /* HSI interrupt request */
#define IMX_IRQ_SJC             136  /* SJC interrupt from General Purpose register */
#define IMX_IRQ_CAAM0           137  /* CAAM job ring 0 interrupt */
#define IMX_IRQ_CAAM1           138  /* CAAM job ring 1 interrupt */
#define IMX_IRQ_RESERVED139     139  /* Reserved */
#define IMX_IRQ_ASC1            140  /* ASC1 interrupt request */
#define IMX_IRQ_ASC2            141  /* ASC2 interrupt request */
#define IMX_IRQ_FLEXCAN1        142  /* FLEXCAN1 interrupt request */
#define IMX_IRQ_FLEXCAN2        143  /* FLEXCAN2 interrupt request */
#define IMX_IRQ_RESERVED144     144  /* Reserved */
#define IMX_IRQ_RESERVED145     145  /* Reserved */
#define IMX_IRQ_RESERVED146     146  /* Reserved */
#define IMX_IRQ_HDMIMSTR        147  /* HDMI master interrupt request */
#define IMX_IRQ_HDMICEC         148  /* HDMI CEC engine dedicated wake-up interrupt */
#define IMX_IRQ_MLB150_0_31     149  /* Channels [31:0] interrupt requests */
#define IMX_IRQ_ENET0           150  /* MAC 0 IRQ */
#define IMX_IRQ_ENET0TMR        151  /* MAC 0 1588 Timer interrupt request */
#define IMX_IRQ_PCIE1           152  /* PCIe interrupt request 1 (intd/msi_ctrl_int) */
#define IMX_IRQ_PCIE2           153  /* PCIe interrupt request 2 (intc) */
#define IMX_IRQ_PCIE3           154  /* PCIe interrupt request 3 (intb) */
#define IMX_IRQ_PCIE4           155  /* PCIe interrupt request 4 (inta) */
#define IMX_IRQ_DCIC1           156  /* DCIC1 interrupt requests */
#define IMX_IRQ_DCIC2           157  /* DCIC2 interrupt requests */
#define IMX_IRQ_MLB150_32_63    158  /* Channel[63:32] interrupt requests */
#define IMX_IRQ_PMUDIGREG       159  /* Brown out of digital regulators occurred */

#define NR_IRQS                 160  /* Total number of interrupts */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
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
#endif

#endif /* __ARCH_ARM_INCLUDE_IMX6_IRQ_H */

