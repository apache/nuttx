/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_icu.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_ICU_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_ICU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Request Enable Register 08 (IER08) */

/* Interrupt Request Enable/Disable(IENn) */

#define _00_ICU_IRQ0_DISABLE    (0x00u)
#define _01_ICU_IRQ0_ENABLE     (0x01u)
#define _00_ICU_IRQ1_DISABLE    (0x00u)
#define _02_ICU_IRQ1_ENABLE     (0x02u)
#define _00_ICU_IRQ2_DISABLE    (0x00u)
#define _04_ICU_IRQ2_ENABLE     (0x04u)
#define _00_ICU_IRQ3_DISABLE    (0x00u)
#define _08_ICU_IRQ3_ENABLE     (0x08u)
#define _00_ICU_IRQ4_DISABLE    (0x00u)
#define _10_ICU_IRQ4_ENABLE     (0x10u)
#define _00_ICU_IRQ5_DISABLE    (0x00u)
#define _20_ICU_IRQ5_ENABLE     (0x20u)
#define _00_ICU_IRQ6_DISABLE    (0x00u)
#define _40_ICU_IRQ6_ENABLE     (0x40u)
#define _00_ICU_IRQ7_DISABLE    (0x00u)
#define _80_ICU_IRQ7_ENABLE     (0x80u)

/* Interrupt Request Enable Register 09 (IER09) */

/* Interrupt Request Enable/Disable(IENn) */

#define _00_ICU_IRQ8_DISABLE      (0x00u)
#define _01_ICU_IRQ8_ENABLE       (0x01u)
#define _00_ICU_IRQ9_DISABLE      (0x00u)
#define _02_ICU_IRQ9_ENABLE       (0x02u)
#define _00_ICU_IRQ10_DISABLE     (0x00u)
#define _04_ICU_IRQ10_ENABLE      (0x04u)
#define _00_ICU_IRQ11_DISABLE     (0x00u)
#define _08_ICU_IRQ11_ENABLE      (0x08u)
#define _00_ICU_IRQ12_DISABLE     (0x00u)
#define _10_ICU_IRQ12_ENABLE      (0x10u)
#define _00_ICU_IRQ13_DISABLE     (0x00u)
#define _20_ICU_IRQ13_ENABLE      (0x20u)
#define _00_ICU_IRQ14_DISABLE     (0x00u)
#define _40_ICU_IRQ14_ENABLE      (0x40u)
#define _00_ICU_IRQ15_DISABLE     (0x00u)
#define _80_ICU_IRQ15_ENABLE      (0x80u)

/* Interrupt Source Priority Register n (IPRn) */

/* Interrupt Priority Level Select (IPR[3:0]) */

#define _00_ICU_PRIORITY_LEVEL0                 (0x00u)
#define _01_ICU_PRIORITY_LEVEL1                 (0x01u)
#define _02_ICU_PRIORITY_LEVEL2                 (0x02u)
#define _03_ICU_PRIORITY_LEVEL3                 (0x03u)
#define _04_ICU_PRIORITY_LEVEL4                 (0x04u)
#define _05_ICU_PRIORITY_LEVEL5                 (0x05u)
#define _06_ICU_PRIORITY_LEVEL6                 (0x06u)
#define _07_ICU_PRIORITY_LEVEL7                 (0x07u)
#define _08_ICU_PRIORITY_LEVEL8                 (0x08u)
#define _09_ICU_PRIORITY_LEVEL9                 (0x09u)
#define _0A_ICU_PRIORITY_LEVEL10                (0x0au)
#define _0B_ICU_PRIORITY_LEVEL11                (0x0bu)
#define _0C_ICU_PRIORITY_LEVEL12                (0x0cu)
#define _0D_ICU_PRIORITY_LEVEL13                (0x0du)
#define _0E_ICU_PRIORITY_LEVEL14                (0x0eu)
#define _0F_ICU_PRIORITY_LEVEL15                (0x0fu)

/* Fast Interrupt Set Register (FIR) */

/* Fast Interrupt Enable (FIEN) */

#define _0000_ICU_FAST_INTERRUPT_DISABLE        (0x0000u)
#define _8000_ICU_FAST_INTERRUPT_ENABLE         (0x8000u)

/* IRQ Control Register i (IRQCRi) (i = 0 to 15) */

/* IRQ Detection Sense Select (IRQMD[1:0]) */

#define _00_ICU_IRQ_EDGE_LOW_LEVEL              (0x00u)
#define _04_ICU_IRQ_EDGE_FALLING                (0x04u)
#define _08_ICU_IRQ_EDGE_RISING                 (0x08u)
#define _0C_ICU_IRQ_EDGE_BOTH                   (0x0cu)

/* IRQ Pin Digital Filter Enable Register 0 (IRQFLTE0) */

/* Digital Filter Enable (FLTEN0n) */

#define _00_ICU_IRQn_FILTER_DISABLE             (0x00u)
#define _01_ICU_IRQ0_FILTER_ENABLE              (0x01u)
#define _02_ICU_IRQ1_FILTER_ENABLE              (0x02u)
#define _04_ICU_IRQ2_FILTER_ENABLE              (0x04u)
#define _08_ICU_IRQ3_FILTER_ENABLE              (0x08u)
#define _10_ICU_IRQ4_FILTER_ENABLE              (0x10u)
#define _20_ICU_IRQ5_FILTER_ENABLE              (0x20u)
#define _40_ICU_IRQ6_FILTER_ENABLE              (0x40u)
#define _80_ICU_IRQ7_FILTER_ENABLE              (0x80u)

/* IRQ Pin Digital Filter Enable Register 1 (IRQFLTE1) */

/* Digital Filter Enable (FLTEN8~15) */

#define _01_ICU_IRQ8_FILTER_ENABLE              (0x01u)
#define _02_ICU_IRQ9_FILTER_ENABLE              (0x02u)
#define _04_ICU_IRQ10_FILTER_ENABLE             (0x04u)
#define _08_ICU_IRQ11_FILTER_ENABLE             (0x08u)
#define _10_ICU_IRQ12_FILTER_ENABLE             (0x10u)
#define _20_ICU_IRQ13_FILTER_ENABLE             (0x20u)
#define _40_ICU_IRQ14_FILTER_ENABLE             (0x40u)
#define _80_ICU_IRQ15_FILTER_ENABLE             (0x80u)

/* IRQ Pin Digital Filter Setting Register 0 (IRQFLTC0) */

/* IRQn Digital Filter Sampling Clock (FCLKSELn) */

#define _0000_ICU_IRQ0_FILTER_PCLK              (0x0000u)
#define _0001_ICU_IRQ0_FILTER_PCLK_8            (0x0001u)
#define _0002_ICU_IRQ0_FILTER_PCLK_32           (0x0002u)
#define _0003_ICU_IRQ0_FILTER_PCLK_64           (0x0003u)
#define _0000_ICU_IRQ1_FILTER_PCLK              (0x0000u)
#define _0004_ICU_IRQ1_FILTER_PCLK_8            (0x0004u)
#define _0008_ICU_IRQ1_FILTER_PCLK_32           (0x0008u)
#define _000C_ICU_IRQ1_FILTER_PCLK_64           (0x000cu)
#define _0000_ICU_IRQ2_FILTER_PCLK              (0x0000u)
#define _0010_ICU_IRQ2_FILTER_PCLK_8            (0x0010u)
#define _0020_ICU_IRQ2_FILTER_PCLK_32           (0x0020u)
#define _0030_ICU_IRQ2_FILTER_PCLK_64           (0x0030u)
#define _0000_ICU_IRQ3_FILTER_PCLK              (0x0000u)
#define _0040_ICU_IRQ3_FILTER_PCLK_8            (0x0040u)
#define _0080_ICU_IRQ3_FILTER_PCLK_32           (0x0080u)
#define _00C0_ICU_IRQ3_FILTER_PCLK_64           (0x00c0u)
#define _0000_ICU_IRQ4_FILTER_PCLK              (0x0000u)
#define _0100_ICU_IRQ4_FILTER_PCLK_8            (0x0100u)
#define _0200_ICU_IRQ4_FILTER_PCLK_32           (0x0200u)
#define _0300_ICU_IRQ4_FILTER_PCLK_64           (0x0300u)
#define _0000_ICU_IRQ5_FILTER_PCLK              (0x0000u)
#define _0400_ICU_IRQ5_FILTER_PCLK_8            (0x0400u)
#define _0800_ICU_IRQ5_FILTER_PCLK_32           (0x0800u)
#define _0C00_ICU_IRQ5_FILTER_PCLK_64           (0x0c00u)
#define _0000_ICU_IRQ6_FILTER_PCLK              (0x0000u)
#define _1000_ICU_IRQ6_FILTER_PCLK_8            (0x1000u)
#define _2000_ICU_IRQ6_FILTER_PCLK_32           (0x2000u)
#define _3000_ICU_IRQ6_FILTER_PCLK_64           (0x3000u)
#define _0000_ICU_IRQ7_FILTER_PCLK              (0x0000u)
#define _4000_ICU_IRQ7_FILTER_PCLK_8            (0x4000u)
#define _8000_ICU_IRQ7_FILTER_PCLK_32           (0x8000u)
#define _C000_ICU_IRQ7_FILTER_PCLK_64           (0xc000u)

/* IRQ Pin Digital Filter Setting Register 0 (IRQFLTC1) */

/* IRQn Digital Filter Sampling Clock (FCLKSEL8~15) */

#define _0000_ICU_IRQ8_FILTER_PCLK              (0x0000u)
#define _0001_ICU_IRQ8_FILTER_PCLK_8            (0x0001u)
#define _0002_ICU_IRQ8_FILTER_PCLK_32           (0x0002u)
#define _0003_ICU_IRQ8_FILTER_PCLK_64           (0x0003u)
#define _0000_ICU_IRQ9_FILTER_PCLK              (0x0000u)
#define _0004_ICU_IRQ9_FILTER_PCLK_8            (0x0004u)
#define _0008_ICU_IRQ9_FILTER_PCLK_32           (0x0008u)
#define _000C_ICU_IRQ9_FILTER_PCLK_64           (0x000cu)
#define _0000_ICU_IRQ10_FILTER_PCLK             (0x0000u)
#define _0010_ICU_IRQ10_FILTER_PCLK_8           (0x0010u)
#define _0020_ICU_IRQ10_FILTER_PCLK_32          (0x0020u)
#define _0030_ICU_IRQ10_FILTER_PCLK_64          (0x0030u)
#define _0000_ICU_IRQ11_FILTER_PCLK             (0x0000u)
#define _0040_ICU_IRQ11_FILTER_PCLK_8           (0x0040u)
#define _0080_ICU_IRQ11_FILTER_PCLK_32          (0x0080u)
#define _00C0_ICU_IRQ11_FILTER_PCLK_64          (0x00c0u)
#define _0000_ICU_IRQ12_FILTER_PCLK             (0x0000u)
#define _0100_ICU_IRQ12_FILTER_PCLK_8           (0x0100u)
#define _0200_ICU_IRQ12_FILTER_PCLK_32          (0x0200u)
#define _0300_ICU_IRQ12_FILTER_PCLK_64          (0x0300u)
#define _0000_ICU_IRQ13_FILTER_PCLK             (0x0000u)
#define _0400_ICU_IRQ13_FILTER_PCLK_8           (0x0400u)
#define _0800_ICU_IRQ13_FILTER_PCLK_32          (0x0800u)
#define _0C00_ICU_IRQ13_FILTER_PCLK_64          (0x0c00u)
#define _0000_ICU_IRQ14_FILTER_PCLK             (0x0000u)
#define _1000_ICU_IRQ14_FILTER_PCLK_8           (0x1000u)
#define _2000_ICU_IRQ14_FILTER_PCLK_32          (0x2000u)
#define _3000_ICU_IRQ14_FILTER_PCLK_64          (0x3000u)
#define _0000_ICU_IRQ15_FILTER_PCLK             (0x0000u)
#define _4000_ICU_IRQ15_FILTER_PCLK_8           (0x4000u)
#define _8000_ICU_IRQ15_FILTER_PCLK_32          (0x8000u)
#define _C000_ICU_IRQ15_FILTER_PCLK_64          (0xc000u)

/* NMI Pin Interrupt Control Register (NMICR) */

/* NMI Detection Set (NMIMD) */

#define _00_ICU_NMI_EDGE_FALLING                (0x00u)
#define _08_ICU_NMI_EDGE_RISING                 (0x08u)

/* NMI Pin Digital Filter Setting Register (NMIFLTC) */

/* NMI Digital Filter Sampling Clock (NFCLKSEL[1:0]) */

#define _00_ICU_NMI_FILTER_PCLK                 (0x00u)
#define _01_ICU_NMI_FILTER_PCLK_8               (0x01u)
#define _02_ICU_NMI_FILTER_PCLK_32              (0x02u)
#define _03_ICU_NMI_FILTER_PCLK_64              (0x03u)

/* EXDMAC Activation Peripheral Interrupt Select Register (SELEXDR) */

/* EXDMAC0 Activation Peripheral Interrupt Select (SELEXD0) */

#define _00_ICU_EXDMAC0_SLIBR144                (0x00u)
#define _01_ICU_EXDMAC0_SLIAR208                (0x01u)

/* EXDMAC1 Activation Peripheral Interrupt Select (SELEXD1) */

#define _00_ICU_EXDMAC1_SLIBR145                (0x00u)
#define _02_ICU_EXDMAC1_SLIAR209                (0x02u)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: r_icu_create
 *
 * Description:
 *   Initializes ICU
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_icu_create(void);

/****************************************************************************
 * Name: r_icu_irq8_start
 *
 * Description:
 *   Enables IRQ8
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_icu_irq8_start(void);

/****************************************************************************
 * Name: r_icu_irq8_stop
 *
 * Description:
 *   Disables IRQ8
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_icu_irq8_stop(void);

/****************************************************************************
 * Name: r_icu_irq9_start
 *
 * Description:
 *   Enables IRQ9
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_icu_irq9_start(void);

/****************************************************************************
 * Name: r_icu_irq9_stop
 *
 * Description:
 *   Disables IRQ9
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_icu_irq9_stop(void);

/****************************************************************************
 * Name: r_config_icu_software_start
 *
 * Description:
 *   Enable Software Interrupt
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_config_icu_software_start(void);

/****************************************************************************
 * Name: r_config_icu_softwareinterrupt_generate
 *
 * Description:
 *   Generate software interrupt
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_config_icu_softwareinterrupt_generate(void);

/****************************************************************************
 * Name: r_config_icu_software_stop
 *
 * Description:
 *   Disable S/W Interrupt
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_config_icu_software_stop(void);

/****************************************************************************
 * Name: r_config_icu_software2_start
 *
 * Description:
 *   Enable S/W Interrupt2
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_config_icu_software2_start(void);

/****************************************************************************
 * Name: r_config_icu_softwareinterrupt2_generate
 *
 * Description:
 *   Generate software interrupt 2
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_config_icu_softwareinterrupt2_generate(void);

/****************************************************************************
 * Name: r_config_icu_software2_stop
 *
 * Description:
 *   Disable software interrupt 2
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_config_icu_software2_stop(void);

/****************************************************************************
 * Name: r_icu_irqisfallingedge
 *
 * Description:
 *   Detect if falling edge interrupt is triggered
 *
 * Input Parameters:
 *   irqno - irq number
 *
 * Returned Value:
 *   1 is returned on success
 *
 ****************************************************************************/

uint8_t r_icu_irqisfallingedge(const uint8_t irq_no);

/****************************************************************************
 * Name: r_icu_irqsetfallingedge
 *
 * Description:
 *   Sets or unsets falling edge triggered
 *
 * Input Parameters:
 *   irqno - irq number
 *   set_f_edge - value that has to be set for falling edge
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_icu_irqsetfallingedge(const uint8_t irq_no, const uint8_t set_f_edge);

/****************************************************************************
 * Name: r_icu_irqsetfallingedge
 *
 * Description:
 *   Sets or unsets falling edge triggered
 *
 * Input Parameters:
 *   irqno - irq number
 *   set_r_edge - value that has to be set for rising edge
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_icu_irqsetrisingedge(const uint8_t irq_no, const uint8_t set_r_edge);

#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_ICU_H */
