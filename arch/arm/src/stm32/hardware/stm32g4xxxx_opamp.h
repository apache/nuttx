/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_opamp.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_OPAMP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_OPAMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_OPAMP1_CSR_OFFSET     0x0000    /* OPAMP1 Control register */
#define STM32_OPAMP2_CSR_OFFSET     0x0004    /* OPAMP2 Control register */
#define STM32_OPAMP3_CSR_OFFSET     0x0008    /* OPAMP3 Control register */
#define STM32_OPAMP4_CSR_OFFSET     0x000c    /* OPAMP4 Control register */
#define STM32_OPAMP5_CSR_OFFSET     0x0010    /* OPAMP5 Control register */
#define STM32_OPAMP6_CSR_OFFSET     0x0014    /* OPAMP6 Control register */
#define STM32_OPAMP1_TCMR_OFFSET    0x0018    /* OPAMP1 timer controlled mode register */
#define STM32_OPAMP2_TCMR_OFFSET    0x001c    /* OPAMP2 timer controlled mode register */
#define STM32_OPAMP3_TCMR_OFFSET    0x0020    /* OPAMP3 timer controlled mode register */
#define STM32_OPAMP4_TCMR_OFFSET    0x0024    /* OPAMP4 timer controlled mode register */
#define STM32_OPAMP5_TCMR_OFFSET    0x0028    /* OPAMP5 timer controlled mode register */
#define STM32_OPAMP6_TCMR_OFFSET    0x002c    /* OPAMP6 timer controlled mode register */

/* Register Addresses *******************************************************/

#define STM32_OPAMP1_CSR            (STM32_OPAMP_BASE+STM32_OPAMP1_CSR_OFFSET)
#define STM32_OPAMP2_CSR            (STM32_OPAMP_BASE+STM32_OPAMP2_CSR_OFFSET)
#define STM32_OPAMP3_CSR            (STM32_OPAMP_BASE+STM32_OPAMP3_CSR_OFFSET)
#define STM32_OPAMP4_CSR            (STM32_OPAMP_BASE+STM32_OPAMP4_CSR_OFFSET)
#define STM32_OPAMP5_CSR            (STM32_OPAMP_BASE+STM32_OPAMP5_CSR_OFFSET)
#define STM32_OPAMP6_CSR            (STM32_OPAMP_BASE+STM32_OPAMP6_CSR_OFFSET)
#define STM32_OPAMP1_TCMR           (STM32_OPAMP_BASE+STM32_OPAMP1_TCMR_OFFSET)
#define STM32_OPAMP2_TCMR           (STM32_OPAMP_BASE+STM32_OPAMP2_TCMR_OFFSET)
#define STM32_OPAMP3_TCMR           (STM32_OPAMP_BASE+STM32_OPAMP3_TCMR_OFFSET)
#define STM32_OPAMP4_TCMR           (STM32_OPAMP_BASE+STM32_OPAMP4_TCMR_OFFSET)
#define STM32_OPAMP5_TCMR           (STM32_OPAMP_BASE+STM32_OPAMP5_TCMR_OFFSET)
#define STM32_OPAMP6_TCMR           (STM32_OPAMP_BASE+STM32_OPAMP6_TCMR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* OPAMP control and status register */

#define OPAMP_CSR_OPAMPEN           (1 << 0)                        /* Bit 0: OPAMP enable */
#define OPAMP_CSR_FORCE_VP          (1 << 1)                        /* Bit 1: FORCE_VP */
#define OPAMP_CSR_VPSEL_SHIFT       (2)                             /* Bits 2-3: OPAMP non inverting input selection */
#define OPAMP_CSR_VPSEL_MASK        (3 << OPAMP_CSR_VPSEL_SHIFT)
#  define OPAMP_CSR_VPSEL_VINP0     (0 << OPAMP_CSR_VPSEL_SHIFT)
#  define OPAMP_CSR_VPSEL_VINP1     (1 << OPAMP_CSR_VPSEL_SHIFT)
#  define OPAMP_CSR_VPSEL_VINP2     (2 << OPAMP_CSR_VPSEL_SHIFT)
#  define OPAMP_CSR_VPSEL_VINP3     (3 << OPAMP_CSR_VPSEL_SHIFT)    /* Only for OPAMP2 */
#  define OPAMP_CSR_VPSEL_DAC3CH1   (3 << OPAMP_CSR_VPSEL_SHIFT)    /* Only for OPAMP1 and OPAMP6 */
#  define OPAMP_CSR_VPSEL_DAC3CH2   (3 << OPAMP_CSR_VPSEL_SHIFT)    /* Only for OPAMP3 */
#  define OPAMP_CSR_VPSEL_DAC4CH1   (3 << OPAMP_CSR_VPSEL_SHIFT)    /* Only for OPAMP4 */
#  define OPAMP_CSR_VPSEL_DAC4CH2   (3 << OPAMP_CSR_VPSEL_SHIFT)    /* Only for OPAMP5 */
#define OPAMP_CSR_USERTRIM          (1 << 4)                        /* Bit 4: User trimming enable */
#define OPAMP_CSR_VMSEL_SHIFT       (5)                             /* Bits 5-6: OPAMP inverting input selection */
#define OPAMP_CSR_VMSEL_MASK        (3 << OPAMP_CSR_VMSEL_SHIFT)
#  define OPAMP_CSR_VMSEL_VINM0     (0 << OPAMP_CSR_VMSEL_SHIFT)
#  define OPAMP_CSR_VMSEL_VINM1     (1 << OPAMP_CSR_VMSEL_SHIFT)
#  define OPAMP_CSR_VMSEL_PGA       (2 << OPAMP_CSR_VMSEL_SHIFT)
#  define OPAMP_CSR_VMSEL_FOLLOWER  (3 << OPAMP_CSR_VMSEL_SHIFT)
#define OPAMP_CSR_OPAHSM            (1 << 7)                        /* Bit 7: OPAMP high-speed mode */

#define OPAMP_CSR_OPAINTOEN         (1 << 8)                        /* Bit 8: OPAMP internal output enable */
                                                                    /* Bits 9-10: Reserved */
#define OPAMP_CSR_CALON             (1 << 11)                       /* Bit 11: Calibration mode enable */
#define OPAMP_CSR_CALSEL_SHIFT      (12)                            /* Bits 12-13: Calibration selection */
#define OPAMP_CSR_CALSEL_MASK       (3 << OPAMP_CSR_CALSEL_SHIFT)
#  define OPAMP_CSR_CALSEL_3P3      (0 << OPAMP_CSR_CALSEL_SHIFT)   /* 00 V_REFOPAMP = 3.3% V_DDA */
#  define OPAMP_CSR_CALSEL_10       (1 << OPAMP_CSR_CALSEL_SHIFT)   /* 01 V_REFOPAMP = 10% V_DDA */
#  define OPAMP_CSR_CALSEL_50       (2 << OPAMP_CSR_CALSEL_SHIFT)   /* 10 V_REFOPAMP = 50% V_DDA */
#  define OPAMP_CSR_CALSEL_90       (3 << OPAMP_CSR_CALSEL_SHIFT)   /* 11 V_REFOPAMP = 90% V_DDA */
#define OPAMP_CSR_PGAGAIN_SHIFT     (14)                            /* Bits 14-18: Gain in PGA mode  */
#define OPAMP_CSR_PGAGAIN_MASK      (31 << OPAMP_CSR_PGAGAIN_SHIFT)
#define OPAMP_CSR_TRIMOFFSETP_SHIFT (19)                            /* Bits 19-23: Offset trimming value (PMOS)*/
#define OPAMP_CSR_TRIMOFFSETP_MASK  (31 << OPAMP_CSR_TRIMOFFSETP_SHIFT)
#define OPAMP_CSR_TRIMOFFSETN_SHIFT (24)                            /* Bits 24-28: Offset trimming value (NMOS) */
#define OPAMP_CSR_TRIMOFFSETN_MASK  (31 << OPAMP_CSR_TRIMOFFSETN_SHIFT)
                                                                    /* Bit 29: Reserved */
#define OPAMP_CSR_OUTCAL            (1 << 30)                       /* Bit 30: OPAMP output status flag */
#define OPAMP_CSR_LOCK              (1 << 31)                       /* Bit 31: OPAMP lock */

/* OPAMP timer controlled mode register */

#define OPAMP_TCMR_VMSSEL           (1 << 0)                        /* Bit 0: OPAMP inverting input secondary selection */
#define OPAMP_TCMR_VPSEL_SHIFT      (1 << 1)                        /* Bits 1-2: OPAMP non inverting input secondary selection */
#define OPAMP_TCMR_VPSEL_MASK       (3 << OPAMP_TCMR_VPSEL_SHIFT)
#  define OPAMP_TCMR_VPSEL_VINP0    (0 << OPAMP_TCMR_VPSEL_SHIFT)
#  define OPAMP_TCMR_VPSEL_VINP1    (1 << OPAMP_TCMR_VPSEL_SHIFT)
#  define OPAMP_TCMR_VPSEL_VINP2    (2 << OPAMP_TCMR_VPSEL_SHIFT)
#  define OPAMP_TCMR_VPSEL_VINP3    (3 << OPAMP_TCMR_VPSEL_SHIFT)   /* Only for OPAMP2 */
#  define OPAMP_TCMR_VPSEL_DAC3CH1  (3 << OPAMP_TCMR_VPSEL_SHIFT)   /* Only for OPAMP1 and OPAMP6 */
#  define OPAMP_TCMR_VPSEL_DAC3CH2  (3 << OPAMP_TCMR_VPSEL_SHIFT)   /* Only for OPAMP3 */
#  define OPAMP_TCMR_VPSEL_DAC4CH1  (3 << OPAMP_TCMR_VPSEL_SHIFT)   /* Only for OPAMP4 */
#  define OPAMP_TCMR_VPSEL_DAC4CH2  (3 << OPAMP_TCMR_VPSEL_SHIFT)   /* Only for OPAMP5 */
#define OPAMP_TCMR_T1CMEN           (1 << 2)                        /* Bit 2: TIM1 controlled mux mode enable */
#define OPAMP_TCMR_T8CMEN           (1 << 3)                        /* Bit 3: TIM8 controlled mux mode enable */
#define OPAMP_TCMR_T20CMEN          (1 << 4)                        /* Bit 4: TIM20 controlled mux mode enable */
                                                                    /* Bits 6-30: Reserved */
#define OPAMP_TCMR_LOCK             (1 << 31)                       /* Bit 31: OPAMP_TCMR lock */

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_OPAMP_H */
