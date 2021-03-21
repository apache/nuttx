/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_axi.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_AXI_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_AXI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_AXI_PERIPH_ID_4_OFFSET          0x01fd0  /* AXI interconnect peripheral ID4 register */
#define STM32_AXI_PERIPH_ID_0_OFFSET          0x01fe0  /* AXI interconnect peripheral ID0 register */
#define STM32_AXI_PERIPH_ID_1_OFFSET          0x01fe4  /* AXI interconnect peripheral ID1 register */
#define STM32_AXI_PERIPH_ID_2_OFFSET          0x01fe8  /* AXI interconnect peripheral ID2 register */
#define STM32_AXI_PERIPH_ID_3_OFFSET          0x01fec  /* AXI interconnect peripheral ID3 register */
#define STM32_AXI_COMP_ID_0_OFFSET            0x01ff0  /* AXI interconnect component  ID0 register */
#define STM32_AXI_COMP_ID_1_OFFSET            0x01ff4  /* AXI interconnect component  ID1 register */
#define STM32_AXI_COMP_ID_2_OFFSET            0x01ff8  /* AXI interconnect component  ID2 register */
#define STM32_AXI_COMP_ID_3_OFFSET            0x01ffc  /* AXI interconnect component  ID3 register */
#define STM32_AXI_TARG1_FN_MOD_ISS_BM_OFFSET  0x02008  /* AXI interconnect TARG 1 bus matrix issuing functionality register */
#define STM32_AXI_TARG2_FN_MOD_ISS_BM_OFFSET  0x03008  /* AXI interconnect TARG 2 bus matrix issuing functionality register */
#define STM32_AXI_TARG3_FN_MOD_ISS_BM_OFFSET  0x04008  /* AXI interconnect TARG 3 bus matrix issuing functionality register */
#define STM32_AXI_TARG4_FN_MOD_ISS_BM_OFFSET  0x05008  /* AXI interconnect TARG 4 bus matrix issuing functionality register */
#define STM32_AXI_TARG5_FN_MOD_ISS_BM_OFFSET  0x06008  /* AXI interconnect TARG 5 bus matrix issuing functionality register */
#define STM32_AXI_TARG6_FN_MOD_ISS_BM_OFFSET  0x07008  /* AXI interconnect TARG 6 bus matrix issuing functionality register */
#define STM32_AXI_TARG7_FN_MOD_ISS_BM_OFFSET  0x08008  /* AXI interconnect TARG 7 bus matrix issuing functionality register */
#define STM32_AXI_TARG1_FN_MOD2_OFFSET        0x02024  /* AXI interconnect TARG 1 bus matrix functionality 2 register */
#define STM32_AXI_TARG2_FN_MOD2_OFFSET        0x03024  /* AXI interconnect TARG 2 bus matrix functionality 2 register */
#define STM32_AXI_TARG7_FN_MOD2_OFFSET        0x08024  /* AXI interconnect TARG 7 bus matrix functionality 2 register */
#define STM32_AXI_TARG1_FN_MOD_LB_OFFSET      0x0202c  /* AXI interconnect TARG 1 long burst functionality modification register */
#define STM32_AXI_TARG2_FN_MOD_LB_OFFSET      0x0302c  /* AXI interconnect TARG 2 long burst functionality modification register */
#define STM32_AXI_TARG1_FN_MOD_OFFSET         0x02108  /* AXI interconnect TARG 1 issuing functionality modification register */
#define STM32_AXI_TARG2_FN_MOD_OFFSET         0x03108  /* AXI interconnect TARG 2 issuing functionality modification register */
#define STM32_AXI_TARG7_FN_MOD_OFFSET         0x08108  /* AXI interconnect TARG 7 issuing functionality modification register */
#define STM32_AXI_INI1_FN_MOD2_OFFSET         0x42024  /* AXI interconnect INI 1 functionality modification 2 register */
#define STM32_AXI_INI3_FN_MOD2_OFFSET         0x44024  /* AXI interconnect INI 3 functionality modification 2 register */
#define STM32_AXI_INI1_FN_MOD_AHB_OFFSET      0x42028  /* AXI interconnect INI 1 AHB functionality modification register */
#define STM32_AXI_INI3_FN_MOD_AHB_OFFSET      0x44028  /* AXI interconnect INI 3 AHB functionality modification register */
#define STM32_AXI_INI1_READ_QOS_OFFSET        0x42100  /* AXI interconnect INI 1 read QoS register */
#define STM32_AXI_INI2_READ_QOS_OFFSET        0x43100  /* AXI interconnect INI 2 read QoS register */
#define STM32_AXI_INI3_READ_QOS_OFFSET        0x44100  /* AXI interconnect INI 3 read QoS register */
#define STM32_AXI_INI4_READ_QOS_OFFSET        0x45100  /* AXI interconnect INI 4 read QoS register */
#define STM32_AXI_INI5_READ_QOS_OFFSET        0x46100  /* AXI interconnect INI 5 read QoS register */
#define STM32_AXI_INI6_READ_QOS_OFFSET        0x47100  /* AXI interconnect INI 6 read QoS register */
#define STM32_AXI_INI1_WRITE_QOS_OFFSET       0x42104  /* AXI interconnect INI 1 write QoS register */
#define STM32_AXI_INI2_WRITE_QOS_OFFSET       0x43104  /* AXI interconnect INI 2 write QoS register */
#define STM32_AXI_INI3_WRITE_QOS_OFFSET       0x44104  /* AXI interconnect INI 3 write QoS register */
#define STM32_AXI_INI4_WRITE_QOS_OFFSET       0x45104  /* AXI interconnect INI 4 write QoS register */
#define STM32_AXI_INI5_WRITE_QOS_OFFSET       0x46104  /* AXI interconnect INI 5 write QoS register */
#define STM32_AXI_INI6_WRITE_QOS_OFFSET       0x47104  /* AXI interconnect INI 6 write QoS register */
#define STM32_AXI_INI1_FN_MOD_OFFSET          0x42108  /* AXI interconnect INI 1 issuing functionality modification register */
#define STM32_AXI_INI2_FN_MOD_OFFSET          0x43108  /* AXI interconnect INI 2 issuing functionality modification register */
#define STM32_AXI_INI3_FN_MOD_OFFSET          0x44108  /* AXI interconnect INI 3 issuing functionality modification register */
#define STM32_AXI_INI4_FN_MOD_OFFSET          0x45108  /* AXI interconnect INI 4 issuing functionality modification register */
#define STM32_AXI_INI5_FN_MOD_OFFSET          0x46108  /* AXI interconnect INI 5 issuing functionality modification register */
#define STM32_AXI_INI6_FN_MOD_OFFSET          0x47108  /* AXI interconnect INI 6 issuing functionality modification register */

/* Register Addresses *******************************************************/

#define STM32_AXI_PERIPH_ID_4                 (STM32_GPV_BASE + STM32_AXI_PERIPH_ID_4_OFFSET)
#define STM32_AXI_PERIPH_ID_0                 (STM32_GPV_BASE + STM32_AXI_PERIPH_ID_0_OFFSET)
#define STM32_AXI_PERIPH_ID_1                 (STM32_GPV_BASE + STM32_AXI_PERIPH_ID_1_OFFSET)
#define STM32_AXI_PERIPH_ID_2                 (STM32_GPV_BASE + STM32_AXI_PERIPH_ID_2_OFFSET)
#define STM32_AXI_PERIPH_ID_3                 (STM32_GPV_BASE + STM32_AXI_PERIPH_ID_3_OFFSET)
#define STM32_AXI_COMP_ID_0                   (STM32_GPV_BASE + STM32_AXI_COMP_ID_0_OFFSET)
#define STM32_AXI_COMP_ID_1                   (STM32_GPV_BASE + STM32_AXI_COMP_ID_1_OFFSET)
#define STM32_AXI_COMP_ID_2                   (STM32_GPV_BASE + STM32_AXI_COMP_ID_2_OFFSET)
#define STM32_AXI_COMP_ID_3                   (STM32_GPV_BASE + STM32_AXI_COMP_ID_3_OFFSET)
#define STM32_AXI_TARG1_FN_MOD_ISS_BM         (STM32_GPV_BASE + STM32_AXI_TARG1_FN_MOD_ISS_BM_OFFSET)
#define STM32_AXI_TARG2_FN_MOD_ISS_BM         (STM32_GPV_BASE + STM32_AXI_TARG2_FN_MOD_ISS_BM_OFFSET)
#define STM32_AXI_TARG3_FN_MOD_ISS_BM         (STM32_GPV_BASE + STM32_AXI_TARG3_FN_MOD_ISS_BM_OFFSET)
#define STM32_AXI_TARG4_FN_MOD_ISS_BM         (STM32_GPV_BASE + STM32_AXI_TARG4_FN_MOD_ISS_BM_OFFSET)
#define STM32_AXI_TARG5_FN_MOD_ISS_BM         (STM32_GPV_BASE + STM32_AXI_TARG5_FN_MOD_ISS_BM_OFFSET)
#define STM32_AXI_TARG6_FN_MOD_ISS_BM         (STM32_GPV_BASE + STM32_AXI_TARG6_FN_MOD_ISS_BM_OFFSET)
#define STM32_AXI_TARG7_FN_MOD_ISS_BM         (STM32_GPV_BASE + STM32_AXI_TARG7_FN_MOD_ISS_BM_OFFSET)
#define STM32_AXI_TARG1_FN_MOD2               (STM32_GPV_BASE + STM32_AXI_TARG1_FN_MOD2_OFFSET)
#define STM32_AXI_TARG2_FN_MOD2               (STM32_GPV_BASE + STM32_AXI_TARG2_FN_MOD2_OFFSET)
#define STM32_AXI_TARG7_FN_MOD2               (STM32_GPV_BASE + STM32_AXI_TARG7_FN_MOD2_OFFSET)
#define STM32_AXI_TARG1_FN_MOD_LB             (STM32_GPV_BASE + STM32_AXI_TARG1_FN_MOD_LB_OFFSET)
#define STM32_AXI_TARG2_FN_MOD_LB             (STM32_GPV_BASE + STM32_AXI_TARG2_FN_MOD_LB_OFFSET)
#define STM32_AXI_TARG1_FN_MOD                (STM32_GPV_BASE + STM32_AXI_TARG1_FN_MOD_OFFSET)
#define STM32_AXI_TARG2_FN_MOD                (STM32_GPV_BASE + STM32_AXI_TARG2_FN_MOD_OFFSET)
#define STM32_AXI_TARG7_FN_MOD                (STM32_GPV_BASE + STM32_AXI_TARG7_FN_MOD_OFFSET)
#define STM32_AXI_INI1_FN_MOD2                (STM32_GPV_BASE + STM32_AXI_INI1_FN_MOD2_OFFSET)
#define STM32_AXI_INI3_FN_MOD2                (STM32_GPV_BASE + STM32_AXI_INI3_FN_MOD2_OFFSET)
#define STM32_AXI_INI1_FN_MOD_AHB             (STM32_GPV_BASE + STM32_AXI_INI1_FN_MOD_AHB_OFFSET)
#define STM32_AXI_INI3_FN_MOD_AHB             (STM32_GPV_BASE + STM32_AXI_INI3_FN_MOD_AHB_OFFSET)
#define STM32_AXI_INI1_READ_QOS               (STM32_GPV_BASE + STM32_AXI_INI1_READ_QOS_OFFSET)
#define STM32_AXI_INI2_READ_QOS               (STM32_GPV_BASE + STM32_AXI_INI2_READ_QOS_OFFSET)
#define STM32_AXI_INI3_READ_QOS               (STM32_GPV_BASE + STM32_AXI_INI3_READ_QOS_OFFSET)
#define STM32_AXI_INI4_READ_QOS               (STM32_GPV_BASE + STM32_AXI_INI4_READ_QOS_OFFSET)
#define STM32_AXI_INI5_READ_QOS               (STM32_GPV_BASE + STM32_AXI_INI5_READ_QOS_OFFSET)
#define STM32_AXI_INI6_READ_QOS               (STM32_GPV_BASE + STM32_AXI_INI6_READ_QOS_OFFSET)
#define STM32_AXI_INI1_WRITE_QOS              (STM32_GPV_BASE + STM32_AXI_INI1_WRITE_QOS_OFFSET)
#define STM32_AXI_INI2_WRITE_QOS              (STM32_GPV_BASE + STM32_AXI_INI2_WRITE_QOS_OFFSET)
#define STM32_AXI_INI3_WRITE_QOS              (STM32_GPV_BASE + STM32_AXI_INI3_WRITE_QOS_OFFSET)
#define STM32_AXI_INI4_WRITE_QOS              (STM32_GPV_BASE + STM32_AXI_INI4_WRITE_QOS_OFFSET)
#define STM32_AXI_INI5_WRITE_QOS              (STM32_GPV_BASE + STM32_AXI_INI5_WRITE_QOS_OFFSET)
#define STM32_AXI_INI6_WRITE_QOS              (STM32_GPV_BASE + STM32_AXI_INI6_WRITE_QOS_OFFSET)
#define STM32_AXI_INI1_FN_MOD                 (STM32_GPV_BASE + STM32_AXI_INI1_FN_MOD_OFFSET)
#define STM32_AXI_INI2_FN_MOD                 (STM32_GPV_BASE + STM32_AXI_INI2_FN_MOD_OFFSET)
#define STM32_AXI_INI3_FN_MOD                 (STM32_GPV_BASE + STM32_AXI_INI3_FN_MOD_OFFSET)
#define STM32_AXI_INI4_FN_MOD                 (STM32_GPV_BASE + STM32_AXI_INI4_FN_MOD_OFFSET)
#define STM32_AXI_INI5_FN_MOD                 (STM32_GPV_BASE + STM32_AXI_INI5_FN_MOD_OFFSET)
#define STM32_AXI_INI6_FN_MOD                 (STM32_GPV_BASE + STM32_AXI_INI6_FN_MOD_OFFSET)

/* AXI Register Bitfield Definitions ****************************************/

/* TARG x bus matrix issuing functionality Register */

#define AXI_TARG_READ_ISS_BM_OVERRIDE         (1 << 0)  /* Bit 0:  Switch matrix read issuing override for target */
#define AXI_TARG_WRITE_ISS_BM_OVERRIDE        (1 << 1)  /* Bit 1:  Switch matrix write issuing override for target */

/* TARG x bus matrix issuing functionality 2 Register */

#define AXI_TARG_BYPASS_MERGE                 (1 << 0)  /* Bit 0:  Disable packing of beats to match the output data width. Unaligned transactions are not realigned to the input data word boundary */

/* TARG x long burst functionality modification Register */

#define AXI_TARG_FN_MOD_LB                    (1 << 0)  /* Bit 0:  Controls burst breaking of long bursts */

/* TARG x bus matrix issuing functionality Register */

#define AXI_TARG_READ_ISS_OVERRIDE            (1 << 0)  /* Bit 0:  Override AMIB read issuing capability */
#define AXI_TARG_WRITE_ISS_OVERRIDE           (1 << 1)  /* Bit 1:  Override AMIB write issuing capability */

/* INI x functionality modification 2 Register */

#define AXI_INI_BYPASS_MERGE                  (1 << 0)  /* Bit 0:  Disables alteration of transactions by the up-sizer unless required by the protocol */

/* INI x AHB functionality modification Register */

#define AXI_INI_RD_INC_OVERRIDE               (1 << 0)  /* Bit 0:  Converts all AHB-Lite read transactions to a series of single beat AXI transactions. */
#define AXI_INI_WR_INC_OVERRIDE               (1 << 1)  /* Bit 1:  Converts all AHB-Lite write transactions to a series of single beat AXI transactions */

/* INI x read QoS Register */

#define AXI_INI_AR_QOS_SHIFT                  (0)  /* Bits 0-3: Read channel QoS setting */
#define AXI_INI_AR_QOS_MASK                   (0xf << AXI_INI_AR_QOS_SHIFT)
#define AXI_INI_AR_QOS(x)                     (((x) & 0xf) << AXI_INI_AR_QOS_SHIFT)

/* INI x write QoS Register */

#define AXI_INI_AW_QOS_SHIFT                  (0)  /* Bits 0-3: Write channel QoS setting */
#define AXI_INI_AW_QOS_MASK                   (0xf << AXI_INI_AW_QOS_SHIFT)
#define AXI_INI_AW_QOS(x)                     (((x) & 0xf) << AXI_INI_AW_QOS_SHIFT)

/* INI x issuing functionality modification Register */

#define AXI_INI_READ_ISS_OVERRIDE            (1 << 0)  /* Bit 0:  Override ASIB read issuing capability */
#define AXI_INI_WRITE_ISS_OVERRIDE           (1 << 1)  /* Bit 1:  Override ASIB write issuing capability */

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_AXI_H */
