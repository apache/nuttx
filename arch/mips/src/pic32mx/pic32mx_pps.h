/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_pps.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_PPS_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_PPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx_memorymap.h"

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Peripheral pin select input registers */

#define PIC32MX_PPS_INT1R_OFFSET    0x0004
#define PIC32MX_PPS_INT2R_OFFSET    0x0008
#define PIC32MX_PPS_INT3R_OFFSET    0x000c
#define PIC32MX_PPS_INT4R_OFFSET    0x0010
#define PIC32MX_PPS_T2CKR_OFFSET    0x0018
#define PIC32MX_PPS_T3CKR_OFFSET    0x001c
#define PIC32MX_PPS_T4CKR_OFFSET    0x0020
#define PIC32MX_PPS_T5CKR_OFFSET    0x0024
#define PIC32MX_PPS_IC1R_OFFSET     0x0028
#define PIC32MX_PPS_IC2R_OFFSET     0x002c
#define PIC32MX_PPS_IC3R_OFFSET     0x0030
#define PIC32MX_PPS_IC4R_OFFSET     0x0034
#define PIC32MX_PPS_IC5R_OFFSET     0x0038
#define PIC32MX_PPS_OCFAR_OFFSET    0x0048
#define PIC32MX_PPS_OCFBR_OFFSET    0x004c
#define PIC32MX_PPS_U1RXR_OFFSET    0x0050
#define PIC32MX_PPS_U1CTSR_OFFSET   0x0054
#define PIC32MX_PPS_U2RXR_OFFSET    0x0058
#define PIC32MX_PPS_U2CTSR_OFFSET   0x005c
#define PIC32MX_PPS_SDI1R_OFFSET    0x0084
#define PIC32MX_PPS_SS1R_OFFSET     0x0088
#define PIC32MX_PPS_SDI2R_OFFSET    0x0090
#define PIC32MX_PPS_SS2R_OFFSET     0x0094
#define PIC32MX_PPS_REFCLKIR_OFFSET 0x00b8

/* Peripheral pin select output registers */

#define PIC32MX_PPS_RPA0R_OFFSET    0x0000
#define PIC32MX_PPS_RPA1R_OFFSET    0x0004
#define PIC32MX_PPS_RPA2R_OFFSET    0x0008
#define PIC32MX_PPS_RPA3R_OFFSET    0x000c
#define PIC32MX_PPS_RPA4R_OFFSET    0x0010
#define PIC32MX_PPS_RPA8R_OFFSET    0x0020
#define PIC32MX_PPS_RPA9R_OFFSET    0x0024
#define PIC32MX_PPS_RPB0R_OFFSET    0x002c
#define PIC32MX_PPS_RPB1R_OFFSET    0x0030
#define PIC32MX_PPS_RPB2R_OFFSET    0x0034
#define PIC32MX_PPS_RPB3R_OFFSET    0x0038
#define PIC32MX_PPS_RPB4R_OFFSET    0x003c
#define PIC32MX_PPS_RPB5R_OFFSET    0x0040
#define PIC32MX_PPS_RPB6R_OFFSET    0x0044
#define PIC32MX_PPS_RPB7R_OFFSET    0x0048
#define PIC32MX_PPS_RPB8R_OFFSET    0x004c
#define PIC32MX_PPS_RPB9R_OFFSET    0x0050
#define PIC32MX_PPS_RPB10R_OFFSET   0x0054
#define PIC32MX_PPS_RPB11R_OFFSET   0x0058
#define PIC32MX_PPS_RPB13R_OFFSET   0x0060
#define PIC32MX_PPS_RPB14R_OFFSET   0x0064
#define PIC32MX_PPS_RPB15R_OFFSET   0x0068
#define PIC32MX_PPS_RPC0R_OFFSET    0x006c
#define PIC32MX_PPS_RPC1R_OFFSET    0x0070
#define PIC32MX_PPS_RPC2R_OFFSET    0x0074
#define PIC32MX_PPS_RPC3R_OFFSET    0x0078
#define PIC32MX_PPS_RPC4R_OFFSET    0x007c
#define PIC32MX_PPS_RPC5R_OFFSET    0x0080
#define PIC32MX_PPS_RPC6R_OFFSET    0x0084
#define PIC32MX_PPS_RPC7R_OFFSET    0x0088
#define PIC32MX_PPS_RPC8R_OFFSET    0x008c
#define PIC32MX_PPS_RPC9R_OFFSET    0x0090

/* Register Addresses *******************************************************/

/* Peripheral pin select input registers */

#define PIC32MX_PPS_INT1R           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_INT1R_OFFSET)
#define PIC32MX_PPS_INT2R           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_INT2R_OFFSET)
#define PIC32MX_PPS_INT3R           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_INT3R_OFFSET)
#define PIC32MX_PPS_INT4R           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_INT4R_OFFSET)
#define PIC32MX_PPS_T2CKR           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_T2CKR_OFFSET)
#define PIC32MX_PPS_T3CKR           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_T3CKR_OFFSET)
#define PIC32MX_PPS_T4CKR           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_T4CKR_OFFSET)
#define PIC32MX_PPS_T5CKR           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_T5CKR_OFFSET)
#define PIC32MX_PPS_IC1R            (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_IC1R_OFFSET)
#define PIC32MX_PPS_IC2R            (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_IC2R_OFFSET)
#define PIC32MX_PPS_IC3R            (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_IC3R_OFFSET)
#define PIC32MX_PPS_IC4R            (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_IC4R_OFFSET)
#define PIC32MX_PPS_IC5R            (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_IC5R_OFFSET)
#define PIC32MX_PPS_OCFAR           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_OCFAR_OFFSET)
#define PIC32MX_PPS_OCFBR           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_OCFBR_OFFSET)
#define PIC32MX_PPS_U1RXR           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_U1RXR_OFFSET)
#define PIC32MX_PPS_U1CTSR          (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_U1CTSR_OFFSET)
#define PIC32MX_PPS_U2RXR           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_U2RXR_OFFSET)
#define PIC32MX_PPS_U2CTSR          (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_U2CTSR_OFFSET)
#define PIC32MX_PPS_SDI1R           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_SDI1R_OFFSET)
#define PIC32MX_PPS_SS1R            (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_SS1R_OFFSET)
#define PIC32MX_PPS_SDI2R           (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_SDI2R_OFFSET)
#define PIC32MX_PPS_SS2R            (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_SS2R_OFFSET)
#define PIC32MX_PPS_REFCLKIR        (PIC32MX_INSEL_K1BASE+PIC32MX_PPS_REFCLKIR_OFFSET)

/* Peripheral pin select output registers */

#define PIC32MX_PPS_RPA0R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPA0R_OFFSET)
#define PIC32MX_PPS_RPA1R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPA1R_OFFSET)
#define PIC32MX_PPS_RPA2R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPA2R_OFFSET)
#define PIC32MX_PPS_RPA3R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPA3R_OFFSET)
#define PIC32MX_PPS_RPA4R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPA4R_OFFSET)
#define PIC32MX_PPS_RPA8R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPA8R_OFFSET)
#define PIC32MX_PPS_RPA9R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPA9R_OFFSET)
#define PIC32MX_PPS_RPB0R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB0R_OFFSET)
#define PIC32MX_PPS_RPB1R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB1R_OFFSET)
#define PIC32MX_PPS_RPB2R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB2R_OFFSET)
#define PIC32MX_PPS_RPB3R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB3R_OFFSET)
#define PIC32MX_PPS_RPB4R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB4R_OFFSET)
#define PIC32MX_PPS_RPB5R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB5R_OFFSET)
#define PIC32MX_PPS_RPB6R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB6R_OFFSET)
#define PIC32MX_PPS_RPB7R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB7R_OFFSET)
#define PIC32MX_PPS_RPB8R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB8R_OFFSET)
#define PIC32MX_PPS_RPB9R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB9R_OFFSET)
#define PIC32MX_PPS_RPB10R          (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB10R_OFFSET)
#define PIC32MX_PPS_RPB11R          (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB11R_OFFSET)
#define PIC32MX_PPS_RPB13R          (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB13R_OFFSET)
#define PIC32MX_PPS_RPB14R          (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB14R_OFFSET)
#define PIC32MX_PPS_RPB15R          (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPB15R_OFFSET)
#define PIC32MX_PPS_RPC0R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC0R_OFFSET)
#define PIC32MX_PPS_RPC1R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC1R_OFFSET)
#define PIC32MX_PPS_RPC2R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC2R_OFFSET)
#define PIC32MX_PPS_RPC3R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC3R_OFFSET)
#define PIC32MX_PPS_RPC4R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC4R_OFFSET)
#define PIC32MX_PPS_RPC5R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC5R_OFFSET)
#define PIC32MX_PPS_RPC6R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC6R_OFFSET)
#define PIC32MX_PPS_RPC7R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC7R_OFFSET)
#define PIC32MX_PPS_RPC8R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC8R_OFFSET)
#define PIC32MX_PPS_RPC9R           (PIC32MX_OUTSEL_K1BASE+PIC32MX_PPS_RPC9R_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Peripheral pin select input registers */

#define PPS_INSEL_MASK              0x0000000f

#define PPS_INSEL_RPA0              0
#define PPS_INSEL_RPB3              1
#define PPS_INSEL_RPB4              2
#define PPS_INSEL_RPB15             3
#define PPS_INSEL_RPB7              4
#define PPS_INSEL_RPC7              5
#define PPS_INSEL_RPC0              6
#define PPS_INSEL_RPC5              7

#define PPS_INSEL_RPA1              0
#define PPS_INSEL_RPB5              1
#define PPS_INSEL_RPB1              2
#define PPS_INSEL_RPB11             3
#define PPS_INSEL_RPB8              4
#define PPS_INSEL_RPA8              5
#define PPS_INSEL_RPC8              6
#define PPS_INSEL_RPA9              7

#define PPS_INSEL_RPA2              0
#define PPS_INSEL_RPB6              1
#define PPS_INSEL_RPA4              2
#define PPS_INSEL_RPB13             3
#define PPS_INSEL_RPB2              4
#define PPS_INSEL_RPC6              5
#define PPS_INSEL_RPC1              6
#define PPS_INSEL_RPC3              7

#define PPS_INSEL_RPA3              0
#define PPS_INSEL_RPB14             1
#define PPS_INSEL_RPB0              2
#define PPS_INSEL_RPB10             3
#define PPS_INSEL_RPB9              4
#define PPS_INSEL_RPC9              5
#define PPS_INSEL_RPC2              6
#define PPS_INSEL_RPC4              7

/* Peripheral pin select output registers */

#define PPS_OUTSEL_MASK             0x0000000f

#define PPS_OUTSEL_NOCONNECT        0

#define PPS_OUTSEL_U1TX             1
#define PPS_OUTSEL_U2RTS            2
#define PPS_OUTSEL_SS1              3
#define PPS_OUTSEL_OC1              5
#define PPS_OUTSEL_C2OUT            7

#define PPS_OUTSEL_SDO1             3
#define PPS_OUTSEL_SDO2             4
#define PPS_OUTSEL_OC2              5

/* #define PPS_OUTSEL_SDO1           3 */

/* #define PPS_OUTSEL_SDO2           4 */
#define PPS_OUTSEL_OC4              5
#define PPS_OUTSEL_OC5              6
#define PPS_OUTSEL_REFCLKO          7

#define PPS_OUTSEL_U1RTS            1
#define PPS_OUTSEL_U2TX             2
#define PPS_OUTSEL_SS2              4
#define PPS_OUTSEL_OC3              5
#define PPS_OUTSEL_C1OUT            7

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

#endif /* __ASSEMBLY__ */
#endif /* CHIP_PIC32MX1 || CHIP_PIC32MX2 */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_PPS_H */
