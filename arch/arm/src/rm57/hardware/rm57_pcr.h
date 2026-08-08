/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_pcr.h
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

/* Register layout taken from TI's HALCoGen HL_reg_pcr.h for
 * RM57L843. Note RM57's PCR frame 1 (0xffff1000) is at a different
 * address than TMS570LS04x's PCR (0xffffe000) - do not assume parity.
 * RM57L843 also has three PCR frames (pcrREG1/2/3) vs. TMS570's one.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_PCR_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_PCR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Peripheral Memory Protection Set Register 0 */
#define RM57_PCR_PMPROTSET0_OFFSET 0x0000

/* Peripheral Memory Protection Set Register 1 */
#define RM57_PCR_PMPROTSET1_OFFSET 0x0004

/* Peripheral Memory Protection Clear Register 0 */
#define RM57_PCR_PMPROTCLR0_OFFSET 0x0010

/* Peripheral Memory Protection Clear Register 1 */
#define RM57_PCR_PMPROTCLR1_OFFSET 0x0014

/* Peripheral Protection Set Register 0 */
#define RM57_PCR_PPROTSET0_OFFSET 0x0020

/* Peripheral Protection Set Register 1 */
#define RM57_PCR_PPROTSET1_OFFSET 0x0024

/* Peripheral Protection Set Register 2 */
#define RM57_PCR_PPROTSET2_OFFSET 0x0028

/* Peripheral Protection Set Register 3 */
#define RM57_PCR_PPROTSET3_OFFSET 0x002c

/* Peripheral Protection Clear Register 0 */
#define RM57_PCR_PPROTCLR0_OFFSET 0x0040

/* Peripheral Protection Clear Register 1 */
#define RM57_PCR_PPROTCLR1_OFFSET 0x0044

/* Peripheral Protection Clear Register 2 */
#define RM57_PCR_PPROTCLR2_OFFSET 0x0048

/* Peripheral Protection Clear Register 3 */
#define RM57_PCR_PPROTCLR3_OFFSET 0x004c

/* Peripheral Clock Set Power-Down Register 0 */
#define RM57_PCR_PCSPWRDWNSET0_OFFSET 0x0060

/* Peripheral Clock Set Power-Down Register 1 */
#define RM57_PCR_PCSPWRDWNSET1_OFFSET 0x0064

/* Peripheral Clock Clear Power-Down Register 0 */
#define RM57_PCR_PCSPWRDWNCLR0_OFFSET 0x0070

/* Peripheral Clock Clear Power-Down Register 1 */
#define RM57_PCR_PCSPWRDWNCLR1_OFFSET 0x0074

/* Peripheral Set Power-Down Register 0 */
#define RM57_PCR_PSPWRDWNSET0_OFFSET 0x0080

/* Peripheral Set Power-Down Register 1 */
#define RM57_PCR_PSPWRDWNSET1_OFFSET 0x0084

/* Peripheral Set Power-Down Register 2 */
#define RM57_PCR_PSPWRDWNSET2_OFFSET 0x0088

/* Peripheral Set Power-Down Register 3 */
#define RM57_PCR_PSPWRDWNSET3_OFFSET 0x008c

/* Peripheral Clear Power-Down Register 0 */
#define RM57_PCR_PSPWRDWNCLR0_OFFSET 0x00a0

/* Peripheral Clear Power-Down Register 1 */
#define RM57_PCR_PSPWRDWNCLR1_OFFSET 0x00a4

/* Peripheral Clear Power-Down Register 2 */
#define RM57_PCR_PSPWRDWNCLR2_OFFSET 0x00a8

/* Peripheral Clear Power-Down Register 3 */
#define RM57_PCR_PSPWRDWNCLR3_OFFSET 0x00ac

/* Peripheral Domain Set Power-Down Register */
#define RM57_PCR_PDPWRDWNSET_OFFSET 0x00c0

/* Peripheral Domain Clear Power-Down Register */
#define RM57_PCR_PDPWRDWNCLR_OFFSET 0x00c4

/* Register Addresses (PCR frame 1) *****************************************/

#define RM57_PCR1_PSPWRDWNSET0 (RM57_PCR1_BASE + RM57_PCR_PSPWRDWNSET0_OFFSET)
#define RM57_PCR1_PSPWRDWNSET1 (RM57_PCR1_BASE + RM57_PCR_PSPWRDWNSET1_OFFSET)
#define RM57_PCR1_PSPWRDWNSET2 (RM57_PCR1_BASE + RM57_PCR_PSPWRDWNSET2_OFFSET)
#define RM57_PCR1_PSPWRDWNSET3 (RM57_PCR1_BASE + RM57_PCR_PSPWRDWNSET3_OFFSET)
#define RM57_PCR1_PSPWRDWNCLR0 (RM57_PCR1_BASE + RM57_PCR_PSPWRDWNCLR0_OFFSET)
#define RM57_PCR1_PSPWRDWNCLR1 (RM57_PCR1_BASE + RM57_PCR_PSPWRDWNCLR1_OFFSET)
#define RM57_PCR1_PSPWRDWNCLR2 (RM57_PCR1_BASE + RM57_PCR_PSPWRDWNCLR2_OFFSET)
#define RM57_PCR1_PSPWRDWNCLR3 (RM57_PCR1_BASE + RM57_PCR_PSPWRDWNCLR3_OFFSET)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_PCR_H */
