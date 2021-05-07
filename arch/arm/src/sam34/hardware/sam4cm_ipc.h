/****************************************************************************
 * arch/arm/src/sam34/hardware/sam4cm_ipc.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_IPC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_IPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IPC register offsets *****************************************************/

#define SAM_SLCDC_CR_OFFSET        0x0000 /* Control Register */

#define SAM_IPC_ISCR_OFFSET        0x0000 /* Interrupt Set Command Register */
#define SAM_IPC_ICCR_OFFSET        0x0004 /* Interrupt Clear Command Register */
#define SAM_IPC_IPR_OFFSET         0x0008 /* Interrupt Pending Register */
#define SAM_IPC_IECR_OFFSET        0x000C /* Interrupt Enable Command Register */
#define SAM_IPC_IDCR_OFFSET        0x0010 /* Interrupt Disable Command Register */
#define SAM_IPC_IMR_OFFSET         0x0014 /* Interrupt Mask Register */
#define SAM_IPC_ISR_OFFSET         0x0018 /* Interrupt Status Register */

/* IPC register addresses ***************************************************/

#define SAM_IPC0_ISCR              (SAM_IPC0_BASE + SAM_IPC_ISCR_OFFSET)
#define SAM_IPC0_ICCR              (SAM_IPC0_BASE + SAM_IPC_ICCR_OFFSET)
#define SAM_IPC0_IPR               (SAM_IPC0_BASE + SAM_IPC_IPR_OFFSET)
#define SAM_IPC0_IECR              (SAM_IPC0_BASE + SAM_IPC_IECR_OFFSET)
#define SAM_IPC0_IDCR              (SAM_IPC0_BASE + SAM_IPC_IDCR_OFFSET)
#define SAM_IPC0_IMR               (SAM_IPC0_BASE + SAM_IPC_IMR_OFFSET)
#define SAM_IPC0_ISR               (SAM_IPC0_BASE + SAM_IPC_ISR_OFFSET)

#define SAM_IPC1_ISCR              (SAM_IPC1_BASE + SAM_IPC_ISCR_OFFSET)
#define SAM_IPC1_ICCR              (SAM_IPC1_BASE + SAM_IPC_ICCR_OFFSET)
#define SAM_IPC1_IPR               (SAM_IPC1_BASE + SAM_IPC_IPR_OFFSET)
#define SAM_IPC1_IECR              (SAM_IPC1_BASE + SAM_IPC_IECR_OFFSET)
#define SAM_IPC1_IDCR              (SAM_IPC1_BASE + SAM_IPC_IDCR_OFFSET)
#define SAM_IPC1_IMR               (SAM_IPC1_BASE + SAM_IPC_IMR_OFFSET)
#define SAM_IPC1_ISR               (SAM_IPC1_BASE + SAM_IPC_ISR_OFFSET)

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_IPC_H */
