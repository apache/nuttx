/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_rstc.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_RSTC_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_RSTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RSTC register offsets ****************************************************/

#define SAM_RSTC_RCAUSE_OFFSET         0x0000  /* Reset cause */
#define SAM_RSTC_BKUPEXIT_OFFSET       0x0002  /* Backup exit source */

/* RSTC register addresses **************************************************/

#define SAM_RSTC_RCAUSE                (SAM_RSTC_BASE + SAM_RSTC_RCAUSE_OFFSET)
#define SAM_RSTC_BKUPEXIT              (SAM_RSTC_BASE + SAM_RSTC_BKUPEXIT_OFFSET)

/* RSTC register bit definitions ********************************************/

/* Reset cause */

#define RSTC_RCAUSE_POR                (1 << 0)  /* Bit 0: Power on reset */
#define RSTC_RCAUSE_BOD12              (1 << 1)  /* Bit 1: Brown out 12 detector reset */
#define RSTC_RCAUSE_BOD33              (1 << 2)  /* Bit 2: Brown out 33 detector reset */
#define RSTC_RCAUSE_NVM                (1 << 3)  /* Bit 3: External reset */
#define RSTC_RCAUSE_EXT                (1 << 4)  /* Bit 4: External reset */
#define RSTC_RCAUSE_WDT                (1 << 5)  /* Bit 5: Watchdog reset */
#define RSTC_RCAUSE_SYST               (1 << 6)  /* Bit 6: System reset request */
#define RSTC_RCAUSE_BACKUP             (1 << 7)  /* Bit 7: Backup reset*/

/* Backup exit source */

#define RSTC_BKUPEXIT_RTC              (1 << 1)  /* Bit 1: Real time counter interrupt */
#define RSTC_BKUPEXIT_BBPS             (1 << 2)  /* Bit 2: Battery backup power switch */
#define RSTC_BKUPEXIT_HIB              (1 << 7)  /* Bit 7: Hibernate */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_RSTC_H */
