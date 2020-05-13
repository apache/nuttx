/****************************************************************************
 * arch/arm/src/samv7/sam_eefc.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_EEFC_H
#define __ARCH_ARM_SRC_SAMV7_SAM_EEFC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "hardware/sam_eefc.h"

#include <nuttx/progmem.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SAM_EFC_ACCESS_MODE_128    0
#define SAM_EFC_ACCESS_MODE_64     EEFC_FMR_FAM

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_eefc_writefmr
 *
 * Description:
 *   Write flash mode register.
 *
 ****************************************************************************/

__ramfunc__ void sam_eefc_writefmr(uint32_t regval);

/****************************************************************************
 * Name: sam_eefc_command
 *
 * Description:
 *   Send a FLASH command.
 *
 ****************************************************************************/

__ramfunc__ int sam_eefc_command(uint32_t cmd, uint32_t arg);

/****************************************************************************
 * Name: sam_eefc_readsequence
 *
 * Description:
 *   Flash sequence read.
 *
 ****************************************************************************/

__ramfunc__ int sam_eefc_readsequence(uint32_t start_cmd, uint32_t stop_cmd,
                                      uint32_t *buffer, size_t bufsize);

/****************************************************************************
 * Name: sam_eefc_initaccess
 *
 * Description:
 *   Initial enhanced embedded flash access mode.
 *
 ****************************************************************************/

void sam_eefc_initaccess(uint32_t access_mode, uint32_t wait_status);

/****************************************************************************
 * Name: sam_eefc_unlock
 *
 * Description:
 *   Make sure that the FLASH is unlocked
 *
 ****************************************************************************/

int sam_eefc_unlock(size_t page, size_t npages);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_EEFC_H */
