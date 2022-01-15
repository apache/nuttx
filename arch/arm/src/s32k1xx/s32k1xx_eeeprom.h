/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_eeeprom.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_EEEPROM_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_EEEPROM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "s32k1xx_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_eeeprom_init
 *
 * Description:
 *   Init FTFC flash controller to run in Enhanced EEPROM mode
 *
 *
 ****************************************************************************/

void s32k1xx_eeeprom_init(void);

/****************************************************************************
 * Name: s32k1xx_eeeprom_register
 *
 * Description:
 *   Non-standard function to register a eeeprom
 *
 * Input Parameters:
 *   minor:     Selects suffix of device named /dev/eeepromN, N={1,2,3...}
 *   size:      The size of eeprom in bytes
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int s32k1xx_eeeprom_register(int minor, uint32_t size);
#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_EEEPROM_H */
