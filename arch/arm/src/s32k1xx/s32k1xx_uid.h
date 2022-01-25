/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_uid.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_UID_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_UID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define S32K1XX_UID_SIZE 16

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void s32k1xx_get_uniqueid(uint8_t *uniqueid);

#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_UID_H */
