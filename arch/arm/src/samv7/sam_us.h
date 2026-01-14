/****************************************************************************
 * arch/arm/src/samv7/sam_us.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_US_H
#define __ARCH_ARM_SRC_SAMV7_SAM_US_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_erase_user_signature
 *
 * Description:
 *   Erases user signature page.
 *
 * Returned Value:
 *   Zero on success, negated errno value on error.
 *
 ****************************************************************************/

int sam_erase_user_signature(void);

/****************************************************************************
 * Name: sam_write_user_signature
 *
 * Description:
 *   Writes data to user signature page.
 *
 * Input Parameters:
 *   buffer  - The buffer to be written to user signature.
 *   buflen  - Number of bytes to be written.
 *
 * Returned Value:
 *   Number of written bytes on success, negated errno on error.
 *
 ****************************************************************************/

int sam_write_user_signature(void *buffer, size_t buflen);

/****************************************************************************
 * Name: sam_get_user_signature
 *
 * Description:
 *   Get bytes from user signature area.
 *
 * Input Parameters:
 *   buffer  - The buffer to store user signature.
 *   buflen  - Number of bytes to be read.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/

int sam_read_user_signature(void *buffer, size_t buflen);

#endif /* __ARCH_ARM_SRC_SAMV7_SAM_US_H */
