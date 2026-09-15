/****************************************************************************
 * arch/arm/src/n32h7/n32_cordic.h
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

#ifndef __ARCH_ARM_SRC_N32_N32_CORDIC_H
#define __ARCH_ARM_SRC_N32_N32_CORDIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/n32h7_cordic.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: n32_cordicinitialize
 *
 * Description:
 *   Initialize the N32H7 CORDIC peripheral. This function must be called
 *   from board-specific logic.
 *
 * Returned Value:
 *   On success, a pointer to the lower half CORDIC driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct cordic_lowerhalf_s *n32_cordicinitialize(void);

#endif /* __ARCH_ARM_SRC_N32H7_N32_CORDIC_H */
