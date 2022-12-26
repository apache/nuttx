/****************************************************************************
 * arch/arm64/src/a64/a64_mipi_dphy.h
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

#ifndef __ARCH_ARM64_SRC_A64_A64_MIPI_DPHY_H
#define __ARCH_ARM64_SRC_A64_A64_MIPI_DPHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/a64_memorymap.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: a64_mipi_dphy_enable
 *
 * Description:
 *   Enable MIPI Display Physical Layer (D-PHY).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_mipi_dphy_enable(void);

#endif /* __ARCH_ARM64_SRC_A64_A64_MIPI_DPHY_H */
