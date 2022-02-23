/****************************************************************************
 * arch/misoc/src/common/hw/ethmac_mem.h
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

#ifndef __ARCH_MISOC_SRC_COMMON_HW_ETHMAC_MEM_H
#define __ARCH_MISOC_SRC_COMMON_HW_ETHMAC_MEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/generated/mem.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHMAC_RX0_BASE	ETHMAC_BASE
#define ETHMAC_RX1_BASE	(ETHMAC_BASE+0x0800)
#define ETHMAC_TX0_BASE	(ETHMAC_BASE+0x1000)
#define ETHMAC_TX1_BASE	(ETHMAC_BASE+0x1800)

#endif /* __ARCH_MISOC_SRC_COMMON_HW_ETHMAC_MEM_H */
