/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_mspi.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_MSPI_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_MSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MSPI_DATA_REG              REG_ADDR8(0x0c)
#define MSPI_CTRL_REG              REG_ADDR8(0x0d)
#define MSPI_MODE_REG              REG_ADDR8(0x0f)

#define MSPI_CTRL_CS_SHIFT         0
#define MSPI_CTRL_CS_MASK          (0x1 << MSPI_CTRL_CS_SHIFT)
#define MSPI_CTRL_CS_HIGH          (0x1 << MSPI_CTRL_CS_SHIFT)
#define MSPI_CTRL_CS_LOW           (0x0 << MSPI_CTRL_CS_SHIFT)

#define MSPI_CTRL_SDO_SHIFT        1
#define MSPI_CTRL_SDO_MASK         (0x1 << MSPI_CTRL_SDO_SHIFT)

#define MSPI_CTRL_CONT_SHIFT       2
#define MSPI_CTRL_CONT_MASK        (0x1 << MSPI_CTRL_CONT_SHIFT)

#define MSPI_CTRL_RD_SHIFT         3
#define MSPI_CTRL_RD_MASK          (0x1 << MSPI_CTRL_RD_SHIFT)

#define MSPI_CTRL_BUSY_SHIFT       4
#define MSPI_CTRL_BUSY_MASK        (0x1 << MSPI_CTRL_BUSY_SHIFT)

#define MSPI_MODE_DUAL_DATA_SHIFT  0
#define MSPI_MODE_DUAL_DATA_MASK   (0x1 << MSPI_MODE_DUAL_DATA_SHIFT)

#define MSPI_MODE_DUAL_ADDR_SHIFT  1
#define MSPI_MODE_DUAL_ADDR_MASK   (0x1 << MSPI_MODE_DUAL_ADDR_SHIFT)

#define MSPI_MODE_CLKDIV_SHIFT     2
#define MSPI_MODE_CLKDIV_MASK      (0x3f << MSPI_MODE_CLKDIV_SHIFT)

/* MSPI Operation Macros, follow the telink SDK */

#define MSPI_CS_HIGH()             (MSPI_CTRL_REG = MSPI_CTRL_CS_MASK)
#define MSPI_CS_LOW()              (MSPI_CTRL_REG = 0)
#define MSPI_WRITE(data)           (MSPI_DATA_REG = (data))
#define MSPI_READ()                (MSPI_DATA_REG)
#define MSPI_WAIT()                while (MSPI_CTRL_REG & MSPI_CTRL_BUSY_MASK)
#define MSPI_AUTO_MODE()           (MSPI_CTRL_REG = 0x0a)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_MSPI_H */
