/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_dfifo.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_DFIFO_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_DFIFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DFIFO = DMA FIFO */

/* DFIFO0 ~ DFIFO2 address and size register definition */

#define DFIFO0_ADDR_REG            REG_ADDR16(0xb00)
#define DFIFO0_SIZE_REG            REG_ADDR8(0xb02)
#define DFIFO0_ADDRHI_REG          REG_ADDR8(0xb03)

#define DFIFO1_ADDR_REG            REG_ADDR16(0xb04)
#define DFIFO1_SIZE_REG            REG_ADDR8(0xb06)
#define DFIFO1_ADDRHI_REG          REG_ADDR8(0xb07)

#define DFIFO2_ADDR_REG            REG_ADDR16(0xb08)
#define DFIFO2_SIZE_REG            REG_ADDR8(0xb0a)
#define DFIFO2_ADDRHI_REG          REG_ADDR8(0xb0b)

#define DFIFO_ADC_ADDR_REG         DFIFO2_ADDR_REG
#define DFIFO_ADC_SIZE_REG         DFIFO2_SIZE_REG
#define DFIFO_ADC_ADDRHI_REG       DFIFO0_ADDRHI_REG

/* DFIFO mode register definition */

#define DFIFO_MODE_REG             REG_ADDR8(0xb10)

/* DFIFO read and write buffer register definition */

#define DFIFO0_RPTR_REG            REG_ADDR16(0xb14)
#define DFIFO0_WPTR_REG            REG_ADDR16(0xb16)

#define DFIFO1_RPTR_REG            REG_ADDR16(0xb18)
#define DFIFO1_WPTR_REG            REG_ADDR16(0xb1a)

#define DFIFO2_RPTR_REG            REG_ADDR16(0xb1c)
#define DFIFO2_WPTR_REG            REG_ADDR16(0xb1e)

/* DFIFO mode bit definition */

#define DFIFO_MODE_DFIFO0_IN       BIT(0)
#define DFIFO_MODE_DFIFO1_IN       BIT(1)
#define DFIFO_MODE_DFIFO2_IN       BIT(2)
#define DFIFO_MODE_DFIFO0_OUT      BIT(3)
#define DFIFO_MODE_DFIFO0_L_INT    BIT(4)
#define DFIFO_MODE_DFIFO0_H_INT    BIT(5)
#define DFIFO_MODE_DFIFO1_H_INT    BIT(6)
#define DFIFO_MODE_DFIFO2_H_INT    BIT(7)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_DFIFO_H */
