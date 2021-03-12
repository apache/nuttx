/****************************************************************************
 * arch/risc-v/src/fe310/hardware/fe310_gpio.h
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

#ifndef __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_GPIO_H
#define __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FE310_GPIO_INPUT_VAL   (FE310_GPIO_BASE + 0x00)
#define FE310_GPIO_INPUT_EN    (FE310_GPIO_BASE + 0x04)
#define FE310_GPIO_OUTPUT_EN   (FE310_GPIO_BASE + 0x08)
#define FE310_GPIO_OUTPUT_VAL  (FE310_GPIO_BASE + 0x0c)
#define FE310_GPIO_PU_EN       (FE310_GPIO_BASE + 0x10)
#define FE310_GPIO_DS          (FE310_GPIO_BASE + 0x14)
#define FE310_GPIO_RISE_IE     (FE310_GPIO_BASE + 0x18)
#define FE310_GPIO_RISE_IP     (FE310_GPIO_BASE + 0x1c)
#define FE310_GPIO_FALL_IE     (FE310_GPIO_BASE + 0x20)
#define FE310_GPIO_FALL_IP     (FE310_GPIO_BASE + 0x24)
#define FE310_GPIO_HIGH_IE     (FE310_GPIO_BASE + 0x28)
#define FE310_GPIO_HIGH_IP     (FE310_GPIO_BASE + 0x2c)
#define FE310_GPIO_LOW_IE      (FE310_GPIO_BASE + 0x30)
#define FE310_GPIO_LOW_IP      (FE310_GPIO_BASE + 0x34)
#define FE310_GPIO_IOF_EN      (FE310_GPIO_BASE + 0x38)
#define FE310_GPIO_IOF_SEL     (FE310_GPIO_BASE + 0x3c)
#define FE310_GPIO_OUTPUT_XOR  (FE310_GPIO_BASE + 0x40)

#endif /* __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_GPIO_H */
