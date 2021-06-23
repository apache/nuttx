/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_corepwm.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_COREPWM_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_COREPWM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CorePWM features *********************************************************/

#define MPFS_MAX_PWM_CHANNELS 16

/* Register Base Address ****************************************************/

#define MPFS_COREPWM0_BASE                    (CONFIG_MPFS_COREPWM0_BASE)
#define MPFS_COREPWM1_BASE                    (CONFIG_MPFS_COREPWM1_BASE)

/* Register offsets *********************************************************/

#define MPFS_COREPWM_PRESCALE_OFFSET           (0x00)
#define MPFS_COREPWM_PERIOD_OFFSET             (0x04)
#define MPFS_COREPWM_PWM_ENABLE_0_7_OFFSET     (0x08)
#define MPFS_COREPWM_PWM_ENABLE_8_15_OFFSET    (0x0C)
#define MPFS_COREPWM_PWM1_POS_EDGE_OFFSET      (0x10)
#define MPFS_COREPWM_PWM1_NEG_EDGE_OFFSET      (0x14)
#define MPFS_COREPWM_PWM2_POS_EDGE_OFFSET      (0x18)
#define MPFS_COREPWM_PWM2_NEG_EDGE_OFFSET      (0x1C)
#define MPFS_COREPWM_PWM3_POS_EDGE_OFFSET      (0x20)
#define MPFS_COREPWM_PWM3_NEG_EDGE_OFFSET      (0x24)
#define MPFS_COREPWM_PWM4_POS_EDGE_OFFSET      (0x28)
#define MPFS_COREPWM_PWM4_NEG_EDGE_OFFSET      (0x2C)
#define MPFS_COREPWM_PWM5_POS_EDGE_OFFSET      (0x30)
#define MPFS_COREPWM_PWM5_NEG_EDGE_OFFSET      (0x34)
#define MPFS_COREPWM_PWM6_POS_EDGE_OFFSET      (0x38)
#define MPFS_COREPWM_PWM6_NEG_EDGE_OFFSET      (0x3C)
#define MPFS_COREPWM_PWM7_POS_EDGE_OFFSET      (0x40)
#define MPFS_COREPWM_PWM7_NEG_EDGE_OFFSET      (0x44)
#define MPFS_COREPWM_PWM8_POS_EDGE_OFFSET      (0x48)
#define MPFS_COREPWM_PWM8_NEG_EDGE_OFFSET      (0x4C)
#define MPFS_COREPWM_PWM9_POS_EDGE_OFFSET      (0x50)
#define MPFS_COREPWM_PWM9_NEG_EDGE_OFFSET      (0x54)
#define MPFS_COREPWM_PWM10_POS_EDGE_OFFSET     (0x58)
#define MPFS_COREPWM_PWM10_NEG_EDGE_OFFSET     (0x5C)
#define MPFS_COREPWM_PWM11_POS_EDGE_OFFSET     (0x60)
#define MPFS_COREPWM_PWM11_NEG_EDGE_OFFSET     (0x64)
#define MPFS_COREPWM_PWM12_POS_EDGE_OFFSET     (0x68)
#define MPFS_COREPWM_PWM12_NEG_EDGE_OFFSET     (0x6C)
#define MPFS_COREPWM_PWM13_POS_EDGE_OFFSET     (0x70)
#define MPFS_COREPWM_PWM13_NEG_EDGE_OFFSET     (0x74)
#define MPFS_COREPWM_PWM14_POS_EDGE_OFFSET     (0x78)
#define MPFS_COREPWM_PWM14_NEG_EDGE_OFFSET     (0x7C)
#define MPFS_COREPWM_PWM15_POS_EDGE_OFFSET     (0x80)
#define MPFS_COREPWM_PWM15_NEG_EDGE_OFFSET     (0x84)
#define MPFS_COREPWM_PWM16_POS_EDGE_OFFSET     (0x88)
#define MPFS_COREPWM_PWM16_NEG_EDGE_OFFSET     (0x8C)
#define MPFS_COREPWM_STRETCH_OFFSET            (0x90)
#define MPFS_COREPWM_TACHPRESCALE_OFFSET       (0x94)
#define MPFS_COREPWM_TACHSTATUS_OFFSET         (0x98)
#define MPFS_COREPWM_TACHIRQMASK_OFFSET        (0x9C)
#define MPFS_COREPWM_TACHMODE_OFFSET           (0xA0)
#define MPFS_COREPWM_TACHPULSEDUR_0_OFFSET     (0xA4)
#define MPFS_COREPWM_TACHPULSEDUR_1_OFFSET     (0xA8)
#define MPFS_COREPWM_TACHPULSEDUR_2_OFFSET     (0xAC)
#define MPFS_COREPWM_TACHPULSEDUR_3_OFFSET     (0xB0)
#define MPFS_COREPWM_TACHPULSEDUR_4_OFFSET     (0xB4)
#define MPFS_COREPWM_TACHPULSEDUR_5_OFFSET     (0xB8)
#define MPFS_COREPWM_TACHPULSEDUR_6_OFFSET     (0xBC)
#define MPFS_COREPWM_TACHPULSEDUR_7_OFFSET     (0xC0)
#define MPFS_COREPWM_TACHPULSEDUR_8_OFFSET     (0xC4)
#define MPFS_COREPWM_TACHPULSEDUR_9_OFFSET     (0xC8)
#define MPFS_COREPWM_TACHPULSEDUR_10_OFFSET    (0xCC)
#define MPFS_COREPWM_TACHPULSEDUR_11_OFFSET    (0xD0)
#define MPFS_COREPWM_TACHPULSEDUR_12_OFFSET    (0xD4)
#define MPFS_COREPWM_TACHPULSEDUR_13_OFFSET    (0xD8)
#define MPFS_COREPWM_TACHPULSEDUR_14_OFFSET    (0xDC)
#define MPFS_COREPWM_TACHPULSEDUR_15_OFFSET    (0xE0)
#define MPFS_COREPWM_SYNC_UPDATE_OFFSET        (0xE4)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_COREPWM_H */
