/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/bb_reg.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_BB_REG_H
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_BB_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32c3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some of the baseband control registers.
 * PU/PD fields defined here are used in sleep related functions.
 */

#define BBPD_CTRL (DR_REG_BB_BASE + 0x0054)
#define BB_FFT_FORCE_PU (BIT(3))
#define BB_FFT_FORCE_PU_M (BIT(3))
#define BB_FFT_FORCE_PU_V 1
#define BB_FFT_FORCE_PU_S 3

#define BB_DC_EST_FORCE_PU (BIT(1))
#define BB_DC_EST_FORCE_PU_M (BIT(1))
#define BB_DC_EST_FORCE_PU_V 1
#define BB_DC_EST_FORCE_PU_S 1

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_BB_REG_H */
