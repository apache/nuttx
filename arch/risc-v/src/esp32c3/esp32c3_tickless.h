/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_tickless.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_TICKLESS_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_TICKLESS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_get_idletime
 *
 * Description:
 *   This function returns the idle time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The time in system ticks remaining for idle.
 *   Zero means system is busy.
 *
 ****************************************************************************/

uint32_t up_get_idletime(void);

/****************************************************************************
 * Name:  up_step_idletime
 *
 * Description:
 *   Add system time by idletime_us.
 *
 * Input Parameters:
 *   idletime_us - Idle time(us)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_step_idletime(uint32_t idletime_us);

#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_TICKLESS_H */
