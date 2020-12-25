/****************************************************************************
 * arch/xtensa/src/esp32/esp32_resetcause.c
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum esp32_resetcause_e
{
  ESP32_RESETCAUSE_SYS_CHIPPOR = 0x01,
  ESP32_RESETCAUSE_SYS_RWDTSR  = 0x10,
  ESP32_RESETCAUSE_SYS_BOR     = 0x0f,
  ESP32_RESETCAUSE_CORE_SOFT   = 0x03,
  ESP32_RESETCAUSE_CORE_DPSP   = 0x05,
  ESP32_RESETCAUSE_CORE_MWDT0  = 0x07,
  ESP32_RESETCAUSE_CORE_MWDT1  = 0x08,
  ESP32_RESETCAUSE_CORE_RWDT   = 0x09,
  ESP32_RESETCAUSE_CPU_MWDT0   = 0x0b,
  ESP32_RESETCAUSE_CPU_SOFT    = 0x0c,
  ESP32_RESETCAUSE_CPU_RWDT    = 0x0d,
  ESP32_RESETCAUSE_CPU_PROCPU  = 0x0e
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_resetcause
 *
 * Description:
 *   Get the cause of the last reset of the given CPU
 *
 ****************************************************************************/

enum esp32_resetcause_e esp32_resetcause(int cpu);

