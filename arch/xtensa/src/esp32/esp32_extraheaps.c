/****************************************************************************
 * arch/xtensa/src/esp32/esp32_extraheaps.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>

#include <sys/types.h>
#include <debug.h>

#include "hardware/esp32_soc.h"

#ifdef CONFIG_ESP32_IRAM_HEAP
#include "esp32_iramheap.h"
#endif

#ifdef CONFIG_ESP32_RTC_HEAP
#include "esp32_rtcheap.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_extraheaps_init
 *
 * Description:
 *   Initialize any extra heap.
 *
 ****************************************************************************/

void up_extraheaps_init(void)
{
#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
  xtensa_imm_initialize();
#endif

#ifdef CONFIG_ESP32_RTC_HEAP
  esp32_rtcheap_initialize();
#endif

#ifdef CONFIG_ESP32_IRAM_HEAP
  esp32_iramheap_initialize();
#endif
}

