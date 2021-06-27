/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_extraheaps.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "hardware/esp32c3_soc.h"

#ifdef CONFIG_ESP32C3_RTC_HEAP
#include "esp32c3_rtcheap.h"
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

void up_extraheaps_init()
{
#ifdef CONFIG_ESP32C3_RTC_HEAP
  /* Initialize the RTC heap */

  esp32c3_rtcheap_initialize();
#endif
}

