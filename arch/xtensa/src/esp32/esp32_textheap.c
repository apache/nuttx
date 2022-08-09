/****************************************************************************
 * arch/xtensa/src/esp32/esp32_textheap.c
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
#include <nuttx/fs/procfs.h>
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
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_ESP32_IRAM_HEAP) && !defined(CONFIG_ESP32_RTC_HEAP)
#error "No suitable heap available. Enable ESP32_IRAM_HEAP or ESP32_RTC_HEAP"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_textheap_memalign
 *
 * Description:
 *   Allocate memory from the text heap with the specified alignment.
 *
 ****************************************************************************/

void *up_textheap_memalign(size_t align, size_t size)
{
  void *ret = NULL;

  /* Prioritise allocating from RTC. If that fails, allocate from the
   * main heap.
   */

#ifdef CONFIG_ESP32_RTC_HEAP
  ret = esp32_rtcheap_memalign(align, size);
#endif

  if (ret == NULL)
    {
      ret = esp32_iramheap_memalign(align, size);
    }

  return ret;
}

/****************************************************************************
 * Name: up_textheap_free
 *
 * Description:
 *   Free memory from the text heap.
 *
 ****************************************************************************/

void up_textheap_free(void *p)
{
  if (p)
    {
#ifdef CONFIG_ESP32_RTC_HEAP
      if (esp32_ptr_rtcslow(p))
        {
          esp32_rtcheap_free(p);
        }
      else
#endif
        {
          esp32_iramheap_free(p);
        }
    }
}

/****************************************************************************
 * Name: up_textheap_heapmember
 *
 * Description:
 *   Test if memory is from text heap.
 *
 ****************************************************************************/

bool up_textheap_heapmember(void *p)
{
  if (p == NULL)
    {
      return false;
    }

#ifdef CONFIG_ESP32_RTC_HEAP
  if (esp32_ptr_rtcslow(p))
    {
      return esp32_rtcheap_heapmember(p);
    }
#endif

  return esp32_iramheap_heapmember(p);
}
