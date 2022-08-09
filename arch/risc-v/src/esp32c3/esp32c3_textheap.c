/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_textheap.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#define D_I_BUS_OFFSET  0x700000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_textheap_memalign()
 *
 * Description:
 *   Allocate memory for module text with the specified alignment.
 *
 ****************************************************************************/

void *up_textheap_memalign(size_t align, size_t size)
{
  void *ret = NULL;

  /* Prioritise allocating from RTC. If that fails, allocate from the
   * main heap.
   */

#ifdef CONFIG_ESP32C3_RTC_HEAP
  ret = esp32c3_rtcheap_memalign(align, size);
#endif

  if (ret == NULL)
    {
      ret = kmm_memalign(align, size);
      if (ret)
        {
          /* kmm_memalign buffer is at the Data bus offset.  Adjust it so we
           * can access it from the Instruction bus.
           */

          ret += D_I_BUS_OFFSET;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: up_textheap_free()
 *
 * Description:
 *   Free memory for module text.
 *
 ****************************************************************************/

void up_textheap_free(void *p)
{
  if (p)
    {
#ifdef CONFIG_ESP32C3_RTC_HEAP
      if (esp32c3_ptr_rtc(p))
        {
          esp32c3_rtcheap_free(p);
        }
      else
#endif
        {
          p -= D_I_BUS_OFFSET;
          kmm_free(p);
        }
    }
}

/****************************************************************************
 * Name: up_textheap_heapmember()
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

#ifdef CONFIG_ESP32C3_RTC_HEAP
  if (esp32c3_ptr_rtc(p))
    {
      return esp32c3_rtcheap_heapmember(p);
    }
#endif

  p -= D_I_BUS_OFFSET;
  return kmm_heapmember(p);
}
