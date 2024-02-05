/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_textheap.c
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
#include <nuttx/kmalloc.h>

#include "hardware/esp32s3_soc.h"

#ifdef CONFIG_ESP32S3_RTC_HEAP
#  include "esp32s3_rtcheap.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ESP32S3_RTC_HEAP
#error "No suitable heap available. Enable ESP32S3_RTC_HEAP."
#endif

#define D_I_BUS_OFFSET  0x700000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_textheap_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space in text heap.
 *
 *   The alignment argument must be a power of two (not checked). 8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 * Parameters:
 *   align - Requested alignment.
 *   size  - Size (in bytes) of the memory region to be allocated.
 *
 * Return Value:
 *   Address of the allocated address. NULL, if allocation fails.
 *
 ****************************************************************************/

void *up_textheap_memalign(size_t align, size_t size)
{
  void *ret = NULL;

  /* Prioritise allocating from RTC. If that fails, allocate from the
   * main heap.
   */

#ifdef CONFIG_ESP32S3_RTC_HEAP
  ret = esp32s3_rtcheap_memalign(align, size);
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
 * Name: up_textheap_free
 *
 * Description:
 *   Free memory from the text heap.
 *
 * Parameters:
 *   mem - Address to be freed.
 *
 ****************************************************************************/

void up_textheap_free(void *p)
{
  if (p)
    {
#ifdef CONFIG_ESP32S3_RTC_HEAP
      if (esp32s3_ptr_rtc(p))
        {
          esp32s3_rtcheap_free(p);
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
 * Name: up_textheap_heapmember
 *
 * Description:
 *   Check if an address lies in text heap.
 *
 * Parameters:
 *   mem - The address to check.
 *
 * Return Value:
 *   True if the address is a member of the text heap. False if not.
 *
 ****************************************************************************/

bool up_textheap_heapmember(void *p)
{
  if (p == NULL)
    {
      return false;
    }

#ifdef CONFIG_ESP32S3_RTC_HEAP
  if (esp32s3_ptr_rtc(p))
    {
      return esp32s3_rtcheap_heapmember(p);
    }
#endif

  p -= D_I_BUS_OFFSET;
  return kmm_heapmember(p);
}
