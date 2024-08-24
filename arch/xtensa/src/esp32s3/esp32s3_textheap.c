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

#include "hal/cache_hal.h"
#include "hardware/esp32s3_soc.h"

#ifdef CONFIG_ESP32S3_RTC_HEAP
#  include "esp32s3_rtcheap.h"
#endif
#include "esp32s3_spiram.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EXTRAM_INSTRUCTION_BUS_LOW  0x42000000
#define EXTRAM_INSTRUCTION_BUS_HIGH 0x44000000

#define EXTRAM_D_I_BUS_OFFSET  0x6000000

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

          uintptr_t addr = (uintptr_t)ret;
          if (SOC_DIRAM_DRAM_LOW <= addr && addr < SOC_DIRAM_DRAM_HIGH)
            {
              addr = MAP_DRAM_TO_IRAM(addr);
            }
          else
            {
              /* extram */

              addr += EXTRAM_D_I_BUS_OFFSET;
            }

          ret = (void *)addr;
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
          p = up_textheap_data_address(p);
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

  p = up_textheap_data_address(p);
  return kmm_heapmember(p);
}

/****************************************************************************
 * Name: up_textheap_data_address
 *
 * Description:
 *   If an instruction bus address is specified, return the corresponding
 *   data bus address. Otherwise, return the given address as it is.
 *
 *   For some platforms, up_textheap_memalign() might return memory regions
 *   with separate instruction/data bus mappings. In that case,
 *   up_textheap_memalign() returns the address of the instruction bus
 *   mapping.
 *   The instruction bus mapping might provide only limited data access.
 *   (For example, only read-only, word-aligned access.)
 *   You can use up_textheap_data_address() to query the corresponding data
 *   bus mapping.
 *
 ****************************************************************************/

void *up_textheap_data_address(void *p)
{
  uintptr_t addr = (uintptr_t)p;
  if (SOC_DIRAM_IRAM_LOW <= addr && addr < SOC_DIRAM_IRAM_HIGH)
    {
      addr = MAP_IRAM_TO_DRAM(addr);
    }
  else if (EXTRAM_INSTRUCTION_BUS_LOW <= addr &&
           addr < EXTRAM_INSTRUCTION_BUS_HIGH)
    {
      /* extram */

      addr -= EXTRAM_D_I_BUS_OFFSET;
    }

  return (void *)addr;
}

/****************************************************************************
 * Name: up_textheap_data_sync
 *
 * Description:
 *   Ensure modifications made on the data bus addresses (the addresses
 *   returned by up_textheap_data_address) fully visible on the corresponding
 *   instruction bus addresses.
 *
 ****************************************************************************/

IRAM_ATTR void up_textheap_data_sync(void)
{
  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_ESP32S3_SPIRAM
  esp_spiram_writeback_cache();
#endif
  cache_hal_disable(CACHE_TYPE_INSTRUCTION);
  cache_hal_enable(CACHE_TYPE_INSTRUCTION);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_copy_section
 *
 * Description:
 *   This function copies a section from a general temporary buffer (src) to
 *   a specific address (dest). This is typically used in architectures that
 *   require specific handling of memory sections.
 *
 * Input Parameters:
 *   dest - A pointer to the destination where the data needs to be copied.
 *   src  - A pointer to the source from where the data needs to be copied.
 *   n    - The number of bytes to be copied from src to dest.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_USE_COPY_SECTION)
int up_copy_section(void *dest, const void *src, size_t n)
{
  memcpy(up_textheap_data_address(dest), src, n);

  return OK;
}
#endif
