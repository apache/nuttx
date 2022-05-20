/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_textheap.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYSBUS_ADDRESS_OFFSET 0x20000000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_textheap_memalign()
 ****************************************************************************/

void *up_textheap_memalign(size_t align, size_t size)
{
  void *ret;
  ret = (void *)kmm_malloc(size);

#ifdef CONFIG_CXD56_USE_SYSBUS
  if (ret)
    {
      binfo("** ret=%p\n", ret);

      /* NOTE:
       * kmm_malloc() will return the address in SYSBUS.
       * So convert the address to I/D BUS.
       */

      ret -= SYSBUS_ADDRESS_OFFSET;

      binfo("** mapped to %p\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: up_textheap_free()
 ****************************************************************************/

void up_textheap_free(void *p)
{
#ifdef CONFIG_CXD56_USE_SYSBUS
  if (p)
    {
      binfo("** p=%p\n", p);

      /* NOTE:
       * The address p will be in I/D BUS.
       * So convert the address to SYSBUS.
       */

      p += SYSBUS_ADDRESS_OFFSET;

      binfo("** mapped to %p\n", p);
    }
#endif

  kmm_free(p);
}

/****************************************************************************
 * Name: up_textheap_heapmember()
 ****************************************************************************/

bool up_textheap_heapmember(void *p)
{
  if (p == NULL)
    {
      return false;
    }

#ifdef CONFIG_CXD56_USE_SYSBUS
  p += SYSBUS_ADDRESS_OFFSET;
#endif

  return kmm_heapmember(p);
}
