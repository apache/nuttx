/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_modtext.c
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

#define D_I_BUS_OFFSET  0x700000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_module_text_init()
 ****************************************************************************/

void up_module_text_init()
{
}

/****************************************************************************
 * Name: up_module_text_memalign()
 ****************************************************************************/

FAR void *up_module_text_memalign(size_t align, size_t size)
{
  FAR void *ret;

  ret = kmm_memalign(align, size);
  if (ret)
    {
      ret += D_I_BUS_OFFSET;
    }

  return ret;
}

/****************************************************************************
 * Name: up_module_text_free()
 ****************************************************************************/

void up_module_text_free(FAR void *p)
{
  if (p)
    {
      p -= D_I_BUS_OFFSET;
    }

  kmm_free(p);
}
