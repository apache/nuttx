/****************************************************************************
 * drivers/addrenv.c
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

#include <string.h>

#include <nuttx/drivers/addrenv.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct simple_addrenv_s g_addrenv_dummy;
static const struct simple_addrenv_s *g_addrenv = &g_addrenv_dummy;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void simple_addrenv_initialize(const struct simple_addrenv_s *addrenv)
{
  g_addrenv = addrenv;
}

void *up_addrenv_pa_to_va(uintptr_t pa)
{
  uint32_t i;

  for (i = 0; g_addrenv[i].size; i++)
    {
      if (pa - g_addrenv[i].pa < g_addrenv[i].size)
        {
          return (void *)(g_addrenv[i].va + B2C(pa - g_addrenv[i].pa));
        }
    }

  return (void *)B2C(pa);
}

uintptr_t up_addrenv_va_to_pa(void *va_)
{
  uintptr_t va = C2B((uintptr_t)va_);
  uint32_t i;

  for (i = 0; g_addrenv[i].size; i++)
    {
      uintptr_t tmp = C2B(g_addrenv[i].va);
      if (va - tmp < g_addrenv[i].size)
        {
          return g_addrenv[i].pa + (va - tmp);
        }
    }

  return va;
}
