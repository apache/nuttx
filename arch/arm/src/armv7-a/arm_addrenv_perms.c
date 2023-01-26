/****************************************************************************
 * arch/arm/src/armv7-a/arm_addrenv_perms.c
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
#include <nuttx/compiler.h>

#include <sys/mman.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_mprot
 *
 * Description:
 *   Modify access rights to an address range.
 *
 * Input Parameters:
 *   addrenv - The address environment to be modified.
 *   addr - Base address of the region.
 *   len - Size of the region.
 *   prot - Access right flags.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_mprot(arch_addrenv_t *addrenv, uintptr_t addr, size_t len,
                     int prot)
{
  /* Nothing needs to be done */

  UNUSED(addrenv);
  UNUSED(addr);
  UNUSED(len);
  UNUSED(prot);

  return OK;
}
