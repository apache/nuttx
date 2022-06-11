/****************************************************************************
 * arch/ceva/src/xc5/up_hardfault.c
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

#include <assert.h>
#include <debug.h>

#include "cpm.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_P_MAPAR                             0x00a0
#define REG_P_MAPSR                             0x00a4
#define REG_UOP_STS                             0x0238
#define REG_UOP_PAR                             0x023c
#define REG_DBG_GEN                             0x028c

#ifdef CONFIG_DEBUG_HARDFAULT
# define hfalert(format, ...)  _alert(format, ##__VA_ARGS__)
#else
# define hfalert(x...)
#endif

#define hfdumpreg1(reg)                           \
  hfalert("%s: %08x\n",                           \
          #reg, getcpm(REG_##reg))

#define hfdumpreg2(reg1, reg2)                    \
  hfalert("%s: %08x %s: %08x\n",                  \
          #reg1, getcpm(REG_##reg1),              \
          #reg2, getcpm(REG_##reg2))

#define hfdumpreg3(reg1, reg2, reg3)              \
  hfalert("%s: %08x %s: %08x %s: %08x\n",         \
          #reg1, getcpm(REG_##reg1),              \
          #reg2, getcpm(REG_##reg2),              \
          #reg3, getcpm(REG_##reg3))

#define hfdumpreg4(reg1, reg2, reg3, reg4)        \
  hfalert("%s: %08x %s: %08x %s: %08x %s: %08x\n",\
          #reg1, getcpm(REG_##reg1),              \
          #reg2, getcpm(REG_##reg2),              \
          #reg3, getcpm(REG_##reg3),              \
          #reg4, getcpm(REG_##reg4))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_hardfault
 *
 * Description:
 *   This is Hard Fault exception handler.
 *
 ****************************************************************************/

int up_hardfault(int irq, void *context, void *arg)
{
  /* Dump some hard fault info */

  hfalert("Hard Fault:\n");
  hfdumpreg4(P_MAPAR, P_MAPSR, UOP_STS, UOP_PAR);
  hfdumpreg1(REG_DBG_GEN);

  PANIC();
  return OK;
}
