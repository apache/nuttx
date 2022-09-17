/****************************************************************************
 * arch/arm/src/common/arm_link_steering.c
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
#include "arm_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern char Image$$text$$Base[];
extern char Image$$text$$Limit[];
extern char Image$$eronly$$Base[];
extern char Image$$data$$Base[];
extern char Image$$data$$RW$$Limit[];
extern char Image$$bss$$Base[];
extern char Image$$bss$$ZI$$Limit[];
extern char Image$$tdata$$Base[];
extern char Image$$tdata$$Limit[];
extern char Image$$tbss$$Base[];
extern char Image$$tbss$$Limit[];
extern char Image$$init_section$$Base[];
extern char Image$$init_section$$Limit[];
extern char Image$$noinit$$Base[];
extern char Image$$noinit$$Limit[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __armlink_symbol_steering
 *
 * Description:
 *   This function allows nuttx to keep using existing linker
 *   symbols defined in scatter and used throughout the code tree.
 *
 ****************************************************************************/

int __armlink_symbol_steering(void)
{
  return (int)Image$$text$$Base
       & (int)Image$$text$$Limit
       & (int)Image$$eronly$$Base
       & (int)Image$$data$$Base
       & (int)Image$$data$$RW$$Limit
       & (int)Image$$bss$$Base
       & (int)Image$$bss$$ZI$$Limit
       & (int)Image$$tdata$$Base
       & (int)Image$$tdata$$Limit
       & (int)Image$$tbss$$Base
       & (int)Image$$tbss$$Limit
       & (int)Image$$init_section$$Base
       & (int)Image$$init_section$$Limit
       & (int)Image$$noinit$$Base
       & (int)Image$$noinit$$Limit;
}
