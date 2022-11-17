/****************************************************************************
 * boards/sparc/bm3803/xx3803/src/bm3803_boot.c
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

#include <debug.h>

#include <arch/board/board.h>

#include "sparc_internal.h"
#include "bm3803.h"
#include "xx3803.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_boardinitialize
 *
 * Description:
 *   All bm3803 architectures must provide the following entry point.
 *   This entry point is called early in the intitialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void bm3803_boardinitialize(void)
{
  BM3803_REG.mecfg2 = 0x0;
  BM3803_REG.mecfg3 = 0x0;
  BM3803_REG.mecfg1 = 0xf0038000;

  BM3803_REG.mem_cfg1 = 0x14f9f91f;
  BM3803_REG.mem_cfg2 = 0x00078c67;
  BM3803_REG.mem_cfg3 = 0x0;

  /* BM3803_REG.CacheCtrl = 0x0; */

  BM3803_REG.timer_ctrl1 = 0;
  BM3803_REG.timer_cnt1 = 0;
  BM3803_REG.timer_load1 = 0;
}
