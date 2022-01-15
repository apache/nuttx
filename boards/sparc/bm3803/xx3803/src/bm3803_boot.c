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

#include "up_arch.h"
#include "up_internal.h"

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
  BM3803_REG.Mecfg2 = 0x0;
  BM3803_REG.Mecfg3 = 0x0;
  BM3803_REG.Mecfg1 = 0xF0038000;

  BM3803_REG.MemCfg1 = 0x14F9F91F;
  BM3803_REG.MemCfg2 = 0x00078C67;
  BM3803_REG.MemCfg3 = 0x0;
  //BM3803_REG.CacheCtrl = 0x0;

  BM3803_REG.TimerCtrl1 = 0;
  BM3803_REG.TimerCnt1 = 0;
  BM3803_REG.TimerLoad1 = 0;
}
