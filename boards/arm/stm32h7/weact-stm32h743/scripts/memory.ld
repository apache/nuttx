/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/scripts/memory.ld
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

/* TODO: Add description for memory organisation */

MEMORY
{
  /* ITCM boot address */

  itcm  (rwx) : ORIGIN = 0x00000000, LENGTH = 64K

  /* 2048Kb FLASH */

  kflash (rx) : ORIGIN = 0x08000000, LENGTH = 128K
  uflash (rx) : ORIGIN = 0x08020000, LENGTH = 128K
  xflash (rx) : ORIGIN = 0x08040000, LENGTH = 1792K

  /* 288Kb SRAM123 */

  ksram (rwx) : ORIGIN = 0x30000000, LENGTH = 16K
  usram (rwx) : ORIGIN = 0x30004000, LENGTH = 16K
  xsram (rwx) : ORIGIN = 0x30008000, LENGTH = 288K - 32K

  /* 512Kb of contiguous AXI SRAM */

  sram (rwx)  : ORIGIN = 0x24000000, LENGTH = 512K

  /* DTCM SRAM */

  dtcm1 (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
  dtcm2 (rwx) : ORIGIN = 0x20010000, LENGTH = 64K

  sram4 (rwx) : ORIGIN = 0x38000000, LENGTH = 64K
  bbram (rwx) : ORIGIN = 0x38800000, LENGTH = 4K
}
