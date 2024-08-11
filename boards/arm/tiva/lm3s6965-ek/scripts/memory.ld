/****************************************************************************
 * boards/arm/tiva/lm3s6965-ek/scripts/memory.ld
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

/* The LM3S6965 has 256Kb of FLASH beginning at address 0x0000:0000 and 64Kb
 * of SRAM beginning at 0x2000:0000.
 */

MEMORY
{
    /* 256Kb FLASH */

  kflash (rx)      : ORIGIN = 0x00000000, LENGTH = 128K
  uflash (rx)      : ORIGIN = 0x00020000, LENGTH = 128K
  xflash (rx)      : ORIGIN = 0x00040000, LENGTH = 0K

    /* 64Kb of contiguous SRAM */

  ksram (rwx)      : ORIGIN = 0x20000000, LENGTH = 16K
  usram (rwx)      : ORIGIN = 0x20004000, LENGTH = 16K
  xsram (rwx)      : ORIGIN = 0x20008000, LENGTH = 32K
}
