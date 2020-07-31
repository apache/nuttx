/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/scripts/memory.ld
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

MEMORY
{
  /* 1024Kb FLASH (tentative) */

  kflash (rx)      : ORIGIN = 0x05000000, LENGTH = 128K
  uflash (rx)      : ORIGIN = 0x05020000, LENGTH = 128K
  xflash (rx)      : ORIGIN = 0x05040000, LENGTH = 768K

  /* 256Kb SRAM (tentative) */

  ksram (rwx)      : ORIGIN = 0x020c0000, LENGTH = 32K
  usram (rwx)      : ORIGIN = 0x020c8000, LENGTH = 32K
  xsram (rwx)      : ORIGIN = 0x020d0000, LENGTH = 192K
}
