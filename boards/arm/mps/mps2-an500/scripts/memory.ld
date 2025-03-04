/****************************************************************************
 * boards/arm/mps/mps2-an500/scripts/memory.ld
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Specify the memory areas
 * see arch/arm/src/mps/hardware/mps_memorymap.h for more details
 */

MEMORY
{

  kflash (rx)  : ORIGIN = 0x00000000, LENGTH = 4M
  uflash (rx)  : ORIGIN = 0x20000000, LENGTH = 4M

  kocram (rwx) : ORIGIN = 0x60000000, LENGTH = 8M
  uocram (rwx) : ORIGIN = 0x60800000, LENGTH = 8M
}

_ram_size = LENGTH(kocram) + LENGTH(uocram);
