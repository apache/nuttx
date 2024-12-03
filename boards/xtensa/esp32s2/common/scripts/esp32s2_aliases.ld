/****************************************************************************
 * boards/xtensa/esp32s2/common/scripts/esp32s2_aliases.ld
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

#include <nuttx/config.h>

/* Lower-case aliases for symbols not compliant to nxstyle */

PROVIDE( cache_allocate_sram = Cache_Allocate_SRAM );
PROVIDE( cache_invalidate_icache_all = Cache_Invalidate_ICache_All );
PROVIDE( cache_resume_dcache = Cache_Resume_DCache );
PROVIDE( cache_resume_icache = Cache_Resume_ICache );
PROVIDE( cache_resume_icache = Cache_Resume_ICache );
PROVIDE( cache_set_icache_mode = Cache_Set_ICache_Mode );
PROVIDE( cache_suspend_dcache = Cache_Suspend_DCache );
PROVIDE( cache_suspend_icache = Cache_Suspend_ICache );
PROVIDE( rom_i2c_readreg = rom_i2c_readReg );
PROVIDE( rom_i2c_readreg_mask = rom_i2c_readReg_Mask );
PROVIDE( rom_i2c_writereg = rom_i2c_writeReg );
PROVIDE( rom_i2c_writereg_mask = rom_i2c_writeReg_Mask );
