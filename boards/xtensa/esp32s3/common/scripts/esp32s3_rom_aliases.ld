/****************************************************************************
 * boards/xtensa/esp32s3/common/scripts/esp32s3_aliases.ld
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

PROVIDE( cache_dbus_mmu_set = Cache_Dbus_MMU_Set );
PROVIDE( cache_ibus_mmu_set = Cache_Ibus_MMU_Set );
PROVIDE( cache_invalidate_addr = Cache_Invalidate_Addr );
PROVIDE( cache_invalidate_dcache_all = Cache_Invalidate_DCache_All );
PROVIDE( cache_invalidate_icache_all = Cache_Invalidate_ICache_All );
PROVIDE( cache_occupy_addr = Cache_Occupy_Addr );
PROVIDE( cache_resume_dcache = Cache_Resume_DCache );
PROVIDE( cache_resume_icache = Cache_Resume_ICache );
PROVIDE( cache_set_idrom_mmu_info = Cache_Set_IDROM_MMU_Info );
PROVIDE( cache_set_idrom_mmu_size = Cache_Set_IDROM_MMU_Size );
PROVIDE( cache_suspend_dcache = Cache_Suspend_DCache );
PROVIDE( cache_suspend_icache = Cache_Suspend_ICache );
PROVIDE( cache_writeback_all = Cache_WriteBack_All );
PROVIDE( cache_writeback_items = Cache_WriteBack_Items );
PROVIDE( cache_writeback_addr = Cache_WriteBack_Addr );
