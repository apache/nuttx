/****************************************************************************
 * boards/risc-v/esp32p4/common/scripts/esp32p4_aliases.ld
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

_interrupt_handler = exception_common;
cache_invalidate_all = Cache_Invalidate_All;
cache_resume_l2_cache = Cache_Resume_L2_Cache;
cache_set_idrom_mmu_size = Cache_Set_IDROM_MMU_Size;
cache_suspend_l2_cache = Cache_Suspend_L2_Cache;
