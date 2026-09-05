/****************************************************************************
 * include/nuttx/linker/common-ram.ld
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

/* Mutable (initialized) iterable sections.  Boards opt in by adding,
 * INSIDE their .data output section (between _sdata and _edata, so the
 * startup FLASH-to-RAM copy initializes the entries):
 *
 *   #include <nuttx/linker/common-ram.ld>
 *
 * Every block below is guarded by its subsystem's Kconfig option, so this
 * file expands to nothing on configurations that do not use iterable
 * sections (zero binary impact).
 */

#include <nuttx/config.h>
#include <nuttx/linker/iterable_sections.ld>

#ifndef CONFIG_ITERABLE_SECTIONS_LINKER_INSERT

/* Extension point for subsystems that need initialized RAM iterable
 * sections; blocks are added below, each guarded by its Kconfig option.
 */

/* zbus needs no RAM iterable sections: notification masks live in .bss
 * and are initialized at runtime from ROM-preserved values.
 */

#endif /* !CONFIG_ITERABLE_SECTIONS_LINKER_INSERT */
