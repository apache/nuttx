/****************************************************************************
 * include/nuttx/linker/common-rom.ld
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

/* Read-only iterable sections.  Boards opt in by adding, INSIDE their
 * read-only output section (typically .text, before _etext):
 *
 *   #include <nuttx/linker/common-rom.ld>
 *
 * Every block below is guarded by its subsystem's Kconfig option, so this
 * file expands to nothing on configurations that do not use iterable
 * sections (zero binary impact).
 */

#include <nuttx/config.h>
#include <nuttx/linker/iterable_sections.ld>

/* When CONFIG_ZBUS_LINKER_INSERT is selected the zbus sections come from
 * the supplementary script <nuttx/linker/zbus.ld> instead (appended to
 * the link via EXTRALINKCMDS), so they must not be emitted here too.
 */

#if defined(CONFIG_ZBUS) && !defined(CONFIG_ZBUS_LINKER_INSERT)
ITERABLE_SECTION(zbus_channel)
ITERABLE_SECTION(zbus_observer)
ITERABLE_SECTION(zbus_channel_observation)
#endif
