/****************************************************************************
 * include/nuttx/linker/common-insert.ld
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

/* Supplementary linker script for the iterable sections "zero-touch" mode
 * (CONFIG_ITERABLE_SECTIONS_LINKER_INSERT): added before the board linker
 * script by the build system, it supplements -- does not replace -- that
 * script through the GNU ld INSERT command, so boards need no edit.  The
 * read-only iterable sections are collected in one output section placed
 * right after .text.
 *
 * Subsystem blocks are added inside the output section below, each
 * guarded by its Kconfig option (same layout as common-rom.ld):
 *
 *   #ifdef CONFIG_MYSUBSYS
 *   ITERABLE_SECTION(mysubsys_entry)
 *   #endif
 *
 * Constraints of this mode:
 *  - GNU ld only (INSERT is not supported by the macOS ld64), and this
 *    script must come BEFORE the board script on the command line (the
 *    build system guarantees that ordering).
 *  - The board script must define an output section named ".text".
 *  - The ROM/flash region must be the first MEMORY region compatible
 *    with read-only sections: GNU ld assigns the INSERTed section to a
 *    region by attribute matching in declaration order, so a board that
 *    declares a generic rwx region at a lower address first (e.g. ITCM
 *    at 0x0) would pull these sections into the wrong region.  Such
 *    boards must use the <nuttx/linker/common-rom.ld> include instead.
 */

#include <nuttx/config.h>
#include <nuttx/linker/iterable_sections.ld>

SECTIONS
{
    .iterable_sections : SUBALIGN(4)
    {
#ifdef CONFIG_ZBUS
        ITERABLE_SECTION(zbus_channel)
        ITERABLE_SECTION(zbus_observer)
        ITERABLE_SECTION(zbus_channel_observation)
#endif
    }
}
INSERT AFTER .text;
