/****************************************************************************
 * include/nuttx/linker/zbus.ld
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

/* Supplementary linker script providing the zbus iterable sections without
 * editing the board linker script ("zero-touch" mode).  Selected by
 * CONFIG_ZBUS_LINKER_INSERT, which adds this file to ARCHSCRIPT (see
 * tools/Config.mk).  The INSERT command makes this script supplement --
 * not replace -- the board script.
 *
 * Constraints of this mode:
 *  - GNU ld only (INSERT is not supported by macOS ld64), and the INSERT
 *    script must come BEFORE the board script on the command line (the
 *    ARCHSCRIPT hook guarantees that ordering).
 *  - The board script must define an output section named ".text".
 *  - The ROM/flash region must be the first MEMORY region compatible
 *    with read-only sections: GNU ld assigns the INSERTed section to a
 *    region by attribute matching in declaration order, so a board that
 *    declares a generic rwx region at a lower address first (e.g. ITCM
 *    at 0x0) would pull these sections into the wrong region.  Such
 *    boards must use the <nuttx/linker/common-rom.ld> include instead.
 *
 * Boards that include <nuttx/linker/common-rom.ld> in their script should
 * NOT enable CONFIG_ZBUS_LINKER_INSERT (the common-rom.ld zbus block is
 * disabled when this mode is selected, so enabling it by mistake is
 * harmless but pointless).
 */

SECTIONS
{
    .zbus : SUBALIGN(4)
    {
        . = ALIGN(4);
        _zbus_channel_list_start = .;
        KEEP(*(SORT_BY_NAME(._zbus_channel.static.*)));
        _zbus_channel_list_end = .;

        . = ALIGN(4);
        _zbus_observer_list_start = .;
        KEEP(*(SORT_BY_NAME(._zbus_observer.static.*)));
        _zbus_observer_list_end = .;

        . = ALIGN(4);
        _zbus_channel_observation_list_start = .;
        KEEP(*(SORT_BY_NAME(._zbus_channel_observation.static.*)));
        _zbus_channel_observation_list_end = .;
    }
}
INSERT AFTER .text;
