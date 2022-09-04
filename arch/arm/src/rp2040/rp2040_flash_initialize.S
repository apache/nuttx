/****************************************************************************
 * arch/arm/src/rp2040/rp2040_flash_initialize.S
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

/****************************************************************************
 * Description:
 *   The low-level format of the smart filesystem for the rp2040 consists of
 *   a series of 4096-byte blocks each of which contain four 1024 byte
 *   logical sectors.  Due to the way flash works the filesystem can only
 *   erase (set to all 1 bits) a whole block.  It can however, write smaller
 *   amounts.  On the rp2040 the smallest write is 256 bytes.  The write can
 *   only flip 1 bits to 0 -- it cannot change a 0 bit to 1.
 *
 *   Each logical sector starts with a five byte sector header.  Most sectors
 *   follow this with a five byte filesystem header. Sector zero does not
 *   follow this rule; it contains the volume label.
 *
 *   The volume will have root directory located in logical sector three. It
 *   may have additional root directories (indicated by a count in the volume
 *   label). This will be in sequential sectors starting with sector four.
 *
 *   Actual files and other subdirectories start in logical sector twelve.
 *
 *   == SECTOR HEADER ==
 *
 *       Logical sector number:  2 bytes
 *       Sequence number:        2 bytes
 *       Flags:                  1 byte
 *
 *     Flag bits are:  C R X S S S V V
 *                C - Set to zero if block committed  0
 *                R - Set to zero if block erased     1
 *                X - CRC status                      1
 *                S - Size Code                       010
 *                V - Version code                    01
 *
 *   The logical sector number does not have to match the actual position
 *   of a sector.  The volume in scanned when mounted and the mapping of
 *   sector number to position in volume is maintained in ram.  If a sector
 *   is written to it will be re-written to a new location.
 *
 *   == VOLUME LABEL ==
 *
 *   Sector zero is the volume label.  It currently consists of the
 *   following fields.
 *
 *       Smart Volume ID:        4 bytes    "SMRT"
 *       Smart version:          1 byte      0x01
 *       Sector size flag:       1 byte      0x10
 *       Extra Root Directories: 1 byte      0x00
 *
 *   == FILE SYSTEM HEADER ==
 *
 *   File and directory sectors have a filesystem header directly following
 *   the sector header.  This filesystem header consists of:
 *
 *       Entry type:             1 byte     0x01 = Directory, 0x02 = File.
 *       Next logical sector:    2 bytes    0xff no more sectors
 *       Number of bytes used    2 bytes    0xff empty sector
 *
 *   == DIRECTORY ENTRIES ==
 *
 *       Directory Flags:        2 bytes
 *
 *   == FILE ENTRIES ==
 *
 *       File data starts following the filesystem header and extends
 *       for a number of bytes as specified in the filesystem header.
 *
 *       Files with a length of greater that 1019 bytes will span multiple
 *       sectors, linked with the next logical sector flag.
 ****************************************************************************/

dir=          1
file=         2
name_length= 16

/****************************************************************************
 * Name: sector
 *
 * Description:
 *    This macro defines a sector header.  It actually sets both the actual
 *    sector header and the filesystem header values.
 *
 * Parameters:
 *    num  - The logical sector number.  Each sector must be unique.
 *    type - The sector type.  Should be 'dir' or 'file'
 *    used - The length of the data in this sector.
 *    next - The logical sector number of the next sector in a chain.
 ****************************************************************************/

    .macro      sector      num, type, used=0xffff, next=0xffff
    .balign     1024, 0xff
    .hword      \num, 0
    .byte       0b01101001, \type
    .hword      \next, \used
    .endm

/****************************************************************************
 * Name: dir_entry
 *
 * Description:
 *    This macro defines a directory entry.
 *
 * Parameters:
 *    perm - Permission bits for this directory entry
 *    addr - Logical sector number of named entry
 *    time - entry creation time stamp
 *    name - name of this entry
 ****************************************************************************/

    .macro      dir_entry   perm, addr, time, name
    .hword      \perm | 0x7e00, \addr
    .word       \time
0:
    .ascii      "\name"
.=      0b + name_length
    .endm

/****************************************************************************
 * Name: file_entry
 *
 * Description:
 *    This macro defines a directory entry.
 *
 * Parameters:
 *    perm - Permission bits for this directory entry
 *    addr - Logical sector number of named entry
 *    time - entry creation time stamp
 *    name - name of this entry
 ****************************************************************************/

    .macro      file_entry   perm, addr, time, name
    .hword      \perm | 0x5e00, \addr
    .word       \time
0:
    .ascii      "\name"
.=      0b + name_length
    .endm

/****************************************************************************
 * Global name of the initial filesystem data
 ****************************************************************************/

 	.cpu 		cortex-m0plus
 	.thumb

    .section    .flash.init, "ax"
    .balign     4096
    .global     rp2040_smart_flash_start
rp2040_smart_flash_start:

/****************************************************************************
 * Volume Label
 ****************************************************************************/

    .ascii      "2040"       /* magic tag for flash initialization */
    .byte       0b01101001
    .ascii      "SMRT"
    .byte       0x01, 0x10, 0

    .balign     4096, 0xff

/****************************************************************************
 * Root directory and initial files in the filesystem
 ****************************************************************************/

    sector      3, dir
    file_entry  0777, 4, 0, "test"

    sector      4, file, used=14
    .ascii      "Hello, world!\n"


    .balign     4096, 0xff
    .global     rp2040_smart_flash_end
rp2040_smart_flash_end:

    .end
