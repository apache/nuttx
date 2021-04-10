/****************************************************************************
 * drivers/audio/vs1053.h
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

#ifndef __DRIVERS_AUDIO_VS1053_H
#define __DRIVERS_AUDIO_VS1053_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SCI Instruction Opcodes **************************************************/

#define VS1053_OPCODE_READ          3
#define VS1053_OPCODE_WRITE         2

/* SCI Registers on the SPI bus *********************************************/

#define VS1053_SCI_MODE             0x00
#define VS1053_SCI_STATUS           0x01
#define VS1053_SCI_BASS             0x02
#define VS1053_SCI_CLOCKF           0x03
#define VS1053_SCI_DECODE_TIME      0x04
#define VS1053_SCI_AUDATA           0x05
#define VS1053_SCI_WRAM             0x06
#define VS1053_SCI_WRAMADDR         0x07
#define VS1053_SCI_HDAT0            0x08
#define VS1053_SCI_HDAT1            0x09
#define VS1053_SCI_AIADDR           0x0A
#define VS1053_SCI_VOL              0x0B
#define VS1053_SCI_AICTRL0          0x0C
#define VS1053_SCI_AICTRL1          0x0E
#define VS1053_SCI_AICTRL2          0x0E
#define VS1053_SCI_AICTRL3          0x0F

/* MODE register bit definitions ********************************************/

#define VS1053_SM_DIFF              0x0001
#define VS1053_SM_LAYER12           0x0002
#define VS1053_SM_RESET             0x0004
#define VS1053_SM_CANCEL            0x0008
#define VS1053_SM_EARSPEAKER_LO     0x0010
#define VS1053_SM_TESTS             0x0020
#define VS1053_SM_STREAM            0x0040
#define VS1053_SM_EARSPEAKER_HI     0x0080
#define VS1053_SM_DACT              0x0100
#define VS1053_SM_SDIORD            0x0200
#define VS1053_SM_SDISHARE          0x0400
#define VS1053_SM_SDINEW            0x0800
#define VS1053_SM_ADPCM             0x1000
#define VS1053_SM_LINE1             0x4000
#define VS1053_SM_CLK_RANGE         0x8000

/* STATUS register bit definitions ******************************************/

#define VS1053_SS_DO_NOT_JUMP       0x8000
#define VS1053_SS_SWING             0x7000
#define VS1053_SS_VCM_OVERLOAD      0x0800
#define VS1053_SS_VCM_DISABLE       0x0400
#define VS1053_SS_VER               0x00F0
#define VS1053_SS_APDOWN2           0x0008
#define VS1053_SS_APDOWN1           0x0004
#define VS1053_SS_AD_CLOCK          0x0002
#define VS1053_SS_REFERENCE_SEL     0x0001

#define VS1053_VER_SHIFT            4
#define VS1053_VER_VS1001           0
#define VS1053_VER_VS1011           1
#define VS1053_VER_VS1002           2
#define VS1053_VER_VS1003           3
#define VS1053_VER_VS1053           4
#define VS1053_VER_VS1033           5
#define VS1053_VER_VS1063           6
#define VS1053_VER_VS1103           7

/* BASS register bit definitions ********************************************/

#define VS1053_ST_AMPLITUDE         0xF000
#define VS1053_ST_FREQLIMIT         0x0F00
#define VS1053_SB_AMPLITUDE         0x00F0
#define VS1053_SB_FREQLIMIT         0x000F

/* CLOCKF register bit definitions ******************************************/

#define VS1053_SC_MULT              0xE000
#define VS1053_SC_MULT_SHIFT        13
#define VS1053_SC_ADD               0x1800
#define VS1053_SC_ADD_SHIFT         11
#define VS1053_SC_FREQ              0x07FF

#define VS1053_SC_MULT_XTALI_X10     0
#define VS1053_SC_MULT_XTALI_X20     1
#define VS1053_SC_MULT_XTALI_X25     2
#define VS1053_SC_MULT_XTALI_X30     3
#define VS1053_SC_MULT_XTALI_X35     4
#define VS1053_SC_MULT_XTALI_X40     5
#define VS1053_SC_MULT_XTALI_X45     6
#define VS1053_SC_MULT_XTALI_X50     7

#define VS1053_SC_ADD_NONE           0
#define VS1053_SC_ADD_XTALI_X10      1
#define VS1053_SC_ADD_XTALI_X15      2
#define VS1053_SC_ADD_XTALI_X20      3

/* WRAM Addresses ***********************************************************/

#define VS1053_XRAM_BASE            0x1800      /* X data RAM */
#define VS1053_XRAM_SIZE            256

#define VS1053_YRAM_BASE            0x5800      /* Y data RAM */
#define VS1053_YRAM_SIZE            256

#define VS1053_IRAM_BASE            0x8040      /* Instruction RAM */
#define VS1053_IRAM_SIZE            0x460

#define VS1053_IO_BASE              0xC000
#define VS1053_IO_SIZE              0x4000

/* HDAT1 register values ****************************************************/

#define VS1053_HDAT1_WAV            0x7665      /* "ve" (as in Wave) */
#define VS1053_HDAT1_ADTS           0x4154      /* "AT" */
#define VS1053_HDAT1_ADIF           0x4144      /* "AD" */
#define VS1053_HDAT1_AAC            0x4D34      /* "M4" */
#define VS1053_HDAT1_WMA            0x574D      /* "WM" */
#define VS1053_HDAT1_MIDI           0x4D54      /* "MT" */
#define VS1053_HDAT1_OGG            0x4F67      /* "Og" */

/* MP3 special definitions for HDAT1 / 0 */

#define VS1053_HDAT1_MP3_SYNC       0xFFE0      /* Stream valid */
#define VS1053_HDAT1_MP3_ID         0x0018      /* MPG id */
#define VS1053_HDAT1_MP3_LAYER      0x0006
#define VS1053_HDAT1_MP3_PROTECT    0x0001      /* CRC Protect bit */
#define VS1053_HDAT0_MP3_BITRATE    0xF000
#define VS1053_HDAT0_MP3_SAMPRATE   0x0C00
#define VS1053_HDAT0_MP3_PAD        0x0200
#define VS1053_HDAT0_MP3_PRIVATE    0x0100
#define VS1053_HDAT0_MP3_MODE       0x00C0
#define VS1053_HDAT0_MP3_EXTENSION  0x0030
#define VS1053_HDAT0_MP3_COPYRIGHT  0x0008
#define VS1053_HDAT0_MP3_ORIGINAL   0x0004
#define VS1053_HDAT0_MP3_EMPHASIS   0x0003

#define VS1053_MP3_ID_SHIFT         7
#define VS1053_MP3_ID_MPEG1         3
#define VS1053_MP3_ID_MPEG2         2
#define VS1053_MP3_ID_MPEG25a       1
#define VS1053_MP3_ID_MPEG25b       0

#define VS1053_MP3_LAYER_SHIFT      1
#define VS1053_MP3_LAYER_I          3
#define VS1053_MP3_LAYER_II         2
#define VS1053_MP3_LAYER_III        1

#define VS1053_MP3_CRC_NONE         1
#define VS1053_MP3_CRC_PROTECT      0

#define VS1053_MP3_PAD_SHIFT        9
#define VS1053_MP3_PAD_ADDITIONAL   1
#define VS1053_MP3_PAD_NORMAL       0

#define VS1053_MP3_MODE_SHIFT       6
#define VS1053_MP3_MODE_MONO        3
#define VS1053_MP3_MODE_DUAL_CH     2
#define VS1053_MP3_MODE_JSTEREO     1
#define VS1053_MP3_MODE_STEREO      0

#define VS1053_END_FILL_BYTE        0x1e06

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_AUDIO */
#endif /* __DRIVERS_AUDIO_VS1053_H */
