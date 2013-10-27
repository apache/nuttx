/****************************************************************************
 * drivers/audio/vs1053.h
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 * Pre-Processor Definitions
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

/* STATUS register bit definitions ****************************************/

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

/* BASS register bit definitions ******************************************/

#define VS1053_ST_AMPLITUDE         0xF000
#define VS1053_ST_FREQLIMIT         0x0F00
#define VS1053_SB_AMPLITUDE         0x00F0
#define VS1053_SB_FREQLIMIT         0x000F

/* CLOCKF register bit definitions ****************************************/

#define VS1053_SC_MULT              0xE000
#define VS1053_SC_MULT_SHIFT        13
#define VS1053_SC_ADD               0x1800
#define VS1053_SC_ADD_SHIFT         11
#define VS1053_SC_FREQ              0x07FF

#define VS1053_SC_MULT_XTALIx10     0
#define VS1053_SC_MULT_XTALIx20     1
#define VS1053_SC_MULT_XTALIx25     2
#define VS1053_SC_MULT_XTALIx30     3
#define VS1053_SC_MULT_XTALIx35     4
#define VS1053_SC_MULT_XTALIx40     5
#define VS1053_SC_MULT_XTALIx45     6
#define VS1053_SC_MULT_XTALIx50     7

#define VS1053_SC_ADD_NONE          0
#define VS1053_SC_ADD_XTALIx10      1
#define VS1053_SC_ADD_XTALIx15      2
#define VS1053_SC_ADD_XTALIx20      3

/* WRAM Addresses **********************************************************/

#define VS1053_XRAM_BASE            0x1800      /* X data RAM */
#define VS1053_XRAM_SIZE            256

#define VS1053_YRAM_BASE            0x5800      /* Y data RAM */
#define VS1053_YRAM_SIZE            256

#define VS1053_IRAM_BASE            0x8040      /* Instruction RAM */
#define VS1053_IRAM_SIZE            0x460

#define VS1053_IO_BASE              0xC000
#define VS1053_IO_SIZE              0x4000

/* HDAT1 register values *************************************************/

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
