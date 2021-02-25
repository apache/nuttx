/****************************************************************************
 * libs/libnx/nxfonts/nxfonts_pixel-lcd-machine.h
 *
 *   Copyright (C) 2014 Pierre-Noel Bouteville. All rights reserved.
 *   Author: Pierre-Noel Bouteville <pnb990@gmail.com>
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

#ifndef __LIBNX_NXFONTS_NXFONTS_PIXEL_LCD_MACHINE_H
#define __LIBNX_NXFONTS_NXFONTS_PIXEL_LCD_MACHINE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Font ID */

#define NXFONT_ID         FONTID_PIXEL_LCD_MACHINE

/* Ranges of 7-bit and 8-bit fonts */

#define NXFONT_MIN7BIT    33
#define NXFONT_MAX7BIT    122

#define NXFONT_MIN8BIT    192
#define NXFONT_MAX8BIT    255

/* Maximum height and width of any glyph in the set */

#define NXFONT_MAXHEIGHT  10
#define NXFONT_MAXWIDTH   33

/* The width of a space */

#define NXFONT_SPACEWIDTH 2

/* 0021 (33) */
#define NXFONT_METRICS_33 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_33 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 0022 (34) */
#define NXFONT_METRICS_34 {1, 3, 4, 0, 0, 0}
#define NXFONT_BITMAP_34 {0xa0, 0xa0, 0xa0, 0xa0}

/* 0023 (35) */
#define NXFONT_METRICS_35 {3, 23, 5, 0, 4, 0}
#define NXFONT_BITMAP_35 {0x72, 0x27, 0x70, 0x8a, 0x28, 0x88, 0x8a, 0x28, 0x88, 0x8a, 0x28, 0x88, 0x89, 0xc8, 0x8a}

/* 0024 (36) */
#define NXFONT_METRICS_36 {5, 33, 9, 0, 0, 0}
#define NXFONT_BITMAP_36 {0x8, 0x8, 0x41, 0xc0, 0x0, 0x8, 0x8, 0x42, 0x20, 0x0, 0x8, 0x8, 0x42, 0x20, 0x0, 0x8, 0x8, 0x42, 0x20, 0x0, 0x71, 0xc0, 0x1, 0xc3, 0x80, 0x8a, 0x28, 0x42, 0x24, 0x0, 0x8a, 0x28, 0x42, 0x24, 0x0, 0x8a, 0x28, 0x42, 0x24, 0x0, 0x71, 0xc7, 0x3a, 0x24, 0x0}

/* 0025 (37) */
#define NXFONT_METRICS_37 {4, 30, 9, 0, 0, 0}
#define NXFONT_BITMAP_37 {0x70, 0x7, 0x0, 0x38, 0x88, 0x8, 0x80, 0x44, 0x88, 0x8, 0x80, 0x44, 0x88, 0x8, 0x80, 0x44, 0x71, 0xc7, 0x1c, 0x38, 0x2, 0x20, 0x22, 0x0, 0x2, 0x20, 0x22, 0x0, 0x2, 0x20, 0x22, 0x0, 0x1, 0xc0, 0x1c, 0x0}

/* 0026 (38) Empty */
#define NXFONT_METRICS_38 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_38 {0x00}

/* 0027 (39) */
#define NXFONT_METRICS_39 {1, 1, 4, 0, 0, 0}
#define NXFONT_BITMAP_39 {0x80, 0x80, 0x80, 0x80}

/* 0028 (40) Empty */
#define NXFONT_METRICS_40 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_40 {0x00}

/* 0029 (41) Empty */
#define NXFONT_METRICS_41 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_41 {0x00}

/* 002A (42) */
#define NXFONT_METRICS_42 {1, 5, 5, 0, 0, 0}
#define NXFONT_BITMAP_42 {0x70, 0x88, 0x88, 0x88, 0x70}

/* 002B (43) Empty */
#define NXFONT_METRICS_43 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_43 {0x00}

/* 002C (44) */
#define NXFONT_METRICS_44 {1, 1, 4, 0, 5, 0}
#define NXFONT_BITMAP_44 {0x80, 0x80, 0x80, 0x80}

/* 002D (45) Empty */
#define NXFONT_METRICS_45 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_45 {0x00}

/* 002E (46) */
#define NXFONT_METRICS_46 {1, 5, 5, 0, 0, 0}
#define NXFONT_BITMAP_46 {0x70, 0x88, 0x88, 0x88, 0x70}

/* 002F (47) Empty */
#define NXFONT_METRICS_47 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_47 {0x00}

/* 0030 (48) */
#define NXFONT_METRICS_48 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_48 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 0031 (49) */
#define NXFONT_METRICS_49 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_49 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 0032 (50) */
#define NXFONT_METRICS_50 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_50 {0xf0, 0x8, 0x8, 0x8, 0x70, 0x80, 0x80, 0x80, 0x78}

/* 0033 (51) */
#define NXFONT_METRICS_51 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_51 {0xf0, 0x8, 0x8, 0x8, 0xf0, 0x8, 0x8, 0x8, 0xf0}

/* 0034 (52) */
#define NXFONT_METRICS_52 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_52 {0x88, 0x88, 0x88, 0x88, 0x70, 0x8, 0x8, 0x8, 0x8}

/* 0035 (53) */
#define NXFONT_METRICS_53 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_53 {0x78, 0x80, 0x80, 0x80, 0x70, 0x8, 0x8, 0x8, 0xf0}

/* 0036 (54) */
#define NXFONT_METRICS_54 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_54 {0x78, 0x80, 0x80, 0x80, 0x70, 0x88, 0x88, 0x88, 0x70}

/* 0037 (55) */
#define NXFONT_METRICS_55 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_55 {0xf8, 0x8, 0x8, 0x8, 0x0, 0x8, 0x8, 0x8, 0x8}

/* 0038 (56) */
#define NXFONT_METRICS_56 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_56 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x70}

/* 0039 (57) */
#define NXFONT_METRICS_57 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_57 {0x70, 0x88, 0x88, 0x88, 0x70, 0x8, 0x8, 0x8, 0x8}

/* 003A (58) Empty */
#define NXFONT_METRICS_58 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_58 {0x00}

/* 003B (59) Empty */
#define NXFONT_METRICS_59 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_59 {0x00}

/* 003C (60) Empty */
#define NXFONT_METRICS_60 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_60 {0x00}

/* 003D (61) */
#define NXFONT_METRICS_61 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_61 {0xf8, 0x0, 0x0, 0x0, 0xf8}

/* 003E (62) Empty */
#define NXFONT_METRICS_62 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_62 {0x00}

/* 003F (63) */
#define NXFONT_METRICS_63 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_63 {0xf0, 0x8, 0x8, 0x8, 0x70, 0x80, 0x80, 0x80, 0x80}

/* 0040 (64) */
#define NXFONT_METRICS_64 {2, 9, 9, 0, 0, 0}
#define NXFONT_BITMAP_64 {0xff, 0x0, 0x0, 0x80, 0x0, 0x80, 0x0, 0x80, 0x70, 0x0, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x77, 0x0}

/* 0041 (65) */
#define NXFONT_METRICS_65 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_65 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 0042 (66) */
#define NXFONT_METRICS_66 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_66 {0x80, 0x80, 0x80, 0x80, 0x70, 0x88, 0x88, 0x88, 0x70}

/* 0043 (67) */
#define NXFONT_METRICS_67 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_67 {0x70, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x70}

/* 0044 (68) */
#define NXFONT_METRICS_68 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_68 {0x8, 0x8, 0x8, 0x8, 0x70, 0x88, 0x88, 0x88, 0x70}

/* 0045 (69) */
#define NXFONT_METRICS_69 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_69 {0x70, 0x80, 0x80, 0x80, 0x70, 0x80, 0x80, 0x80, 0x70}

/* 0046 (70) */
#define NXFONT_METRICS_70 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_70 {0x70, 0x80, 0x80, 0x80, 0x70, 0x80, 0x80, 0x80, 0x80}

/* 0047 (71) */
#define NXFONT_METRICS_71 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_71 {0x78, 0x80, 0x80, 0x80, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 0048 (72) */
#define NXFONT_METRICS_72 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_72 {0x88, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 0049 (73) */
#define NXFONT_METRICS_73 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_73 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 004A (74) */
#define NXFONT_METRICS_74 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_74 {0x8, 0x8, 0x8, 0x8, 0x0, 0x8, 0x8, 0x8, 0xf0}

/* 004B (75) */
#define NXFONT_METRICS_75 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_75 {0x80, 0x80, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 004C (76) */
#define NXFONT_METRICS_76 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_76 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x70}

/* 004D (77) */
#define NXFONT_METRICS_77 {2, 9, 9, 0, 0, 0}
#define NXFONT_BITMAP_77 {0x77, 0x0, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x0, 0x0, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80}

/* 004E (78) */
#define NXFONT_METRICS_78 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_78 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x88}

/* 004F (79) */
#define NXFONT_METRICS_79 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_79 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 0050 (80) */
#define NXFONT_METRICS_80 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_80 {0x70, 0x88, 0x88, 0x88, 0x70, 0x80, 0x80, 0x80, 0x80}

/* 0051 (81) */
#define NXFONT_METRICS_81 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_81 {0x70, 0x88, 0x88, 0x88, 0x70, 0x8, 0x8, 0x8, 0x8}

/* 0052 (82) */
#define NXFONT_METRICS_82 {1, 4, 5, 0, 4, 0}
#define NXFONT_BITMAP_82 {0x70, 0x80, 0x80, 0x80, 0x80}

/* 0053 (83) */
#define NXFONT_METRICS_83 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_83 {0x78, 0x80, 0x80, 0x80, 0x70, 0x8, 0x8, 0x8, 0xf0}

/* 0054 (84) */
#define NXFONT_METRICS_84 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_84 {0x80, 0x80, 0x80, 0x80, 0x70, 0x80, 0x80, 0x80, 0x70}

/* 0055 (85) */
#define NXFONT_METRICS_85 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_85 {0x88, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 0056 (86) */
#define NXFONT_METRICS_86 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_86 {0x88, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 0057 (87) */
#define NXFONT_METRICS_87 {2, 9, 9, 0, 0, 0}
#define NXFONT_BITMAP_87 {0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x0, 0x0, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x77, 0x0}

/* 0058 (88) */
#define NXFONT_METRICS_88 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_88 {0x88, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 0059 (89) */
#define NXFONT_METRICS_89 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_89 {0x88, 0x88, 0x88, 0x88, 0x70, 0x8, 0x8, 0x8, 0xf0}

/* 005A (90) */
#define NXFONT_METRICS_90 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_90 {0xf0, 0x8, 0x8, 0x8, 0x70, 0x80, 0x80, 0x80, 0x78}

/* 005B (91) Empty */
#define NXFONT_METRICS_91 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_91 {0x00}

/* 005C (92) Empty */
#define NXFONT_METRICS_92 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_92 {0x00}

/* 005D (93) Empty */
#define NXFONT_METRICS_93 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_93 {0x00}

/* 005E (94) Empty */
#define NXFONT_METRICS_94 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_94 {0x00}

/* 005F (95) */
#define NXFONT_METRICS_95 {1, 3, 1, 0, 8, 0}
#define NXFONT_BITMAP_95 {0xe0}

/* 0060 (96) Empty */
#define NXFONT_METRICS_96 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_96 {0x00}

/* 0061 (97) */
#define NXFONT_METRICS_97 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_97 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 0062 (98) */
#define NXFONT_METRICS_98 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_98 {0x80, 0x80, 0x80, 0x80, 0x70, 0x88, 0x88, 0x88, 0x70}

/* 0063 (99) */
#define NXFONT_METRICS_99 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_99 {0x78, 0x80, 0x80, 0x80, 0x78}

/* 0064 (100) */
#define NXFONT_METRICS_100 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_100 {0x8, 0x8, 0x8, 0x8, 0x70, 0x88, 0x88, 0x88, 0x70}

/* 0065 (101) */
#define NXFONT_METRICS_101 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_101 {0x70, 0x88, 0x88, 0x88, 0x70, 0x80, 0x80, 0x80, 0x78}

/* 0066 (102) */
#define NXFONT_METRICS_102 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_102 {0x70, 0x80, 0x80, 0x80, 0x70, 0x80, 0x80, 0x80, 0x80}

/* 0067 (103) */
#define NXFONT_METRICS_103 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_103 {0x70, 0x88, 0x88, 0x88, 0x70, 0x8, 0x8, 0x8, 0xf0}

/* 0068 (104) */
#define NXFONT_METRICS_104 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_104 {0x80, 0x80, 0x80, 0x80, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 0069 (105) */
#define NXFONT_METRICS_105 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_105 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 006A (106) */
#define NXFONT_METRICS_106 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_106 {0x10, 0x10, 0x10, 0x10, 0x0, 0x10, 0x10, 0x10, 0xe0}

/* 006B (107) */
#define NXFONT_METRICS_107 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_107 {0x80, 0x80, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 006C (108) */
#define NXFONT_METRICS_108 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_108 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x70}

/* 006D (109) */
#define NXFONT_METRICS_109 {2, 9, 5, 0, 4, 0}
#define NXFONT_BITMAP_109 {0x77, 0x0, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80}

/* 006E (110) */
#define NXFONT_METRICS_110 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_110 {0x70, 0x88, 0x88, 0x88, 0x88}

/* 006F (111) */
#define NXFONT_METRICS_111 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_111 {0x70, 0x88, 0x88, 0x88, 0x70}

/* 0070 (112) */
#define NXFONT_METRICS_112 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_112 {0x70, 0x88, 0x88, 0x88, 0x70, 0x80, 0x80, 0x80, 0x80}

/* 0071 (113) */
#define NXFONT_METRICS_113 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_113 {0x70, 0x88, 0x88, 0x88, 0x70, 0x8, 0x8, 0x8, 0x8}

/* 0072 (114) */
#define NXFONT_METRICS_114 {1, 4, 5, 0, 4, 0}
#define NXFONT_BITMAP_114 {0x70, 0x80, 0x80, 0x80, 0x80}

/* 0073 (115) */
#define NXFONT_METRICS_115 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_115 {0x78, 0x80, 0x80, 0x80, 0x70, 0x8, 0x8, 0x8, 0xf0}

/* 0074 (116) */
#define NXFONT_METRICS_116 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_116 {0x80, 0x80, 0x80, 0x80, 0x70, 0x80, 0x80, 0x80, 0x70}

/* 0075 (117) */
#define NXFONT_METRICS_117 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_117 {0x88, 0x88, 0x88, 0x88, 0x70}

/* 0076 (118) */
#define NXFONT_METRICS_118 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_118 {0x88, 0x88, 0x88, 0x88, 0x70}

/* 0077 (119) */
#define NXFONT_METRICS_119 {2, 9, 5, 0, 4, 0}
#define NXFONT_BITMAP_119 {0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x77, 0x0}

/* 0078 (120) */
#define NXFONT_METRICS_120 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_120 {0x88, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 0079 (121) */
#define NXFONT_METRICS_121 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_121 {0x88, 0x88, 0x88, 0x88, 0x70, 0x8, 0x8, 0x8, 0xf0}

/* 007A (122) */
#define NXFONT_METRICS_122 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_122 {0xf0, 0x8, 0x8, 0x8, 0x70, 0x80, 0x80, 0x80, 0x78}

/* 00C0 (192) */
#define NXFONT_METRICS_192 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_192 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00C1 (193) */
#define NXFONT_METRICS_193 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_193 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00C2 (194) */
#define NXFONT_METRICS_194 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_194 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00C3 (195) */
#define NXFONT_METRICS_195 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_195 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00C4 (196) */
#define NXFONT_METRICS_196 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_196 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00C5 (197) */
#define NXFONT_METRICS_197 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_197 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00C6 (198) Empty */
#define NXFONT_METRICS_198 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_198 {0x00}

/* 00C7 (199) */
#define NXFONT_METRICS_199 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_199 {0x70, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x70}

/* 00C8 (200) */
#define NXFONT_METRICS_200 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_200 {0x70, 0x80, 0x80, 0x80, 0x70, 0x80, 0x80, 0x80, 0x70}

/* 00C9 (201) */
#define NXFONT_METRICS_201 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_201 {0x70, 0x80, 0x80, 0x80, 0x70, 0x80, 0x80, 0x80, 0x70}

/* 00CA (202) */
#define NXFONT_METRICS_202 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_202 {0x70, 0x80, 0x80, 0x80, 0x70, 0x80, 0x80, 0x80, 0x70}

/* 00CB (203) */
#define NXFONT_METRICS_203 {1, 4, 9, 0, 0, 0}
#define NXFONT_BITMAP_203 {0x70, 0x80, 0x80, 0x80, 0x70, 0x80, 0x80, 0x80, 0x70}

/* 00CC (204) */
#define NXFONT_METRICS_204 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_204 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 00CD (205) */
#define NXFONT_METRICS_205 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_205 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 00CE (206) */
#define NXFONT_METRICS_206 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_206 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 00CF (207) */
#define NXFONT_METRICS_207 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_207 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 00D0 (208) Empty */
#define NXFONT_METRICS_208 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_208 {0x00}

/* 00D1 (209) */
#define NXFONT_METRICS_209 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_209 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x88}

/* 00D2 (210) */
#define NXFONT_METRICS_210 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_210 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00D3 (211) */
#define NXFONT_METRICS_211 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_211 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00D4 (212) */
#define NXFONT_METRICS_212 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_212 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00D5 (213) */
#define NXFONT_METRICS_213 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_213 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00D6 (214) */
#define NXFONT_METRICS_214 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_214 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00D7 (215) Empty */
#define NXFONT_METRICS_215 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_215 {0x00}

/* 00D8 (216) */
#define NXFONT_METRICS_216 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_216 {0x70, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00D9 (217) */
#define NXFONT_METRICS_217 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_217 {0x88, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00DA (218) */
#define NXFONT_METRICS_218 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_218 {0x88, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00DB (219) */
#define NXFONT_METRICS_219 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_219 {0x88, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00DC (220) */
#define NXFONT_METRICS_220 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_220 {0x88, 0x88, 0x88, 0x88, 0x0, 0x88, 0x88, 0x88, 0x70}

/* 00DD (221) Empty */
#define NXFONT_METRICS_221 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_221 {0x00}

/* 00DE (222) Empty */
#define NXFONT_METRICS_222 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_222 {0x00}

/* 00DF (223) Empty */
#define NXFONT_METRICS_223 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_223 {0x00}

/* 00E0 (224) */
#define NXFONT_METRICS_224 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_224 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00E1 (225) */
#define NXFONT_METRICS_225 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_225 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00E2 (226) */
#define NXFONT_METRICS_226 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_226 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00E3 (227) */
#define NXFONT_METRICS_227 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_227 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00E4 (228) */
#define NXFONT_METRICS_228 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_228 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00E5 (229) */
#define NXFONT_METRICS_229 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_229 {0x70, 0x88, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x88}

/* 00E6 (230) Empty */
#define NXFONT_METRICS_230 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_230 {0x00}

/* 00E7 (231) */
#define NXFONT_METRICS_231 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_231 {0x78, 0x80, 0x80, 0x80, 0x78}

/* 00E8 (232) */
#define NXFONT_METRICS_232 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_232 {0x70, 0x88, 0x88, 0x88, 0x70, 0x80, 0x80, 0x80, 0x78}

/* 00E9 (233) */
#define NXFONT_METRICS_233 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_233 {0x70, 0x88, 0x88, 0x88, 0x70, 0x80, 0x80, 0x80, 0x78}

/* 00EA (234) */
#define NXFONT_METRICS_234 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_234 {0x70, 0x88, 0x88, 0x88, 0x70, 0x80, 0x80, 0x80, 0x78}

/* 00EB (235) */
#define NXFONT_METRICS_235 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_235 {0x70, 0x88, 0x88, 0x88, 0x70, 0x80, 0x80, 0x80, 0x78}

/* 00EC (236) */
#define NXFONT_METRICS_236 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_236 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 00ED (237) */
#define NXFONT_METRICS_237 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_237 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 00EE (238) */
#define NXFONT_METRICS_238 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_238 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 00EF (239) */
#define NXFONT_METRICS_239 {1, 1, 9, 0, 0, 0}
#define NXFONT_BITMAP_239 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 00F0 (240) Empty */
#define NXFONT_METRICS_240 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_240 {0x00}

/* 00F1 (241) */
#define NXFONT_METRICS_241 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_241 {0x70, 0x88, 0x88, 0x88, 0x88}

/* 00F2 (242) */
#define NXFONT_METRICS_242 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_242 {0x70, 0x88, 0x88, 0x88, 0x70}

/* 00F3 (243) */
#define NXFONT_METRICS_243 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_243 {0x70, 0x88, 0x88, 0x88, 0x70}

/* 00F4 (244) */
#define NXFONT_METRICS_244 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_244 {0x70, 0x88, 0x88, 0x88, 0x70}

/* 00F5 (245) */
#define NXFONT_METRICS_245 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_245 {0x70, 0x88, 0x88, 0x88, 0x70}

/* 00F6 (246) */
#define NXFONT_METRICS_246 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_246 {0x70, 0x88, 0x88, 0x88, 0x70}

/* 00F7 (247) Empty */
#define NXFONT_METRICS_247 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_247 {0x00}

/* 00F8 (248) */
#define NXFONT_METRICS_248 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_248 {0x70, 0x88, 0x88, 0x88, 0x70}

/* 00F9 (249) */
#define NXFONT_METRICS_249 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_249 {0x88, 0x88, 0x88, 0x88, 0x70}

/* 00FA (250) */
#define NXFONT_METRICS_250 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_250 {0x88, 0x88, 0x88, 0x88, 0x70}

/* 00FB (251) */
#define NXFONT_METRICS_251 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_251 {0x88, 0x88, 0x88, 0x88, 0x70}

/* 00FC (252) */
#define NXFONT_METRICS_252 {1, 5, 5, 0, 4, 0}
#define NXFONT_BITMAP_252 {0x88, 0x88, 0x88, 0x88, 0x70}

/* 00FD (253) Empty */
#define NXFONT_METRICS_253 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_253 {0x00}

/* 00FE (254) Empty */
#define NXFONT_METRICS_254 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_254 {0x00}

/* 00FF (255) */
#define NXFONT_METRICS_255 {1, 5, 9, 0, 0, 0}
#define NXFONT_BITMAP_255 {0x88, 0x88, 0x88, 0x88, 0x70, 0x8, 0x8, 0x8, 0xf0}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __LIBNX_NXFONTS_NXFONTS_PIXEL_LCD_MACHINE_H */
