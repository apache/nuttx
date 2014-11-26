/****************************************************************************
 * libnx/nxfonts/nxfonts_pixel-unicode.h
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

#ifndef __LIBNX_NXFONTS_NXFONTS_PIXEL_UNICODE_H
#define __LIBNX_NXFONTS_NXFONTS_PIXEL_UNICODE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Font ID */

#define NXFONT_ID         FONTID_PIXEL_UNICODE

/* Ranges of 7-bit and 8-bit fonts */

#define NXFONT_MIN7BIT    33
#define NXFONT_MAX7BIT    126

#define NXFONT_MIN8BIT    161
#define NXFONT_MAX8BIT    255

/* Maximum height and width of any glyph in the set */

#define NXFONT_MAXHEIGHT  18
#define NXFONT_MAXWIDTH   29

/* The width of a space */

#define NXFONT_SPACEWIDTH 2

/* 0021 (33) */
#define NXFONT_METRICS_33 {1, 1, 8, 0, 5, 0}
#define NXFONT_BITMAP_33 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0, 0x80}

/* 0022 (34) */
#define NXFONT_METRICS_34 {1, 3, 2, 0, 5, 0}
#define NXFONT_BITMAP_34 {0xa0, 0xa0}

/* 0023 (35) */
#define NXFONT_METRICS_35 {1, 6, 8, 0, 5, 0}
#define NXFONT_BITMAP_35 {0x48, 0x48, 0xfc, 0x48, 0x48, 0xfc, 0x48, 0x48}

/* 0024 (36) */
#define NXFONT_METRICS_36 {1, 5, 10, 0, 4, 0}
#define NXFONT_BITMAP_36 {0x20, 0x70, 0xa8, 0xa0, 0xa0, 0x70, 0x28, 0xa8, 0x70, 0x20}

/* 0025 (37) */
#define NXFONT_METRICS_37 {1, 8, 8, 0, 5, 0}
#define NXFONT_BITMAP_37 {0x61, 0x92, 0x94, 0x68, 0x16, 0x29, 0x49, 0x86}

/* 0026 (38) */
#define NXFONT_METRICS_38 {1, 6, 8, 0, 5, 0}
#define NXFONT_BITMAP_38 {0x60, 0x90, 0x90, 0x60, 0x64, 0x94, 0x88, 0x74}

/* 0027 (39) */
#define NXFONT_METRICS_39 {1, 1, 2, 0, 5, 0}
#define NXFONT_BITMAP_39 {0x80, 0x80}

/* 0028 (40) */
#define NXFONT_METRICS_40 {1, 2, 9, 0, 5, 0}
#define NXFONT_BITMAP_40 {0x40, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x40}

/* 0029 (41) */
#define NXFONT_METRICS_41 {1, 2, 9, 0, 5, 0}
#define NXFONT_BITMAP_41 {0x80, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x80}

/* 002A (42) */
#define NXFONT_METRICS_42 {1, 3, 3, 0, 5, 0}
#define NXFONT_BITMAP_42 {0xa0, 0x40, 0xa0}

/* 002B (43) */
#define NXFONT_METRICS_43 {1, 5, 5, 0, 6, 0}
#define NXFONT_BITMAP_43 {0x20, 0x20, 0xf8, 0x20, 0x20}

/* 002C (44) */
#define NXFONT_METRICS_44 {1, 2, 3, 0, 11, 0}
#define NXFONT_BITMAP_44 {0x40, 0x40, 0x80}

/* 002D (45) */
#define NXFONT_METRICS_45 {1, 4, 1, 0, 8, 0}
#define NXFONT_BITMAP_45 {0xf0}

/* 002E (46) */
#define NXFONT_METRICS_46 {1, 1, 1, 0, 12, 0}
#define NXFONT_BITMAP_46 {0x80}

/* 002F (47) */
#define NXFONT_METRICS_47 {1, 6, 8, 0, 5, 0}
#define NXFONT_BITMAP_47 {0x4, 0x8, 0x10, 0x10, 0x20, 0x40, 0x40, 0x80}

/* 0030 (48) */
#define NXFONT_METRICS_48 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_48 {0x70, 0x88, 0x88, 0x98, 0xa8, 0xc8, 0x88, 0x70}

/* 0031 (49) */
#define NXFONT_METRICS_49 {1, 2, 8, 0, 5, 0}
#define NXFONT_BITMAP_49 {0xc0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}

/* 0032 (50) */
#define NXFONT_METRICS_50 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_50 {0x70, 0x88, 0x8, 0x10, 0x20, 0x40, 0x80, 0xf8}

/* 0033 (51) */
#define NXFONT_METRICS_51 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_51 {0x70, 0x88, 0x8, 0x30, 0x8, 0x8, 0x88, 0x70}

/* 0034 (52) */
#define NXFONT_METRICS_52 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_52 {0x30, 0x50, 0x90, 0x90, 0xf8, 0x10, 0x10, 0x10}

/* 0035 (53) */
#define NXFONT_METRICS_53 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_53 {0xf8, 0x80, 0x80, 0xb0, 0xc8, 0x8, 0x88, 0x70}

/* 0036 (54) */
#define NXFONT_METRICS_54 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_54 {0x70, 0x88, 0x80, 0x80, 0xf0, 0x88, 0x88, 0x70}

/* 0037 (55) */
#define NXFONT_METRICS_55 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_55 {0xf8, 0x8, 0x10, 0x20, 0x20, 0x40, 0x40, 0x80}

/* 0038 (56) */
#define NXFONT_METRICS_56 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_56 {0x70, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x70}

/* 0039 (57) */
#define NXFONT_METRICS_57 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_57 {0x70, 0x88, 0x88, 0x78, 0x8, 0x8, 0x88, 0x70}

/* 003A (58) */
#define NXFONT_METRICS_58 {1, 1, 5, 0, 8, 0}
#define NXFONT_BITMAP_58 {0x80, 0x0, 0x0, 0x0, 0x80}

/* 003B (59) */
#define NXFONT_METRICS_59 {1, 2, 6, 0, 8, 0}
#define NXFONT_BITMAP_59 {0x40, 0x0, 0x0, 0x0, 0x40, 0x80}

/* 003C (60) */
#define NXFONT_METRICS_60 {1, 4, 7, 0, 6, 0}
#define NXFONT_BITMAP_60 {0x10, 0x20, 0x40, 0x80, 0x40, 0x20, 0x10}

/* 003D (61) */
#define NXFONT_METRICS_61 {1, 5, 4, 0, 7, 0}
#define NXFONT_BITMAP_61 {0xf8, 0x0, 0x0, 0xf8}

/* 003E (62) */
#define NXFONT_METRICS_62 {1, 4, 7, 0, 6, 0}
#define NXFONT_BITMAP_62 {0x80, 0x40, 0x20, 0x10, 0x20, 0x40, 0x80}

/* 003F (63) */
#define NXFONT_METRICS_63 {1, 6, 8, 0, 5, 0}
#define NXFONT_BITMAP_63 {0x78, 0x84, 0x84, 0x4, 0x18, 0x20, 0x0, 0x20}

/* 0040 (64) */
#define NXFONT_METRICS_64 {1, 7, 7, 0, 5, 0}
#define NXFONT_BITMAP_64 {0x7c, 0x82, 0x9a, 0xaa, 0x9c, 0x80, 0x7e}

/* 0041 (65) */
#define NXFONT_METRICS_65 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_65 {0x20, 0x50, 0x88, 0x88, 0xf8, 0x88, 0x88, 0x88}

/* 0042 (66) */
#define NXFONT_METRICS_66 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_66 {0xf0, 0x88, 0x88, 0xf0, 0x88, 0x88, 0x88, 0xf0}

/* 0043 (67) */
#define NXFONT_METRICS_67 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_67 {0x70, 0x88, 0x80, 0x80, 0x80, 0x80, 0x88, 0x70}

/* 0044 (68) */
#define NXFONT_METRICS_68 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_68 {0xf0, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0xf0}

/* 0045 (69) */
#define NXFONT_METRICS_69 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_69 {0xf8, 0x80, 0x80, 0xe0, 0x80, 0x80, 0x80, 0xf8}

/* 0046 (70) */
#define NXFONT_METRICS_70 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_70 {0xf8, 0x80, 0x80, 0xe0, 0x80, 0x80, 0x80, 0x80}

/* 0047 (71) */
#define NXFONT_METRICS_71 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_71 {0x70, 0x88, 0x80, 0x80, 0xb8, 0x88, 0x88, 0x70}

/* 0048 (72) */
#define NXFONT_METRICS_72 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_72 {0x88, 0x88, 0x88, 0xf8, 0x88, 0x88, 0x88, 0x88}

/* 0049 (73) */
#define NXFONT_METRICS_73 {1, 3, 8, 0, 5, 0}
#define NXFONT_BITMAP_73 {0xe0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xe0}

/* 004A (74) */
#define NXFONT_METRICS_74 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_74 {0x38, 0x10, 0x10, 0x10, 0x10, 0x10, 0x90, 0x60}

/* 004B (75) */
#define NXFONT_METRICS_75 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_75 {0x88, 0x90, 0xa0, 0xc0, 0xa0, 0x90, 0x90, 0x88}

/* 004C (76) */
#define NXFONT_METRICS_76 {1, 4, 8, 0, 5, 0}
#define NXFONT_BITMAP_76 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xf0}

/* 004D (77) */
#define NXFONT_METRICS_77 {1, 7, 8, 0, 5, 0}
#define NXFONT_BITMAP_77 {0xc6, 0xaa, 0x92, 0x92, 0x92, 0x82, 0x82, 0x82}

/* 004E (78) */
#define NXFONT_METRICS_78 {1, 7, 8, 0, 5, 0}
#define NXFONT_BITMAP_78 {0xc2, 0xa2, 0xa2, 0x92, 0x8a, 0x8a, 0x86, 0x82}

/* 004F (79) */
#define NXFONT_METRICS_79 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_79 {0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 0050 (80) */
#define NXFONT_METRICS_80 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_80 {0xf0, 0x88, 0x88, 0x88, 0xf0, 0x80, 0x80, 0x80}

/* 0051 (81) */
#define NXFONT_METRICS_81 {1, 6, 8, 0, 5, 0}
#define NXFONT_BITMAP_81 {0x78, 0x84, 0x84, 0x84, 0x84, 0x94, 0x88, 0x74}

/* 0052 (82) */
#define NXFONT_METRICS_82 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_82 {0xf0, 0x88, 0x88, 0x88, 0xf0, 0xa0, 0x90, 0x88}

/* 0053 (83) */
#define NXFONT_METRICS_83 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_83 {0x70, 0x88, 0x80, 0x70, 0x8, 0x8, 0x88, 0x70}

/* 0054 (84) */
#define NXFONT_METRICS_84 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_84 {0xf8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20}

/* 0055 (85) */
#define NXFONT_METRICS_85 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_85 {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 0056 (86) */
#define NXFONT_METRICS_86 {1, 7, 8, 0, 5, 0}
#define NXFONT_BITMAP_86 {0x82, 0x82, 0x44, 0x44, 0x28, 0x28, 0x10, 0x10}

/* 0057 (87) */
#define NXFONT_METRICS_87 {2, 9, 8, 0, 5, 0}
#define NXFONT_BITMAP_87 {0x80, 0x80, 0x88, 0x80, 0x88, 0x80, 0x55, 0x0, 0x55, 0x0, 0x55, 0x0, 0x22, 0x0, 0x22, 0x0}

/* 0058 (88) */
#define NXFONT_METRICS_88 {1, 7, 8, 0, 5, 0}
#define NXFONT_BITMAP_88 {0x82, 0x44, 0x28, 0x10, 0x10, 0x28, 0x44, 0x82}

/* 0059 (89) */
#define NXFONT_METRICS_89 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_89 {0x88, 0x88, 0x50, 0x20, 0x20, 0x20, 0x20, 0x20}

/* 005A (90) */
#define NXFONT_METRICS_90 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_90 {0xf8, 0x8, 0x10, 0x20, 0x20, 0x40, 0x80, 0xf8}

/* 005B (91) */
#define NXFONT_METRICS_91 {1, 2, 9, 0, 5, 0}
#define NXFONT_BITMAP_91 {0xc0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xc0}

/* 005C (92) */
#define NXFONT_METRICS_92 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_92 {0x80, 0x80, 0x40, 0x20, 0x20, 0x10, 0x8, 0x8}

/* 005D (93) */
#define NXFONT_METRICS_93 {1, 2, 9, 0, 5, 0}
#define NXFONT_BITMAP_93 {0xc0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xc0}

/* 005E (94) */
#define NXFONT_METRICS_94 {1, 5, 4, 0, 5, 0}
#define NXFONT_BITMAP_94 {0x20, 0x50, 0x50, 0x88}

/* 005F (95) */
#define NXFONT_METRICS_95 {1, 5, 1, 0, 13, 0}
#define NXFONT_BITMAP_95 {0xf8}

/* 0060 (96) */
#define NXFONT_METRICS_96 {1, 3, 3, 0, 5, 0}
#define NXFONT_BITMAP_96 {0x80, 0x40, 0x20}

/* 0061 (97) */
#define NXFONT_METRICS_97 {1, 6, 6, 0, 7, 0}
#define NXFONT_BITMAP_97 {0x70, 0x88, 0x8, 0x78, 0x88, 0x74}

/* 0062 (98) */
#define NXFONT_METRICS_98 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_98 {0x80, 0x80, 0xb0, 0xc8, 0x88, 0x88, 0xc8, 0xb0}

/* 0063 (99) */
#define NXFONT_METRICS_99 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_99 {0x70, 0x88, 0x80, 0x80, 0x88, 0x70}

/* 0064 (100) */
#define NXFONT_METRICS_100 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_100 {0x8, 0x8, 0x68, 0x98, 0x88, 0x88, 0x98, 0x68}

/* 0065 (101) */
#define NXFONT_METRICS_101 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_101 {0x70, 0x88, 0xf0, 0x80, 0x88, 0x70}

/* 0066 (102) */
#define NXFONT_METRICS_102 {1, 4, 8, 0, 5, 0}
#define NXFONT_BITMAP_102 {0x30, 0x40, 0xe0, 0x40, 0x40, 0x40, 0x40, 0x40}

/* 0067 (103) */
#define NXFONT_METRICS_103 {1, 5, 9, 0, 7, 0}
#define NXFONT_BITMAP_103 {0x68, 0x98, 0x88, 0x88, 0x98, 0x68, 0x8, 0x88, 0x70}

/* 0068 (104) */
#define NXFONT_METRICS_104 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_104 {0x80, 0x80, 0xb0, 0xc8, 0x88, 0x88, 0x88, 0x88}

/* 0069 (105) */
#define NXFONT_METRICS_105 {1, 1, 8, 0, 5, 0}
#define NXFONT_BITMAP_105 {0x80, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* 006A (106) */
#define NXFONT_METRICS_106 {1, 4, 9, 0, 5, 0}
#define NXFONT_BITMAP_106 {0x10, 0x0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x90, 0x60}

/* 006B (107) */
#define NXFONT_METRICS_107 {1, 4, 8, 0, 5, 0}
#define NXFONT_BITMAP_107 {0x80, 0x80, 0x90, 0xa0, 0xc0, 0xa0, 0x90, 0x90}

/* 006C (108) */
#define NXFONT_METRICS_108 {1, 1, 8, 0, 5, 0}
#define NXFONT_BITMAP_108 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* 006D (109) */
#define NXFONT_METRICS_109 {1, 8, 6, 0, 7, 0}
#define NXFONT_BITMAP_109 {0xb6, 0xc9, 0x89, 0x89, 0x89, 0x89}

/* 006E (110) */
#define NXFONT_METRICS_110 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_110 {0xb0, 0xc8, 0x88, 0x88, 0x88, 0x88}

/* 006F (111) */
#define NXFONT_METRICS_111 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_111 {0x70, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 0070 (112) */
#define NXFONT_METRICS_112 {1, 5, 8, 0, 7, 0}
#define NXFONT_BITMAP_112 {0xb0, 0xc8, 0x88, 0x88, 0xc8, 0xb0, 0x80, 0x80}

/* 0071 (113) */
#define NXFONT_METRICS_113 {1, 5, 8, 0, 7, 0}
#define NXFONT_BITMAP_113 {0x68, 0x98, 0x88, 0x88, 0x98, 0x68, 0x8, 0x8}

/* 0072 (114) */
#define NXFONT_METRICS_114 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_114 {0xb0, 0xc8, 0x80, 0x80, 0x80, 0x80}

/* 0073 (115) */
#define NXFONT_METRICS_115 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_115 {0x78, 0x80, 0x70, 0x8, 0x88, 0x70}

/* 0074 (116) */
#define NXFONT_METRICS_116 {1, 4, 8, 0, 5, 0}
#define NXFONT_BITMAP_116 {0x40, 0x40, 0xe0, 0x40, 0x40, 0x40, 0x50, 0x20}

/* 0075 (117) */
#define NXFONT_METRICS_117 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_117 {0x88, 0x88, 0x88, 0x88, 0x98, 0x68}

/* 0076 (118) */
#define NXFONT_METRICS_118 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_118 {0x88, 0x88, 0x50, 0x50, 0x20, 0x20}

/* 0077 (119) */
#define NXFONT_METRICS_119 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_119 {0x88, 0xa8, 0xa8, 0xa8, 0x50, 0x50}

/* 0078 (120) */
#define NXFONT_METRICS_120 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_120 {0x88, 0x50, 0x20, 0x20, 0x50, 0x88}

/* 0079 (121) */
#define NXFONT_METRICS_121 {1, 5, 8, 0, 7, 0}
#define NXFONT_BITMAP_121 {0x88, 0x88, 0x50, 0x50, 0x20, 0x20, 0x40, 0x80}

/* 007A (122) */
#define NXFONT_METRICS_122 {1, 5, 6, 0, 7, 0}
#define NXFONT_BITMAP_122 {0xf8, 0x8, 0x10, 0x20, 0x40, 0xf8}

/* 007B (123) -- NOTE: Xoffset should be -1, not 0. */
#define NXFONT_METRICS_123 {1, 4, 9, 0, 5, 0}
#define NXFONT_BITMAP_123 {0x30, 0x40, 0x40, 0x40, 0x80, 0x40, 0x40, 0x40, 0x30}

/* 007C (124) */
#define NXFONT_METRICS_124 {1, 1, 9, 0, 5, 0}
#define NXFONT_BITMAP_124 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* 007D (125) */
#define NXFONT_METRICS_125 {1, 4, 9, 0, 5, 0}
#define NXFONT_BITMAP_125 {0xc0, 0x20, 0x20, 0x20, 0x10, 0x20, 0x20, 0x20, 0xc0}

/* 007E (126) */
#define NXFONT_METRICS_126 {1, 6, 2, 0, 8, 0}
#define NXFONT_BITMAP_126 {0x64, 0x98}

/* 00A1 (161) */
#define NXFONT_METRICS_161 {1, 1, 8, 0, 6, 0}
#define NXFONT_BITMAP_161 {0x80, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* 00A2 (162) */
#define NXFONT_METRICS_162 {1, 5, 8, 0, 6, 0}
#define NXFONT_BITMAP_162 {0x20, 0x70, 0xa8, 0xa0, 0xa0, 0xa8, 0x70, 0x20}

/* 00A3 (163) */
#define NXFONT_METRICS_163 {1, 6, 8, 0, 5, 0}
#define NXFONT_BITMAP_163 {0x38, 0x44, 0x40, 0xf0, 0x40, 0x40, 0x44, 0xb8}

/* 00A4 (164) */
#define NXFONT_METRICS_164 {1, 6, 6, 0, 6, 0}
#define NXFONT_BITMAP_164 {0x84, 0x78, 0x48, 0x48, 0x78, 0x84}

/* 00A5 (165) */
#define NXFONT_METRICS_165 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_165 {0x88, 0x88, 0x50, 0x20, 0xf8, 0x20, 0xf8, 0x20}

/* 00A6 (166) */
#define NXFONT_METRICS_166 {1, 1, 9, 0, 5, 0}
#define NXFONT_BITMAP_166 {0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80}

/* 00A7 (167) */
#define NXFONT_METRICS_167 {1, 5, 9, 0, 5, 0}
#define NXFONT_BITMAP_167 {0x70, 0x88, 0x80, 0x70, 0x88, 0x70, 0x8, 0x88, 0x70}

/* 00A8 (168) */
#define NXFONT_METRICS_168 {1, 3, 1, 0, 5, 0}
#define NXFONT_BITMAP_168 {0xa0}

/* 00A9 (169) */
#define NXFONT_METRICS_169 {1, 7, 7, 0, 6, 0}
#define NXFONT_BITMAP_169 {0x7c, 0x82, 0xba, 0xa2, 0xba, 0x82, 0x7c}

/* 00AA (170) */
#define NXFONT_METRICS_170 {1, 4, 5, 0, 5, 0}
#define NXFONT_BITMAP_170 {0xe0, 0x10, 0x70, 0x90, 0x70}

/* 00AB (171) */
#define NXFONT_METRICS_171 {1, 6, 5, 0, 7, 0}
#define NXFONT_BITMAP_171 {0x24, 0x48, 0x90, 0x48, 0x24}

/* 00AC (172) */
#define NXFONT_METRICS_172 {1, 5, 3, 0, 7, 0}
#define NXFONT_BITMAP_172 {0xf8, 0x8, 0x8}

/* 00AD (173) */
#define NXFONT_METRICS_173 {1, 4, 8, 0, 5, 0}
#define NXFONT_BITMAP_173 {0x70, 0xf0, 0xc0, 0xc0, 0xf0, 0xf0, 0xc0, 0xc0}

/* 00AE (174) */
#define NXFONT_METRICS_174 {1, 7, 7, 0, 6, 0}
#define NXFONT_BITMAP_174 {0x7c, 0x82, 0xba, 0xb2, 0xaa, 0x82, 0x7c}

/* 00AF (175) */
#define NXFONT_METRICS_175 {1, 5, 1, 0, 5, 0}
#define NXFONT_BITMAP_175 {0xf8}

/* 00B0 (176) */
#define NXFONT_METRICS_176 {1, 3, 3, 0, 5, 0}
#define NXFONT_BITMAP_176 {0x40, 0xa0, 0x40}

/* 00B1 (177) */
#define NXFONT_METRICS_177 {1, 5, 7, 0, 6, 0}
#define NXFONT_BITMAP_177 {0x20, 0x20, 0xf8, 0x20, 0x20, 0x0, 0xf8}

/* 00B2 (178) */
#define NXFONT_METRICS_178 {1, 3, 4, 0, 5, 0}
#define NXFONT_BITMAP_178 {0xe0, 0x20, 0x40, 0xe0}

/* 00B3 (179) */
#define NXFONT_METRICS_179 {1, 3, 5, 0, 5, 0}
#define NXFONT_BITMAP_179 {0xc0, 0x20, 0x40, 0x20, 0xc0}

/* 00B4 (180) */
#define NXFONT_METRICS_180 {1, 3, 3, 0, 5, 0}
#define NXFONT_BITMAP_180 {0x20, 0x40, 0x80}

/* 00B5 (181) */
#define NXFONT_METRICS_181 {1, 5, 8, 0, 7, 0}
#define NXFONT_BITMAP_181 {0x88, 0x88, 0x88, 0x88, 0xc8, 0xb8, 0x80, 0x80}

/* 00B6 (182) */
#define NXFONT_METRICS_182 {1, 8, 8, 0, 5, 0}
#define NXFONT_BITMAP_182 {0x7f, 0xff, 0xfb, 0x7b, 0x1b, 0x1b, 0x1b, 0x1b}

/* 00B7 (183) */
#define NXFONT_METRICS_183 {1, 1, 1, 0, 9, 0}
#define NXFONT_BITMAP_183 {0x80}

/* 00B8 (184) */
#define NXFONT_METRICS_184 {1, 2, 3, 0, 11, 0}
#define NXFONT_BITMAP_184 {0x80, 0x40, 0xc0}

/* 00B9 (185) */
#define NXFONT_METRICS_185 {1, 2, 4, 0, 5, 0}
#define NXFONT_BITMAP_185 {0xc0, 0x40, 0x40, 0x40}

/* 00BA (186) */
#define NXFONT_METRICS_186 {1, 3, 5, 0, 5, 0}
#define NXFONT_BITMAP_186 {0x40, 0xa0, 0x40, 0x0, 0xe0}

/* 00BB (187) */
#define NXFONT_METRICS_187 {1, 6, 5, 0, 7, 0}
#define NXFONT_BITMAP_187 {0x90, 0x48, 0x24, 0x48, 0x90}

/* 00BC (188) */
#define NXFONT_METRICS_188 {1, 8, 8, 0, 5, 0}
#define NXFONT_BITMAP_188 {0xc1, 0x42, 0x44, 0x8, 0x13, 0x25, 0x47, 0x81}

/* 00BD (189) */
#define NXFONT_METRICS_189 {1, 8, 8, 0, 5, 0}
#define NXFONT_BITMAP_189 {0xc1, 0x42, 0x44, 0x8, 0x16, 0x21, 0x46, 0x87}

/* 00BE (190) */
#define NXFONT_METRICS_190 {1, 8, 8, 0, 5, 0}
#define NXFONT_BITMAP_190 {0xc1, 0x22, 0xc4, 0x28, 0xd3, 0x25, 0x47, 0x81}

/* 00BF (191) */
#define NXFONT_METRICS_191 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_191 {0x20, 0x0, 0x20, 0x40, 0x80, 0x80, 0x88, 0x70}

/* 00C0 (192) */
#define NXFONT_METRICS_192 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_192 {0x40, 0x20, 0x0, 0x20, 0x50, 0x88, 0xf8, 0x88, 0x88, 0x88}

/* 00C1 (193) */
#define NXFONT_METRICS_193 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_193 {0x10, 0x20, 0x0, 0x20, 0x50, 0x88, 0xf8, 0x88, 0x88, 0x88}

/* 00C2 (194) */
#define NXFONT_METRICS_194 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_194 {0x20, 0x50, 0x0, 0x20, 0x50, 0x88, 0xf8, 0x88, 0x88, 0x88}

/* 00C3 (195) */
#define NXFONT_METRICS_195 {1, 6, 10, 0, 3, 0}
#define NXFONT_BITMAP_195 {0x64, 0x98, 0x0, 0x20, 0x50, 0x88, 0xf8, 0x88, 0x88, 0x88}

/* 00C4 (196) */
#define NXFONT_METRICS_196 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_196 {0x50, 0x0, 0x20, 0x50, 0x88, 0x88, 0xf8, 0x88, 0x88, 0x88}

/* 00C5 (197) */
#define NXFONT_METRICS_197 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_197 {0x20, 0x50, 0x20, 0x50, 0x88, 0x88, 0xf8, 0x88, 0x88, 0x88}

/* 00C6 (198) */
#define NXFONT_METRICS_198 {1, 8, 8, 0, 5, 0}
#define NXFONT_BITMAP_198 {0x1f, 0x18, 0x28, 0x28, 0x7e, 0x48, 0x88, 0x8f}

/* 00C7 (199) */
#define NXFONT_METRICS_199 {1, 5, 9, 0, 5, 0}
#define NXFONT_BITMAP_199 {0x70, 0x88, 0x80, 0x80, 0x80, 0x88, 0x70, 0x20, 0x60}

/* 00C8 (200) */
#define NXFONT_METRICS_200 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_200 {0x40, 0x20, 0x0, 0xf8, 0x80, 0x80, 0xe0, 0x80, 0x80, 0xf8}

/* 00C9 (201) */
#define NXFONT_METRICS_201 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_201 {0x10, 0x20, 0x0, 0xf8, 0x80, 0x80, 0xe0, 0x80, 0x80, 0xf8}

/* 00CA (202) */
#define NXFONT_METRICS_202 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_202 {0x20, 0x50, 0x0, 0xf8, 0x80, 0x80, 0xe0, 0x80, 0x80, 0xf8}

/* 00CB (203) */
#define NXFONT_METRICS_203 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_203 {0x50, 0x0, 0xf8, 0x80, 0x80, 0xe0, 0x80, 0x80, 0x80, 0xf8}

/* 00CC (204) */
#define NXFONT_METRICS_204 {1, 3, 10, 0, 3, 0}
#define NXFONT_BITMAP_204 {0x80, 0x40, 0x0, 0xe0, 0x40, 0x40, 0x40, 0x40, 0x40, 0xe0}

/* 00CD (205) */
#define NXFONT_METRICS_205 {1, 3, 10, 0, 3, 0}
#define NXFONT_BITMAP_205 {0x20, 0x40, 0x0, 0xe0, 0x40, 0x40, 0x40, 0x40, 0x40, 0xe0}

/* 00CE (206) */
#define NXFONT_METRICS_206 {1, 3, 10, 0, 3, 0}
#define NXFONT_BITMAP_206 {0x40, 0xa0, 0x0, 0xe0, 0x40, 0x40, 0x40, 0x40, 0x40, 0xe0}

/* 00CF (207) */
#define NXFONT_METRICS_207 {1, 3, 10, 0, 3, 0}
#define NXFONT_BITMAP_207 {0xa0, 0x0, 0xe0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xe0}

/* 00D0 (208) */
#define NXFONT_METRICS_208 {1, 6, 8, 0, 5, 0}
#define NXFONT_BITMAP_208 {0x78, 0x44, 0x44, 0xf4, 0x44, 0x44, 0x44, 0x78}

/* 00D1 (209) */
#define NXFONT_METRICS_209 {1, 7, 10, 0, 3, 0}
#define NXFONT_BITMAP_209 {0x32, 0x4c, 0x0, 0xc2, 0xa2, 0x92, 0x8a, 0x8a, 0x86, 0x82}

/* 00D2 (210) */
#define NXFONT_METRICS_210 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_210 {0x40, 0x20, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00D3 (211) */
#define NXFONT_METRICS_211 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_211 {0x10, 0x20, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00D4 (212) */
#define NXFONT_METRICS_212 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_212 {0x20, 0x50, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00D5 (213) */
#define NXFONT_METRICS_213 {1, 6, 10, 0, 3, 0}
#define NXFONT_BITMAP_213 {0x64, 0x98, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00D6 (214) */
#define NXFONT_METRICS_214 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_214 {0x50, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00D7 (215) */
#define NXFONT_METRICS_215 {1, 5, 5, 0, 7, 0}
#define NXFONT_BITMAP_215 {0x88, 0x50, 0x20, 0x50, 0x88}

/* 00D8 (216) */
#define NXFONT_METRICS_216 {1, 7, 8, 0, 5, 0}
#define NXFONT_BITMAP_216 {0x3a, 0x44, 0x44, 0x4c, 0x54, 0x64, 0x44, 0xb8}

/* 00D9 (217) */
#define NXFONT_METRICS_217 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_217 {0x40, 0x20, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00DA (218) */
#define NXFONT_METRICS_218 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_218 {0x10, 0x20, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00DB (219) */
#define NXFONT_METRICS_219 {1, 5, 10, 0, 3, 0}
#define NXFONT_BITMAP_219 {0x20, 0x50, 0x0, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00DC (220) */
#define NXFONT_METRICS_220 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_220 {0x50, 0x0, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00DD (221) */
#define NXFONT_METRICS_221 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_221 {0x10, 0x20, 0x88, 0x88, 0x50, 0x20, 0x20, 0x20, 0x20}

/* 00DE (222) */
#define NXFONT_METRICS_222 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_222 {0x80, 0x80, 0xf0, 0x88, 0x88, 0xf0, 0x80, 0x80}

/* 00DF (223) */
#define NXFONT_METRICS_223 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_223 {0x30, 0x48, 0x88, 0x90, 0xa0, 0x90, 0x88, 0xb0}

/* 00E0 (224) */
#define NXFONT_METRICS_224 {1, 6, 9, 0, 4, 0}
#define NXFONT_BITMAP_224 {0x40, 0x20, 0x0, 0x70, 0x88, 0x8, 0x78, 0x88, 0x74}

/* 00E1 (225) */
#define NXFONT_METRICS_225 {1, 6, 9, 0, 4, 0}
#define NXFONT_BITMAP_225 {0x10, 0x20, 0x0, 0x70, 0x88, 0x8, 0x78, 0x88, 0x74}

/* 00E2 (226) */
#define NXFONT_METRICS_226 {1, 6, 9, 0, 4, 0}
#define NXFONT_BITMAP_226 {0x20, 0x50, 0x0, 0x70, 0x88, 0x8, 0x78, 0x88, 0x74}

/* 00E3 (227) */
#define NXFONT_METRICS_227 {1, 6, 9, 0, 4, 0}
#define NXFONT_BITMAP_227 {0x64, 0x98, 0x0, 0x70, 0x88, 0x8, 0x78, 0x88, 0x74}

/* 00E4 (228) */
#define NXFONT_METRICS_228 {1, 6, 8, 0, 5, 0}
#define NXFONT_BITMAP_228 {0x50, 0x0, 0x70, 0x88, 0x8, 0x78, 0x88, 0x74}

/* 00E5 (229) */
#define NXFONT_METRICS_229 {1, 6, 9, 0, 4, 0}
#define NXFONT_BITMAP_229 {0x20, 0x50, 0x20, 0x70, 0x88, 0x8, 0x78, 0x88, 0x74}

/* 00E6 (230) */
#define NXFONT_METRICS_230 {2, 9, 6, 0, 7, 0}
#define NXFONT_BITMAP_230 {0x77, 0x0, 0x88, 0x80, 0xf, 0x0, 0x78, 0x0, 0x88, 0x80, 0x77, 0x0}

/* 00E7 (231) */
#define NXFONT_METRICS_231 {1, 5, 8, 0, 7, 0}
#define NXFONT_BITMAP_231 {0x70, 0x88, 0x80, 0x80, 0x88, 0x70, 0x20, 0x60}

/* 00E8 (232) */
#define NXFONT_METRICS_232 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_232 {0x40, 0x20, 0x0, 0x70, 0x88, 0xf0, 0x80, 0x88, 0x70}

/* 00E9 (233) */
#define NXFONT_METRICS_233 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_233 {0x10, 0x20, 0x0, 0x70, 0x88, 0xf0, 0x80, 0x88, 0x70}

/* 00EA (234) */
#define NXFONT_METRICS_234 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_234 {0x20, 0x50, 0x0, 0x70, 0x88, 0xf0, 0x80, 0x88, 0x70}

/* 00EB (235) */
#define NXFONT_METRICS_235 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_235 {0x50, 0x0, 0x70, 0x88, 0xf0, 0x80, 0x88, 0x70}

/* 00EC (236) */
#define NXFONT_METRICS_236 {1, 2, 9, 0, 4, 0}
#define NXFONT_BITMAP_236 {0x80, 0x40, 0x0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}

/* 00ED (237) */
#define NXFONT_METRICS_237 {1, 2, 9, 0, 4, 0}
#define NXFONT_BITMAP_237 {0x40, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* 00EE (238) */
#define NXFONT_METRICS_238 {1, 3, 9, 0, 4, 0}
#define NXFONT_BITMAP_238 {0x40, 0xa0, 0x0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}

/* 00EF (239) */
#define NXFONT_METRICS_239 {1, 3, 8, 0, 5, 0}
#define NXFONT_BITMAP_239 {0xa0, 0x0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}

/* 00F0 (240) */
#define NXFONT_METRICS_240 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_240 {0xa0, 0x40, 0xa0, 0x10, 0x70, 0x88, 0x88, 0x88, 0x70}

/* 00F1 (241) */
#define NXFONT_METRICS_241 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_241 {0x68, 0x90, 0x0, 0xb0, 0xc8, 0x88, 0x88, 0x88, 0x88}

/* 00F2 (242) */
#define NXFONT_METRICS_242 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_242 {0x40, 0x20, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00F3 (243) */
#define NXFONT_METRICS_243 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_243 {0x10, 0x20, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00F4 (244) */
#define NXFONT_METRICS_244 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_244 {0x20, 0x50, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00F5 (245) */
#define NXFONT_METRICS_245 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_245 {0x48, 0xb0, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00F6 (246) */
#define NXFONT_METRICS_246 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_246 {0x50, 0x0, 0x70, 0x88, 0x88, 0x88, 0x88, 0x70}

/* 00F7 (247) */
#define NXFONT_METRICS_247 {1, 6, 5, 0, 7, 0}
#define NXFONT_BITMAP_247 {0x30, 0x0, 0xfc, 0x0, 0x30}

/* 00F8 (248) */
#define NXFONT_METRICS_248 {1, 6, 6, 0, 7, 0}
#define NXFONT_BITMAP_248 {0x74, 0x88, 0x98, 0xa8, 0xc8, 0x70}

/* 00F9 (249) */
#define NXFONT_METRICS_249 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_249 {0x40, 0x20, 0x88, 0x88, 0x88, 0x88, 0x98, 0x68}

/* 00FA (250) */
#define NXFONT_METRICS_250 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_250 {0x10, 0x20, 0x88, 0x88, 0x88, 0x88, 0x98, 0x68}

/* 00FB (251) */
#define NXFONT_METRICS_251 {1, 5, 9, 0, 4, 0}
#define NXFONT_BITMAP_251 {0x20, 0x50, 0x0, 0x88, 0x88, 0x88, 0x88, 0x98, 0x68}

/* 00FC (252) */
#define NXFONT_METRICS_252 {1, 5, 8, 0, 5, 0}
#define NXFONT_BITMAP_252 {0x50, 0x0, 0x88, 0x88, 0x88, 0x88, 0x98, 0x68}

/* 00FD (253) */
#define NXFONT_METRICS_253 {1, 5, 10, 0, 5, 0}
#define NXFONT_BITMAP_253 {0x10, 0x20, 0x88, 0x88, 0x50, 0x50, 0x20, 0x20, 0x40, 0x40}

/* 00FE (254) */
#define NXFONT_METRICS_254 {1, 5, 9, 0, 5, 0}
#define NXFONT_BITMAP_254 {0x80, 0x80, 0xb0, 0xc8, 0x88, 0xc8, 0xb0, 0x80, 0x80}

/* 00FF (255) */
#define NXFONT_METRICS_255 {1, 5, 10, 0, 5, 0}
#define NXFONT_BITMAP_255 {0x50, 0x0, 0x88, 0x88, 0x50, 0x50, 0x20, 0x20, 0x40, 0x80}

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
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __LIBNX_NXFONTS_NXFONTS_PIXEL_UNICODE_H */
