/****************************************************************************
 * arch/arm/src/dm320/dm320_osd.h
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

#ifndef __ARCH_ARM_SRC_DM320_DM320_OSD_H
#define __ARCH_ARM_SRC_DM320_DM320_OSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* On Screen Display Register Map (OSD) *************************************/

#define DM320_OSD_OSDMODE     (DM320_OSD_REGISTER_BASE+0x0000) /* OSD Mode Setup */
#define DM320_OSD_VIDWINMD    (DM320_OSD_REGISTER_BASE+0x0002) /* Video Window Mode Setup */
#define DM320_OSD_OSDWIN0MD   (DM320_OSD_REGISTER_BASE+0x0004) /* OSD Window 0 Mode Setup */
#define DM320_OSD_OSDWIN1MD   (DM320_OSD_REGISTER_BASE+0x0006) /* OSD Window 1 Mode Setup */
#define DM320_OSD_OSDATRMD    (DM320_OSD_REGISTER_BASE+0x0006) /* OSD Attribute Window Mode Setup */
#define DM320_OSD_RECTCUR     (DM320_OSD_REGISTER_BASE+0x0008) /* Rectangular Cursor Setup */
#define DM320_OSD_VIDWIN0OFST (DM320_OSD_REGISTER_BASE+0x000c) /* Video Window 0 Offset */
#define DM320_OSD_VIDWIN1OFST (DM320_OSD_REGISTER_BASE+0x000e) /* Video Window 1 Offset */
#define DM320_OSD_OSDWIN0OFST (DM320_OSD_REGISTER_BASE+0x0010) /* OSD Window 0 Offset */
#define DM320_OSD_OSDWIN1OFST (DM320_OSD_REGISTER_BASE+0x0012) /* OSD Window 1 Offset */
#define DM320_OSD_VIDWINADH   (DM320_OSD_REGISTER_BASE+0x0014) /* Video Window 0/1 Address - High */
#define DM320_OSD_VIDWIN0ADL  (DM320_OSD_REGISTER_BASE+0x0016) /* Video Window 0 Address - Low */
#define DM320_OSD_VIDWIN1ADL  (DM320_OSD_REGISTER_BASE+0x0018) /* Video Window 1 Address - Low */
#define DM320_OSD_OSDWINADH   (DM320_OSD_REGISTER_BASE+0x001a) /* OSD Window 0/1 Address - High */
#define DM320_OSD_OSDWIN0ADL  (DM320_OSD_REGISTER_BASE+0x001c) /* OSD Window 0 Address - Low */
#define DM320_OSD_OSDWIN1ADL  (DM320_OSD_REGISTER_BASE+0x001e) /* OSD Window 1 Address - Low */
#define DM320_OSD_BASEPX      (DM320_OSD_REGISTER_BASE+0x0020) /* Base Pixel X */
#define DM320_OSD_BASEPY      (DM320_OSD_REGISTER_BASE+0x0022) /* Base Pixel Y */
#define DM320_OSD_VIDWIN0XP   (DM320_OSD_REGISTER_BASE+0x0024) /* Video Window 0 X-Position */
#define DM320_OSD_VIDWIN0YP   (DM320_OSD_REGISTER_BASE+0x0026) /* Video Window 0 Y-Position */
#define DM320_OSD_VIDWIN0XL   (DM320_OSD_REGISTER_BASE+0x0028) /* Video Window 0 X-Size */
#define DM320_OSD_VIDWIN0YL   (DM320_OSD_REGISTER_BASE+0x002a) /* Video Window 0 Y-Size */
#define DM320_OSD_VIDWIN1XP   (DM320_OSD_REGISTER_BASE+0x002c) /* Video Window 1 X-Position */
#define DM320_OSD_VIDWIN1YP   (DM320_OSD_REGISTER_BASE+0x002e) /* Video Window 1 Y-Position */
#define DM320_OSD_VIDWIN1XL   (DM320_OSD_REGISTER_BASE+0x0030) /* Video Window 1 X-Size */
#define DM320_OSD_VIDWIN1YL   (DM320_OSD_REGISTER_BASE+0x0032) /* Video Window 1 Y-Size */
#define DM320_OSD_OSDWIN0XP   (DM320_OSD_REGISTER_BASE+0x0034) /* OSD Bitmap Window 0 X-Position */
#define DM320_OSD_OSDWIN0YP   (DM320_OSD_REGISTER_BASE+0x0036) /* OSD Bitmap Window 0 Y-Position */
#define DM320_OSD_OSDWIN0XL   (DM320_OSD_REGISTER_BASE+0x0038) /* OSD Bitmap Window 0 X-Size */
#define DM320_OSD_OSDWIN0YL   (DM320_OSD_REGISTER_BASE+0x003a) /* OSD Bitmap Window 0 Y-Size */
#define DM320_OSD_OSDWIN1XP   (DM320_OSD_REGISTER_BASE+0x003c) /* OSD Bitmap Window 1 X-Position */
#define DM320_OSD_OSDWIN1YP   (DM320_OSD_REGISTER_BASE+0x003e) /* OSD Bitmap Window 1 Y-Position */
#define DM320_OSD_OSDWIN1XL   (DM320_OSD_REGISTER_BASE+0x0040) /* OSD Bitmap Window 1 X-Size */
#define DM320_OSD_OSDWIN1YL   (DM320_OSD_REGISTER_BASE+0x0042) /* OSD Bitmap Window 1 Y-Size */
#define DM320_OSD_CURXP       (DM320_OSD_REGISTER_BASE+0x0044) /* Rectangular Cursor Window X-Position */
#define DM320_OSD_CURYP       (DM320_OSD_REGISTER_BASE+0x0046) /* Rectangular Cursor Window Y-Position */
#define DM320_OSD_CURXL       (DM320_OSD_REGISTER_BASE+0x0048) /* Rectangular Cursor Window X-Size */
#define DM320_OSD_CURYL       (DM320_OSD_REGISTER_BASE+0x004a) /* Rectangular Cursor Window Y-Size */
#define DM320_OSD_W0BMP01     (DM320_OSD_REGISTER_BASE+0x0050) /* Window 0 Bitmap Value to Palette Map 0/1 */
#define DM320_OSD_W0BMP23     (DM320_OSD_REGISTER_BASE+0x0052) /* Window 0 Bitmap Value to Palette Map 2/3 */
#define DM320_OSD_W0BMP45     (DM320_OSD_REGISTER_BASE+0x0054) /* Window 0 Bitmap Value to Palette Map 4/5 */
#define DM320_OSD_W0BMP67     (DM320_OSD_REGISTER_BASE+0x0056) /* Window 0 Bitmap Value to Palette Map 6/7 */
#define DM320_OSD_W0BMP89     (DM320_OSD_REGISTER_BASE+0x0058) /* Window 0 Bitmap Value to Palette Map 8/9 */
#define DM320_OSD_W0BMPAB     (DM320_OSD_REGISTER_BASE+0x005a) /* Window 0 Bitmap Value to Palette Map A/B */
#define DM320_OSD_W0BMPCD     (DM320_OSD_REGISTER_BASE+0x005c) /* Window 0 Bitmap Value to Palette Map C/D */
#define DM320_OSD_W0BMPEF     (DM320_OSD_REGISTER_BASE+0x005e) /* Window 0 Bitmap Value to Palette Map E/F */
#define DM320_OSD_W1BMP01     (DM320_OSD_REGISTER_BASE+0x0060) /* Window 1 Bitmap Value to Palette Map 0/1 */
#define DM320_OSD_W1BMP23     (DM320_OSD_REGISTER_BASE+0x0062) /* Window 1 Bitmap Value to Palette Map 2/3 */
#define DM320_OSD_W1BMP45     (DM320_OSD_REGISTER_BASE+0x0064) /* Window 1 Bitmap Value to Palette Map 4/5 */
#define DM320_OSD_W1BMP67     (DM320_OSD_REGISTER_BASE+0x0066) /* Window 1 Bitmap Value to Palette Map 6/7 */
#define DM320_OSD_W1BMP89     (DM320_OSD_REGISTER_BASE+0x0068) /* Window 1 Bitmap Value to Palette Map 8/9 */
#define DM320_OSD_W1BMPAB     (DM320_OSD_REGISTER_BASE+0x006a) /* Window 1 Bitmap Value to Palette Map A/B */
#define DM320_OSD_W1BMPCD     (DM320_OSD_REGISTER_BASE+0x006c) /* Window 1 Bitmap Value to Palette Map C/D */
#define DM320_OSD_W1BMPEF     (DM320_OSD_REGISTER_BASE+0x006e) /* Window 1 Bitmap Value to Palette Map E/F */
#define DM320_OSD_MISCCTL     (DM320_OSD_REGISTER_BASE+0x0074) /* Miscellaneous Control */
#define DM320_OSD_CLUTRAMYCB  (DM320_OSD_REGISTER_BASE+0x0076) /* CLUT RAM Y/Cb Setup */
#define DM320_OSD_CLUTRAMCR   (DM320_OSD_REGISTER_BASE+0x0078) /* CLUT RAM Cr/Mapping Setup */
#define DM320_OSD_RSV5        (DM320_OSD_REGISTER_BASE+0x007a) /* CLUT RAM Cr/Mapping Setup */
#define DM320_OSD_PPVWIN0ADH  (DM320_OSD_REGISTER_BASE+0x007c) /* Ping-Pong Video Window 0 Address (High) */
#define DM320_OSD_PPVWIN0ADL  (DM320_OSD_REGISTER_BASE+0x007e) /* Ping-Pong Video Window 0 Address (Low) */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_DM320_DM320_OSD_H */
