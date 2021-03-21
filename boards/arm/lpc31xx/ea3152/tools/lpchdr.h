/****************************************************************************
 * boards/arm/lpc31xx/ea3152/tools/lpchdr.h
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

#ifndef __BOARDS_ARM_LPC31XX_EA3152_TOOLS_LPCHDR_H
#define __BOARDS_ARM_LPC31XX_EA3152_TOOLS_LPCHDR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lpc31_header_s
{
                              /* OFFS DESCRIPTION */

  uint32_t vector;            /* 0x00    Valid ARM instruction. Usually this
                               *    will be a branch instruction to entry
                               *    point of the image.
                               */
  uint32_t magic;             /* 0x04    This field is used by boot ROM to
                               *    detect a valid image header. This field
                               *    should always be set to 0x41676d69.
                               */
  uint32_t execution_crc32;   /* 0x08    CRC32 value of execution part of
                               *    the image. If the ‘image_type’ is set
                               *    to ‘0xA’, this field is ignored by boot
                               *    ROM.
                               */
  uint32_t reserved0[4];      /* 0x0c-0x18: Should be zero. */
  uint32_t imagetype;         /* 0x1c Specifies whether CRC check should be
                               *    done on the image or not:
                               *      0xA – No CRC check required.
                               *      0xB – Do CRC32 check on both header and
                               *            execution part of the image.
                               */
  uint32_t imagelength;       /* 0x20    Total image length including header
                               *    rounded up to the nearest 512 byte
                               *    boundary. In C language the field can be
                               *    computed as:
                               *    imagelength = (Actual length + 511) & ~0x1FF;
                               */
  uint32_t releaseid;         /* 0x24    Release or version number of the
                               *    image.
                               *    Note, this field is not used by boot ROM
                               *    but is provided to track the image
                               *    versions.
                               */
  uint32_t buildtime;         /* 0x28 Time (expressed in EPOC time format) at
                               *    which image is built. Note, this field is
                               *    not used by boot ROM but is provided to
                               *    track the image versions.
                               */
  uint32_t sbzbootparameter;  /* 0x2c    hould be zero. */
  uint32_t cust_reserved[15]; /* 0x30-0x68: Reserved for customer use
                               *                   (60 bytes)
                               */
  uint32_t header_crc32;      /* 0x6c CRC32 value of the header
                               *    (bytes 0x00 to 0x6C of the image).
                               *    If the ‘image_type’ is set to ‘0xA’,
                               *    this field is ignored by boot ROM.
                               */
  uint32_t reserved1[4];      /* 0x70-0x7c: Should be zero. */
                              /* 0x80    Start of program code (128Kb max).
                               *    The final image has to be padded to the
                               *    nearest 512 byte boundary
                               */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

extern uint32_t crc32part(const uint8_t *src, size_t len, uint32_t crc32val);
extern uint32_t crc32(const uint8_t *src, size_t len);

#endif /* __BOARDS_ARM_LPC31XX_EA3152_TOOLS_LPCHDR_H */
