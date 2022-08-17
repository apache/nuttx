/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_firmware.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _STR(x) #x
#define STR(x) _STR(x)

#define ROUNDED_FIRMWARE_LEN ((CONFIG_CYW43439_FIRMWARE_LEN + 0xff) & ~0xff)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_IEEE80211_INFINEON_CYW43439

/****************************************************************************
 * Character array of NVRAM image
 * Generated from cyw943439wlpth_rev1_0.txt
 * $ Copyright Broadcom Corporation $
 ****************************************************************************/

const uint8_t g_cyw43439_nvram_image[] __attribute__((aligned(4))) =
        "NVRAMRev=$Rev$"                         "\x00"
        "manfid=0x2d0"                           "\x00"
        "prodid=0x0727"                          "\x00"
        "vendid=0x14e4"                          "\x00"
        "devid=0x43e2"                           "\x00"
        "boardtype=0x0887"                       "\x00"
        "boardrev=0x1100"                        "\x00"
        "boardnum=22"                            "\x00"
        "macaddr=00:A0:50:b5:59:5e"              "\x00"
        "sromrev=11"                             "\x00"
        "boardflags=0x00404001"                  "\x00"
        "boardflags3=0x04000000"                 "\x00"
        "xtalfreq=37400"                         "\x00"
        "nocrc=1"                                "\x00"
        "ag0=255"                                "\x00"
        "aa2g=1"                                 "\x00"
        "ccode=ALL"                              "\x00"
        "pa0itssit=0x20"                         "\x00"
        "extpagain2g=0"                          "\x00"
        "pa2ga0=-168,6649,-778"                  "\x00"
        "AvVmid_c0=0x0,0xc8"                     "\x00"
        "cckpwroffset0=5"                        "\x00"
        "maxp2ga0=84"                            "\x00"
        "txpwrbckof=6"                           "\x00"
        "cckbw202gpo=0"                          "\x00"
        "legofdmbw202gpo=0x66111111"             "\x00"
        "mcsbw202gpo=0x77711111"                 "\x00"
        "propbw202gpo=0xdd"                      "\x00"
        "ofdmdigfilttype=18"                     "\x00"
        "ofdmdigfilttypebe=18"                   "\x00"
        "papdmode=1"                             "\x00"
        "papdvalidtest=1"                        "\x00"
        "pacalidx2g=45"                          "\x00"
        "papdepsoffset=-30"                      "\x00"
        "papdendidx=58"                          "\x00"
        "ltecxmux=0"                             "\x00"
        "ltecxpadnum=0x0102"                     "\x00"
        "ltecxfnsel=0x44"                        "\x00"
        "ltecxgcigpio=0x01"                      "\x00"
        "il0macaddr=00:90:4c:c5:12:38"           "\x00"
        "wl0id=0x431b"                           "\x00"
        "deadman_to=0xffffffff"                  "\x00"
        "muxenab=0x100"                          "\x00"
        "spurconfig=0x3"                         "\x00"
        "glitch_based_crsmin=1"                  "\x00"
        "btc_mode=1"                             "\x00"
        "\x00\x00";

const unsigned int   g_cyw43439_nvram_len = sizeof(g_cyw43439_nvram_image);

/****************************************************************************
 * Include the firmware blob.  This assembly code defines the global
 * symbol g_cyw43439_firmware_image as the address of this included
 * blob. This is similar in effect to the C statement:
 *     const uint8_t g_cyw43439_firmware_image[]  = {...};
 *     const uint8_t ng_cyw43439_clm_blob_image[] = {...};
 ****************************************************************************/

/* These are defined as array because the symbols name an actual address
 * not a pointer to an address.  This is not one of the many cases where
 * pointers and arrays are interchangable in C.
 */

extern const uint8_t       g_cyw43439_firmware_image[];
extern const uint8_t       g_cyw43439_clm_blob_image[];
extern const unsigned int  g_cyw43439_clm_blob_len;

/* This assembly code does the following:
 *    - Force 16-byte alignment
 *    - Defines g_cyw43439_firmware_image as a location in memory
 *    - Copies the firmware image file data to that location.
 *    - Defines g_cyw43439_firmware_end as the location directly beyond
 *      that data.
 *    - Defines g_cyw43439_clm_blob_image as a location withing that
 *      data where the clm_blob begins.
 *    - Force 4-byte alignment
 *    - Allocates an integer named g_cyw43439_clm_blob_len that
 *      contains the length of the clm_blob.
 */

__asm__("\n     .balign  16"
        "\n     .globl   g_cyw43439_firmware_image"
        "\n     .globl   g_cyw43439_clm_blob_image"
        "\n     .globl   g_cyw43439_clm_blob_len"
        "\n g_cyw43439_firmware_image:"
        "\n     .incbin  \"cyw43439.firmware.image\""
        "\n firmware_end:"
        "\n g_cyw43439_clm_blob_image=g_cyw43439_firmware_image+"
                         STR(ROUNDED_FIRMWARE_LEN)
        "\n     .balign  4"
        "\n g_cyw43439_clm_blob_len:"
        "\n     .word    firmware_end-g_cyw43439_clm_blob_image"
        "\n");

/****************************************************************************
 * Other CYW43439 Firmware global definitions
 ****************************************************************************/

#ifndef CONFIG_IEEE80211_BROADCOM_FWFILES

const unsigned int  g_cyw43439_firmware_len = CONFIG_CYW43439_FIRMWARE_LEN;

#endif /* CONFIG_IEEE80211_BROADCOM_FWFILES */

#endif /* CONFIG_IEEE80211_INFINEON_CYW43439 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/
