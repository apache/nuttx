/*
 * Copyright (c) 2015 Broadcom
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of Broadcom nor the names of other contributors to this
 * software may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * 4. This software may not be used as a standalone product, and may only be used as
 * incorporated in your product or device that incorporates Broadcom wireless connectivity
 * products and solely for the purpose of enabling the functionalities of such Broadcom products.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY WARRANTIES OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT, ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BCM43362_CONSTANTS_H_
#define BCM43362_CONSTANTS_H_

/******************************************************
 *             Architecture Constants
 ******************************************************/

/* General chip stats */
#define CHIP_RAM_SIZE      0x3C000

/* Backplane architecture */
#define CHIPCOMMON_BASE_ADDRESS  0x18000000    /* Chipcommon core register region   */
#define DOT11MAC_BASE_ADDRESS    0x18001000    /* dot11mac core register region     */
#define SDIO_BASE_ADDRESS        0x18002000    /* SDIOD Device core register region */
#define WLAN_ARMCM3_BASE_ADDRESS 0x18003000    /* ARMCM3 core register region       */
#define SOCSRAM_BASE_ADDRESS     0x18004000    /* SOCSRAM core register region      */
#define BACKPLANE_ADDRESS_MASK   0x7FFF

/* Maximum value of bus data credit difference */
#define CHIP_MAX_BUS_DATA_CREDIT_DIFF    7

/* Chipcommon registers */
#define CHIPCOMMON_GPIO_CONTROL ((uint32_t) (CHIPCOMMON_BASE_ADDRESS + 0x6C) )

/******************************************************
 *             Bit Masks
 ******************************************************/

#define WL_CHANSPEC_BAND_MASK             (0xf000)
#define WL_CHANSPEC_BAND_5G               (0x1000)
#define WL_CHANSPEC_BAND_2G               (0x2000)
#define WL_CHANSPEC_CTL_SB_MASK           (0x0300)
#define WL_CHANSPEC_CTL_SB_LOWER          (0x0100)
#define WL_CHANSPEC_CTL_SB_UPPER          (0x0200)
#define WL_CHANSPEC_CTL_SB_NONE           (0x0300)
#define WL_CHANSPEC_BW_MASK               (0x0C00)
#define WL_CHANSPEC_BW_10                 (0x0400)
#define WL_CHANSPEC_BW_20                 (0x0800)
#define WL_CHANSPEC_BW_40                 (0x0C00)

#endif /* ifndef BCM43362_CONSTANTS_H_ */
