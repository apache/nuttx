/*****************************************************************************
 * arch/arm/src/tiva/tiv_chipinfo.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible BSD
 * license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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
 *****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_CHIPINFO_H
#define __ARCH_ARM_SRC_TIVA_TIVA_CHIPINFO_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "arm_arch.h"
#include "hardware/tiva_fcfg1.h"

/* Currently only applies to the CC13x0 and CC13x2 families */

#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)

/*****************************************************************************
 * Public Types
 *****************************************************************************/

enum cc13xx_protocol_e
{
  PROTOCOL_UNKNOWN       = 0x00, /* None of the known protocols are supported */
  PROTOCOL_BLE           = 0x02, /* Bit[1] Bluetooth Low Energy is supported */
  PROTOCOL_IEEE_802_15_4 = 0x04, /* Bit[2] IEEE 802.15.4 is supported */
  PROTOCOL_PROPRIETARY   = 0x08  /* Bit[3] proprietary protocols are supported */
};

enum cc13xx_package_e
{
  PACKAGE_UNKNOWN        = -1,   /* -1 means that current package type is unknown */
  PACKAGE_4x4            =  0,   /*  0 This is a 4x4 mm QFN (RHB) package */
  PACKAGE_5x5            =  1,   /*  1 This is a 5x5 mm QFN (RSM) package */
  PACKAGE_7x7            =  2,   /*  2 This is a 7x7 mm QFN (RGZ) package */
  PACKAGE_WAFER          =  3,   /*  3 This is a wafer sale package (naked die) */
  PACKAGE_WCSP           =  4,   /*  4 This is a 2.7x2.7 mm WCSP (YFV) */
  PACKAGE_7x7_Q1         =  5    /*  5 This is a 7x7 mm QFN package with Wettable Flanks */
};

enum cc13xx_chipfamily_e
{
  FAMILY_UNKNOWN         = -1,   /* -1 The chip's family member is unknown */
  FAMILY_CC26x0          =  0,   /*  0 The chip is a CC26x0 family member */
  FAMILY_CC13x0          =  1,   /*  1 The chip is a CC13x0 family member */
  FAMILY_CC26x1          =  2,   /*  2 The chip is a CC26x1 family member */
  FAMILY_CC26x0R2        =  3,   /*  3 The chip is a CC26x0R2 family (new ROM contents) */
  FAMILY_CC13x2_CC26x2   =  4    /*  4 The chip is a CC13x2, CC26x2 family member */
};

enum cc13xx_chiptype_e
{
  CHIP_TYPE_UNKNOWN      = -1,   /* -1 The chip type is unknown */
  CHIP_TYPE_CC1310       =  0,   /*  0 This is a CC1310 chip */
  CHIP_TYPE_CC1350       =  1,   /*  1 This is a CC1350 chip */
  CHIP_TYPE_CC2620       =  2,   /*  2 This is a CC2620 chip */
  CHIP_TYPE_CC2630       =  3,   /*  3 This is a CC2630 chip */
  CHIP_TYPE_CC2640       =  4,   /*  4 This is a CC2640 chip */
  CHIP_TYPE_CC2650       =  5,   /*  5 This is a CC2650 chip */
  CHIP_TYPE_CUSTOM_0     =  6,   /*  6 This is a CUSTOM_0 chip */
  CHIP_TYPE_CUSTOM_1     =  7,   /*  7 This is a CUSTOM_1 chip */
  CHIP_TYPE_CC2640R2     =  8,   /*  8 This is a CC2640R2 chip */
  CHIP_TYPE_CC2642       =  9,   /*  9 This is a CC2642 chip */
  CHIP_TYPE_UNUSED       =  10,  /* 10 unused value */
  CHIP_TYPE_CC2652       =  11,  /* 11 This is a CC2652 chip */
  CHIP_TYPE_CC1312       =  12,  /* 12 This is a CC1312 chip */
  CHIP_TYPE_CC1352       =  13,  /* 13 This is a CC1352 chip */
  CHIP_TYPE_CC1352P      =  14   /* 14 This is a CC1352P chip */
};

enum cc13xx_revision_e
{
  HWREV_UNKNOWN          = -1,   /* -1 The chip's HW revision is unknown */
  HWREV_1_0              =  10,  /* 10 The chip's HW revision is 1.0 */
  HWREV_1_1              =  11,  /* 11 The chip's HW revision is 1.1 */
  HWREV_2_0              =  20,  /* 20 The chip's HW revision is 2.0 */
  HWREV_2_1              =  21,  /* 21 The chip's HW revision is 2.1 */
  HWREV_2_2              =  22,  /* 22 The chip's HW revision is 2.2 */
  HWREV_2_3              =  23,  /* 23 The chip's HW revision is 2.3 */
  HWREV_2_4              =  24   /* 24 The chip's HW revision is 2.4 */
};

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/*****************************************************************************
 * Name: chipinfo_protocols
 *
 * Description:
 *    Returns a bit set indicating supported protocols.
 *
 * Returned Value:
 *    Returns a bit set indicating supported protocols.
 *
 *****************************************************************************/

enum cc13xx_protocol_e chipinfo_protocols(void);

/*****************************************************************************
 * Name: chipinfo_packagetype
 *
 * Description:
 *   Returns an enumeration value indicating the package type.
 *
 * Returned Value:
 *   Returns an enumeration value indicating the package type.
 *
 *****************************************************************************/

enum cc13xx_package_e chipinfo_packagetype(void);

/*****************************************************************************
 * Name: chipinfo_hwrevcode
 *
 * Description:
 *   Returns the internal chip HW revision code (in range 0-15)
 *
 * Returned Value:
 *   Returns the internal chip HW revision code (in range 0-15)
 *
 *****************************************************************************/

static inline uint32_t chipinfo_hwrevcode(void)
{
  uint32_t regval = getreg32(TIVA_FCFG1_ICEPICK_DEVICE_ID);

  /* Returns HwRevCode = FCFG1_O_ICEPICK_DEVICE_ID[31:28] */

  return (regval &  FCFG1_ICEPICK_DEVICE_ID_PG_REV_MASK) >>
          FCFG1_ICEPICK_DEVICE_ID_PG_REV_SHIFT;
}

/*****************************************************************************
 * Name: chipinfo_hwminorrev
 *
 * Description:
 *   Returns the minor hardware revision number (in range 0-127)
 *
 *   The minor revision number is set to 0 for the first market released chip
 *   and thereafter incremented by 1 for each minor hardware change.
 *
 * Returned Value:
 *   Returns the minor hardware revision number (in range 0-127)
 *
 *****************************************************************************/

static inline uint32_t chipinfo_hwminorrev(void)
{
  uint32_t regval   = getreg32(TIVA_FCFG1_MISC_CONF_1);
  uint32_t minorrev = (regval & FCFG1_MISC_CONF_1_DEVICE_MINOR_REV_MASK) >>
                       FCFG1_MISC_CONF_1_DEVICE_MINOR_REV_SHIFT;

   if (minorrev >= 0x80)
     {
       minorrev = 0;
     }

   return minorrev;
}

/*****************************************************************************
 * Name: chipinfo_userid
 *
 * Description:
 *   Returns the 32 bits USER_ID field
 *
 *   See the Technical Reference Manual (TRM) for how to decode the USER_ID
 *   field.
 *
 * Returned Value:
 *   Returns the 32 bits USER_ID field
 *
 *****************************************************************************/

static inline uint32_t chipinfo_userid(void)
{
  return getreg32(TIVA_FCFG1_USER_ID);
}

/*****************************************************************************
 * Name: chipinfo_chiptype
 *
 * Description:
 *   Returns an enumeration value indicating the chip type
 *
 * Returned Value:
 *   Returns an enumeration value indicating the chip type
 *
 *****************************************************************************/

enum cc13xx_chiptype_e chipinfo_chiptype(void);

/*****************************************************************************
 * Name: chipinfo_chipfamily
 *
 * Description:
 *   Returns an enumeration value indicating the chip family
 *
 * Returned Value:
 *   Returns an enumeration value indicating the chip family
 *
 *****************************************************************************/

enum cc13xx_chipfamily_e chipinfo_chipfamily(void);

/*****************************************************************************
 * Name: chipinfo_hwrevision
 *
 * Description:
 *   Returns an enumeration value indicating the hardware revision of the chip
 *
 * Returned Value:
 *   Returns an enumeration value indicating the hardware revision of the chip
 *
 *****************************************************************************/

enum cc13xx_revision_e chipinfo_hwrevision(void);

/*****************************************************************************
 * Name: chipinfo_verify
 *
 * Description:
 *   Verifies that system is correctly configured for the current chip.  This
 *   function will assert if that the system is NOT correctly configured.
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

#ifdef CONFIG_DEBUG_ASSERTIONS
void chipinfo_verify(void);
#else
#  define chipinfo_verify()
#endif

#endif /* CONFIG_ARCH_CHIP_CC13X0 || CONFIG_ARCH_CHIP_CC13X2 */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_CHIPINFO_H */
