/*****************************************************************************
 * arch/arm/src/tiva/tiv_chipinfo.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI file that has a compatible BSD
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

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <assert.h>

#include "hardware/tiva_prcm.h"
#include "tiva_chipinfo.h"

/*****************************************************************************
 * Public Functions
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

enum cc13xx_protocol_e chipinfo_protocols(void)
{
  /* Return allowed RTC modes.
   * REVISIT:  Per fcfg1 header file, the allowed RGC modes are in bits 0-2. */

   uint32_t regval = getreg32(TIVA_PRCM_RFCMODEHWOPT);
   return (enum cc13xx_protocol_e)(regval & 0x0e);
}

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

enum cc13xx_package_e chipinfo_packagetype(void)
{
  uint32_t regval = getreg32(TIVA_FCFG1_USER_ID);
  enum cc13xx_package_e pkgtype;

  pkgtype = (enum cc13xx_package_e)((regval & FCFG1_USER_ID_PKG_MASK) >>
                                    FCFG1_USER_ID_PKG_SHIFT);

  if (pkgtype < PACKAGE_4x4 || pkgtype > PACKAGE_4x4)
    {
      pkgtype = PACKAGE_UNKNOWN;
    }

  return pkgtype;
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

enum cc13xx_chiptype_e chipinfo_chiptype(void)
{
  enum cc13xx_chiptype_e chiptype;
  enum cc13xx_chipfamily_e chipfamily;
  uint32_t userid;
  uint32_t protocol;
#if defined(CONFIG_ARCH_CHIP_CC13X2)
  bool cc13;
  bool pa;
#endif

  chiptype    = CHIP_TYPE_UNKNOWN;
  chipfamily  = chipinfo_chipfamily();
  userid      = chipinfo_userid();
  protocol    = ((userid & FCFG1_USER_ID_PROTOCOL_MASK) >>
                 FCFG1_USER_ID_PROTOCOL_SHIFT);

#if defined(CONFIG_ARCH_CHIP_CC13X0)
  if (chipfamily == FAMILY_CC13x0)
    {
      switch (protocol)
        {
         case 0x8:
            chiptype = CHIP_TYPE_CC1310;
            break;

         case 0xf:
            chiptype = CHIP_TYPE_CC1350;
            break;
         }
    }

#elif defined(CONFIG_ARCH_CHIP_CC13X2)

   cc13 = ((userid & FCFG1_USER_ID_CC13) != 0); /*  CC13xx device type (vs CC26xx) */
   pa   = ((userid & FCFG1_USER_ID_PA) != 0);   /*  Supports 20dBM PA */

   if (chipfamily == FAMILY_CC13x2_CC26x2)
     {
       switch (protocol)
         {
          case 0xf:
            if (cc13)
              {
                if (pa)
                  {
                   chiptype = CHIP_TYPE_CC1352P;
                  }
                else
                  {
                     chiptype = CHIP_TYPE_CC1352;
                  }
              }
            else
              {
                chiptype = CHIP_TYPE_CC2652;
              }

           break;

      case 0x9:
         if (pa)
           {
             chiptype = CHIP_TYPE_UNUSED;
           }
         else
           {
            chiptype = CHIP_TYPE_CC2642;
           }

         break;

      case 0x8 :
         chiptype = CHIP_TYPE_CC1312;
         break;
      }
   }
#endif

  return chiptype;
}

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

enum cc13xx_chipfamily_e chipinfo_chipfamily(void)
{
  enum cc13xx_chipfamily_e chipfamily = FAMILY_UNKNOWN;
  uint32_t regval;
  uint32_t waferid;

  regval  = getreg32(TIVA_FCFG1_ICEPICK_DEVICE_ID);
  waferid = (regval & FCFG1_ICEPICK_DEVICE_ID_WAFER_ID_MASK) >>
                      FCFG1_ICEPICK_DEVICE_ID_WAFER_ID_SHIFT;

#if defined(CONFIG_ARCH_CHIP_CC13X0)
  if (waferid == 0xb9be)
    {
      chipfamily = FAMILY_CC13x0;
     }

#elif defined(CONFIG_ARCH_CHIP_CC13X2)
  if (waferid == 0xbb41)
    {
      chipfamily = FAMILY_CC13x2_CC26x2;
     }
#endif

   return chipfamily;
}

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

enum cc13xx_revision_e chipinfo_hwrevision(void)
{
  enum cc13xx_revision_e hwrev;
  enum cc13xx_chipfamily_e chipfamily;
  uint32_t fcg1rev;
  uint32_t hwminorrev;

  hwrev      = HWREV_UNKNOWN;
  chipfamily = chipinfo_chipfamily();
  fcg1rev    = chipinfo_hwrevcode();
  hwminorrev = chipinfo_hwminorrev();

#if defined(CONFIG_ARCH_CHIP_CC13X0)
  if (chipfamily == FAMILY_CC13x0)
    {
      switch (fcg1rev)
        {
          case 0:  /* CC13x0 PG1.0 */
            hwrev = HWREV_1_0;
            break;

          case 2:  /* CC13x0 PG2.0 (or later) */
            hwrev = (enum cc13xx_revision_e)(((uint32_t)HWREV_2_0) + hwminorrev);
            break;
        }
    }

#elif defined(CONFIG_ARCH_CHIP_CC13X2)
  if (chipfamily == FAMILY_CC13x2_CC26x2)
    {
      switch (fcg1rev)
        {
          case 0:  /* CC13x2, CC26x2 - PG1.0 */
          case 1:  /* CC13x2, CC26x2 - PG1.01 (will also show up as PG1.0) */
            hwrev = (enum cc13xx_revision_e)((uint32_t)HWREV_1_0);
            break;

          case 2:  /* CC13x2, CC26x2 - PG1.1 (or later) */
            hwrev = (enum cc13xx_revision_e)(((uint32_t)HWREV_1_1) + hwminorrev);
            break;

          case 3:  /* CC13x2, CC26x2 - PG2.1 (or later) */
            hwrev = (enum cc13xx_revision_e)(((uint32_t)HWREV_2_1) + hwminorrev);
            break;
        }
    }

#endif

   return hwrev;
}

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
void chipinfo_verify(void)
{
  enum cc13xx_chipfamily_e chip_family;
  enum cc13xx_revision_e hwrev;

  chip_family = chipinfo_chipfamily();

#if defined(CONFIG_ARCH_CHIP_CC13X0)
  DEBUGASSERT(chip_family == FAMILY_CC13x0);
#elif defined(CONFIG_ARCH_CHIP_CC13X2)
  DEBUGASSERT(chip_family == FAMILY_CC13x2_CC26x2);
#else
  DEBUPANIC();
#endif

  hwrev = chipinfo_hwrevision();

#if defined(CONFIG_ARCH_CHIP_CC13XX_V1)
  DEBUGASSERT(hwrev >= HWREV_1_0 && hwrev < HWREV_2_0);
#elif defined(CONFIG_ARCH_CHIP_CC13XX_V2)
  DEBUGASSERT(hwrev >= HWREV_2_0);
#else
  DEBUPANIC();
#endif
}
#endif
