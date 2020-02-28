/************************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_k28k66fmc.h
 *
 *   Copyright (C) 2016-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_K28K66FMC_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_K28K66FMC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_FMC_PFAPR_OFFSET        0x0000 /* Flash Access Protection Register */
#define KINETIS_FMC_PFB01CR_OFFSET      0x0004 /* Flash Bank 0-1 Control Register */
#define KINETIS_FMC_PFB23CR_OFFSET      0x0008 /* Flash Bank 2-3 Control Register */

/* Cache Directory Storage for way=w and set=s, w=0..3, s=0..7 */

#define KINETIS_FMC_TAGVD_OFFSET(w,s)   (0x100 + ((w) << 5) + ((s) << 2))

#define KINETIS_FMC_TAGVDW0S0_OFFSET    0x0100 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW0S1_OFFSET    0x0104 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW0S2_OFFSET    0x0108 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW0S3_OFFSET    0x010c /* Cache Directory Storage */

#define KINETIS_FMC_TAGVDW1S0_OFFSET    0x0110 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW1S1_OFFSET    0x0114 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW1S2_OFFSET    0x0118 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW1S3_OFFSET    0x011c /* Cache Directory Storage */

#define KINETIS_FMC_TAGVDW2S0_OFFSET    0x0120 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW2S1_OFFSET    0x0124 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW2S2_OFFSET    0x0128 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW2S3_OFFSET    0x012c /* Cache Directory Storage */

#define KINETIS_FMC_TAGVDW3S0_OFFSET    0x0130 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW3S1_OFFSET    0x0134 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW3S2_OFFSET    0x0138 /* Cache Directory Storage */
#define KINETIS_FMC_TAGVDW3S3_OFFSET    0x013c /* Cache Directory Storage */

/* Cache Data Storage (UpperMost, MidUpper, MidLower LowerMost) for way=w and set=s, w=0..3, s=0..7 */

#define KINETIS_FMC_DATAUM_OFFSET(w,s)  (0x200 + ((w) << 6) + ((s) << 2))
#define KINETIS_FMC_DATAMU_OFFSET(w,s)  (0x204 + ((w) << 6) + ((s) << 2))
#define KINETIS_FMC_DATAML_OFFSET(w,s)  (0x208 + ((w) << 6) + ((s) << 2))
#define KINETIS_FMC_DATALM_OFFSET(w,s)  (0x20c + ((w) << 6) + ((s) << 2))

#define KINETIS_FMC_DATAW0S0UM_OFFSET   0x0200 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW0S0MU_OFFSET   0x0204 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW0S0ML_OFFSET   0x0208 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW0S0LM_OFFSET   0x020c /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW0S1UM_OFFSET   0x0210 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW0S1MU_OFFSET   0x0214 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW0S1ML_OFFSET   0x0218 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW0S1LM_OFFSET   0x021c /* Cache Data Storage (lowermost word) */

#define KINETIS_FMC_DATAW0S2UM_OFFSET   0x0220 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW0S2MU_OFFSET   0x0224 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW0S2ML_OFFSET   0x0228 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW0S2LM_OFFSET   0x022c /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW0S3UM_OFFSET   0x0230 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW0S3MU_OFFSET   0x0234 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW0S3ML_OFFSET   0x0238 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW0S3LM_OFFSET   0x023c /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW1S0UM_OFFSET   0x0240 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW1S0MU_OFFSET   0x0244 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW1S0ML_OFFSET   0x0248 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW1S0LM_OFFSET   0x024c /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW1S1UM_OFFSET   0x0250 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW1S1MU_OFFSET   0x0254 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW1S1ML_OFFSET   0x0258 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW1S1LM_OFFSET   0x025c /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW1S2UM_OFFSET   0x0260 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW1S2MU_OFFSET   0x0264 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW1S2ML_OFFSET   0x0268 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW1S2LM_OFFSET   0x026c /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW1S3UM_OFFSET   0x0270 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW1S3MU_OFFSET   0x0274 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW1S3ML_OFFSET   0x0278 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW1S3LM_OFFSET   0x027c /* Cache Data Storage (lowermost word) */

#define KINETIS_FMC_DATAW2S0UM_OFFSET   0x0280 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW2S0MU_OFFSET   0x0284 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW2S0ML_OFFSET   0x0288 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW2S0LM_OFFSET   0x028c /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW2S1UM_OFFSET   0x0290 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW2S1MU_OFFSET   0x0294 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW2S1ML_OFFSET   0x0298 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW2S1LM_OFFSET   0x029c /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW2S2UM_OFFSET   0x02a0 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW2S2MU_OFFSET   0x02a4 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW2S2ML_OFFSET   0x02a8 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW2S2LM_OFFSET   0x02ac /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW2S3UM_OFFSET   0x02b0 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW2S3MU_OFFSET   0x02b4 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW2S3ML_OFFSET   0x02b8 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW2S3LM_OFFSET   0x02bc /* Cache Data Storage (lowermost word) */

#define KINETIS_FMC_DATAW3S0UM_OFFSET   0x02c0 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW3S0MU_OFFSET   0x02c4 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW3S0ML_OFFSET   0x02c8 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW3S0LM_OFFSET   0x02cc /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW3S1UM_OFFSET   0x02d0 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW3S1MU_OFFSET   0x02d4 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW3S1ML_OFFSET   0x02d8 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW3S1LM_OFFSET   0x02dc /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW3S2UM_OFFSET   0x02e0 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW3S2MU_OFFSET   0x02e4 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW3S2ML_OFFSET   0x02e8 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW3S2LM_OFFSET   0x02ec /* Cache Data Storage (lowermost word) */
#define KINETIS_FMC_DATAW3S3UM_OFFSET   0x02f0 /* Cache Data Storage (uppermost word) */
#define KINETIS_FMC_DATAW3S3MU_OFFSET   0x02f4 /* Cache Data Storage (mid-upper word) */
#define KINETIS_FMC_DATAW3S3ML_OFFSET   0x02f8 /* Cache Data Storage (mid-lower word) */
#define KINETIS_FMC_DATAW3S3LM_OFFSET   0x02fc /* Cache Data Storage (lowermost word) */

/* Register Addresses ***************************************************************/

#define KINETIS_FMC_PFAPR               (KINETIS_FMC_BASE+KINETIS_FMC_PFAPR_OFFSET)
#define KINETIS_FMC_PFB01CR             (KINETIS_FMC_BASE+KINETIS_FMC_PFB01CR_OFFSET)
#define KINETIS_FMC_PFB23CR             (KINETIS_FMC_BASE+KINETIS_FMC_PFB23CR_OFFSET)

/* Cache Directory Storage for way=w and set=s, w=0..3, s=0..7 */

#define KINETIS_FMC_TAGVD(w,s)          (KINETIS_FMC_BASE+KINETIS_FMC_TAGVD_OFFSET(w,s))

#define KINETIS_FMC_TAGVDW0S0           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW0S0_OFFSET)
#define KINETIS_FMC_TAGVDW0S1           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW0S1_OFFSET)
#define KINETIS_FMC_TAGVDW0S2           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW0S2_OFFSET)
#define KINETIS_FMC_TAGVDW0S3           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW0S3_OFFSET)

#define KINETIS_FMC_TAGVDW1S0           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW1S0_OFFSET)
#define KINETIS_FMC_TAGVDW1S1           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW1S1_OFFSET)
#define KINETIS_FMC_TAGVDW1S2           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW1S2_OFFSET)
#define KINETIS_FMC_TAGVDW1S3           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW1S3_OFFSET)

#define KINETIS_FMC_TAGVDW2S0           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW2S0_OFFSET)
#define KINETIS_FMC_TAGVDW2S1           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW2S1_OFFSET)
#define KINETIS_FMC_TAGVDW2S2           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW2S2_OFFSET)
#define KINETIS_FMC_TAGVDW2S3           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW2S3_OFFSET)

#define KINETIS_FMC_TAGVDW3S0           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW3S0_OFFSET)
#define KINETIS_FMC_TAGVDW3S1           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW3S1_OFFSET)
#define KINETIS_FMC_TAGVDW3S2           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW3S2_OFFSET)
#define KINETIS_FMC_TAGVDW3S3           (KINETIS_FMC_BASE+KINETIS_FMC_TAGVDW3S3_OFFSET)

/* Cache Data Storage (UpperMost, MidUpper, MidLower LowerMost) for way=w and set=s, w=0..3, s=0..7 */

#define KINETIS_FMC_DATAUM(w,s)         (KINETIS_FMC_BASE+KINETIS_FMC_DATAUM_OFFSET(w,s))
#define KINETIS_FMC_DATAMU(w,s)         (KINETIS_FMC_BASE+KINETIS_FMC_DATAMU_OFFSET(w,s))
#define KINETIS_FMC_DATAML(w,s)         (KINETIS_FMC_BASE+KINETIS_FMC_DATAML_OFFSET(w,s))
#define KINETIS_FMC_DATALM(w,s)         (KINETIS_FMC_BASE+KINETIS_FMC_DATALM_OFFSET(w,s))

#define KINETIS_FMC_DATAW0S0UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S0UM_OFFSET)
#define KINETIS_FMC_DATAW0S0MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S0MU_OFFSET)
#define KINETIS_FMC_DATAW0S0ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S0ML_OFFSET)
#define KINETIS_FMC_DATAW0S0LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S0LM_OFFSET)
#define KINETIS_FMC_DATAW0S1UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S1UM_OFFSET)
#define KINETIS_FMC_DATAW0S1MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S1MU_OFFSET)
#define KINETIS_FMC_DATAW0S1ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S1ML_OFFSET)
#define KINETIS_FMC_DATAW0S1LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S1LM_OFFSET)
#define KINETIS_FMC_DATAW0S2UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S2UM_OFFSET)
#define KINETIS_FMC_DATAW0S2MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S2MU_OFFSET)
#define KINETIS_FMC_DATAW0S2ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S2ML_OFFSET)
#define KINETIS_FMC_DATAW0S2LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S2LM_OFFSET)
#define KINETIS_FMC_DATAW0S3UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S3UM_OFFSET)
#define KINETIS_FMC_DATAW0S3MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S3MU_OFFSET)
#define KINETIS_FMC_DATAW0S3ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S3ML_OFFSET)
#define KINETIS_FMC_DATAW0S3LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW0S3LM_OFFSET)

#define KINETIS_FMC_DATAW1S0UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S0UM_OFFSET)
#define KINETIS_FMC_DATAW1S0MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S0MU_OFFSET)
#define KINETIS_FMC_DATAW1S0ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S0ML_OFFSET)
#define KINETIS_FMC_DATAW1S0LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S0LM_OFFSET)
#define KINETIS_FMC_DATAW1S1UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S1UM_OFFSET)
#define KINETIS_FMC_DATAW1S1MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S1MU_OFFSET)
#define KINETIS_FMC_DATAW1S1ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S1ML_OFFSET)
#define KINETIS_FMC_DATAW1S1LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S1LM_OFFSET)
#define KINETIS_FMC_DATAW1S2UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S2UM_OFFSET)
#define KINETIS_FMC_DATAW1S2MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S2MU_OFFSET)
#define KINETIS_FMC_DATAW1S2ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S2ML_OFFSET)
#define KINETIS_FMC_DATAW1S2LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S2LM_OFFSET)
#define KINETIS_FMC_DATAW1S3UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S3UM_OFFSET)
#define KINETIS_FMC_DATAW1S3MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S3MU_OFFSET)
#define KINETIS_FMC_DATAW1S3ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S3ML_OFFSET)
#define KINETIS_FMC_DATAW1S3LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW1S3LM_OFFSET)

#define KINETIS_FMC_DATAW2S0UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S0UM_OFFSET)
#define KINETIS_FMC_DATAW2S0MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S0MU_OFFSET)
#define KINETIS_FMC_DATAW2S0ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S0ML_OFFSET)
#define KINETIS_FMC_DATAW2S0LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S0LM_OFFSET)
#define KINETIS_FMC_DATAW2S1UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S1UM_OFFSET)
#define KINETIS_FMC_DATAW2S1MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S1MU_OFFSET)
#define KINETIS_FMC_DATAW2S1ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S1ML_OFFSET)
#define KINETIS_FMC_DATAW2S1LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S1LM_OFFSET)
#define KINETIS_FMC_DATAW2S2UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S2UM_OFFSET)
#define KINETIS_FMC_DATAW2S2MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S2MU_OFFSET)
#define KINETIS_FMC_DATAW2S2ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S2ML_OFFSET)
#define KINETIS_FMC_DATAW2S2LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S2LM_OFFSET)
#define KINETIS_FMC_DATAW2S3UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S3UM_OFFSET)
#define KINETIS_FMC_DATAW2S3MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S3MU_OFFSET)
#define KINETIS_FMC_DATAW2S3ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S3ML_OFFSET)
#define KINETIS_FMC_DATAW2S3LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW2S3LM_OFFSET)

#define KINETIS_FMC_DATAW3S0UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S0UM_OFFSET)
#define KINETIS_FMC_DATAW3S0MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S0MU_OFFSET)
#define KINETIS_FMC_DATAW3S0ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S0ML_OFFSET)
#define KINETIS_FMC_DATAW3S0LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S0LM_OFFSET)
#define KINETIS_FMC_DATAW3S1UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S1UM_OFFSET)
#define KINETIS_FMC_DATAW3S1MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S1MU_OFFSET)
#define KINETIS_FMC_DATAW3S1ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S1ML_OFFSET)
#define KINETIS_FMC_DATAW3S1LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S1LM_OFFSET)
#define KINETIS_FMC_DATAW3S2UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S2UM_OFFSET)
#define KINETIS_FMC_DATAW3S2MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S2MU_OFFSET)
#define KINETIS_FMC_DATAW3S2ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S2ML_OFFSET)
#define KINETIS_FMC_DATAW3S2LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S2LM_OFFSET)
#define KINETIS_FMC_DATAW3S3UM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S3UM_OFFSET)
#define KINETIS_FMC_DATAW3S3MU          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S3MU_OFFSET)
#define KINETIS_FMC_DATAW3S3ML          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S3ML_OFFSET)
#define KINETIS_FMC_DATAW3S3LM          (KINETIS_FMC_BASE+KINETIS_FMC_DATAW3S3LM_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Flash Access Protection Register */
/* Access protection bits (all masters) */

#define FMC_PFAPR_NONE                  0 /* No access may be performed by this master */
#define FMC_PFAPR_RDONLY                1 /* Only read accesses may be performed by this master */
#define FMC_PFAPR_WRONLY                2 /* Only write accesses may be performed by this master */
#define FMC_PFAPR_RDWR                  3 /* Both read and write accesses may be performed by this master */

#define FMC_PFAPR_M0AP_SHIFT            (0)       /* Bits 0-1: Master 0 Access Protection */
#define FMC_PFAPR_M0AP_MASK             (3 << FMC_PFAPR_M0AP_SHIFT)
#define FMC_PFAPR_M1AP_SHIFT            (2)       /* Bits 2-3: Master 1 Access Protection */
#define FMC_PFAPR_M1AP_MASK             (3 << FMC_PFAPR_M1AP_SHIFT)
#define FMC_PFAPR_M2AP_SHIFT            (4)       /* Bits 4-5: Master 2 Access Protection */
#define FMC_PFAPR_M2AP_MASK             (3 << FMC_PFAPR_M2AP_SHIFT)
#define FMC_PFAPR_M3AP_SHIFT            (6)       /* Bits 6-7: Master 3 Access Protection */
#define FMC_PFAPR_M3AP_MASK             (3 << FMC_PFAPR_M3AP_SHIFT)
#define FMC_PFAPR_M4AP_SHIFT            (8)       /* Bits 8-9: Master 4 Access Protection */
#define FMC_PFAPR_M4AP_MASK             (3 << FMC_PFAPR_M4AP_SHIFT)
#define FMC_PFAPR_M5AP_SHIFT            (10)      /* Bits 10-11: Master 5 Access Protection */
#define FMC_PFAPR_M5AP_MASK             (3 << FMC_PFAPR_M5AP_SHIFT)
#define FMC_PFAPR_M6AP_SHIFT            (12)      /* Bits 12-13: Master 6 Access Protection */
#define FMC_PFAPR_M6AP_MASK             (3 << FMC_PFAPR_M6AP_SHIFT)
#define FMC_PFAPR_M7AP_SHIFT            (14)      /* Bits 14-15: Master 7 Access Protection */
#define FMC_PFAPR_M7AP_MASK             (3 << FMC_PFAPR_M7AP_SHIFT)
#define FMC_PFAPR_M0PFD                 (1 << 16) /* Bit 16: Master 0 Prefetch Disable */
#define FMC_PFAPR_M1PFD                 (1 << 17) /* Bit 17: Master 1 Prefetch Disable */
#define FMC_PFAPR_M2PFD                 (1 << 18) /* Bit 18: Master 2 Prefetch Disable */
#define FMC_PFAPR_M3PFD                 (1 << 19) /* Bit 19: Master 3 Prefetch Disable */
#define FMC_PFAPR_M4PFD                 (1 << 20) /* Bit 20: Master 4 Prefetch Disable */
#define FMC_PFAPR_M5PFD                 (1 << 21) /* Bit 21: Master 5 Prefetch Disable */
#define FMC_PFAPR_M6PFD                 (1 << 22) /* Bit 22: Master 6 Prefetch Disable */
#define FMC_PFAPR_M7PFD                 (1 << 23) /* Bit 23: Master 7 Prefetch Disable */
                                                  /* Bits 24-31: Reserved */
/* Flash Bank 0-1 Control Register */

#define FMC_PFB01CR_RFU                 (1 << 0)  /* Bit 0:  Reserved for future use */
#define FMC_PFB01CR_B0IPE               (1 << 1)  /* Bit 1:  Bank 0 Instruction Prefetch Enable */
#define FMC_PFB01CR_B0DPE               (1 << 2)  /* Bit 2:  Bank 0 Data Prefetch Enable */
#define FMC_PFB01CR_B0ICE               (1 << 3)  /* Bit 3:  Bank 0 Instruction Cache Enable */
#define FMC_PFB01CR_B0DCE               (1 << 4)  /* Bit 4:  Bank 0 Data Cache Enable */
#define FMC_PFB01CR_CRC_SHIFT           (5)       /* Bits 5-7: Cache Replacement Control */
#define FMC_PFB01CR_CRC_MASK            (7 << FMC_PFB01CR_CRC_SHIFT)
#  define FMC_PFB01CR_CRC_ALL           (0 << FMC_PFB01CR_CRC_SHIFT) /* LRU all four ways */
#  define FMC_PFB01CR_CRC_I01D23        (2 << FMC_PFB01CR_CRC_SHIFT) /* LRU ifetches 0-1 data 2-3 */
#  define FMC_PFB01CR_CRC_I012D3        (3 << FMC_PFB01CR_CRC_SHIFT) /* LRU ifetches 0-3 data 3 */
                                                  /* Bits 8-16: Reserved */
#define FMC_PFB01CR_B0MW_SHIFT          (17)      /* Bits 17-18: Bank 0 Memory Width */
#define FMC_PFB01CR_B0MW_MASK           (3 << FMC_PFB01CR_B0MW_SHIFT)
#  define FMC_PFB01CR_B0MW_32BITS       (0 << FMC_PFB01CR_B0MW_SHIFT) /* 32 bits */
#  define FMC_PFB01CR_B0MW_64BITS       (1 << FMC_PFB01CR_B0MW_SHIFT) /* 64 bits */
#  define FMC_PFB01CR_B0MW_128BITS      (2 << FMC_PFB01CR_B0MW_SHIFT) /* 128 bits */
#define FMC_PFB01CR_S_B_INV             (1 << 19) /* Bit 19:  Invalidate Prefetch Speculation Buffer */
#define FMC_PFB01CR_CINV_WAY_SHIFT      (20)      /* Bits 20-23: Cache Invalidate Way x */
#define FMC_PFB01CR_CINV_WAY_MASK       (15 << FMC_PFB01CR_CINV_WAY_SHIFT)
#define FMC_PFB01CR_CLCK_WAY_SHIFT      (24)      /* Bits 24-27: Cache Lock Way x */
#define FMC_PFB01CR_CLCK_WAY_MASK       (15 << FMC_PFB01CR_CLCK_WAY_SHIFT)
#define FMC_PFB01CR_B0RWSC_SHIFT        (28)      /* Bits 28-31: Bank 0 Read Wait State Control */
#define FMC_PFB01CR_B0RWSC_MASK         (15 << FMC_PFB01CR_B0RWSC_SHIFT)

/* Flash Bank 2-3 Control Register */

#define FMC_PFB23CR_RFU                 (1 << 0)  /* Bit 0:  served for future use */
#define FMC_PFB23CR_B1IPE               (1 << 1)  /* Bit 1:  Bank 1 Instruction Prefetch Enable */
#define FMC_PFB23CR_B1DPE               (1 << 2)  /* Bit 2:  Bank 1 Data Prefetch Enable */
#define FMC_PFB23CR_B1ICE               (1 << 3)  /* Bit 3:  Bank 1 Instruction Cache Enable */
#define FMC_PFB23CR_B1DCE               (1 << 4)  /* Bit 4:  Bank 1 Data Cache Enable */
                                                  /* Bits 5-16: Reserved */
#define FMC_PFB23CR_B1MW_SHIFT          (17)      /* Bits 17-18: Bank 1 Memory Width */
#define FMC_PFB23CR_B1MW_MASK           (3 << FMC_PFB23CR_B1MW_SHIFT)
#  define FMC_PFB23CR_B1MW_32BITS       (0 << FMC_PFB23CR_B1MW_SHIFT) /* 32 bits */
#  define FMC_PFB23CR_B1MW_64BITS       (1 << FMC_PFB23CR_B1MW_SHIFT) /* 64 bits */
#  define FMC_PFB23CR_B1MW_128BITS      (2 << FMC_PFB23CR_B1MW_SHIFT) /* 128 bits */
                                                  /* Bits 19-27: Reserved */
#define FMC_PFB23CR_B1RWSC_SHIFT        (28)      /* Bits 28-31: Bank 1 Read Wait State Control */
#define FMC_PFB23CR_B1RWSC_MASK         (15 << FMC_PFB23CR_B0RWSC_SHIFT)

/* Cache Directory Storage for way=w and set=s, w=0..3, s=0..7 */

#define FMC_TAGVD_VALID                 (1 << 0)  /* Bit 0:  1-bit valid for cache entry */
                                                  /* Bits 1-5: Reserved */
#define FMC_TAGVD_TAG_SHIFT             (6)       /* Bits 6-21: 16-bit tag for cache entry */
#define FMC_TAGVD_TAG_MASK              (0xffff << FMC_TAGVD_TAG_SHIFT)
                                                  /* Bits 19-31: Reserved */

/* Cache Data Storage (UpperMost, MidUpper, MidLower LowerMost) for way=w and set=s,
 * w=0..3, s=0..7  128-bit data in four 32-bit registers.
 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_K28K66FMC_H */
