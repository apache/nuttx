/****************************************************************************
 * arch/arm/src/common/hwcap.h
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

#ifndef __ARCH_ARM_SRC_COMMON_HWCAP_H
#define __ARCH_ARM_SRC_COMMON_HWCAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Hardware capabilities */

#define HWCAP_SWP       "swp"       /* SWP instruction */
#define HWCAP_HALF      "half"      /* Half word */
#define HWCAP_THUMB     "thumb"     /* Thumb instruction set support */
#define HWCAP_26BIT     "26bit"     /* Play it safe */
#define HWCAP_FAST_MULT "fastmult"  /* Long multiply such as umull and umlal */
#define HWCAP_FPA       "fpa"
#define HWCAP_VFP       "vfp"
#define HWCAP_EDSP      "edsp"      /* Enhanced dsp instruction */
#define HWCAP_JAVA      "java"
#define HWCAP_IWMMXT    "iwmmxt"
#define HWCAP_CRUNCH    "crunch"
#define HWCAP_THUMBEE   "thumbee"
#define HWCAP_NEON      "neon"      /* Advanced SIMD extension */
#define HWCAP_VFPV3     "vfpv3"     /* Floating-point extension version */
#define HWCAP_VFPV3D16  "vfpv3d16"  /* Also set for vfpv4-d16 */
#define HWCAP_TLS       "tls"       /* Thread local storage */
#define HWCAP_VFPV4     "vfpv4"     /* Floating-point extension version */
#define HWCAP_IDIVA     "idiva"     /* SDIV and UDIV instruction */
#define HWCAP_IDIVT     "idivt"     /* SDIV and UDIV in thumb instruction set */
#define HWCAP_VFPD32    "vfpd32"    /* Set if VFP has 32 64-bit regs (not 16) */
#define HWCAP_LPAE      "lpae"      /* LDRD/STRD instructions */
#define HWCAP_EVTSTRM   "evtstrm"
#define HWCAP_FPHP      "fphp"      /* Half-precision instructions support */
#define HWCAP_FPSP      "fpsp"      /* Single-precision instructions support */
#define HWCAP_FPDP      "fpdp"      /* Double-precision instructions support */
#define HWCAP_FPV5      "fpv5"      /* Floating-point extension version */
#define HWCAP_MVE       "mve"       /* M-profile vector extension */

/* Hardware capabilities added form armv8 */

#define HWCAP2_AES      "aes"
#define HWCAP2_PMULL    "pmull"
#define HWCAP2_SHA1     "sha1"
#define HWCAP2_SHA2     "sha2"
#define HWCAP2_CRC32    "crc32"

#endif /* __ARCH_ARM_SRC_COMMON_HWCAP_H */
