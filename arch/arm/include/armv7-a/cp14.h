/****************************************************************************
 * arch/arm/include/armv7-a/cp14.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_INCLUDE_ARMV7_A_CP14_H
#define __ARCH_ARM_INCLUDE_ARMV7_A_CP14_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/bits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__
#  define _CP14(op1, rd, crn, crm, op2) p14, op1, rd, crn, crm, op2
#else
#  define _CP14(op1, rd, crn, crm, op2) "p14, " #op1 ", %0, " #crn ", " #crm ", " #op2
#endif

/* Debug Registers
 *
 * Available only in DBGv7
 * DBGECR, DBGDSCCR, DBGDSMCR, DBGDRCR
 *
 * Available only in DBGv7.1
 * DBGBXVRm, DBGOSDLR, DBGDEVID2, DBGDEVID1
 *
 * Read only
 * DBGDIDR, DBGDSCR, DBGDTRRXint, DBGDRAR, DBGOSLSR, DBGOSSRR, DBGPRSR,
 * DBGPRSR, DBGDSAR, DBGAUTHSTATUS, DBGDEVID2, DBGDEVID1, DBGDEVID
 *
 * Write only
 * DBGDTRTXint, DBGOSLAR
 */

#define CP14_DBGDIDR(r)                      _CP14(0, r, c0, c0, 0)
#define CP14_DBGDSCRINT(r)                   _CP14(0, r, c0, c1, 0)
#define CP14_DBGDTRRXINT(r)                  _CP14(0, r, c0, c5, 0)
#define CP14_DBGDTRTXINT(r)                  _CP14(0, r, c0, c5, 0)
#define CP14_DBGWFAR(r)                      _CP14(0, r, c0, c6, 0)
#define CP14_DBGVCR(r)                       _CP14(0, r, c0, c7, 0)
#define CP14_DBGECR(r)                       _CP14(0, r, c0, c9, 0)
#define CP14_DBGDSCCR(r)                     _CP14(0, r, c0, c10, 0)
#define CP14_DBGDSMCR(r)                     _CP14(0, r, c0, c11, 0)
#define CP14_DBGDTRRXEXT(r)                  _CP14(0, r, c0, c0, 2)
#define CP14_DBGDSCREXT(r)                   _CP14(0, r, c0, c2, 2)
#define CP14_DBGDTRTXEXT(r)                  _CP14(0, r, c0, c3, 2)
#define CP14_DBGDRCR(r)                      _CP14(0, r, c0, c4, 2)
#define CP14_DBGBVR0(r)                      _CP14(0, r, c0, c0, 4)
#define CP14_DBGBVR1(r)                      _CP14(0, r, c0, c1, 4)
#define CP14_DBGBVR2(r)                      _CP14(0, r, c0, c2, 4)
#define CP14_DBGBVR3(r)                      _CP14(0, r, c0, c3, 4)
#define CP14_DBGBVR4(r)                      _CP14(0, r, c0, c4, 4)
#define CP14_DBGBVR5(r)                      _CP14(0, r, c0, c5, 4)
#define CP14_DBGBVR6(r)                      _CP14(0, r, c0, c6, 4)
#define CP14_DBGBVR7(r)                      _CP14(0, r, c0, c7, 4)
#define CP14_DBGBVR8(r)                      _CP14(0, r, c0, c8, 4)
#define CP14_DBGBVR9(r)                      _CP14(0, r, c0, c9, 4)
#define CP14_DBGBVR10(r)                     _CP14(0, r, c0, c10, 4)
#define CP14_DBGBVR11(r)                     _CP14(0, r, c0, c11, 4)
#define CP14_DBGBVR12(r)                     _CP14(0, r, c0, c12, 4)
#define CP14_DBGBVR13(r)                     _CP14(0, r, c0, c13, 4)
#define CP14_DBGBVR14(r)                     _CP14(0, r, c0, c14, 4)
#define CP14_DBGBVR15(r)                     _CP14(0, r, c0, c15, 4)
#define CP14_DBGBCR0(r)                      _CP14(0, r, c0, c0, 5)
#define CP14_DBGBCR1(r)                      _CP14(0, r, c0, c1, 5)
#define CP14_DBGBCR2(r)                      _CP14(0, r, c0, c2, 5)
#define CP14_DBGBCR3(r)                      _CP14(0, r, c0, c3, 5)
#define CP14_DBGBCR4(r)                      _CP14(0, r, c0, c4, 5)
#define CP14_DBGBCR5(r)                      _CP14(0, r, c0, c5, 5)
#define CP14_DBGBCR6(r)                      _CP14(0, r, c0, c6, 5)
#define CP14_DBGBCR7(r)                      _CP14(0, r, c0, c7, 5)
#define CP14_DBGBCR8(r)                      _CP14(0, r, c0, c8, 5)
#define CP14_DBGBCR9(r)                      _CP14(0, r, c0, c9, 5)
#define CP14_DBGBCR10(r)                     _CP14(0, r, c0, c10, 5)
#define CP14_DBGBCR11(r)                     _CP14(0, r, c0, c11, 5)
#define CP14_DBGBCR12(r)                     _CP14(0, r, c0, c12, 5)
#define CP14_DBGBCR13(r)                     _CP14(0, r, c0, c13, 5)
#define CP14_DBGBCR14(r)                     _CP14(0, r, c0, c14, 5)
#define CP14_DBGBCR15(r)                     _CP14(0, r, c0, c15, 5)
#define CP14_DBGWVR0(r)                      _CP14(0, r, c0, c0, 6)
#define CP14_DBGWVR1(r)                      _CP14(0, r, c0, c1, 6)
#define CP14_DBGWVR2(r)                      _CP14(0, r, c0, c2, 6)
#define CP14_DBGWVR3(r)                      _CP14(0, r, c0, c3, 6)
#define CP14_DBGWVR4(r)                      _CP14(0, r, c0, c4, 6)
#define CP14_DBGWVR5(r)                      _CP14(0, r, c0, c5, 6)
#define CP14_DBGWVR6(r)                      _CP14(0, r, c0, c6, 6)
#define CP14_DBGWVR7(r)                      _CP14(0, r, c0, c7, 6)
#define CP14_DBGWVR8(r)                      _CP14(0, r, c0, c8, 6)
#define CP14_DBGWVR9(r)                      _CP14(0, r, c0, c9, 6)
#define CP14_DBGWVR10(r)                     _CP14(0, r, c0, c10, 6)
#define CP14_DBGWVR11(r)                     _CP14(0, r, c0, c11, 6)
#define CP14_DBGWVR12(r)                     _CP14(0, r, c0, c12, 6)
#define CP14_DBGWVR13(r)                     _CP14(0, r, c0, c13, 6)
#define CP14_DBGWVR14(r)                     _CP14(0, r, c0, c14, 6)
#define CP14_DBGWVR15(r)                     _CP14(0, r, c0, c15, 6)
#define CP14_DBGWCR0(r)                      _CP14(0, r, c0, c0, 7)
#define CP14_DBGWCR1(r)                      _CP14(0, r, c0, c1, 7)
#define CP14_DBGWCR2(r)                      _CP14(0, r, c0, c2, 7)
#define CP14_DBGWCR3(r)                      _CP14(0, r, c0, c3, 7)
#define CP14_DBGWCR4(r)                      _CP14(0, r, c0, c4, 7)
#define CP14_DBGWCR5(r)                      _CP14(0, r, c0, c5, 7)
#define CP14_DBGWCR6(r)                      _CP14(0, r, c0, c6, 7)
#define CP14_DBGWCR7(r)                      _CP14(0, r, c0, c7, 7)
#define CP14_DBGWCR8(r)                      _CP14(0, r, c0, c8, 7)
#define CP14_DBGWCR9(r)                      _CP14(0, r, c0, c9, 7)
#define CP14_DBGWCR10(r)                     _CP14(0, r, c0, c10, 7)
#define CP14_DBGWCR11(r)                     _CP14(0, r, c0, c11, 7)
#define CP14_DBGWCR12(r)                     _CP14(0, r, c0, c12, 7)
#define CP14_DBGWCR13(r)                     _CP14(0, r, c0, c13, 7)
#define CP14_DBGWCR14(r)                     _CP14(0, r, c0, c14, 7)
#define CP14_DBGWCR15(r)                     _CP14(0, r, c0, c15, 7)
#define CP14_DBGDRAR(r)                      _CP14(0, r, c1, c0, 0)
#define CP14_DBGBXVR0(r)                     _CP14(0, r, c1, c0, 1)
#define CP14_DBGBXVR1(r)                     _CP14(0, r, c1, c1, 1)
#define CP14_DBGBXVR2(r)                     _CP14(0, r, c1, c2, 1)
#define CP14_DBGBXVR3(r)                     _CP14(0, r, c1, c3, 1)
#define CP14_DBGBXVR4(r)                     _CP14(0, r, c1, c4, 1)
#define CP14_DBGBXVR5(r)                     _CP14(0, r, c1, c5, 1)
#define CP14_DBGBXVR6(r)                     _CP14(0, r, c1, c6, 1)
#define CP14_DBGBXVR7(r)                     _CP14(0, r, c1, c7, 1)
#define CP14_DBGBXVR8(r)                     _CP14(0, r, c1, c8, 1)
#define CP14_DBGBXVR9(r)                     _CP14(0, r, c1, c9, 1)
#define CP14_DBGBXVR10(r)                    _CP14(0, r, c1, c10, 1)
#define CP14_DBGBXVR11(r)                    _CP14(0, r, c1, c11, 1)
#define CP14_DBGBXVR12(r)                    _CP14(0, r, c1, c12, 1)
#define CP14_DBGBXVR13(r)                    _CP14(0, r, c1, c13, 1)
#define CP14_DBGBXVR14(r)                    _CP14(0, r, c1, c14, 1)
#define CP14_DBGBXVR15(r)                    _CP14(0, r, c1, c15, 1)
#define CP14_DBGOSLSR(r)                     _CP14(0, r, c1, c1, 4)
#define CP14_DBGOSSRR(r)                     _CP14(0, r, c1, c2, 4)
#define CP14_DBGOSDLR(r)                     _CP14(0, r, c1, c3, 4)
#define CP14_DBGPRCR(r)                      _CP14(0, r, c1, c4, 4)
#define CP14_DBGPRSR(r)                      _CP14(0, r, c1, c5, 4)
#define CP14_DBGDSAR(r)                      _CP14(0, r, c2, c0, 0)
#define CP14_DBGITCTRL(r)                    _CP14(0, r, c7, c0, 4)
#define CP14_DBGCLAIMSET(r)                  _CP14(0, r, c7, c8, 6)
#define CP14_DBGCLAIMCLR(r)                  _CP14(0, r, c7, c9, 6)
#define CP14_DBGAUTHSTATUS(r)                _CP14(0, r, c7, c14, 6)
#define CP14_DBGDEVID2(r)                    _CP14(0, r, c7, c0, 7)
#define CP14_DBGDEVID1(r)                    _CP14(0, r, c7, c1, 7)
#define CP14_DBGDEVID(r)                     _CP14(0, r, c7, c2, 7)

/* ETM Registers
 *
 * Available only in ETMv3.3, 3.4, 3.5
 * ETMASICCR, ETMTECR2, ETMFFRR, ETMVDEVR, ETMVDCR1, ETMVDCR2, ETMVDCR3,
 * ETMDCVRn, ETMDCMRn
 *
 * Available only in ETMv3.5 as read only
 * ETMIDR2
 *
 * Available only in ETMv3.5, PFTv1.0, 1.1
 * ETMTSEVR, ETMVMIDCVR, ETMPDCR
 *
 * Read only
 * ETMCCR, ETMSCR, ETMIDR, ETMCCER, ETMOSLSR
 * ETMLSR, ETMAUTHSTATUS, ETMDEVID, ETMDEVTYPE, ETMPIDR4, ETMPIDR5, ETMPIDR6,
 * ETMPIDR7, ETMPIDR0, ETMPIDR1, ETMPIDR2, ETMPIDR2, ETMPIDR3, ETMCIDR0,
 * ETMCIDR1, ETMCIDR2, ETMCIDR3
 *
 * Write only
 * ETMOSLAR, ETMLAR
 * Note: ETMCCER[11] controls WO nature of certain regs. Refer ETM arch spec.
 */

#define CP14_ETMCR(r)                        _CP14(1, r, c0, c0, 0)
#define CP14_ETMCCR(r)                       _CP14(1, r, c0, c1, 0)
#define CP14_ETMTRIGGER(r)                   _CP14(1, r, c0, c2, 0)
#define CP14_ETMASICCR(r)                    _CP14(1, r, c0, c3, 0)
#define CP14_ETMSR(r)                        _CP14(1, r, c0, c4, 0)
#define CP14_ETMSCR(r)                       _CP14(1, r, c0, c5, 0)
#define CP14_ETMTSSCR(r)                     _CP14(1, r, c0, c6, 0)
#define CP14_ETMTECR2(r)                     _CP14(1, r, c0, c7, 0)
#define CP14_ETMTEEVR(r)                     _CP14(1, r, c0, c8, 0)
#define CP14_ETMTECR1(r)                     _CP14(1, r, c0, c9, 0)
#define CP14_ETMFFRR(r)                      _CP14(1, r, c0, c10, 0)
#define CP14_ETMFFLR(r)                      _CP14(1, r, c0, c11, 0)
#define CP14_ETMVDEVR(r)                     _CP14(1, r, c0, c12, 0)
#define CP14_ETMVDCR1(r)                     _CP14(1, r, c0, c13, 0)
#define CP14_ETMVDCR2(r)                     _CP14(1, r, c0, c14, 0)
#define CP14_ETMVDCR3(r)                     _CP14(1, r, c0, c15, 0)
#define CP14_ETMACVR0(r)                     _CP14(1, r, c0, c0, 1)
#define CP14_ETMACVR1(r)                     _CP14(1, r, c0, c1, 1)
#define CP14_ETMACVR2(r)                     _CP14(1, r, c0, c2, 1)
#define CP14_ETMACVR3(r)                     _CP14(1, r, c0, c3, 1)
#define CP14_ETMACVR4(r)                     _CP14(1, r, c0, c4, 1)
#define CP14_ETMACVR5(r)                     _CP14(1, r, c0, c5, 1)
#define CP14_ETMACVR6(r)                     _CP14(1, r, c0, c6, 1)
#define CP14_ETMACVR7(r)                     _CP14(1, r, c0, c7, 1)
#define CP14_ETMACVR8(r)                     _CP14(1, r, c0, c8, 1)
#define CP14_ETMACVR9(r)                     _CP14(1, r, c0, c9, 1)
#define CP14_ETMACVR10(r)                    _CP14(1, r, c0, c10, 1)
#define CP14_ETMACVR11(r)                    _CP14(1, r, c0, c11, 1)
#define CP14_ETMACVR12(r)                    _CP14(1, r, c0, c12, 1)
#define CP14_ETMACVR13(r)                    _CP14(1, r, c0, c13, 1)
#define CP14_ETMACVR14(r)                    _CP14(1, r, c0, c14, 1)
#define CP14_ETMACVR15(r)                    _CP14(1, r, c0, c15, 1)
#define CP14_ETMACTR0(r)                     _CP14(1, r, c0, c0, 2)
#define CP14_ETMACTR1(r)                     _CP14(1, r, c0, c1, 2)
#define CP14_ETMACTR2(r)                     _CP14(1, r, c0, c2, 2)
#define CP14_ETMACTR3(r)                     _CP14(1, r, c0, c3, 2)
#define CP14_ETMACTR4(r)                     _CP14(1, r, c0, c4, 2)
#define CP14_ETMACTR5(r)                     _CP14(1, r, c0, c5, 2)
#define CP14_ETMACTR6(r)                     _CP14(1, r, c0, c6, 2)
#define CP14_ETMACTR7(r)                     _CP14(1, r, c0, c7, 2)
#define CP14_ETMACTR8(r)                     _CP14(1, r, c0, c8, 2)
#define CP14_ETMACTR9(r)                     _CP14(1, r, c0, c9, 2)
#define CP14_ETMACTR10(r)                    _CP14(1, r, c0, c10, 2)
#define CP14_ETMACTR11(r)                    _CP14(1, r, c0, c11, 2)
#define CP14_ETMACTR12(r)                    _CP14(1, r, c0, c12, 2)
#define CP14_ETMACTR13(r)                    _CP14(1, r, c0, c13, 2)
#define CP14_ETMACTR14(r)                    _CP14(1, r, c0, c14, 2)
#define CP14_ETMACTR15(r)                    _CP14(1, r, c0, c15, 2)
#define CP14_ETMDCVR0(r)                     _CP14(1, r, c0, c0, 3)
#define CP14_ETMDCVR2(r)                     _CP14(1, r, c0, c2, 3)
#define CP14_ETMDCVR4(r)                     _CP14(1, r, c0, c4, 3)
#define CP14_ETMDCVR6(r)                     _CP14(1, r, c0, c6, 3)
#define CP14_ETMDCVR8(r)                     _CP14(1, r, c0, c8, 3)
#define CP14_ETMDCVR10(r)                    _CP14(1, r, c0, c10, 3)
#define CP14_ETMDCVR12(r)                    _CP14(1, r, c0, c12, 3)
#define CP14_ETMDCVR14(r)                    _CP14(1, r, c0, c14, 3)
#define CP14_ETMDCMR0(r)                     _CP14(1, r, c0, c0, 4)
#define CP14_ETMDCMR2(r)                     _CP14(1, r, c0, c2, 4)
#define CP14_ETMDCMR4(r)                     _CP14(1, r, c0, c4, 4)
#define CP14_ETMDCMR6(r)                     _CP14(1, r, c0, c6, 4)
#define CP14_ETMDCMR8(r)                     _CP14(1, r, c0, c8, 4)
#define CP14_ETMDCMR10(r)                    _CP14(1, r, c0, c10, 4)
#define CP14_ETMDCMR12(r)                    _CP14(1, r, c0, c12, 4)
#define CP14_ETMDCMR14(r)                    _CP14(1, r, c0, c14, 4)
#define CP14_ETMCNTRLDVR0(r)                 _CP14(1, r, c0, c0, 5)
#define CP14_ETMCNTRLDVR1(r)                 _CP14(1, r, c0, c1, 5)
#define CP14_ETMCNTRLDVR2(r)                 _CP14(1, r, c0, c2, 5)
#define CP14_ETMCNTRLDVR3(r)                 _CP14(1, r, c0, c3, 5)
#define CP14_ETMCNTENR0(r)                   _CP14(1, r, c0, c4, 5)
#define CP14_ETMCNTENR1(r)                   _CP14(1, r, c0, c5, 5)
#define CP14_ETMCNTENR2(r)                   _CP14(1, r, c0, c6, 5)
#define CP14_ETMCNTENR3(r)                   _CP14(1, r, c0, c7, 5)
#define CP14_ETMCNTRLDEVR0(r)                _CP14(1, r, c0, c8, 5)
#define CP14_ETMCNTRLDEVR1(r)                _CP14(1, r, c0, c9, 5)
#define CP14_ETMCNTRLDEVR2(r)                _CP14(1, r, c0, c10, 5)
#define CP14_ETMCNTRLDEVR3(r)                _CP14(1, r, c0, c11, 5)
#define CP14_ETMCNTVR0(r)                    _CP14(1, r, c0, c12, 5)
#define CP14_ETMCNTVR1(r)                    _CP14(1, r, c0, c13, 5)
#define CP14_ETMCNTVR2(r)                    _CP14(1, r, c0, c14, 5)
#define CP14_ETMCNTVR3(r)                    _CP14(1, r, c0, c15, 5)
#define CP14_ETMSQ12EVR(r)                   _CP14(1, r, c0, c0, 6)
#define CP14_ETMSQ21EVR(r)                   _CP14(1, r, c0, c1, 6)
#define CP14_ETMSQ23EVR(r)                   _CP14(1, r, c0, c2, 6)
#define CP14_ETMSQ31EVR(r)                   _CP14(1, r, c0, c3, 6)
#define CP14_ETMSQ32EVR(r)                   _CP14(1, r, c0, c4, 6)
#define CP14_ETMSQ13EVR(r)                   _CP14(1, r, c0, c5, 6)
#define CP14_ETMSQR(r)                       _CP14(1, r, c0, c7, 6)
#define CP14_ETMEXTOUTEVR0(r)                _CP14(1, r, c0, c8, 6)
#define CP14_ETMEXTOUTEVR1(r)                _CP14(1, r, c0, c9, 6)
#define CP14_ETMEXTOUTEVR2(r)                _CP14(1, r, c0, c10, 6)
#define CP14_ETMEXTOUTEVR3(r)                _CP14(1, r, c0, c11, 6)
#define CP14_ETMCIDCVR0(r)                   _CP14(1, r, c0, c12, 6)
#define CP14_ETMCIDCVR1(r)                   _CP14(1, r, c0, c13, 6)
#define CP14_ETMCIDCVR2(r)                   _CP14(1, r, c0, c14, 6)
#define CP14_ETMCIDCMR(r)                    _CP14(1, r, c0, c15, 6)
#define CP14_ETMIMPSPEC0(r)                  _CP14(1, r, c0, c0, 7)
#define CP14_ETMIMPSPEC1(r)                  _CP14(1, r, c0, c1, 7)
#define CP14_ETMIMPSPEC2(r)                  _CP14(1, r, c0, c2, 7)
#define CP14_ETMIMPSPEC3(r)                  _CP14(1, r, c0, c3, 7)
#define CP14_ETMIMPSPEC4(r)                  _CP14(1, r, c0, c4, 7)
#define CP14_ETMIMPSPEC5(r)                  _CP14(1, r, c0, c5, 7)
#define CP14_ETMIMPSPEC6(r)                  _CP14(1, r, c0, c6, 7)
#define CP14_ETMIMPSPEC7(r)                  _CP14(1, r, c0, c7, 7)
#define CP14_ETMSYNCFR(r)                    _CP14(1, r, c0, c8, 7)
#define CP14_ETMIDR(r)                       _CP14(1, r, c0, c9, 7)
#define CP14_ETMCCER(r)                      _CP14(1, r, c0, c10, 7)
#define CP14_ETMEXTINSELR(r)                 _CP14(1, r, c0, c11, 7)
#define CP14_ETMTESSEICR(r)                  _CP14(1, r, c0, c12, 7)
#define CP14_ETMEIBCR(r)                     _CP14(1, r, c0, c13, 7)
#define CP14_ETMTSEVR(r)                     _CP14(1, r, c0, c14, 7)
#define CP14_ETMAUXCR(r)                     _CP14(1, r, c0, c15, 7)
#define CP14_ETMTRACEIDR(r)                  _CP14(1, r, c1, c0, 0)
#define CP14_ETMIDR2(r)                      _CP14(1, r, c1, c2, 0)
#define CP14_ETMVMIDCVR(r)                   _CP14(1, r, c1, c0, 1)
#define CP14_ETMOSLSR(r)                     _CP14(1, r, c1, c1, 4)

/* Not available in PFTv1.1 */

#define CP14_ETMOSSRR(r)                     _CP14(1, r, c1, c2, 4)
#define CP14_ETMPDCR(r)                      _CP14(1, r, c1, c4, 4)
#define CP14_ETMPDSR(r)                      _CP14(1, r, c1, c5, 4)
#define CP14_ETMITCTRL(r)                    _CP14(1, r, c7, c0, 4)
#define CP14_ETMCLAIMSET(r)                  _CP14(1, r, c7, c8, 6)
#define CP14_ETMCLAIMCLR(r)                  _CP14(1, r, c7, c9, 6)
#define CP14_ETMLSR(r)                       _CP14(1, r, c7, c13, 6)
#define CP14_ETMAUTHSTATUS(r)                _CP14(1, r, c7, c14, 6)
#define CP14_ETMDEVID(r)                     _CP14(1, r, c7, c2, 7)
#define CP14_ETMDEVTYPE(r)                   _CP14(1, r, c7, c3, 7)
#define CP14_ETMPIDR4(r)                     _CP14(1, r, c7, c4, 7)
#define CP14_ETMPIDR5(r)                     _CP14(1, r, c7, c5, 7)
#define CP14_ETMPIDR6(r)                     _CP14(1, r, c7, c6, 7)
#define CP14_ETMPIDR7(r)                     _CP14(1, r, c7, c7, 7)
#define CP14_ETMPIDR0(r)                     _CP14(1, r, c7, c8, 7)
#define CP14_ETMPIDR1(r)                     _CP14(1, r, c7, c9, 7)
#define CP14_ETMPIDR2(r)                     _CP14(1, r, c7, c10, 7)
#define CP14_ETMPIDR3(r)                     _CP14(1, r, c7, c11, 7)
#define CP14_ETMCIDR0(r)                     _CP14(1, r, c7, c12, 7)
#define CP14_ETMCIDR1(r)                     _CP14(1, r, c7, c13, 7)
#define CP14_ETMCIDR2(r)                     _CP14(1, r, c7, c14, 7)
#define CP14_ETMCIDR3(r)                     _CP14(1, r, c7, c15, 7)

#define CP14_GET(reg)                        \
  ({                                         \
     uint32_t _val;                          \
     __asm__ __volatile__                    \
     (                                       \
       "mrc " CP14_ ## reg(0) "\n"           \
       : "=r"(_val) :: "memory"              \
     );                                      \
     _val;                                   \
  })                                         \

#define CP14_SET(reg, val)                   \
  do                                         \
    {                                        \
      __asm__ __volatile__                   \
      (                                      \
        "mcr " CP14_ ## reg(0) "\n"          \
        :: "r"(val): "memory"                \
      );                                     \
    }                                        \
  while(0)                                   \

#define CP14_MOD(reg, val, mask)             \
  CP14_SET(reg, ((CP14_GET(reg) & ~(mask)) | ((uintptr_t)(val) & (mask))))

#define CP14_SET_CASE(reg, n, val)           \
  case n:                                    \
    CP14_SET(reg ## n, val);                 \
    break;

#define CP14_GET_CASE(reg, n, val)           \
  case n:                                    \
    val = CP14_GET(reg ## n);                \
    break;

#define CP14_SETN(reg, n, val)               \
  switch (n)                                 \
    {                                        \
      CP14_SET_CASE(reg, 0, val)             \
      CP14_SET_CASE(reg, 1, val)             \
      CP14_SET_CASE(reg, 2, val)             \
      CP14_SET_CASE(reg, 3, val)             \
      CP14_SET_CASE(reg, 4, val)             \
      CP14_SET_CASE(reg, 5, val)             \
      CP14_SET_CASE(reg, 6, val)             \
      CP14_SET_CASE(reg, 7, val)             \
      CP14_SET_CASE(reg, 8, val)             \
      CP14_SET_CASE(reg, 9, val)             \
      CP14_SET_CASE(reg, 10, val)            \
      CP14_SET_CASE(reg, 11, val)            \
      CP14_SET_CASE(reg, 12, val)            \
      CP14_SET_CASE(reg, 13, val)            \
      CP14_SET_CASE(reg, 14, val)            \
      CP14_SET_CASE(reg, 15, val)            \
    }

#define CP14_GETN(reg, n)                    \
  ({                                         \
    uint32_t _val = 0;                       \
    switch (n)                               \
      {                                      \
        CP14_GET_CASE(reg, 0, _val)          \
        CP14_GET_CASE(reg, 1, _val)          \
        CP14_GET_CASE(reg, 2, _val)          \
        CP14_GET_CASE(reg, 3, _val)          \
        CP14_GET_CASE(reg, 4, _val)          \
        CP14_GET_CASE(reg, 5, _val)          \
        CP14_GET_CASE(reg, 6, _val)          \
        CP14_GET_CASE(reg, 7, _val)          \
        CP14_GET_CASE(reg, 8, _val)          \
        CP14_GET_CASE(reg, 9, _val)          \
        CP14_GET_CASE(reg, 10, _val)         \
        CP14_GET_CASE(reg, 11, _val)         \
        CP14_GET_CASE(reg, 12, _val)         \
        CP14_GET_CASE(reg, 13, _val)         \
        CP14_GET_CASE(reg, 14, _val)         \
        CP14_GET_CASE(reg, 15, _val)         \
      }                                      \
    _val;                                    \
  })

#define CP14_MODN(reg, n, val, mask)         \
  CP14_SETN(reg, n, ((CP14_GETN(reg, n) & ~(mask)) | ((uintptr_t)(val) & (mask))))

#define CP14_MASK_ADDR(addr) ((uint32_t)(addr) & ~0x3)

/* Debug ID Register (DIDR) */

#define CP14_DBGDIDR_WRPS_OFFSET             28
#define CP14_DBGDIDR_WRPS_MASK               0xf
#define CP14_DBGDIDR_BRPS_OFFSET             24
#define CP14_DBGDIDR_BRPS_MASK               0xf

#define CP14_DBGDIDR_MAX_BRP                 16
#define CP14_DBGDIDR_MAX_WRP                 16

/* Breakpoint Control Register bit */

#define CP14_DBGBCR_E                        BIT(0)

#define CP14_DBGBCR_PAC_PRIV                 BIT(1)
#define CP14_DBGBCR_PAC_USER                 BIT(2)
#define CP14_DBGBCR_PAC_ALL                  (CP14_DBGBCR_PAC_PRIV | CP14_DBGBCR_PAC_USER)

#define CP14_DBGBCR_LSC_OFFSET               3
#define CP14_DBGBCR_LSC_EXECUTE              0
#define CP14_DBGBCR_LSC_LOAD                 1
#define CP14_DBGBCR_LSC_STORE                2

#define CP14_DBGBCR_BAS_OFFSET               5
#define CP14_DBGBCR_BAS_LEN_1                0x1
#define CP14_DBGBCR_BAS_LEN_2                0x3
#define CP14_DBGBCR_BAS_LEN_4                0xf
#define CP14_DBGBCR_BAS_LEN_8                0xff

/* Watchpoint Control Register bit */

#define CP14_DBGWCR_E                        BIT(0)

#define CP14_DBGWCR_PAC_PRIV                 BIT(1)
#define CP14_DBGWCR_PAC_USER                 BIT(2)

#define CP14_DBGWCR_LSC_OFFSET               3
#define CP14_DBGWCR_LSC_EXECUTE              0
#define CP14_DBGWCR_LSC_LOAD                 1
#define CP14_DBGWCR_LSC_STORE                2

#define CP14_DBGWCR_BAS_OFFSET               5
#define CP14_DBGWCR_BAS_LEN_1                0x1
#define CP14_DBGWCR_BAS_LEN_2                0x3
#define CP14_DBGWCR_BAS_LEN_4                0xf
#define CP14_DBGWCR_BAS_LEN_8                0xff

/* Debug Status and Control Register (DSCR) */

#define CP14_DBGDSCRINT_HDBGEN               BIT(14)
#define CP14_DBGDSCRINT_MDBGEN               BIT(15)

#define CP14_DBGDSCRINT_MOE_OFFSET           2
#define CP14_DBGDSCRINT_MOE_MASK             0Xf
#define CP14_DBGDSCRINT_MOE_BREAKPOINT       0x1
#define CP14_DBGDSCRINT_MOE_ASYNC_WATCHPOINT 0x2
#define CP14_DBGDSCRINT_MOE_CFI_BREAKPOINT   0x3
#define CP14_DBGDSCRINT_MOE_SYNC_WATCHPOINT  0xa

#endif /* __ARCH_ARM_INCLUDE_ARMV7_A_CP14_H */
