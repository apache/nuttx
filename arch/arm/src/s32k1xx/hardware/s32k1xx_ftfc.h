/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_ftfc.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FTFC_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FTFC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FTFC Register Offsets ****************************************************/

/* FTFC Register Offsets ****************************************************/

#define S32K1XX_FTFC_FSTAT_OFFSET	                     0x0000
#define S32K1XX_FTFC_FCNFG_OFFSET	                     0x0001
#define S32K1XX_FTFC_FSEC_OFFSET	                     0x0002
#define S32K1XX_FTFC_FOPT_OFFSET	                     0x0003
#define S32K1XX_FTFC_FCCOB3_OFFSET	                     0x0004
#define S32K1XX_FTFC_FCCOB2_OFFSET	                     0x0005
#define S32K1XX_FTFC_FCCOB1_OFFSET	                     0x0006
#define S32K1XX_FTFC_FCCOB0_OFFSET	                     0x0007
#define S32K1XX_FTFC_FCCOB7_OFFSET	                     0x0008
#define S32K1XX_FTFC_FCCOB6_OFFSET	                     0x0009
#define S32K1XX_FTFC_FCCOB5_OFFSET	                     0x000a
#define S32K1XX_FTFC_FCCOB4_OFFSET	                     0x000b
#define S32K1XX_FTFC_FCCOBB_OFFSET	                     0x000c
#define S32K1XX_FTFC_FCCOBA_OFFSET	                     0x000d
#define S32K1XX_FTFC_FCCOB9_OFFSET	                     0x000e
#define S32K1XX_FTFC_FCCOB8_OFFSET	                     0x000f
#define S32K1XX_FTFC_FPROT3_OFFSET	                     0x0010
#define S32K1XX_FTFC_FPROT2_OFFSET	                     0x0011
#define S32K1XX_FTFC_FPROT1_OFFSET	                     0x0012
#define S32K1XX_FTFC_FPROT0_OFFSET	                     0x0013
#define S32K1XX_FTFC_FEPROT_OFFSET	                     0x0016
#define S32K1XX_FTFC_FDPROT_OFFSET	                     0x0017
#define S32K1XX_FTFC_FCSESTAT_OFFSET                     0x002c
#define S32K1XX_FTFC_FERSTAT_OFFSET                      0x002e
#define S32K1XX_FTFC_FERCNFG_OFFSET                      0x002f

/* FTFC Register Addresses **************************************************/

#define S32K1XX_FTFC_FSTAT                               (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FSTAT_OFFSET)
#define S32K1XX_FTFC_FCNFG                               (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCNFG_OFFSET)
#define S32K1XX_FTFC_FSEC                                (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FSEC_OFFSET)
#define S32K1XX_FTFC_FOPT                                (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FOPT_OFFSET)
#define S32K1XX_FTFC_FCCOB3                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB3_OFFSET)
#define S32K1XX_FTFC_FCCOB2                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB2_OFFSET)
#define S32K1XX_FTFC_FCCOB1                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB1_OFFSET)
#define S32K1XX_FTFC_FCCOB0                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB0_OFFSET)
#define S32K1XX_FTFC_FCCOB7                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB7_OFFSET)
#define S32K1XX_FTFC_FCCOB6                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB6_OFFSET)
#define S32K1XX_FTFC_FCCOB5                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB5_OFFSET)
#define S32K1XX_FTFC_FCCOB4                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB4_OFFSET)
#define S32K1XX_FTFC_FCCOBB                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOBB_OFFSET)
#define S32K1XX_FTFC_FCCOBA                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOBA_OFFSET)
#define S32K1XX_FTFC_FCCOB9                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB9_OFFSET)
#define S32K1XX_FTFC_FCCOB8                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCCOB8_OFFSET)
#define S32K1XX_FTFC_FPROT3                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FPROT3_OFFSET)
#define S32K1XX_FTFC_FPROT2                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FPROT2_OFFSET)
#define S32K1XX_FTFC_FPROT1                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FPROT1_OFFSET)
#define S32K1XX_FTFC_FPROT0                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FPROT0_OFFSET)
#define S32K1XX_FTFC_FEPROT                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FEPROT_OFFSET)
#define S32K1XX_FTFC_FDPROT                              (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FDPROT_OFFSET)
#define S32K1XX_FTFC_FCSESTAT                            (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FCSESTAT_OFFSET)
#define S32K1XX_FTFC_FERSTAT                             (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FERSTAT_OFFSET)
#define S32K1XX_FTFC_FERCNFG                             (S32K1XX_FTFC_BASE + S32K1XX_FTFC_FERCNFG_OFFSET)

/* FTFC Register Bitfield Definitions ***************************************/

#define FTTC_FSTAT_MGSTAT0                               (1 << 0)
#define FTTC_FSTAT_FPVIOL                                (1 << 4)
#define FTTC_FSTAT_ACCERR                                (1 << 5)
#define FTTC_FSTAT_RDCOLERR                              (1 << 6)
#define FTTC_FSTAT_CCIF                                  (1 << 7)

#define FTTC_FCNFG_EEERDY                                (1 << 0)
#define FTTC_FCNFG_RAMRDY                                (1 << 1)

/* Flash controller command numbers *****************************************/

#define S32K1XX_FTFC_VERIFY_BLOCK                        0x00 /* RD1BLK*/
#define S32K1XX_FTFC_VERIFY_SECTION                      0x01 /* RD1SEC*/
#define S32K1XX_FTFC_PROGRAM_CHECK                       0x02 /* PGMCHK*/
#define S32K1XX_FTFC_READ_RESOURCE                       0x03 /* RDRSRC*/
#define S32K1XX_FTFC_PROGRAM_LONGWORD                    0x06 /* PGM4*/
#define S32K1XX_FTFC_PROGRAM_PHRASE                      0x07 /* PGM8*/
#define S32K1XX_FTFC_ERASE_BLOCK                         0x08 /* ERSBLK*/
#define S32K1XX_FTFC_ERASE_SECTOR                        0x09 /* ERSSCR*/
#define S32K1XX_FTFC_PROGRAM_SECTION                     0x0B /* PGMSEC*/
#define S32K1XX_FTFC_GENERATE_CRC                        0x0C /* CRCGEN*/
#define S32K1XX_FTFC_VERIFY_ALL_BLOCK                    0x40 /* RD1ALL*/
#define S32K1XX_FTFC_READ_ONCE                           0x41 /* RDONCE or RDINDEX*/
#define S32K1XX_FTFC_PROGRAM_ONCE                        0x43 /* PGMONCE or PGMINDEX*/
#define S32K1XX_FTFC_ERASE_ALL_BLOCK                     0x44 /* ERSALL*/
#define S32K1XX_FTFC_SECURITY_BY_PASS                    0x45 /* VFYKEY*/
#define S32K1XX_FTFC_SWAP_CONTROL                        0x46 /* SWAP*/
#define S32K1XX_FTFC_ERASE_ALL_BLOCK_UNSECURE            0x49 /* ERSALLU*/
#define S32K1XX_FTFC_VERIFY_ALL_EXECUTE_ONLY_SEGMENT     0x4A /* RD1XA*/
#define S32K1XX_FTFC_ERASE_ALL_EXECUTE_ONLY_SEGMENT      0x4B /* ERSXA*/
#define S32K1XX_FTFC_PROGRAM_PARTITION                   0x80 /* PGMPART */
#define S32K1XX_FTFC_SET_FLEXRAM_FUNCTION                0x81 /* SETRAM */

#define S32K1XX_FTFC_EEEPROM_BASE                        0x14000000

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FTFC_H */
