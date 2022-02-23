/****************************************************************************
 * arch/ceva/include/xm6/reg.h
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

#ifndef __ARCH_CEVA_INCLUDE_XM6_REG_H
#define __ARCH_CEVA_INCLUDE_XM6_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ Stack Frame Format: */

/* The following registers are stored by the interrupt handling
 * logic.
 */

#define REG_SP                0
#define REG_MODI              1
#define REG_MODG              2
#define REG_MODE              3
#define REG_MODD              4
#define REG_MODC              5
#define REG_MODA              6
#define REG_R56               7
#define REG_R57               8
#define REG_R58               9
#define REG_R59               10
#define REG_R60               11
#define REG_R61               12
#define REG_R62               13
#define REG_R63               14
#define REG_R48               15
#define REG_R49               16
#define REG_R50               17
#define REG_R51               18
#define REG_R52               19
#define REG_R53               20
#define REG_R54               21
#define REG_R55               22
#define REG_R40               23
#define REG_R41               24
#define REG_R42               25
#define REG_R43               26
#define REG_R44               27
#define REG_R45               28
#define REG_R46               29
#define REG_R47               30
#define REG_R32               31
#define REG_R33               32
#define REG_R34               33
#define REG_R35               34
#define REG_R36               35
#define REG_R37               36
#define REG_R38               37
#define REG_R39               38
#define REG_R24               39
#define REG_R25               40
#define REG_R26               41
#define REG_R27               42
#define REG_R28               43
#define REG_R29               44
#define REG_R30               45
#define REG_R31               46
#define REG_R16               47
#define REG_R17               48
#define REG_R18               49
#define REG_R19               50
#define REG_R20               51
#define REG_R21               52
#define REG_R22               53
#define REG_R23               54
#define REG_R8                55
#define REG_R9                56
#define REG_R10               57
#define REG_R11               58
#define REG_R12               59
#define REG_R13               60
#define REG_R14               61
#define REG_R15               62
#define REG_R0                63
#define REG_R1                64
#define REG_R2                65
#define REG_R3                66
#define REG_R4                67
#define REG_R5                68
#define REG_R6                69
#define REG_R7                70
#define REG_RETREGN           71
#define REG_RETREG            72
#define REG_RETREG_TEMP       73
#define REG_RETREGI           74
#define REG_MODVL0            75
#define REG_MODVL1            76
#define REG_MODVLL            77
#define REG_MODVFP            78

/* The total number of registers is saved on the stack */

#define XCPTCONTEXT_REGS      79
#define XCPTCONTEXT_SIZE      (4 * XCPTCONTEXT_REGS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_CEVA_INCLUDE_XM6_REG_H */
