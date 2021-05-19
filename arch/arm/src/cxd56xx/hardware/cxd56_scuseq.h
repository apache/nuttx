/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_scuseq.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SCUSEQ_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SCUSEQ_H

#define SCUSEQ_SW_REVISION_DATE             (CXD56_SCU_SEQ_DRAM_BASE + 0x000)
#define SCUSEQ_SW_REVISION_TIME             (CXD56_SCU_SEQ_DRAM_BASE + 0x004)
#define SCUSEQ_SW_REVISION_GIT_HASH         (CXD56_SCU_SEQ_DRAM_BASE + 0x008)
#define SCUSEQ_SRC_SEL                      (CXD56_SCU_SEQ_DRAM_BASE + 0x00c)
#define SCUSEQ_REPEAT_TXABORT               (CXD56_SCU_SEQ_DRAM_BASE + 0x010)
#define SCUSEQ_PROPERTY(s)                  (CXD56_SCU_SEQ_DRAM_BASE + 0x020 + ((s) * 0x20))
#define SCUSEQ_OUT_FORMAT(s)                (CXD56_SCU_SEQ_DRAM_BASE + 0x024 + ((s) * 0x20))
#define SCUSEQ_MATH_PROC_OFST_GAIN_X(s)     (CXD56_SCU_SEQ_DRAM_BASE + 0x028 + ((s) * 0x20))
#define SCUSEQ_MATH_PROC_OFST_GAIN_Y(s)     (CXD56_SCU_SEQ_DRAM_BASE + 0x02c + ((s) * 0x20))
#define SCUSEQ_MATH_PROC_OFST_GAIN_Z(s)     (CXD56_SCU_SEQ_DRAM_BASE + 0x030 + ((s) * 0x20))

#define SCUSEQ_INSTRUCTION(x)               (CXD56_SCU_SEQ_DRAM_BASE + 0x160 + ((x) * 2))
#define SCUSEQ_ADC_PROPERTY                 (CXD56_SCU_SEQ_DRAM_BASE + 0x260)
#define SCUSEQ_ADC_MATH_PROC_OFST_GAIN(s)   (CXD56_SCU_SEQ_DRAM_BASE + 0x264 + ((s) * 4))
#define SCUSEQ_FIFOWREVNTCTRL(x)            (CXD56_SCU_SEQ_DRAM_BASE + 0x280 + ((x) * 4))
#define SCUSEQ_FIFOSRAMPOWCTRL              (CXD56_SCU_SEQ_DRAM_BASE + 0x2C0)
#define SCUSEQ_SYNCRO_CPU2ISOP              (CXD56_SCU_SEQ_DRAM_BASE + 0x2C4)
#define SCUSEQ_SYNCRO_ISOP2CPU              (CXD56_SCU_SEQ_DRAM_BASE + 0x2C8)
#define SCUSEQ_RAM_OUT_DATA0                (CXD56_SCU_SEQ_DRAM_BASE + 0x2CC)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SCUSEQ_H */
