/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_scuseq.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SCUSEQ_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SCUSEQ_H

#define SCUSEQ_SRC_SEL                      (CXD56_SCU_SEQ_DRAM_BASE + 0x00c)
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
