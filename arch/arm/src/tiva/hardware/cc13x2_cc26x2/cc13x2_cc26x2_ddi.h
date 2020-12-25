/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_ddi.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible
 * BSD license:
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_DDI_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_DDI_H

/****************************************************************************
 * This file contains macros for controlling the DDI master and
 * accessing DDI Slave registers via the DDI Master.
 *
 * There are 3 categories of macros in this file:
 *   - macros that provide an offsets to and addresses of a register located
 *     within the DDI Master itself.
 *   - macros that define bits or bitfields within the DDI Master Registers.
 *   - macros that provide an "instruction offset" that are used when
 *     accessing a DDI Slave.
 *
 * The macros that that provide DDI Master register offsets and define bits
 * and bitfields for those registers are the typical macros that appear in
 * most register definition header files.  In the following example
 * TIVA_DDI_CFG_OFFSET is a macro for a register offset, TIVA_DDI_CFG is
 * the corresponding macro for a register address, and DDI_CFG_WAITFORACK
 * is a macro for a bit in that register.  This example code will set the
 * WAITFORACK bit in register TVIA_DDI_CFG of the DDI Master. (Note: this
 * access the Master not the Slave).
 *
 *   uint32_t regval;
 *   regval  = getreg32(TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI_CFG_OFFSET);
 *   regval |= DDI_CFG_WAITFORACK
 *   putreg32(regval, TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI_CFG_OFFSET);
 *
 * The "instruction offset" macros are used to pass an instruction to
 * the DDI Master when accessing DDI slave registers. These macros are
 * only used when accessing DDI Slave Registers. (Remember DDI
 * Master Registers are accessed normally).
 *
 * The instructions supported when accessing a DDI Slave Register follow:
 *   - Direct Access to a DDI Slave register. I.e. read or write the
 *     register.
 *   - Set the specified bits in a DDI Slave register.
 *   - Clear the specified bits in a DDI Slave register.
 *   - Mask write of 4 bits to the a DDI Slave register.
 *   - Mask write of 8 bits to the a DDI Slave register.
 *   - Mask write of 16 bits to the a DDI Slave register.
 *
 * Note: only the "Direct Access" offset should be used when reading
 * a DDI Slave register. Only 8- and 16-bit reads are supported.
 *
 * The generic format of using this marcos for a read follows:
 *
 *   - Read low 16-bits in DDI_SLAVE_OFFSET
 *
 *     myushortvar = getreg16(DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                            DDI_SLAVE_OFFSET);
 *
 *   - Read high 16-bits in DDI_SLAVE_OFFSET add 2 for data[31:16]
 *
 *     myushortvar = getreg16(DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                            DDI_SLAVE_OFFSET + 2);
 *
 *   - Read data[31:24] byte in DDI_SLAVE_OFFSET add 3 for data[31:24]
 *
 *     myuchar = getreg8(DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                       DDI_SLAVE_OFFSET + 3);
 *
 * Notes: In the above example:
 *
 *   - DDI_MASTER_BASE is the base address of the master
 *   - Where TIVA_DDI_DIR_OFFSET is offset of the of the Direct Access
 *     instruction.
 *   - DDI_SLAVE_OFFSET is the DDI Slave offset defined in the
 *     cc13x2_cc25x2_<ddi_slave>.h header file (e.g.
 *     cc13x2_cc25x2_osc_top.h for the OSCSC oscillator modules.
 *
 * Writes can use any of the "instruction macros".
 * The following examples do a "direct write" to DDI Slave register
 * DDI_SLAVE_OFFSET using different size operands:
 *
 * DIRECT WRITES
 *   - Write 32-bits aligned
 *
 *     putreg32(0x12345678, DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                          DDI_SLAVE_OFFSET);
 *
 *   - Write 16-bits aligned to high 16-bits then low 16-bits
 *     Add 2 to get to high 16-bits.
 *
 *     putreg16(0xabcd, DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                      DDI_SLAVE_OFFSET + 2);
 *     putreg16(0xef01, DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                      DDI_SLAVE_OFFSET);
 *
 *   - Write each byte at DDI_SLAVE_OFFSET, one at a time.
 *     Add 1,2,or 3 to get to bytes 1,2, or 3.
 *
 *     putreg8(0x33, DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                   DDI_SLAVE_OFFSET);
 *     putreg8(0x44, DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                   DDI_SLAVE_OFFSET + 1);
 *     putreg8(0x55, DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                   DDI_SLAVE_OFFSET + 2);
 *     putreg8(0x66, DDI_MASTER_BASE + TIVA_DDI_DIR_OFFSET +
 *                   DDI_SLAVE_OFFSET + 3);
 *
 * SET/CLR
 *
 *   The set and clear functions behave similarly to each other. Each
 *   can be performed on an 8-, 16-, or 32-bit operand.
 *   Examples follow:
 *
 *   - Set all odd bits in a 32-bit words
 *
 *     putreg32(0xaaaaaaaa, DDI_MASTER_BASE + TIVA_DDI_SET_OFFSET +
 *                          DDI_SLAVE_OFFSET);
 *
 *   - Clear all bits in byte 2 (data[23:16]) using 32-bit operand
 *
 *      putreg32(0x00ff0000, DDI_MASTER_BASE + TIVA_DDI_CLR_OFFSET +
 *               DDI_SLAVE_OFFSET);
 *
 *   - Set even bits in byte 2 (data[23:16]) using 8-bit operand
 *
 *     putreg8(0x55, DDI_MASTER_BASE + TIVA_DDI_CLR_OFFSET +
 *                   DDI_SLAVE_OFFSET  + 2);
 *
 * MASKED WRITES
 *
 *   The mask writes are a bit different. They operate on nibbles,
 *   bytes, and 16-bit elements. Two operands are required; a 'mask'
 *   and 'data'; The operands are concatenated and written to the master.
 *   e.g. the mask and data are combined as follows for a 16 bit masked
 *   write: (mask << 16) | data;  Examples follow:
 *
 *   - Write 5555 to low 16-bits of DDI_SLAVE_OFFSET register
 *     a long write is needed (32-bits).
 *
 *     putreg32(0xffff5555, DDI_MASTER_BASE + TIVA_DDI_MASK16B_OFFSET +
 *                          DDI_SLAVE_OFFSET);
 *
 *   - Write 1AA to data bits 24:16 in high 16-bits of DDI_SLAVE_OFFSET
 *     register.  Note add 4 for high 16-bits at DDI_SLAVE_OFFSET; mask is
 *     1ff!
 *
 *     putreg32(0x01ff01aa, DDI_MASTER_BASE + TIVA_DDI_MASK16B_OFFSET +
 *                          DDI_SLAVE_OFFSET + 4);
 *
 *   - Do an 8 bit masked write of 00 to low byte of register (data[7:0]).
 *     a short write is needed (16-bits).
 *
 *     putreg16(0xff00, DDI_MASTER_BASE + TIVA_DDI_MASK16B_OFFSET +
 *                      DDI_SLAVE_OFFSET);
 *
 *   - Do an 8 bit masked write of 11 to byte 1 of register (data[15:8]).
 *     add 2 to get to byte 1.
 *
 *     putreg16(0xff11, DDI_MASTER_BASE + TIVA_DDI_MASK16B_OFFSET +
 *                      DDI_SLAVE_OFFSET + 2);
 *
 *   - Do an 8 bit masked write of 33 to high byte of register (data[31:24]).
 *     add 6 to get to byte 3.
 *
 *     putreg16(0xff33, DDI_MASTER_BASE + TIVA_DDI_MASK16B_OFFSET +
 *                      DDI_SLAVE_OFFSET + 6);
 *
 *   - Do an 4 bit masked write (Nibble) of 7 to data[3:0]).
 *     Byte write is needed.
 *
 *     putreg8(0xf7, DDI_MASTER_BASE + TIVA_DDI_MASK16B_OFFSET +
 *                   DDI_SLAVE_OFFSETSET + DDI_MASK16B_OFFSET);
 *
 *   - Do an 4 bit masked write of 4 to data[7:4]).
 *     Add 1 for next nibble
 *
 *     putreg8(0xf4, DDI_MASTER_BASE + TIVA_DDI_MASK16B_OFFSET +
 *                   DDI_SLAVE_OFFSETSET + 1);
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DDI Register Offsets *****************************************************/

#define TIVA_DDI_DIR_OFFSET       0x0000  /* Offset for the direct access instruction */
#define TIVA_DDI_SET_OFFSET       0x0080  /* Offset for 'Set' instruction */
#define TIVA_DDI_CLR_OFFSET       0x0100  /* Offset for 'Clear' instruction */
#define TIVA_DDI_MASK4B_OFFSET    0x0200  /* Offset for 4-bit masked access */
#define TIVA_DDI_MASK8B_OFFSET    0x0300  /* Offset for 8-bit masked access */
#define TIVA_DDI_MASK16B_OFFSET   0x0400  /* Offset for 16-bit masked access */

/* DDI Register Addresses ***************************************************/

/* Register base addresses depend on that base address of the master, e.g.
 * TIVA_AUX_DDI0_OSC_BASE.
 */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_DDI_H */
