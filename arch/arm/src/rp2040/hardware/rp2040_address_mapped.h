/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_address_mapped.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ADDRESS_MAPPED_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ADDRESS_MAPPED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_ALIAS_XOR_BITS (0x1u << 12u)
#define hw_alias_check_addr(addr) ((uintptr_t)(addr))
#define hw_xor_alias_untyped(addr) ((void *)(REG_ALIAS_XOR_BITS | hw_alias_check_addr(addr)))

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

always_inline_function static void hw_xor_bits(volatile uint32_t *addr,
  uint32_t mask)
{
    *(volatile uint32_t *)hw_xor_alias_untyped((volatile void *)addr) = mask;
}

always_inline_function static void hw_write_masked(volatile uint32_t *addr,
  uint32_t values, uint32_t write_mask)
{
    hw_xor_bits(addr, (*addr ^ values) & write_mask);
}

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ADDRESS_MAPPED_H */
