/****************************************************************************
 * arch/risc-v/src/allwinner-d1/allwinner_d1_head.S
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/arch.h>
#include <arch/irq.h>

#include "chip.h"
#include "riscv_internal.h"
#include "riscv_macros.S"

  .section .text
  .global __start
  .type   __start, function

__start:
  /* Linux-compatible RISC-V image header.  The image is linked 2 MiB above
   * the D1 DRAM base, as required by the RISC-V boot protocol.
   */

  c.li    s4, -13
  j       real_start
  .long   0
  .quad   0x200000
  .quad   _ebss - __start
  .quad   0
  .long   2
  .long   0
  .quad   0
  .ascii  "RISCV\x00\x00\x00"
  .ascii  "RSC\x05"
  .long   0

real_start:
#ifdef CONFIG_SMP
  li      t1, CONFIG_SMP_NCPUS
#else
  li      t1, 1
#endif
  blt     a0, t1, 1f
  csrw    CSR_SIE, zero
0:
  wfi
  j       0b

1:
  riscv_set_inital_sp ALLWINNER_D1_IDLESTACK_BASE, SMP_STACK_SIZE, a0
  csrw    CSR_SIE, zero
  la      t0, __trap_vec
  csrw    CSR_STVEC, t0
  jal     x1, allwinner_d1_start

  .global _init
  .global _fini
_init:
_fini:
  ret
