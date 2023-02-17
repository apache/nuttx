/****************************************************************************
 * arch/sparc/src/bm3823/bm3823_head.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#define RTRAP(_vector, _handler)  mov   %g0, %l0 ; sethi %hi(_handler), %l4 ; jmp %l4+%lo(_handler); mov _vector, %l3
#define TRAP(_vector, _handler)   mov   %psr, %l0; sethi %hi(_handler), %l4 ; jmp %l4+%lo(_handler); mov _vector, %l3

/* Unexpected trap will halt the processor by forcing it to error state */
#define BAD_TRAP          ta 0; nop; nop; nop;
#define SOFT_TRAP         BAD_TRAP   /* Software trap. Treat as BAD_TRAP */

/*
 * System call optimized trap table entry
 */
#define IRQDIS_TRAP(_handler)  \
  mov   %psr, %l0 ; \
  sethi %hi(_handler), %l4 ; \
  jmp   %l4+%lo(_handler); \
   or   %l0, 0x0f00, %l3; ! Set PIL=0xf to disable IRQ

/*
 * System call optimized trap table entry
 */
#define IRQEN_TRAP(_handler)  \
  mov   %psr, %l0 ; \
  sethi %hi(_handler), %l4 ; \
  jmp   %l4+%lo(_handler); \
   andn %l0, 0xf00, %l3; ! Set PIL=0 to Enable IRQ

/*
 * Window Overflow optimized trap table entry
 */
#define WOTRAP(_vector, _handler)  \
  sethi %hi(_handler), %l4; \
  jmp   %l4+%lo(_handler); \
  save; \
   nop

/*
 * Window Underflow optimized trap table entry
 */
#define WUTRAP(_vector, _handler)  \
  mov   %wim, %l3 ; \
  sethi %hi(_handler), %l4 ; \
  jmp   %l4+%lo(_handler); \
   sll  %l3, 1, %l4 ! l4 = WIM << 1

#define BM3823_STACK_TOP       _end+CONFIG_IDLETHREAD_STACKSIZE

    .text
    .global _trap_table, _hardreset
    .global __start
    .global up_lowinit			/* Perform low level initialization */
    .global nx_start			/* NuttX entry point */
    .global _userinit, _end
    .global _window_overflow,  _window_underflow,  _flush_windows, _fpdis_enable
    .global syscall_irqdis, syscall_irqen
    .global _ISR_Handler

    /* Hardware traps */
_trap_table:
_hardreset:
    RTRAP(0,__start);            ! 00 reset trap
    BAD_TRAP;			 ! 01 instruction_access_exception
    BAD_TRAP;			 ! 02 illegal_instruction
    BAD_TRAP;			 ! 03 privileged_instruction
    TRAP(4,_fpdis_enable);	 ! 04 fp_disabled
    WOTRAP(5,_window_overflow);	 ! 05 window_overflow
    WUTRAP(6,_window_underflow); ! 06 window_underflow
    BAD_TRAP;			 ! 07 memory_add0ress_not_aligned
    BAD_TRAP;			 ! 08 fp_exception
    BAD_TRAP;			 ! 09 data_access_exception
    BAD_TRAP;			 ! 0A tag_overflow
    BAD_TRAP;			 ! 0B undefined
    BAD_TRAP;			 ! 0C undefined
    BAD_TRAP;			 ! 0D undefined
    BAD_TRAP;			 ! 0E undefined
    BAD_TRAP;			 ! 0F undefined
    BAD_TRAP;			 ! 10 undefined

    /* Interrupt entries */
    TRAP(0x11,_ISR_Handler) 	       	   ! 11 interrupt level 1
    TRAP(0x12,_ISR_Handler) 		   ! 12 interrupt level 2
    TRAP(0x13,_ISR_Handler) 		   ! 13 interrupt level 3
    TRAP(0x14,_ISR_Handler) 		   ! 14 interrupt level 4
    TRAP(0x15,_ISR_Handler) 		   ! 15 interrupt level 5
    TRAP(0x16,_ISR_Handler) 		   ! 16 interrupt level 6
    TRAP(0x17,_ISR_Handler) 		   ! 17 interrupt level 7
    TRAP(0x18,_ISR_Handler) 		   ! 18 interrupt level 8
    TRAP(0x19,_ISR_Handler) 		   ! 19 interrupt level 9
    TRAP(0x1A,_ISR_Handler) 		   ! 1A interrupt level 1
    TRAP(0x1B,_ISR_Handler) 		   ! 1B interrupt level 11
    TRAP(0x1C,_ISR_Handler) 		   ! 1C interrupt level 12
    TRAP(0x1D,_ISR_Handler) 		   ! 1D interrupt level 13
    TRAP(0x1E,_ISR_Handler) 		   ! 1E interrupt level 14
    TRAP(0x1F,_ISR_Handler) 		   ! 1F interrupt level 15

    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 20 - 23 undefined
    BAD_TRAP;		                            ! 24      cp_disabled
    BAD_TRAP; BAD_TRAP; BAD_TRAP;	            ! 25 - 27 undefined
    BAD_TRAP;		                            ! 28      cp_exception
    BAD_TRAP; BAD_TRAP; BAD_TRAP;	            ! 29 - 2B undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 2C - 2F undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 30 - 33 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 34 - 37 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 38 - 3B undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 3C - 3F undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 40 - 43 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 44 - 47 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 48 - 4B undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 4C - 4F undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 50 - 53 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 54 - 57 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 58 - 5B undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 5C - 5F undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 60 - 63 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 64 - 67 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 68 - 6B undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 6C - 6F undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 70 - 73 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 74 - 77 undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 78 - 7B undefined
    BAD_TRAP; BAD_TRAP; BAD_TRAP; BAD_TRAP;	    ! 7C - 7F undefined

  /*
   *  Software traps
   *
   *  NOTE: At the risk of being redundant... this is not a full
   *        table.  The setjmp on the SPARC requires a window flush trap
   *        handler and RTEMS will preserve the entries that were
   *        installed before.
   */

    TRAP( 0x80, _ISR_Handler );                 ! 80 halt syscall SW trap
    SOFT_TRAP; SOFT_TRAP;                       ! 81 - 82
    TRAP( 0x83, _flush_windows);  		        ! 83 flush windows SW trap

    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; ! 84 - 87
    TRAP( 0x88, _ISR_Handler );                 ! 88

  /*
   *  SW Trap 9-15 Reserved for Operating System
   *
   *  SPARC_SWTRAP_IRQDIS
   *  SPARC_SWTRAP_IRQEN
   */
    IRQDIS_TRAP(syscall_irqdis);              	! 89 IRQ Disable syscall trap
    IRQEN_TRAP(syscall_irqen);                  ! 8A IRQ Enable syscall trap

    SOFT_TRAP;                                  ! 8B
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! 8C - 8F
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! 90 - 93
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! 94 - 97
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! 98 - 9B
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! 9C - 9F
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! A0 - A3
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! A4 - A7
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! A8 - AB
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! AC - AF
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! B0 - B3
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! B4 - B7
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! B8 - BB
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! BC - BF
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! C0 - C3
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! C4 - C7
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! C8 - CB
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! CC - CF
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! D0 - D3
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! D4 - D7
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! D8 - DB
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! DC - DF
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! E0 - E3
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! E4 - E7
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! E8 - EB
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! EC - EF
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! F0 - F3
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! F4 - F7
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! F8 - FB
    SOFT_TRAP; SOFT_TRAP; SOFT_TRAP; SOFT_TRAP;	! FC - FF

/* trap table end*/

/*
 * Init for SPARC-CPU
 */
#define   PSR_INIT   0x10c0       /* Disable traps, set s and ps */

__start:
        mov %g0, %asr16
        mov %g0, %asr17
        nop
        nop

  	! initial the IU register !disable trap
        set 0x10c0, %g1
        mov %g1,%psr
        nop
        nop
        nop

        mov %g0,%wim
        nop
        nop
        nop

        mov %g0,%g1
        mov %g0,%g2
        mov %g0,%g3
        mov %g0,%g4
        mov %g0,%g5
        mov %g0,%g6
        mov %g0,%g7

        /*Fill local and in/out register*/
        mov 0x8,%g1
1:      mov %g0,%l0
        mov %g0,%l1
        mov %g0,%l2
        mov %g0,%l3
        mov %g0,%l4
        mov %g0,%l5
        mov %g0,%l6
        mov %g0,%l7
        mov %g0,%i0
        mov %g0,%i1
        mov %g0,%i2
        mov %g0,%i3
        mov %g0,%i4
        mov %g0,%i5
        mov %g0,%i6
        mov %g0,%i7
        subcc %g1,1,%g1
        save
        bne 1b
        flush
        nop

        ! initial the WIM register = 2
        set  2, %g1
        mov  %g1, %wim
        nop
        nop
        nop

        ! initial the TBR table
        sethi %hi(_trap_table), %g1
        mov  %g1, %tbr
        nop
        nop
        nop

        ! enable traps,fpu in psr
        set 0x10e0, %g1
        mov %g1,%psr
        nop
        nop
        nop
        set EXTER_REG,%g1
        ld  [%g1+4], %g2
        or  %g2, 0x4f, %g2
        st  %g2, [%g1+4]
        nop
        nop

        ! initial the FSR register
        set   _fsrinit,%g1
        ld    [%g1] , %fsr
        nop
        nop
        nop

        ! initial the FPU %f register
        set    _fpdata,%g1
        ldd    [%g1], %f0
        ldd    [%g1], %f2
        ldd    [%g1], %f4
        ldd    [%g1], %f6
        ldd    [%g1], %f8
        ldd    [%g1], %f10
        ldd    [%g1], %f12
        ldd    [%g1], %f14
        ldd    [%g1], %f16
        ldd    [%g1], %f18
        ldd    [%g1], %f20
        ldd    [%g1], %f22
        ldd    [%g1], %f24
        ldd    [%g1], %f26
        ldd    [%g1], %f28
        ldd    [%g1], %f30
	nop

	! initial cache controller
        set   EXTER_REG,   %g1
        set     0x81000f,%g2		!open dcache,icache, dcache snoop,icache burst
        ! set   0x80000f,%g2		!open dcache,icache, dcache snoop
        ! set   0x01000f,%g2		!open dcache,icache, icache burst
        ! set   0x00000f,%g2 		!open dcache,icache
        ! set   0x000000,%g2 		!close dcache,icache
        st    %g2, [%g1 + 0x14]
        nop

        set   EXTER_REG,%g1
        ld    [%g1+4], %g2
        or    %g2, 0x4f, %g2        	/*将存储器配置为读改写（区别于写选通），等待后期设置为最大*/
        st    %g2, [%g1+4]
        nop

        ! initial the stack point
        sethi  %hi(BM3823_STACK_TOP), %g1
        or     %g1, %lo(BM3823_STACK_TOP), %g1
        subcc  %g1, CPU_MINIMUM_STACK_FRAME_SIZE, %g1
        mov    %g1, %sp
        nop

        !call _userinit
        nop

        call    up_lowinit	         ! bm3803_lowinit.c
        nop

        call    nx_start	         ! sched/init/nx_start.c
        nop

_exit:
        ta   0
        nop
        nop

_userinit:        /* clear the bss */
        save   %sp, -64, %sp
        sethi  %hi(__bss_start),%g2
        or     %g2,%lo(__bss_start),%g2  ! g2 = start of bss
        sethi  %hi(_end),%g3
        or     %g3,%lo(_end),%g3         ! g3 = end of bss
        mov    %g0,%g1                   ! so std has two zeros
        sub    %g3, %g2, %g3
zerobss:
        subcc  %g3, 8, %g3
        bge,a   zerobss
        std    %g0,[%g2+%g3]
        set    _end, %o0
        st     %g0,[%o0]
        nop

        ret
        restore

/*
 * Init END
 */

/* Number of register windows */
#define  NWINDOWS  8

        /*
         *  Window overflow trap handler.
         *
         *  On entry:
         *
         *    prev regwin l1 = pc
         *    prev regwin l2 = npc
         */
      .global _window_overflow
_window_overflow:

        /*
         *  Calculate new WIM by "rotating" the valid bits in the WIM right
         *  by one position.  The following shows how the bits move for a SPARC
         *  cpu implementation where SPARC_NUMBER_OF_REGISTER_WINDOWS is 8.
         *
         *    OLD WIM = 76543210
         *    NEW WIM = 07654321
         *
         *  NOTE: New WIM must be stored in a global register since the
         *        "save" instruction just prior to the load of the wim
         *        register will result in the local register set changing.
         */

        std  %l0, [%sp + 0x00]           ! save local register set
        std  %l2, [%sp + 0x08]
        mov  %wim, %l3
        sll  %l3, NWINDOWS-1 , %l2
                                         ! l2  = WIM << (Number Windows - 1)
        std  %l4, [%sp + 0x10]
        std  %l6, [%sp + 0x18]
        srl  %l3, 1, %l3                 ! l3  = WIM >> 1
        wr   %l3, %l2, %wim              ! WIM = (WIM >> 1) ^
                                         !       (WIM << (Number Windows - 1))
                                         ! 3 instruction delay not needed here
        std  %i0, [%sp + 0x20]           ! save input register set
        std  %i2, [%sp + 0x28]
        std  %i4, [%sp + 0x30]
        std  %i6, [%sp + 0x38]
        restore                          ! Go back to trap window.
        jmp  %l1                         ! Re-execute save.
         rett %l2

        /*
         *  Window underflow trap handler.
         *
         *  On entry:
         *
         *    l1 = pc
         *    l2 = npc
         *    l3 = wim (from trap vector)
         *    l4 = wim << 1 (from trap vector)
         */
      .global  _window_underflow
_window_underflow:

        /*
         *  Calculate new WIM by "rotating" the valid bits in the WIM left
         *  by one position.  The following shows how the bits move for a SPARC
         *  cpu implementation where SPARC_NUMBER_OF_REGISTER_WINDOWS is 8.
         *
         *    OLD WIM = 76543210
         *    NEW WIM = 07654321
         *
         *  NOTE: New WIM must be stored in a global register since the
         *        "save" instruction just prior to the load of the wim
         *        register will result in the local register set changing.
         */

        srl  %l3, NWINDOWS-1, %l5
        or   %l5, %l4, %l5              ! l5 = (WIM << 1) |
                                        !      (WIM >> (Number Windows-1))
        mov  %l5, %wim                  ! load the new WIM
        nop; nop; nop                   ! 3 slot delay
        restore                         ! Two restores to get into the
        restore                         ! window to restore
        ldd  [%sp + 0x00], %l0          ! First the local register set
        ldd  [%sp + 0x08], %l2
        ldd  [%sp + 0x10], %l4
        ldd  [%sp + 0x18], %l6
        ldd  [%sp + 0x20], %i0          ! Then the input registers
        ldd  [%sp + 0x28], %i2
        ldd  [%sp + 0x30], %i4
        ldd  [%sp + 0x38], %i6
        save                            ! Get back to the trap window.
        save
        jmp  %l1                        ! Re-execute restore.
         rett  %l2

        /*
         *  Flush All Windows trap handler.
         *
         *  Flush all windows with valid contents except the current one
         *  and the one we will be returning to.
         *
         *  In examining the set register windows, one may logically divide
         *  the windows into sets (some of which may be empty) based on their
         *  current status:
         *
         *    + current (i.e. in use),
         *    + used (i.e. a restore would not trap)
         *    + invalid (i.e. 1 in corresponding bit in WIM)
         *    + unused
         *
         *  Either the used or unused set of windows may be empty.
         *
         *  NOTE: We assume only one bit is set in the WIM at a time.
         *
         *  Given a CWP of 5 and a WIM of 0x1, the registers are divided
         *  into sets as follows:
         *
         *    + 0   - invalid
         *    + 1-4 - unused
         *    + 5   - current
         *    + 6-7 - used
         *
         *  In this case, we only would save the used windows which we
         *  will not be returning to -- 6.
         *
         *    Register Usage while saving the windows:
         *      g1 = current PSR
         *      g2 = current wim
         *      g3 = CWP
         *      g4 = wim scratch
         *      g5 = scratch
         *
         *  On entry:
         *
         *    l0 = psr (from trap table)
         *    l1 = pc
         *    l2 = npc
         */
    .global    _flush_windows
_flush_windows:
        /*
         *  Save the global registers we will be using
         */

        mov     %g1, %l3
        mov     %g2, %l4
        mov     %g3, %l5
        mov     %g4, %l6
        mov     %g5, %l7

        mov     %l0, %g1                      ! g1 = psr
        mov     %wim, %g2                     ! g2 = wim
        and     %l0, SPARC_PSR_CWP_MASK, %g3  ! g3 = CWP

        add     %g3, 1, %g5                   ! g5 = CWP + 1
        and     %g5, SPARC_NUMBER_OF_REGISTER_WINDOWS - 1, %g5

        mov     1, %g4
        sll     %g4, %g5, %g4                 ! g4 = WIM mask for CWP+1 invalid

        restore                               ! go back one register window

save_frame_loop:
        sll     %g4, 1, %g5                   ! rotate the "wim" left 1
        srl     %g4, SPARC_NUMBER_OF_REGISTER_WINDOWS - 1, %g4
        or      %g4, %g5, %g4                 ! g4 = wim if we do one restore

        /*
         *  If a restore would not underflow, then continue.
         */

        andcc   %g4, %g2, %g0                 ! Any windows to flush?
        bnz     done_flushing                 ! No, then continue
        nop

        restore                               ! back one window

        /*
         *  Now save the window just as if we overflowed to it.
         */

        std     %l0, [%sp + CPU_STACK_FRAME_L0_OFFSET]
        std     %l2, [%sp + CPU_STACK_FRAME_L2_OFFSET]
        std     %l4, [%sp + CPU_STACK_FRAME_L4_OFFSET]
        std     %l6, [%sp + CPU_STACK_FRAME_L6_OFFSET]

        std     %i0, [%sp + CPU_STACK_FRAME_I0_OFFSET]
        std     %i2, [%sp + CPU_STACK_FRAME_I2_OFFSET]
        std     %i4, [%sp + CPU_STACK_FRAME_I4_OFFSET]
        std     %i6, [%sp + CPU_STACK_FRAME_I6_FP_OFFSET]

        ba      save_frame_loop
        nop

done_flushing:

        add     %g3, 2, %g3                   ! calculate desired WIM
        and     %g3, SPARC_NUMBER_OF_REGISTER_WINDOWS - 1, %g3
        mov     1, %g4
        sll     %g4, %g3, %g4                 ! g4 = new WIM
        mov     %g4, %wim

        mov     %g1, %psr                     ! restore PSR
        nop
        nop
        nop

        /*
         *  Restore the global registers we used
         */

        mov     %l3, %g1
        mov     %l4, %g2
        mov     %l5, %g3
        mov     %l6, %g4
        mov     %l7, %g5

        jmpl    %l2, %g0
        rett    %l2 + 4

/* fpdis_enable */
    .global  _fpdis_enable
_fpdis_enable:

        jmpl    %l2, %g0
        rett    %l2 + 4
        nop
        nop
/*************************************************/

/*************************************************/
	.data
	.global _fsrinit, _fpdata, g_idle_topstack
	.type	g_idle_topstack, object
.align 8
_fsrinit:
	.word 0x0,0

.align 8
_fpdata:
    .word 0,0

.align 8
g_idle_topstack:
	.long	BM3823_STACK_TOP
	.size	g_idle_topstack, .-g_idle_topstack
	.end

