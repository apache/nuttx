/****************************************************************************
 * arch/sparc/src/bm3823/bm3823_exceptions.S
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

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.global         _ISR_Handler
	.global		sparc_doirq	/* Dispatch an IRQ                  */

/*
 *  void _ISR_Handler()
 *
 *  This routine provides the RTEMS interrupt management.
 *
 *  We enter this handler from the 4 instructions in the trap table with
 *  the following registers assumed to be set as shown:
 *
 *    l0 = PSR
 *    l1 = PC
 *    l2 = nPC
 *    l3 = trap type
 *
 *  NOTE: By an executive defined convention, trap type is between 0 and 255 if
 *        it is an asynchonous trap and 256 and 511 if it is synchronous.
 */

        .align 4

_ISR_Handler:

        /*
         *  Fix the return address for synchronous traps.
         */

        and     %l3, 0xF0, %l6
        cmp     %l6, 0x10              ! Is this a synchronous trap?
        be,a    dont_do_the_window            ! No, then skip the adjustment
        nop                           ! DELAY
        mov     %l2, %l1              ! do not return to the instruction
        add     %l2, 4, %l2           ! indicated

dont_do_the_window:
        /*
         *  Global registers %g4 and %g5 are saved directly from %l4 and
         *  %l5 directly into the ISF below.
         */
        mov     %g4, %l4                 ! save the globals this block uses
        mov     %g5, %l5

        restore
        std	%l0, [%sp + 0 * 4]	! save L & I registers
        std	%l2, [%sp + 2 * 4]
        std	%l4, [%sp + 4 * 4]
        std	%l6, [%sp + 6 * 4]

        std	%i0, [%sp + 8 * 4]
        std	%i2, [%sp + 10 * 4]
        std	%i4, [%sp + 12 * 4]
        std	%i6, [%sp + 14 * 4]
        save
save_isf:

        /*
         *  Save the state of the interrupted task -- especially the global
         *  registers -- in the Interrupt Stack Frame.  Note that the ISF
         *  includes a regular minimum stack frame which will be used if
         *  needed by register window overflow and underflow handlers.
         *
         *  REGISTERS SAME AS AT _ISR_Handler
         */

        sub     %fp, CONTEXT_CONTROL_INTERRUPT_FRAME_SIZE, %sp
                                               ! make space for ISF

        std     %l0, [%sp + ISF_PSR_OFFSET]    ! save psr, PC
        st      %l2, [%sp + ISF_NPC_OFFSET]    ! save nPC
        st      %g1, [%sp + ISF_G1_OFFSET]     ! save g1
        std     %g2, [%sp + ISF_G2_OFFSET]     ! save g2, g3
        std     %l4, [%sp + ISF_G4_OFFSET]     ! save g4, g5 -- see above
        std     %g6, [%sp + ISF_G6_OFFSET]     ! save g6, g7

        std     %i0, [%sp + ISF_I0_OFFSET]     ! save i0, i1
        std     %i2, [%sp + ISF_I2_OFFSET]     ! save i2, i3
        std     %i4, [%sp + ISF_I4_OFFSET]     ! save i4, i5
        std     %i6, [%sp + ISF_I6_FP_OFFSET]  ! save i6/fp, i7

        rd      %y, %g1
        st      %g1, [%sp + ISF_Y_OFFSET]      ! save y

        mov     %sp, %o1                       ! 2nd arg to ISR Handler
        !add     %o1, 0x60,%o1

        st      %fsr, [%sp + ISF_FSR_OFFSET]
        std     %f0, [%sp + ISF_F0_OFFSET]
        std     %f2, [%sp + ISF_F2_OFFSET]
        std     %f4, [%sp + ISF_F4_OFFSET]
        std     %f6, [%sp + ISF_F6_OFFSET]
        std     %f8, [%sp + ISF_F8_OFFSET]
        std     %f10, [%sp + ISF_F10_OFFSET]
        std     %f12, [%sp + ISF_F12_OFFSET]
        std     %f14, [%sp + ISF_F14_OFFSET]
        std     %f16, [%sp + ISF_F16_OFFSET]
        std     %f18, [%sp + ISF_F18_OFFSET]
        std     %f20, [%sp + ISF_F20_OFFSET]
        std     %f22, [%sp + ISF_F22_OFFSET]
        std     %f24, [%sp + ISF_F24_OFFSET]
        std     %f26, [%sp + ISF_F26_OFFSET]
        std     %f28, [%sp + ISF_F28_OFFSET]
        std     %f30, [%sp + ISF_F30_OFFSET]   ! total 32 word

fix_pil:
        mov     %l0,%g5
        or      %g5, SPARC_PSR_PIL_MASK, %g5   /* 0x00000F00 */
        wr      %g5, SPARC_PSR_ET_MASK, %psr ! **** ENABLE TRAPS **** /* 0x00000020  */
        nop
        nop
        nop
/*==========================================================================*/
                                        ! o1 = 2nd arg = address of the ISF
                                        !   WAS LOADED WHEN ISF WAS SAVED!!!
        mov      %l3, %o0               ! o0 = 1st arg = vector number
        call	sparc_doirq			/* call ISR dispatcher */
        nop
/*==========================================================================*/
        mov      %l0, %psr              ! **** DISABLE TRAPS ****
        nop;
        nop;
        nop;

simple_return:
        ldd     [%o0 + ISF_I6_FP_OFFSET], %i6 ! restore i6/fp, i7
        sub     %fp, CONTEXT_CONTROL_INTERRUPT_FRAME_SIZE, %sp

        ld      [%o0 + ISF_Y_OFFSET], %l5      ! restore y
        wr      %l5, 0, %y

        ldd     [%o0 + ISF_PSR_OFFSET], %l0    ! restore psr, PC
        ld      [%o0 + ISF_NPC_OFFSET], %l2    ! restore nPC
        rd      %psr, %l3
        and     %l3, SPARC_PSR_CWP_MASK, %l3   ! want "current" CWP
        andn    %l0, SPARC_PSR_CWP_MASK, %l0   ! want rest from task
        or      %l3, %l0, %l0                  ! install it later...
        andn    %l0, SPARC_PSR_ET_MASK, %l0

        /*
         *  Restore tasks global and out registers
         */
        ld      [%o0 + ISF_G1_OFFSET], %g1    ! restore g1   ! g1 is restored later
        ldd     [%o0 + ISF_G2_OFFSET], %g2    ! restore g2, g3
        ldd     [%o0 + ISF_G4_OFFSET], %g4    ! restore g4, g5
        ldd     [%o0 + ISF_G6_OFFSET], %g6    ! restore g6, g7

        ldd     [%o0 + ISF_I0_OFFSET], %i0    ! restore i0, i1
        ldd     [%o0 + ISF_I2_OFFSET], %i2    ! restore i2, i3
        ldd     [%o0 + ISF_I4_OFFSET], %i4    ! restore i4, i5
        !ldd     [%o0 + ISF_I6_FP_OFFSET], %i6 ! restore i6/fp, i7

        ldd     [%o0 + PRE_STACK_FRAME_L0_OFFSET], %l4
        std	    %l4, [%fp + CPU_STACK_FRAME_L0_OFFSET]
        ldd     [%o0 + PRE_STACK_FRAME_L2_OFFSET], %l4
        std	    %l4, [%fp + CPU_STACK_FRAME_L2_OFFSET]
        ldd     [%o0 + PRE_STACK_FRAME_L4_OFFSET], %l4
        std	    %l4, [%fp + CPU_STACK_FRAME_L4_OFFSET]
        ldd     [%o0 + PRE_STACK_FRAME_L6_OFFSET], %l4
        std	    %l4, [%fp + CPU_STACK_FRAME_L6_OFFSET]

        ldd     [%o0 + PRE_STACK_FRAME_I0_OFFSET], %l4
        std	    %l4, [%fp + CPU_STACK_FRAME_I0_OFFSET]
        ldd     [%o0 + PRE_STACK_FRAME_I2_OFFSET], %l4
        std	    %l4, [%fp + CPU_STACK_FRAME_I2_OFFSET]
        ldd     [%o0 + PRE_STACK_FRAME_I4_OFFSET], %l4
        std	    %l4, [%fp + CPU_STACK_FRAME_I4_OFFSET]
        ldd     [%o0 + PRE_STACK_FRAME_I6_FP_OFFSET], %l4
        std	    %l4, [%fp + CPU_STACK_FRAME_I6_FP_OFFSET]

        ldd     [%o0 + ISF_F0_OFFSET]  ,%f0
        ldd     [%o0 + ISF_F2_OFFSET]  ,%f2
        ldd     [%o0 + ISF_F4_OFFSET]  ,%f4
        ldd     [%o0 + ISF_F6_OFFSET]  ,%f6
        ldd     [%o0 + ISF_F8_OFFSET]  ,%f8
        ldd     [%o0 + ISF_F10_OFFSET],%f10
        ldd     [%o0 + ISF_F12_OFFSET],%f12
        ldd     [%o0 + ISF_F14_OFFSET],%f14
        ldd     [%o0 + ISF_F16_OFFSET],%f16
        ldd     [%o0 + ISF_F18_OFFSET],%f18
        ldd     [%o0 + ISF_F20_OFFSET],%f20
        ldd     [%o0 + ISF_F22_OFFSET],%f22
        ldd     [%o0 + ISF_F24_OFFSET],%f24
        ldd     [%o0 + ISF_F26_OFFSET],%f26
        ldd     [%o0 + ISF_F28_OFFSET],%f28
        ldd     [%o0 + ISF_F30_OFFSET],%f30
        ld      [%o0 + ISF_FSR_OFFSET],%fsr
        nop
        nop
        nop

good_task_window:

        restore				    ! Interruptee~s window
        ldd	[%sp + 0 * 4], %l0	! restore L & I registers
        ldd	[%sp + 2 * 4], %l2
        ldd	[%sp + 4 * 4], %l4
        ldd	[%sp + 6 * 4], %l6

        ldd	[%sp + 8 * 4], %i0
        ldd	[%sp + 10 * 4], %i2
        ldd	[%sp + 12 * 4], %i4
        ldd	[%sp + 14 * 4], %i6
        save

        mov     %l0, %psr                  !  **** DISABLE TRAPS ****
        nop;
        nop;
        nop
                                           !  and restore condition codes.

        jmp     %l1                        ! transfer control and
        rett    %l2                        ! go back to tasks window
        nop
 /* isr end */

 /* trap handler end*/

