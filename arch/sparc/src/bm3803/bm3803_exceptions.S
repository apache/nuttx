/****************************************************************************
 * arch/sparc/src/sparc_v8/up_exceptions.S
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
 *        it is an asynchronous trap and 256 and 511 if it is synchronous.
 */

        .align 4

_ISR_Handler:

        /*
         *  Fix the return address for synchronous traps.
         */

        and     %l3, 0xF0, %l6
        cmp     %l6, 0x10             ! Is this a synchronous trap?
        be,a    win_ovflow            ! No, then skip the adjustment
        nop                           ! DELAY
        mov     %l2, %l1              ! do not return to the instruction
        add     %l2, 4, %l2           ! indicated

win_ovflow:
        /*
         *  Save the globals this block uses.
         *
         *  These registers are not restored from the locals.  Their contents
         *  are saved directly from the locals into the ISF below.
         */

        mov     %g4, %l4                 ! save the globals this block uses
        mov     %g5, %l5
        /*
         *  When at a "window overflow" trap, (wim == (1 << cwp)).
         *  If we get here like that, then process a window overflow.
         */
        rd      %wim, %g4
        srl     %g4,  %l0, %g5           ! g5 = win >> cwp ; shift count and CWP
                                         !   are LS 5 bits ; how convenient :)
        cmp     %g5, 1                   ! Is this an invalid window?
        bne     dont_do_the_window       ! No, then skip all this stuff
        nop
        ! we are using the delay slot

        /*
         *  The following is same as a 1 position right rotate of WIM
         */
        srl     %g4, 1, %g5                    ! g5 = WIM >> 1
        sll     %g4, SPARC_NUMBER_OF_REGISTER_WINDOWS-1 , %g4
                                               ! g4 = WIM << (Number Windows - 1)
        or      %g4, %g5, %g4                  ! g4 = (WIM >> 1) |
                                               !      (WIM << (Number Windows - 1))

        /*
         *  At this point:
         *
         *    g4 = the new WIM
         *    g5 is free
         */

        /*
         *  Since we are tinkering with the register windows, we need to
         *  make sure that all the required information is in global registers.
         */
        save                             ! Save into the window
        wr      %g4, 0, %wim             ! WIM = new WIM
        nop                              ! delay slots
        nop
        nop

        /*
         *  Now save the window just as if we overflowed to it.
         */

SAVE_WINDOW:
        std     %l0, [%sp + CPU_STACK_FRAME_L0_OFFSET]
        std     %l2, [%sp + CPU_STACK_FRAME_L2_OFFSET]
        std     %l4, [%sp + CPU_STACK_FRAME_L4_OFFSET]
        std     %l6, [%sp + CPU_STACK_FRAME_L6_OFFSET]

        std     %i0, [%sp + CPU_STACK_FRAME_I0_OFFSET]
        std     %i2, [%sp + CPU_STACK_FRAME_I2_OFFSET]
        std     %i4, [%sp + CPU_STACK_FRAME_I4_OFFSET]
        std     %i6, [%sp + CPU_STACK_FRAME_I6_FP_OFFSET]

        restore
        nop

dont_do_the_window:
        /*
         *  Global registers %g4 and %g5 are saved directly from %l4 and
         *  %l5 directly into the ISF below.
         */

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
        mov     %l0, %g5
        or      %g5, SPARC_PSR_PIL_MASK, %g5   /* 0x00000F00 */
        wr      %g5, SPARC_PSR_ET_MASK, %psr ! **** ENABLE TRAPS **** /* 0x00000020  */
        nop
        nop
        nop
/*==========================================================================*/
                                       ! o1 = 2nd arg = address of the ISF
                                       !   WAS LOADED WHEN ISF WAS SAVED!!!
        mov     %l3, %o0               ! o0 = 1st arg = vector number
        call	sparc_doirq			/* call ISR dispatcher */
        nop
/*==========================================================================*/
        mov     %l0, %psr              ! **** DISABLE TRAPS ****
        nop;
        nop;
        nop;
       /* If a context switch occurred while processing the interrupt then
        * %o0 may have change value.  If we return any value different
        * from the input regs %o1, then the lower level will know that a context
        * switch occurred during interrupt processing.
        */
        mov     %o0, %g6              ! g6 = %o0
        cmp     %g6, %sp              ! Is this a context switch occurred?
        be,a    simple_return         ! No, then skip the window save
        nop                           ! DELAY

        /*
         *  Flush all windows with valid contents except the current one.
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
         *  In this case, we only would save the used windows -- 6 and 7.
         *
         *   Traps are disabled for the same logical period as in a
         *     flush all windows trap handler.
         *
         *    Register Usage while saving the windows:
         *      g1 = current PSR
         *      g2 = current wim
         *      g3 = CWP
         *      g4 = wim scratch
         *      g5 = scratch
         */

        and     %l0, SPARC_PSR_CWP_MASK, %g3  ! g3 = CWP
        andn    %l0, SPARC_PSR_ET_MASK, %g1   ! g1 = psr with traps disabled
        ! mov     %g1, %psr                     ! **** DISABLE TRAPS ****
        mov     %wim, %g2                     ! g2 = wim
        mov     1, %g4
        sll     %g4, %g3, %g4                 ! g4 = WIM mask for CW invalid

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

        ! Wait three instructions after the write to PSR before using
        ! non-global registers or instructions affecting the CWP
					      ! g1 = psr with traps disabled
					      ! g3 = CWP (interrupt regs window)
        mov     %g1, %psr                     ! restore cwp
        add     %g3, 1, %g2                   ! calculate desired WIM
        and     %g2, SPARC_NUMBER_OF_REGISTER_WINDOWS - 1, %g2
        mov     1, %g4
        sll     %g4, %g2, %g4                 ! g4 = new WIM
        mov     %g4, %wim

        mov     %g6, %o0                      ! %o0 = sp of context switch to

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
        andn    %l0, SPARC_PSR_ET_MASK, %l0    ! **** DISABLE TRAPS ****

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

        /*
         *  Registers:
         *
         *   ALL global registers EXCEPT G1 and the input registers have
         *   already been restored and thuse off limits.
         *
         *   The following is the contents of the local registers:
         *
         *     %l0 = original psr
         *     %l1 = return address (i.e. PC)
         *     %l2 = nPC
         *     %l3 = %tbr
         */

        /*
         *  if (CWP + 1) is an invalid window then we need to reload it.
         *
         *  WARNING: Traps should now be disabled
         */

        mov     %l0, %psr                  !  **** DISABLE TRAPS ****
        nop
        nop
        nop
        rd      %wim, %l4
        add     %l0, 1, %l6                ! l6 = cwp + 1
        and     %l6, SPARC_PSR_CWP_MASK, %l6 ! do the modulo on it
        srl     %l4, %l6, %l5              ! l5 = win >> cwp + 1 ; shift count
                                           !  and CWP are conveniently LS 5 bits
        cmp     %l5, 1                     ! Is tasks window invalid?
        bne     good_task_window

        /*
         *  The following code is the same as a 1 position left rotate of WIM.
         */
        sll     %l4, 1, %l5                    ! l5 = WIM << 1
        srl     %l4, SPARC_NUMBER_OF_REGISTER_WINDOWS-1 , %l4
                                               ! l4 = WIM >> (Number Windows - 1)
        or      %l4, %l5, %l4                  ! l4 = (WIM << 1) |
                                               !      (WIM >> (Number Windows - 1))
        /*
         *  Now restore the window just as if we underflowed to it.
         */
        wr      %l4, 0, %wim                   ! WIM = new WIM
        nop                                    ! must delay after writing WIM
        nop
        nop

        restore                                ! now into the tasks window

        ldd     [%sp + CPU_STACK_FRAME_L0_OFFSET], %l0
        ldd     [%sp + CPU_STACK_FRAME_L2_OFFSET], %l2
        ldd     [%sp + CPU_STACK_FRAME_L4_OFFSET], %l4
        ldd     [%sp + CPU_STACK_FRAME_L6_OFFSET], %l6
        ldd     [%sp + CPU_STACK_FRAME_I0_OFFSET], %i0
        ldd     [%sp + CPU_STACK_FRAME_I2_OFFSET], %i2
        ldd     [%sp + CPU_STACK_FRAME_I4_OFFSET], %i4
        ldd     [%sp + CPU_STACK_FRAME_I6_FP_OFFSET],%i6
                                           ! reload of sp clobbers ISF
        save                               ! Back to ISR dispatch window

good_task_window:

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

