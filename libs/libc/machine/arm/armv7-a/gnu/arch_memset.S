/*
 * Copyright (C) 2013 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
	.arm
	.syntax unified
	.global memset
	.type	memset, %function
memset:
        mov         r3, r0
        // At this point only d0, d1 are going to be used below.
        vdup.8      q0, r1
        cmp         r2, #16
        blo         .L_set_less_than_16_unknown_align

.L_check_alignment:
        // Align destination to a double word to avoid the store crossing
        // a cache line boundary.
        ands        ip, r3, #7
        bne         .L_do_double_word_align

.L_double_word_aligned:
        // Duplicate since the less than 64 can use d2, d3.
        vmov        q1, q0
        subs        r2, #64
        blo         .L_set_less_than_64

        // Duplicate the copy value so that we can store 64 bytes at a time.
        vmov        q2, q0
        vmov        q3, q0

1:      // Main loop stores 64 bytes at a time.
        subs        r2, #64
        vstmia      r3!, {d0 - d7}
        bge         1b

.L_set_less_than_64:
        // Restore r2 to the count of bytes left to set.
        add         r2, #64
        lsls        ip, r2, #27
        bcc         .L_set_less_than_32
        // Set 32 bytes.
        vstmia      r3!, {d0 - d3}

.L_set_less_than_32:
        bpl         .L_set_less_than_16
        // Set 16 bytes.
        vstmia      r3!, {d0, d1}

.L_set_less_than_16:
        // Less than 16 bytes to set.
        lsls        ip, r2, #29
        bcc         .L_set_less_than_8

        // Set 8 bytes.
        vstmia      r3!, {d0}

.L_set_less_than_8:
        bpl         .L_set_less_than_4
        // Set 4 bytes
        vst1.32     {d0[0]}, [r3]!

.L_set_less_than_4:
        lsls        ip, r2, #31
        it          ne
        strbne      r1, [r3], #1
        itt         cs
        strbcs      r1, [r3], #1
        strbcs      r1, [r3]
        bx          lr

.L_do_double_word_align:
        rsb         ip, ip, #8
        sub         r2, r2, ip

        // Do this comparison now, otherwise we'll need to save a
        // register to the stack since we've used all available
        // registers.
        cmp         ip, #4
        blo         1f

        // Need to do a four byte copy.
        movs        ip, ip, lsl #31
        it          mi
        strbmi      r1, [r3], #1
        itt         cs
        strbcs      r1, [r3], #1
        strbcs      r1, [r3], #1
        vst1.32     {d0[0]}, [r3]!
        b           .L_double_word_aligned

1:
        // No four byte copy.
        movs        ip, ip, lsl #31
        it          mi
        strbmi      r1, [r3], #1
        itt         cs
        strbcs      r1, [r3], #1
        strbcs      r1, [r3], #1
        b           .L_double_word_aligned

.L_set_less_than_16_unknown_align:
        // Set up to 15 bytes.
        movs        ip, r2, lsl #29
        bcc         1f
        vst1.8      {d0}, [r3]!
1:      bge         2f
        vst1.32     {d0[0]}, [r3]!
2:      movs        ip, r2, lsl #31
        it          mi
        strbmi      r1, [r3], #1
        itt         cs
        strbcs      r1, [r3], #1
        strbcs      r1, [r3], #1
        bx          lr
	.size memset, . - memset