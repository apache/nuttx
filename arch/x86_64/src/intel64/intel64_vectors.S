/****************************************************************************
 * arch/x86_64/src/intel64/intel64_vector.S
 *
 * SPDX-License-Identifier: Apache-2.0
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

	.file "intel64_vectors.S"

/****************************************************************************
 * .text
 ****************************************************************************/

	.text
	.code64

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl    irq_handler
	.globl    irq_xcp_regs
	.globl    g_interrupt_stack
	.globl    g_interrupt_stack_end
	.globl    g_isr_stack
	.globl    g_isr_stack_end

/****************************************************************************
 * Macros
 ****************************************************************************/

	/* This macro creates a stub for an ISR which does NOT pass it's own
	 * error code (adds a dummy errcode byte).
	 */

	.macro ISR_NOERRCODE, intno
	.globl vector_isr\intno
vector_isr\intno:
	cli               /* Disable interrupts firstly. */

	/* CPU has sawitched to the ISR stack using IST */

	pushq  $0         /* Push a dummy error code. */

	/* Save rdi, rsi */
	pushq   %rdi
	pushq   %rsi
	movq    $\intno,   %rsi     /* INT Number is saved to 2nd parameter of function call */
	jmp     irq_common           /* Go to the common IRQ handler code. */
	.endm

	/* This macro creates a stub for an ISR which passes it's own
	 * error code.
	 */

	.macro  ISR_ERRCODE, intno
	.globl  vector_isr\intno
vector_isr\intno:
	cli               /* Disable interrupts firstly. */

	/* CPU has sawitched to the ISR stack using IST */

	/* Save rdi, rsi */
	pushq   %rdi
	pushq   %rsi
	movq    $\intno,   %rsi     /* INT Number is saved to 2nd parameter of function call */
	jmp     irq_common          /* Go to the common IRQ handler code. */
	.endm

	/* This macro creates a stub for an IRQ - the first parameter is
	 * the IRQ number, the second is the ISR number it is remapped to.
	 */

	.macro  IRQ, irqno, intno
	.globl  vector_irq\irqno
	vector_irq\irqno:
	cli               /* Disable interrupts firstly. */

	/* CPU has switched to the IRQ stack using IST */

	pushq   $0         /* Push a dummy error code. */

	/* Save rdi, rsi */
	pushq   %rdi
	pushq   %rsi
	movq    $\intno,   %rsi     /* INT Number is saved to 2nd parameter of function call */
	jmp     irq_common          /* Go to the common IRQ handler code. */
	.endm

/****************************************************************************
 * IDT Vectors
 ****************************************************************************/
	/* The following will be the vector addresses programmed into the IDT */

	ISR_NOERRCODE     ISR0
	.balign 16
	ISR_NOERRCODE     ISR1
	.balign 16
	ISR_NOERRCODE     ISR2
	.balign 16
	ISR_NOERRCODE     ISR3
	.balign 16
	ISR_NOERRCODE     ISR4
	.balign 16
	ISR_NOERRCODE     ISR5
	.balign 16
	ISR_NOERRCODE     ISR6
	.balign 16
	ISR_NOERRCODE     ISR7
	.balign 16
	ISR_ERRCODE       ISR8
	.balign 16
	ISR_NOERRCODE     ISR9
	.balign 16
	ISR_ERRCODE       ISR10
	.balign 16
	ISR_ERRCODE       ISR11
	.balign 16
	ISR_ERRCODE       ISR12
	.balign 16
	ISR_ERRCODE       ISR13
	.balign 16
	ISR_ERRCODE       ISR14
	.balign 16
	ISR_NOERRCODE     ISR15
	.balign 16
	ISR_NOERRCODE     ISR16
	.balign 16
	ISR_NOERRCODE     ISR17
	.balign 16
	ISR_NOERRCODE     ISR18
	.balign 16
	ISR_NOERRCODE     ISR19
	.balign 16
	ISR_NOERRCODE     ISR20
	.balign 16
	ISR_NOERRCODE     ISR21
	.balign 16
	ISR_NOERRCODE     ISR22
	.balign 16
	ISR_NOERRCODE     ISR23
	.balign 16
	ISR_NOERRCODE     ISR24
	.balign 16
	ISR_NOERRCODE     ISR25
	.balign 16
	ISR_NOERRCODE     ISR26
	.balign 16
	ISR_NOERRCODE     ISR27
	.balign 16
	ISR_NOERRCODE     ISR28
	.balign 16
	ISR_NOERRCODE     ISR29
	.balign 16
	ISR_NOERRCODE     ISR30
	.balign 16
	ISR_NOERRCODE     ISR31
	.balign 16
	IRQ          0,   IRQ0
	.balign 16
	IRQ          1,   IRQ1
	.balign 16
	IRQ          2,   IRQ2
	.balign 16
	IRQ          3,   IRQ3
	.balign 16
	IRQ          4,   IRQ4
	.balign 16
	IRQ          5,   IRQ5
	.balign 16
	IRQ          6,   IRQ6
	.balign 16
	IRQ          7,   IRQ7
	.balign 16
	IRQ          8,   IRQ8
	.balign 16
	IRQ          9,   IRQ9
	.balign 16
	IRQ         10,   IRQ10
	.balign 16
	IRQ         11,   IRQ11
	.balign 16
	IRQ         12,   IRQ12
	.balign 16
	IRQ         13,   IRQ13
	.balign 16
	IRQ         14,   IRQ14
	.balign 16
	IRQ         15,   IRQ15
	.balign 16
	IRQ         16,   IRQ16
	.balign 16
	IRQ         17,   IRQ17
	.balign 16
	IRQ         18,   IRQ18
	.balign 16
	IRQ         19,   IRQ19
	.balign 16
	IRQ         20,   IRQ20
	.balign 16
	IRQ         21,   IRQ21
	.balign 16
	IRQ         22,   IRQ22
	.balign 16
	IRQ         23,   IRQ23
	.balign 16
	IRQ         24,   IRQ24
	.balign 16
	IRQ         25,   IRQ25
	.balign 16
	IRQ         26,   IRQ26
	.balign 16
	IRQ         27,   IRQ27
	.balign 16
	IRQ         28,   IRQ28
	.balign 16
	IRQ         29,   IRQ29
	.balign 16
	IRQ         30,   IRQ30
	.balign 16
	IRQ         31,   IRQ31
	.balign 16
	IRQ         32,   IRQ32
	.balign 16
	IRQ         33,   IRQ33
	.balign 16
	IRQ         34,   IRQ34
	.balign 16
	IRQ         35,   IRQ35
	.balign 16
	IRQ         36,   IRQ36
	.balign 16
	IRQ         37,   IRQ37
	.balign 16
	IRQ         38,   IRQ38
	.balign 16
	IRQ         39,   IRQ39
	.balign 16
	IRQ         40,   IRQ40
	.balign 16
	IRQ         41,   IRQ41
	.balign 16
	IRQ         42,   IRQ42
	.balign 16
	IRQ         43,   IRQ43
	.balign 16
	IRQ         44,   IRQ44
	.balign 16
	IRQ         45,   IRQ45
	.balign 16
	IRQ         46,   IRQ46
	.balign 16
	IRQ         47,   IRQ47
	.balign 16
	IRQ         48,   IRQ48
	.balign 16
	IRQ         49,   IRQ49
	.balign 16
	IRQ         50,   IRQ50
	.balign 16
	IRQ         51,   IRQ51
	.balign 16
	IRQ         52,   IRQ52
	.balign 16
	IRQ         53,   IRQ53
	.balign 16
	IRQ         54,   IRQ54
	.balign 16
	IRQ         55,   IRQ55
	.balign 16
	IRQ         56,   IRQ56
	.balign 16
	IRQ         57,   IRQ57
	.balign 16
	IRQ         58,   IRQ58
	.balign 16
	IRQ         59,   IRQ59
	.balign 16
	IRQ         60,   IRQ60
	.balign 16
	IRQ         61,   IRQ61
	.balign 16
	IRQ         62,   IRQ62
	.balign 16
	IRQ         63,   IRQ63
	.balign 16
	IRQ         64,   IRQ64
	.balign 16
	IRQ         65,   IRQ65
	.balign 16
	IRQ         66,   IRQ66
	.balign 16
	IRQ         67,   IRQ67
	.balign 16
	IRQ         68,   IRQ68
	.balign 16
	IRQ         69,   IRQ69
	.balign 16
	IRQ         70,   IRQ70
	.balign 16
	IRQ         71,   IRQ71
	.balign 16
	IRQ         72,   IRQ72
	.balign 16
	IRQ         73,   IRQ73
	.balign 16
	IRQ         74,   IRQ74
	.balign 16
	IRQ         75,   IRQ75
	.balign 16
	IRQ         76,   IRQ76
	.balign 16
	IRQ         77,   IRQ77
	.balign 16
	IRQ         78,   IRQ78
	.balign 16
	IRQ         79,   IRQ79
	.balign 16
	IRQ         80,   IRQ80
	.balign 16
	IRQ         81,   IRQ81
	.balign 16
	IRQ         82,   IRQ82
	.balign 16
	IRQ         83,   IRQ83
	.balign 16
	IRQ         84,   IRQ84
	.balign 16
	IRQ         85,   IRQ85
	.balign 16
	IRQ         86,   IRQ86
	.balign 16
	IRQ         87,   IRQ87
	.balign 16
	IRQ         88,   IRQ88
	.balign 16
	IRQ         89,   IRQ89
	.balign 16
	IRQ         90,   IRQ90
	.balign 16
	IRQ         91,   IRQ91
	.balign 16
	IRQ         92,   IRQ92
	.balign 16
	IRQ         93,   IRQ93
	.balign 16
	IRQ         94,   IRQ94
	.balign 16
	IRQ         95,   IRQ95
	.balign 16
	IRQ         96,   IRQ96
	.balign 16
	IRQ         97,   IRQ97
	.balign 16
	IRQ         98,   IRQ98
	.balign 16
	IRQ         99,   IRQ99
	.balign 16
	IRQ         100,  IRQ100
	.balign 16
	IRQ         101,  IRQ101
	.balign 16
	IRQ         102,  IRQ102
	.balign 16
	IRQ         103,  IRQ103
	.balign 16
	IRQ         104,  IRQ104
	.balign 16
	IRQ         105,  IRQ105
	.balign 16
	IRQ         106,  IRQ106
	.balign 16
	IRQ         107,  IRQ107
	.balign 16
	IRQ         108,  IRQ108
	.balign 16
	IRQ         109,  IRQ109
	.balign 16
	IRQ         110,  IRQ110
	.balign 16
	IRQ         111,  IRQ111
	.balign 16
	IRQ         112,  IRQ112
	.balign 16
	IRQ         113,  IRQ113
	.balign 16
	IRQ         114,  IRQ114
	.balign 16
	IRQ         115,  IRQ115
	.balign 16
	IRQ         116,  IRQ116
	.balign 16
	IRQ         117,  IRQ117
	.balign 16
	IRQ         118,  IRQ118
	.balign 16
	IRQ         119,  IRQ119
	.balign 16
	IRQ         120,  IRQ120
	.balign 16
	IRQ         121,  IRQ121
	.balign 16
	IRQ         122,  IRQ122
	.balign 16
	IRQ         123,  IRQ123
	.balign 16
	IRQ         124,  IRQ124
	.balign 16
	IRQ         125,  IRQ125
	.balign 16
	IRQ         126,  IRQ126
	.balign 16
	IRQ         127,  IRQ127
	.balign 16
	IRQ         128,  IRQ128
	.balign 16
	IRQ         129,  IRQ129
	.balign 16
	IRQ         130,  IRQ130
	.balign 16
	IRQ         131,  IRQ131
	.balign 16
	IRQ         132,  IRQ132
	.balign 16
	IRQ         133,  IRQ133
	.balign 16
	IRQ         134,  IRQ134
	.balign 16
	IRQ         135,  IRQ135
	.balign 16
	IRQ         136,  IRQ136
	.balign 16
	IRQ         137,  IRQ137
	.balign 16
	IRQ         138,  IRQ138
	.balign 16
	IRQ         139,  IRQ139
	.balign 16
	IRQ         140,  IRQ140
	.balign 16
	IRQ         141,  IRQ141
	.balign 16
	IRQ         142,  IRQ142
	.balign 16
	IRQ         143,  IRQ143
	.balign 16
	IRQ         144,  IRQ144
	.balign 16
	IRQ         145,  IRQ145
	.balign 16
	IRQ         146,  IRQ146
	.balign 16
	IRQ         147,  IRQ147
	.balign 16
	IRQ         148,  IRQ148
	.balign 16
	IRQ         149,  IRQ149
	.balign 16
	IRQ         150,  IRQ150
	.balign 16
	IRQ         151,  IRQ151
	.balign 16
	IRQ         152,  IRQ152
	.balign 16
	IRQ         153,  IRQ153
	.balign 16
	IRQ         154,  IRQ154
	.balign 16
	IRQ         155,  IRQ155
	.balign 16
	IRQ         156,  IRQ156
	.balign 16
	IRQ         157,  IRQ157
	.balign 16
	IRQ         158,  IRQ158
	.balign 16
	IRQ         159,  IRQ159
	.balign 16
	IRQ         160,  IRQ160
	.balign 16
	IRQ         161,  IRQ161
	.balign 16
	IRQ         162,  IRQ162
	.balign 16
	IRQ         163,  IRQ163
	.balign 16
	IRQ         164,  IRQ164
	.balign 16
	IRQ         165,  IRQ165
	.balign 16
	IRQ         166,  IRQ166
	.balign 16
	IRQ         167,  IRQ167
	.balign 16
	IRQ         168,  IRQ168
	.balign 16
	IRQ         169,  IRQ169
	.balign 16
	IRQ         170,  IRQ170
	.balign 16
	IRQ         171,  IRQ171
	.balign 16
	IRQ         172,  IRQ172
	.balign 16
	IRQ         173,  IRQ173
	.balign 16
	IRQ         174,  IRQ174
	.balign 16
	IRQ         175,  IRQ175
	.balign 16
	IRQ         176,  IRQ176
	.balign 16
	IRQ         177,  IRQ177
	.balign 16
	IRQ         178,  IRQ178
	.balign 16
	IRQ         179,  IRQ179
	.balign 16
	IRQ         180,  IRQ180
	.balign 16
	IRQ         181,  IRQ181
	.balign 16
	IRQ         182,  IRQ182
	.balign 16
	IRQ         183,  IRQ183
	.balign 16
	IRQ         184,  IRQ184
	.balign 16
	IRQ         185,  IRQ185
	.balign 16
	IRQ         186,  IRQ186
	.balign 16
	IRQ         187,  IRQ187
	.balign 16
	IRQ         188,  IRQ188
	.balign 16
	IRQ         189,  IRQ189
	.balign 16
	IRQ         190,  IRQ190
	.balign 16
	IRQ         191,  IRQ191
	.balign 16
	IRQ         192,  IRQ192
	.balign 16
	IRQ         193,  IRQ193
	.balign 16
	IRQ         194,  IRQ194
	.balign 16
	IRQ         195,  IRQ195
	.balign 16
	IRQ         196,  IRQ196
	.balign 16
	IRQ         197,  IRQ197
	.balign 16
	IRQ         198,  IRQ198
	.balign 16
	IRQ         199,  IRQ199
	.balign 16
	IRQ         200,  IRQ200
	.balign 16
	IRQ         201,  IRQ201
	.balign 16
	IRQ         202,  IRQ202
	.balign 16
	IRQ         203,  IRQ203
	.balign 16
	IRQ         204,  IRQ204
	.balign 16
	IRQ         205,  IRQ205
	.balign 16
	IRQ         206,  IRQ206
	.balign 16
	IRQ         207,  IRQ207
	.balign 16
	IRQ         208,  IRQ208
	.balign 16
	IRQ         209,  IRQ209
	.balign 16
	IRQ         210,  IRQ210
	.balign 16
	IRQ         211,  IRQ211
	.balign 16
	IRQ         212,  IRQ212
	.balign 16
	IRQ         213,  IRQ213
	.balign 16
	IRQ         214,  IRQ214
	.balign 16
	IRQ         215,  IRQ215
	.balign 16
	IRQ         216,  IRQ216
	.balign 16
	IRQ         217,  IRQ217
	.balign 16
	IRQ         218,  IRQ218
	.balign 16
	IRQ         219,  IRQ219
	.balign 16
	IRQ         220,  IRQ220
	.balign 16
	IRQ         221,  IRQ221
	.balign 16
	IRQ         222,  IRQ222
	.balign 16
	IRQ         223,  IRQ223
	.balign 16
	IRQ         224,  IRQ224
	.balign 16
	IRQ         225,  IRQ225
	.balign 16
	IRQ         226,  IRQ226
	.balign 16
	IRQ         227,  IRQ227
	.balign 16
	IRQ         228,  IRQ228
	.balign 16
	IRQ         229,  IRQ229
	.balign 16
	IRQ         230,  IRQ230
	.balign 16
	IRQ         231,  IRQ231
	.balign 16
	IRQ         232,  IRQ232
	.balign 16
	IRQ         233,  IRQ233
	.balign 16
	IRQ         234,  IRQ234
	.balign 16
	IRQ         235,  IRQ235
	.balign 16
	IRQ         236,  IRQ236
	.balign 16
	IRQ         237,  IRQ237
	.balign 16
	IRQ         238,  IRQ238
	.balign 16
	IRQ         239,  IRQ239
	.balign 16
	IRQ         240,  IRQ240
	.balign 16
	IRQ         241,  IRQ241
	.balign 16
	IRQ         242,  IRQ242
	.balign 16
	IRQ         243,  IRQ243
	.balign 16
	IRQ         244,  IRQ244
	.balign 16
	IRQ         245,  IRQ245
	.balign 16
	IRQ         246,  IRQ246
	.balign 16
	IRQ         247,  IRQ247
	.balign 16
	IRQ         248,  IRQ248
	.balign 16
	IRQ         249,  IRQ249
	.balign 16
	IRQ         250,  IRQ250
	.balign 16
	IRQ         251,  IRQ251
	.balign 16
	IRQ         252,  IRQ252
	.balign 16
	IRQ         253,  IRQ253
	.balign 16
	IRQ         254,  IRQ254
	.balign 16
	IRQ         255,  IRQ255
	.balign 16

/****************************************************************************
 * Name: irq_common
 *
 * Description:
 *   This is the common IRQ logic. It saves the processor state, sets up for
 *   kernel mode segments, calls the C-level fault handler, and finally
 *   restores the stack frame.
 *
 ****************************************************************************/

	.type   irq_common, @function
irq_common:
	/* Already swap to the interrupt stack
	 * stack is automatically recovered by iretq using task state
	 */

	/* Get current task regs area - this logic assumes that irq_xcp_regs
	 * corrupts only RAX, RDI and RDX registers.
	 */

	pushq   %rax
	pushq   %rdx
	call    irq_xcp_regs
	movq    %rax, %rdi

	/* x86_64 don't have pusha, we have to do things manually.
	 * RDI and RSI are pushed above for handling IRQ no.
	 * RAX is on stack now, so we have to pop it.
	 */

	popq    (8*REG_RDX)(%rdi)
	popq    (8*REG_RAX)(%rdi)
	movq    %rcx, (8*REG_RCX)(%rdi)
	movq    %r8,  (8*REG_R8)(%rdi)
	movq    %r9,  (8*REG_R9)(%rdi)

	movq    %r15, (8*REG_R15)(%rdi)
	movq    %r14, (8*REG_R14)(%rdi)
	movq    %r13, (8*REG_R13)(%rdi)
	movq    %r12, (8*REG_R12)(%rdi)
	movq    %r11, (8*REG_R11)(%rdi)
	movq    %r10, (8*REG_R10)(%rdi)
	movq    %rbp, (8*REG_RBP)(%rdi)
	movq    %rbx, (8*REG_RBX)(%rdi)

	xor     %rax, %rax             /* Reset rax */
	mov     %ds, %ax               /* Lower 16-bits of rax. */
	movq    %rax, (8*REG_DS)(%rdi) /* Save the data segment descriptor */
	mov     %es, %ax               /* Lower 16-bits of rax. */
	movq    %rax, (8*REG_ES)(%rdi) /* Save the data segment descriptor */
	mov     %gs, %ax               /* Lower 16-bits of rax. */
	movq    %rax, (8*REG_GS)(%rdi) /* Save the data segment descriptor */
	mov     %fs, %ax               /* Lower 16-bits of rax. */
	movq    %rax, (8*REG_FS)(%rdi) /* Save the data segment descriptor */

	/* Save registers from stack */

	movq    0(%rsp), %rcx
	movq    %rcx, (8*REG_RSI)(%rdi)
	movq    8(%rsp), %rcx
	movq    %rcx, (8*REG_RDI)(%rdi)
	movq    16(%rsp), %rcx
	movq    %rcx, (8*REG_ERRCODE)(%rdi)
	movq    24(%rsp), %rcx
	movq    %rcx, (8*REG_RIP)(%rdi)
	movq    32(%rsp), %rcx
	movq    %rcx, (8*REG_CS)(%rdi)
	movq    40(%rsp), %rcx
	movq    %rcx, (8*REG_RFLAGS)(%rdi)
	movq    48(%rsp), %rcx
	movq    %rcx, (8*REG_RSP)(%rdi)
	movq    56(%rsp), %rcx
	movq    %rcx, (8*REG_SS)(%rdi)

	/* Registers in RDI are already properly aligned */

#ifndef CONFIG_ARCH_X86_64_HAVE_XSAVE
	fxsaveq (%rdi)
#else
	movl    $XSAVE_STATE_COMPONENTS, %eax
	xor     %edx, %edx
	xsave   (%rdi)
#endif

	/* The current value of the RDI points to the beginning of the state save
	 * structure. Push this value on stack and also push a dummy value so
	 * that we don't lose the correct stack alignment for vector operations
	 */

	pushq   %rdi
	pushq   $0
	call    irq_handler
	add     $8, %rsp
	popq    %rdi

	/* The common return point for irq_handler */

.Lreturn:

	/* Check if full context switch is required for signal handling */

	movq    (8*REG_AUX)(%rax), %rcx
	cmp     $(REG_AUX_FULLCONTEXT), %rcx
	je      .Lfullswitch

	/* EAX may possibly hold a pointer to a different register save area on
	 * return.  Are we switching to a new context?
	 */

	cmp     %rax, %rdi
	je      .Lnoswitch

.Lfullswitch:
	/* Reset flag */

	movq    $0x0, (8*REG_AUX)(%rdi)

	/* A context swith will be performed. RAX holds the address of the new
	 * register save structure.
	 *
	 * Jump to x86_64_fullcontextrestore().  We perform a call here, but that function
	 * never returns.  The address of the new register save block is the argument
	 * to the x86_64_fullcontextrestore().
	 */

	movq    %rax, %rdi
	call    x86_64_fullcontextrestore

.Lnoswitch:
#ifndef CONFIG_ARCH_X86_64_HAVE_XSAVE
	fxrstorq (%rdi)
#else
	movl $XSAVE_STATE_COMPONENTS, %eax
	xor     %edx, %edx
	xrstor  (%rdi)
#endif

	movq    (8*REG_FS)(%rdi), %rax
	mov     %fs, %ax
	movq    (8*REG_GS)(%rdi), %rax
	mov     %gs, %ax
	movq    (8*REG_ES)(%rdi), %rax
	mov     %es, %ax
	movq    (8*REG_DS)(%rdi), %rax
	mov     %ds, %ax

	movq    (8*REG_RAX)(%rdi), %rax
	movq    (8*REG_RBX)(%rdi), %rbx
	movq    (8*REG_RBP)(%rdi), %rbp
	movq    (8*REG_R10)(%rdi), %r10
	movq    (8*REG_R11)(%rdi), %r11
	movq    (8*REG_R12)(%rdi), %r12
	movq    (8*REG_R13)(%rdi), %r13
	movq    (8*REG_R14)(%rdi), %r14
	movq    (8*REG_R15)(%rdi), %r15

	movq    (8*REG_R9)(%rdi), %r9
	movq    (8*REG_R8)(%rdi), %r8
	movq    (8*REG_RCX)(%rdi), %rcx
	movq    (8*REG_RDX)(%rdi), %rdx

	/* Pop RDI and RSI pushed on interrupt entry */

	popq   %rsi
	popq   %rdi

	/* Cleans up the pushed error code */

	add     $8, %rsp

	iretq                /* Pops 5 things at once: RIP, CS, RFLAGS RSP and SS */
	.size	irq_common, . - irq_common
	.end

