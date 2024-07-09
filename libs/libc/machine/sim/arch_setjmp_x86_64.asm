;***************************************************************************
; libs/libc/machine/sim/arch_setjmp_x86_64.asm
;
; Licensed to the Apache Software Foundation (ASF) under one or more
; contributor license agreements.  See the NOTICE file distributed with
; this work for additional information regarding copyright ownership.  The
; ASF licenses this file to you under the Apache License, Version 2.0 (the
; "License"); you may not use this file except in compliance with the
; License.  You may obtain a copy of the License at
;
;   http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
; WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
; License for the specific language governing permissions and limitations
; under the License.
;
;***************************************************************************

;***************************************************************************
; Public Functions
;***************************************************************************

.code

public setjmp
public longjmp

setjmp:

	; Get the return address, adjusting the stack pointer

	pop rsi

	; Set up the return value

	xor rax, rax

	; Save 1: rbx

	mov [rcx], rbx

	; Save 2: Value of the rsp *after* returning

	mov [rcx+8], rsp

	; Fix up the return stack

	push rsi

	; Save registers
	; Storage order: %rbx, %rsp, %rbp, %r12, %r13, %r14, %r15, %rip

	mov [rcx+10h], rbp
	mov [rcx+18h], r12
	mov [rcx+20h], r13
	mov [rcx+28h], r14
	mov [rcx+30h], r15
	mov [rcx+38h], rsi

	ret

longjmp:

	; Setup return value

	mov rax, rdx

	; Restore registers

	mov rbx, [rcx]
	mov rsp, [rcx+8]
	mov rbp, [rcx+10h]
	mov r12, [rcx+18h]
	mov r13, [rcx+20h]
	mov r14, [rcx+28h]
	mov r15, [rcx+30h]

	; Jump to the saved return address (rsi)

	jmp qword ptr [rcx+38h]

end

