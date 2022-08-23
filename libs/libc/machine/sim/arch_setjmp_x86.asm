;***************************************************************************
; libs/libc/machine/sim/arch_setjmp_x86.asm
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

.model flat, c
.code

public setjmp
public longjmp

setjmp:
	mov	eax, dword ptr [esp+4]
	mov	dword ptr [eax], ebx
	mov	dword ptr [eax+4], esi
	mov	dword ptr [eax+8], edi

	; Save the value of SP as will be after we return

	lea	ecx, dword ptr [esp+4]
	mov	dword ptr [eax+10h], ecx

	; Save the return PC

	mov	ecx, dword ptr [esp]
	mov	dword ptr [eax+14h], ecx

	; Save the framepointer

	mov	dword ptr [eax+0ch], ebp

	; And return 0

	xor	eax, eax
	ret

longjmp:
	mov	ecx, dword ptr [esp+4]	; jmpbuf in ecx.
	mov	eax, dword ptr [esp+8]	; Second argument is return value

	; Save the return address now

	mov	edx, dword ptr [ecx+14h]

	; Restore registers

	mov	ebx, dword ptr [ecx]
	mov	esi, dword ptr [ecx+4]
	mov	edi, dword ptr [ecx+8]
	mov	ebp, dword ptr [ecx+0ch]
	mov	esp, dword ptr [ecx+10h]
	jmp	dword ptr [ecx+14h]

end
