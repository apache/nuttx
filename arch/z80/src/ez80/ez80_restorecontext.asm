;**************************************************************************
; arch/z80/src/ez80/ez80_restorcontext.asm
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
;**************************************************************************

;**************************************************************************
; Global Symbols Imported
;**************************************************************************

;**************************************************************************
; Global Symbols Expported
;**************************************************************************

	xdef	_ez80_restorecontext

;**************************************************************************
; Code
;**************************************************************************

	segment	CODE
	.assume ADL=1

;**************************************************************************
; ez80_restorecontext
;**************************************************************************

_ez80_restorecontext:
	; On entry, stack contains return address (not used), then address
	; of the register save structure

	; Discard the return address, we won't be returning

	pop	hl

	; Get the address of the beginning of the state save area.  Each
	; pop will increment to the next element of the structure

	pop	hl		; BC = Address of save structure
	ld	sp, hl		; SP points to top of storage area

	; Disable interrupts while we muck with the alternative registers.  The
	; Correct interrupt state will be restore below

	di

	; Restore registers.  HL points to the beginning of the reg structure to restore

	ex	af, af'			; Select alternate AF
	pop	af			; Offset 0: AF' = I with interrupt state in parity
	ex	af, af'			;   Restore original AF
	pop	bc			; Offset 1: BC
	pop	de			; Offset 2: DE
	pop	ix			; Offset 3: IX
	pop	iy			; Offset 4: IY
	exx				;   Use alternate BC/DE/HL
	pop	hl			; Offset 5: HL' = Stack pointer after return
	exx				;   Restore original BC/DE/HL
	pop	hl			; Offset 6: HL
	pop	af			; Offset 7: AF

	; Restore the stack pointer

	exx				; Use alternate BC/DE/HL
	pop	de			; DE' = return address
	ld	sp, hl			; Set SP = saved stack pointer value before return
	push	de			; Save return address for ret instruction
	exx				; Restore original BC/DE/HL

	; Restore interrupt state

	ex	af, af'			; Recover interrupt state
	jp	po, noinrestore		; Odd parity, IFF2=0, means disabled
	ex	af, af'			; Restore AF (before enabling interrupts)
	ei				; yes.. Enable interrupts
	ret				; and return
noinrestore:
	ex	af, af'			; Restore AF
	ret				; Return with interrupts disabled
	end
