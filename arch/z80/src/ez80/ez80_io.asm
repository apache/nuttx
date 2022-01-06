;**************************************************************************
; arch/z80/src/ze80/ez80_io.c
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

	xdef	_outp
	xdef	_inp

;**************************************************************************
; Global Symbols Expported
;**************************************************************************

CONFIG_EZ80_Z80MODE	equ	0

;**************************************************************************
; Code
;**************************************************************************

	segment CODE
	.assume ADL=1

;**************************************************************************
; Name: void outp(uint16_t p, uint8_t c)
;
; Description:
;   Output byte c on port p
;
;**************************************************************************

_outp:
	; Create a stack frame

	push	ix
	ld	ix, 0
	add	ix, sp

	; Get the arguments from the stack

	.if CONFIG_EZ80_Z80MODE
	; Stack looks like:
	;
	;	7-8	Unused
	;	 6	Value
	;	4-5	Port
	;	2-3	Return address
	; SP:	0-1	Caller's fame pointer

	ld	bc, (ix + 4)	; Port
	ld	a, (ix + 6)	; Value

	.else
	; Stack looks like:
	;
	;	10-11	Unused
	;	 9	Value
	;	 8	Unused
	;	6-7	Port
	;	3-5	Return address
	; SP:	0-2	Caller's frame pointer

	ld	bc, (ix + 6)	; Port (upper 8 bits not used)
	ld	a, (ix + 9)	; Value

	.endif

	; Output the specified by to the specified 8-bit I/O address

	out	(bc), a
	pop	ix
	ret

;**************************************************************************
; Name: uint8_t inp(uint16_t p)
;
; Description:
;   Input byte from port p
;
;**************************************************************************

_inp:
	; Create a stack frame

	push	ix
	ld	ix, 0
	add	ix, sp

	; Get the arguments from the stack

	.if CONFIG_EZ80_Z80MODE
	; Stack looks like:
	;
	;	4-5	Port
	;	2-3	Return address
	; SP:	0-1	Caller's fame pointer

	ld	bc, (ix + 4)	; Port

	.else
	; Stack looks like:
	;
	;	 8	Unused
	;	6-7	Port
	;	3-5	Return address
	; SP:	0-2	Caller's frame pointer

	ld	bc, (ix + 6)	; Port (upper 8 bits not used)

	.endif

	; Return port value in A

	in	a, (bc)
	pop	ix
	ret

	end
