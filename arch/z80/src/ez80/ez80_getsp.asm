;**************************************************************************
; arch/z80/src/ez80/ez80_getsp.asm
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
; Global Symbols Exported
;**************************************************************************

	xdef	_up_getsp

;**************************************************************************
; Code
;**************************************************************************

	segment CODE
	.assume ADL=1

;**************************************************************************
;* Name: _up_getsp
;*
;* Description:
;*   Return the current value of the stack pointer
;*
;**************************************************************************

_up_getsp:
	ld		hl, 0			; Initialize HL to zero
	add		hl, sp			; Add the stack pointer to HL
	ret						; Return stack pointer in HL

	end
