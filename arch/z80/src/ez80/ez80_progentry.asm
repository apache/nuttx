;**************************************************************************
; arch/z80/src/ez80/ez80_progentry.asm
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

	xref	_ez80_startup

;**************************************************************************
; Global Symbols Exported
;**************************************************************************

	xdef	_ez80_reset

;**************************************************************************
; Reset entry points
;**************************************************************************

	define	.RESET, space = RAM
	segment	.RESET

_ez80_reset:
	di						; Disable interrupts
	rsmix					; Set MADL control bit to 0
	jp.lil	_ez80_startup	; Perform normal startup
	end
