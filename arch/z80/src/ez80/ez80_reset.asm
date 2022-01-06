;**************************************************************************
; arch/z80/src/ez80/ez80_reset.asm
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
; Macros
;**************************************************************************

; Define one reset handler
;  1. Disable interrupts
;  2. Clear mixed memory mode (MADL) flag
;  3. jump to initialization procedure with jp.lil to set ADL

rstvector: macro
	di
	rsmix
	jp.lil	_ez80_startup
	endmac	rstvector

;**************************************************************************
; Reset entry points
;**************************************************************************

	define	.RESET, space = ROM
	segment	.RESET

_ez80_reset:
_rst0:
	rstvector
_rst8:
	rstvector
_rst10:
	rstvector
_rst18:
	rstvector
_rst20:
	rstvector
_rst28:
	rstvector
_rst30:
	rstvector
_rst38:
	rstvector
	ds 026h
_nmi:
	retn
	end
