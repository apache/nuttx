;**************************************************************************
; arch/z80/src/ez80/clang-compat.asm
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

; ZDS-II macros end with 'endmac' and the name of the macro. A preprocessor
; macro substitutes this with the GNU assembler's endm and comments out the
; macro name.
#define endmac		endm ;

; The ZDS-II assembler directive DEFINE constructs a segment. This macro
; doesn't provide a complete implementation of the DEFINE directive, but
; instead supports the use of DEFINE required for the arch build. This
; use includes a .RESET section that may be in ROM or RAM, a .STARTUP
; section that is always in ROM, and an IVECTS section that's initialized
; data for the eZ80F92 (in ROM or RAM) and uninitialized data for the
; eZ80F91.
	.macro	define name space align=0
	.ifc \name,.STARTUP
		.section	.startup,"ax",@progbits
	.else
	.ifc \name,.RESET
		.ifc \space,ROM
			.section	.reset,"ax",@progbits
		.else
			.section	.reset,"axw",@progbits
		.endif
	.else
	.ifc \name,.IVECTS
#		ifdef CONFIG_ARCH_CHIP_EZ80F91
			.section	.ivects,"wx",@nobits
#		else
			.section	.ivects,"awx",@nobits
#		endif
	.else
		error "Unsupported define for \name"
	.endif
	.endif
	.endif
	.if \align > 0
		.balign \align
	.endif
	.endm

; The segment macro ensures the SEGMENT directive matches the define macro,
; and includes the CODE segment which should be translated to be the .text
; section.
	.macro	segment name
	.ifc \name,CODE
		.section	.text
	.else
	.ifc \name,.STARTUP
		.section	.startup
	.else
	.ifc \name,.RESET
		.section	.reset
	.else
	.ifc \name,.IVECTS
		.section	.ivects
	.else
		error "Unsupported segment \name"
	.endif ; .ifc name,.IVECTS
	.endif ; .ifc name,.RESET
	.endif ; .ifc name,.STARTUP
	.endif ; .ifc name,CODE
	.endm
