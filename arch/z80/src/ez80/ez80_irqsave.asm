;**************************************************************************
; arch/z80/src/ez80/ez80_irqsave.asm
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
; Global Symbols Exported
;**************************************************************************

	xdef	_up_irq_save
	xdef	_up_irq_restore
	xdef	_up_irq_enable

;**************************************************************************
; Code
;**************************************************************************

	segment CODE
	.assume ADL=1

;**************************************************************************
;* Name: irqstate_t up_irq_save(void)
;*
;* Description:
;*   Disable all interrupts; return previous interrupt state
;*
;**************************************************************************

_up_irq_save:
	ld		a, i			; AF = interrupt state
	di						; Interrupts are disabled (does not affect F)
	push	af				; Transfer to HL via the stack
	pop		hl				;
	ret						; And return

;**************************************************************************
;* Name: void up_irq_restore(irqstate_t flags)
;*
;* Description:
;*   Restore previous interrupt state
;*
;**************************************************************************

_up_irq_restore:
	di						; Assume disabled
	pop		hl				; HL = return address
	pop		af				; AF Parity bit holds interrupt state
	jp		po, _disabled	; Skip over re-enable if Parity odd
	ei						; Re-enable interrupts
_disabled:
	push	af				; Restore stack
	push	hl				;
	ret						; and return

;**************************************************************************
;* Name: irqstate_t up_irq_enable(void)
;*
;* Description:
;*   Enable all interrupts; return previous interrupt state
;*
;**************************************************************************

_up_irq_enable:
	ld		a, i			; AF = interrupt state
	ei						; Interrupts are enabled (does not affect F)
	push	af				; Transfer to HL via the stack
	pop		hl				;
	ret						; And return

	end
