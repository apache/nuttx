irqhandler:	.macro vectno
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af					; Offset 7: AF (retaining flags)
	ld		a, \vectno			; A = vector number
	jp		_ez80_irq_common		; Remaining RST handling is common
	.endm
