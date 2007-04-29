/*
 *  The STARTUP.S code is executed after CPU Reset. This file may be 
 *  translated with the following SET symbols. In uVision these SET 
 *  symbols are entered under Options - ASM - Set.
 *
 *  REMAP: when set the startup code initializes the register MEMMAP 
 *  which overwrites the settings of the CPU configuration pins. The 
 *  startup and interrupt vectors are remapped from:
 *     0x00000000  default setting (not remapped)
 *     0x80000000  when EXTMEM_MODE is used
 *     0x40000000  when RAM_MODE is used
 *
 *  EXTMEM_MODE: when set the device is configured for code execution
 *  from external memory starting at address 0x80000000. The startup
 *  vectors are located to 0x80000000.
 *
 *  RAM_MODE: when set the device is configured for code execution
 *  from on-chip RAM starting at address 0x40000000. The startup
 *  vectors are located to 0x40000000.
 */

Reset_Handler:  



/* Memory Mapping */


/* Setup Stack for each mode */
	ldr	r0, =Top_Stack

/* Enter Undefined Instruction Mode and set its Stack Pointer */
	msr     CPSR_c, #UND_MODE | PSR_I_BIT | PSR_F_BIT
	mov     SP, r0
	sub	r0, r0, #UND_Stack_Size

/* Enter Abort Mode and set its Stack Pointer */
	msr     CPSR_c, #ABT_MODE | PSR_I_BIT | PSR_F_BIT
	mov     SP, r0
	sub	r0, r0, #ABT_Stack_Size

/* Enter FIQ Mode and set its Stack Pointer */
	msr     CPSR_c, #FIQ_MODE | PSR_I_BIT | PSR_F_BIT
	mov     SP, r0
	sub	r0, r0, #FIQ_Stack_Size

/* Enter IRQ Mode and set its Stack Pointer */
	msr     CPSR_c, #IRQ_MODE | PSR_I_BIT | PSR_F_BIT
	mov     SP, r0
	sub	r0, r0, #IRQ_Stack_Size

/* Enter Supervisor Mode and set its Stack Pointer */
	msr     CPSR_c, #SVC_MODE | PSR_I_BIT | PSR_F_BIT
	mov     SP, r0
	sub	r0, r0, #SVC_Stack_Size

/* Enter User Mode and set its Stack Pointer */
	msr     CPSR_c, #USR_MODE
	mov     SP, r0

/* Enter the C code */
	ldr	r0,=?C?INIT
	tst	r0,#1	; Bit-0 set: INIT is Thumb
	ldreq   LR,=exit?A  ; ARM Mode
	ldrne   LR,=exit?T  ; Thumb Mode
	bx	r0
	ENDP

PUBLIC exit?A
exit?A	   PROC    CODE32
	B	exit?A
	ENDP

PUBLIC exit?T
exit?T	   PROC    CODE16
exit:	    B	exit?T
	ENDP


	END
