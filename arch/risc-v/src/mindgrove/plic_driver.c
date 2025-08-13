/**
 * Project                               : Secure IoT SoC
 * Name of the file                      : plic_driver.c
 * Brief Description of file             : This is a Baremetal UART Driver file for Mindgrove Silicon's UART Peripheral.
 * Name of Author                        : Sathya Narayanan N, Vignesh Kumar. J 
 * Email ID                              : <sathya281@gmail.com>  <vigneshkumar@mindgrovetech.in>
 * 
 * @file plic_driver.c
 * @brief source file for plic driver
 * @details This file contains the driver code for plic device. The functions to
 * setup each plic registers, isr routine and plic interrupt handler are here.
 * @version 1.0
 * @date 2024-10-08
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2024. All rights reserved.
 * 
 */

#include "plic.h"


/* Macros */

/* Offsets for different registers in plic */

#define PLIC_PRIORITY_OFFSET            0x0000UL
#define PLIC_PENDING_OFFSET             0x1000UL
#define PLIC_ENABLE_OFFSET              0x2000UL

//#if defined(SOS) 
#define PLIC_THRESHOLD_OFFSET           0x200000UL
#define PLIC_CLAIM_OFFSET               0x200004UL
/*#else
//Only context 0 supported*/
//#define PLIC_THRESHOLD_OFFSET           0x10000UL
//#define PLIC_CLAIM_OFFSET               0x10010UL
//#endif

/* The priority value for each int src can be found at addresses 4 bytes apart
   starting from base address + priority offset */

#define PLIC_PRIORITY_SHIFT_PER_INT  2


/*
   Global interrupt data maintenance structure
 */
extern uint16_t interrupt_id;
plic_fptr_t isr_table[PLIC_MAX_INTERRUPT_SRC];
interrupt_data_t hart0_interrupt_matrix[PLIC_MAX_INTERRUPT_SRC];
void *interrupt_arg[PLIC_MAX_INTERRUPT_SRC];

void INTERRUPT_Complete(uint32_t interrupt_id)
{
	//log_trace("\nINTERRUPT_Complete entered\n");

	
	uint32_t *claim_addr =  (uint32_t *) (PLIC_BASE +
					      PLIC_CLAIM_OFFSET);

	*claim_addr = interrupt_id;
	hart0_interrupt_matrix[interrupt_id].state = SERVICED;
	hart0_interrupt_matrix[interrupt_id].count++;

	//log_debug("interrupt id %d, state changed to %d\n", interrupt_id,
		//   hart0_interrupt_matrix[interrupt_id].state);

	//log_debug("interrupt id = %x \n reset to default values state = %x \
		  \n priority = %x\n count = %x\n", \
		  hart0_interrupt_matrix[interrupt_id].id, \
		  hart0_interrupt_matrix[interrupt_id].state, \
		  hart0_interrupt_matrix[interrupt_id].priority, \
		  hart0_interrupt_matrix[interrupt_id].count);

	//log_trace("INTERRUPT_Complete exited\n");
}

/** @fn uint32_t INTERRUPT_Claim_Request()
 * @brief know the id of the interrupt
 * @details read the interrupt claim register to know the interrupt id
 *           of the highest priority pending interrupt
 * @return uint32_t
 */
uint32_t INTERRUPT_Claim_Request()
{
	uint32_t *interrupt_claim_address = NULL;
	uint32_t interrupt_id;

	//log_trace("\nINTERRUPT_Claim_Request entered\n");

	/*
	   return the interrupt id. This will be used to index into the plic isr table.
	   From the isr table, the exact isr will be called

	   refer https://gitlab.com/shaktiproject/uncore/devices/blob/master/plic/plic.bsv as on 26/8/2019
	 */

	
	interrupt_claim_address = (uint32_t *)(PLIC_BASE +
					       PLIC_CLAIM_OFFSET);

	interrupt_id = *interrupt_claim_address;

	//log_debug("interrupt id [%x] claimed  at address %x\n", interrupt_id,
		//   interrupt_claim_address );

	//log_trace("INTERRUPT_Claim_Request exited\n");

	return interrupt_id;
}


void PLIC_Handler( __attribute__((unused)) uintptr_t int_id)
{
	uint32_t  interrupt_id;

	//log_trace("\nPLIC_Handler entered\n");

	interrupt_id = INTERRUPT_Claim_Request();

	//log_debug("interrupt id claimed = %x\n", interrupt_id);

	if (interrupt_id <= 0 || interrupt_id > PLIC_MAX_INTERRUPT_SRC)
	{
		//log_fatal("Fatal error, interrupt id [%x] claimed is wrong\n", interrupt_id);
	}

	/*change state to active*/
	hart0_interrupt_matrix[interrupt_id].state = ACTIVE;

	//log_debug("interrupt id %d, state changed to %d\n",
		//   interrupt_id,hart0_interrupt_matrix[interrupt_id].state);

	/*call relevant interrupt service routine*/
	isr_table[interrupt_id]((uint64_t)interrupt_arg[interrupt_id]);

	INTERRUPT_Complete(interrupt_id);

	//log_debug("interrupt id %d complete \n", interrupt_id);

	//log_trace("\nPLIC_Handler exited\n");
}

/** @fn uint32_t ISR_Default(uint32_t interrupt_id) 
 * @brief default interrupt service routine
 * @details Default isr. Use it when you dont know what to do with interrupts
 * @param uint32_t interrupt_id
 * @return uint32_t
 */
static inline void ISR_Default(uint64_t interrupt_id)
{
	//log_trace("\nISR_Default entered\n");

	if( interrupt_id > 0 && interrupt_id < 7 )  //PWM Interrupts
	{
		/*
		   Assuming 6 pwm's are there
		 */
/**#ifndef SOS
		if(pwm_check_continuous_mode((6-interrupt_id)) == 0)
		{
			set_pwm_control_register((6-interrupt_id),0x80);
		}
#endif
*/
	}

	//log_info("interrupt [%d] serviced\n",interrupt_id);

	//log_trace("\nISR_Default exited\n");
}


void INTERRUPT_Enable(uint32_t interrupt_id)
{
	uint32_t *interrupt_enable_addr;
	uint32_t current_value = 0x00, new_value;

	//log_trace("\nINTERRUPT_Enable entered \n");

	//log_info("interrupt_id = %x\n", interrupt_id);

	//log_debug("PLIC BASE ADDRESS = %x, PLIC ENABLE OFFSET = %x\n" \
			,PLIC_BASE, PLIC_ENABLE_OFFSET);
	
	interrupt_enable_addr = (uint32_t *) (PLIC_BASE +
			PLIC_ENABLE_OFFSET +
			((interrupt_id / 32)*sizeof(uint32_t)));

	current_value = *interrupt_enable_addr;

	//log_info("interrupt_enable_addr = %x current_value = %x \n", \
			interrupt_enable_addr, current_value);

	/*set the bit corresponding to the interrupt src*/
	new_value = current_value | (0x1 << (interrupt_id % 32)); 

	*((uint32_t*)interrupt_enable_addr) = new_value;

	//log_debug("value read: new_value = %x\n", new_value);

	//log_trace("\nINTERRUPT_Enable exited \n");

}

uint8_t INTERRUPT_Disable(uint32_t interrupt_id)
{
	uint32_t *interrupt_disable_addr = 0;
	uint32_t current_value = 0x00, new_value;

	//log_trace("\nINTERRUPT_Disable entered \n");

	//log_debug("interrupt_id = %x\n", interrupt_id);

	//log_debug("PLIC BASE ADDRESS = %x, PLIC ENABLE OFFSET = %x interrupt_id = %x\n",
				//   PLIC_BASE, PLIC_ENABLE_OFFSET,interrupt_id);

	interrupt_disable_addr = (uint32_t *) (PLIC_BASE +
					      PLIC_ENABLE_OFFSET +
					      (interrupt_id / 32)*sizeof(uint32_t));

	current_value = *interrupt_disable_addr;

	//log_debug("interrupt_disable_addr = %x current_value = %x \n",
		//   interrupt_disable_addr, current_value);

	/*unset the bit corresponding to the interrupt src*/
	new_value = current_value & (~(0x1 << (interrupt_id % 32)));

	*interrupt_disable_addr = new_value;

	hart0_interrupt_matrix[interrupt_id].state = INACTIVE;

	//log_debug("interrupt id %d, state changed to %d\n",
		//   interrupt_id,hart0_interrupt_matrix[interrupt_id].state);

	//log_trace("INTERRUPT_Disable exited\n");

	return 0;
}


uint8_t INTERRUPT_Threshold(uint32_t priority_value)
{
	//log_trace("\nINTERRUPT_Threshold entered\n");

	uint32_t *interrupt_threshold_priority = NULL;

	interrupt_threshold_priority = (uint32_t *) (PLIC_BASE +
						     PLIC_THRESHOLD_OFFSET);

	*interrupt_threshold_priority = priority_value;

	//log_info("plic threshold set to %d\n", *interrupt_threshold_priority);

	//log_trace("INTERRUPT_Threshold exited\n");

	return 0;
}

void SET_Interrupt_Priority(uint32_t priority_value, uint32_t int_id)
{
	//log_trace("\n SET_Interrupt_Priority entered %x\n", priority_value);

	uint32_t * interrupt_priority_address;

	/*
	   base address + priority offset + 4*interruptId
	 */

	
	interrupt_priority_address = (uint32_t *) (PLIC_BASE +
						   PLIC_PRIORITY_OFFSET +
						   (int_id <<
						    PLIC_PRIORITY_SHIFT_PER_INT));

	//log_debug("interrupt_priority_address = %x\n", interrupt_priority_address);

	//log_debug("current data at interrupt_priority_address = %x\n", *interrupt_priority_address);

	*interrupt_priority_address = priority_value;

	//log_debug(" new data at interrupt_priority_address = %x\n", *interrupt_priority_address);

	//log_trace("SET_Interrupt_Priority exited\n");

	// return 0;
}

uint8_t PLIC_Init()
{
	uint32_t int_id = 0;

	//log_trace("\nPLIC_Init entered\n");

	/*Assign service routine for external interrupt in machine mode*/
	// mcause_interrupt_table[MACH_EXTERNAL_INTERRUPT] = PLIC_Handler;

	//log_debug("Assigned mach_plic_handler to trap id : %d\n", MACH_EXTERNAL_INTERRUPT);
	
	// hart0_interrupt_matrix[0].state = INACTIVE;
	// hart0_interrupt_matrix[0].id = 0;
	// hart0_interrupt_matrix[0].priority = 0;
	// hart0_interrupt_matrix[0].count = 0;

	for(int_id = 1; int_id <= PLIC_MAX_INTERRUPT_SRC; int_id++)
	{
		// hart0_interrupt_matrix[int_id].state = INACTIVE;
		// hart0_interrupt_matrix[int_id].id = int_id;
		// hart0_interrupt_matrix[int_id].priority = PLIC_PRIORITY_3;
		// hart0_interrupt_matrix[int_id].count = 0;

		//log_debug("\n************************************************");

		/*Disable all interrupts at the beginning*/
		// INTERRUPT_Disable(int_id);

		/*assign a default isr for all interrupts*/
		// isr_table[int_id] = ISR_Default;

		/*set priority for all interrupts*/

		SET_Interrupt_Priority(PLIC_PRIORITY_3, int_id);


		//log_debug("\ninterrupt id = %x \nreset to default values state = %x \
			  \npriority = %x\ncount = %x\n \
			//   \n*************************************************",
			//   hart0_interrupt_matrix[int_id].id, \
			//   hart0_interrupt_matrix[int_id].state, \
			//   hart0_interrupt_matrix[int_id].priority, \
			//   hart0_interrupt_matrix[int_id].count);
	}
  //log_trace("PLIC_Init exited \n");
	INTERRUPT_Threshold(PLIC_PRIORITY_2);
	


  return 0;
}

uint8_t IRQ_Connect(PLIC_Config_t *plic_config)
{

	//log_trace("\nIRQ_Connect entered\n");
	INTERRUPT_Threshold(PLIC_PRIORITY_2);

	INTERRUPT_Enable(plic_config->interrupt_id);

	asm volatile("li      t0, 8\t\n"
		     "csrrs   zero, mstatus, t0\t\n"
		    );
	asm volatile("li      t0, 0x800\t\n"
		     "csrrs   zero, mie, t0\t\n"
		    );

	isr_table[plic_config->interrupt_id] = plic_config->fptr;
    
	SET_Interrupt_Priority(plic_config->priority_value,plic_config->interrupt_id);

	//log_trace("IRQ_Connect exited \n");

	return 0;
}