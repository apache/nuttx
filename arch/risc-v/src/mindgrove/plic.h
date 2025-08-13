/**
 * Project                               : Secure IoT SoC
 * Name of the file                      : plic_driver.h
 * Brief Description of file             : This is a Baremetal UART Driver file for Mindgrove Silicon's UART Peripheral.
 * Name of Author                        : Sathya Narayanan N, Vignesh Kumar. J 
 * Email ID                              : <sathya281@gmail.com>  <vigneshkumar@mindgrovetech.in>
 */
/**
 * @file plic_driver.h
 * @brief  Header file for plic driver.c
 * @details This file contains the definitions for plic driver. The memory map
 * for the plic module is also defined here.
 * @version 1.0
 * @date 2024-10-08
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2024. All rights reserved.  
 */


#ifndef PLIC_H
#define PLIC_H


#ifdef __cplusplus
extern "C" {
#endif


// #include "traps.h"
#include"secure_iot_reg.h"
// #include "log.h"
#include "stddef.h"
// #include"errors.h"

#define PLIC_MAX_INTERRUPT_SRC  58

extern void *interrupt_arg[];

/* 7 priority levels are supported.
   PLIC_PRIORITY_1 means 'no interrupt */

#define PLIC_PRIORITY_1 0X00
#define PLIC_PRIORITY_2 0X01
#define PLIC_PRIORITY_3 0X02
#define PLIC_PRIORITY_4 0X04
#define PLIC_PRIORITY_5 0X08
#define PLIC_PRIORITY_6 0X10
#define PLIC_PRIORITY_7 0X20

#define PLIC_PENDING_SHIFT_PER_SOURCE   0

/* Enumerators */

typedef enum
{
	INACTIVE = 0,
	ACTIVE   = 1,
	SERVICED = 2,
	MASKED
}interrupt_status_e;

/* Structures and Unions */

typedef struct
{
	uint32_t id; /*id of the interrupt target source*/
	
	uint32_t priority; /*priority assigned to it*/
	
	interrupt_status_e state; /*state of the interrupt*/
	
	uint32_t count; /*number of times this interrupt occured*/

} interrupt_data_t;

/* Platform Level Interrupt Controller (PLIC) table
   Each entry in the table corresponds to an interrupt service routine */

typedef void (*plic_fptr_t) (uint64_t);
extern plic_fptr_t isr_table[PLIC_MAX_INTERRUPT_SRC];


typedef struct
{
  uint32_t interrupt_id; //id of the interrupt target source.

  uint32_t priority_value;//priority assigned to it.
  
  void* fptr;//Function pointer points the ISR function.
 
} PLIC_Config_t;
 

/* Function prototypes */

/** 
 * @fn void PLIC_Handler(uintptr_t int_id, uintptr_t epc)
 * 
 * @brief handle machine mode plic interrupts.
 * 
 * @details find the int id that caused of interrupt, 
 *	    process it and complete the interrupt.
 * 
 * @param uintptr_t int_id
 * 
 * @param uintptr_t epc
 */

void PLIC_Handler(uintptr_t int_id);

/** @fn void INTERRUPT_Enable(uint32_t interrupt_id)
 * 
 * @brief Enable the interrupt
 * 
 * @details A single bit that enables an interrupt. The bit position corresponds to the interrupt id
 * 
 * @param uint32_t interrupt_id
 */

void INTERRUPT_Enable(uint32_t interrupt_id);
/** 
 * @fn void INTERRUPT_Disable(uint32_t interrupt_id) 
 * 
 * @brief disable an interrupt.
 * 
 * @details A single bit that enables an interrupt.
 *          The bit position corresponds to the interrupt id.
 * 
 * @param uint32_t interrupt_id
 */

uint8_t INTERRUPT_Disable(uint32_t interrupt_id);

/** @fn void INTERRUPT_Threshold(uint32_t priority_value)
 * 
 * @brief set priority threshold for all interrupts.
 * 
 * @details set a threshold on interrrupt priority. Any interruptthat has lesser priority than the threshold is ignored.
 * 
 * @param uint32_t priority_value
 */

uint8_t INTERRUPT_Threshold(uint32_t priority_value);

/** @fn void PLIC_Init()
 * 
 * @brief intitializes the plic module to the default vlaues.
 * 
 * @details Intitializes the plic registers to default values.
 *          Sets up the plic meta data table. Assigns the plic
 *          handler to mcause_interrupt_table.By default interrupts are disabled.
 * 
 */

uint8_t PLIC_Init();

/** @fn void IRQ_Connect(PLIC_Config plic_config)
 * 
 * @brief intitializes the plic module, enable the global interrupts 
 *        and enables the interrupt and corresponding physical pin.
 *        It also assigns isr for the corresponding interrupts and
 *        set priority for an interrupt source.
 * @details Intitializes the plic registers to default values.
 *          Sets up the plic meta data table. Assigns the plic
 *          handler to mcause_interrupt_table.By default interrupts are disabled.
 *          And then enables the interrupt and corresponding physical pin.
 * 
 * @param PLIC_Config_t plic_config which contains the interrupt id and its 
 *        corresponding isr function. It also contain the priority_value.
 */

uint8_t IRQ_Connect(PLIC_Config_t *plic_config);

uint32_t INTERRUPT_Claim_Request();
void INTERRUPT_Complete(uint32_t interrupt_id);

#ifdef __cplusplus
}
#endif

#endif
