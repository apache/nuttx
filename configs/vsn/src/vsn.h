/************************************************************************************
 * configs/vsn/src/vsn.h
 * arch/arm/src/board/vsn.n
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Copyright (c) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Gregory Nutt <spudmonkey@racsa.co.cr>
 *            Uros Platise <uros.platise@isotel.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIGS_VSN_1_2_SRC_VSN_INTERNAL_H
#define __CONFIGS_VSN_SRC_VSN_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* LED */

#define GPIO_LED		(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTB|GPIO_PIN2 |GPIO_OUTPUT_CLEAR)
                         
/* BUTTON - Note that after a good second button causes hardware reset */

#define GPIO_PUSHBUTTON	(GPIO_INPUT |GPIO_CNF_INFLOAT  |GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN5 )

/* Power Management Pins */

#define GPIO_PVS		(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN7 |GPIO_OUTPUT_CLEAR)
#define GPIO_PST		(GPIO_INPUT |GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN13)
#define GPIO_SCTC		(GPIO_INPUT |GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8 )
#define GPIO_PCLR		(GPIO_INPUT |GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN1 )	// by default this pin is OSCOUT, requires REMAP
#define GPIO_XPWR		(GPIO_INPUT |GPIO_CNF_INFLOAT  |GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN4 )

/* FRAM */

#define GPIO_FRAM_CS	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN15|GPIO_OUTPUT_SET)


/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_lowprintf
#  else
#    define message printf
#  endif
#endif


/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the VSN board.
 *
 ************************************************************************************/

extern void weak_function stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the VSN board.
 *
 ************************************************************************************/

extern void weak_function stm32_usbinitialize(void);


/************************************************************************************
 * Init Power Module and set board system voltage
 ************************************************************************************/

extern void board_power_init(void);


#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_VSN_SRC_VSN_INTERNAL_H */

