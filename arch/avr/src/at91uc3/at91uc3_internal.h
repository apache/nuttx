/************************************************************************************
 * arch/avr/src/at91uc3b/at91uc3_internal.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_AVR_SRC_AVR32_AT91UC3_INTERNAL_H
#define __ARCH_AVR_SRC_AVR32_AT91UC3_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Bit-encoded input to at91uc3_configgpio() ****************************************/

/* 32-bit Encoding:
 * xxxx xxxx xxxx xxxF MMIU VXXG PPPB BBBB
 */

/* Glitch Filter Enable:
 * .... .... .... ...F .... .... .... ....
 */

 #define GPIO_GLITCH               (1 << 16) /* Bit 16: Glitch filter enable */

/* Interrupt modes (valid only if GPIO_INTR==1)
 * .... .... .... .... MM.. .... .... ....
 */

#define GPIO_INTMODE_SHIFT         (14)      /* Bits 14-15: Interrupt mode */
#define GPIO_INTMODE_MASK          (3 << GPIO_INTMODE_SHIFT)
#  define GPIO_INTMODE_BOTH        (0 << GPIO_INTMODE_SHIFT)
#  define GPIO_INTMODE_RISING      (1 << GPIO_INTMODE_SHIFT)
#  define GPIO_INTMODE_FALLING     (2 << GPIO_INTMODE_SHIFT)

/* Interrupt enable
 * .... .... .... .... ..I. .... .... ....
 */

#define GPIO_INTR                  (1 << 13) /* Bit 13: Interrupt enable */

/* Pull-up enable
 * .... .... .... .... ...U .... .... ....
 */

#define GPIO_PULLUP                (1 << 12) /* Bit 12: Pull-up enable */

/* Output value (Valid only if GPIO_ENABLE==1 and GPIO_OUTPUT==1)
 * .... .... .... .... .... V... .... ....
 */

#define GPIO_VALUE                 (1 << 11) /* Bit 11: Output value */

/* Peripheral MUX setting (valid only if GPIO_PERIPH)
 * .... .... .... .... .... .XX. .... ....
 */

#define GPIO_MUX_SHIFT             (9)       /* Bits 9-10: Peripheral MUX */
#define GPIO_MUX_MASK              (3 << GPIO_MUX_SHIFT)
#  define GPIO_MUX_0               (0 << GPIO_MUX_SHIFT) /* PMR0=0 PMR1=0 */
#  define GPIO_MUX_1               (1 << GPIO_MUX_SHIFT) /* PMR0=1 PMR1=0 */
#  define GPIO_MUX_2               (2 << GPIO_MUX_SHIFT) /* PMR0=0 PMR1=1 */
#  define GPIO_MUX_3               (3 << GPIO_MUX_SHIFT) /* PMR0=1 PMR1=1 */

/* GPIO Enable (1) or Peripheral Enable (0)
 * .... .... .... .... .... ...G .... ....
 */

#define GPIO_ENABLE                (1 << 8)  /* Bit 8:  GPIO enable */
#define GPIO_PERIPH                (0)


/* Port Number
 * .... .... .... .... .... .... PPP. ....
 */

#define GPIO_PORT_SHIFT            (5)       /* Bits 5-7: Port number */
#define GPIO_PORT_MASK             (7 << GPIO_PORT_SHIFT)

/* Pin number:
 * .... .... .... .... .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT             (0)       /* Bits 0-4: Port number */
#define GPIO_PIN_MASK              (0x1f << GPIO_PIN_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: up_clkinit
 *
 * Description:
 *   Initialiaze clock/PLL settings per the definitions in the board.h file.
 *
 ************************************************************************************/

EXTERN void up_clkinitialize(void);

/******************************************************************************
 * Name: usart_reset
 *
 * Description:
 *   Reset a USART.
 *
 ******************************************************************************/

EXTERN void usart_reset(uintptr_t usart_base);

/******************************************************************************
 * Name: usart_configure
 *
 * Description:
 *   Configure a USART as a RS-232 UART.
 *
 ******************************************************************************/

void usart_configure(uintptr_t usart_base, uint32_t baud, unsigned int parity,
                     unsigned int nbits, bool stop2);

/************************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the intialization sequence to configure the serial console uart
 *   (only).
 *
 ************************************************************************************/

EXTERN void up_consoleinit(void);

/************************************************************************************
 * Name: up_boardinit
 *
 * Description:
 *   This function must be provided by the board-specific logic in the directory
 *   configs/<board-name>/up_boot.c.
 *
 ************************************************************************************/

EXTERN void up_boardinitialize(void);

/************************************************************************************
 * Name: at91uc3_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

EXTERN int at91uc3_configgpio(uint32_t cfgset);

/************************************************************************************
 * Name: at91uc3_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

EXTERN void at91uc3_gpiowrite(uint32_t pinset, bool value);

/************************************************************************************
 * Name: at91uc3_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

EXTERN bool at91uc3_gpioread(uint32_t pinset);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVR32_AT91UC3_INTERNAL_H */

