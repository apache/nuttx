/************************************************************************************
 * configs/sama5d4-ek/include/board.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __CONFIGS_SAMA5D4_EK_INCLUDE_BOARD_H
#define __CONFIGS_SAMA5D4_EK_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* After power-on reset, the SAMA5 device is running on a 12MHz internal RC.  These
 * definitions will configure operational clocking.
 */

#if defined(CONFIG_SAMA5_BOOT_SDRAM)
/* When booting from SDRAM, NuttX is loaded in SDRAM by an intermediate bootloader.
 * That bootloader had to have already configured the PLL and SDRAM for proper
 * operation.
 *
 * In this case, we don not reconfigure the clocking.  Rather, we need to query
 * the register settings to determine the clock frequencies.  We can only assume that
 * the Main clock source in the on-board 12MHz crystal.
 */

#  include <arch/board/board_sdram.h>

#elif defined(CONFIG_SAMA5D4EK_384MHZ)
/* OHCI Only.  This is an alternative slower configuration that will produce a 48MHz
 * USB clock with the required accuracy using only PLLA.  When PPLA is used to clock
 * OHCI, an additional requirement is the PLLACK be a multiple of 48MHz.  This setup
 * results in a CPU clock of 384MHz.
 *
 * This case is only interesting for experimentation.
 */

#  include <arch/board/board_384mhz.h>

#elif defined(CONFIG_SAMA5D4EK_528MHZ)
/* This is the configuration results in a CPU clock of 528MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_528mhz.h>

#else /* #elif defined(CONFIG_SAMA5D4EK_396MHZ) */
/* This is the configuration provided in the Atmel example code.  This setup results
 * in a CPU clock of 396MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_396mhz.h>

#endif

/* LED definitions ******************************************************************/
/* There are 3 LEDs on the SAMA5D4-EK:
 *
 * ------------------------------ ------------------- -------------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -------------------------
 * PE28/NWAIT/RTS4/A19            1Wire_PE28          1-WIRE ROM, LCD, D8 (green)
 * PE8/A8/TCLK3/PWML3             LED_USER_PE8        LED_USER (D10)
 * PE9/A9/TIOA2                   LED_POWER_PE9       LED_POWER (D9, Red)
 * ------------------------------ ------------------- -------------------------
 *
 * - D8: D8 is shared with other functions and cannot be used if the 1-Wire ROM
 *   is used.  I am not sure of the LCD function, but the LED may not be available
 *   if the LCD is used either.  We will avoid using D8 just for simplicity.
 * - D10:  Nothing special here.  A low output illuminates.
 * - D9: The Power ON LED.  Connects to the via an IRLML2502 MOSFET.  This LED will
 *   be on when power is applied but otherwise a low output value will turn it
 *   off.
 */

/* LED index values for use with sam_setled() */

#define BOARD_USER        0
#define BOARD_POWER       1
#define BOARD_NLEDS       2

/* LED bits for use with sam_setleds() */

#define BOARD_USER_BIT    (1 << BOARD_BLUE)
#define BOARD_POWER_BIT   (1 << BOARD_RED)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *      SYMBOL            Val    Meaning                     LED state
 *                                                         Blue     Red
 *      ----------------- ---   -----------------------  -------- --------   */
#define LED_STARTED       0  /* NuttX has been started     OFF      OFF      */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated    OFF      OFF      */
#define LED_IRQSENABLED   0  /* Interrupts enabled         OFF      OFF      */
#define LED_STACKCREATED  1  /* Idle stack created         ON       OFF      */
#define LED_INIRQ         2  /* In an interrupt              No change       */
#define LED_SIGNAL        2  /* In a signal handler          No change       */
#define LED_ASSERTION     2  /* An assertion failed          No change       */
#define LED_PANIC         3  /* The system has crashed     OFF      Blinking */
#undef  LED_IDLE             /* MCU is is sleep mode         Not used        */

/* Thus if the D0 and D9 are statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the red D9 LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions ***************************************************************/
/* A single button, PB_USER1 (PB2), is available on the SAMA5D4-EK:
 *
 * ------------------------------ ------------------- -------------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -------------------------
 * PE13/A13/TIOB1/PWML2           PB_USER1_PE13       PB_USER1
 * ------------------------------ ------------------- -------------------------
 *
 * Closing JP2 will bring PE13 to ground so 1) PE13 should have a weak pull-up,
 * and 2) when PB2 is pressed, a low value will be senses.
 */

#define BUTTON_USER       0
#define NUM_BUTTONS       1

#define BUTTON_USER_BIT   (1 << BUTTON_USER)

/* NAND *****************************************************************************/

/* Address for transferring command bytes to the nandflash, CLE A22*/

#define BOARD_EBICS3_NAND_CMDADDR   0x60400000

/* Address for transferring address bytes to the nandflash, ALE A21*/

#define BOARD_EBICS3_NAND_ADDRADDR  0x60200000

/* Address for transferring data bytes to the nandflash.*/

#define BOARD_EBICS3_NAND_DATAADDR  0x60000000

/* PIO configuration ****************************************************************/

/************************************************************************************
 * Assembly Language Macros
 ************************************************************************************/

#ifdef __ASSEMBLY__
	.macro	config_sdram
	.endm
#endif /* __ASSEMBLY__ */

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAMA5 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void sam_boardinitialize(void);

/************************************************************************************
 * Name: sam_phyirq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will be
 *   called when an interrupt is received from a PHY.
 *
 ************************************************************************************/

#if defined(CONFIG_NET) && (defined(CONFIG_SAMA5_EMAC0) || defined(CONFIG_SAMA5_GMAC)) && \
    defined(CONFIG_SAMA5_PIOE_IRQ)
xcpt_t sam_phyirq(int intf, xcpt_t irqhandler);
#endif

/************************************************************************************
 * Name:  sam_ledinit, sam_setled, and sam_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfaces are available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void sam_ledinit(void);
void sam_setled(int led, bool ledon);
void sam_setleds(uint8_t ledset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* !__ASSEMBLY__ */
#endif  /* __CONFIGS_SAMA5D4_EK_INCLUDE_BOARD_H */
