/****************************************************************************
 * include/nuttx/board.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/
/* This header file contains function prototypes for the interfaces between
 * (1) the nuttx core-code, (2) the microprocessor specific logic that
 * resides under the arch/ sub-directory, and (3) the board-specific logic
 * that resides under configs/
 *
 * Naming conventions:
 *
 * 1. Common Microprocessor Interfaces.
 *
 *    Any interface that is common across all microprocessors should be
 *    prefixed with up_ and prototyped in this header file. These
 *    definitions provide the common interface between NuttX and the
 *    architecture-specific implementation in arch/
 *
 *    These definitions are retained in the the header file nuttx/include/arch.h
 *
 *    NOTE: up_ is supposed to stand for microprocessor; the u is like the
 *    Greek letter micron: µ. So it would be µP which is a common shortening
 *    of the word microprocessor.
 *
 * 2. Microprocessor-Specific Interfaces.
 *
 *    An interface which is unique to a certain microprocessor should be
 *    prefixed with the name of the microprocessor, for example stm32_,
 *    and be prototyped in some header file in the arch/ directories.
 *
 *    There is also a arch/<architecture>/include/<chip>/chip.h header file
 *    that can be used to communicate other microprocessor-specific
 *    information between the board logic and even application logic.
 *    Application logic may, for example, need to know specific capabilities
 *    of the chip.  Prototypes in that chip.h header file should follow the
 *    microprocessor specific naming convention.
 *
 * 3. Common Board Interfaces.
 *
 *    Any interface that is common across all boards should be prefixed
 *    with board_ and should be prototyped in this header file. These
 *    board_ definitions provide the interface between the board-level
 *    logic and the architecture-specific logic.
 *
 *    Board related declarations are retained in this header file.
 *
 *    There is also a configs/<board>/include/board.h header file that
 *    can be used to communicate other board-specific information between
 *    the architecture logic and even application logic.  Any definitions
 *    which are common between a single architecture and several boards
 *    should go in this board.h header file; this file is reserved for
 *    board-related definitions common to all architectures.
 *
 * 4. Board-Specific Interfaces.
 *
 *    Any interface which is unique to a board should be prefixed with
 *    the board name, for example stm32f4discovery_. Sometimes the board
 *    name is too long so stm32_ would be okay too. These should be
 *    prototyped in configs/<board>/src/<board>.h and should not be used
 *    outside of that board directory since board-specific definitions
 *    have no meaning outside of the board directory.
 */

#ifndef __INCLUDE_NUTTX_BOARD_H
#define __INCLUDE_NUTTX_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/compiler.h>

#ifdef CONFIG_ARCH_IRQBUTTONS
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Public Function Prototypes
 *
 * These are all standard board interfaces that are exported from board-
 * specific logic to OS internal logic.  These should never be accessed
 * directly from application code but may be freely used within the OS
 *
 ****************************************************************************/

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void);
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 *****************************************************************************/

#ifdef CONFIG_LIB_BOARDCTL
int board_app_initialize(void);
#endif /* CONFIG_LIB_BOARDCTL */

/****************************************************************************
 * Name: board_tsc_setup
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   configure the touchscreen device.  This function will register the driver
 *   as /dev/inputN where N is the minor device number.
 *
 *   This is an internal OS interface but may be invoked indirectly from
 *   application-level touchscreen testing logic (perhaps by
 *   apps/examples/touchscreen).  If CONFIG_LIB_BOARDCTL=y and
 *   CONFIG_BOARDCTL_TSCTEST=y, then this functions will be invoked via the
 *   (non-standard) boardctl() interface using the commands
 *   BOARDIOC_TSCTEST_SETUP command.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_tsc_setup(int minor);

/****************************************************************************
 * Name: board_tsc_teardown
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   uninitialize the touchscreen device.
 *
 *   This is an internal OS interface but may be invoked indirectly from
 *   application-level touchscreen testing logic (perhaps by
 *   apps/examples/touchscreen).  If CONFIG_LIB_BOARDCTL=y and
 *   CONFIG_BOARDCTL_TSCTEST=y, then this functions will be invoked via the
 *   (non-standard) boardctl() interface using the commands
 *   BOARDIOC_TSCTEST_TEARDOWN command.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void board_tsc_teardown(void);

/****************************************************************************
 * Name: board_adc_setup
 *
 * Description:
 *   All architectures must provide the following interface in order to
 *   work with examples/adc.
 *
 *   This is an internal OS interface but may be invoked indirectly from
 *   application-level graphics logic.  If CONFIG_LIB_BOARDCTL=y and
 *   CONFIG_BOARDCTL_ADCTEST=y, then this functions will be invoked via the
 *   (non-standard) boardctl() interface using the commands
 *   BOARDIOC_ADCTEST_SETUP command.
 *
 ****************************************************************************/

int board_adc_setup(void);

/****************************************************************************
 * Name: board_pwm_setup
 *
 * Description:
 *   All architectures must provide the following interface in order to
 *   work with examples/pwm.
 *
 *   This is an internal OS interface but may be invoked indirectly from
 *   application-level graphics logic.  If CONFIG_LIB_BOARDCTL=y and
 *   CONFIG_BOARDCTL_PWMTEST=y, then this functions will be invoked via the
 *   (non-standard) boardctl() interface using the commands
 *   BOARDIOC_PWMTEST_SETUP command.
 *
 ****************************************************************************/

int board_pwm_setup(void);

/****************************************************************************
 * Name: board_graphics_setup
 *
 * Description:
 *   If the driver for the graphics device on the platform some unusual
 *   initialization, then this board interface should be provided.
 *
 *   This is an internal OS interface but may be invoked indirectly from
 *   application-level graphics logic.  If CONFIG_LIB_BOARDCTL=y and
 *   CONFIG_BOARDCTL_GRAPHICS=y, then this functions will be invoked via the
 *   (non-standard) boardctl() interface using the commands
 *   BOARDIOC_GRAPHICS_SETUP command.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_LCDDRIVER
struct lcd_dev_s;
FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno);
#else
struct fb_vtable_s;
FAR struct fb_vtable_s *board_graphics_setup(unsigned int devno);
#endif

/****************************************************************************
 * Name: board_ioctl
 *
 * Description:
 *   If CONFIG_LIB_BOARDCTL=y, boards may also select CONFIG_BOARDCTL_IOCTL=y
 *   enable board specific commands.  In this case, all commands not
 *   recognized by boardctl() will be forwarded to the board-provided
 *   board_ioctl() function.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg);
#endif

/****************************************************************************
 * Name: board_lcd_initialize, board_lcd_getdev, board_lcd_uninitialize
 *
 * Description:
 *   If an architecture supports a parallel or serial LCD, then it must
 *   provide APIs to access the LCD as follows:
 *
 *   board_lcd_initialize   - Initialize the LCD video hardware.  The initial
 *                            state of the LCD is fully initialized, display
 *                            memory cleared, and the LCD ready to use, but
 *                            with the power setting at 0 (full off).
 *   board_lcd_getdev       - Return a a reference to the LCD object for
 *                            the specified LCD.  This allows support for
 *                            multiple LCD devices.
 *   board_lcd_uninitialize - Uninitialize the LCD support
 *
 ***************************************************************************/

#ifdef CONFIG_LCD
struct lcd_dev_s; /* Forward reference */

int board_lcd_initialize(void);
FAR struct lcd_dev_s *board_lcd_getdev(int lcddev);
void board_lcd_uninitialize(void);
#endif

/****************************************************************************
 * Name: board_led_initialize
 *
 * Description:
 *   This functions is called very early in initialization to perform board-
 *   specific initialization of LED-related resources.  This includes such
 *   things as, for example, configure GPIO pins to drive the LEDs and also
 *   putting the LEDs in their correct initial state.
 *
 *   NOTE: In most architectures, board_led_initialize() is called from
 *   board-specific initialization logic.  But there are a few architectures
 *   where this initialization function is still called from common chip
 *   architecture logic.  This interface is not, however, a common board
 *   interface in any event and, hence, the usage of the name
 *   board_led_initialize is deprecated.
 *
 *   WARNING: This interface name will eventually be removed; do not use it
 *   in new board ports.  New implementations should use the naming
 *   conventions for "Microprocessor-Specific Interfaces" or the "Board-
 *   Specific Interfaces" as described above.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_led_initialize(void);
#else
# define board_led_initialize()
#endif

/****************************************************************************
 * Name: board_led_on
 *
 * Description:
 *   Set the LED configuration into the ON condition for the state provided
 *   by the led parameter.  This may be one of:
 *
 *     LED_STARTED       NuttX has been started
 *     LED_HEAPALLOCATE  Heap has been allocated
 *     LED_IRQSENABLED   Interrupts enabled
 *     LED_STACKCREATED  Idle stack created
 *     LED_INIRQ         In an interrupt
 *     LED_SIGNAL        In a signal handler
 *     LED_ASSERTION     An assertion failed
 *     LED_PANIC         The system has crashed
 *     LED_IDLE          MCU is in sleep mode
 *
 *   Where these values are defined in a board-specific way in the standard
 *   board.h header file exported by every architecture.
 *
 * Input Parameters:
 *   led - Identifies the LED state to put in the ON state (which may or may
 *         not equate to turning an LED on)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_led_on(int led);
#else
# define board_led_on(led)
#endif

/****************************************************************************
 * Name: board_led_off
 *
 * Description:
 *   Set the LED configuration into the OFF condition for the state provided
 *   by the led parameter.  This may be one of:
 *
 *     LED_INIRQ         Leaving an interrupt
 *     LED_SIGNAL        Leaving a signal handler
 *     LED_ASSERTION     Recovering from an assertion failure
 *     LED_PANIC         The system has crashed (blinking).
 *     LED_IDLE          MCU is not in sleep mode
 *
 *   Where these values are defined in a board-specific way in the standard
 *   board.h header file exported by every architecture.
 *
 * Input Parameters:
 *   led - Identifies the LED state to put in the OFF state (which may or may
 *         not equate to turning an LED off)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_led_off(int led);
#else
# define board_led_off(led)
#endif

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state of
 *   all buttons or board_button_irq() may be called to register button interrupt
 *   handlers.
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports button interfaces, then CONFIG_ARCH_BUTTONS
 *   will be defined.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
void board_button_initialize(void);
#endif

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   8-bit bit set with each bit associated with a button.  A bit set to
 *   "1" means that the button is depressed; a bit set to "0" means that
 *   the button is released.  The correspondence of the each button bit
 *   and physical buttons is board-specific.
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports button interfaces, then
 *   CONFIG_ARCH_BUTTONS will be defined
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
uint8_t board_buttons(void);
#endif

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource.
 *   The previous interrupt handler address is returned (so that it may
 *   restored, if so desired).
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports any button interfaces, then
 *   CONFIG_ARCH_BUTTONS will be defined; If the board supports interrupt
 *   buttons, then CONFIG_ARCH_IRQBUTTONS will also be defined.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
xcpt_t board_button_irq(int id, xcpt_t irqhandler);
#endif

/****************************************************************************
 * Name: board_crashdump
 *
 * Description:
 *   If CONFIG_BOARD_CRASHDUMP is selected then up_asseert will call out to
 *   board_crashdump prior to calling exit in the case of an assertion failure.
 *   Or in the case of a hardfault looping indefinitely. board_crashdump then
 *   has a chance to save the state of the machine. The provided
 *   board_crashdump should save as much information as it can about the cause
 *   of the fault and then most likely reset the system.
 *
 *   N.B. There are limited system resources that can be used by the provided
 *   board_crashdump function. The tems from the fact that most critical/fatal
 *   crashes are because of a hard fault or during interrupt processing.
 *   In these cases, up_assert is running from the context of an interrupt
 *   handlerand it is impossible to use any device driver in this context.
 *
 *   Also consider the following: Who knows what state the system is in? Is
 *   memory trashed? Is the Heap intact? Therefore all we can expect to do in
 *   board_crashdump is save the "machine state" in a place where on the next
 *   reset we can write it to more sophisticated storage in a sane operating
 *   environment.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_CRASHDUMP
void board_crashdump(uint32_t currentsp, void *tcb, uint8_t *filename,
                     int lineno);
#endif

#endif /* __INCLUDE_NUTTX_BOARD_H */
