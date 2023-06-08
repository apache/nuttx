/****************************************************************************
 * include/nuttx/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* This header file contains function prototypes for the interfaces between
 * (1) the nuttx core-code, (2) the microprocessor specific logic that
 * resides under the arch/ sub-directory, and (3) the board-specific logic
 * that resides under boards/
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
 *    These definitions are retained in the header file
 *    nuttx/include/arch.h
 *
 *    NOTE: up_ is supposed to stand for microprocessor; the u is like the
 *    Greek letter micron: µ. So it would be µP which is a common
 *    shortening of the word microprocessor.
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
 *    There is also a boards/<arch>/<chip>/<board>/include/board.h header
 *    file that can be used to communicate other board-specific information
 *    between the architecture logic and even application logic.  Any
 *    definitions that are common between a single architecture and several
 *    boards should go in this board.h header file; this file is reserved
 *    for board-related definitions common to all architectures.
 *
 * 4. Board-Specific Interfaces.
 *
 *    Any interface that is unique to a board should be prefixed with
 *    the board name, for example stm32f4discovery_. Sometimes the board
 *    name is too long so stm32_ would be okay too. These should be
 *    prototyped in boards/<arch>/<chip>/<board>/src/<board>.h and should
 *    not be used outside of that board directory since board-specific
 *    definitions have no meaning outside of the board directory.
 */

#ifndef __INCLUDE_NUTTX_BOARD_H
#define __INCLUDE_NUTTX_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/compiler.h>

#ifdef CONFIG_ARCH_IRQBUTTONS
#  include <nuttx/irq.h>
#endif

#ifdef CONFIG_BOARDCTL_RESET_CAUSE
#  include <sys/boardctl.h>
#endif

/****************************************************************************
 * Public Function Prototypes
 *
 * These are all standard board interfaces that are exported from board-
 * specific logic to OS internal logic.  These should never be accessed
 * directly from application code but may be freely used within the OS
 *
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: board_early_initialize
 *
 * Description:
 *   If CONFIG_BOARD_EARLY_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_early_initialize().  board_early_initialize()
 *   will be called immediately after up_initialize() and well before
 *   board_early_initialize() is called and the initial application is
 *   started.  The context in which board_early_initialize() executes is
 *   suitable for early initialization of most, simple device drivers and
 *   is a logical, board-specific extension of up_initialize().
 *
 *   board_early_initialize() runs on the startup, initialization thread.
 *   Some initialization operations cannot be performed on the start-up,
 *   initialization thread.  That is because the initialization thread
 *   cannot wait for event.  Waiting may be required, for example, to
 *   mount a file system or or initialize a device such as an SD card.
 *   For this reason, such driver initialize must be deferred to
 *   board_late_initialize().

 ****************************************************************************/

#ifdef CONFIG_BOARD_EARLY_INITIALIZE
void board_early_initialize(void);
#endif

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called after up_initialize() and board_early_initialize() and just
 *   before the initial application is started.  This additional
 *   initialization phase may be used, for example, to initialize board-
 *   specific device drivers for which board_early_initialize() is not
 *   suitable.
 *
 *   Waiting for events, use of I2C, SPI, etc are permissible in the context
 *   of board_late_initialize().  That is because board_late_initialize()
 *   will run on a temporary, internal kernel thread.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void);
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg);

/****************************************************************************
 * Name: board_app_finalinitialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command
 *   BOARDIOC_FINALINIT.
 *
 * Input Parameters:
 *   arg - The argument has no meaning.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_FINALINIT
int board_app_finalinitialize(uintptr_t arg);
#endif

/****************************************************************************
 * Name: board_power_off
 *
 * Description:
 *   Power off the board.  This function may or may not be supported by a
 *   particular board architecture.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *     meaning of this status information is board-specific.  If not used by
 *     a board, the value zero may be provided in calls to board_power_off.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_POWEROFF
int board_power_off(int status);
#endif

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  Support for this function is required by board-level
 *   logic if CONFIG_BOARDCTL_RESET is selected.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *            meaning of this status information is board-specific.  If not
 *            used by a board, the value zero may be provided in calls to
 *            board_reset().
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET
int board_reset(int status);
#endif

/****************************************************************************
 * Name: board_uniqueid
 *
 * Description:
 *   Return a unique ID associated with the board.  The meaning of this
 *   unique ID is not specified.  It may be a chip identifying number, a
 *   serial number, a MAC address, etc.  It may be in binary or it may be
 *   ASCII.  The only only requirement is that the length of the unique
 *   ID be exactly CONFIG_BOARDCTL_UNIQUEID_SIZE in length.
 *
 * Input Parameters:
 *   uniqueid - A reference to a writable memory location provided by the
 *     caller to receive the board unique ID.  The memory memory referenced
 *     by this pointer must be at least CONFIG_BOARDCTL_UNIQUEID_SIZE in
 *     length.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_UNIQUEID
int board_uniqueid(FAR uint8_t *uniqueid);
#endif

/****************************************************************************
 * Name: board_uniquekey
 *
 * Description:
 *   Return a unique KEY associated with the board.  The meaning of this
 *   unique KEY is not specified.  It may be a trusted key or a private
 *   identity, etc.  The only requirement is that the length of the
 *   unique KEY be exactly CONFIG_BOARDCTL_UNIQUEKEY_SIZE in length.
 *
 * Input Parameters:
 *   uniquekey - A reference to a writable memory location provided by the
 *     caller to receive the board unique KEY.  The memory memory referenced
 *     by this pointer must be at least CONFIG_BOARDCTL_UNIQUEKEY_SIZE in
 *     length.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_UNIQUEKEY
int board_uniquekey(FAR uint8_t *uniquekey);
#endif

/****************************************************************************
 * Name:  board_switch_boot
 *
 * Description:
 *   BOARDIOC_SWITCH_BOOT is required to communicate the boot partition from
 *   userspace (OTA subsystem) to board, it can be used to change the system
 *   boot behavior. It's useful for A/B boot or even in the single boot case.
 *
 * Input Parameters:
 *   system - The boot system updated or specified
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_SWITCH_BOOT
int board_switch_boot(FAR const char *system);
#endif

/****************************************************************************
 * Name:  board_boot_image
 *
 * Description:
 *   Boot a new application firmware image. Execute the required actions for
 *   booting a new application firmware image (e.g. deinitialize peripherals,
 *   load the Program Counter register with the application firmware image
 *   entry point address).
 *
 * Input Parameters:
 *   path     - Path to the new application firmware image to be booted.
 *   hdr_size - Image header size in bytes. This value may be useful for
 *              skipping metadata information preprended to the application
 *              image.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to load the
 *   application firmware image due to some constraints. The return value in
 *   this case is a board-specific reason for the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_BOOT_IMAGE
int board_boot_image(FAR const char *path, uint32_t hdr_size);
#endif

/****************************************************************************
 * Name:  board_timerhook
 *
 * Description:
 *   If the system is not configured for Tickless operation, then a system
 *   timer interrupt will be used.  If CONFIG_SYSTEMTICK_HOOK is selected
 *   then the OS will call out to this user-provided function on every
 *   timer interrupt.  This permits custom actions that may be performed on
 *   each by boad-specific, OS internal logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSTEMTICK_HOOK
void board_timerhook(void);
#endif

/****************************************************************************
 * Name:  board_<usbdev>_initialize
 *
 * Description:
 *   Initialize the USB device <usbdev> on the specified USB device port.
 *
 * Input Parameters:
 *   port- The USB device port.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_USBDEVCTRL
#ifdef CONFIG_CDCACM
#endif

#ifdef CONFIG_USBMSC
int board_usbmsc_initialize(int port);
#endif

#ifdef CONFIG_USBDEV_COMPOSITE
int board_composite_initialize(int port);
#endif
#endif

/****************************************************************************
 * Name:  board_composite_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port using
 *   the specified configuration.  The interpretation of the configid is
 *   board specific.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *   configid - The USB composite configuration
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)
FAR void *board_composite_connect(int port, int configid);
#endif

/****************************************************************************
 * Name:  board_usbdev_serialstr
 *
 * Description:
 *   Use board unique serial number string to iSerialNumber field in the
 *   device descriptor. This is for determining the board when multiple
 *   boards on the same host.
 *
 * Returned Value:
 *   The board unique serial number string.
 *
 ****************************************************************************/

#if defined(CONFIG_BOARD_USBDEV_SERIALSTR)
FAR const char *board_usbdev_serialstr(void);
#endif

/****************************************************************************
 * Name: board_graphics_setup
 *
 * Description:
 *   If the driver for the graphics device on the platform some unusual
 *   initialization, then this board interface should be provided.
 *
 *   This is an internal OS interface. It is invoked by graphics sub-system
 *   initialization logic (nxmu_start()) or from the LCD framebuffer driver
 *   (when the NX server is not used).
 *
 ****************************************************************************/

#if defined(CONFIG_NX_LCDDRIVER) || defined(CONFIG_LCD_FRAMEBUFFER)
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
 *   If CONFIG_BOARDCTL=y, boards may also select CONFIG_BOARDCTL_IOCTL=y
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
 *  Alternatively, board_graphics_setup() may be used if external graphics
 *  initialization is configured.
 *
 ****************************************************************************/

#ifdef CONFIG_LCD
struct lcd_dev_s; /* Forward reference */

int board_lcd_initialize(void);
FAR struct lcd_dev_s *board_lcd_getdev(int lcddev);
void board_lcd_uninitialize(void);
#endif

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   This function is called very early in initialization to perform board-
 *   specific initialization of LED-related resources.  This includes such
 *   things as, for example, configure GPIO pins to drive the LEDs and also
 *   putting the LEDs in their correct initial state.
 *
 *   NOTE: In most architectures, board_autoled_initialize() is called from
 *   board-specific initialization logic.  But there are a few architectures
 *   where this initialization function is still called from common chip
 *   architecture logic.  This interface is not, however, a common board
 *   interface in any event and, hence, the usage of the name
 *   board_autoled_initialize is deprecated.
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
void board_autoled_initialize(void);
#else
#  define board_autoled_initialize()
#endif

/****************************************************************************
 * Name: board_autoled_on
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
void board_autoled_on(int led);
#else
#  define board_autoled_on(led)
#endif

/****************************************************************************
 * Name: board_autoled_off
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
void board_autoled_off(int led);
#else
#  define board_autoled_off(led)
#endif

/****************************************************************************
 * Name:  board_userled_initialize
 *
 * Description:
 *   This function may called from application-specific logic during its
 *   to perform board-specific initialization of LED resources.  This
 *   includes such things as, for example, configure GPIO pins to drive the
 *   LEDs and also putting the LEDs in their correct initial state.
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then this interfaces may be
 *   available to control the LEDs directly from user board logic or
 *   indirectly user applications (via the common LED character driver).
 *
 *   Most boards have only a few LEDs and in those cases all LEDs may be
 *   used by the NuttX LED logic exclusively and may not be available for
 *   use by user logic if CONFIG_ARCH_LEDS=y.
 *
 *   NOTE: The LED number is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_LEDS
uint32_t board_userled_initialize(void);
#endif

/****************************************************************************
 * Name:  board_userled
 *
 * Description:
 *   This interface may be used by application specific logic to set the
 *   state of a single LED.  Definitions for the led identification are
 *   provided in the board-specific board.h header file that may be included
 *   like:
 *
 *     #included <arch/board/board.h>
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then this interfaces may be
 *   available to control the LEDs directly from user board logic or
 *   indirectly user applications (via the common LED character driver).
 *
 *   Most boards have only a few LEDs and in those cases all LEDs may be
 *   used by the NuttX LED logic exclusively and may not be available for
 *   use by user logic if CONFIG_ARCH_LEDS=y.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_LEDS
void board_userled(int led, bool ledon);
#endif

/****************************************************************************
 * Name:  board_userled_all
 *
 * Description:
 *   This interface may be used by application specific logic to set the
 *   state of all board LED.  Definitions for the led set member
 *   identification is provided in the board-specific board.h header file
 *   that may be includedlike:
 *
 *     #included <arch/board/board.h>
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then this interfaces may be
 *   available to control the LEDs directly from user board logic or
 *   indirectly user applications (via the common LED character driver).
 *
 *   Most boards have only a few LEDs and in those cases all LEDs may be
 *   used by the NuttX LED logic exclusively and may not be available for
 *   use by user logic if CONFIG_ARCH_LEDS=y.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_LEDS
void board_userled_all(uint32_t ledset);
#endif

/****************************************************************************
 * Name:  board_userled_getall
 *
 * Description:
 *   This interface may be used by application specific logic to read the
 *   state of all board LEDs.  Definitions for the led set member
 *   identification is provided in the board-specific board.h header file
 *   that may be included like:
 *
 *     #included <arch/board/board.h>
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then this interfaces may be
 *   available to check the LEDs directly from user board logic or indirectly
 *   user applications (via the common LED character driver).
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_HAVE_LEDS) && defined(CONFIG_USERLED_LOWER_READSTATE)
void board_userled_getall(uint32_t *ledset);
#endif

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic. If the board supports button interfaces, then CONFIG_ARCH_BUTTONS
 *   will be defined.
 *   NOTE: The button number is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
uint32_t board_button_initialize(void);
#endif

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  A bit set to
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
uint32_t board_buttons(void);
#endif

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource.
 *
 *   NOTE: This interface may or may not be supported by board-specific
 *   logic.  If the board supports any button interfaces, then
 *   CONFIG_ARCH_BUTTONS will be defined; If the board supports interrupt
 *   buttons, then CONFIG_ARCH_IRQBUTTONS will also be defined.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg);
#endif

/****************************************************************************
 * Name: board_crashdump
 *
 * Description:
 *   If CONFIG_BOARD_CRASHDUMP is selected then up_asseert will call out to
 *   board_crashdump prior to calling exit in the case of an assertion
 *   failure. Or in the case of a hardfault looping indefinitely.
 *   board_crashdump then has a chance to save the state of the machine.
 *   The provided board_crashdump should save as much information as it can
 *   about the cause of the fault and then most likely reset the system.
 *
 *   N.B. There are limited system resources that can be used by the provided
 *   board_crashdump. The tems from the fact that most critical/fatal
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
struct tcb_s;
void board_crashdump(uintptr_t sp, FAR struct tcb_s *tcb,
                     FAR const char *filename, int lineno,
                     FAR const char *msg, FAR void *regs);
#endif

/****************************************************************************
 * Name: board_init_rngseed
 *
 * Description:
 *   If CONFIG_BOARD_INITRNGSEED is selected then board_init_rngseed is
 *   called at up_randompool_initialize() to feed initial random seed
 *   to RNG. Implementation of this functions should feed at least
 *   MIN_SEED_NEW_ENTROPY_WORDS 32-bit random words to entropy-pool using
 *   up_rngaddentropy() or up_rngaddint().
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITRNGSEED
void board_init_rngseed(void);
#endif

/****************************************************************************
 * Name: board_reset_cause
 *
 * Description:
 *   This interface may be used by application specific logic to get the
 *   cause of last reset. Support for this function is required by
 *   board-level logic if CONFIG_BOARDCTL_RESET is selected.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET_CAUSE
int board_reset_cause(FAR struct boardioc_reset_cause_s *cause);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_BOARD_H */
