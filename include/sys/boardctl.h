/****************************************************************************
 * include/sys/boardctl.h
 *
 *   Copyright (C) 2015-2019 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_BOARDCTL_H
#define __INCLUDE_SYS_BOARDCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_VNCSERVER
#  include <nuttx/nx/nx.h>
#endif

#ifdef CONFIG_NXTERM
#  include <nuttx/nx/nxterm.h>
#endif

#ifdef CONFIG_LIB_BOARDCTL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Common commands
 *
 * CMD:           BOARDIOC_INIT
 * DESCRIPTION:   Perform one-time application initialization.
 * ARG:           The boardctl() argument is passed to the
 *                board_app_initialize() implementation without modification.
 *                The argument has no meaning to NuttX; the meaning of the
 *                argument is a contract between the board-specific
 *                initalization logic and the matching application logic.
 *                The value cold be such things as a mode enumeration value,
 *                a set of DIP switch switch settings, a pointer to
 *                configuration data read from a file or serial FLASH, or
 *                whatever you would like to do with it.  Every
 *                implementation should accept zero/NULL as a default
 *                configuration.
 * CONFIGURATION: CONFIG_LIB_BOARDCTL
 * DEPENDENCIES:  Board logic must provide board_app_initialize()
 *
 * CMD:           BOARDIOC_POWEROFF
 * DESCRIPTION:   Power off the board
 * ARG:           Integer value providing power off status information
 * CONFIGURATION: CONFIG_BOARDCTL_POWEROFF
 * DEPENDENCIES:  Board logic must provide the board_power_off() interface.
 *
 * CMD:           BOARDIOC_RESET
 * DESCRIPTION:   Reset the board
 * ARG:           Integer value providing power off status information
 * CONFIGURATION: CONFIG_BOARDCTL_RESET
 * DEPENDENCIES:  Board logic must provide the board_reset() interface.
 *
 * CMD:           BOARDIOC_UNIQUEID
 * DESCRIPTION:   Return a unique ID associated with the board (such as a
 *                serial number or a MAC address).
 * ARG:           A writable array of size CONFIG_BOARDCTL_UNIQUEID_SIZE in
 *                which to receive the board unique ID.
 * DEPENDENCIES:  Board logic must provide the board_uniqueid() interface.
 *
 * CMD:           BOARDIOC_APP_SYMTAB
 * DESCRIPTION:   Select the application symbol table.  This symbol table
 *                provides the symbol definitions exported to application
 *                code from application space.
 * ARG:           A pointer to an instance of struct boardioc_symtab_s
 * CONFIGURATION: CONFIG_BOARDCTL_APP_SYMTAB
 * DEPENDENCIES:  None
 *
 * CMD:           BOARDIOC_OS_SYMTAB
 * DESCRIPTION:   Select the OS symbol table.  This symbol table provides
 *                the symbol definitions exported by the OS to kernel
 *                modules.
 * ARG:           A pointer to an instance of struct boardioc_symtab_s
 * CONFIGURATION: CONFIG_BOARDCTL_OS_SYMTAB
 * DEPENDENCIES:  None
 *
 * CMD:           BOARDIOC_USBDEV_CONTROL
 * DESCRIPTION:   Manage USB device classes
 * ARG:           A pointer to an instance of struct boardioc_usbdev_ctrl_s
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_USBDEVCTRL
 * DEPENDENCIES:  Board logic must provide board_<usbdev>_initialize()
 *
 * CMD:           BOARDIOC_NX_START
 * DESCRIPTION:   Start the NX server
 * ARG:           Integer display number to be served by this NXMU instance.
 * CONFIGURATION: CONFIG_NX
 * DEPENDENCIES:  Base graphics logic provides nxmu_start()
 *
 * CMD:           BOARDIOC_VNC_START
 * DESCRIPTION:   Start the NX server and framebuffer driver.
 * ARG:           A reference readable instance of struct
 *                boardioc_vncstart_s
 * CONFIGURATION: CONFIG_VNCSERVER
 * DEPENDENCIES:  VNC server provides vnc_default_fbinitialize()
 *
 * CMD:           BOARDIOC_NXTERM
 * DESCRIPTION:   Create an NX terminal device
 * ARG:           A reference readable/writable instance of struct
 *                boardioc_nxterm_create_s
 * CONFIGURATION: CONFIG_NXTERM
 * DEPENDENCIES:  Base NX terminal logic provides nx_register() and
 *                nxtk_register()
 *
 * CMD:           BOARDIOC_NXTERM_REDRAW
 * DESCRIPTION:   Re-draw a portion of the NX console.  This function
 *                should be called from the appropriate window callback
 *                logic.
 * ARG:           A reference readable instance of struct
 *                boardioc_nxterm_redraw_s
 * CONFIGURATION: CONFIG_NXTERM
 * DEPENDENCIES:  Base NX terminal logic provides nxterm_redraw()
 *
 * CMD:           BOARDIOC_NXTERM_KBDIN
 * DESCRIPTION:   Provide NxTerm keyboard input to NX.
 * ARG:           A reference readable instance of struct
 *                boardioc_nxterm_kbdin_s
 * CONFIGURATION: CONFIG_NXTERM_NXKBDIN
 * DEPENDENCIES:  Base NX terminal logic provides nxterm_kbdin()
 *
 * CMD:           BOARDIOC_TESTSET
 * DESCRIPTION:   Access architecture-specific up_testset() operation
 * ARG:           A pointer to a write-able spinlock object.  On success
 *                the  preceding spinlock state is returned:  0=unlocked,
 *                1=locked.
 * CONFIGURATION: CONFIG_BOARDCTL_TESTSET
 * DEPENDENCIES:  Architecture-specific logic provides up_testset()
 */

#define BOARDIOC_INIT              _BOARDIOC(0x0001)
#define BOARDIOC_FINALINIT         _BOARDIOC(0x0002)
#define BOARDIOC_POWEROFF          _BOARDIOC(0x0003)
#define BOARDIOC_RESET             _BOARDIOC(0x0004)
#define BOARDIOC_UNIQUEID          _BOARDIOC(0x0005)
#define BOARDIOC_APP_SYMTAB        _BOARDIOC(0x0006)
#define BOARDIOC_OS_SYMTAB         _BOARDIOC(0x0007)
#define BOARDIOC_USBDEV_CONTROL    _BOARDIOC(0x0008)
#define BOARDIOC_NX_START          _BOARDIOC(0x0009)
#define BOARDIOC_VNC_START         _BOARDIOC(0x000a)
#define BOARDIOC_NXTERM            _BOARDIOC(0x000b)
#define BOARDIOC_NXTERM_REDRAW     _BOARDIOC(0x000c)
#define BOARDIOC_NXTERM_KBDIN      _BOARDIOC(0x000d)
#define BOARDIOC_TESTSET           _BOARDIOC(0x000e)

/* If CONFIG_BOARDCTL_IOCTL=y, then board-specific commands will be support.
 * In this case, all commands not recognized by boardctl() will be forwarded
 * to the board-provided board_ioctl() function.
 *
 * User defined board commands may begin with this value:
 */

#define BOARDIOC_USER              _BOARDIOC(0x000f)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Structures used with IOCTL commands */

/* In order to full describe a symbol table, a vector containing the address
 * of the symbol table and the number of elements in the symbol table is
 * required.
 */

struct symtab_s; /* Forward reference */
struct boardioc_symtab_s
{
  FAR struct symtab_s *symtab;
  int nsymbols;
};

#ifdef CONFIG_BOARDCTL_USBDEVCTRL
/* This structure provides the argument BOARDIOC_USBDEV_CONTROL and
 * describes which device should be controlled and what should be
 * done.
 *
 * enum boardioc_usbdev_identifier_e: Identifies the USB device class.
 *   In the case of multiple instances of the USB device class, the
 *   specific instance is identifed by the 'inst' field of the structure.
 *
 * enum boardioc_usbdev_action_e: Identifies the action to peform on
 *   the USB device class instance.
 *
 * struct boardioc_usbdev_ctrl_s:
 *   - usbdev: A value from enum boardioc_usbdev_identifier_e that
 *     identifies the USB device class.
 *   - action: The action to be performed on the USB device class.
 *   - instance:  If there are multiple instances of the USB device
 *     class, this identifies the particular instance.  This is normally
 *     zero but could be non-zero if there are multiple USB device ports
 *     supported by the board.  For CDC/ACM, this is the /dev/ttyACM minor
 *     number; For other devices this would be a port number.
 *   - handle:  This value is returned by the BOARDIOC_USBDEV_CONNECT
 *     action and must be provided to the BOARDIOC_USBDEV_DISCONNECT
 *     action.  It is not used with the BOARDIOC_USBDEV_INITIALIZE action.
 */

enum boardioc_usbdev_identifier_e
{
  BOARDIOC_USBDEV_NONE = 0        /* Not valid */
#ifdef CONFIG_CDCACM
  , BOARDIOC_USBDEV_CDCACM        /* CDC/ACM */
#endif
#ifdef CONFIG_PL2303
  , BOARDIOC_USBDEV_PL2303        /* PL2303 serial */
#endif
#ifdef CONFIG_USBMSC
  , BOARDIOC_USBDEV_MSC           /* Mass storage class */
#endif
#ifdef CONFIG_USBDEV_COMPOSITE
  , BOARDIOC_USBDEV_COMPOSITE     /* Composite device */
#endif
};

enum boardioc_usbdev_action_e
{
  BOARDIOC_USBDEV_INITIALIZE = 0, /* Initialize USB device */
  BOARDIOC_USBDEV_CONNECT,        /* Connect the USB device */
  BOARDIOC_USBDEV_DISCONNECT,     /* Disconnect the USB device */
};

struct boardioc_usbdev_ctrl_s
{
  uint8_t usbdev;                 /* See enum boardioc_usbdev_identifier_e */
  uint8_t action;                 /* See enum boardioc_usbdev_action_e */
  uint8_t instance;               /* Identifies the USB device class instance */
  uint8_t config;                 /* Configuration used with BOARDIOC_USBDEV_CONNECT */
  FAR void **handle;              /* Connection handle */
};
#endif /* CONFIG_BOARDCTL_USBDEVCTRL */

#ifdef CONFIG_VNCSERVER
/* Argument passed with the BOARDIOC_VNC_START command */

struct boardioc_vncstart_s
{
  int display;                    /* Display number */
  NXHANDLE handle;                /* Handle returned by nx_connect */
};
#endif

#ifdef CONFIG_NXTERM
/* Arguments passed with the BOARDIOC_NXTERM command */

enum boardioc_termtype_e
{
  BOARDIOC_XTERM_RAW = 0,         /* Raw NX terminal window */
  BOARDIOC_XTERM_FRAMED,          /* Framed NxTK terminal window */
  BOARDIOC_XTERM_TOOLBAR          /* Tooolbar of framed NxTK terminal window */
};

struct boardioc_nxterm_create_s
{
  NXTERM nxterm;                  /* Returned NXTERM handle */
  FAR void *hwnd;                 /* Window handle (NXWINDOW or NXTKWINDOW). */
  struct nxterm_window_s wndo;    /* Describes the initial window: color, size, font */
  enum boardioc_termtype_e type;  /* Terminal window type  */
  uint8_t minor;                  /* Terminal device minor number, N, in
                                   * /dev/nxtermN.  0 <= N <= 255 */
};

/* Arguments passed with the BOARDIOC_NXTERM_REDRAW command */

struct boardioc_nxterm_redraw_s
{
  NXTERM handle;                            /* NxTerm handle */
  struct nxgl_rect_s rect;                  /* Rectangle to be re-drawn */
  bool more;                                /* True: More redraw commands follow */
};

#ifdef CONFIG_NXTERM_NXKBDIN
/* Arguments passed with the BOARDIOC_NXTERM_KBDIN command */

struct boardioc_nxterm_kbdin_s
{
  NXTERM handle;                            /* NxTerm handle */
  FAR const uint8_t *buffer;                /* Buffered keyboard data */
  uint8_t buflen;                           /* Amount of data in buffer */
};
#endif

#endif /* CONFIG_NXTERM */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: boardctl
 *
 * Description:
 *   In a small embedded system, there will typically be a much greater
 *   interaction between application and low-level board features.  The
 *   canonically correct to implement such interactions is by implementing a
 *   character driver and performing the interactions via low level ioctl
 *   calls.  This, however, may not be practical in many cases and will lead
 *   to "correct" but awkward implementations.
 *
 *   boardctl() is non-standard OS interface to alleviate the problem.  It
 *   basically circumvents the normal device driver ioctl interlace and allows
 *   the application to perform direct IOCTL-like calls to the board-specific
 *   logic.  It is especially useful for setting up board operational and
 *   test configurations.
 *
 * Input Parameters:
 *   cmd - Identifies the board command to be executed
 *   arg - The argument that accompanies the command.  The nature of the
 *         argument is determined by the specific command.
 *
 * Returned Value:
 *   On success zero (OK) is returned; -1 (ERROR) is returned on failure
 *   with the errno variable to to indicate the nature of the failure.
 *
 ****************************************************************************/

int boardctl(unsigned int cmd, uintptr_t arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_LIB_BOARDCTL */
#endif /* __INCLUDE_SYS_BOARDCTL_H */
