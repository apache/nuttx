/****************************************************************************
 * include/sys/boardctl.h
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

#ifndef __INCLUDE_SYS_BOARDCTL_H
#define __INCLUDE_SYS_BOARDCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_PM
#  include <nuttx/power/pm.h>
#endif

#ifdef CONFIG_VNCSERVER
#  include <nuttx/nx/nx.h>
#endif

#ifdef CONFIG_NXTERM
#  include <nuttx/nx/nxterm.h>
#endif

#ifdef CONFIG_BOARDCTL

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
 *                initialization logic and the matching application logic.
 *                The value could be such things as a mode enumeration value,
 *                a set of DIP switch switch settings, a pointer to
 *                configuration data read from a file or serial FLASH, or
 *                whatever you would like to do with it.  Every
 *                implementation should accept zero/NULL as a default
 *                configuration.
 * CONFIGURATION: CONFIG_BOARDCTL
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
 * CMD:           BOARDIOC_PM_CONTROL
 * DESCRIPTION:   Manage power state transition and query
 * ARG:           A pointer to an instance of struct boardioc_pm_ctrl_s
 * CONFIGURATION: CONFIG_PM
 * DEPENDENCIES:  None
 *
 * CMD:           BOARDIOC_UNIQUEID
 * DESCRIPTION:   Return a unique ID associated with the board (such as a
 *                serial number or a MAC address).
 * ARG:           A writable array of size CONFIG_BOARDCTL_UNIQUEID_SIZE in
 *                which to receive the board unique ID.
 * DEPENDENCIES:  Board logic must provide the board_uniqueid() interface.
 *
 * CMD:           BOARDIOC_MKRD
 * DESCRIPTION:   Create a RAM disk
 * ARG:           Pointer to read-only instance of struct boardioc_mkrd_s.
 * CONFIGURATION: CONFIG_BOARDCTL_MKRD
 * DEPENDENCIES:  None
 *
 * CMD:           BOARDIOC_ROMDISK
 * DESCRIPTION:   Register a ROM disk
 * ARG:           Pointer to read-only instance of struct boardioc_romdisk_s.
 * CONFIGURATION: CONFIG_BOARDCTL_ROMDISK
 * DEPENDENCIES:  None
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
 * CMD:           BOARDIOC_BUILTINS
 * DESCRIPTION:   Provide the user-space list of built-in applications for
 *                use by BINFS in protected mode.  Normally this is small
 *                set of globals provided by user-space logic.  It provides
 *                name-value pairs for associating built-in application
 *                names with user-space entry point addresses.  These
 *                globals are only needed for use by BINFS which executes
 *                built-in applications from kernel-space in PROTECTED mode.
 *                In the FLAT build, the user space globals are readily
 *                available.  (BINFS is not supportable in KERNEL mode since
 *                user-space address have no general meaning that
 *                configuration).
 * ARG:           A pointer to an instance of struct boardioc_builtin_s
 * CONFIGURATION: This BOARDIOC command is always available when
 *                CONFIG_BUILTIN is enabled, but does nothing unless
 *                CONFIG_BUILD_PROTECTED is also selected.
 * DEPENDENCIES:  None
 *
 * CMD:           BOARDIOC_USBDEV_CONTROL
 * DESCRIPTION:   Manage USB device classes
 * ARG:           A pointer to an instance of struct boardioc_usbdev_ctrl_s
 * CONFIGURATION: CONFIG_BOARDCTL && CONFIG_BOARDCTL_USBDEVCTRL
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
 * DEPENDENCIES:  VNC server provides nx_vnc_fbinitialize()
 *
 * CMD:           BOARDIOC_NXTERM
 * DESCRIPTION:   Create an NX terminal device
 * ARG:           A reference readable/writable instance of struct
 *                boardioc_nxterm_create_s
 * CONFIGURATION: CONFIG_NXTERM
 * DEPENDENCIES:  Base NX terminal logic provides nx_register() and
 *                nxtk_register()
 *
 * CMD:           BOARDIOC_NXTERM_IOCTL
 * DESCRIPTION:   Create an NX terminal IOCTL command.  Normal IOCTLs
 *                cannot be be performed in most graphics contexts since
 *                the depend on the task holding an open file descriptor
 * ARG:           A reference readable/writable instance of struct
 *                boardioc_nxterm_ioctl_s
 * CONFIGURATION: CONFIG_NXTERM
 * DEPENDENCIES:  Base NX terminal logic provides nxterm_ioctl_tap()
 *
 * CMD:           BOARDIOC_TESTSET
 * DESCRIPTION:   Access architecture-specific up_testset() operation
 * ARG:           A pointer to a write-able spinlock object.  On success
 *                the  preceding spinlock state is returned:  0=unlocked,
 *                1=locked.
 * CONFIGURATION: CONFIG_BOARDCTL_TESTSET
 * DEPENDENCIES:  Architecture-specific logic provides up_testset()
 *
 * CMD:           BOARDIOC_RESET_CAUSE
 * DESCRIPTION:   Get the cause of last-time board reset
 * ARG:           A pointer to an instance of struct boardioc_reset_cause_s
 * CONFIGURATION: CONFIG_BOARDCTL_RESET_CAUSE
 * DEPENDENCIES:  Board logic must provide the board_reset_cause() interface.
 */

#define BOARDIOC_INIT              _BOARDIOC(0x0001)
#define BOARDIOC_FINALINIT         _BOARDIOC(0x0002)
#define BOARDIOC_POWEROFF          _BOARDIOC(0x0003)
#define BOARDIOC_RESET             _BOARDIOC(0x0004)
#define BOARDIOC_PM_CONTROL        _BOARDIOC(0x0005)
#define BOARDIOC_UNIQUEID          _BOARDIOC(0x0006)
#define BOARDIOC_MKRD              _BOARDIOC(0x0007)
#define BOARDIOC_ROMDISK           _BOARDIOC(0x0008)
#define BOARDIOC_APP_SYMTAB        _BOARDIOC(0x0009)
#define BOARDIOC_OS_SYMTAB         _BOARDIOC(0x000a)
#define BOARDIOC_BUILTINS          _BOARDIOC(0x000b)
#define BOARDIOC_USBDEV_CONTROL    _BOARDIOC(0x000c)
#define BOARDIOC_NX_START          _BOARDIOC(0x000d)
#define BOARDIOC_VNC_START         _BOARDIOC(0x000e)
#define BOARDIOC_NXTERM            _BOARDIOC(0x000f)
#define BOARDIOC_NXTERM_IOCTL      _BOARDIOC(0x0010)
#define BOARDIOC_TESTSET           _BOARDIOC(0x0011)
#define BOARDIOC_UNIQUEKEY         _BOARDIOC(0x0012)
#define BOARDIOC_SWITCH_BOOT       _BOARDIOC(0x0013)
#define BOARDIOC_BOOT_IMAGE        _BOARDIOC(0x0014)
#define BOARDIOC_RESET_CAUSE       _BOARDIOC(0x0015)

/* If CONFIG_BOARDCTL_IOCTL=y, then board-specific commands will be support.
 * In this case, all commands not recognized by boardctl() will be forwarded
 * to the board-provided board_ioctl() function.
 *
 * User defined board commands may begin with this value:
 */

#define BOARDIOC_USER              _BOARDIOC(0x0016)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Structures used with IOCTL commands */

#ifdef CONFIG_PM
/* Arguments passed with the BOARDIOC_PM_CONTROL command */

enum boardioc_action_e
{
  BOARDIOC_PM_ACTIVITY = 0,
  BOARDIOC_PM_STAY,
  BOARDIOC_PM_RELAX,
  BOARDIOC_PM_STAYCOUNT,
  BOARDIOC_PM_QUERYSTATE,
  BOARDIOC_PM_CHANGESTATE,
  BOARDIOC_PM_CHECKSTATE
};

struct boardioc_pm_ctrl_s
{
  uint32_t action;
  uint32_t domain;
  uint32_t state;
  uint32_t count;
  uint32_t priority;
};
#endif

#ifdef CONFIG_BOARDCTL_MKRD
/* Describes the RAM disk to be created */

struct boardioc_mkrd_s
{
  uint8_t minor;      /* Minor device number of the RAM disk. */
  uint32_t nsectors;  /* The number of sectors in the RAM disk */
  uint16_t sectsize;  /* The size of one sector in bytes */
  uint8_t rdflags;    /* See RD_FLAGS_* definitions in include/nuttx/ramdisk.h */
};
#endif

#ifdef CONFIG_BOARDCTL_ROMDISK
/* Describes the ROM disk image to be registered.
 *
 * The image points to a image of the file system in some read-only memory.
 * This image must conform to certain requirements:  (1) it must be
 * accessible to the kernel code with no special address mapping, and (2)
 * it must be virtually contiguous.
 */

struct boardioc_romdisk_s
{
  uint8_t minor;      /* Minor device number of the RAM disk. */
  uint32_t nsectors;  /* The number of sectors in the RAM disk */
  uint16_t sectsize;  /* The size of one sector in bytes */
  FAR const uint8_t *image;
};
#endif

/* In order to full describe a symbol table, a vector containing the address
 * of the symbol table and the number of elements in the symbol table is
 * required.
 */

struct symtab_s;  /* Forward reference */
struct boardioc_symtab_s
{
  FAR struct symtab_s *symtab;
  int nsymbols;
};

#ifdef CONFIG_BUILTIN
/* Arguments passed with the BOARDIOC_BUILTIN command */

struct builtin_s;  /* Forward reference */
struct boardioc_builtin_s
{
  FAR const struct builtin_s *builtins;
  int count;
};
#endif

#ifdef CONFIG_BOARDCTL_USBDEVCTRL
/* This structure provides the argument BOARDIOC_USBDEV_CONTROL and
 * describes which device should be controlled and what should be
 * done.
 *
 * enum boardioc_usbdev_identifier_e: Identifies the USB device class.
 *   In the case of multiple instances of the USB device class, the
 *   specific instance is identified by the 'inst' field of the structure.
 *
 * enum boardioc_usbdev_action_e: Identifies the action to perform on
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
  BOARDIOC_XTERM_TOOLBAR          /* Toolbar of framed NxTK terminal window */
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

struct boardioc_nxterm_ioctl_s
{
  int cmd;                         /* IOCTL command */
  uintptr_t arg;                   /* IOCTL argument */
};
#endif /* CONFIG_NXTERM */

#ifdef CONFIG_BOARDCTL_BOOT_IMAGE

/* Structure containing the arguments to the BOARDIOC_BOOT_IMAGE command */

struct boardioc_boot_info_s
{
  FAR const char *path;           /* Path to application firmware image */
  uint32_t        header_size;    /* Size of the image header in bytes */
};
#endif

#ifdef CONFIG_BOARDCTL_RESET_CAUSE
/* Describes the reason of last reset */

enum boardioc_reset_cause_e
{
  BOARDIOC_RESETCAUSE_NONE = 0,
  BOARDIOC_RESETCAUSE_SYS_CHIPPOR,      /* chip power on */
  BOARDIOC_RESETCAUSE_SYS_RWDT,         /* RTC watchdog system reset */
  BOARDIOC_RESETCAUSE_SYS_BOR,          /* brown-out system reset */
  BOARDIOC_RESETCAUSE_CORE_SOFT,        /* software core reset */
  BOARDIOC_RESETCAUSE_CORE_DPSP,        /* deep-sleep core reset */
  BOARDIOC_RESETCAUSE_CORE_MWDT,        /* main watchdog core reset */
  BOARDIOC_RESETCAUSE_CORE_RWDT,        /* RTC watchdog core reset */
  BOARDIOC_RESETCAUSE_CPU_MWDT,         /* main watchdog cpu reset */
  BOARDIOC_RESETCAUSE_CPU_SOFT,         /* software cpu reset */
  BOARDIOC_RESETCAUSE_CPU_RWDT,         /* RTC watchdog cpu reset */
  BOARDIOC_RESETCAUSE_PIN,              /* Pin reset */
  BOARDIOC_RESETCAUSE_LOWPOWER,         /* Low power reset */
  BOARDIOC_RESETCAUSE_UNKOWN            /* Unknown reset cause */
};

enum boardioc_softreset_subreason_e
{
  BOARDIOC_SOFTRESETCAUSE_USER_REBOOT = 0,
  BOARDIOC_SOFTRESETCAUSE_ENTER_BOOTLOADER,
  BOARDIOC_SOFTRESETCAUSE_ENTER_RECOVERY,
  BOARDIOC_SOFTRESETCAUSE_RESTORE_FACTORY,
  BOARDIOC_SOFTRESETCAUSE_PANIC,
  BOARDIOC_SOFTRESETCAUSE_ASSERT
};

struct boardioc_reset_cause_s
{
  enum boardioc_reset_cause_e cause;  /* The reason of last reset */
  uint32_t flag;                      /* watchdog number when watchdog reset,
                                       * or soft-reset subreason */
};
#endif

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
 *   boardctl() is non-standard OS interface to alleviate the problem.
 *   It basically circumvents the normal device driver ioctl interlace and
 *   allows the application to perform direct IOCTL-like calls to the
 *   board-specific logic.
 *   It is especially useful for setting up board operational and test
 *   configurations.
 *
 * Input Parameters:
 *   cmd - Identifies the board command to be executed
 *   arg - The argument that accompanies the command.  The nature of the
 *         argument is determined by the specific command.
 *
 * Returned Value:
 *   On success zero (OK) is returned; -1 (ERROR) is returned on failure
 *   with the errno variable set to indicate the nature of the failure.
 *
 ****************************************************************************/

int boardctl(unsigned int cmd, uintptr_t arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_BOARDCTL */
#endif /* __INCLUDE_SYS_BOARDCTL_H */
