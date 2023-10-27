===========
Board IOCTL
===========

In a small embedded system, there will typically be a much
greater interaction between application and low-level board features.
The canonically correct to implement such interactions is by
implementing a character driver and performing the interactions
via low level ``ioctl()`` calls. This, however, may not be practical
in many cases and will lead to "correct" but awkward implementations.

:c:func:`boardctl` is non-standard OS interface to alleviate the problem.
It basically circumvents the normal device driver ``ioctl()``
interface and allows the application to perform direct
IOCTL-like calls to the board-specific logic. It is especially
useful for setting up board operational and test configurations.

:c:func:`boardctl` is an application interface to the OS.
There is no point, in fact, of using :c:func:`boardctl` within the OS;
the board interfaces prototyped in :file:`include/nuttx/board.h` may
be called directly from within the OS.

.. c:function:: int boardctl(unsigned int cmd, uintptr_t arg)

  :param cmd: Identifies the board command to be executed. See
    :file:`include/sys/boardctl.h` for the complete list of common
    board commands. Provisions are made to support non-common,
    board-specific commands as well.
  :param arg: The argument that accompanies the command. The nature
    of the argument is determined by the specific command.

  :return: On success zero (OK) is returned; -1 (ERROR) is
    returned on failure with the errno variable set to indicate the nature of the failure.

Supported commands
==================

The following is the list of supported :c:func:`boardctl` commands.
Besides this list, board logic can implement handling of custom commands by
implementing the :c:func:`board_ioctl` interface.

System state control
--------------------

.. c:macro:: BOARDIOC_INIT

   Perform one-time application initialization.

   :Argument: The argument is passed to the
     :c:func:`board_app_initialize` implementation without modification.
     The argument has no meaning to NuttX; the meaning of the
     argument is a contract between the board-specific
     initialization logic and the matching application logic.
     The value could be such things as a mode enumeration value,
     a set of DIP switch switch settings, a pointer to
     configuration data read from a file or serial FLASH, or
     whatever you would like to do with it.  Every
     implementation should accept zero/NULL as a default
     configuration.
     
   :Dependencies: Board logic must provide :c:func:`board_app_initialize`.
   
.. c:macro:: BOARDIOC_POWEROFF

   Power off the board
   
   :Argument: Integer value providing power off status information
   
   :configuration: CONFIG_BOARDCTL_POWEROFF
   
   :dependencies: Board logic must provide the :c:func:`board_power_off` interface.
   
.. c:macro:: BOARDIOC_RESET

   Reset the board
   
   :Argument: Integer value providing power off status information
   
   :configuration: CONFIG_BOARDCTL_RESET
   
   :dependencies: Board logic must provide the :c:func:`board_reset` interface.
   
Power Management
----------------
   
.. c:macro:: BOARDIOC_PM_CONTROL

   Manage power state transition and query. The supplied argument
   indicates the specific PM operation to perform, which map to
   corresponding internal ``pm_<operation>`` functions
   (see :doc:`/components/drivers/special/power/pm/index`).
   
   With this interface you can interact with PM handling arch/board logic
   (typically done in IDLE loop) or you can directly manage state transitions
   from userspace.
   
   :Argument: A pointer to an instance of :c:struct:`boardioc_pm_ctrl_s`.
   
   :configuration: CONFIG_PM
   
Board information
-----------------
   
.. c:macro:: BOARDIOC_UNIQUEID

   Return a unique ID associated with the board (such as a
   serial number or a MAC address).
   
   :Argument: A writable array of size :c:macro:`CONFIG_BOARDCTL_UNIQUEID_SIZE` in
     which to receive the board unique ID.
 
   :dependencies: Board logic must provide the :c:func:`board_uniqueid` interface.
   
Filesystems
-----------
   
.. c:macro:: BOARDIOC_MKRD

   Create a RAM disk
   
   :Argument: Pointer to read-only instance of :c:struct:`boardioc_mkrd_s`.
   
   :configuration: CONFIG_BOARDCTL_MKRD

.. c:macro:: BOARDIOC_ROMDISK

   Register a ROM disk
   
   :Argument: Pointer to read-only instance of :c:struct:`boardioc_romdisk_s`.
   
   :configuration: CONFIG_BOARDCTL_ROMDISK
   
Symbol Handling
---------------
   
.. c:macro:: BOARDIOC_APP_SYMTAB

   Select the application symbol table.  This symbol table
   provides the symbol definitions exported to application
   code from application space.
     
   :Argument: A pointer to an instance of :c:struct:`boardioc_symtab_s`.
   
   :configuration: CONFIG_BOARDCTL_APP_SYMTAB
 
.. c:macro:: BOARDIOC_OS_SYMTAB

   Select the OS symbol table.  This symbol table provides
   the symbol definitions exported by the OS to kernel
   modules.
   
   :Argument: A pointer to an instance of :c:struct:`boardioc_symtab_s`.
   
   :configuration: CONFIG_BOARDCTL_OS_SYMTAB
 
.. c:macro:: BOARDIOC_BUILTINS

   Provide the user-space list of built-in applications for
   use by BINFS in protected mode.  Normally this is small
   set of globals provided by user-space logic.  It provides
   name-value pairs for associating built-in application
   names with user-space entry point addresses.  These
   globals are only needed for use by BINFS which executes
   built-in applications from kernel-space in PROTECTED mode.
   In the FLAT build, the user space globals are readily
   available.  (BINFS is not supportable in KERNEL mode since
   user-space address have no general meaning that
   configuration).
   
   :Argument: A pointer to an instance of :c:struct:`boardioc_builtin_s`.
   
   :configuration: This command is always available when
     CONFIG_BUILTIN is enabled, but does nothing unless
     CONFIG_BUILD_PROTECTED is also selected.
     
USB
---
 
.. c:macro:: BOARDIOC_USBDEV_CONTROL

   Manage USB device classes
   
   :Argument: A pointer to an instance of :c:struct:`boardioc_usbdev_ctrl_s`.
   
   :configuration: CONFIG_BOARDCTL && CONFIG_BOARDCTL_USBDEVCTRL
   
   :dependencies: Board logic must provide `board_<usbdev>_initialize()`.
   
Graphics
--------
   
.. c:macro:: BOARDIOC_NX_START

   Start the NX server
   
   :Argument: Integer display number to be served by this NXMU instance.
   
   :configuration: CONFIG_NX
   
   :dependencies: Base graphics logic provides :c:func:`nxmu_start`.
   
.. c:macro:: BOARDIOC_VNC_START

   Start the NX server and framebuffer driver.
   
   :Argument: A reference readable instance of :c:struct:`boardioc_vncstart_s`.
   
   :configuration: CONFIG_VNCSERVER
   
   :dependencies: VNC server provides :c:func:`nx_vnc_fbinitialize`.
   
.. c:macro:: BOARDIOC_NXTERM

   Create an NX terminal device
   
   :Argument: A reference readable/writable instance of
     :c:struct:`boardioc_nxterm_create_s`.
     
   :configuration: CONFIG_NXTERM
   
   :dependencies: Base NX terminal logic provides :c:func:`nx_register` and
     :c:func:`nxtk_register`.
 
.. c:macro:: BOARDIOC_NXTERM_IOCTL

   Create an NX terminal IOCTL command.  Normal IOCTLs
   cannot be be performed in most graphics contexts since
   the depend on the task holding an open file descriptor
   
   :Argument: A reference readable/writable instance of
     :c:struct:`boardioc_nxterm_ioctl_s`.
     
   :configuration: CONFIG_NXTERM
   
   :dependencies: Base NX terminal logic provides :c:func:`nxterm_ioctl_tap`.
   
Testing
-------
   
.. c:macro:: BOARDIOC_TESTSET

   Access architecture-specific up_testset() operation
   
   :Argument: A pointer to a write-able spinlock object. On success
     the  preceding spinlock state is returned: 0=unlocked,
     1=locked.
   
   :configuration: CONFIG_BOARDCTL_TESTSET
   
   :dependencies: Architecture-specific logic provides :c:func:`up_testset`.

