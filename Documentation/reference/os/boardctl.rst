====================================
``boardctl()`` Application Interface
====================================

.. c:function:: int boardctl(unsigned int cmd, uintptr_t arg)

  In a small embedded system, there will typically be a much
  greater interaction between application and low-level board features.
  The canonically correct to implement such interactions is by
  implementing a character driver and performing the interactions
  via low level ioctl() calls. This, however, may not be practical
  in many cases and will lead to "correct" but awkward implementations.

  boardctl() is non-standard OS interface to alleviate the problem.
  It basically circumvents the normal device driver ioctl()
  interlace and allows the application to perform direct
  IOCTL-like calls to the board-specific logic. It is especially
  useful for setting up board operational and test configurations.

  NOTE: The other interfaces described in this document are internal
  OS interface. boardctl() is an application interface to the OS.
  There is no point, in fact, of using boardctl() within the OS;
  the board interfaces prototyped in include/nuttx/board.h may
  be called directly from within the OS.

  Application interfaces are described in the NuttX User Guide.
  This application interface interface is described here only
  because it is so non-standard and because it is so closely
  tied to board porting logic.

  :param cmd: Identifies the board command to be executed. See
    ``include/sys/boardctl.h`` for the complete list of common
    board commands. Provisions are made to support non-common,
    board-specific commands as well.
  :param arg: The argument that accompanies the command. The nature
    of the argument is determined by the specific command.

  :return: On success zero (OK) is returned; -1 (ERROR) is
    returned on failure with the errno variable set to indicate the nature of the failure.

