==============================================
APIs Exported by Board-Specific Logic to NuttX
==============================================

Exported board-specific interfaces are prototyped in the header
file ``include/nuttx/board.h``. There are many interfaces exported
from board- to architecture-specific logic. But there are only a
few exported from board-specific logic to common NuttX logic.
Those few of those related to initialization will be discussed in
this paragraph. There are others, like those used by
```boardctl()`` <#boardctl>`__ that will be discussed in other
paragraphs.

All of the board-specific interfaces used by the NuttX OS logic
are for controlled board initialization. There are three points in
time where you can insert custom, board-specific initialization
logic:

First, ``<arch>_board_initialize()``: This function is *not*
called from the common OS logic, but rather from the
architecture-specific power on reset logic. This is used only for
initialization of very low-level things like configuration of GPIO
pins, power settings, DRAM initialization, etc. The OS has not
been initialized at this point, so you cannot allocate memory or
initialize device drivers.

The other two board initialization *hooks* are called from the OS
start-up logic and are described in the following paragraphs:

.. c:function:: void board_early_initialize(void)

  The next level of initialization is performed by a call to
  ``up_initialize()`` (in
  ``arch/<arch>/src/common/up_initialize.c``). The OS has been
  initialized at this point and it is okay to initialize drivers in
  this phase. ``up_initialize()`` is *not* a board-specific
  interface, but rather an architecture-specific, board-independent
  interface.

  But at this same point in time, the OS will also call a
  board-specific initialization function named
  ``board_early_initialize()`` if
  ``CONFIG_BOARD_EARLY_INITIALIZE=y`` is selected in the
  configuration. The context in which ``board_early_initialize()``
  executes is suitable for early initialization of most, simple
  device drivers and is a logical, board-specific extension of
  up_initialize().

  ``board_early_initialize()`` runs on the startup, initialization
  thread. Some initialization operations cannot be performed on the
  start-up, initialization thread. That is because the
  initialization thread cannot wait for event. Waiting may be
  required, for example, to mount a file system or or initialize a
  device such as an SD card. For this reason, such driver initialize
  must be deferred to ``board_late_initialize()``.

.. c:function:: void board_late_initialize(void)

  And, finally, just before the user application code starts. If
  ``CONFIG_BOARD_LATE_INITIALIZE=y`` is selected in the
  configuration, then an final, additional initialization call will
  be performed in the boot-up sequence to a function called
  ``board_late_initialize()``. ``board_late_initialize()`` will be
  called well after ``up_initialize()`` and
  ``board_early_initialize()`` are called.
  ``board_late_initialize()`` will be called just before the main
  application task is started. This additional initialization phase
  may be used, for example, to initialize more complex,
  board-specific device drivers.

  Waiting for events, use of I2C, SPI, etc are permissible in the
  context of board_late_initialize(). That is because
  ``board_late_initialize()`` will run on a temporary, internal
  kernel thread.
