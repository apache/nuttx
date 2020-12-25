=====================
Note Driver Interface
=====================

Note driver is the interface to access the instrumentation data.
The following devices are provided.

- :ref:`notectl`
- :ref:`noteram`

.. _notectl:

Notectl Device (``/dev/notectl``)
=================================

  ``/dev/notectl`` is the device to control an instrumentation filter in NuttX kernel.
  The device has only ioctl function to control the filter.

``/dev/notectl`` Header Files
-----------------------------

  The header file ``include/nuttx/note/notectl_driver.h`` provides the interface definitions of the device.

``/dev/notectl`` Data Structures
--------------------------------

.. c:struct:: note_filter_mode_s

  .. code-block:: c

    struct note_filter_mode_s
    {
      unsigned int flag;          /* Filter mode flag */
    #ifdef CONFIG_SMP
      unsigned int cpuset;        /* The set of monitored CPUs */
    #endif
    };

  - ``flag`` : Filter mode flag. The bitwise OR of the following defines are available.

    .. c:macro:: NOTE_FILTER_MODE_FLAG_ENABLE

      Enable instrumentation

    .. c:macro:: NOTE_FILTER_MODE_FLAG_SYSCALL

      Enable syscall instrumentation

    .. c:macro:: NOTE_FILTER_MODE_FLAG_IRQ

      Enable IRQ instrumentaiton

  - ``cpuset`` : (SMP only) Monitor only CPUs in the bitset. Bit 0=CPU0, Bit1=CPU1, etc.

.. c:struct:: note_filter_syscall_s

  .. code-block:: c

    struct note_filter_syscall_s
    {
      uint8_t syscall_mask[];
    };

  - ``syscall_mask`` : A bitmap array of the syscall filter. If a bit is set, the corresponding syscall is not recorded.
    The following helper macros are available:

    .. c:macro:: NOTE_FILTER_SYSCALLMASK_SET(nr, s)

      Set syscall number `nr` as masked. `s` specifies the variable of `struct note_filter_syscall_s`

    .. c:macro:: NOTE_FILTER_SYSCALLMASK_CLR(nr, s)

      Set syscall number `nr` as unmasked.

    .. c:macro:: NOTE_FILTER_SYSCALLMASK_ISSET(nr, s)

      Check whether syscall number `nr` is masked or not. True if masked.

    .. c:macro:: NOTE_FILTER_SYSCALLMASK_ZERO(s)

      Clear all masks.

.. c:struct:: note_filter_irq_s

  .. code-block:: c

    struct note_filter_irq_s
    {
      uint8_t irq_mask[];
    };

  - ``irq_mask`` : A bitmap array of the IRQ filter. If a bit is set, the corresponding IRQ is not recorded.
    The following helper macros are available:

    .. c:macro:: NOTE_FILTER_IRQMASK_SET(nr, s)

      Set IRQ number `nr` as masked. `s` specifies the variable of `struct note_filter_irq_s`

    .. c:macro:: NOTE_FILTER_IRQMASK_CLR(nr, s)

      Set IRQ number `nr` as unmasked.

    .. c:macro:: NOTE_FILTER_IRQMASK_ISSET(nr, s)

      Check whether IRQ number `nr` is masked or not. True if masked.

    .. c:macro:: NOTE_FILTER_IRQMASK_ZERO(s)

      Clear all masks.

``/dev/notectl`` Ioctls
-----------------------

.. c:macro:: NOTECTL_GETMODE

  Get note filter mode

  :argument: A writable pointer to :c:struct:`note_filter_mode_s`

  :return: If success, 0 (``OK``) is returned and current note filter mode is stored into the given pointer.
    If failed, a negated ``errno`` is returned.

.. c:macro:: NOTECTL_SETMODE

  Set note filter mode

  :argument: A read-only pointer to :c:struct:`note_filter_mode_s`

  :return: If success, 0 (``OK``) is returned and the given filter mode is set as the current settings.
    If failed, a negated ``errno`` is returned.

.. c:macro:: NOTECTL_GETSYSCALLFILTER

  Get syscall filter setting

  :argument: A writable pointer to :c:struct:`note_filter_syscall_s`

  :return: If success, 0 (``OK``) is returned and current syscall filter mode is stored into the given pointer.
    If failed, a negated ``errno`` is returned.

.. c:macro:: NOTECTL_SETSYSCALLFILTER

  Set syscall filter setting

  :argument: A read-only pointer to :c:struct:`note_filter_syscall_s`

  :return: If success, 0 (``OK``) is returned and the given syscall filter mode is set as the current settings.
    If failed, a negated ``errno`` is returned.

.. c:macro:: NOTECTL_GETIRQFILTER

  Get IRQ filter setting

  :argument: A writable pointer to :c:struct:`note_filter_irq_s`

  :return: If success, 0 (``OK``) is returned and current IRQ filter mode is stored into the given pointer.
    If failed, a negated ``errno`` is returned.

.. c:macro:: NOTECTL_SETIRQFILTER

  Set IRQ filter setting

  :argument: A read-only pointer to :c:struct:`note_filter_irq_s`

  :return: If success, 0 (``OK``) is returned and the given IRQ filter mode is set as the current settings.
    If failed, a negated ``errno`` is returned.


.. _noteram:

Noteram Device (``/dev/note``)
==============================

  ``/dev/note`` is the device to get the trace (instrumentation) data.
  The device has read function to get the data and ioctl function to control the buffer mode.


``/dev/note`` Header Files
--------------------------

  The header file ``include/nuttx/note/noteram_driver.h`` provides the interface definitions of the device.

``/dev/note`` Data Structures
--------------------------------

.. c:struct:: noteram_get_taskname_s

  .. code-block:: c

    struct noteram_get_taskname_s
    {
      pid_t pid;
      char taskname[CONFIG_TASK_NAME_SIZE + 1];
    };

  - ``pid`` : Task ID to get the task name.

  - ``taskname`` : The task name string corresponding to given pid.

``/dev/note`` Ioctls
--------------------

.. c:macro:: NOTERAM_CLEAR

  Clear all contents of the circular buffer

  :argument: Ignored

  :return: Always returns 0.

.. c:macro:: NOTERAM_GETMODE

  Get overwrite mode

  :argument: A writable pointer to ``unsigned int``.
    The overwrite mode takes one of the following values.

    .. c:macro:: NOTERAM_MODE_OVERWRITE_DISABLE

      Overwrite mode is disabled. When the buffer is full, accepting the data will be stopped.

    .. c:macro:: NOTERAM_MODE_OVERWRITE_ENABLE

      Overwrite mode is enabled.

    .. c:macro:: NOTERAM_MODE_OVERWRITE_OVERFLOW

      Overwrite mode is disabled and the buffer is already full.

  :return: If success, 0 (``OK``) is returned and current overwrite mode is stored into the given pointer.
           If failed, a negated ``errno`` is returned.

.. c:macro:: NOTERAM_SETMODE

  Set overwrite mode

  :argument: A read-only pointer to ``unsigned int``.

  :return: If success, 0 (``OK``) is returned and the given overwriter mode is set as the current settings.
    If failed, a negated ``errno`` is returned.

.. c:macro:: NOTERAM_GETTASKNAME

  Get task name string

  :argument: A writable pointer to :c:struct:`noteram_get_taskname_s`

  :return: If success, 0 (``OK``) is returned and the task name corresponding to given pid is stored into the given pointer.
           If failed, a negated ``errno`` is returned.

Filter control APIs
===================

The following APIs are the functions to control note filters directly.
These are kernel APIs and application can use them only in FLAT build.

The header file ``include/nuttx/sched_note.h`` is needed to use the following APIs.

API description
---------------

.. c:function:: void sched_note_filter_mode(struct note_filter_mode_s *oldm, struct note_filter_mode_s *newm);

  Set and get note filter mode.
  (Same as :c:macro:`NOTECTL_GETMODE` / :c:macro:`NOTECTL_SETMODE` ioctls)

  :param oldm: A writable pointer to :c:struct:`note_filter_mode_s` to get current filter mode.
    If 0, no data is written.
  :param newm: A read-only pointer to :c:struct:`note_filter_mode_s` which holds the new filter mode.
    If 0, the filter mode is not updated.

  :return: None

.. c:function:: void sched_note_filter_syscall(struct note_filter_syscall_s *oldf, struct note_filter_syscall_s *newf);

  Set and get syscall filter setting.
  (Same as :c:macro:`NOTECTL_GETSYSCALLFILTER` / :c:macro:`NOTECTL_SETSYSCALLFILTER` ioctls)

  :param oldf: A writable pointer to :c:struct:`note_filter_syscall_s` to get current syscall filter setting.
    If 0, no data is written.
  :param newf: A read-only pointer to :c:struct:`note_filter_syscall_s` of the new syscall filter setting.
    If 0, the setting is not updated.

  :return: None

.. c:function:: void sched_note_filter_irq(struct note_filter_irq_s *oldf, struct note_filter_irq_s *newf);

  Set and get IRQ filter setting.
  (Same as :c:macro:`NOTECTL_GETIRQFILTER` / :c:macro:`NOTECTL_SETIRQFILTER` ioctls)

  :param oldf: A writable pointer to :c:struct:`note_filter_irq_s` to get current IRQ filter setting.
    If 0, no data is written.
  :param newf: A read-only pointer to :c:struct:`note_filter_irq_s` of the new IRQ filter setting.
    If 0, the setting is not updated.

  :return: None
