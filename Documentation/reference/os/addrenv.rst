====================
Address Environments
====================

CPUs that support memory management units (MMUs) may provide
*address environments* within which tasks and their child threads
execute. The configuration indicates the CPUs ability to support
address environments by setting the configuration variable
``CONFIG_ARCH_HAVE_ADDRENV=y``. That will enable the selection of
the actual address environment support which is indicated by the
selection of the configuration variable ``CONFIG_ARCH_ADDRENV=y``.
These address environments are created only when tasks are created
via ``exec()`` or ``exec_module()`` (see
``include/nuttx/binfmt/binfmt.h``).

When ``CONFIG_ARCH_ADDRENV=y`` is set in the board configuration,
the CPU-specific logic must provide a set of interfaces as defined
in the header file ``include/nuttx/arch.h``. These interfaces are
listed below and described in detail in the following paragraphs.

The CPU-specific logic must provide two categories in interfaces:

#. **Binary Loader Support**. These are low-level interfaces used
   in ``binfmt/`` to instantiate tasks with address environments.
   These interfaces all operate on type ``arch_addrenv_t`` which
   is an abstract representation of a task group's address
   environment and the type must be defined in\ ``arch/arch.h`` if
   ``CONFIG_ARCH_ADDRENV`` is defined. These low-level interfaces
   include:

   - :c:func:`up_addrenv_create()`: Create an address environment.
   - :c:func:`up_addrenv_destroy()`: Destroy an address environment.
   - :c:func:`up_addrenv_vtext()`: Returns the virtual base address of the ``.text`` address environment.
   - :c:func:`up_addrenv_vdata()`: Returns the virtual base address of the ``.bss``/``.data`` address environment.
   - :c:func:`up_addrenv_heapsize()`: Return the initial heap size.
   - :c:func:`up_addrenv_select()`: Instantiate an address environment.
   - :c:func:`up_addrenv_restore()`: Restore an address environment.
   - :c:func:`up_addrenv_clone()`: Copy an address environment from one location to another.

#. **Tasking Support**. Other interfaces must be provided to
   support higher-level interfaces used by the NuttX tasking
   logic. These interfaces are used by the functions in ``sched/``
   and all operate on the task group which as been assigned an
   address environment by ``up_addrenv_clone()``.

   - :c:func:`up_addrenv_attach()`: Clone the group address environment assigned to a new
     thread. This operation is done when a pthread is created
     that share's the same address environment.
   - :c:func:`up_addrenv_detach()`: Release the thread's reference to a group address
     environment when a task/thread exits.

#. **Dynamic Stack Support**. ``CONFIG_ARCH_STACK_DYNAMIC=y``
   indicates that the user process stack resides in its own
   address space. This option is also *required* if
   ``CONFIG_BUILD_KERNEL`` and ``CONFIG_LIBC_EXECFUNCS`` are
   selected. Why? Because the caller's stack must be preserved in
   its own address space when we instantiate the environment of
   the new process in order to initialize it.

   **NOTE:** The naming of the ``CONFIG_ARCH_STACK_DYNAMIC``
   selection implies that dynamic stack allocation is supported.
   Certainly this option must be set if dynamic stack allocation
   is supported by a platform. But the more general meaning of
   this configuration environment is simply that the stack has its
   own address space.

   If ``CONFIG_ARCH_STACK_DYNAMIC=y`` is selected then the
   platform specific code must export these additional interfaces:

   - :c:func:`up_addrenv_ustackalloc()`: Create a stack address environment
   - :c:func:`up_addrenv_ustackfree()`: Destroy a stack address environment.
   - :c:func:`up_addrenv_vustack()`: Returns the virtual base address of the stack
   - :c:func:`up_addrenv_ustackselect()`: Instantiate a stack address environment

#. If ``CONFIG_ARCH_KERNEL_STACK`` is selected, then each user
   process will have two stacks: (1) a large (and possibly
   dynamic) user stack and (2) a smaller kernel stack. However,
   this option is *required* if both ``CONFIG_BUILD_KERNEL`` and
   ``CONFIG_LIBC_EXECFUNCS`` are selected. Why? Because when we
   instantiate and initialize the address environment of the new
   user process, we will temporarily lose the address environment
   of the old user process, including its stack contents. The
   kernel C logic will crash immediately with no valid stack in
   place.

   If ``CONFIG_ARCH_KERNEL_STACK=y`` is selected then the platform
   specific code must export these additional interfaces:

   - :c:func:`up_addrenv_kstackalloc`: Allocate the process kernel stack.

.. c:function:: int up_addrenv_create(size_t textsize, size_t datasize, \
  size_t heapsize, FAR arch_addrenv_t *addrenv);

  This function is called when a new task is created in order to
  instantiate an address environment for the new task group.
  up_addrenv_create() is essentially the allocator of the physical memory for the new task.

  :param textsize: The size (in bytes) of the ``.text`` address
    environment needed by the task. This region may be read/execute
    only.
  :param datasize: The size (in bytes) of the ``.bss/.data`` address
    environment needed by the task. This region may be read/write
    only.
  :param heapsize: The initial size (in bytes) of the heap address
    environment needed by the task. This region may be read/write
    only.
  :param addrenv: The location to return the representation of the
    task address environment.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_destroy(arch_addrenv_t *addrenv)

  This function is called when a final thread leaves the task
  group and the task group is destroyed. This function then destroys
  the defunct address environment, releasing the underlying physical
  memory allocated by up_addrenv_create().

  :param addrenv: The representation of the task address environment
    previously returned by ``up_addrenv_create()``.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_vtext(FAR arch_addrenv_t addrenv, FAR void **vtext)

  Return the virtual .text address associated with the newly create
  address environment. This function is used by the binary loaders
  in order get an address that can be used to initialize the new task.

  :param addrenv: The representation of the task address environment
     previously returned by ``up_addrenv_create()``.
  :param vtext: The location to return the virtual address.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_vdata(FAR arch_addrenv_t *addrenv, size_t textsize, FAR void **vdata)

  Return the virtual .text address associated with the newly create
  address environment. This function is used by the binary loaders
  in order get an address that can be used to initialize the new task.

  :param addrenv: The representation of the task address environment
    previously returned by ``up_addrenv_create()``.
  :param textsize: For some implementations, the text and data will
    be saved in the same memory region (read/write/execute) and, in
    this case, the virtual address of the data just lies at this
    offset into the common region.
  :param vdata: The location to return the virtual address.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: ssize_t up_addrenv_heapsize(FAR const arch_addrenv_t *addrenv)

  Return the initial heap allocation size. That is the amount of
  memory allocated by up_addrenv_create() when the heap memory
  region was first created. This may or may not differ from the
  heapsize parameter that was passed to up_addrenv_create().

  :param addrenv: The representation of the task address environment
    previously returned by ``up_addrenv_create()``.

  :return: The initial heap size allocated is returned on success;
    a negated errno value on failure.

.. c:function:: int up_addrenv_select(arch_addrenv_t *addrenv, save_addrenv_t *oldenv)

  After an address environment has been established for a task
  (via up_addrenv_create()), this function may be called to instantiate
  that address environment in the virtual address space. This might be
  necessary, for example, to load the code for the task from a file or
  to access address environment private data.

  :param addrenv: The representation of the task address environment
    previously returned by ``up_addrenv_create()``.
  :param oldenv: The address environment that was in place before
    ``up_addrenv_select()`` was called. This may be used with
    ``up_addrenv_restore()`` to restore the original address
    environment that was in place before ``up_addrenv_select()``
    was called. Note that this may be a task agnostic,
    platform-specific representation that may or may not be
    different from ``arch_addrenv_t``.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_restore(save_addrenv_t oldenv)

  After an address environment has been temporarily instantiated
  by up_addrenv_select, this function may be called to restore
  the original address environment.

  :param oldenv: The platform-specific representation of the address
    environment previously returned by ``up_addrenv_select()``.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_clone(FAR const task_group_s *src, FAR struct task_group_s *dest)

  Duplicate an address environment. This does not copy the underlying
  memory, only the representation that can be used to instantiate
  that memory as an address environment.

  :param src: The address environment to be copied.
  :param dest: The location to receive the copied address
    environment.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_attach(FAR struct task_group_s *group, FAR struct tcb_s *tcb)

  This function is called from the core scheduler logic when a
  thread is created that needs to share the address environment
  of its task group. In this case, the group's address environment
  may need to be "cloned" for the child thread.

  NOTE: In most platforms, nothing will need to be done in this case.
  Simply being a member of the group that has the address environment
  may be sufficient.

  :param group: The task group to which the new thread belongs.
  :param ctcb: The TCB of the thread needing the address
    environment.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_detach(FAR struct task_group_s *group, FAR struct task_group_s *tcb)

  This function is called when a task or thread exits in order
  to release its reference to an address environment. The address
  environment, however, should persist until up_addrenv_destroy()
  is called when the task group is itself destroyed. Any resources
  unique to this thread may be destroyed now.

  :param group: The group to which the thread belonged.
  :param tcb: The TCB of the task or thread whose the address
    environment will be released.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_ustackalloc(FAR struct tcb_s *tcb, size_t stacksize)

  This function is called when a new thread is created in order
  to instantiate an address environment for the new thread's stack.
  up_addrenv_ustackalloc() is essentially the allocator of the
  physical memory for the new task's stack.

  :param tcb: The TCB of the thread that requires the stack address
    environment.
  :param stacksize: The size (in bytes) of the initial stack address
    environment needed by the task. This region may be read/write
    only.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_ustackfree(FAR struct tcb_s *tcb)

  This function is called when any thread exits. This function then
  destroys the defunct address environment for the thread's stack,
  releasing the underlying physical memory.

  :param tcb: The TCB of the thread that no longer requires the
    stack address environment.

  :return: Zero (OK) on success; a negated errno value on failure

.. c:function:: int up_addrenv_vustack(FAR const struct tcb_s *tcb, FAR void **vstack)

  Return the virtual address associated with the newly create stack address environment.

  :param tcb: The TCB of the thread with the stack address environment of interest.
  :param vstack: The location to return the stack virtual base address.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_ustackselect(FAR const struct tcb_s *tcb)

  After an address environment has been established for a task's
  stack (via up_addrenv_ustackalloc(). This function may be called to
  instantiate that address environment in the virtual address space.
  This is a necessary step before each context switch to the newly
  created thread (including the initial thread startup).

  :param tcb: The TCB of the thread with the stack address
    environment to be instantiated.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_kstackalloc(FAR struct tcb_s *tcb)

  This function is called when a new thread is created to allocate the
  new thread's kernel stack. This function may be called for certain
  terminating threads which have no kernel stack. It must be
  tolerant of that case.

  :param tcb: The TCB of the thread that requires the kernel stack.

  :return: Zero (OK) on success; a negated errno value on failure.

.. c:function:: int up_addrenv_kstackfree(FAR struct tcb_s *tcb);

  This function is called when any thread exits. This function frees the kernel stack.

  :param tcb: The TCB of the thread that no longer requires the
    kernel stack.

  :return: Zero (OK) on success; a negated errno value on failure.
