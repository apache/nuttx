=====================================================
APIs Exported by Architecture-Specific Logic to NuttX
=====================================================

.. c:function:: void up_initialize(void)

  Called once during OS
  initialization after the basic OS services have been initialized.
  The architecture specific details of initializing the OS will be
  handled here. Such things as setting up interrupt service
  routines, starting the clock, and registering device
  are some of the things that are
  different for each processor and hardware platform.

  ``up_initialize()`` is called after the OS initialized but before
  the init process has been started and before the libraries have
  been initialized. OS services and driver services are available.

.. c:function:: void up_idle(void)

  The logic that will be executed
  when their is no other ready-to-run task. This is processor idle
  time and will continue until some interrupt occurs to cause a
  context switch from the idle task.

  Processing in this state may be processor-specific. e.g., this is
  where power management operations might be performed.

.. c:function:: void up_initial_state(FAR struct tcb_s *tcb)

  A new thread is being started and a new TCB has
  been created. This function is called to initialize the processor
  specific portions of the new TCB.

  This function must setup the initial architecture registers and/or
  stack so that execution will begin at tcb->start on the next
  context switch.

  This function may also need to set up processor registers so that
  the new thread executes with the correct privileges. If
  ``CONFIG_BUILD_PROTECTED`` or ``CONFIG_BUILD_KERNEL`` have been
  selected in the NuttX configuration, then special initialization
  may need to be performed depending on the task type specified in
  the TCB's flags field: Kernel threads will require kernel-mode
  privileges; User tasks and pthreads should have only user-mode
  privileges. If neither ``CONFIG_BUILD_PROTECTED`` nor
  ``CONFIG_BUILD_KERNEL`` have been selected, then all threads
  should have kernel-mode privileges.

.. c:function:: STATUS up_create_stack(FAR struct tcb_s *tcb, size_t stack_size, uint8_t ttype)

  Allocate a stack for a new thread and setup up
  stack-related information in the TCB.

  The following TCB fields must be initialized:

  -  ``adj_stack_size``: Stack size after adjustment for hardware,
     processor, etc. This value is retained only for debug purposes.
  -  ``stack_alloc_ptr``: Pointer to allocated stack
  -  ``stack_base_ptr``: Adjusted stack base pointer after the TLS Data
     and Arguments has been removed from the stack allocation.

  :param tcb: The TCB of new task.
  :param stack_size: The requested stack size. At least this much
    must be allocated.
  :param ttype: The thread type. This may be one of following
     (defined in ``include/nuttx/sched.h``):

     -  ``TCB_FLAG_TTYPE_TASK``: Normal user task
     -  ``TCB_FLAG_TTYPE_PTHREAD``: User pthread
     -  ``TCB_FLAG_TTYPE_KERNEL``: Kernel thread

     This thread type is normally available in the flags field of
     the TCB, however, there are certain contexts where the TCB may
     not be fully initialized when up_create_stack is called.

     If ``CONFIG_BUILD_PROTECTED`` or ``CONFIG_BUILD_KERNEL`` are
     defined, then this thread type may affect how the stack is
     allocated. For example, kernel thread stacks should be
     allocated from protected kernel memory. Stacks for user tasks
     and threads must come from memory that is accessible to user
     code.

.. c:function:: STATUS up_use_stack(FAR struct tcb_s *tcb, FAR void *stack, size_t stack_size)

  Setup up stack-related information in the TCB
  using pre-allocated stack memory. This function is called only
  from ``nxtask_init()`` when a task or kernel thread is started
  (never for pthreads).

  The following TCB fields must be initialized:

  -  ``adj_stack_size``: Stack size after adjustment for hardware,
     processor, etc. This value is retained only for debug purposes.
  -  ``stack_alloc_ptr``: Pointer to allocated stack
  -  ``stack_base_ptr``: Adjusted stack base pointer after the TLS Data
     and Arguments has been removed from the stack allocation.

  :param tcb: The TCB of new task.
  :param stack_size: The allocated stack size.

  NOTE: Unlike ``up_stack_create()`` and ``up_stack_release``, this
  function does not require the task type (``ttype``) parameter. The
  TCB flags will always be set to provide the task type to
  ``up_use_stack()`` if the information needs that information.

.. c:function:: FAR void *up_stack_frame(FAR struct tcb_s *tcb, size_t frame_size)

  Allocate a stack frame in the TCB's stack to hold
  thread-specific data. This function may be called any time after
  ``up_create_stack()`` or ``up_use_stack()`` have been called but
  before the task has been started.

  Thread data may be kept in the stack (instead of in the TCB) if it
  is accessed by the user code directly. This includes such things
  as ``argv[]``. The stack memory is guaranteed to be in the same
  protection domain as the thread.

  The following TCB fields will be re-initialized:

  -  ``adj_stack_size``: Stack size after removal of the stack frame
     from the stack.
  -  ``stack_base_ptr``: Adjusted stack base pointer after the TLS Data
     and Arguments has been removed from the stack allocation.

  Here is the diagram after some allocation(tls, arg)::

                     +-------------+ <-stack_alloc_ptr(lowest)
                     |  TLS Data   |
                     +-------------+
                     |  Arguments  |
    stack_base_ptr-> +-------------+\
                     |  Available  | +
                     |    Stack    | |
                  |  |             | |
                  |  |             | +->adj_stack_size
                  v  |             | |
                     |             | |
                     |             | +
                     +-------------+/

  :param tcb: The TCB of new task.
  :param frame_size: The size of the stack frame to allocate.

  :return:
    A pointer to bottom of the allocated stack
    frame. NULL will be returned on any failures. The alignment of the
    returned value is the same as the alignment of the stack itself

.. c:function:: void up_release_stack(FAR struct tcb_s *dtcb)

  A task has been stopped. Free all stack related
  resources retained int the defunct TCB.

  :param dtcb: The TCB containing information about the stack to be
     released.

  :param ttype: The thread type. This may be one of following
     (defined in ``include/nuttx/sched.h``):

     -  ``TCB_FLAG_TTYPE_TASK``: Normal user task
     -  ``TCB_FLAG_TTYPE_PTHREAD``: User pthread
     -  ``TCB_FLAG_TTYPE_KERNEL``: Kernel thread

     This thread type is normally available in the flags field of
     the TCB, however, there are certain error recovery contexts
     where the TCB may not be fully initialized when
     up_release_stack is called.

     If ``CONFIG_BUILD_PROTECTED`` or ``CONFIG_BUILD_KERNEL`` are
     defined, then this thread type may affect how the stack is
     freed. For example, kernel thread stacks may have been
     allocated from protected kernel memory. Stacks for user tasks
     and threads must have come from memory that is accessible to
     user

.. c:function:: void up_unblock_task(FAR struct tcb_s *tcb)

  A task is currently in an inactive task list but
  has been prepped to execute. Move the TCB to the ready-to-run
  list, restore its context, and start execution.

  This function is called only from the NuttX scheduling logic.
  Interrupts will always be disabled when this function is called.

  :param tcb: Refers to the tcb to be unblocked. This tcb is in one
    of the waiting tasks lists. It must be moved to the
    ready-to-run list and, if it is the highest priority ready to
    run tasks, executed.

.. c:function:: void up_block_task(FAR struct tcb_s *tcb, tstate_t task_state)

  The currently executing task at the head of the
  ready to run list must be stopped. Save its context and move it to
  the inactive list specified by task_state. This function is called
  only from the NuttX scheduling logic. Interrupts will always be
  disabled when this function is called.

  :param tcb: Refers to a task in the ready-to-run list (normally
     the task at the head of the list). It must be stopped, its
     context saved and moved into one of the waiting task lists. If
     it was the task at the head of the ready-to-run list, then a
     context switch to the new ready to run task must be performed.
  :param task_state: Specifies which waiting task list should be
     hold the blocked task TCB.

.. c:function:: void up_release_pending(void)

  When tasks become ready-to-run but cannot run
  because pre-emption is disabled, they are placed into a pending
  task list. This function releases and makes ready-to-run all of
  the tasks that have collected in the pending task list. This can
  cause a context switch if a new task is placed at the head of the
  ready to run list.

  This function is called only from the NuttX scheduling logic when
  pre-emption is re-enabled. Interrupts will always be disabled when
  this function is called.

.. c:function:: void up_reprioritize_rtr(FAR struct tcb_s *tcb, uint8_t priority)

  Called when the priority of a running or
  ready-to-run task changes and the reprioritization will cause a
  context switch. Two cases:

  #. The priority of the currently running task drops and the next
     task in the ready to run list has priority.
  #. An idle, ready to run task's priority has been raised above the
     the priority of the current, running task and it now has the
     priority.

  This function is called only from the NuttX scheduling logic.
  Interrupts will always be disabled when this function is called.

  :param tcb: The TCB of the task that has been reprioritized
  :param priority: The new task priority

.. c:macro:: noreturn_function

.. c:function:: void up_exit(int status) noreturn_function;

  This function causes the currently executing task
  to cease to exist. This is a special case of task_delete().

  Unlike other UP APIs, this function may be called directly from
  user programs in various states. The implementation of this
  function should disable interrupts before performing scheduling
  operations.

.. c:function:: void up_assert(FAR const char *filename, int linenum)

  Assertions may be handled in an
  architecture-specific way.

.. c:function:: void up_schedule_sigaction(FAR struct tcb_s *tcb, sig_deliver_t sigdeliver)

  This function is called by the OS when one or
  more signal handling actions have been queued for execution. The
  architecture specific code must configure things so that the
  'sigdeliver' callback is executed on the thread specified by 'tcb'
  as soon as possible.

  This function may be called from interrupt handling logic.

  This operation should not cause the task to be unblocked nor
  should it cause any immediate execution of sigdeliver. Typically,
  a few cases need to be considered:

    #. This function may be called from an interrupt handler During
       interrupt processing, all xcptcontext structures should be
       valid for all tasks. That structure should be modified to
       invoke sigdeliver() either on return from (this) interrupt or
       on some subsequent context switch to the recipient task.
    #. If not in an interrupt handler and the tcb is NOT the currently
       executing task, then again just modify the saved xcptcontext
       structure for the recipient task so it will invoke sigdeliver
       when that task is later resumed.
    #. If not in an interrupt handler and the tcb IS the currently
       executing task -- just call the signal handler now.

.. c:function:: void up_allocate_heap(FAR void **heap_start, size_t *heap_size)

  This function will be called to dynamically set
  aside the heap region.

  For the kernel build (``CONFIG_BUILD_PROTECTED=y`` or
  ``CONFIG_BUILD_KERNEL=y``) with both kernel- and user-space heaps
  (``CONFIG_MM_KERNEL_HEAP=y``), this function provides the size of
  the unprotected, user-space heap. If a protected kernel-space heap
  is provided, the kernel heap must be allocated (and protected) by
  an analogous ``up_allocate_kheap()``.

.. c:function:: bool up_interrupt_context(void)

  Return true if we are currently executing in the
  interrupt handler context.

.. c:function::  void up_disable_irq(int irq)

  Disable the IRQ specified by 'irq' On many
  architectures, there are three levels of interrupt enabling: (1)
  at the global level, (2) at the level of the interrupt controller,
  and (3) at the device level. In order to receive interrupts, they
  must be enabled at all three levels.

  This function implements enabling of the device specified by 'irq'
  at the interrupt controller level if supported by the architecture
  (up_irq_save() supports the global level, the device level is
  hardware specific).

  If the architecture does not support ``up_disable_irq``,
  ``CONFIG_ARCH_NOINTC`` should be defined in the NuttX
  configuration file. Since this API cannot be supported on all
  architectures, it should be avoided in common implementations
  where possible.

.. c:function:: void up_enable_irq(int irq)

  This function implements disabling of the device
  specified by 'irq' at the interrupt controller level if supported
  by the architecture (up_irq_restore() supports the global level,
  the device level is hardware specific).

  If the architecture does not support ``up_disable_irq``,
  ``CONFIG_ARCH_NOINTC`` should be defined in the NuttX
  configuration file. Since this API cannot be supported on all
  architectures, it should be avoided in common implementations
  where possible.

.. c:function:: void up_prioritize_irq(int irq)

  Set the priority of an IRQ.

  If the architecture supports ``up_enable_irq``,
  ``CONFIG_ARCH_IRQPRIO`` should be defined in the NuttX
  configuration file. Since this API cannot be supported on all
  architectures, it should be avoided in common implementations
  where possible.

.. c:function::  int up_putc(int ch)

  This is a debug interface exported by the
  architecture-specific logic. Output one character on the console

