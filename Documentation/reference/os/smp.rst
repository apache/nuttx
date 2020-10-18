===========================================
Symmetric Multiprocessing (SMP) Application
===========================================

According to Wikipedia: "Symmetric multiprocessing (SMP) involves
a symmetric multiprocessor system hardware and software
architecture where two or more identical processors connect to a
single, shared main memory, have full access to all I/O devices,
and are controlled by a single operating system instance that
treats all processors equally, reserving none for special
purposes. Most multiprocessor systems today use an SMP
architecture. In the case of multi-core processors, the SMP
architecture applies to the cores, treating them as separate
processors.

"SMP systems are tightly coupled multiprocessor systems with a
pool of homogeneous processors running independently, each
processor executing different programs and working on different
data and with capability of sharing common resources (memory, I/O
device, interrupt system and so on) and connected using a system
bus or a crossbar."

For a technical description of the NuttX implementation of SMP,
see the NuttX `SMP Wiki
Page <https://cwiki.apache.org/confluence/display/NUTTX/SMP>`__.

.. c:function:: spinlock_t up_testset(volatile FAR spinlock_t *lock)

  Perform and atomic test and set operation on the provided spinlock.

  :param lock: The address of spinlock object.

  :return: The spinlock is always locked upon return. The value
    of previous value of the spinlock variable is returned,
    either SP_LOCKED if the spinlock was previously locked
    (meaning that the test-and-set operation failed to obtain the lock)
    or SP_UNLOCKED if the spinlock was previously unlocked
    (meaning that we successfully obtained the lock)

.. c:function:: int up_cpu_index(void)

  Return an index in the range of 0 through (CONFIG_SMP_NCPUS-1)
  that corresponds to the currently executing CPU.

  :return: An integer index in the range of 0 through
    (CONFIG_SMP_NCPUS-1) that corresponds to the
    currently executing CPU.

.. c:function:: int up_cpu_start(int cpu)

  In an SMP configuration, only one CPU is initially active (CPU 0).
  System initialization occurs on that single thread. At the
  completion of the initialization of the OS, just before
  beginning normal multitasking, the additional CPUs would
  be started by calling this function.

  Each CPU is provided the entry point to is IDLE task when started.
  A TCB for each CPU's IDLE task has been initialized and
  placed in the CPU's g_assignedtasks[cpu] list. A stack
  has also been allocated and initialized.

  The OS initialization logic calls this function repeatedly until
  each CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).

  :param cpu: The index of the CPU being started. This will be a
    numeric value in the range of from one to
    ``(CONFIG_SMP_NCPUS-1)``). (CPU 0 is already active).

  :return: Zero (OK) is returned on success; a negated errno value on failure.

.. c:function:: int up_cpu_pause(int cpu)

  Save the state of the current task at the head of the
  ``g_assignedtasks[cpu]`` task list and then pause task execution on the CPU.

  This function is called by the OS when the logic executing on
  one CPU needs to modify the state of the ``g_assignedtasks[cpu]``
  list for another CPU.

  :param cpu: The index of the CPU to be paused. This will not be
    the index of the currently executing CPU.

  :return: Zero (OK) is returned on success; a negated errno value on failure.

.. c:function:: int up_cpu_resume(int cpu)

  Restart the cpu after it was paused via up_cpu_pause(),
  restoring the state of the task at the head of the
  ``g_assignedtasks[cpu]`` list, and resume normal tasking.

  This function is called after ``up_cpu_pause()`` in order
  resume operation of the CPU after modifying its
  ``g_assignedtasks[cpu]`` list.

  :param cpu: The index of the CPU being resumed. This will not be
    the index of the currently executing CPU.

  :return: Zero (OK) is returned on success; a negated errno value on failure.

