.. _tasks-vs-threads:

=================
Tasks vs. Threads
=================

What is the difference between a thread and a task in Nuttx?
============================================================

Tasks and threads in NuttX try to emulate processes and threads
in the standard Unix environment.
I think of a process as a "container" of resources that are shared
by the threads that execute within the context of the process.

The process has one special thread, the "main" thread.
This is the special thread that is started when the process was created.

Since there are no processes in NuttX, the distinction is not so simple.
But in a similar way, tasks under NuttX have mostly private resources:
Their own file descriptors, their own ``FILE`` streams, their own ``errno``
variable, there own environment, etc.

Threads under NuttX, on the other hand, share task resources in the same way
that threads within the same process share the process resources.

The task resources were created when a task was created (``task_create()``,
``execv()``, ``posix_spawn()``, etc.).
When the task creates new ``pthreads``, those pthreads share the resources
of the parent task.
There is little to distinguish the main thread in NuttX.
All threads are essentially equal (as they were in old LinuxThreads library).

All task resources that are shared amongst threads reside in a "break-away",
reference-counted structure called struct ``task_group_s``.
The Task Control Block (TCB) of each thread that is a member of the task group
holds a reference to the same instance of this breakaway structure
(see ``include/nuttx/sched.h``).

The reference count of the shared,
task group resources is initialized to one when the task is created
(via ``task_create()``, for example); that task is the first member
of the task group.
The reference count is incremented when each additional thread is created
(via ``pthread_create()``) and joins task group.
The reference count equal to the number of members of the task group.

If any thread in the task group creates a new task (vs. thread),
that establishes a new task group but has no effect on the creator's
task group. The first task creates the task group;
the group membership only grows when threads are created via
``pthread_create()``.

As threads exit, they leave the task group and the reference count
on the struct ``task_group_s`` resources used by the thread is decremented.
When the reference count goes to zero, there are no members in the task group
and so the task group resources are finally deleted.
In this way, the life of a task resource begins when the task is created
and persists until the last thread in the task group exits.

Here is the definitive list of what is shared. Most of these resources
reside within the task group structure struct ``task_group_s``:

* Child task exit status.
* Pthread join data.
* Environment variables.
* File descriptors.
* FILE streams.
* Sockets.
* Opened message queues.
* ``pthread`` keys.
* Support data for ``atexit()``, ``on_exit()``, and/or ``waitpid()``.

The exception is the PIC address space used with NXFLAT.
It currently has its own data structure (struct ``dspace_s``,
but logically also belongs in ``task_group_s``):

* PIC data space and address environments.

.. note::

  The ``errno`` is not one of the shared resources.
  In NuttX ``errno`` is thread-private; each thread has
  its own ``errno``. But for bug-for-bug compatibility, the ``errno``
  variable really should also be moved into ``task_group_s``.

NuttX tasks and threads are discussed in much more detail in the
:ref:`nuttx-tasking` section.


When would I want to use a task? When would I want to use a thread?
===================================================================

Threads are very light weight; the memory cost for the thread is the cost
of the task control block plus the size of the stack.
That is perhaps 0.8-2KB, depending mostly on the configured stack size.

The memory cost of a task is higher because it includes most of the costs
of a pthread plus the cost of the task group structure.
The cost of the task group structure varies with such things as:

* The number of file descriptors you have configured (controlled by
  ``CONFIG_NFILE_DESCRIPTORS``),
* The number of streams that you have configured for C buffered I/O
  (controlled by ``CONFIG_NFILE_STREAMS``),
* The size of the I/O buffer configured for for each stream
  (controlled by ``CONFIG_STDIO_BUFFER_SIZE``). This setting is especially
  important because there is an I/O buffer for each opened stream.
* The number of sockets that you have configured
  (controlled by ``CONFIG_NSOCKET_DESCRIPTORS``).
* The size of dynamic data such as environment variables
  (size is determined by your usage).

The size of task group can then become largely depending upon how these things
are configured, typically in the range 0.5-1KB.

So why would you ever use a task if pthreads use so much less memory?
**Pthreads and tasks share resources. So they are not independent.**
There is strong coupling between the threads in a task group
and what one thread does can affect the behavior of another thread.

Normally, each major block of functionality is implemented with a separate
task. With each of those ``_task group_s``, however, you may also want several
helper threads to implement asynchronous and concurrent behaviors.


How do signals work in a task group with many pthreads?
=======================================================

The behavior of signals in the multi-thread task group is complex.
NuttX emulates a process model with task ``group_s`` and follows the POSIX
rules for signalling behavior.

Normally when you signal the ``_task`` group you would signal using
the task ID of the main task that created the group (in practice, a different
task should not know the IDs of the internal threads created within
the task group); that ID is remembered by the task group
(even if the main task thread exits).

Here are some of the things that should happen when you signal
a multi-threaded task group:

* If a task group receives a signal then one and only one indeterminate thread
  in the task group which is not blocking that signal will receive the signal.
* If a task group receives a signal and more than one thread is waiting
  on that signal, then one and only one indeterminate thread out of that
  waiting group will receive the signal.

You can mask out that signal using ``sigprocmask()``
(or ``pthread_sigmask()``). That signal will then be effectively disabled
and will never be received in those threads that have the signal masked.
On creation of a new thread, the new thread will inherit the signal mask
of the parent thread that created it.

So you if block signal signals on one thread then create new threads,
those signals will also be blocked in the new threads as well.

You can control which thread receives the signal by controlling
the signal mask.
You can, for example, create a single thread whose sole purpose
it is to catch a particular signal and respond to it:
Simply block the signal in the main task; then the signal will be blocked
in all of the pthreads in the group too.

In the one "signal processing" pthread, enable the blocked signal.
This thread will then be only thread that will receive the signal.


How do ``atexit()`` and ``on_exit()`` work with task groups?
============================================================

``atexit()`` and ``on_exit()`` callbacks must be registered using the task ID
of the main task (which makes sense since pthreads do not have task IDs
of type ``pid_t``).
The callback is not necessarily made when the task thread exits;
the ``atexit()`` and ``on_exit()`` callbacks will be executed when
the task group terminates, that is, when the final thread
of the task group terminates.


How does ``waitpid()`` work with task groups?
=============================================

In a single-thread task group, ``waitpid()`` will wait until the single,
main thread of the task group exits (i.e., the one created
by ``task_start()``). This is the intuitive behavior.
But the behavior may be less intuitive for multi-threaded task groups.
In a multi-threaded task group ``waitpid()`` will wait until
every thread in the task group exits.
Nothing special happens when the main thread of the task group exits.


What are privileged threads? How are threads handled differently when NuttX is built as a Kernel
================================================================================================

NuttX supports a build mode where it is built as a monolithic kernel.
This mode is selected with the configuration option
``CONFIG_BUILD_PROTECTED=y`` and is currently only supported
for a few architectures.

When used this way, NuttX is built as a separate kernel mode "blob"
and the applications are built as another separate user mode "blob".
The kernel runs in kernel mode and the applications run in user mode
(with the MPU restricting user mode accesses).
Access to the kernel from the user blob is only via system calls (SVCalls).

Thread and tasks that execute within the user mode "blob"
are all unprivileged, user mode threads. Exactly what "unprivileged"
means depends upon the memory protection architecture.
But it generally means that there are regions of memory where unprivileged
threads are prohibited from reading and/or writing data and/or from executing
code. Tasks created in the user-space are unprivileged;
all pthreads are unprivileged.

Certain threads are created within the kernel space to perform OS housekeeping
operations. Those are referred to as "kernel threads".
They are essentially the same as user-mode tasks but run in a privileged mode
and have full access to all of the restricted resources.
