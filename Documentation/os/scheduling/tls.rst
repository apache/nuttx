.. _thread-local-storage:

=========================
TLS: Thread Local Storage
=========================

Historical Background
=====================

Thread Local Storage (TLS) was originally implemented to support
pthread-specific data and a per-thread ``errno`` variable.

``errno``
=========

The user ``errno`` value was originally kept in the TCB and could be accessed
via an OS system call called ``__errno()``:

.. code-block:: c

  # define errno *__errno()

The ``errno`` value was kept in the thread's TCB in the ``pterrno`` field.
The ``__errno()`` function finds the TCB associated with the current thread
and returns a pointer to that errno storage location.

This worked in the FLAT build mode because it dereferenced the address
of the errno in TCB as both an ``RVALUE``:

.. code-block:: c

  errcode = errno;

And as an ``LVALUE``:

.. code-block:: c

  errno = errcode;

That works, however, it is rather inefficient.

However, that will not work at all in the PROTECTED or KERNEL build modes
because the memory holding the TCB is protected and cannot be directly
accessed from the user mode application.
Instead, it is accessed through accessor functions:

.. code-block:: c

  void set_errno(int errcode);
  int  get_errno(void);

Accessing the ``errno`` via these functions in PROTECTED or KERNEL mode
is a huge performance hit, because the calls must go through a system call
which is implemented as a software interrupt.

And in this case, the errno is defined to be:

.. code-block:: c

  # define errno get_errno()

This works fine as an ``RVALUE``:

  errcode = errno;

But will cause a compilation error if used as an ``LVALUE``:

.. code-block:: c

  errno = errcode;

Instead user code must explicitly call ``set_errno(errcode)``.
This a a violation of the accepted usage of the POSIX ``errno`` variable.

TLS-Based ``errno``
-------------------

The current solution has moved the ``errno`` storage location out of the
protected TCB memory and into the unprotected application thread's stack
memory. Then it can be accessed with the appropriate TLS
(Thread Local Storage) interfaces.


Task-Specific Globals in the FLAT and PROTECTED builds
======================================================

A special case is the use of TLS to support task-specific
(vs. thread specific) global data.

Task-specific differs from thread-specific in that all of the threads
in the task group share the global data, however, each task group has
its own set of task-specific global variables.

This feature permits a high-fidelity emulation of process global data
as you would see in the KERNEL build where each process has its own copy
of all global variables.

Task-specific data is implemented by keeping such globals in the stack
of the main thread. A single main thread is always present and, hence,
provides user-accessible, per-task storage.

Task-specific data is accessed like TLS except that the task-specific data
is accessed from the stack of the main thread,
rather than the stack stack of the main thread.


Re-Entrant ``getopt()``
=======================

One use of task-specific data is for global variables used by the ``getopt()``
function. ``getopt()`` is an important C library function used by applications
for parsing of task command line parameters.

It is, however, not re-entrant due to the use of global variables:
``optarg``, ``opterr``, ``optind``, and ``optopt``.
There are several, non-standard implementations for a re-entrant version
of called ``getopt_r()``, however, these are all wildly different
and do not conform to any standard.

.. important:: Non-standard interfaces are not desirable in NuttX.

In the FLAT and PROTECTED builds, this non-reentrant limitation
poses a problem. Unlike the KERNEL build mode, there is one instance
of of the globals ``optarg``, ``opterr``, ``optind``, and ``optopt``
shared across all task groups.
It is not a problem in the KERNEL build where there is a separate instance
of the globals in each process address space.

Using task-specific data, however, it is possible to make ``getopt()``
thread-safe. This amounts to keeping the ``getopt()`` globals in the main
thread's TLS so that there is an individual copy for each thread.
That would keep both the standard form of the ``getopt()`` interfaces
as well as making ``getopt()`` full re-entrant with respect to task groups.


Unaligned TLS
=============

Historically, TLS worked by aligning the stack base address then
simply AND'ing the current stack pointer to obtain the base address
of the stack where the TLS data can be found (as struct ``tls_info_s``).
This mechanism is very efficient because no OS system call is required,
as application logic can obtain the TLS data directly form its stack
via AND'ing and casting.

This works well in the KERNEL build mode where the stack resides at highly
aligned virtual address, but does not work well in FLAT and PROTECTED modes:

1. The alignment must be large. That is because it also determines
   the maximum size of the thread's stack. If the size of the thread's stack
   exceeds the maximum value determined by the alignment, then
   the AND operation will alias to the wrong address.
2. If all of the addresses are highly aligned then
   (1) you need to have much more memory available for stack allocation.
   This is because (2) the large alignment causes bad memory fragmentation
   and degraded use of memory.

An alternative way to get the stack base address is to call into the OS
to get the unaligned stack base address.
This involves a system call, but is more usable that other alternatives
in the FLAT and PROTECTED modes.

We have implemented the configuration: ``CONFIG_TLS_ALIGNED`` that selects
the legacy aligned stack for TLS access. If this is not defined, then
the new unaligned stack TLS logic is used.
We have also implemented aligned and unaligned TLS support
for every architecture.

Addition implementation steps:

1. Moved the ``errno`` storage location out of the TCB and in TLS
   (into the struct ``tls_info_s``).
2. Modified the ``errno`` access definitions and logic that was
   in ``sched/errno`` to use the TLS logic in user space.
   That logic now resides in ``libs/libc/errno`` since it is now
   a user library interface, not a core OS interface.
3. TLS is now enabled by default. It is enabled in the unaligned mode
   for the FLAT and PROTECTED build modes but in the highly efficient
   aligned mode for KERNEL build mode.
4. There are no longer an OS system calls related to the error
   (with the exception of a call to get the struct ``tls_info_s``
   in the unaligned TLS mode).


Push-Up vs. Push-Down Stacks
============================

TLS data is always located at the beginning thread's stack.
This is true for both CPUs with push-up stacks and CPUs with push-down stacks.
This location required in order to access the TLS by ANDing the aligned stack
pointer address.
The stack memory maps, differ only in the usage of the available stack::

     Push Down          Push Up
  +-------------+   +-------------+ <- Stack memory allocation
  |  TLS Data   |   |  TLS Data   |
  +-------------+   +-------------+
  | Task Data*  |   | Task Data*  |
  +-------------+   +-------------+
  |  Arguments  |   |  Arguments  |
  +-------------+   +-------------+ |
  |             |   |             |  v
  | Available   |   | Available   |
  |   Stack     |   |   Stack     |
  |             |   |             |
  |             |   |             |
  |             | ^ |             |
  +-------------+ | +-------------+

\*) Task data is allocated in the main's thread's stack only


TLS interfaces
==============

TLS is a non-standard, but more general interface.
It differs from pthread-specific data only in that its semantics are general;
the semantics of the pthread-specific data interfaces are focused on pthreads.
But they really should share the same common underlying logic.

Currently, there are four TLS interfaces:

.. code-block:: c

  int tls_alloc(void);
  int tls_free(int tlsindex);
  uintptr_t tls_get_value(int tlsindex);
  int tls_set_value(int tlsindex, uintptr_t tlsvalue);

as prototyped and documented in ``include/nuttx/tls.h``.

These interfaces are basically adaptations from the Windows TLS interfaces
which are directly analogous to the POSIX pthread-specific data interfaces,
adapted to NuttX coding standards (see, for example, links available at
https://docs.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-tlsalloc
).

There are no POSIX TLS interfaces and Linux (actual GLIBC) does not provide
a good mechanism. It relies on a storage class that is specific
to ELF binaries. That is not useful in an embedded system where
no ELF information is present.


pthread-specific data
=====================

Pthread-specific data is another mechanism for accessing thread-specific data.
It consists of these POSIX standard interfaces:

.. code-block:: c

  int pthread_key_create(FAR pthread_key_t *key,
                         CODE void (*destructor)(FAR void *));
  int pthread_setspecific(pthread_key_t key, FAR const void *value);
  FAR void *pthread_getspecific(pthread_key_t key);
  int pthread_key_delete(pthread_key_t key);

This, historically, was separate from TLS data and managed internal
to the OS. pthread-specific data was only accessible through OS system calls.

But it is  a natural extension of the TLS logic to support both
non-standard TLS access and standard pthread-specific data access.
This would amount to:

1. Moving the pthread-specific storage out of the TCB
   and into the TLS data.
2. Moving the pthread-specific data accessors out of ``sched/pthread``
   and into ``libs/libc/pthread``.
3. Remove the pthread-specific data system calls.


Accessing the ``errno`` from kernel space
=========================================

.. important:: We should never access the errno from kernel space.

This is especially dangerous from kernel space because we can't be certain
which user address environment is in effect or which stack is being used
(if we were to use separate kernel mode stacks in the future as Linux does,
for security reasons, etc).

Most OS interfaces have two parts:

1. A user callable function, say ``osapi()`` that sets the ``errno`` value
   is necessary (and may implement cancellation points) and
2. An internal OS version which might then be ``nx_osapi()`` which
   does not modify the ``errno`` value.

Ideally, all of the user callable OS interfaces (the ``osapi()``) should be
moved to the location in ``libs/libc`` the system call (``sycall/``) should be
redirected to ``nx_osapi()``.

A complication to doing this is that the OS interfaces which are also
cancellation points call special internal interfaces,
``enter_cancellation_point()`` and ``leave_cancellation_point()``
that are not available from user space.
So some addition partitioning would be need to accomplish this.

This separation of user and OS interfaces has not been implemented
as of this writing.


Wild Ideas
==========

The primary beneficiary of TLS is the C library.
**The TLS interfaces are non-standard and should not be used directly
by any portable application code.**
However, these non-standard TLS interfaces can be used within the C library
to implement improved, standard user interfaces.

Local Process ID
----------------

Another data item that should, eventually, be included in the TLS data
is the process ID (``pid``) of the currently executing thread.
In some analysis of PROTECTED and KERNEL builds, it was found that
``getpid()`` was the most highly accessed OS interface.
By moving the PID into TLS, we could eliminate this system call overhead
(at least in the aligned TLS case).
The same mechanism would be used by ``pthread_self()`` which, in NuttX,
would be the equivalent function, but following pthread semantics.

Streams
-------

In NuttX, C buffered I/O streams are represented with a pre-group array
of type FILE. A stream is accessed from the OS via the
``nxched_get_streams()`` interface (system call).
The streams array is not used by OS and would be another candidate
to move into TLS.

.. note:: In retrospect, this is not such a good idea because the I/O streams
          are not unique per-thread; they are common across all threads within
          a task group: That includes main thread of the group and all child
          pthreads whose parent, grandparent, of great-grandparent is the
          main thread. They all share the same I/O stream array.
          As a result, I/O streams must be one per task-group.

          This works now because the I/O stream array is a part
          of the internal OS group structure which is one per task group.
          While it would be nice to get the I/O stream around out of the OS,
          TLS is probably not the right solution to do that.


Code References
===============

Things to look at:

* ``include/errno.h`` - Defines current errno access.
* ``include/nuttx/tls.h`` - Defines the tls_info_s structure.
* ``include/nuttx/sched.h`` - Group related TLS structures.
* ``libs/libc/errno`` - The TLS-based errno logic.
* ``libs/libc/tls`` - The implementation of the most TLS interfaces.
* ``sched/group`` - Group-related implementation of certain TLS interfaces.
* ``libs/libc/pthread`` - The implementation of the pthread-specific dta interfaces.

