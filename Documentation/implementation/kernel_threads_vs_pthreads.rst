.. _kernel-threads-vs-pthreads:

===========================
Kernel Threads vs. Pthreads
===========================

Why Can't Kernel Threads Have pthreads?
=======================================

Kernel threads are special "tasks" that reside within the OS.
They are similar to application tasks, so why can't they have pthreads
like application tasks?


The FLAT build
==============

In the FLAT build, kernel threads are, in fact, identical to application
threads except that:

1. They follow some slightly different syntax, AND
2. Can only use OS internal interfaces, never application interfaces.

Since they are otherwise identical, there is really no technical reason
why pthreads could not be supported.

The real reasons in this case is:

1. It is inappropriate, AND
2. It is incompatible with the PROTECTED and KERNEL builds where pthreads
   cannot be supported.

Why inappropriate?  Because..

pthread Interfaces Are User Interfaces
--------------------------------------

In all Unix systems, pthread interface support is not provided
by the operating system but rather by the user-facing C library.
The role of the OS is only to provide some low level hooks needed by
C library implementation of pthread interfaces.
But all of the implementation is in the C library and only for use
by applications.

That is not the current situation in NuttX.
Currently pthread support is deeply entangled in the OS.
This is problem that must be fixed someday and must not exploited as some
permanent feature of the OS. It is not.
It will go away some day when NuttX becomes a mature Unix-family system.

As user facing interfaces, pthread interfaces include some behaviors that
are not appropriate within the OS.
Inappropriate behaviors include modification of the ``errno`` value and
cancellation points. Those are user-only features.
While pthread interfaces do not, in general, modify the errno setting
they do create cancellation points which is not desirable within the OS.

PROTECTED and KERNEL Builds
---------------------------

The PROTECTED and KERNEL builds differ from the FLAT build in that they
segregate the memory into two regions:
A privileged kernel address space and unprivileged, user address space(s).

The primary difference is that the PROTECTED build uses a physical address
space and different regions of the physical address space have different
properties. This is usually accomplished using a Memory Protection Unit (MPU).
There is one protected kernel address space and one unprotected
user address space.

The KERNEL build, on the other hand, uses a Memory Management Unit (MMU)
to create a virtual address space in which there is still a separate
protected kernel address space, but many user address spaces for user
programs. These are usually called processes.
This is the familiar build model that you find with high-end Unix-like
systems such as Linux.

See Memory Configurations for additional information.

Address Spaces, Memory Allocators, and User Mode
------------------------------------------------

What would happen if you tried to create a pthread in the PROTECTED or KERNEL
build? The effect of these address space differences become very pronounced.
pthreads are, by their very definition, user-space threads.
That means that the OS will attempt to create a user-space environment
for the pthread: The thread's stack and other resources.

And when the pthread is started, it will be started in user mode,
not kernel mode.

It would require a significant change to the OS to alter that behavior
and that is not under consideration (because the change is also inappropriate).

Entry points
------------

Because the application space and the kernel space are separately built
and separately linked in the PROTECTED and KERNEL builds.
No application addresses are known by the kernel and no kernel addresses
are known by the application (applications interface via system call traps,
not C function calls). So what address would the kernel thread provide
for the pthread entry point? A known kernel space address?

Of course, the system would crash with a memory fault immediately
if a protected address were executed in user mode.

Mutexes
-------

Okay, so there are no kernel pthreads. But could other pthread resources
be usable in the kernel. The short answer is no.

Consider mutexes, for example. By definition, a mutex can only be used within
threads of the current process (or task groups as they are often called
in NuttX). They have no meaning outside of the task group.

Since the kernel thread "task group" can have no pthreads,
how could these mutexes be used under the proper standard definition
of what a mutex must be?

This is true of all pthread interfaces: None of the pthread interfaces were
intended from inter-process communications; only for inter-pthread
communications within the same process (task group).


Roadmap
=======

As alluded to before about the appropriateness of pthread in the OS.
There is a roadmap for these kinds of features.
That roadmap is to continue to conform strictly to the standard OS definitions
of <Opengroup.org>_ and to continue to evolve as a fully compliant,
very standard, tiny Unix-like operating system.

1. Part of this is getting all user interfaces out of the OS.
2. Another part is migrating all pthread support out of operating system
   and into the user-facing C library.
3. An additional objective on the roadmap is to streamline the kernel threads
   and to disconnect them from task groups.
   Kernel threads should not be part of any task group.

In regard to this last point, the following is taken from Apache NuttX
Issue 1108: "Remove streams from Kernel Threads".

No Streams in Kernel Threads
----------------------------

Kernel threads are not permitted to use the C library buffer I/O, "stream",
interfaces. Those are interfaces like ``fopen()``, ``fread()``, ``fwrite()``,
``fclose()``, etc.
Those are strictly for use by user applications.
This is because these functions modify errno variables and create
cancellation points and perhaps other things that are undesirable
within the OS.

The thread create logic is largely the same for both kernel threads
and for application threads: They both allocate a large buffer
from the user memory pool for the stream ``FILE`` array.
Since kernel threads this is a waste of memory since the kernel threads
should not be using streams.

Remove Kernel Thread Stream Allocation
--------------------------------------

The proposal is to:

1. Verify that there is no use of streams within the OS,
2. Remove the stream allocation for kernel threads,
3. Assure that there are proper checks in place so that there are
   no uses of the ``NULL`` stream array pointer.

For the most part, the OS is clean, there are essentially no use
of streams within the OS.
There are, however, a few violations of this that will need to be fixed;
``fopen()`` is called in some of the ``lc3850`` code.

Remove the File Descriptor Array Too
------------------------------------

A second phase would be to remove the file descriptor array from kernel
threads. File descriptors are, again, only for use by applications;
within the OS, file access is done using the struct file (aka, ``FILE``),
structure directly.
However, I suspect that there are many hidden uses of file descriptors
in the system.
For one, ``file_open()`` which opens the detached file, is not fully
implemented; it cheats and uses file descriptors.

So let's consider removal of the file descriptor allocation
as a second step after the stream allocations have been removed.
