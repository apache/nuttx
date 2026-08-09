=========================================
Migrating to separate ``fork``/``vfork``
=========================================

What changed
============

NuttX used to implement ``fork()`` and ``vfork()`` as the same function.  Both
were thin libc wrappers around a single ``up_fork()`` syscall; ``vfork()``
differed only by a trailing ``waitpid()``.  Underneath, the child joined the
parent's address environment -- the same ``addrenv_join()`` that
``pthread_create()`` uses -- and got a private *copy of the stack*.  So the
child shared ``.data``, ``.bss`` and the heap with its parent, and ran
concurrently with it.

That was not ``fork()``.  It was ``vfork()``-with-a-private-stack published
under ``fork()``'s name.  The history says so plainly: the ``fork()`` this
replaces was NuttX's old ``vfork()``, renamed in 2023 without any change of
behaviour.  And the consequence was silent: a program written against POSIX
``fork()`` compiled and ran, and its child's writes quietly landed in the
parent's variables.

There are now two distinct primitives, and each means what its name says:

.. list-table::
   :header-rows: 1
   :widths: 14 30 26 30

   * - API
     - Memory
     - Parent
     - Availability
   * - ``fork()``
     - child gets **its own copy** at the same virtual addresses
     - runs concurrently
     - ``CONFIG_ARCH_HAVE_FORK`` -- only where an address environment can be
       duplicated
   * - ``vfork()``
     - child **shares** the parent's memory
     - **suspended** until the child ``_exit()``\ s or ``exec()``\ s
     - ``CONFIG_ARCH_HAVE_VFORK`` -- no address environment needed

.. note::

   ``fork()`` is provided only where the architecture implements
   ``up_addrenv_fork()`` and therefore selects ``CONFIG_ARCH_HAVE_FORK``; it
   becomes available architecture by architecture as that hook lands.  Check
   ``CONFIG_ARCH_HAVE_FORK`` in your own configuration rather than assuming
   either way.  Where it is unset, ``vfork()`` is what the configuration
   offers.

This is a breaking change
=========================

Two things break, and they break loudly rather than quietly:

**Code calling** ``fork()`` **on a target without a duplicable address
environment no longer builds.**  ``fork()`` is not declared in ``unistd.h``
there, so you get a compile error naming the function.  That is the intended
outcome:  a build error is strictly better than the silent wrongness it
replaces.  Today that is every in-tree architecture, so every caller of
``fork()`` has to be looked at.

**Code calling** ``fork()`` **on a target that does have real** ``fork()``
**changes behaviour** -- from sharing to copying.  Code that (perhaps
unknowingly) relied on the sharing will now see the parent and child diverge.

Which replacement do I want?
============================

Answer the question "why did I call ``fork()``?".

*I want the child to run a different program.*
   Use :c:func:`posix_spawn` or ``task_spawn()``.  This is the single most
   common reason to call ``fork()``, NuttX has always provided a better answer
   for it, and that answer does not have the pid discontinuity that
   ``fork()``\ +\ ``exec()`` has.  If you must keep the two-step idiom, use
   ``vfork()`` + ``exec*()``:  that is exactly what ``vfork()`` is for, and
   unlike ``fork()`` it needs no duplicable address environment.

   .. code-block:: c

      #include <unistd.h>

      pid = vfork();         /* was: pid = fork(); */
      if (pid == 0)
        {
          execv(path, argv); /* or _exit() on failure */
          _exit(EXIT_FAILURE);
        }

   Note the restriction that comes with it:  between the ``vfork()`` and the
   ``exec()`` the child shares the parent's memory and runs on the parent's
   behalf, so it must not modify anything, must not return from the calling
   function, and must not call anything other than ``_exit()`` or an ``exec``
   family function.

*I want a second flow of control that shares my memory.*
   Use ``pthread_create()``.  That is the same memory relationship the old
   ``fork()`` gave you, spelled clearly, with a normal entry point instead of a
   function that returns twice.  There is no longer a returns-twice primitive
   with concurrent sharing semantics:  the old behaviour was not POSIX, and
   ``vfork()`` is not a drop-in for it -- the parent is suspended, so parent and
   child never run concurrently.

*I want a genuinely independent copy of this process.*
   Keep calling ``fork()``, and make sure your configuration selects
   ``CONFIG_ARCH_HAVE_FORK``.  Be aware there is no copy-on-write: the copy is
   eager, so forking a large process needs as much free memory as the process
   occupies and fails with ``ENOMEM`` otherwise.

Configuration symbols
=====================

``CONFIG_ARCH_HAVE_VFORK``
   Hidden.  The architecture can implement POSIX ``vfork()``.  Selected exactly
   where ``ARCH_HAVE_FORK`` used to be, so every configuration that had the old
   ``fork()`` has ``vfork()``.

``CONFIG_ARCH_HAVE_FORK``
   Hidden, ``depends on ARCH_ADDRENV``.  It no longer means "``fork()``
   exists"; it means "this configuration can provide POSIX ``fork()``
   semantics", which requires an address environment and an
   ``up_addrenv_fork()`` to duplicate it with.

Notes for architecture maintainers
==================================

The register/stack snapshot machinery is common to both primitives.  Each
architecture exposes one entry point, ``up_fork(bool vfork)``, whose argument
says which primitive the caller used and is handed to ``nxtask_setup_fork()``.
That is where the memory semantics are decided: ``addrenv_join()`` for
``vfork()``, ``addrenv_fork()`` for ``fork()``.

Adding real ``fork()`` to an architecture
-----------------------------------------

Two things are needed, and the second is the one that is easy to miss.

**Implement** ``up_addrenv_fork()``.  It duplicates an address environment:
allocate fresh pages, copy the parent's contents into them, and map them at the
*same* virtual addresses.  ``up_addrenv_clone()`` is not this -- it copies only
the representation and leaves both processes pointing at one set of page
tables.  Then give ``ARCH_HAVE_FORK`` a ``default y if <arch>`` line in
``arch/Kconfig``.

**Build the child from the caller's saved system call frame.**  In a kernel
build ``fork()`` is reached through a system call, so the return address and
stack pointer the architecture's fork entry point can observe for itself belong
to the *kernel*, not to the caller; a child built from those resumes at a
kernel address on a kernel stack.  The architecture must record the caller's
exception frame when it traps -- ``xcp.sregs`` is the field that exists for
this -- and build the child from that instead, while a kernel thread that calls
the entry point directly still takes the ordinary path.

Four architectures do it, and they are worth copying:

* RISC-V: ``riscv_swint.c`` stores the frame in ``xcp.sregs``, and
  ``riscv_fork.c`` rebuilds the child from it.
* arm64: ``arm64_vectors.S`` hands the frame to ``dispatch_syscall()``, which
  stores it in ``xcp.sregs``; ``arm64_fork()`` then dispatches to
  ``arm64_fork_syscall()`` or ``arm64_fork_direct()`` according to whether
  ``TCB_FLAG_SYSCALL`` is set, so a kernel thread that calls the entry point
  directly still works.
* armv7-a: ``arm_syscall.c`` stores the frame in ``xcp.sregs``, and
  ``arm_fork()`` dispatches to ``arm_fork_syscall()`` or
  ``arm_fork_direct()``.  The discriminator here is a saved user stack
  pointer, ``xcp.ustkptr``, rather than ``TCB_FLAG_SYSCALL``:  armv7-a
  dispatches a system call by re-pointing the caller's own exception frame at
  ``dispatch_syscall()``, so the caller *is* the task that runs the kernel
  side of the call.  What makes its snapshot useless is not the system call
  as such but the switch to the kernel stack, which leaves the kernel-side
  frames on a stack the child gets no copy of.  A build without a kernel
  stack dispatches on the caller's own stack, so there the frames are copied
  along with the caller's and ``arm_fork_direct()`` remains correct.  Because
  ``arm_syscall()`` has already re-pointed the frame by the time
  ``arm_fork()`` runs, its PC, CPSR and SP are the kernel's; the caller's are
  read from where ``arm_syscall()`` put them -- ``syscall[0].sysreturn``,
  ``syscall[0].cpsr`` and ``ustkptr``.
* x86_64: ``x86_64_syscall()`` stores the frame in ``xcp.sregs``, and
  ``x86_64_fork()`` dispatches to ``x86_64_fork_syscall()`` or
  ``x86_64_fork_direct()``.  The discriminator here is ``xcp.sregs`` itself
  being non-NULL, because raising ``TCB_FLAG_SYSCALL`` would also defer signal
  actions -- something x86_64 has never done and its kernel-build signal path
  does not currently survive.  Two properties of ``SYSCALL``/``SYSRET`` shape
  the child's frame:  the instruction leaves the caller's RIP and RFLAGS in
  RCX and R11 rather than on a stack, so they have to be moved into the RIP
  and RFLAGS slots of the interrupt frame the child is resumed from; and the
  hardware never records the caller's CS and SS at all -- ``SYSRETQ``
  reconstructs them from ``IA32_STAR`` -- so the child's have to be filled in
  with the user code and data selectors at RPL 3.  For the same reason the
  saved frame is not copied wholesale:  only the extended state and the
  general registers are inherited, and the segment registers and thread
  pointer come from the frame ``up_initial_state()`` built for the child.

Nothing else is required:  the ``up_fork()`` entry point and the libc wrapper
are already there and become live automatically.

Note that ``ARCH_HAVE_FORK`` is about a *per-process* address environment.  A
protected build has one address space carved up once at boot, whether the
boundaries are drawn by an MPU or by a fixed set of MMU mappings; its
``up_addrenv_*()`` are stubs, and there is no mapping to duplicate at the same
virtual addresses.  ``CONFIG_ARCH_ADDRENV`` being set is therefore not by
itself evidence that ``fork()`` can be provided.  ``vfork()``, which shares the
parent's memory, works there as everywhere else.

Known gaps
==========

``fork()`` **is gained one architecture at a time.**  The generic machinery is
complete -- ``addrenv_fork()``, the ``up_addrenv_fork()`` hook, the syscall, the
libc wrapper and the ``ostest`` case -- so an architecture provides ``fork()``
by implementing ``up_addrenv_fork()`` and selecting ``CONFIG_ARCH_HAVE_FORK``,
with no further generic work.

**A windowed ABI needs its stack rebased, not just copied.**  On Xtensa,
giving a child a relocated copy of the parent's stack takes more than the copy:
the register-window save areas embedded in the stack hold absolute stack
pointers, so each one has to be rebased along with the copy, or the child
reloads a pointer into the *parent's* stack on its very first window underflow.
That rebasing is architecture-specific and belongs with the Xtensa entry points
rather than here.

Note also on ``waitpid()`` after ``vfork()``
============================================

The ``vfork()`` parent is now resumed when the child's TCB is torn down, so by
the time it runs the child is completely gone.  Where the child called
``exec()`` this makes no difference -- ``exec_swap()`` has already given the
loaded program the child's pid, and that program is still running, so
``waitpid()`` behaves normally.  Where the child called ``_exit()``,
``waitpid()`` can only return its status if ``CONFIG_SCHED_CHILD_STATUS`` is
enabled; otherwise it returns ``ECHILD``, because NuttX does not retain the
status of a task that no longer exists.  That is a pre-existing property of
that configuration, not a change:  the previous implementation blocked in a
libc ``waitpid(WNOWAIT)`` and an application's own ``waitpid()`` afterwards hit
the same wall.
