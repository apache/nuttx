.. _capabilities:

====================
Process capabilities
====================

A process holds three capabilities. Without one, the calls it guards fail
with ``EPERM``. Every build, the kernel and init start with all three.
``CONFIG_SCHED_CAPABILITIES``, off with ``DEFAULT_SMALL``, builds the checks.

=================  ==================================================
``PR_CAP_RAWIO``   ``open()`` of block, MTD and BCH nodes,
                   ``mount()``, ``umount2()``
``PR_CAP_SPAWN``   ``posix_spawn()``, ``task_spawn()``,
                   ``task_create()``, ``exec()``, ``execve()``
``PR_CAP_ADMIN``   ``boardctl()`` reset and poweroff
=================  ==================================================

.. code-block:: c

  prctl(PR_CAPS_DROP, PR_CAP_RAWIO | PR_CAP_SPAWN);
  int caps = prctl(PR_CAPS_GET);

A drop is permanent. The set lives in the task group and a new group copies
its creator's. The checks sit in the internal functions, so kernel code
running for a process is held to that process's set. Kernel threads hold
all three.
