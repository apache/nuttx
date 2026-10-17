============================================
``sandbox`` Protected-Build Containment Test
============================================

``sandbox`` checks that a protected build or a kernel build contains a user
process that makes a forbidden access: the process is terminated, and the
rest of the system keeps running. Each target has the result that it
expects, so the test fails a build that refuses all accesses and a build
that permits all accesses.

Usage::

  sandbox [r|w] [target ...]

``r`` (the default) reads the target and ``w`` writes it. With no target,
the four named targets run:

=============  ===========================  ============
Target         Address                      Expected
=============  ===========================  ============
``self``       data of the process          must succeed
``kernel``     kernel memory                must fault
``periph``     a peripheral register        must fault
``unmapped``   an address with no mapping   must fault
``0x...``      a literal address            must fault
=============  ===========================  ============

``self`` is the control. Without it, a build that refuses every access
passes all other checks. A target with no address in the configuration is
reported and not run.

For each target, ``sandbox`` starts itself again with ``posix_spawn()`` as
a separate process, which makes the access. Before the access, that process
allocates ``CONFIG_EXAMPLES_SANDBOX_ALLOC`` bytes, writes all of them, and
opens a file. ``sandbox`` then checks that:

- the process is terminated (or, for ``self``, exits normally),
- another thread of ``sandbox`` kept running,
- the memory in use, read from ``/proc/meminfo``, went up while the process
  lived and came back after it was reaped,
- ``/proc/<pid>/group/fd`` showed open descriptors while the process lived,
  and the group is gone after.

The exit status is the number of targets that failed. The test needs
``CONFIG_FS_PROCFS``.

Configuration
=============

- ``CONFIG_EXAMPLES_SANDBOX`` -- Enable the test.
- ``CONFIG_EXAMPLES_SANDBOX_ALLOC`` -- Bytes that the process holds when it
  dies. Default 65536.
- ``CONFIG_EXAMPLES_SANDBOX_KERNEL_ADDR`` -- A mapped kernel address. A
  protected build can leave it at 0, because ``CONFIG_NUTTX_USERSPACE``
  gives the boundary. 0 means that the target is not available.
- ``CONFIG_EXAMPLES_SANDBOX_PERIPH_ADDR`` -- A peripheral register that user
  code must not reach. 0 means that the target is not available.
- ``CONFIG_EXAMPLES_SANDBOX_UNMAPPED_ADDR`` -- An address with no mapping.
  0 means that the target is not available.
- ``CONFIG_EXAMPLES_SANDBOX_PROGNAME``, ``CONFIG_EXAMPLES_SANDBOX_PRIORITY``
  and ``CONFIG_EXAMPLES_SANDBOX_STACKSIZE`` -- Program name, priority and
  stack size.

:ref:`esp32s3-devkit:kernel_oct <platforms/xtensa/esp32s3/boards/esp32s3-devkit/index:kernel_oct>`
enables the test with all three addresses and shows a run.
