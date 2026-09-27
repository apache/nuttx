=====================
Shared Memory Support
=====================

Prerequisites
-------------

These features must be enabled before shared memory support can be
provided:

- ``CONFIG_ARCH_ADDRENV=y`` - Support for per-task address environment using a
  MMU.
- ``CONFIG_BUILD_KERNEL=y`` - Support for protected kernel-/user-space memory
  regions must be provided by the MMU.
- ``CONFIG_GRAN=y`` - The granule allocation is the allocation underlying all
  paged allocations.
- ``CONFIG_MM_PGALLOC=y`` - Enables the physical page allocator
- ``CONFIG_MM_PGSIZE`` - Determines the size of one page that can be mapped by
  the MMU.

And then finally:

- ``CONFIG_MM_SHM=y`` - Enables shared memory support.  Kconfig enforces two
  of the prerequisites above for you: it carries ``depends on MM_PGALLOC &&
  BUILD_KERNEL``, and it selects ``CONFIG_ARCH_VMA_MAPPING``.
- ``CONFIG_ARCH_SHM_VBASE`` - The virtual address of the beginning of the
  shared memory region.
- ``CONFIG_ARCH_SHM_MAXREGIONS`` - The maximum number of regions that can
  allocated for the shared memory space.  This hard-coded value permits
  static allocation of the shared memory data structures and serves no
  other purpose. Default is 1.
- ``CONFIG_ARCH_SHM_NPAGES`` - The maximum number of pages that can allocated
  for the shared memory region.  Default is 1.

The option has grown past SysV shared memory: its Kconfig help also covers
device mapping interfaces such as ``vm_map_region()``, and
``CONFIG_ARCH_SHM_NPAGES`` is described there as the maximum size of a
userspace VM mapping, shared memory being only one use of it.

The size of the virtual shared memory address space is then determined by
the product of the maximum number of regions, the maximum number of pages
per region, and the configured size of each page.

Concepts
--------

Each process has a task group structure, struct task_group_s, that holds
information common to all threads in the group.  If ``CONFIG_MM_SHM=y``, then
this includes data structures for the per-process shared memory virtual
page allocator.

A memory region is accessed using:

.. code-block:: C

    int shmget(key_t key, size_t size, int shmflg);

by a lookup using internal shared memory data sets with key as the lookup
match value.  On success, shmget returns the shared memory identifier for
the match -- in this implementation that identifier is simply the table
index of the match.

If the memory region does not exist, it may also be created by shmget (if
the IPC_CREAT bit is set in the shmflag).  When a shared memory region is
created, the following things happen:

- A new entry is set aside in the internal data set.  The key value is
  assigned to the entry and the table index is the new shared memory
  identifier.

- The requested size is rounded up to rounded up to full pages, each of
  size ``CONFIG_MM_PGSIZE``.

- A set of physical pages are allocated and the physical address of
  these pages is retained in the internal data set.

Now the key maps to and shared memory identifier (the table index) and
the table index provides access to the list of physical pages making up
the shared memory region.

NOTE: An improved implementation my perform a "lazy" back up of the
physical memory, i.e., do not allocate the physical memory until the
memory is required, for example, when a page fault occurs when a
application tries to allocate the memory.

A shared memory region is destroyed via:

.. code-block:: C

    int shmctl(int shmid, int cmd, struct shmid_ds *buf);

In order for a process to make use of the memory region, it must be
"attached" the process using:

.. code-block:: C

    FAR void *shmat(int shmid, FAR const void *shmaddr, int shmflg);

``shmat()`` returns the virtual address where the shared memory can be found
in the user process.  Attaching the shared memory region involves the
following steps:

- Use the shmid as a table index to look up the mapping in the shared
  memory internal data structures.

- Allocate a virtual address spaces of the same size as the physical
  address space using the per-process virtual shared memory virtual
  page allocator that can be found in the calling process's task group
  structure.

- Use platform specific interfaces to mapy the physical memory to the
  selected virtual address space, and

- Return the allocated virtual base address to the caller.

The memory region can be detached from the user process using:

.. code-block:: C

    int shmdt(FAR const void *shmaddr);

Relevant header files
---------------------

- ``include/sys/shm.h`` - Shared memory interface declarations
- ``include/sys/ipc.h`` - Provides additional definitions used by the shared
  memory interfaces
- ``include/nuttx/addrenv.h`` - Defines the virtual address space of the
  process.
- ``include/nuttx/pgalloc.h`` - Page allocator interfaces
- ``mm/shm/shm.h`` - Internal shared memory definitions.  This includes the
    definitions of the internal shared memory data structures.

Architecture Interface
======================

A port that supports shared memory has to provide these.  They are called
by the OS, never by an application.

Shared memory interfaces are only available with the NuttX kernel
build (``CONFIG_BUILD_KERNEL=y``). These interfaces support user
memory regions that can be shared between multiple user processes.
The user interfaces are provided in the standard header file
``include/sys/shm.h``. All logic to support shared memory is
implemented within the NuttX kernel with the exception of two
low-level functions that are required to configure the
platform-specific MMU resources. Those interfaces are described
below:

.. c:function:: int up_shmat(FAR uintptr_t *pages, unsigned int npages, uintptr_t vaddr)

  Attach, that is map, a shared memory region to a user virtual address.

  :param pages: A pointer to the first element in a array of
    physical address, each corresponding to one page of memory.
  :param npages: The number of pages in the list of physical pages
    to be mapped.
  :param vaddr: The virtual address corresponding to the beginning
    of the (contiguous) virtual address region.

  :return: Zero (OK) is returned on success; a negated errno value is returned on failure.

.. c:function:: int up_shmdt(uintptr_t vaddr, unsigned int npages)

  Detach, that is unmap, a shared memory region from a user virtual address.

  :param vaddr: The virtual address corresponding to the beginning
    of the (contiguous) virtual address region.
  :param npages: The number of pages to be unmapped.

  :return: Zero (OK) is returned on success; a negated errno value is returned on failure.
