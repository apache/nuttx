=============
Shared Memory
=============

Shared memory interfaces are only available with the NuttX kernel
build (``CONFIG_BUILD_KERNEL=y``). These interfaces support user
memory regions that can be shared between multiple user processes.
The user interfaces are provided in the standard header file
``include/sys/shm.h>``. All logic to support shared memory is
implemented within the NuttX kernel with the exception of two
low-level functions that are require to configure the
platform-specific MMU resources. Those interfaces are described
below:

.. c:function:: int up_shmat(FAR uintptr_t *pages, unsigned int npages, uintptr_t vaddr)

  Attach, i.e, map, on shared memory region to a user virtual address.

  :param pages: A pointer to the first element in a array of
    physical address, each corresponding to one page of memory.
  :param npages: The number of pages in the list of physical pages
    to be mapped.
  :param vaddr: The virtual address corresponding to the beginning
    of the (contiguous) virtual address region.

  :return: Zero (OK) is returned on success; a negated errno value is returned on failure.


.. c:function:: int up_shmdt(uintptr_t vaddr, unsigned int npages)

  Detach, i.e, unmap, on shared memory region from a user virtual address.

  :param vaddr: The virtual address corresponding to the beginning
    of the (contiguous) virtual address region.
  :param npages: T The number of pages to be unmapped.

  :return: Zero (OK) is returned on success; a negated errno value is returned on failure.

