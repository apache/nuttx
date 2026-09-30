=================
Architecture APIs
=================

The file ``include/nuttx/arch.h`` identifies by prototype all of
the APIs that must be provided by the architecture specific logic.
The internal OS APIs that architecture-specific logic must
interface with are also identified in ``include/nuttx/arch.h`` or
in other header files.

.. toctree::
  paging.rst
  shm.rst
  smp.rst
