=================
Architecture APIs
=================

The file ``include/nuttx/arch.h`` identifies by prototype all of
the APIs that must be provided by the architecture specific logic.
The internal OS APIs that architecture-specific logic must
interface with are also identified in ``include/nuttx/arch.h`` or
in other header files.

.. toctree::
  conventions.rst
  arch.rst
  board.rst
  time_clock.rst
  wqueue.rst
  addrenv.rst
  nuttx.rst
  app_vs_os.rst
  smp.rst
  shm.rst
  paging.rst
  led.rst
  iob.rst
