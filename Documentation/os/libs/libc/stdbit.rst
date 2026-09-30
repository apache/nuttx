========
stdbit.h
========

The optional C23 ``stdbit.h`` header provides bit manipulation macros
(endianness, leading/trailing zeros and ones, count, single-bit test,
bit width, bit floor, bit ceil). NuttX provides this header only when
explicitly enabled via Kconfig.

Configuration
=============

- **CONFIG_ARCH_HAVE_STDBIT_H** (bool, selected by arch)
  Architecture indicates it provides ``arch/<arch>/include/stdbit.h``.

- **CONFIG_ARCH_STDBIT_H** (bool "stdbit.h", depends on ARCH_HAVE_STDBIT_H)
  Use the redirecting header. The build copies
  ``include/nuttx/lib/stdbit.h`` to ``include/stdbit.h``; that header
  then includes ``<arch/stdbit.h>`` when this option is set.

- **CONFIG_LIBC_STDBIT_GENERIC** (bool "stdbit.h (generic C23)")
  Use the generic C23 implementation. The same redirecting file
  ``include/nuttx/lib/stdbit.h`` is copied to ``include/stdbit.h``,
  and the generic implementation is used (no arch header). Requires
  compiler builtins (e.g. ``__builtin_clz``, ``__builtin_ctz``,
  ``__builtin_popcount``); see ``CONFIG_HAVE_BUILTIN_*`` in
  ``nuttx/compiler.h``.

Either **CONFIG_ARCH_STDBIT_H** or **CONFIG_LIBC_STDBIT_GENERIC** may be
enabled so that ``#include <stdbit.h>`` is available.
