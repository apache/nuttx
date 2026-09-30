=========================
Toolchain support library
=========================

``libs/libbuiltin/`` holds the pieces that come from the toolchain rather
than from NuttX: the compiler's own runtime, and the instrumentation
libraries behind coverage and profiling.  It has two provider directories,
``libgcc/`` and ``compiler-rt/``, and three independent choices in Kconfig.

Builtin runtime
===============

The helper routines a compiler emits calls to for work the target has no
instruction for.

``CONFIG_BUILTIN_TOOLCHAIN``
   Link the toolchain's own builtin library to the OS.  This is the default.

``CONFIG_BUILTIN_COMPILER_RT``
   Compile LLVM's ``libclang_rt.builtins`` into the OS instead.  Note that it
   depends on ``CONFIG_ARCH_TOOLCHAIN_GNU``: it is for using LLVM's runtime
   from a GNU toolchain, not for Clang builds.

Code coverage
=============

Off by default -- ``CONFIG_COVERAGE_NONE`` -- with three ways to turn it on:

``CONFIG_COVERAGE_TOOLCHAIN``
   Link the toolchain's gcov library.  Needs a GCC toolchain and
   ``CONFIG_HAVE_CXXINITIALIZE``.

``CONFIG_COVERAGE_COMPILER_RT``
   LLVM's ``libclang_rt.profile``, for a Clang toolchain, which brings the
   ``-fprofile-*`` options with it.

``CONFIG_COVERAGE_MINI``
   A cut-down library for either toolchain.

``CONFIG_COVERAGE_ALL`` instruments every module, which the Kconfig warns
costs "a large performance penalty"; without it you add the compiler flags
to the one module you care about.  Data is written under
``CONFIG_COVERAGE_DEFAULT_PREFIX``, ``/data`` by default, and
``CONFIG_COVERAGE_GCOV_DUMP_REBOOT`` dumps it on reboot.  From a shell it is
``gcov dump -d <path>``.

Profiling
=========

``CONFIG_PROFILE_MINI`` enables gprof call graphs; it needs
``CONFIG_FRAME_POINTER`` and the ``-pg`` flag on whatever you want to
profile.  ``CONFIG_PROFILE_ALL`` applies it to every module, with the same
penalty as its coverage counterpart.  The default is
``CONFIG_PROFILE_NONE``.
