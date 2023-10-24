=====================================
``arch_libc`` Arch-specific libc Test
=====================================

This is a test for arch-specific libc function. Arch-specific libc functions are often implemented in
assembly language, here is the test for these functions. The test focuses on key features in assembly
language, including aligned access, speed, callee saved register check and so on.
Currently, the test only contains a subset of possible arch-specific libc functions. You are welcomed
to put more cases here.

Options:
- ``CONFIG_TESTING_ARCH_LIBC`` – Enable the test.
- ``CONFIG_TESTING_ARCH_LIBC_XXXXX`` – Enable test for function XXXXX.
