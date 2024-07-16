=========================
Thread Local Storage
=========================

Thread local storage (TLS) is a mechanism that allows each thread to have its own
copy of a variable.  This is useful for variables that are used by multiple
functions in a thread, but should not be shared with other threads.

There are some approaches to using TLS in NuttX:

1. Use the ``pthread_key_create()`` and ``pthread_setspecific()`` that from the POSIX standard. This is the most portable approach, but it requires that
the platform support pthreads.
2. Use the ``thread_local`` or ``__thread`` keyword from the C standard: https://gcc.gnu.org/onlinedocs/gcc/extensions-to-the-c-language-family/thread-local-storage.html

Configuration
=============

.. code-block:: console

    CONFIG_SCHED_THREAD_LOCAL  /* Enable native thread local storage support */


Enable it to support native thread local storage, which is required that the compiler configured with ``--enable-tls`` option, this approach is more efficient if the compiler support it.

If your compiler support it then you still need further configuration to use it:

1. Enable ``CONFIG_SCHED_THREAD_LOCAL`` in menuconfig
2. Handle ``tbss`` and ``tdata`` sections in linker script, your can refer to the example in rv-virt

To confirm that your compiler supports TLS, you can try this command:
.. code-block:: console

    arm-none-eabi-gcc --verbose
    COLLECT_GCC=arm-none-eabi-gcc
    COLLECT_LTO_WRAPPER=/home/huang/.local/pkg/arm/bin/../libexec/gcc/arm-none-eabi/13.3.1/lto-wrapper
    Target: arm-none-eabi
    Configured with: /data/jenkins/workspace/GNU-toolchain/arm-13/src/gcc/configure --target=arm-none-eabi --prefix=/data/jenkins/workspace/GNU-toolchain/arm-13/build-arm-none-eabi/install --with-gmp=/data/jenkins/workspace/GNU-toolchain/arm-13/build-arm-none-eabi/host-tools --with-mpfr=/data/jenkins/workspace/GNU-toolchain/arm-13/build-arm-none-eabi/host-tools --with-mpc=/data/jenkins/workspace/GNU-toolchain/arm-13/build-arm-none-eabi/host-tools --with-isl=/data/jenkins/workspace/GNU-toolchain/arm-13/build-arm-none-eabi/host-tools --disable-shared --disable-nls --disable-threads --disable-tls --enable-checking=release --enable-languages=c,c++,fortran --with-newlib --with-gnu-as --with-headers=yes --with-gnu-ld --with-native-system-header-dir=/include --with-sysroot=/data/jenkins/workspace/GNU-toolchain/arm-13/build-arm-none-eabi/install/arm-none-eabi --with-multilib-list=aprofile,rmprofile --with-pkgversion='Arm GNU Toolchain 13.3.Rel1 (Build arm-13.24)' --with-bugurl=https://bugs.linaro.org/
    Thread model: single
    Supported LTO compression algorithms: zlib
    gcc version 13.3.1 20240614 (Arm GNU Toolchain 13.3.Rel1 (Build arm-13.24))

Then you can see ``--disable-tls`` in the output, which means that your compiler does not support TLS.

In this case, you can still use the thread local relative keyword, but it would be implemented by libgcc's emutls.
