===============
Rust in NuttX
===============

.. warning::
    This guide is under development. Rust support in NuttX is experimental.

Introduction
============
NuttX is exploring Rust integration to provide memory safety guarantees and modern
language features while maintaining its small footprint and real-time capabilities.

This guide covers:

- Setting up Rust toolchain for NuttX development
- Building Rust components with NuttX
- Interoperability between Rust and C
- Testing Rust components

Prerequisites
=============
- Rust toolchain installed (rustup recommended)
- NuttX build environment configured
- Basic knowledge of Rust and NuttX development

Supported Platforms
===================
- AArch64 (WIP)
- ARMv7-A (WIP)
- ARMv6-M
- ARMv7-M
- ARMv8-M
- RISCV32
- RISCV64

Getting Started
===============
1. Install Rust toolchain and switch to nightly

Please refer to the official Rust installation guide for more details: https://www.rust-lang.org/tools/install

.. code-block:: bash

    rustup toolchain install nightly
    rustup default nightly

2. Prepare NuttX build environment

Please ensure that you have a working NuttX build environment, and with the following PR merged or cherry-picked:
- https://github.com/apache/nuttx-apps/pull/2487
- https://github.com/apache/nuttx/pull/15469

3. Enable essential kernel configurations

Please enable the following configurations in your NuttX configuration:

- CONFIG_SYSTEM_TIME64
- CONFIG_FS_LARGEFILE
- CONFIG_TLS_NELEM = 16
- CONFIG_DEV_URANDOM

The `rv-virt:nsh` board using make as the build system is recommended for testing Rust applications as it has been verified to work with this configuration.

For `rv-virt:nsh` board, you should disable `CONFIG_ARCH_FPU` configuration since RISCV32 with FPU is not supported yet.

4. Enable sample application

Please enable the sample application in your NuttX configuration:
- CONFIG_EXAMPLES_HELLO_RUST_CARGO

5. Build and run the sample application

Build the NuttX image and run it on your target platform:

.. code-block:: bash

    qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 -bios nuttx/nuttx -nographic

    NuttShell (NSH) NuttX-12.8.0
    nsh> hello_rust_cargo
    {"name":"John","age":30}
    {"name":"Jane","age":25}
    Deserialized: Alice is 28 years old
    Pretty JSON:
    {
    "name": "Alice",
    "age": 28
    }
    Hello world from tokio!

Congratulations! You have successfully built and run a Rust application on NuttX.
