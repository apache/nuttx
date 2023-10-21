==========================
Enjoy Digital LiteX FPGA's
==========================

The LiteX framework provides a convenient and efficient infrastructure to create FPGA Cores/SoCs, to explore various digital design architectures and create full FPGA based systems. 

Information specific to Litex and supported boards can be found on the project's homepage: https://github.com/enjoy-digital/litex

Nuttx has basic support for two softcores

 - vexriscv: FPGA friendly RISC-V ISA CPU implementation
 - vexriscv_smp: A more fully featured, Linux compatible core.

Currently, the only configured development board in the Arty A7 https://digilent.com/reference/programmable-logic/arty-a7/start. However, many Litex supported boards 
should work with either core, requiring minimal adjustment to the configuration.


Toolchain
=========

Litex projects can be built with a generic RISC-V GCC toolchain. There are currently two options.

Prebuilt toolchain
------------------

A prebuilt RISC-V toolchain from SiFive can be used to build Litex projects::

 # Download the prebuilt toolchain
 $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz \ 
                   > riscv64-unknown-elf-gcc.tar.gz

 # Unpack the archive
 $ tar -xf riscv64-unknown-elf-gcc.tar.gz 

 # Add to path
 $ export PATH="$HOME/path/to/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14/bin:$PATH

Custom built toolchain
----------------------

The toolchain needs to be compiled locally in order to use a more modern version. At the time of writing, 
the source can be obtained from https://github.com/riscv-collab/riscv-gnu-toolchain and built with the following configuration::

  $ CFLAGS="-g0 -Os"
  $ CXXFLAGS="-g0 -Os"
  $ LDFLAGS="-s"

  $ ./configure \
    CFLAGS_FOR_TARGET='-O2 -mcmodel=medany' \
    CXXFLAGS_FOR_TARGET='-O2 -mcmodel=medany' \
    --prefix=path/to/install/to \
    --with-system-zlib \
    --with-arch=rv32ima \
    --with-abi=ilp32

   $ make

.. important:: The vexriscv_smp core requires `with-arch=rv32imac`. 
   
Check the linked github repository for other options, including building with multilib enabled.

Core specific information
=========================

.. toctree::
   :glob:
   :maxdepth: 1

   cores/*/*

