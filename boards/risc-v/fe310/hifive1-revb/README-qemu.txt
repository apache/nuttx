1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Build and install qemu

  $ git clone https://github.com/qemu/qemu
  $ cd qemu
  $ ./configure --target-list=riscv32-softmmu
  $ make
  $ sudo make install

3. Modify flash origin address

index 559c1813b8..a67c37b576 100644
--- a/boards/risc-v/fe310/hifive1-revb/scripts/ld.script
+++ b/boards/risc-v/fe310/hifive1-revb/scripts/ld.script
@@ -35,7 +35,7 @@

 MEMORY
 {
-  flash (rx) : ORIGIN = 0x20010000, LENGTH = 4096K
+  flash (rx) : ORIGIN = 0x20400000, LENGTH = 4096K
   sram (rwx) : ORIGIN = 0x80000000, LENGTH = 16K
 }

4. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://bitbucket.org/nuttx/nuttx.git
  $ git clone https://bitbucket.org/nuttx/apps.git
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh hifive1-revb:nsh
  $ make V=1

5. Run the nuttx with qemu

  $ qemu-system-riscv32 -nographic -machine sifive_e -kernel ./nuttx

6. TODO

  Support RISC-V User mode
