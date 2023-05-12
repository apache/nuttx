1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Build and install qemu

  $ git clone https://github.com/qemu/qemu
  $ cd qemu
  $ ./configure --target-list=riscv32-softmmu,riscv64-softmmu
  $ make
  $ sudo make install

3.1. Configure and build NuttX for BUILD_FLAT

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh rv-virt:nsh
  $ make V=1 -j7

3.2 Configure and build NuttX for BUILD_KERNEL, 64-bit or 32-bit

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ # For 64-bit build.
  $ ./tools/configure.sh rv-virt:knsh64 
  $ # For 32-bit build.
  $ ./tools/configure.sh rv-virt:knsh32
  $ make V=1 -j7
  $ make export V=1
  $ cd ../apps
  $ ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
  $ make import V=1
  $ cd ../nuttx

4. Run the nuttx with qemu

  $ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 -bios none -kernel nuttx -nographic

  or

  $ qemu-system-riscv64 -semihosting -M virt,aclint=on -cpu rv64 -smp 8 -bios none -kernel nuttx -nographic

  NuttShell (NSH) NuttX-10.3.0-RC1
  nsh> mount -t hostfs -o fs=. /host
  nsh> cat /host/AUTHORS
  This is a list of all the contributors that have submitted ICLA, SGA
  If you are not on this list and believe you should be, please inform us.

  ICLA
  ====
  ...
  nsh>

4. Run the virtio network and block driver with qemu

  $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

  $ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 \
  -global virtio-mmio.force-legacy=false \
  -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd -device virtio-blk-device,drive=hd \
  -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
  -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
  -bios none -kernel nuttx -nographic

  or

  $ qemu-system-riscv64 -semihosting -M virt,aclint=on -cpu rv64 -smp 8 \
  -global virtio-mmio.force-legacy=false \
  -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd -device virtio-blk-device,drive=hd \
  -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
  -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
  -bios none -kernel nuttx -nographic

5. TODO

  Support FPU
  Support RISC-V User mode
