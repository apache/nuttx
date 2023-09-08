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

4. Run the virtio network, block, serial and rng driver with qemu

  $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

  $ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 \
    -global virtio-mmio.force-legacy=false \
    -device virtio-serial-device,bus=virtio-mmio-bus.0 \
    -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
    -device virtconsole,chardev=foo \
    -device virtio-rng-device,bus=virtio-mmio-bus.1 \
    -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
    -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
    -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
    -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
    -bios none -kernel ./nuttx/nuttx -nographic

  or

  $ qemu-system-riscv64 -semihosting -M virt,aclint=on -cpu rv64 -smp 8 \
    -global virtio-mmio.force-legacy=false \
    -device virtio-serial-device,bus=virtio-mmio-bus.0 \
    -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
    -device virtconsole,chardev=foo \
    -device virtio-rng-device,bus=virtio-mmio-bus.1 \
    -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
    -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
    -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
    -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
    -bios none -kernel ./nuttx/nuttx -nographic

5. Run the virtio gpu driver with qemu and test fb demo
  $ # For 32-bit build.
  $ ./tools/configure.sh rv-virt:fb
  $ make -j
  $ qemu-system-riscv32 -semihosting -M virt -cpu rv32 -smp 8 \
    -chardev stdio,id=con,mux=on \
    -serial chardev:con \
    -device virtio-gpu-device,xres=640,yres=480,bus=virtio-mmio-bus.0 \
    -mon chardev=con,mode=readline \
    -bios none -kernel nuttx

  NuttShell (NSH) NuttX-10.4.0
  nsh> fb

  $ # For 64-bit build.
  $ ./tools/configure.sh rv-virt:fb64
  $ make -j
  $ qemu-system-riscv64 -semihosting -M virt -cpu rv64 -smp 8 \
    -chardev stdio,id=con,mux=on \
    -serial chardev:con \
    -device virtio-gpu-device,xres=640,yres=480,bus=virtio-mmio-bus.0 \
    -mon chardev=con,mode=readline \
    -bios none -kernel nuttx

  NuttShell (NSH) NuttX-10.4.0
  nsh> fb

6. TODO

  Support FPU
  Support RISC-V User mode
