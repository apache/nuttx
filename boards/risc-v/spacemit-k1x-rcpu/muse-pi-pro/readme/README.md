# Running NuttX on the SpacemiT K1 RCPU

The K1 has a real-time CPU next to the eight application cores. Its firmware is
delivered by the application processor: the kernel loads it at boot through the
standard remoteproc interface, exactly the way it loads the vendor firmware.
This directory holds the small changes the SpacemiT vendor tree needs so that it
loads NuttX instead, plus instructions for getting the firmware onto a board.

The NuttX side is upstream: build `muse-pi-pro:nsh` in the NuttX tree and use
the resulting `nuttx` ELF here.

## Vendor tree

The changes apply to the SpacemiT Buildroot SDK,
<https://gitee.com/spacemit-buildroot/buildroot-sdk>, which is what builds the
kernel, the initramfs and the flashable images for this board.

Two places in it matter:

```
bsp-src/linux-6.6/                     kernel, including the device tree
buildroot-ext/board/spacemit/k1/       initramfs (dracut) and root file system
```

## Applying the changes

Apply both patches at the top of the SDK:

```
cd buildroot-sdk
patch -p1 < nuttx/boards/risc-v/spacemit-k1x-rcpu/muse-pi-pro/readme/0001-*.patch
patch -p1 < nuttx/boards/risc-v/spacemit-k1x-rcpu/muse-pi-pro/readme/0002-*.patch
```

**0001-dts-spacemit-k1-x-set-rcpu-firmware-and-entry.patch** changes the
`rcpu_rproc@0` node in `arch/riscv/boot/dts/spacemit/k1-x.dtsi`:

* adds `firmware-name = "nuttx.elf"`, which is the name the kernel looks for;
  without it the driver falls back to the vendor name `esos.elf`,
* sets `esos-entry-point` to `0x30300000`, the address this firmware is linked
  at,
* removes the second, later `firmware-name` in the same node.  Device tree
  properties are last-one-wins, so leaving it there would silently undo the
  first change.

**0002-dracut-include-the-nuttx-rcpu-firmware.patch** adds one line to
`buildroot-ext/board/spacemit/k1/dracut-modules/99custom/module-setup.sh`,
which already copies the vendor firmware into the initramfs.  The initramfs is
where the kernel reads the firmware from, so the file has to be listed there.

## Putting the firmware on a board

Copy the ELF into the board overlay before building:

```
cp nuttx/nuttx buildroot-ext/board/spacemit/k1/target_overlay/lib/firmware/nuttx.elf
```

Then rebuild the kernel and the image:

```
cd output/k1_v2
make linux-rebuild
make
```

### Method 1: flash the image

`images/Buildroot-k1_v2-sdcard.img` is the full image, `images/bootfs.img` is
the boot file system that carries the kernel, the device tree and the initramfs.

Flashing the boot file system alone is enough when the board already runs a
matching image, and is much faster than a full flash.  It is partition 5 of the
eMMC:

```
adb push images/bootfs.img /tmp/bootfs.img
adb shell "dd if=/tmp/bootfs.img of=/dev/mmcblk2p5 bs=1M"
adb reboot
```

For a board that does not boot yet, flash `Buildroot-k1_v2-sdcard.img` with the
vendor flashing procedure (hold the boot select, feed the image with the
flashserver tool from the SDK, then power cycle).

Either way the shell comes up on the real-time CPU's UART at 115200 8N1:

```
NuttShell (NSH) NuttX-13.1.0
nsh>
```

### Method 2: load it with a debugger

Faster while developing: no image rebuild and no reflash.  The same ELF works
because it is linked for the address it runs at.

The application processor has to have booted at least once, since it is what
programs the DDR window translation the firmware is linked against.

Start OpenOCD on the real-time CPU and load the image with GDB:

```
openocd -f interface/jlink.cfg -c "transport select jtag" \
        -f target/riscv.cfg \
        -c "set _TARGETNAME k1.rcpu" \
        -c "riscv create k1.rcpu 0" \
        -c "k1.rcpu configure -event examine-end { riscv set_ir idle }" \
        -c "init" -c "targets k1.rcpu"
```

```
riscv64-unknown-elf-gdb nuttx
(gdb) set architecture riscv:rv32
(gdb) target remote 127.0.0.1:1024
(gdb) monitor halt
(gdb) load
(gdb) set $pc = 0x30300000
(gdb) monitor resume
(gdb) detach
(gdb) quit
```

`set $pc` is needed because `load` does not set the program counter and the
core would otherwise continue wherever it was.

Do not use OpenOCD's `reset` on this board: it resets the whole SoC without
bringing the application processor back up, and recovering needs a power cycle.

## Going back to the vendor firmware

Remove or rename `nuttx.elf` and drop the patches, or simply set
`firmware-name` back to `esos.elf`, then rebuild and reflash.
