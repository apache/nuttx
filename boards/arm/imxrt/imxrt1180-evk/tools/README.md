# MIMXRT1180-EVK tools

Helpers for programming and debugging NuttX on the MIMXRT1180-EVK.

## Prerequisites

- A **SEGGER J-Link** probe connected to the EVK JTAG header **J37** (SWD),
  or a **MCU-Link** probe (U55 on the board) reflashed with SEGGER's
  "MCU-Link J-Link" firmware and used via **J53**.
- **J-Link Software and Documentation Pack v7.98a or later**, which adds
  built-in `MIMXRT1189xxx8` device support (including a default FlexSPI1
  Port A NOR flash loader).  Download from
  <https://www.segger.com/downloads/jlink/>.
- The EVK's DIP switch **SW5 = 0100** (BOOT_MODE[2:0] = 100 =
  FlexSPI Quad SPI NOR boot) so the ROM boots from the flashed image
  after reset.

## Flash layout

FlexSPI1 NOR is split between the two cores:

| Flash address | Offset      | Size   | Contents                          |
|---------------|-------------|--------|-----------------------------------|
| `0x28000000`  | `0x0`       | 512 KB | Cortex-M33 image (`flash.bin`)    |
| `0x28080000`  | `0x80000`   | rest   | Cortex-M7 image (`nuttx.bin`)     |

The M33 image is the bootable one: it carries the FCB (`@0x400`) and the
AHAB container (`@0x1000`) that the ROM consumes.  The M7 image is a raw
XIP payload with no headers of its own.

The configurations are:

  * `imxrt1180-evk:bl` — minimal M33 bootloader; its `bootloader_main()`
    only releases the M7 and returns.
  * `imxrt1180-evk:nsh-m33` — full NSH running on the M33.
  * `imxrt1180-evk:nsh` — NSH running on the M7, started by the `bl`
    image.

## Programming

`flash-jlink.sh` takes the image and the flash offset, so each image is
programmed with its own invocation:

    # Cortex-M33 bootloader (built from imxrt1180-evk:bl)
    boards/arm/imxrt/imxrt1180-evk/tools/flash-jlink.sh flash.bin 0x0

    # Cortex-M7 NuttX payload (built from imxrt1180-evk:nsh)
    boards/arm/imxrt/imxrt1180-evk/tools/flash-jlink.sh nuttx.bin 0x80000

The offset may also be given as an absolute flash address
(`0x28080000`).  Running the script without arguments prints its usage.

The script drives `JLinkExe` connected to the **Cortex-M33** profile
(`MIMXRT1189xxx8_M33`).  RT1180 always boots the M33 first, and the ROM
brings FlexSPI1 up as part of that boot — the SEGGER flash loader needs
that init to work.  Each invocation resets and halts the CM33,
`loadbin`s the image at the requested address, then resets and starts
execution.  J-Link erases only the sectors the image covers, so flashing
one image leaves the other intact.

At boot the ROM starts the M33 image; the `bl` bootloader programs
`INITVTOR`, releases the M7, and the M7 boots NuttX from `0x28080000`.

NSH prints to LPUART1 (VCOM on the MCU-Link USB, 115200 8N1).

> Note — connecting with `MIMXRT1189xxx8_M7` for programming does *not*
> work: the M7 device profile releases the M7 without running the
> ROM/M33 FlexSPI init and the flash loader fails with
> `RAMCode-sided Prepare()`.  Always flash with the `_M33` profile.

## Debugging with GDB

Start the J-Link GDB server in one terminal (this uses the `_M7`
profile so you can step through the running NuttX code):

    boards/arm/imxrt/imxrt1180-evk/tools/gdbserver-jlink.sh

In another terminal, attach:

    arm-none-eabi-gdb nuttx
    (gdb) target extended-remote localhost:2331
    (gdb) monitor reset
    (gdb) continue

## Environment overrides

Both scripts honour these variables:

| Variable            | Default                   | Purpose                       |
|---------------------|---------------------------|-------------------------------|
| `JLINK_EXE`         | `JLinkExe`                | Commander binary              |
| `JLINK_GDBSERVER`   | `JLinkGDBServer`          | GDB server binary             |
| `JLINK_SPEED`       | `4000`                    | SWD speed in kHz              |
| `JLINK_IF`          | `SWD`                     | `SWD` or `JTAG`               |
| `JLINK_PORT`        | `2331`                    | GDB listen port               |

The J-Link device name is set per script:
  * `flash-jlink.sh` uses `MIMXRT1189xxx8_M33` — the M33 is the RT1180
    boot core and its ROM init brings FlexSPI1 up so the SEGGER flash
    loader can program it.
  * `gdbserver-jlink.sh` uses `MIMXRT1189xxx8_M7` so you attach to the
    core actually running NuttX.

## Optional: NXP's SDK connect scripts

For advanced scenarios (e.g. attaching to the CM7 while the CM33 is
holding it in reset, or debugging both cores simultaneously), NXP ships
two connect scripts in the MCUXpresso SDK:

- `mcuxsdk/examples/_boards/evkmimxrt1180/jlinkscript/evkmimxrt1180_cm7.jlinkscript`
- `mcuxsdk/examples/_boards/evkmimxrt1180/jlinkscript/evkmimxrt1180_cm33.jlinkscript`

To use one of these, pass it to `JLinkExe` via `-jlinkscriptfile`, e.g.:

    JLINK_EXE="JLinkExe -jlinkscriptfile /path/to/evkmimxrt1180_cm7.jlinkscript" \
        ./flash-jlink.sh

These scripts are BSD-3-Clause licensed by NXP and are **not** bundled
here; obtain them from the MCUXpresso SDK for your board.
