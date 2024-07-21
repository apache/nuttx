!#/bin/sh

curl -O -L "https://raw.githubusercontent.com/raspberrypi/firmware/refs/heads/stable/boot/bcm2711-rpi-4-b.dtb"
curl -O -L "https://raw.githubusercontent.com/raspberrypi/firmware/refs/heads/stable/boot/fixup4.dat"
curl -O -L "https://raw.githubusercontent.com/raspberrypi/firmware/refs/heads/stable/boot/fixup4cd.dat"
curl -O -L "https://raw.githubusercontent.com/raspberrypi/firmware/refs/heads/stable/boot/fixup4db.dat"
curl -O -L "https://raw.githubusercontent.com/raspberrypi/firmware/refs/heads/stable/boot/fixup4x.dat"
curl -O -L "https://raw.githubusercontent.com/raspberrypi/firmware/refs/heads/stable/boot/start4.elf"
curl -O -L "https://raw.githubusercontent.com/raspberrypi/firmware/refs/heads/stable/boot/start4cd.elf"
curl -O -L "https://raw.githubusercontent.com/raspberrypi/firmware/refs/heads/stable/boot/start4db.elf"
curl -O -L "https://raw.githubusercontent.com/raspberrypi/firmware/refs/heads/stable/boot/start4x.elf"
