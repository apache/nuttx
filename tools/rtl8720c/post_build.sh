#!/bin/sh

if [ -z $1 ]
then
  echo "Please specify the outdir"
  exit
fi

WORKDIR=$(cd $(dirname $0); pwd)
OUTDIR=`readlink -f $1`

cd $WORKDIR

mkdir -p application_is/Debug/bin/
cp $OUTDIR/nuttx application_is/Debug/bin/application_is.axf
./elf2bin.linux keygen keycfg.json
./elf2bin.linux convert amebaz2_bootloader.json BOOTLOADER secure_bit=0
./elf2bin.linux convert amebaz2_bootloader.json PARTITIONTABLE secure_bit=0
./elf2bin.linux convert amebaz2_firmware_is.json FIRMWARE secure_bit=0
./add_crc application_is/Debug/bin/firmware_is.bin $OUTDIR/firmware_is.bin
./elf2bin.linux combine $OUTDIR/flash_is.bin PTAB=partition.bin,BOOT=bootloader/Debug/bin/bootloader.bin,FW1=application_is/Debug/bin/firmware_is.bin


./AmebaZII_PGTool_Linux_v1.0.8 -set generate_bin 1 ./partition.bin 0x0000
./AmebaZII_PGTool_Linux_v1.0.8 -set generate_bin 2 ./bootloader/Debug/bin/bootloader.bin 0x4000
./AmebaZII_PGTool_Linux_v1.0.8 -set generate_bin 3 ./firmware_is.bin 0xc000
./AmebaZII_PGTool_Linux_v1.0.8 -set generate_bin 4 $OUTDIR/firmware_is.bin 0xe4000
./AmebaZII_PGTool_Linux_v1.0.8 -set generate_bin 5 ./empty.bin 0x200000
./AmebaZII_PGTool_Linux_v1.0.8 -generate

BUILD_TIME=$(date "+%Y%m%d")
FACTORY_BIN_NAME=MHCWB4P_flash_${BUILD_TIME}.bin

mv ./flash.bin ${FACTORY_BIN_NAME}

