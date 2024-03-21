#!/usr/bin/env python3

import os
import re
import subprocess
import time
from enum import Enum

import pexpect
import pexpect.fdpexpect
import pexpect.spawnbase
import serial

rootPath = os.path.dirname(os.path.abspath(__file__))

tmp_read_nonblocking = pexpect.spawnbase.SpawnBase.read_nonblocking


def enhanced_read_nonblocking(self, size=1, timeout=None):
    return re.sub(
        r"(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]",
        "",
        tmp_read_nonblocking(self, size, timeout).decode(errors="ignore"),
    ).encode()


pexpect.spawnbase.SpawnBase.read_nonblocking = enhanced_read_nonblocking


class StatusCodeEnum(Enum):
    NORMAL = (0, "Normal")
    TIMEOUT_ERR = (-1, "Timeout")
    EOF_ERR = (-2, "EOF")
    CRASH_ERR = (-3, "Crash happened")
    BUSYLOOP_ERR = (-4, "Busy loop happened")
    UNKNOWN_ERR = (-5, "Unknown")

    @staticmethod
    def get_enum_msg_by_code(status_code):
        for status in StatusCodeEnum:
            if status.value[0] == status_code:
                return status.value[1]


class connectNuttx(object):
    def __init__(
        self,
        board,
        vela_path,
        dev,
        log_path,
        fs,
        ota_version,
        core,
        sudo,
        ci,
        method,
        target,
    ):
        self.board = board
        self.path = vela_path
        self.dev = dev
        self.log_path = log_path
        self.fs = fs
        self.version = ota_version
        self.sudo = sudo
        self.ci = ci
        self.core = core
        self.method = method
        self.target = target
        self.enter = "\r"
        self.debug_flag = 0
        self.format_str_len = 105
        # get PROMPT value and rate value
        self.PROMPT = getConfigValue(
            self.path, self.board, core=self.core, flag="NSH_PROMPT_STRING"
        )
        self.rate = getConfigValue(
            self.path, self.board, core=self.core, flag="UART0_BAUD"
        )

        if not os.path.exists(self.log_path):
            os.makedirs(self.log_path)

    def setup(self):
        self.start_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        print("============ start at %s ============" % self.start_time)
        if self.target == "sim":
            # start nuttx for sim
            start.startNuttx(self, self.path, self.log_path, self.board, self.sudo)
        if self.target == "qemu":
            start.startQemu(self, self.path, self.log_path, self.board, self.sudo)
        if self.target in ["target", "module"]:
            # start minicom
            if self.method == "minicom":
                start.startMinicom(
                    self, self.dev, self.board, self.log_path, self.core, self.rate
                )
            else:
                start.startSerial(
                    self, self.dev, self.board, self.log_path, self.core, self.rate
                )

    def sendcontrol(self, char):
        char = char.lower()
        a = ord(char)
        if 97 <= a <= 122:
            a = a - ord("a") + 1
            byte = bytes([a])
            return byte
        d = {
            "@": 0,
            "`": 0,
            "[": 27,
            "{": 27,
            "\\": 28,
            "|": 28,
            "]": 29,
            "}": 29,
            "^": 30,
            "~": 30,
            "_": 31,
            "?": 127,
        }
        if char not in d:
            return b""
        byte = bytes(d[char])
        return byte

    def sendControlCmd(self, cmd, expect="ap>", timeout=10):
        if self.method == "minicom":
            self.process.sendcontrol(cmd)
            time.sleep(1)
            self.process.sendcontrol(cmd)
        else:
            byte = self.sendcontrol(cmd)
            time.sleep(1)
            self.process.send(byte)
            time.sleep(1)
            self.process.send(byte)

    def print_format_str(self, string, type="text"):
        str_prefix = "+"
        str_suffix = "+"

        if type == "head":
            rest_char_len = self.format_str_len - 2 - len(string)
            half_len = int(rest_char_len / 2)
            print(
                str_prefix
                + "-" * half_len
                + string
                + "-" * (rest_char_len - half_len)
                + str_suffix
            )
        elif type == "tail":
            rest_char_len = self.format_str_len - 2
            print(str_prefix + "-" * rest_char_len + str_suffix)
        elif type == "text":
            str_prefix = "| "
            str_suffix = " |"
            rest_char_len = (
                self.format_str_len - len(str_prefix) - len(str_suffix) - len(string)
            )
            print(
                str_prefix
                + string
                + " " * (1 if rest_char_len < 1 else rest_char_len)
                + str_suffix
            )
        else:
            print(string)

    def clean_buffer(self):
        i = -1
        while True:
            if (
                (
                    self.process.before is not None
                    and self.process.before.decode(errors="ignore")
                    .replace("\r", "")
                    .replace("\n", "")
                    != ""
                )
                or (
                    self.process.after is not None
                    and self.process.after != pexpect.TIMEOUT
                    and self.process.after.decode(errors="ignore")
                    .replace("\r", "")
                    .replace("\n", "")
                    != ""
                )
                or i == 0
            ):
                i = self.process.expect(
                    [re.compile(b".+"), pexpect.TIMEOUT, pexpect.EOF], timeout=0.1
                )
            else:
                while True:
                    try:
                        self.process.read_nonblocking(
                            size=self.process.maxread, timeout=0.1
                        )
                    except Exception:
                        break
                self.process.before = b""
                self.process.after = b""
                break

    # send command to nsh
    def sendCommand(self, cmd, *argc, **argv):
        expect = []
        timeout = 10
        ret = StatusCodeEnum.NORMAL.value[0]
        length = len(argc)
        if length == 0:
            expect.append(self.PROMPT)
        else:
            for i in argc:
                expect.append(i)
        length = len(argv)
        if length != 0:
            for key, value in argv.items():
                if key == "timeout":
                    timeout = value
        if self.method != "minicom":
            time.sleep(0.5)
        if self.target == "qemu":
            self.clean_buffer()
            self.process.sendline(cmd)
        else:
            self.clean_buffer()
            self.process.sendline(cmd)
            time.sleep(0.1)
            self.process.send("\r\n\r\n")
        try:
            for i in expect:
                ret = self.process.expect(i, timeout=timeout)
        except Exception as e:
            self.print_format_str(" Catch Exception ", type="head")

            if isinstance(e, pexpect.TIMEOUT):
                ret = StatusCodeEnum.TIMEOUT_ERR.value[0]

            elif isinstance(e, pexpect.EOF):
                ret = StatusCodeEnum.EOF_ERR.value[0]
                self.print_format_str(f"An pexpect.EOF error occurred: {str(e)}")

            else:
                ret = StatusCodeEnum.UNKNOWN_ERR.value[0]
                self.print_format_str(f"An unexpected error occurred: {str(e)}")

            self.print_format_str(" Result ", type="head")
            self.print_format_str(f"Command     : '{cmd}'")
            self.print_format_str(f"Expect value: {str(expect)}")
            self.print_format_str(f"Timeout     : {timeout}s")
            self.print_format_str(
                f"Test result : {StatusCodeEnum.get_enum_msg_by_code(ret)}"
            )
            self.print_format_str("", type="tail")

        finally:
            self.debug(cmd, ret)

            if self.method != "minicom":
                time.sleep(0.5)
            return ret

    def switch_to_original_core(self):
        if self.target == "target":
            self.sendControlCmd("c")
            if self.core != "ap":
                self.process.sendline("cu -l /dev/tty%s\n" % self.core.upper())
                self.process.expect_exact(self.PROMPT)

    def debug(self, cmd, ret):
        if self.debug_flag:
            print("********************* DEBUG START ********************")
            if cmd == "\n":
                cmd = r"\n"
            print("cmd: {}".format(cmd))
            print("ret: {}".format(ret))
            print("before: {}".format(self.process.before.decode(errors="ignore")))
            print("after: {}".format(self.process.after.decode(errors="ignore")))
            print("buffer: {}".format(self.process.buffer.decode(errors="ignore")))
            print("********************** DEBUG END **********************")

    def cleanup(self):
        if self.target == "sim":
            self.process.sendline("poweroff")
        if self.target == "qemu":
            self.sendControlCmd("a", self.PROMPT)
            self.process.sendline("x")


class start:
    def startMinicom(self, dev, board, log_path, core, rate):
        self.log = "{}/{}_{}.cap".format(log_path, dev[-4:], self.start_time)
        self.process = pexpect.spawn(
            r"sudo minicom -D {} -b {} -o -C {}".format(dev, rate, self.log),
            maxread=200000,
        )
        self.process.expect("Welcome to minicom")
        self.switch_to_original_core()

    def startSerial(self, dev, board, log_path, core, rate):
        self.log = "{}/{}_{}.cap".format(log_path, dev[-4:], self.start_time)
        self.logFile = open(self.log, "ab+")
        self.ser = serial.Serial(port=dev, baudrate=int(rate))
        self.process = pexpect.fdpexpect.fdspawn(
            self.ser, "wb", maxread=20000, logfile=self.logFile
        )
        self.switch_to_original_core()

    def startNuttx(self, path, log_path, board="sim", sudo=False):
        os.chdir(path)
        self.log = "{}/{}_{}.log".format(log_path, board, self.start_time)
        if sudo:
            if board in ["sim_rpserver", "sim_rpproxy"]:
                os.chdir(path)
                self.process = pexpect.spawn(
                    "bash", ["-c", "sudo ./rpserver/nuttx/nuttx | tee %s" % self.log]
                )
            else:
                self.process = pexpect.spawn(
                    "bash", ["-c", "sudo ./nuttx | tee %s" % self.log]
                )
        else:
            self.process = pexpect.spawn("bash", ["-c", "./nuttx | tee %s" % self.log])
        self.process.expect(self.PROMPT)

    def startQemu(self, path, log_path, board="qemu", sudo=False):
        os.chdir(path)
        self.log = "{}/{}_{}.log".format(log_path, board, self.start_time)
        flag1 = getConfigValue(path, board, core=None, flag="ARCH_CHIP")
        if flag1 == "imx6":
            self.process = pexpect.spawn(
                "bash",
                [
                    "-c",
                    "qemu-system-arm -semihosting -M sabrelite -m 1024 -smp 4 -kernel ./nuttx -nographic | tee %s"
                    % self.log,
                ],
            )
        if flag1 == "qemu-rv":
            flag2 = getConfigValue(path, board, core=None, flag="ARCH_RV64")
            options = ""
            if flag2:
                riscv = "qemu-system-riscv64"
            else:
                riscv = "qemu-system-riscv32"
            fs_flag = getConfigValue(path, board, core=None, flag="DRIVERS_VIRTIO_BLK")
            if fs_flag:
                os.system("dd if=/dev/zero of=fatfs.img bs=512 count=128K")
                os.system("mkfs.fat fatfs.img")
                os.system("chmod 777 ./fatfs.img")
                options = (
                    "-drive index=0,id=userdata,if=none,format=raw,file=./fatfs.img "
                    "-device virtio-blk-device,bus=virtio-mmio-bus.0,drive=userdata"
                )
            self.process = pexpect.spawn(
                "bash",
                [
                    "-c",
                    "%s -M virt -bios ./nuttx -nographic %s | tee %s"
                    % (riscv, options, self.log),
                ],
            )
        self.process.expect(self.PROMPT)


def runCmd(cmd):
    p = subprocess.Popen(
        cmd,
        shell=True,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    stdout, stderr = p.communicate()
    recode = p.returncode

    if recode != 0:
        print("Debug: run command '%s' failed" % cmd)

    return stdout


# find file
def findFile(path, flag, board, core=None, match=True):
    fList = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if not match and flag in name:
                fList.append(os.path.join(root, name))
            if match and name == flag and ".o" not in name:
                fList.append(os.path.join(root, name))
    if len(fList) != 1:
        fList = [x for x in fList if board in x and core + "/" in x]
    print(fList)
    return fList


# get CONFIG_NSH_PROMPT_STRING value
def getConfigValue(path, board, core, flag):
    value = ""
    l1 = findFile(path, ".config", board, core=core)
    with open(l1[0], "r+") as f:
        lines = f.readlines()
    f.close()
    print(lines)
    print(flag)
    for line in lines:
        if flag + "=" in line:
            value = line.split("=")[1]
        if '"' in value:
            value = value.strip('"').strip()
    print(value)
    return value


# read log and extract timestamp
def getTimestamp(filename, str1, str2, pattern):
    with open(filename, "r") as f:
        buff = f.read()
        pat = re.compile(str1 + "(.*?)" + str2, re.S)
        result = pat.findall(buff)
        timestamp = re.findall(pattern, str(result))  # extract timestamp
        return timestamp


# calculate the time difference for each time
def getDif(timestamp, rateANDinterance):
    lst = []
    for i in range(1, len(timestamp)):
        difts = int(timestamp[i]) - int(timestamp[i - 1])
        float_num = (difts / 1000000) - rateANDinterance
        lst.append(float_num)
    return lst


# read log
def getLog(filename, str1, str2):
    l1 = []
    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        buff = f.read()
        f.close()
    pat = re.compile(str1 + "(.*?)" + str2, re.S)
    result = pat.findall(buff)

    if "gtest" in str1:
        return result[0].strip("\n")
    else:
        # add line to l1
        for i in result[0].strip("\n").split("\n"):
            l1.append(i)
        return l1


def rmfile(p, core, file):
    if p.core == core:
        p.sendCommand("rm -r" + file)
    else:
        p.process.sendline("cu -l /dev/tty" + core.upper())
        p.sendCommand("\n", core, flag=core + ">")
        p.sendCommand("rm -r " + file, core, flag=core + ">")
        if p.core == "ap":
            p.sendControlCmd("c")
        else:
            p.switch_to_original_core(p)
