#!/usr/bin/env python3

import os
import re
import subprocess
import time

import pexpect
import pexpect.fdpexpect
import serial

rootPath = os.path.dirname(os.path.abspath(__file__))


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
        time.sleep(1)
        self.process.sendline("\n")
        ret = self.process.expect_exact(expect)
        return ret

    # send command to nsh
    def sendCommand(self, cmd, expect="", timeout=10, flag=""):
        if self.method != "minicom":
            time.sleep(0.5)
        if not expect:
            expect = self.PROMPT
        self.process.buffer = b""
        self.process.sendline(cmd)
        try:
            ret = self.process.expect(expect, timeout=timeout)
        except pexpect.TIMEOUT:
            print("Debug: TIMEOUT '%s' exist and run next test case" % cmd)
            ret = -1
        except pexpect.EOF:
            print("Debug: EOF raise exception")
            ret = -2
        finally:
            if self.debug_flag:
                self.debug(cmd, ret)
            self.process.buffer = b""
            self.process.sendline("\n")
            if flag:
                is_newline = self.process.expect_exact(flag, timeout=timeout)
            else:
                is_newline = self.process.expect_exact(self.PROMPT, timeout=timeout)
            if self.debug_flag:
                self.debug("NEWLINE", is_newline)
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
        print("********************* DEBUG START ********************")
        if cmd == "\n":
            cmd = r"\n"
        print("cmd: %s\n" % cmd)
        print("ret: %s\n" % str(ret))
        print("before: %s\n" % repr(self.process.before))
        print("after: %s\n" % repr(self.process.after))
        print("buffer: %s\n" % repr(self.process.buffer))
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
            if flag2:
                riscv = "qemu-system-riscv64"
            else:
                riscv = "qemu-system-riscv32"
            self.process = pexpect.spawn(
                "bash",
                [
                    "-c",
                    "%s -M virt -bios ./nuttx -nographic | tee %s" % (riscv, self.log),
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
