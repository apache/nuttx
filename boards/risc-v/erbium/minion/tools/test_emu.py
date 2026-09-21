#!/usr/bin/env python3
# boards/risc-v/erbium/minion/tools/test_emu.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements. See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to you under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise prebuilt Erbium NuttX images and record reproducible results."""

import argparse
import hashlib
import json
import re
import subprocess
import time
from pathlib import Path

FAILURE = re.compile(
    r"\bERROR\b|Assertion failed|ASSERTION|PANIC|riscv_exception:|Unhandled exception"
    r"|nsh: .*?(?:failed|not found|invalid)"
    r"|ostest_main: Exiting with status -?[1-9]",
    re.I,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--emu", type=Path, required=True)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--mode", choices=["nsh", "ostest"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=int, default=14400)
    parser.add_argument("--max-cycles", default="100000000000")
    parser.add_argument("--extra-harts", action="store_true")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    for path in (args.emu, args.elf):
        if not path.is_file():
            parser.error(f"Input file does not exist: {path}")
    args.output.mkdir(parents=True, exist_ok=True)
    uart = args.output / "uart.log"
    emulator = args.output / "emulator.log"
    uart.write_bytes(b"")
    command = [
        str(args.emu.resolve()),
        "-elf_load",
        str(args.elf.resolve()),
        "-reset_pc",
        "0x40000200",
        "-max_cycles",
        args.max_cycles,
        "-uart_tx_file",
        str(uart.resolve()),
        "-mem_reset",
        "165",
    ]
    if args.extra_harts:
        command += ["-minions", "3"]
    else:
        command += ["-single_thread"]
    start = time.monotonic()
    deadline = start + args.timeout
    proc = None
    checks = []
    failure = None
    try:
        with emulator.open("wb") as log:
            proc = subprocess.Popen(
                command, stdin=subprocess.PIPE, stdout=log, stderr=log
            )

            def wait_for(needle, offset=0):
                while True:
                    output = uart.read_text(errors="replace") if uart.exists() else ""
                    if FAILURE.search(output):
                        raise RuntimeError("Guest reported a failure; inspect uart.log")
                    if needle in output[offset:]:
                        return output
                    if proc.poll() is not None:
                        raise RuntimeError(
                            f"Emulator exited with {proc.returncode} before {needle!r}"
                        )
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"Timed out waiting for {needle!r}")
                    time.sleep(0.1)

            if args.mode == "ostest":
                text = wait_for("ostest_main: Exiting with status 0")
                if "Final memory usage:" not in text:
                    raise RuntimeError("Missing final memory report")
                for marker in [
                    "user_main: Exiting",
                    "FPU#1: Succeeded",
                    "FPU#2: Succeeded",
                    "user_main: semaphore test",
                    "user_main: signal handler test",
                    "user_main: pthread_exit() test",
                    "wqueue_test: teardown done",
                ]:
                    if marker.lower() not in text.lower():
                        raise RuntimeError(
                            f"Missing expected ostest coverage: {marker}"
                        )
                checks.append("Complete ostest, including FPU context switching")
            else:
                text = wait_for("nsh> ")
                checks.append("NSH boot")

                def run_command(command_text, expected=None):
                    nonlocal text
                    offset = len(text)
                    # The emulator drains host input into a finite RX FIFO.
                    # Wait for each echoed character to avoid overrunning it.
                    for i, char in enumerate(command_text):
                        proc.stdin.write(char.encode())
                        proc.stdin.flush()
                        wait_for(command_text[: i + 1], offset)
                    proc.stdin.write(b"\n")
                    proc.stdin.flush()
                    text = wait_for("nsh> ", offset)
                    response = text[offset:].split("\n", 1)[-1]
                    if expected and expected not in response:
                        raise RuntimeError(
                            f"Missing {expected!r} in response to {command_text!r}"
                        )
                    checks.append(command_text)
                    return response

                def uptime():
                    response = run_command("cat /proc/uptime")
                    match = re.search(r"^\s*(\d+\.\d+)\s*$", response, re.M)
                    if not match:
                        raise RuntimeError("Missing numeric procfs uptime")
                    return float(match[1])

                for command_text, expected in [
                    ("uname -a", "risc-v"),
                    ("help", "uname"),
                    ("ps", "Idle_Task"),
                ]:
                    run_command(command_text, expected)

                before = uptime()
                run_command("sleep 1")
                if uptime() - before < 0.99:
                    raise RuntimeError("Guest uptime did not advance by one second")
                checks.append("Timer advances during sleep")
                run_command("echo ERBIUM_UART_TIMER_OK", "ERBIUM_UART_TIMER_OK")
                run_command("free", "total")
                time.sleep(0.5)
                run_command("echo ERBIUM_LATE_RX_OK", "ERBIUM_LATE_RX_OK")
                checks.append("UART RX after idle")
    except (Exception, KeyboardInterrupt) as error:
        failure = str(error) or type(error).__name__
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
        result = {
            "passed": failure is None,
            "error": failure,
            "checks": checks,
            "command": command,
            "duration_seconds": time.monotonic() - start,
            "elf_sha256": hashlib.sha256(args.elf.read_bytes()).hexdigest(),
            "emulator_sha256": hashlib.sha256(args.emu.read_bytes()).hexdigest(),
            "emulator_returncode": None if proc is None else proc.returncode,
        }
        if failure is None and re.search(
            r": (ERROR|FATAL) ", emulator.read_text(errors="replace")
        ):
            failure = "Emulator reported an error; inspect emulator.log"
            result["passed"] = False
            result["error"] = failure
        (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
