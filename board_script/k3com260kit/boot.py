#!/usr/bin/env python3
"""Stage and boot StarryOS on the SpacemiT K3 COM260 Kit."""

from __future__ import annotations

import argparse
import os
import re
import select
import shutil
import subprocess
import sys
import termios
import threading
import time
import tty
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_KERNEL = Path("target/riscv64gc-unknown-none-elf/release/starryos.bin")
DEFAULT_DTB = Path("os/StarryOS/configs/board/spacemit-k3-com260-ifx.dtb")

KERNEL_LOAD_ADDR = "0x140000000"
KERNEL_LOAD_SIZE = "0x04000000"
DTB_LOAD_ADDR = "0x138000000"
DTB_LOAD_SIZE = "0x00100000"


def status(message: str) -> None:
    print(f"[k3com260kit] {message}", file=sys.stderr, flush=True)


def resolve_project_path(value: Path) -> Path:
    path = value.expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path.resolve()


class SerialConsole:
    def __init__(self, device: Path, baud: int, prompt_regex: bytes) -> None:
        self.device = device
        self.prompt_regex = re.compile(prompt_regex, re.MULTILINE)
        self._fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._output = bytearray()
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._configure(baud)
        self._reader = threading.Thread(
            target=self._read_serial,
            name="k3-serial-reader",
            daemon=True,
        )
        self._reader.start()

    def _configure(self, baud: int) -> None:
        speed = getattr(termios, f"B{baud}", None)
        if speed is None:
            raise ValueError(f"unsupported baud rate: {baud}")

        attrs = termios.tcgetattr(self._fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        if hasattr(termios, "CRTSCTS"):
            attrs[2] &= ~termios.CRTSCTS
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        termios.tcflush(self._fd, termios.TCIOFLUSH)

    def _read_serial(self) -> None:
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.1)
                if not ready:
                    continue
                data = os.read(self._fd, 4096)
                if not data:
                    continue
            except OSError:
                if not self._stop.is_set():
                    status("serial reader stopped unexpectedly")
                return

            try:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
            except BrokenPipeError:
                pass

            with self._condition:
                self._output.extend(data)
                if len(self._output) > 256 * 1024:
                    del self._output[: len(self._output) - 128 * 1024]
                self._condition.notify_all()

    def clear_output(self) -> None:
        with self._condition:
            self._output.clear()

    def write(self, data: bytes) -> None:
        offset = 0
        while offset < len(data):
            try:
                offset += os.write(self._fd, data[offset:])
            except BlockingIOError:
                select.select([], [self._fd], [], 0.1)

    def send_command(self, command: str) -> None:
        status(f"U-Boot: {command}")
        self.clear_output()
        self.write(command.encode("ascii") + b"\r")

    def wait_for_prompt(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not self.prompt_regex.search(bytes(self._output)):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"U-Boot prompt not seen on {self.device} within {timeout:g}s"
                    )
                self._condition.wait(min(remaining, 0.25))

    def interrupt_to_prompt(self, timeout: float) -> None:
        status("U-Boot: Ctrl-C")
        self.clear_output()
        self.write(b"\x03")
        time.sleep(0.1)
        self.write(b"\r")
        self.wait_for_prompt(timeout)

    def interrupt_autoboot(self, timeout: float) -> None:
        status("U-Boot: repeatedly sending 's' to interrupt autoboot")
        deadline = time.monotonic() + timeout
        self.clear_output()

        while True:
            with self._condition:
                if self.prompt_regex.search(bytes(self._output)):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"U-Boot prompt not seen on {self.device} within {timeout:g}s"
                    )

            self.write(b"s")
            with self._condition:
                self._condition.wait(min(remaining, 0.02))

        self.interrupt_to_prompt(max(deadline - time.monotonic(), 1.0))

    def interact(self) -> None:
        if not sys.stdin.isatty():
            status("stdin is not a terminal; serial console handoff skipped")
            return

        status("serial console attached; press Ctrl-] to exit")
        stdin_fd = sys.stdin.fileno()
        saved_attrs = termios.tcgetattr(stdin_fd)
        try:
            tty.setraw(stdin_fd)
            while True:
                ready, _, _ = select.select([stdin_fd], [], [], 0.25)
                if not ready:
                    continue
                data = os.read(stdin_fd, 1024)
                if not data:
                    return
                exit_at = data.find(b"\x1d")
                if exit_at >= 0:
                    if exit_at:
                        self.write(data[:exit_at])
                    return
                self.write(data)
        finally:
            termios.tcsetattr(stdin_fd, termios.TCSANOW, saved_attrs)
            print()

    def close(self) -> None:
        self._stop.set()
        try:
            os.close(self._fd)
        finally:
            self._reader.join(timeout=1)

    def __enter__(self) -> SerialConsole:
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()


def run_host_fastboot(
    fastboot: str,
    artifact: Path,
    timeout: float,
    dry_run: bool,
) -> None:
    command = [fastboot, "stage", str(artifact)]
    status("Host: " + " ".join(command))
    if dry_run:
        return
    subprocess.run(command, cwd=PROJECT_ROOT, check=True, timeout=timeout)


def stage_artifact(
    console: SerialConsole,
    uboot_command: str,
    artifact: Path,
    args: argparse.Namespace,
) -> None:
    console.send_command(uboot_command)
    time.sleep(args.usb_settle)
    try:
        run_host_fastboot(args.fastboot, artifact, args.stage_timeout, False)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        try:
            console.interrupt_to_prompt(args.prompt_timeout)
        except TimeoutError:
            pass
        raise
    console.interrupt_to_prompt(args.prompt_timeout)


def print_dry_run(args: argparse.Namespace, kernel: Path, dtb: Path) -> None:
    commands = [
        ("U-Boot", "Repeated 's' until the autoboot is interrupted"),
        ("U-Boot", f"fastboot -l {KERNEL_LOAD_ADDR} -s {KERNEL_LOAD_SIZE} usb 0"),
        ("Host", f"{args.fastboot} stage {kernel}"),
        ("U-Boot", "Ctrl-C"),
        ("U-Boot", f"fastboot -l {DTB_LOAD_ADDR} -s {DTB_LOAD_SIZE} usb 0"),
        ("Host", f"{args.fastboot} stage {dtb}"),
        ("U-Boot", "Ctrl-C"),
        ("U-Boot", f"booti {KERNEL_LOAD_ADDR} - {DTB_LOAD_ADDR}"),
    ]
    for side, command in commands:
        print(f"{side}: {command}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stage StarryOS and its DTB through K3 U-Boot fastboot, then boot it."
    )
    parser.add_argument(
        "--serial",
        type=Path,
        default=Path(os.environ.get("K3_SERIAL", "/dev/ttyUSB0")),
        help="U-Boot serial device (default: %(default)s or K3_SERIAL)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=int(os.environ.get("K3_BAUD", "115200")),
        help="serial baud rate (default: %(default)s or K3_BAUD)",
    )
    parser.add_argument("--kernel", type=Path, default=DEFAULT_KERNEL)
    parser.add_argument("--dtb", type=Path, default=DEFAULT_DTB)
    parser.add_argument(
        "--fastboot",
        default=os.environ.get("FASTBOOT", "fastboot"),
        help="Host fastboot executable (default: %(default)s or FASTBOOT)",
    )
    parser.add_argument("--prompt-regex", default=r"(?m)^=>\s*")
    parser.add_argument("--prompt-timeout", type=float, default=10.0)
    parser.add_argument("--stage-timeout", type=float, default=120.0)
    parser.add_argument("--usb-settle", type=float, default=1.0)
    parser.add_argument(
        "--no-console",
        action="store_true",
        help="exit after booti instead of attaching stdin to the serial console",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the Host/U-Boot sequence without opening the serial device",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    kernel = resolve_project_path(args.kernel)
    dtb = resolve_project_path(args.dtb)

    for artifact in (kernel, dtb):
        if not artifact.is_file():
            status(f"missing artifact: {artifact}")
            return 2

    if shutil.which(args.fastboot) is None:
        status(f"Host fastboot executable not found: {args.fastboot}")
        return 2

    if args.dry_run:
        print_dry_run(args, kernel, dtb)
        return 0

    kernel_fastboot = f"fastboot -l {KERNEL_LOAD_ADDR} -s {KERNEL_LOAD_SIZE} usb 0"
    dtb_fastboot = f"fastboot -l {DTB_LOAD_ADDR} -s {DTB_LOAD_SIZE} usb 0"

    try:
        with SerialConsole(
            args.serial,
            args.baud,
            args.prompt_regex.encode("utf-8"),
        ) as console:
            console.interrupt_autoboot(args.prompt_timeout)
            stage_artifact(console, kernel_fastboot, kernel, args)
            stage_artifact(console, dtb_fastboot, dtb, args)
            console.send_command(f"booti {KERNEL_LOAD_ADDR} - {DTB_LOAD_ADDR}")
            if args.no_console:
                time.sleep(0.5)
            else:
                console.interact()
    except (OSError, ValueError, TimeoutError, subprocess.SubprocessError) as error:
        status(f"failed: {error}")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
