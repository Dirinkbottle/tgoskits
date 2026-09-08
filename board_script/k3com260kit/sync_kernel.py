#!/usr/bin/env python3
"""Build and atomically deploy the K3 StarryOS GRUB2 bundle to one board."""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import os
import secrets
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Sequence

PROJECT_ROOT = Path(__file__).resolve().parents[2]
BUILD_CONFIG = "spacemitk3-com260kit"
BUNDLE_DIR = PROJECT_ROOT / "target" / "starry-grub" / BUILD_CONFIG
EFI_NAME = "starryos.efi"
DTB_NAME = "spacemit-k3-com260-ifx.dtb"
GRUB_SCRIPT_NAME = "42_starryos"
GRUB_DEFAULT_NAME = "60-starryos.cfg"
BOARD_USER = "ubuntu"
BOARD_ROOT_DEVICE = "/dev/sda3"
BOARD_ESP_DEVICE = "/dev/sda1"
REMOTE_INSTALL_DIR = "/boot/starryos"
REMOTE_GRUB_SCRIPT = "/etc/grub.d/42_starryos"
REMOTE_GRUB_DEFAULT = "/etc/default/grub.d/60-starryos.cfg"
GRUB_MENU_ID = "starryos-k3-com260-ifx"


def status(message: str) -> None:
    print(f"[k3com260kit-sync] {message}", file=sys.stderr, flush=True)


def run(command: Sequence[str], *, cwd: Path = PROJECT_ROOT) -> None:
    status("$ " + shlex.join(command))
    subprocess.run(command, cwd=cwd, check=True)


def output(command: Sequence[str], *, cwd: Path = PROJECT_ROOT) -> str:
    status("$ " + shlex.join(command))
    return subprocess.check_output(command, cwd=cwd, text=True).strip()


def require_command(command: str) -> None:
    if shutil.which(command) is None:
        raise ValueError(f"required host command is not available: {command}")


def parse_board_ip(value: str) -> str:
    address = ipaddress.ip_address(value)
    if address.version != 4:
        raise ValueError("the K3 deployment script currently requires an IPv4 board address")
    return str(address)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as artifact:
        for block in iter(lambda: artifact.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def boot_contract(dtb: Path) -> tuple[str, str]:
    bootargs = output(["fdtget", str(dtb), "/chosen", "bootargs"])
    root_label = next(
        (
            argument.removeprefix("root=PARTLABEL=")
            for argument in bootargs.split()
            if argument.startswith("root=PARTLABEL=")
        ),
        None,
    )
    esp_partuuid = next(
        (
            argument.removeprefix("bootfs=PARTUUID=")
            for argument in bootargs.split()
            if argument.startswith("bootfs=PARTUUID=")
        ),
        None,
    )
    if not root_label or not esp_partuuid:
        raise ValueError(f"DTB bootargs lack root=PARTLABEL or bootfs=PARTUUID: {bootargs}")
    return root_label, esp_partuuid


def build_bundle() -> None:
    run(["cargo", "xtask", "starry", "grub", "package", "--config", BUILD_CONFIG])


def validate_bundle() -> tuple[dict[str, Path], str, str, dict[str, str]]:
    artifacts = {
        "efi": BUNDLE_DIR / EFI_NAME,
        "dtb": BUNDLE_DIR / DTB_NAME,
        "grub_script": BUNDLE_DIR / "grub.d" / GRUB_SCRIPT_NAME,
        "grub_default": BUNDLE_DIR / "default-grub.d" / GRUB_DEFAULT_NAME,
    }
    missing = [str(path) for path in artifacts.values() if not path.is_file()]
    if missing:
        raise ValueError("GRUB bundle is incomplete:\n" + "\n".join(missing))

    run(["grub-file", "--is-riscv64-efi", str(artifacts["efi"])])
    root_label, esp_partuuid = boot_contract(artifacts["dtb"])
    checksums = {name: sha256(path) for name, path in artifacts.items()}
    return artifacts, root_label, esp_partuuid, checksums


class BoardConnection:
    """One SSH multiplexed connection that never stores credentials on disk."""

    def __init__(self, board_ip: str) -> None:
        self.target = f"{BOARD_USER}@{board_ip}"
        self.control_path = Path(f"/tmp/k3-grub-sync-{os.getpid()}-{secrets.token_hex(4)}.sock")
        self.options = [
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-o",
            "ConnectTimeout=8",
            "-o",
            "ControlMaster=auto",
            "-o",
            "ControlPersist=120",
            "-o",
            f"ControlPath={self.control_path}",
        ]

    def __enter__(self) -> BoardConnection:
        status(f"connecting to {self.target}; SSH may prompt for its password")
        run(["ssh", *self.options, self.target, "true"])
        return self

    def __exit__(self, _exc_type: object, _exc_value: object, _traceback: object) -> None:
        subprocess.run(
            ["ssh", *self.options, "-O", "exit", self.target],
            cwd=PROJECT_ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    def remote(self, script: str, *, tty: bool = False) -> None:
        command = ["ssh", *self.options]
        if tty:
            command.append("-tt")
        command.extend([self.target, "bash -lc " + shlex.quote(script)])
        status("$ " + shlex.join(command[:-1]) + " <remote command>")
        subprocess.run(command, cwd=PROJECT_ROOT, check=True)

    def copy(self, sources: Sequence[Path], destination: str) -> None:
        command = ["scp", *self.options, *(str(source) for source in sources)]
        command.append(f"{self.target}:{destination}")
        run(command)


def prepare_remote_stage(connection: BoardConnection) -> str:
    stage = f"/tmp/starryos-grub-sync-{os.getpid()}-{secrets.token_hex(6)}"
    connection.remote(
        f"""set -eu
umask 077
mkdir -p {stage}/grub.d {stage}/default-grub.d
"""
    )
    return stage


def upload_bundle(
    connection: BoardConnection,
    stage: str,
    artifacts: dict[str, Path],
) -> None:
    connection.copy([artifacts["efi"], artifacts["dtb"]], f"{stage}/")
    connection.copy([artifacts["grub_script"]], f"{stage}/grub.d/")
    connection.copy([artifacts["grub_default"]], f"{stage}/default-grub.d/")


def deployment_script(
    stage: str,
    root_label: str,
    esp_partuuid: str,
    checksums: dict[str, str],
) -> str:
    return f"""set -eu

stage={stage}
expected_efi={checksums["efi"]}
expected_dtb={checksums["dtb"]}
expected_grub_script={checksums["grub_script"]}
expected_grub_default={checksums["grub_default"]}

test "$(sha256sum "$stage/{EFI_NAME}" | awk '{{print $1}}')" = "$expected_efi"
test "$(sha256sum "$stage/{DTB_NAME}" | awk '{{print $1}}')" = "$expected_dtb"
test "$(sha256sum "$stage/grub.d/{GRUB_SCRIPT_NAME}" | awk '{{print $1}}')" = "$expected_grub_script"
test "$(sha256sum "$stage/default-grub.d/{GRUB_DEFAULT_NAME}" | awk '{{print $1}}')" = "$expected_grub_default"

echo "Authenticating sudo; its password is requested interactively and is not stored."
sudo -v

test "$(findmnt -n -o SOURCE /)" = "{BOARD_ROOT_DEVICE}"
test "$(findmnt -n -o SOURCE /boot/efi)" = "{BOARD_ESP_DEVICE}"
test "$(blkid -s PARTLABEL -o value {BOARD_ROOT_DEVICE})" = "{root_label}"
test "$(blkid -s PARTUUID -o value {BOARD_ESP_DEVICE})" = "{esp_partuuid}"

boot_order_before=$(sudo efibootmgr | awk -F': ' '/^BootOrder:/ {{print $2}}')
test -n "$boot_order_before"

stamp=$(date -u +%Y%m%d-%H%M%S)
backup=/boot/starryos.backup-$stamp-sync
test ! -e "$backup"
sudo install -d -m 0755 "$backup/grub"
if test -e {REMOTE_INSTALL_DIR}; then
    sudo cp -a {REMOTE_INSTALL_DIR} "$backup/starryos"
fi
if test -e {REMOTE_GRUB_SCRIPT}; then
    sudo cp -a {REMOTE_GRUB_SCRIPT} "$backup/grub/{GRUB_SCRIPT_NAME}"
fi
if test -e {REMOTE_GRUB_DEFAULT}; then
    sudo cp -a {REMOTE_GRUB_DEFAULT} "$backup/grub/{GRUB_DEFAULT_NAME}"
fi

sudo install -d -m 0755 {REMOTE_INSTALL_DIR} /etc/default/grub.d
sudo install -m 0755 "$stage/{EFI_NAME}" {REMOTE_INSTALL_DIR}/.{EFI_NAME}.$stamp.new
sudo install -m 0644 "$stage/{DTB_NAME}" {REMOTE_INSTALL_DIR}/.{DTB_NAME}.$stamp.new
sudo mv -f {REMOTE_INSTALL_DIR}/.{EFI_NAME}.$stamp.new {REMOTE_INSTALL_DIR}/{EFI_NAME}
sudo mv -f {REMOTE_INSTALL_DIR}/.{DTB_NAME}.$stamp.new {REMOTE_INSTALL_DIR}/{DTB_NAME}
sudo install -m 0755 "$stage/grub.d/{GRUB_SCRIPT_NAME}" /etc/grub.d/.{GRUB_SCRIPT_NAME}.$stamp.new
sudo mv -f /etc/grub.d/.{GRUB_SCRIPT_NAME}.$stamp.new {REMOTE_GRUB_SCRIPT}
sudo install -m 0644 "$stage/default-grub.d/{GRUB_DEFAULT_NAME}" /etc/default/grub.d/.{GRUB_DEFAULT_NAME}.$stamp.new
sudo mv -f /etc/default/grub.d/.{GRUB_DEFAULT_NAME}.$stamp.new {REMOTE_GRUB_DEFAULT}

sudo update-grub
sudo grub-script-check /boot/grub/grub.cfg
grep -Fq -- "--id {GRUB_MENU_ID}" /boot/grub/grub.cfg
grep -Fq "linux {REMOTE_INSTALL_DIR}/{EFI_NAME}" /boot/grub/grub.cfg
grep -Fq "devicetree {REMOTE_INSTALL_DIR}/{DTB_NAME}" /boot/grub/grub.cfg
sudo sync

test "$(sha256sum {REMOTE_INSTALL_DIR}/{EFI_NAME} | awk '{{print $1}}')" = "$expected_efi"
test "$(sha256sum {REMOTE_INSTALL_DIR}/{DTB_NAME} | awk '{{print $1}}')" = "$expected_dtb"
boot_order_after=$(sudo efibootmgr | awk -F': ' '/^BootOrder:/ {{print $2}}')
test "$boot_order_before" = "$boot_order_after"

printf 'Installed StarryOS bundle; backup: %s\n' "$backup"
printf 'EFI BootOrder unchanged: %s\n' "$boot_order_after"
printf 'Temporary uploaded bundle retained at: %s\n' "$stage"
"""


def print_dry_run(board_ip: str) -> None:
    print(f"cargo xtask starry grub package --config {BUILD_CONFIG}")
    print(f"ssh {BOARD_USER}@{board_ip} 'create a private /tmp staging directory'")
    print(f"scp {BUNDLE_DIR}/{{{EFI_NAME},{DTB_NAME}}} {BOARD_USER}@{board_ip}:<stage>/")
    print(f"ssh -tt {BOARD_USER}@{board_ip} 'preflight, backup, atomic install, update-grub'")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build and deploy the StarryOS K3 GRUB2 bundle. "
            "The only required argument is the board's IPv4 address."
        )
    )
    parser.add_argument("board_ip", help="K3 Ubuntu IPv4 address, for example 10.42.0.100")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the deployment sequence without building or contacting the board",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        board_ip = parse_board_ip(args.board_ip)
        if args.dry_run:
            print_dry_run(board_ip)
            return 0

        for command in ("cargo", "ssh", "scp", "grub-file", "fdtget"):
            require_command(command)

        build_bundle()
        artifacts, root_label, esp_partuuid, checksums = validate_bundle()
        with BoardConnection(board_ip) as connection:
            stage = prepare_remote_stage(connection)
            upload_bundle(connection, stage, artifacts)
            connection.remote(
                deployment_script(stage, root_label, esp_partuuid, checksums),
                tty=True,
            )
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        status(f"failed: {error}")
        return 1

    status("completed; Ubuntu remains the default boot entry")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
