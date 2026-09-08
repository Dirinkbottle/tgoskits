# K3 COM260 IFX 的 GRUB2/UEFI 启动

`spacemitk3-com260kit` 可以生成同时兼容 RISC-V Linux Image 直启和
PE32+ UEFI Application 的 StarryOS 镜像。开发板通过现有 Ubuntu GRUB2 加载
该镜像；部署过程不会替换 Ubuntu 的 EFI 文件，也不会修改 EFI BootOrder。

## 生成启动包

在仓库根目录执行：

```bash
cargo xtask starry grub package --config spacemitk3-com260kit
```

输出目录为 `target/starry-grub/spacemitk3-com260kit/`，其中包含：

- `starryos.efi`
- `spacemit-k3-com260-ifx.dtb`
- `grub.d/42_starryos`
- `default-grub.d/60-starryos.cfg`

打包命令会先执行正常的 StarryOS 构建，再检查内核 ELF 是无 TLS 的 ET_DYN，
并检查输出镜像的 RISC-V PE32+ 入口、节表和 EFI Application subsystem。DTS
通过 `dtc` 编译，因此主机还需要安装 device-tree-compiler。

若使用仓库中的 K3 同步脚本，构建、上传、板端分区预检、备份、原子安装和 GRUB
配置更新可由一个命令完成；唯一必填参数是开发板的 IPv4 地址：

```bash
board_script/k3com260kit/sync_kernel.py 10.42.0.100
```

脚本会按正常 SSH 和 `sudo` 交互流程请求认证，但不会把凭据写入文件、环境变量或
命令行。它会从新 DTB 读取预期的根标签和 ESP PARTUUID，拒绝部署到不匹配的
`/dev/sda3`、`/dev/sda1`，并在部署前后确认 EFI `BootOrder` 未改变。可先查看
不执行构建或远程操作的命令序列：

```bash
board_script/k3com260kit/sync_kernel.py --dry-run 10.42.0.100
```

## 部署前检查

在开发板 Ubuntu 中重新确认磁盘身份：

```bash
lsblk -o NAME,FSTYPE,LABEL,PARTLABEL,UUID,PARTUUID,MOUNTPOINTS
findmnt / /boot/efi
```

当前配置要求根分区标签为 `writable`，ESP 为 `/dev/sda1`，且 ESP PARTUUID 为
`3ae56870-fdf8-4df5-acc0-6d0edc70637a`。若分区表被重建或 PARTUUID 改变，必须
先更新 `spacemit-k3-com260-ifx.dts` 中的 `bootfs=` 并重新打包，不能继续部署旧
DTB。StarryOS 的实际根挂载由 `root=PARTLABEL=writable` 决定；`bootfs=` 只保留
板级启动信息。

## 安装菜单

先把整个输出目录上传到开发板的临时目录。安装前备份以下已有路径（若存在）：

```text
/boot/starryos
/etc/grub.d/42_starryos
/etc/default/grub.d/60-starryos.cfg
```

然后以 root 权限安装：

```bash
install -d -m 0755 /boot/starryos
install -m 0644 starryos.efi spacemit-k3-com260-ifx.dtb /boot/starryos/
install -m 0755 grub.d/42_starryos /etc/grub.d/42_starryos
install -d -m 0755 /etc/default/grub.d
install -m 0644 default-grub.d/60-starryos.cfg /etc/default/grub.d/60-starryos.cfg
update-grub
grub-script-check /boot/grub/grub.cfg
sync
```

该片段把 Ubuntu 保持为第 0 个默认菜单项，并把菜单显示时间设为 5 秒。StarryOS
菜单通过 `search --label writable` 定位根分区，不使用 initrd。

## 一次性启动与回退

先用 115200 8N1 连接开发板串口并开始记录，再执行：

```bash
grub-reboot starryos-k3-com260-ifx
sync
reboot
```

成功日志应显示 UEFI 入口、FDT、ExitBootServices、boot hart 0，并最终出现
`root@starry:`。`grub-reboot` 只影响下一次启动；StarryOS 再次重启或开发板复位
后会回到默认的 Ubuntu 项。

若日志停在 `VM Load Offset`，先核对 `Found FDT at address` 是否等于镜像装载基址
加 PE `SizeOfImage`。GRUB 可以把 DTB 紧贴在 EFI 镜像之后；someboot 必须只保留和
映射按 4 KiB 页对齐的实际镜像范围，不能把物理末端扩到 2 MiB 后误占 DTB。

该 Ubuntu 根文件系统的用户态会在启动早期调用 `prctl(PR_GET_AUXV)`。若日志中
出现 option `0x41555856` 不支持，说明运行的是未包含对应 Linux ABI 兼容修复的
旧 StarryOS 镜像，应重新执行打包和部署，而不是修改 GRUB 或分区配置。

若要完全移除菜单，恢复部署前备份（或删除上述三个 StarryOS 路径），重新执行
`update-grub` 和 `grub-script-check /boot/grub/grub.cfg`，最后 `sync`。整个流程不应
调用 `efibootmgr` 修改 BootOrder，也不应覆盖 Ubuntu 的 GRUB EFI 文件。
