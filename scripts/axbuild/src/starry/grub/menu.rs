use std::{fs, path::Path};

use anyhow::Context;

use super::config::GrubPackageConfig;

pub(super) fn stage_grub_fragments(
    config: &GrubPackageConfig,
    staging_dir: &Path,
) -> anyhow::Result<()> {
    let menu_dir = staging_dir.join("grub.d");
    let defaults_dir = staging_dir.join("default-grub.d");
    fs::create_dir_all(&menu_dir)
        .with_context(|| format!("failed to create {}", menu_dir.display()))?;
    fs::create_dir_all(&defaults_dir)
        .with_context(|| format!("failed to create {}", defaults_dir.display()))?;

    let menu_path = menu_dir.join("42_starryos");
    fs::write(&menu_path, render_menu(config))
        .with_context(|| format!("failed to write {}", menu_path.display()))?;
    set_executable(&menu_path)?;

    let defaults_path = defaults_dir.join("60-starryos.cfg");
    fs::write(&defaults_path, render_defaults())
        .with_context(|| format!("failed to write {}", defaults_path.display()))?;
    Ok(())
}

fn render_menu(config: &GrubPackageConfig) -> String {
    format!(
        "#!/bin/sh\nexec tail -n +3 $0\nmenuentry '{}' --id {} {{\ninsmod part_gpt\ninsmod \
         ext2\nsearch --no-floppy --label --set=root {}\nlinux {}/starryos.efi\ndevicetree \
         {}/spacemit-k3-com260-ifx.dtb\n}}\n",
        config.menu_title,
        config.menu_id,
        config.root_label,
        config.install_dir,
        config.install_dir
    )
}

fn render_defaults() -> &'static str {
    "# Keep Ubuntu as the default while exposing the one-shot StarryOS \
     entry.\nGRUB_DEFAULT=0\nGRUB_TIMEOUT_STYLE=menu\nGRUB_TIMEOUT=5\n"
}

#[cfg(unix)]
fn set_executable(path: &Path) -> anyhow::Result<()> {
    use std::os::unix::fs::PermissionsExt;

    let mut permissions = fs::metadata(path)?.permissions();
    permissions.set_mode(0o755);
    fs::set_permissions(path, permissions)
        .with_context(|| format!("failed to mark {} executable", path.display()))
}

#[cfg(not(unix))]
fn set_executable(_path: &Path) -> anyhow::Result<()> {
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn menu_uses_label_linux_and_repository_dtb() {
        let menu = render_menu(&test_config());

        for expected in [
            "menuentry 'StarryOS (SpacemiT K3 COM260 IFX)' --id starryos-k3-com260-ifx",
            "search --no-floppy --label --set=root writable",
            "linux /boot/starryos/starryos.efi",
            "devicetree /boot/starryos/spacemit-k3-com260-ifx.dtb",
        ] {
            assert!(menu.contains(expected), "missing {expected:?} in:\n{menu}");
        }
        assert!(!menu.contains("initrd"));
    }

    #[test]
    fn defaults_keep_ubuntu_first_and_show_menu_for_five_seconds() {
        assert_eq!(
            render_defaults(),
            "# Keep Ubuntu as the default while exposing the one-shot StarryOS \
             entry.\nGRUB_DEFAULT=0\nGRUB_TIMEOUT_STYLE=menu\nGRUB_TIMEOUT=5\n"
        );
    }

    fn test_config() -> GrubPackageConfig {
        GrubPackageConfig {
            dts: "spacemit-k3-com260-ifx.dts".to_string(),
            install_dir: "/boot/starryos".to_string(),
            root_label: "writable".to_string(),
            menu_id: "starryos-k3-com260-ifx".to_string(),
            menu_title: "StarryOS (SpacemiT K3 COM260 IFX)".to_string(),
        }
    }
}
