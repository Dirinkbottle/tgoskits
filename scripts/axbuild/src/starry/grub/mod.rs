//! GRUB2/UEFI packaging for StarryOS board images.

mod artifact;
mod config;
mod menu;

use std::{
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, bail};
use clap::{Args, Subcommand};

use super::{Starry, build};
use crate::context::{SnapshotPersistence, StarryCliArgs};

#[derive(Args)]
pub struct ArgsGrub {
    #[command(subcommand)]
    command: GrubCommand,
}

#[derive(Subcommand)]
enum GrubCommand {
    /// Build and stage a validated GRUB2/UEFI bundle
    Package(ArgsGrubPackage),
}

#[derive(Args)]
struct ArgsGrubPackage {
    /// Board config path or board name under StarryOS configs/board
    #[arg(long)]
    config: PathBuf,

    /// Override the final bundle directory
    #[arg(long)]
    output_dir: Option<PathBuf>,
}

pub(super) async fn execute(starry: &mut Starry, args: ArgsGrub) -> anyhow::Result<()> {
    match args.command {
        GrubCommand::Package(args) => package(starry, args).await,
    }
}

async fn package(starry: &mut Starry, args: ArgsGrubPackage) -> anyhow::Result<()> {
    let request = starry.prepare_request(
        StarryCliArgs {
            config: Some(args.config),
            arch: None,
            target: None,
            smp: None,
            debug: false,
        },
        None,
        None,
        SnapshotPersistence::Discard,
    )?;
    if request.arch != "riscv64" {
        bail!(
            "Starry GRUB packaging currently requires a RISC-V board config, got `{}`",
            request.arch
        );
    }

    let package_config = config::GrubPackageConfig::load(&request.build_info_path)?;
    let output_dir = resolve_output_dir(
        starry.app.workspace_root(),
        &request.build_info_path,
        args.output_dir,
    )?;

    starry.app.set_debug_mode(false)?;
    let cargo = build::load_cargo_config(&request)?;
    let output = starry.build_artifact(&request, cargo).await?;
    artifact::validate_kernel_elf(output.elf_path())?;

    let staging_dir = staging_path(&output_dir)?;
    if staging_dir.exists() {
        fs::remove_dir_all(&staging_dir).with_context(|| {
            format!(
                "failed to remove stale staging directory {}",
                staging_dir.display()
            )
        })?;
    }
    fs::create_dir_all(&staging_dir).with_context(|| {
        format!(
            "failed to create staging directory {}",
            staging_dir.display()
        )
    })?;

    let stage_result = (|| {
        artifact::stage_kernel_and_dtb(
            output.elf_path(),
            &request.build_info_path,
            &package_config,
            &staging_dir,
        )?;
        menu::stage_grub_fragments(&package_config, &staging_dir)
    })();
    if let Err(error) = stage_result {
        let _ = fs::remove_dir_all(&staging_dir);
        return Err(error);
    }

    replace_bundle(&staging_dir, &output_dir)?;
    println!("Starry GRUB bundle: {}", output_dir.display());
    Ok(())
}

fn resolve_output_dir(
    workspace_root: &Path,
    build_config_path: &Path,
    explicit: Option<PathBuf>,
) -> anyhow::Result<PathBuf> {
    if let Some(path) = explicit {
        return Ok(if path.is_absolute() {
            path
        } else {
            workspace_root.join(path)
        });
    }
    let board_name = build_config_path
        .file_stem()
        .and_then(|name| name.to_str())
        .context("Starry board config filename is not valid UTF-8")?;
    Ok(workspace_root.join("target/starry-grub").join(board_name))
}

fn staging_path(output_dir: &Path) -> anyhow::Result<PathBuf> {
    let parent = output_dir.parent().with_context(|| {
        format!(
            "GRUB bundle output {} has no parent directory",
            output_dir.display()
        )
    })?;
    let name = output_dir
        .file_name()
        .and_then(|name| name.to_str())
        .context("GRUB bundle output filename is not valid UTF-8")?;
    fs::create_dir_all(parent)
        .with_context(|| format!("failed to create bundle parent {}", parent.display()))?;
    Ok(parent.join(format!(".{name}.staging-{}", std::process::id())))
}

fn replace_bundle(staging_dir: &Path, output_dir: &Path) -> anyhow::Result<()> {
    let backup_dir = output_dir.with_extension(format!("previous-{}", std::process::id()));
    if backup_dir.exists() {
        fs::remove_dir_all(&backup_dir).with_context(|| {
            format!(
                "failed to remove stale bundle backup {}",
                backup_dir.display()
            )
        })?;
    }
    if output_dir.exists() {
        fs::rename(output_dir, &backup_dir).with_context(|| {
            format!(
                "failed to move existing bundle {} to {}",
                output_dir.display(),
                backup_dir.display()
            )
        })?;
    }

    if let Err(error) = fs::rename(staging_dir, output_dir) {
        if backup_dir.exists() {
            let _ = fs::rename(&backup_dir, output_dir);
        }
        return Err(error).with_context(|| {
            format!(
                "failed to commit staged GRUB bundle {} to {}",
                staging_dir.display(),
                output_dir.display()
            )
        });
    }
    if backup_dir.exists() {
        fs::remove_dir_all(&backup_dir).with_context(|| {
            format!(
                "failed to remove replaced bundle backup {}",
                backup_dir.display()
            )
        })?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use tempfile::tempdir;

    use super::*;

    #[test]
    fn default_output_uses_board_config_stem() {
        let root = Path::new("/workspace");
        let output = resolve_output_dir(
            root,
            Path::new("/workspace/os/StarryOS/configs/board/k3.toml"),
            None,
        )
        .unwrap();

        assert_eq!(output, Path::new("/workspace/target/starry-grub/k3"));
    }

    #[test]
    fn replacement_commits_complete_staging_directory() {
        let root = tempdir().unwrap();
        let output = root.path().join("bundle");
        let staging = root.path().join(".bundle.staging");
        fs::create_dir_all(&output).unwrap();
        fs::write(output.join("old"), "old").unwrap();
        fs::create_dir_all(&staging).unwrap();
        fs::write(staging.join("new"), "new").unwrap();

        replace_bundle(&staging, &output).unwrap();

        assert!(!staging.exists());
        assert!(!output.join("old").exists());
        assert_eq!(fs::read_to_string(output.join("new")).unwrap(), "new");
    }
}
