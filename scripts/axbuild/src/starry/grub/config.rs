use std::{fs, path::Path};

use anyhow::{Context, bail};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct GrubConfigFile {
    grub: Option<GrubPackageConfig>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
pub(super) struct GrubPackageConfig {
    pub(super) dts: String,
    pub(super) install_dir: String,
    pub(super) root_label: String,
    pub(super) menu_id: String,
    pub(super) menu_title: String,
}

impl GrubPackageConfig {
    pub(super) fn load(path: &Path) -> anyhow::Result<Self> {
        let contents = fs::read_to_string(path)
            .with_context(|| format!("failed to read Starry board config {}", path.display()))?;
        let file: GrubConfigFile = toml::from_str(&contents)
            .with_context(|| format!("failed to parse Starry board config {}", path.display()))?;
        let config = file.grub.with_context(|| {
            format!(
                "Starry board config {} does not define a [grub] table",
                path.display()
            )
        })?;
        config.validate()?;
        Ok(config)
    }

    fn validate(&self) -> anyhow::Result<()> {
        validate_component("dts", &self.dts, false)?;
        validate_component("root_label", &self.root_label, false)?;
        validate_component("menu_id", &self.menu_id, false)?;
        validate_component("menu_title", &self.menu_title, true)?;
        if !self.install_dir.starts_with('/')
            || self.install_dir.ends_with('/')
            || self.install_dir.contains("..")
            || self.install_dir.chars().any(char::is_whitespace)
        {
            bail!(
                "GRUB install_dir must be an absolute path without whitespace, `..`, or a \
                 trailing slash"
            );
        }
        Ok(())
    }
}

fn validate_component(field: &str, value: &str, allow_spaces: bool) -> anyhow::Result<()> {
    if value.is_empty()
        || value.contains(['\n', '\r', '\'', '"'])
        || (!allow_spaces && value.chars().any(char::is_whitespace))
    {
        bail!("GRUB {field} contains unsupported characters");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use tempfile::tempdir;

    use super::*;

    #[test]
    fn loads_complete_grub_table() {
        let root = tempdir().unwrap();
        let config_path = root.path().join("k3.toml");
        fs::write(
            &config_path,
            r#"
target = "riscv64gc-unknown-none-elf"

[grub]
dts = "board.dts"
install_dir = "/boot/starryos"
root_label = "writable"
menu_id = "starryos-k3"
menu_title = "StarryOS K3"
"#,
        )
        .unwrap();

        let config = GrubPackageConfig::load(&config_path).unwrap();

        assert_eq!(config.dts, "board.dts");
        assert_eq!(config.install_dir, "/boot/starryos");
        assert_eq!(config.root_label, "writable");
        assert_eq!(config.menu_id, "starryos-k3");
        assert_eq!(config.menu_title, "StarryOS K3");
    }

    #[test]
    fn rejects_missing_grub_table_and_script_injection() {
        let root = tempdir().unwrap();
        let missing = root.path().join("missing.toml");
        fs::write(&missing, "target = \"riscv64gc-unknown-none-elf\"\n").unwrap();
        assert!(
            GrubPackageConfig::load(&missing)
                .unwrap_err()
                .to_string()
                .contains("does not define a [grub] table")
        );

        let injected = root.path().join("injected.toml");
        fs::write(
            &injected,
            r#"
[grub]
dts = "board.dts"
install_dir = "/boot/starryos"
root_label = "writable"
menu_id = "bad'id"
menu_title = "StarryOS"
"#,
        )
        .unwrap();
        assert!(GrubPackageConfig::load(&injected).is_err());
    }
}
