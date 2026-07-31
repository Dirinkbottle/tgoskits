//! Kernel build script: generates `linker.x` from `linker.ld` and `amp_gen.rs`
//! from the outer project's `amp.toml`.
//!
//! The kernel sits at `tgoskits/os/StarryOS/kernel/` inside the tgoskits
//! monorepo submodule. `amp.toml` lives at the outer project root.
//!
//! We parse the TOML manually to avoid dependency on the `toml` crate
//! (which has version conflicts across the workspace).

use std::collections::HashMap;
use std::path::Path;

fn main() {
    println!("cargo:rerun-if-changed=linker.ld");
    println!("cargo:rustc-check-cfg=cfg(axtest)");

    let out_dir = std::env::var("OUT_DIR").unwrap();
    let linker = format!("{out_dir}/linker.x");

    std::fs::write(&linker, include_str!("linker.ld")).unwrap();
    println!("cargo:rustc-link-search={out_dir}");

    let target_dir = std::path::Path::new(&out_dir).join("../../..");
    std::fs::write(target_dir.join("linker.x"), include_str!("linker.ld")).unwrap();

    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();

    let amp_toml_path = std::env::var("AMP_TOML_PATH")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| {
            Path::new(&manifest_dir)
                .join("../../../..") // kernel → StarryOS → os → tgoskits → outer
                .join("amp.toml")
        });

    println!("cargo:rerun-if-changed={}", amp_toml_path.display());
    println!("cargo:rerun-if-env-changed=AMP_TOML_PATH");

    let config = load_amp_toml(&amp_toml_path);
    generate_amp_rs(&config, Path::new(&out_dir));
}

fn load_amp_toml(path: &Path) -> HashMap<String, String> {
    let content =
        std::fs::read_to_string(path).unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
    let mut map = HashMap::new();
    let mut in_table = false;

    for line in content.lines() {
        let trimmed = line.trim();

        // Skip comments, empty lines, and table headers
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        if trimmed.starts_with('[') {
            in_table = true;
            continue;
        }
        if in_table {
            continue;
        }

        // Parse `KEY = VALUE` (with optional quotes around strings)
        if let Some(eq) = trimmed.find('=') {
            let key = trimmed[..eq].trim().to_string();
            let val = trimmed[eq + 1..].trim();
            // Strip surrounding quotes
            let val = val.trim_matches('"').trim_matches('\'');
            map.insert(key, val.to_string());
        }
    }
    map
}

fn generate_amp_rs(config: &HashMap<String, String>, out_dir: &Path) {
    let keys = [
        "SHMBASE",
        "SHMSIZE",
        "CLINTBASE",
        "UART0BASE",
        "UART1BASE",
        "OPENSBIBASE",
        "STARRYOSBASE",
        "RTASYNCBASE",
        "RTASYNCSIZE",
    ];

    let mut buf = String::from("// Auto-generated from amp.toml. Do not edit.\n\n");
    for key in &keys {
        if let Some(val) = config.get(*key) {
            buf.push_str(&format!("pub const {key}: usize = {val};\n"));
        }
    }

    let out_path = out_dir.join("amp_gen.rs");
    std::fs::write(&out_path, &buf)
        .unwrap_or_else(|e| panic!("failed to write {}: {}", out_path.display(), e));
}
