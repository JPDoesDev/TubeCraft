//! Build tasks, so the plugin builds with only Rust installed:
//!
//! - `cargo xtask build`  compile to wasm32 and wrap the module as a component in `tubecraft/`
//! - `cargo xtask check`  build, then `solidcraft-cli plugin check tubecraft`
//! - `cargo xtask test`   build, then `solidcraft-cli plugin test tubecraft --require-cases`
//! - `cargo xtask dist`   build, check, then zip the package into `dist/` (tests left out)
//! - `cargo xtask fixtures` remake the test designs that hold earlier tubes: each
//!   `fixtures/NN-<name>.json` is a test case run on its design, in file order, and the result is
//!   saved as `tubecraft/tests/<name>.sc` for later fixtures and cases to start from
//!
//! `build` does what `wasm-tools component new` does, through the same `wit-component` library.

use std::path::{Path, PathBuf};
use std::process::{exit, Command};

const PACKAGE: &str = "tubecraft";
const CRATE_WASM: &str = "tubecraft_plugin.wasm";

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap().to_path_buf()
}

fn run(cmd: &mut Command) {
    eprintln!("$ {cmd:?}");
    let status = cmd.status().unwrap_or_else(|e| {
        eprintln!("could not start {cmd:?}: {e}");
        exit(1)
    });
    if !status.success() {
        exit(status.code().unwrap_or(1));
    }
}

fn build() {
    let root = root();
    run(Command::new(env!("CARGO"))
        .current_dir(&root)
        .args(["build", "-p", "tubecraft-plugin", "--release", "--target", "wasm32-unknown-unknown"]));
    let module = root.join("target/wasm32-unknown-unknown/release").join(CRATE_WASM);
    let bytes = std::fs::read(&module).unwrap_or_else(|e| {
        eprintln!("{}: {e}", module.display());
        exit(1)
    });
    let component = wit_component::ComponentEncoder::default()
        .module(&bytes)
        .and_then(|e| e.validate(true).encode())
        .unwrap_or_else(|e| {
            eprintln!("could not make a component: {e:?}");
            exit(1)
        });
    let out = root.join(PACKAGE).join(format!("{PACKAGE}.wasm"));
    std::fs::write(&out, &component).unwrap();
    eprintln!("wrote {} ({} bytes)", out.display(), component.len());
}

fn cli(args: &[&str]) {
    run(Command::new("solidcraft-cli").current_dir(root()).args(args));
}

fn version() -> String {
    let manifest = std::fs::read_to_string(root().join(PACKAGE).join("plugin.json")).unwrap();
    let at = manifest.find("\"version\"").expect("plugin.json has a version");
    let rest = &manifest[at + 9..];
    let start = rest.find('"').unwrap() + 1;
    let end = start + rest[start..].find('"').unwrap();
    rest[start..end].to_string()
}

fn dist() {
    let root = root();
    let dir = root.join("dist");
    std::fs::create_dir_all(&dir).unwrap();
    let zip = dir.join(format!("{PACKAGE}-{}.zip", version()));
    let _ = std::fs::remove_file(&zip);
    // The zip holds one top-level folder named after the plugin id, without tests/.
    let mut files = vec![format!("{PACKAGE}/plugin.json"), format!("{PACKAGE}/{PACKAGE}.wasm")];
    let icons = root.join(PACKAGE).join("icons");
    if let Ok(entries) = std::fs::read_dir(&icons) {
        for e in entries.flatten() {
            files.push(format!("{PACKAGE}/icons/{}", e.file_name().to_string_lossy()));
        }
    }
    run(Command::new("zip").current_dir(&root).arg("-X").arg(&zip).args(&files));
    cli(&["plugin", "check", zip.to_str().unwrap()]);
    eprintln!("wrote {}", zip.display());
}

fn fixtures() {
    build();
    let root = root();
    let tests = root.join(PACKAGE).join("tests");
    let mut stages: Vec<PathBuf> = std::fs::read_dir(root.join("fixtures"))
        .expect("fixtures/")
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    stages.sort();
    for stage in stages {
        let stem = stage.file_stem().unwrap().to_string_lossy().to_string();
        let name = stem.split_once('-').map_or(stem.as_str(), |(_, n)| n).to_string();
        let work = root.join("target/fixtures");
        let _ = std::fs::remove_dir_all(&work);
        let pkg = work.join(PACKAGE);
        std::fs::create_dir_all(pkg.join("tests")).unwrap();
        for f in ["plugin.json", &format!("{PACKAGE}.wasm")] {
            std::fs::copy(root.join(PACKAGE).join(f), pkg.join(f)).unwrap();
        }
        for e in std::fs::read_dir(&tests).unwrap().flatten() {
            if e.path().extension().is_some_and(|x| x == "sc") {
                std::fs::copy(e.path(), pkg.join("tests").join(e.file_name())).unwrap();
            }
        }
        std::fs::copy(&stage, pkg.join("tests").join(format!("{name}.json"))).unwrap();
        let out = work.join("out");
        cli(&["plugin", "test", pkg.to_str().unwrap(), "--out", out.to_str().unwrap()]);
        std::fs::copy(out.join(format!("{name}.sc")), tests.join(format!("{name}.sc"))).unwrap();
        eprintln!("wrote tubecraft/tests/{name}.sc");
    }
}

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("build") => build(),
        Some("check") => {
            build();
            cli(&["plugin", "check", PACKAGE]);
        }
        Some("test") => {
            build();
            cli(&["plugin", "test", PACKAGE, "--require-cases"]);
        }
        Some("dist") => {
            build();
            cli(&["plugin", "check", PACKAGE]);
            dist();
        }
        Some("fixtures") => fixtures(),
        _ => {
            eprintln!("usage: cargo xtask build|check|test|dist|fixtures");
            exit(2);
        }
    }
}
