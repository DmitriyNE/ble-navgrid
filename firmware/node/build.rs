//! Puts `memory.x` on the linker search path, adds the cortex-m-rt / defmt linker scripts, and
//! stamps the build with its commit: `NAVGRID_COMMIT` (the first 8 hex digits) and
//! `NAVGRID_DIRTY` (`-dirty` when tracked files had uncommitted changes).

use std::path::PathBuf;
use std::process::Command;
use std::{env, fs};

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo::rustc-link-search={}", out.display());
    println!("cargo::rerun-if-changed=memory.x");

    println!("cargo::rustc-link-arg-bins=--nmagic");
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rustc-link-arg-bins=-Tdefmt.x");

    let commit = git(&["rev-parse", "HEAD"])
        .and_then(|h| h.get(..8).map(str::to_owned))
        .unwrap_or_else(|| "00000000".into());
    let dirty = git(&["status", "--porcelain", "--untracked-files=no"]).is_none_or(|s| !s.is_empty());
    println!("cargo::rustc-env=NAVGRID_COMMIT={commit}");
    println!("cargo::rustc-env=NAVGRID_DIRTY={}", if dirty { "-dirty" } else { "" });
    // Restamp on a commit or checkout, and on any change to what the image is built from.
    if let Some(root) = git(&["rev-parse", "--show-toplevel"]) {
        for path in [
            ".git/logs/HEAD",
            ".git/index",
            "crates",
            "firmware/node/src",
            "firmware/node/Cargo.toml",
            "firmware/Cargo.lock",
        ] {
            println!("cargo::rerun-if-changed={root}/{path}");
        }
    }
}
