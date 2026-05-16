// Find the actual path of memory.x and add it to link search, required for building in workspace
fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={}", manifest_dir);
}
