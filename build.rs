
use std::process::Command;
use std::path::Path;
use std::{env, fs};

fn main() {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let src = Path::new(&manifest_dir);

    let out_dir = env::var("OUT_DIR").unwrap();
    let dst = Path::new(&out_dir);

    let mut conf = Command::new("cmake");
    conf.current_dir(&src.join("Chipmunk2D"))
        .arg("-DBUILD_DEMOS=NO")
        .arg("-DBUILD_SHARED=NO")
        .arg("-DBUILD_STATIC=YES")
        .arg("-DCMAKE_BUILD_TYPE=Release")
        .arg(&format!("-DCMAKE_C_FLAGS={} {}", "-fPIC", "-DCP_USE_DOUBLES=0"));
    run(&mut conf);


    let mut build = Command::new("make");
    build.current_dir(&src.join("Chipmunk2D"));
    run(&mut build);

    fs::copy(
        &src.join("Chipmunk2D/src/libchipmunk.a"), 
        &dst.join("libchipmunk.a")
    ).unwrap();

    println!("cargo:rustc-flags= -l chipmunk:static -L {}", dst.display());
}

fn run(cmd: &mut Command){
    println!("[running] {:?}", cmd);
    let output = cmd.output().unwrap();
    println!("[status] {:?}", output.status);
    println!("[stdout] {:?}", String::from_utf8_lossy(&output.stdout));
    println!("[stderr] {:?}", String::from_utf8_lossy(&output.stderr));
    assert!(output.status.success());
}
