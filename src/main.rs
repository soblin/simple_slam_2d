use std::fs;
use std::path::Path;
use std::process::Command;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // (1) call ros2 pkg prefix --share simple_slam_2d and get path to share directory
    let cmd = Command::new("ros2")
        .arg("pkg")
        .arg("prefix")
        .arg("--share")
        .arg("simple_slam_2d")
        .output()?;

    let pkg_share_directory = String::from_utf8(cmd.stdout)?;
    println!("pkg_share_directory = {}", pkg_share_directory);

    // (2) then open data/*.lsc
    Ok(())
}
