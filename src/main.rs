use std::fs::File;
use std::io::{self, prelude::*, BufReader};
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
    let pkg_share_directory = pkg_share_directory.trim();

    // (2) then open data/*.lsc
    let pkg_share_directory = Path::new(&pkg_share_directory);
    let data_directory = pkg_share_directory.join("dataset");
    let file_name = String::from("corridor.lsc");
    let lsc_file_path = data_directory.join(&file_name);
    let fp = BufReader::new(File::open(lsc_file_path).expect("Unable to open file"));
    println!("there are {} lines.", fp.lines().count());

    // (3) main loop: iteratively read each line and process them
    Ok(())
}
