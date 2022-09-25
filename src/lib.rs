pub fn get_package_share_directory(pkg: &str) -> Result<String, Box<dyn std::error::Error>> {
    let cmd = std::process::Command::new("ros2")
        .arg("pkg")
        .arg("prefix")
        .arg("--share")
        .arg(pkg)
        .output()?;
    // TODO: in case pkg is not found
    let pkg_share_dir = String::from_utf8(cmd.stdout)?;
    return Ok(pkg_share_dir.trim().to_string());
}
