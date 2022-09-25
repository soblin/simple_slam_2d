use yaml_rust::{YamlEmitter, YamlLoader};

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

pub fn load_config(yaml_path: &str) -> Result<Vec<yaml_rust::Yaml>, Box<dyn std::error::Error>> {
    let f = std::fs::read_to_string(yaml_path)?;
    let s = f.to_string();
    let docs = YamlLoader::load_from_str(&s)?;
    Ok(docs)
}
