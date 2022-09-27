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

pub fn quat2rpy(quat: &[f32; 4]) -> [f32; 3] {
    let (q0, q1, q2, q3) = (quat[0], quat[1], quat[2], quat[3]);
    let r: f32 = (2.0 * (q0 * q1 + q2 * q3)).atan2(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    let p: f32 = (2.0 * (q0 * q2 - q1 * q3)).asin();
    let y: f32 = (2.0 * (q0 * q3 + q1 * q2)).atan2(q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    [r, p, y]
}
