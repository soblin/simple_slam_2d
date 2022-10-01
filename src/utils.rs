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

pub fn get_stamp() -> builtin_interfaces::msg::Time {
    let cur_time = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .unwrap();
    builtin_interfaces::msg::Time {
        sec: cur_time.as_secs() as i32,
        nanosec: cur_time.subsec_nanos() as u32,
    }
}

pub fn elapsed_ms(stamp: &builtin_interfaces::msg::Time) -> i64 {
    let cur_time = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .unwrap();
    let sec = cur_time.as_secs() as i64 - stamp.sec as i64;
    let nanosec = (cur_time.subsec_nanos() as i64) - (stamp.nanosec as i64);
    return sec * 1000 + (nanosec / 1000000) as i64;
}
