pub fn quat2rpy(quat: &[f32; 4]) -> [f32; 3] {
    let (q0, q1, q2, q3) = (quat[0], quat[1], quat[2], quat[3]);
    let r: f32 = (2.0 * (q0 * q1 + q2 * q3)).atan2(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    let p: f32 = (2.0 * (q0 * q2 - q1 * q3)).asin();
    let y: f32 = (2.0 * (q0 * q3 + q1 * q2)).atan2(q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    [r, p, y]
}

pub fn rpy2quat(rpy: &[f32; 3]) -> [f32; 4] {
    let cos_r: f32 = (rpy[0] / 2.0).cos();
    let sin_r: f32 = (rpy[0] / 2.0).sin();
    let cos_p: f32 = (rpy[1] / 2.0).cos();
    let sin_p: f32 = (rpy[1] / 2.0).sin();
    let cos_y: f32 = (rpy[2] / 2.0).cos();
    let sin_y: f32 = (rpy[2] / 2.0).sin();
    return [
        sin_r * cos_p * cos_y - cos_r * sin_p * sin_y, // x
        cos_r * sin_p * cos_y + sin_r * cos_p * sin_y, // y
        cos_r * cos_p * sin_y - sin_r * sin_p * cos_y, // z
        cos_r * cos_p * cos_y + sin_r * sin_p * sin_y, // w
    ];
}

#[derive(Default, Clone)]
pub struct Pose2D {
    pub x: f32,
    pub y: f32,
    pub th: f32,
}

impl Pose2D {
    pub fn to_pose(&self) -> geometry_msgs::msg::Pose {
        let quat = rpy2quat(&[0.0, 0.0, self.th]);
        return geometry_msgs::msg::Pose {
            position: geometry_msgs::msg::Point {
                x: self.x as f64,
                y: self.y as f64,
                z: 0.0,
            },
            orientation: geometry_msgs::msg::Quaternion {
                x: quat[0] as f64,
                y: quat[1] as f64,
                z: quat[2] as f64,
                w: quat[3] as f64,
            },
        };
    }
    pub fn update(&mut self, v: f32, omega: f32, dt: f32) {
        let (dx, dy, dth) = (v * self.th.cos(), v * self.th.sin(), omega);
        self.x += dx * dt;
        self.y += dy * dt;
        self.th += dth * dt;
    }
}
