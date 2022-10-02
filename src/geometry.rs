use crate::utils;

pub fn quat2rpy(quat: &[f64; 4]) -> [f64; 3] {
    let (q0, q1, q2, q3) = (quat[0], quat[1], quat[2], quat[3]);
    let r: f64 = (2.0 * (q0 * q1 + q2 * q3)).atan2(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    let p: f64 = (2.0 * (q0 * q2 - q1 * q3)).asin();
    let y: f64 = (2.0 * (q0 * q3 + q1 * q2)).atan2(q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    [r, p, y]
}

pub fn rpy2quat(rpy: &[f64; 3]) -> [f64; 4] {
    let cos_r: f64 = (rpy[0] / 2.0).cos();
    let sin_r: f64 = (rpy[0] / 2.0).sin();
    let cos_p: f64 = (rpy[1] / 2.0).cos();
    let sin_p: f64 = (rpy[1] / 2.0).sin();
    let cos_y: f64 = (rpy[2] / 2.0).cos();
    let sin_y: f64 = (rpy[2] / 2.0).sin();
    return [
        sin_r * cos_p * cos_y - cos_r * sin_p * sin_y, // x
        cos_r * sin_p * cos_y + sin_r * cos_p * sin_y, // y
        cos_r * cos_p * sin_y - sin_r * sin_p * cos_y, // z
        cos_r * cos_p * cos_y + sin_r * sin_p * sin_y, // w
    ];
}

#[derive(Default, Clone)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub th: f64,
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
    pub fn integrate(&mut self, v: f64, omega: f64, dt: f64) {
        let (dx, dy, dth) = (v * self.th.cos(), v * self.th.sin(), omega);
        self.x += dx * dt;
        self.y += dy * dt;
        self.th += dth * dt;
    }
}

pub fn polar2cartesian(base: &Pose2D, radius: f64, angle: f64) -> Pose2D {
    let th = base.th + angle;
    return Pose2D {
        x: base.x + radius * th.cos(),
        y: base.y + radius * th.sin(),
        th: th,
    };
}

pub struct OdomIntegrator {
    pub odom: Pose2D,
    pub v: f64,
    pub omega: f64,
    pub stamp: Option<builtin_interfaces::msg::Time>,
    pub stopped: bool,
}

impl OdomIntegrator {
    pub fn new(x: f64, y: f64, th: f64) -> Self {
        OdomIntegrator {
            odom: Pose2D { x: x, y: y, th: th },
            v: 0.0,
            omega: 0.0,
            stamp: None,
            stopped: false,
        }
    }
    pub fn update_velocity(&mut self, v: f32, omega: f32) {
        self.v = v as f64;
        self.omega = omega as f64;
        if v.abs() < 1e-5 && omega.abs() < 1e-5 {
            self.stopped = true;
        } else {
            self.stopped = false;
        }
    }
    pub fn update_pose(&mut self, pose: &Pose2D) {
        self.odom = pose.clone();
        self.stamp = Some(utils::get_stamp());
    }
    pub fn integrate_pose(&mut self) {
        if let Some(stamp) = &self.stamp {
            if !self.stopped {
                let dt = utils::elapsed_ms(&stamp) as f64 / 1000.0;
                self.odom.integrate(self.v, self.omega, dt);
            }
        }
        self.stamp = Some(utils::get_stamp());
    }
}
