use crate::geometry;

// TODO: trait for slam method
pub struct OdometryMapping {
    scan: sensor_msgs::msg::LaserScan,
    pub points: Vec<geometry_msgs::msg::Point32>,
    pub channels: Vec<f32>,
}

impl OdometryMapping {
    pub fn new() -> Self {
        OdometryMapping {
            scan: sensor_msgs::msg::LaserScan::default(),
            points: vec![],
            channels: vec![],
        }
    }
    pub fn set_scan(&mut self, scan: sensor_msgs::msg::LaserScan) {
        self.scan = scan;
    }
    pub fn odom_mapping(&mut self, pose: &geometry::Pose2D) {
        // these are in radian
        let angle_min = self.scan.angle_min;
        let angle_increment = self.scan.angle_increment;

        // push new points
        for (i, range) in self.scan.ranges.iter().enumerate() {
            if *range == std::f32::INFINITY {
                continue;
            }
            let th: f32 = pose.th + (angle_min + (i as f32) * angle_increment);
            let x_glob: f32 = pose.x + range * th.cos();
            let y_glob: f32 = pose.y + range * th.sin();
            self.points.push(geometry_msgs::msg::Point32 {
                x: x_glob,
                y: y_glob,
                z: 0.0,
            });
            let (r, g, b) = (233, 163, 38);
            let color: u32 = r << 16 | g << 8 | b;
            let color_float: *const f32 = &color as *const u32 as *const f32;
            unsafe {
                self.channels.push(*color_float);
            }
        }
    }
}

pub struct ICPMapping {
    scan: sensor_msgs::msg::LaserScan,
    ref_points: Vec<geometry_msgs::msg::Point32>,
    //    odoms: Vec<geometry::Pose2D>,
    pub points: Vec<geometry_msgs::msg::Point32>,
    pub channels: Vec<f32>,
}
