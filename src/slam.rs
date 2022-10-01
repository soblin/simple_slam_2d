use crate::geometry;

// TODO: trait for slam method
pub struct OdometryMapping {
    scan: sensor_msgs::msg::LaserScan,
    twist: geometry_msgs::msg::Twist,
    pub points: Vec<geometry_msgs::msg::Point32>,
    pub channels: Vec<f32>,
}

impl OdometryMapping {
    pub fn new() -> Self {
        OdometryMapping {
            scan: sensor_msgs::msg::LaserScan::default(),
            twist: geometry_msgs::msg::Twist::default(),
            points: vec![],
            channels: vec![],
        }
    }
    pub fn set_scan(&mut self, scan: sensor_msgs::msg::LaserScan) {
        self.scan = scan;
    }
    pub fn set_twist(&mut self, twist: geometry_msgs::msg::Twist) {
        self.twist = twist;
    }
    pub fn odom_mapping(&mut self, pose: &geometry::Pose2D) {
        let process_tm = std::time::Instant::now();

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
        // print processing time
        println!(
            "Processing time = {}[ms], #points = {}",
            process_tm.elapsed().as_micros() as f32 / 1000.0,
            self.points.len(),
        );
    }
}

pub struct ICPMapping {}
