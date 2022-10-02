use crate::geometry;

// TODO: trait for slam method
pub struct OdometryMapping {
    scan: Vec<(f64, f64)>, // (radius, angle)
    pub points: Vec<geometry_msgs::msg::Point32>,
    pub channels: Vec<f32>,
}

impl OdometryMapping {
    pub fn new() -> Self {
        OdometryMapping {
            scan: vec![],
            points: vec![],
            channels: vec![],
        }
    }
    pub fn set_scan(&mut self, scan: sensor_msgs::msg::LaserScan) {
        // these are in radian
        self.scan.clear(); // !important!
        let angle_min = scan.angle_min as f64;
        let angle_increment = scan.angle_increment as f64;
        for (i, range) in scan.ranges.iter().enumerate() {
            if *range == std::f32::INFINITY {
                continue;
            }
            let angle = angle_min + (i as f64) * angle_increment;
            self.scan.push((*range as f64, angle));
        }
    }
    pub fn do_slam(&mut self, pose: &geometry::Pose2D) {
        let (r, g, b) = (233, 163, 38);
        let color: u32 = r << 16 | g << 8 | b;
        let color_float: *const f32 = &color as *const u32 as *const f32;
        let color_float = unsafe { *color_float };
        // push new points
        for (radius, angle) in self.scan.iter() {
            let p = geometry::polar2cartesian(pose, *radius, *angle);
            self.points.push(geometry_msgs::msg::Point32 {
                x: p.x as f32,
                y: p.y as f32,
                z: 0.0,
            });
            self.channels.push(color_float);
        }
    }
}

pub struct ICPMapping {
    scan: Vec<(f64, f64)>, // (radius, angle)
    ref_map: Vec<geometry_msgs::msg::Point32>,
    pub points: Vec<geometry_msgs::msg::Point32>,
    pub channels: Vec<f32>,
}

impl ICPMapping {
    pub fn new() -> Self {
        ICPMapping {
            scan: vec![],
            ref_map: vec![],
            points: vec![],
            channels: vec![],
        }
    }
    pub fn set_scan(&mut self, scan: sensor_msgs::msg::LaserScan) {
        // these are in radian
        self.scan.clear(); // !important!
        let angle_min = scan.angle_min as f64;
        let angle_increment = scan.angle_increment as f64;
        for (i, range) in scan.ranges.iter().enumerate() {
            if *range == std::f32::INFINITY {
                continue;
            }
            let angle = angle_min + (i as f64) * angle_increment;
            self.scan.push((*range as f64, angle));
        }
    }
    pub fn do_slam(&mut self, pose: &geometry::Pose2D) -> geometry::Pose2D {
        let (r, g, b) = (233, 163, 38);
        let color: u32 = r << 16 | g << 8 | b;
        let color_float: *const f32 = &color as *const u32 as *const f32;
        let color_float = unsafe { *color_float };
        if self.ref_map.len() == 0 {
            // just push new points
            for (radius, angle) in self.scan.iter() {
                let p = geometry::polar2cartesian(pose, *radius, *angle);
                let p = geometry_msgs::msg::Point32 {
                    x: p.x as f32,
                    y: p.y as f32,
                    z: 0.0,
                };
                self.ref_map.push(p.clone());
                self.points.push(p.clone());
                self.channels.push(color_float);
            }
            return pose.clone();
        }
        // associate data
        // corresponding indices of ref_map
        let mut scan_pairs = vec![];
        for (radius, angle) in self.scan.iter() {
            let pose = geometry::polar2cartesian(pose, *radius, *angle);
            let (mut dmax, mut ind) = (std::f64::INFINITY, 0);
            for (j, point) in self.ref_map.iter().enumerate() {
                let dist = (pose.x - point.x as f64).powi(2) + (pose.y - point.y as f64).powi(2);
                if dist < dmax {
                    dmax = dist;
                    ind = j;
                }
            }
            scan_pairs.push(ind)
        }
        // iteratively update last_pos to new pose
        let ll = 0.001;
        let f_thre = 0.00001;
        let (dd, dth) = (0.05, 0.01);
        let max_iter = 50;
        let mut est_pose = pose.clone();
        let mut fprev = self.icp_score(&scan_pairs, est_pose.x, est_pose.y, est_pose.th);
        for i in 1..=max_iter {
            let f = fprev;
            let fx = self.icp_score(&scan_pairs, est_pose.x + dd, est_pose.y, est_pose.th);
            let fy = self.icp_score(&scan_pairs, est_pose.x, est_pose.y + dd, est_pose.th);
            let fth = self.icp_score(&scan_pairs, est_pose.x, est_pose.y, est_pose.th + dth);
            let (dx, dy, dth) = ((fx - f) / dd, (fy - f) / dd, (fth - f) / dth);
            println!("dx = {}, dy = {}, dth = {}", dx, dy, dth);
            est_pose.x -= dx * ll;
            est_pose.y -= dy * ll;
            est_pose.th -= dth * ll;
            fprev = self.icp_score(&scan_pairs, est_pose.x, est_pose.y, est_pose.th);
            println!("{}-th iteration: score = {}", i, fprev);
            if (fprev - f) < 0.0 && (fprev - f).abs() < f_thre {
                break;
            }
        }
        self.ref_map.clear();
        for (range, angle) in self.scan.iter() {
            let p = geometry::polar2cartesian(&est_pose, *range, *angle);
            let p = geometry_msgs::msg::Point32 {
                x: p.x as f32,
                y: p.y as f32,
                z: 0.0,
            };
            self.ref_map.push(p.clone());
            self.points.push(p.clone());
            self.channels.push(color_float);
        }
        return est_pose;
    }
    fn icp_score(&self, scan_pairs: &Vec<usize>, x: f64, y: f64, th: f64) -> f64 {
        let mut score = 0.0;
        for i in 0..scan_pairs.len() {
            let j = scan_pairs[i];
            let (radius, angle): &(f64, f64) = &self.scan[i];
            let pt_th = angle + th;
            let pt_x = x + radius * pt_th.cos();
            let pt_y = y + radius * pt_th.sin();
            let dist = (pt_x - self.ref_map[j].x as f64).powi(2)
                + (pt_y - self.ref_map[j].y as f64).powi(2);
            score += dist;
        }
        return score;
    }
}
