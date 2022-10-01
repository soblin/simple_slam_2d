use crate::geometry;

// TODO: trait for slam method
pub struct OdometryMapping {
    scan: Vec<(f32, f32)>, // (radius, angle)
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
        let angle_min = scan.angle_min;
        let angle_increment = scan.angle_increment;
        for (i, range) in scan.ranges.iter().enumerate() {
            if *range == std::f32::INFINITY {
                continue;
            }
            let angle = angle_min + (i as f32) * angle_increment;
            self.scan.push((*range, angle));
        }
    }
    pub fn do_slam(&mut self, pose: &geometry::Pose2D) {
        // push new points
        for (range, angle) in self.scan.iter() {
            let th: f32 = pose.th + (*angle);
            let x_glob: f32 = pose.x + (*range) * th.cos();
            let y_glob: f32 = pose.y + (*range) * th.sin();
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
    ref_map: Vec<geometry_msgs::msg::Point32>,
    poses: Vec<geometry::Pose2D>,
    pub points: Vec<geometry_msgs::msg::Point32>,
    pub channels: Vec<f32>,
}

impl ICPMapping {
    pub fn new(init_pose: &geometry::Pose2D) -> Self {
        ICPMapping {
            scan: sensor_msgs::msg::LaserScan::default(),
            ref_map: vec![],
            poses: vec![],
            points: vec![],
            channels: vec![],
        }
    }
    pub fn set_scan(&mut self, scan: sensor_msgs::msg::LaserScan) {
        self.scan = scan;
    }
    pub fn do_slam(&mut self, pose: &geometry::Pose2D) {
        // these are in radian
        let angle_min = self.scan.angle_min;
        let angle_increment = self.scan.angle_increment;

        if self.ref_map.len() == 0 {
            // just push new points
            for (i, range) in self.scan.ranges.iter().enumerate() {
                if *range == std::f32::INFINITY {
                    continue;
                }
                let th: f32 = pose.th + (angle_min + (i as f32) * angle_increment);
                let p = geometry_msgs::msg::Point32 {
                    x: pose.x + range * th.cos(),
                    y: pose.y + range * th.sin(),
                    z: 0.0,
                };
                self.ref_map.push(p.clone());
                self.points.push(p.clone());
                let (r, g, b) = (233, 163, 38);
                let color: u32 = r << 16 | g << 8 | b;
                let color_float: *const f32 = &color as *const u32 as *const f32;
                unsafe {
                    self.channels.push(*color_float);
                }
            }
            self.poses.push(pose.clone());
            return;
        }
        // associate data
        let last_pose = self.poses.last().unwrap();
        // a list of (sensor_range index, ref_map_point index)
        // TODO:change this [], []
        let mut scan_pairs = vec![];
        for (i, range) in self.scan.ranges.iter().enumerate() {
            if *range == std::f32::INFINITY {
                continue;
            }
            let th: f32 = pose.th + (angle_min + (i as f32) * angle_increment);
            let x: f32 = last_pose.x + range * th.cos();
            let y: f32 = last_pose.y + range * th.sin();
            let (mut dmax, mut ind) = (std::f32::INFINITY, 0);
            for (j, point) in self.ref_map.iter().enumerate() {
                let dist = ((x - point.x).powi(2) + (y - point.y).powi(2)).sqrt().abs();
                if dist < dmax {
                    dmax = dist;
                    ind = j;
                }
            }
            scan_pairs.push((i, ind))
        }
        // iteratively update last_pos to new pose
        let ll = 0.001;
        let (dd, dth) = (0.1, 0.01);
        let max_iter = 30;
        let mut est_pose = last_pose.clone();
        let mut opt_pose = last_pose.clone();
        let mut fprev = self.icp_score(&scan_pairs, est_pose.x, est_pose.y, est_pose.th);
        let mut fmin = std::f32::INFINITY;
        for i in 1..=max_iter {
            let fx = self.icp_score(&scan_pairs, est_pose.x + dd, est_pose.y, est_pose.th);
            let fy = self.icp_score(&scan_pairs, est_pose.x, est_pose.y + dd, est_pose.th);
            let fth = self.icp_score(&scan_pairs, est_pose.x, est_pose.y, est_pose.th + dth);
            let (dx, dy, dth) = ((fx - fprev) / dd, (fy - fprev) / dd, (fth - fprev) / dth);
            est_pose.x -= dx * ll;
            est_pose.y -= dy * ll;
            est_pose.th -= dth * ll;
            fprev = self.icp_score(&scan_pairs, est_pose.x, est_pose.y, est_pose.th);
            if fprev < fmin {
                fmin = fprev;
                opt_pose = est_pose.clone();
            }
        }
        self.poses.push(opt_pose.clone());
    }
    fn icp_score(&self, scan_pairs: &Vec<(usize, usize)>, x: f32, y: f32, th: f32) -> f32 {
        // these are in radian
        let angle_min = self.scan.angle_min;
        let angle_increment = self.scan.angle_increment;
        let mut score = 0.0;
        for (i, j) in scan_pairs.iter() {
            let range = self.scan.ranges[*i];
            let angle = th + (angle_min + (*i as f32) * angle_increment);
            let pt_x = x + range * angle.cos();
            let pt_y = y + range * angle.sin();
            let dist = (pt_x - (self.ref_map[*j].x)).powi(2) + (pt_y - self.ref_map[*j].y).powi(2);
            score += dist;
        }
        return score;
    }
}
