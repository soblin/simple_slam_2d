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
        let (r, g, b) = (233, 163, 38);
        let color: u32 = r << 16 | g << 8 | b;
        let color_float: *const f32 = &color as *const u32 as *const f32;
        let color_float = unsafe { *color_float };
        // push new points
        for (radius, angle) in self.scan.iter() {
            let p = geometry::polar2cartesian(pose, *radius, *angle);
            self.points.push(geometry_msgs::msg::Point32 {
                x: p.x,
                y: p.y,
                z: 0.0,
            });
            self.channels.push(color_float);
        }
    }
}

pub struct ICPMapping {
    scan: Vec<(f32, f32)>, // (radius, angle)
    ref_map: Vec<geometry_msgs::msg::Point32>,
    pub poses: Vec<geometry::Pose2D>, // poses.last() is the latest estimation
    pub points: Vec<geometry_msgs::msg::Point32>,
    pub channels: Vec<f32>,
}

impl ICPMapping {
    pub fn new(init_pose: &geometry::Pose2D) -> Self {
        ICPMapping {
            scan: vec![],
            ref_map: vec![],
            poses: vec![],
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
        let (r, g, b) = (233, 163, 38);
        let color: u32 = r << 16 | g << 8 | b;
        let color_float: *const f32 = &color as *const u32 as *const f32;
        let color_float = unsafe { *color_float };
        if self.ref_map.len() == 0 {
            // just push new points
            for (radius, angle) in self.scan.iter() {
                let p = geometry::polar2cartesian(pose, *radius, *angle);
                let p = geometry_msgs::msg::Point32 {
                    x: p.x,
                    y: p.y,
                    z: 0.0,
                };
                self.ref_map.push(p.clone());
                self.points.push(p.clone());
                self.channels.push(color_float);
            }
            self.poses.push(pose.clone());
            return;
        }
        // associate data
        let last_pose = self.poses.last().unwrap();
        // corresponding indices of ref_map
        let mut scan_pairs = vec![];
        for (radius, angle) in self.scan.iter() {
            let pose = geometry::polar2cartesian(last_pose, *radius, *angle);
            let (mut dmax, mut ind) = (std::f32::INFINITY, 0);
            for (j, point) in self.ref_map.iter().enumerate() {
                let dist = ((pose.x - point.x).powi(2) + (pose.y - point.y).powi(2))
                    .sqrt()
                    .abs();
                if dist < dmax {
                    dmax = dist;
                    ind = j;
                }
            }
            scan_pairs.push(ind)
        }
        // iteratively update last_pos to new pose
        let ll = 0.001;
        let f_thre = 0.01;
        let (dd, dth) = (0.1, 0.01);
        let max_iter = 30;
        let mut est_pose = last_pose.clone();
        let mut opt_pose = last_pose.clone();
        let mut fprev = self.icp_score(&scan_pairs, est_pose.x, est_pose.y, est_pose.th);
        let mut fmin = std::f32::INFINITY;
        for i in 1..=max_iter {
            let f = fprev;
            let fx = self.icp_score(&scan_pairs, est_pose.x + dd, est_pose.y, est_pose.th);
            let fy = self.icp_score(&scan_pairs, est_pose.x, est_pose.y + dd, est_pose.th);
            let fth = self.icp_score(&scan_pairs, est_pose.x, est_pose.y, est_pose.th + dth);
            let (dx, dy, dth) = ((fx - f) / dd, (fy - f) / dd, (fth - f) / dth);
            est_pose.x -= dx * ll;
            est_pose.y -= dy * ll;
            est_pose.th -= dth * ll;
            fprev = self.icp_score(&scan_pairs, est_pose.x, est_pose.y, est_pose.th);
            if fprev < fmin {
                fmin = fprev;
                opt_pose = est_pose.clone();
            }
            if (f - fprev).abs() < f_thre {
                break;
            }
        }
        self.poses.push(opt_pose.clone());
        for (range, angle) in self.scan.iter() {
            let p = geometry::polar2cartesian(&opt_pose, *range, *angle);
            self.points.push(geometry_msgs::msg::Point32 {
                x: p.x,
                y: p.y,
                z: 0.0,
            });
            self.channels.push(color_float);
        }
    }
    fn icp_score(&self, scan_pairs: &Vec<usize>, x: f32, y: f32, th: f32) -> f32 {
        let mut score = 0.0;
        for (i, j) in scan_pairs.iter().enumerate() {
            let (radius, angle): &(f32, f32) = &self.scan[i];
            let pt_x = x + radius * angle.cos();
            let pt_y = y + radius * angle.sin();
            let dist = (pt_x - (self.ref_map[*j].x)).powi(2) + (pt_y - self.ref_map[*j].y).powi(2);
            score += dist;
        }
        return score;
    }
}
