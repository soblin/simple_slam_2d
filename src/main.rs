use std::path::Path;
use std::sync::{Arc, Mutex};
use std::{error::Error, result::Result};

pub struct SimpleSlam2DParams {
    pub input_scan: String,
    pub input_odom: String,
    pub input_cmd: String,
    pub output_map: String,
    pub output_odom: String,
    pub map_frame_id: String,
}

#[derive(Default, Clone)]
pub struct Pose2D {
    pub x: f32,
    pub y: f32,
    pub th: f32,
}

impl Pose2D {
    pub fn to_pose(&self) -> geometry_msgs::msg::Pose {
        let quat = simple_slam_2d::rpy2quat(&[0.0, 0.0, self.th]);
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

pub struct SimpleSlam2D {
    pub scan: sensor_msgs::msg::LaserScan,
    pub twist: geometry_msgs::msg::Twist,
    pub points: Vec<geometry_msgs::msg::Point32>,
    pub channels: Vec<f32>,
    pub pose: Pose2D,
    pub twist_stamp: std::time::Instant,
}

impl SimpleSlam2D {
    fn set_scan(&mut self, scan: sensor_msgs::msg::LaserScan) {
        self.scan = scan;
    }
    fn set_twist(&mut self, twist: geometry_msgs::msg::Twist) {
        self.twist = twist;
        self.twist_stamp = std::time::Instant::now();
    }
    fn update_pose(&mut self, dt: f32) {
        self.pose
            .update(self.twist.linear.x as f32, self.twist.angular.z as f32, dt);
    }
    fn odom_mapping(&mut self) {
        let process_tm = std::time::Instant::now();

        // these are in radian
        let angle_min = self.scan.angle_min;
        let angle_increment = self.scan.angle_increment;

        // push new points
        for (i, range) in self.scan.ranges.iter().enumerate() {
            if *range == std::f32::INFINITY {
                continue;
            }
            let th: f32 = self.pose.th + (angle_min + (i as f32) * angle_increment);
            let x_glob: f32 = self.pose.x + range * th.cos();
            let y_glob: f32 = self.pose.y + range * th.sin();
            self.points.push(geometry_msgs::msg::Point32 {
                x: x_glob,
                y: y_glob,
                z: 0.0,
            });
            // TODO: change color based on timestamp ?
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

pub struct SimpleSlam2DNode {
    node: rclrs::Node,
    scan_sub: Arc<rclrs::Subscription<sensor_msgs::msg::LaserScan>>,
    twist_sub: Arc<rclrs::Subscription<geometry_msgs::msg::Twist>>,
    map_pub: rclrs::Publisher<sensor_msgs::msg::PointCloud>,
    odom_pub: rclrs::Publisher<nav_msgs::msg::Odometry>,
    slam: Arc<Mutex<SimpleSlam2D>>,
    params: SimpleSlam2DParams,
}

//
impl SimpleSlam2DNode {
    fn new(context: &rclrs::Context, param_path: &str) -> Result<Self, Box<dyn Error>> {
        // load param yaml
        let docs = simple_slam_2d::load_config(param_path)?;
        let doc = &docs[0];
        let params = SimpleSlam2DParams {
            input_scan: doc["input_scan"].as_str().unwrap().to_string(),
            input_odom: doc["input_odom"].as_str().unwrap().to_string(),
            input_cmd: doc["input_cmd"].as_str().unwrap().to_string(),
            output_map: doc["output_map"].as_str().unwrap().to_string(),
            output_odom: doc["output_odom"].as_str().unwrap().to_string(),
            map_frame_id: doc["map_frame_id"].as_str().unwrap().to_string(),
        };
        // init node
        let mut node = rclrs::Node::new(context, "simple_slam_2d_node")?;
        // SimpleSlam2D
        let slam = Arc::new(Mutex::new(SimpleSlam2D {
            scan: sensor_msgs::msg::LaserScan::default(),
            twist: geometry_msgs::msg::Twist::default(),
            points: vec![],
            channels: vec![],
            pose: Pose2D::default(),
            twist_stamp: std::time::Instant::now(),
        }));
        // scan sub
        let slam_sync_scan = Arc::clone(&slam);
        let scan_sub = {
            node.create_subscription(
                &params.input_scan,
                rclrs::QOS_PROFILE_SENSOR_DATA,
                move |msg: sensor_msgs::msg::LaserScan| {
                    let mut slam = slam_sync_scan.lock().unwrap();
                    slam.set_scan(msg);
                },
            )?
        };
        // twist sub
        let slam_sync_twist = Arc::clone(&slam);
        let twist_sub = {
            node.create_subscription(
                &params.input_cmd,
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: geometry_msgs::msg::Twist| {
                    // TODO: store nav_msgs::msg::Odometry in SimpleSlam2D and calcualte elapsed from ros::Time
                    let mut slam = slam_sync_twist.lock().unwrap();
                    let dt: f32 = slam.twist_stamp.elapsed().as_millis() as f32 / 1000.0;
                    slam.set_twist(msg);
                    // update odometry
                    slam.update_pose(dt)
                },
            )?
        };
        // map pub
        let map_pub = node.create_publisher(&params.output_map, rclrs::QOS_PROFILE_SENSOR_DATA)?;
        // odom_pub
        let odom_pub =
            node.create_publisher(&params.output_odom, rclrs::QOS_PROFILE_SENSOR_DATA)?;
        Ok(Self {
            node,
            scan_sub,
            twist_sub,
            map_pub,
            odom_pub,
            slam,
            params,
        })
    }
    fn publish(&self) -> Result<(), rclrs::RclrsError> {
        // prepare pointcloud msg
        let cur_time = std::time::SystemTime::now()
            .duration_since(std::time::SystemTime::UNIX_EPOCH)
            .unwrap();
        let cur_time = builtin_interfaces::msg::Time {
            sec: cur_time.as_secs() as i32,
            nanosec: cur_time.subsec_nanos() as u32,
        };
        let mut map_msg = sensor_msgs::msg::PointCloud {
            header: std_msgs::msg::Header {
                frame_id: String::from(&self.params.map_frame_id),
                stamp: cur_time.clone(),
            },
            points: vec![],
            channels: vec![sensor_msgs::msg::ChannelFloat32 {
                name: String::from("rgb"),
                values: vec![],
            }],
        };
        // do SLAM
        let mut slam = self.slam.lock().unwrap();
        slam.odom_mapping();
        // publish map
        map_msg.points = slam.points.clone();
        map_msg.channels[0].values = slam.channels.clone();
        self.map_pub.publish(map_msg)?;
        // publish odom
        let mut odom_msg = nav_msgs::msg::Odometry::default();
        odom_msg.header = std_msgs::msg::Header {
            frame_id: String::from(&self.params.map_frame_id),
            stamp: cur_time.clone(),
        };
        odom_msg.pose.pose = slam.pose.to_pose();
        self.odom_pub.publish(odom_msg)?;
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let ctx = rclrs::Context::new(std::env::args())?;

    // path to /config/param.yaml
    let pkg_share_directory = simple_slam_2d::get_package_share_directory("simple_slam_2d")?;
    let param_path = Path::new(&pkg_share_directory)
        .join("config")
        .join("param.yaml");
    let param_path = param_path.into_os_string().into_string().unwrap();

    // init node
    let simple_slam_2d_node = Arc::new(SimpleSlam2DNode::new(&ctx, &param_path)?);
    let simple_slam_2d_node_other_thread = Arc::clone(&simple_slam_2d_node);

    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(500));
            simple_slam_2d_node_other_thread.publish()?;
        }
    });
    rclrs::spin(&simple_slam_2d_node.node).map_err(|err| err.into())
}
