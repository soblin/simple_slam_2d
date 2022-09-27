use std::path::Path;
use std::sync::{Arc, Mutex};
use std::{error::Error, result::Result};

pub struct SimpleSlam2DParams {
    pub input_scan: String,
    pub input_odom: String,
    pub output_map: String,
    pub map_frame_id: String,
}

pub struct SimpleSlam2DNode {
    node: rclrs::Node,
    scan_sub: Arc<rclrs::Subscription<sensor_msgs::msg::LaserScan>>,
    odom_sub: Arc<rclrs::Subscription<nav_msgs::msg::Odometry>>,
    map_pub: rclrs::Publisher<sensor_msgs::msg::PointCloud>,
    scan_data: Arc<Mutex<Option<sensor_msgs::msg::LaserScan>>>,
    odom_data: Arc<Mutex<Option<nav_msgs::msg::Odometry>>>,
    map_msg_data: Arc<Mutex<sensor_msgs::msg::PointCloud>>,
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
            output_map: doc["output_map"].as_str().unwrap().to_string(),
            map_frame_id: doc["map_frame_id"].as_str().unwrap().to_string(),
        };
        // init node
        let mut node = rclrs::Node::new(context, "simple_slam_2d_node")?;
        // scan sub
        let scan_data = Arc::new(Mutex::new(None));
        let scan_data_cb = Arc::clone(&scan_data);
        let scan_sub = {
            node.create_subscription(
                &params.input_scan,
                rclrs::QOS_PROFILE_SENSOR_DATA,
                move |msg: sensor_msgs::msg::LaserScan| {
                    *scan_data_cb.lock().unwrap() = Some(msg);
                },
            )?
        };
        // odom sub
        let odom_data = Arc::new(Mutex::new(None));
        let odom_data_cb = Arc::clone(&odom_data);
        let odom_sub = {
            node.create_subscription(
                &params.input_odom,
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: nav_msgs::msg::Odometry| {
                    *odom_data_cb.lock().unwrap() = Some(msg);
                },
            )?
        };
        // map_points
        let map_msg_data = Arc::new(Mutex::new(sensor_msgs::msg::PointCloud {
            header: std_msgs::msg::Header::default(),
            points: Vec::new(),
            channels: vec![sensor_msgs::msg::ChannelFloat32 {
                name: String::from("rgb"),
                values: Vec::new(),
            }],
        }));
        // map pub
        let map_pub = node.create_publisher(&params.output_map, rclrs::QOS_PROFILE_SENSOR_DATA)?;
        Ok(Self {
            node,
            scan_sub,
            odom_sub,
            map_pub,
            scan_data,
            odom_data,
            map_msg_data,
            params,
        })
    }
    fn publish(&self) -> Result<(), rclrs::RclrsError> {
        if let (Some(odom_msg), Some(scan_msg)) = (
            &*self.odom_data.lock().unwrap(),
            &*self.scan_data.lock().unwrap(),
        ) {
            // process self.map_msg_data
            self.odom_slam(&odom_msg.pose.pose, &scan_msg);
            // update self.map_msg_data
            let mut map_msg = self.map_msg_data.lock().unwrap();
            let cur_time = std::time::SystemTime::now()
                .duration_since(std::time::SystemTime::UNIX_EPOCH)
                .unwrap();
            map_msg.header.frame_id = String::from(&self.params.map_frame_id);
            map_msg.header.stamp = builtin_interfaces::msg::Time {
                sec: cur_time.as_secs() as i32,
                nanosec: cur_time.subsec_nanos() as u32,
            };
            println!("points size is {} as of now.", map_msg.points.len());
            self.map_pub.publish(map_msg.clone())?;
        }
        Ok(())
    }
    fn odom_slam(&self, pose: &geometry_msgs::msg::Pose, scan: &sensor_msgs::msg::LaserScan) {
        // place Laserscan along the odometry naively
        let position: &geometry_msgs::msg::Point = &pose.position;
        let quat: &geometry_msgs::msg::Quaternion = &pose.orientation;
        let ranges: &Vec<f32> = &scan.ranges;
        let yaw =
            simple_slam_2d::quat2rpy(&[quat.x as f32, quat.y as f32, quat.z as f32, quat.w as f32])
                [0];
        // these are in radian
        let angle_min = scan.angle_min;
        let angle_max = scan.angle_max;
        let angle_increment = scan.angle_increment;
        // push points in odom frame
        let mut new_points = Vec::new();
        let mut new_channel_values = Vec::new();
        for (i, range) in ranges.iter().enumerate() {
            if *range == std::f32::INFINITY {
                continue;
            }
            let theta: f32 = yaw + (angle_min + (i as f32) * angle_increment);
            let x_glob: f32 = (position.x as f32) + range * (theta.cos() as f32);
            let y_glob: f32 = (position.y as f32) + range * (theta.sin() as f32);
            new_points.push(geometry_msgs::msg::Point32 {
                x: x_glob,
                y: y_glob,
                z: 0.0,
            });
            let (r, g, b) = (200, 10, 10);
            let color: u32 = r << 16 | g << 8 | b;
            let color_float: *const f32 = &color as *const u32 as *const f32;
            unsafe {
                new_channel_values.push(*color_float);
            }
        }
        let mut map_msg = self.map_msg_data.lock().unwrap();
        map_msg.points.append(&mut new_points);
        map_msg.channels[0].values.append(&mut new_channel_values);
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
