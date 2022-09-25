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
    map_pub: rclrs::Publisher<nav_msgs::msg::OccupancyGrid>,
    scan_data: Arc<Mutex<Option<sensor_msgs::msg::LaserScan>>>,
    odom_data: Arc<Mutex<Option<nav_msgs::msg::Odometry>>>,
    params: SimpleSlam2DParams,
}

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
                    // let position: &geometry_msgs::msg::Point = &msg.pose.pose.position;
                    *odom_data_cb.lock().unwrap() = Some(msg);
                },
            )?
        };
        // map pub
        let map_pub = node.create_publisher(&params.output_map, rclrs::QOS_PROFILE_SENSOR_DATA)?;
        Ok(Self {
            node,
            scan_sub,
            odom_sub,
            map_pub,
            scan_data,
            odom_data,
            params,
        })
    }
    fn publish(&self) -> Result<(), rclrs::RclrsError> {
        let mut map_msg = nav_msgs::msg::OccupancyGrid::default();
        let cur_time = std::time::SystemTime::now();
        let cur_time = cur_time
            .duration_since(std::time::SystemTime::UNIX_EPOCH)
            .unwrap();
        map_msg.header.stamp = builtin_interfaces::msg::Time {
            sec: cur_time.as_secs() as i32,
            nanosec: cur_time.subsec_nanos() as u32,
        };
        if let Some(odom_msg) = &*self.odom_data.lock().unwrap() {
            println!(
                "cur_time header is: {}-{}",
                map_msg.header.stamp.sec, map_msg.header.stamp.nanosec
            );
            map_msg.header.frame_id = String::from(&self.params.map_frame_id);
            map_msg.info.origin = odom_msg.pose.pose.clone();
            map_msg.info.resolution = 0.5;
            map_msg.info.width = 40;
            map_msg.info.height = 40;
            map_msg.data = vec![0; 1600];
            self.map_pub.publish(map_msg)?;
        }
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
    let simple_slam_2d_node = Arc::new(SimpleSlam2DNode::new(&ctx, &param_path)?);
    let simple_slam_2d_node_other_thread = Arc::clone(&simple_slam_2d_node);

    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(100));
            simple_slam_2d_node_other_thread.publish()?;
        }
    });
    rclrs::spin(&simple_slam_2d_node.node).map_err(|err| err.into())
}
