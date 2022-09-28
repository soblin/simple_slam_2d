use std::path::Path;
use std::sync::{Arc, Mutex};
use std::{error::Error, result::Result};

pub struct SimpleSlam2DNode {
    node: rclrs::Node,
    scan_sub: Arc<rclrs::Subscription<sensor_msgs::msg::LaserScan>>,
    twist_sub: Arc<rclrs::Subscription<geometry_msgs::msg::Twist>>,
    map_pub: rclrs::Publisher<sensor_msgs::msg::PointCloud>,
    odom_pub: rclrs::Publisher<nav_msgs::msg::Odometry>,
    slam: Arc<Mutex<simple_slam_2d::slam::OdometryMapping>>,
    params: simple_slam_2d::param::SimpleSlam2DParams,
}

//
impl SimpleSlam2DNode {
    fn new(context: &rclrs::Context, param_path: &str) -> Result<Self, Box<dyn Error>> {
        // load param yaml
        let docs = simple_slam_2d::utils::load_config(param_path)?;
        let doc = &docs[0];
        let params = simple_slam_2d::param::SimpleSlam2DParams {
            input_scan: doc["input_scan"].as_str().unwrap().to_string(),
            input_cmd: doc["input_cmd"].as_str().unwrap().to_string(),
            output_map: doc["output_map"].as_str().unwrap().to_string(),
            output_odom: doc["output_odom"].as_str().unwrap().to_string(),
            map_frame_id: doc["map_frame_id"].as_str().unwrap().to_string(),
        };
        // init node
        let mut node = rclrs::Node::new(context, "simple_slam_2d_node")?;
        // SimpleSlam2D
        let slam = Arc::new(Mutex::new(simple_slam_2d::slam::OdometryMapping {
            scan: sensor_msgs::msg::LaserScan::default(),
            twist: geometry_msgs::msg::Twist::default(),
            points: vec![],
            channels: vec![],
            pose: simple_slam_2d::geometry::Pose2D::default(),
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
    let pkg_share_directory = simple_slam_2d::utils::get_package_share_directory("simple_slam_2d")?;
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
