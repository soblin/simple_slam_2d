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
    pose_integrator: Arc<Mutex<simple_slam_2d::geometry::Pose2DIntegrator>>,
    params: simple_slam_2d::param::SimpleSlam2DParams,
}

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
        // TODO: initial pose
        // SimpleSlam2D
        let slam = Arc::new(Mutex::new(simple_slam_2d::slam::OdometryMapping {
            scan: sensor_msgs::msg::LaserScan::default(),
            twist: geometry_msgs::msg::Twist::default(),
            points: vec![],
            channels: vec![],
        }));
        // pose integral
        let pose_integrator = Arc::new(Mutex::new(
            simple_slam_2d::geometry::Pose2DIntegrator::new(0.0, 0.0, 0.0),
        ));
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
        let pose_integrator_sync = Arc::clone(&pose_integrator);
        let twist_sub = {
            node.create_subscription(
                &params.input_cmd,
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: geometry_msgs::msg::Twist| {
                    let mut pose_integrator = pose_integrator_sync.lock().unwrap();
                    pose_integrator.update_velocity(msg.linear.x as f32, msg.angular.z as f32);
                    let mut slam = slam_sync_twist.lock().unwrap();
                    slam.set_twist(msg);
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
            pose_integrator,
            params,
        })
    }
    fn update_pose(&self) -> Result<(), rclrs::RclrsError> {
        let mut pose_integrator = self.pose_integrator.lock().unwrap();
        // update odometry
        pose_integrator.update_pose();
        Ok(())
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
        // get current pose
        let current_pose = {
            let mut pose_integrator = self.pose_integrator.lock().unwrap();
            pose_integrator.pose.clone()
        };
        // do SLAM
        let mut slam = self.slam.lock().unwrap();
        slam.odom_mapping(&current_pose);
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
        odom_msg.pose.pose = current_pose.to_pose();
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
    // mapping timer
    let mapping_thread = Arc::clone(&simple_slam_2d_node);
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            std::thread::sleep(std::time::Duration::from_millis(500));
            mapping_thread.publish()?;
        }
    });
    // position integral timer
    let pose_integral_thread = Arc::clone(&simple_slam_2d_node);
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            std::thread::sleep(std::time::Duration::from_millis(100));
            pose_integral_thread.update_pose()?;
        }
    });
    rclrs::spin(&simple_slam_2d_node.node).map_err(|err| err.into())
}
