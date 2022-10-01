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
    odom_integrator: Arc<Mutex<simple_slam_2d::geometry::OdomIntegrator>>,
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
        let slam = Arc::new(Mutex::new(simple_slam_2d::slam::OdometryMapping::new()));
        // pose integral
        let odom_integrator = Arc::new(Mutex::new(simple_slam_2d::geometry::OdomIntegrator::new(
            0.0, 0.0, 0.0,
        )));
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
        let odom_integrator_sync = Arc::clone(&odom_integrator);
        let twist_sub = {
            node.create_subscription(
                &params.input_cmd,
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: geometry_msgs::msg::Twist| {
                    let mut odom_integrator = odom_integrator_sync.lock().unwrap();
                    odom_integrator.update_velocity(msg.linear.x as f32, msg.angular.z as f32);
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
            odom_integrator,
            params,
        })
    }
    fn update_pose(&self) -> Result<(), rclrs::RclrsError> {
        let mut odom_integrator = self.odom_integrator.lock().unwrap();
        // update odometry
        odom_integrator.update_pose();
        Ok(())
    }
    fn publish(&self) -> Result<(), rclrs::RclrsError> {
        let elapsed = std::time::SystemTime::now();
        // get current pose
        let (cur_odom, stopped) = {
            let mut odom_integrator = self.odom_integrator.lock().unwrap();
            (odom_integrator.odom.clone(), odom_integrator.stopped)
        };

        // do SLAM
        let mut slam = self.slam.lock().unwrap();
        if !stopped {
            slam.do_slam(&cur_odom);
        }

        // publish map
        let map_msg = sensor_msgs::msg::PointCloud {
            header: std_msgs::msg::Header {
                frame_id: String::from(&self.params.map_frame_id),
                stamp: simple_slam_2d::utils::get_stamp(),
            },
            points: slam.points.clone(),
            channels: vec![sensor_msgs::msg::ChannelFloat32 {
                name: String::from("rgb"),
                values: slam.channels.clone(),
            }],
        };
        self.map_pub.publish(map_msg)?;

        // publish odom
        let odom_msg = nav_msgs::msg::Odometry {
            header: std_msgs::msg::Header {
                frame_id: String::from(&self.params.map_frame_id),
                stamp: simple_slam_2d::utils::get_stamp(),
            },
            child_frame_id: String::from(""),
            pose: geometry_msgs::msg::PoseWithCovariance {
                pose: cur_odom.to_pose(),
                covariance: [0.0; 36],
            },
            twist: geometry_msgs::msg::TwistWithCovariance::default(),
        };
        self.odom_pub.publish(odom_msg)?;

        // stat
        println!(
            "Processing time: {}[ms], #points = {}",
            elapsed.elapsed().unwrap().as_millis(),
            slam.points.len()
        );
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
    let odom_integral_thread = Arc::clone(&simple_slam_2d_node);
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            std::thread::sleep(std::time::Duration::from_millis(100));
            odom_integral_thread.update_pose()?;
        }
    });
    rclrs::spin(&simple_slam_2d_node.node).map_err(|err| err.into())
}
