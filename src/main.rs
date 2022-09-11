use anyhow::{Error, Result};
use std::env;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "minimal_subscriber")?;

    let _subscription_scan = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        "/burger1_scan",
        rclrs::QOS_PROFILE_SENSOR_DATA,
        move |msg: sensor_msgs::msg::LaserScan| {
            println!("scan points: #{}", msg.ranges.len());
        },
    )?;

    let _subscription_odom = node.create_subscription::<nav_msgs::msg::Odometry, _>(
        "/burger1_odom",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: nav_msgs::msg::Odometry| {
            let position = msg.pose.pose.position;
            println!("Odometry is x = {}, y = {}", position.x, position.y);
        },
    )?;

    let pub_map = node.create_publisher::<nav_msgs::msg::OccupancyGrid>(
        "/burger1/map",
        rclrs::QOS_PROFILE_SENSOR_DATA,
    )?;

    while context.ok() {
        let mut occ_grid_msg = nav_msgs::msg::OccupancyGrid::default();
        occ_grid_msg.header.frame_id = String::from("burger1");
        let cur_time = std::time::SystemTime::now();
        let cur_time = cur_time.duration_since(std::time::SystemTime::UNIX_EPOCH)?;
        occ_grid_msg.header.stamp = builtin_interfaces::msg::Time {
            sec: cur_time.as_secs() as i32,
            nanosec: cur_time.subsec_nanos() as u32,
        };
        println!("{:?}", occ_grid_msg.header.stamp);
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    rclrs::spin(&node).map_err(|err| err.into())
}
