use anyhow::{Error, Result};
use std::env;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "minimal_subscriber")?;

    let _subscription_scan = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        "/burger1_scan",
        rclrs::QOS_PROFILE_SENSOR_DATA,
        move |msg: sensor_msgs::msg::LaserScan| {
            println!("LaserScan info:\nangle_min = {}\nangle_max = {}\nscan_time = {}\nrange_min = {}\nrange_max = {}\nnumber of point is {}", msg.angle_min, msg.angle_max, msg.scan_time, msg.range_min, msg.range_max, msg.ranges.len());
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
    rclrs::spin(&node).map_err(|err| err.into())
}
