use anyhow::{Error, Result};
use std::env;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "minimal_subscriber")?;
    let mut num_messages: usize = 0;

    let _subscription = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        "/burger1_scan",
        rclrs::QOS_PROFILE_SENSOR_DATA,
        move |msg: sensor_msgs::msg::LaserScan| {
            num_messages += 1;
            println!("I heard: {}", msg.range_min);
            println!("Got {} messages", num_messages);
        },
    )?;

    rclrs::spin(&node).map_err(|err| err.into())
}
