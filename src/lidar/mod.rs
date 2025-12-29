// 1. Declare sub-modules within the `lidar` module.
// Since they are not `pub`, they are only visible within this module.
mod comm;
pub mod config;
mod driver;
mod protocol;

// 2. Re-export the public API from the sub-modules.
// This is the official public API of the `lidar` module.
pub use self::config::load_lidar_configurations;
pub use self::driver::{start_lidar_thread, LidarInfo};
