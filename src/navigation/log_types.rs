use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type")]
pub enum LogMessage {
    Start { session_name: String },
    Data { timestamp: u64, payload: LogPayload },
    Stop,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum LogPayload {
    RobotPose {
        x: f32,
        y: f32,
        theta: f32,
    },
    Odometry {
        x: f32,
        y: f32,
        theta: f32,
    },
    // 将来的に追加:
    // Scan { points: Vec<(f32, f32)> },
    // Command { v: f32, w: f32 },
}