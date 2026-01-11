use crate::config::MotorConfig;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};
#[derive(Debug, Clone, Copy)]
pub enum MotorCommand {
    /// Set the target velocity of the robot.
    /// (linear_velocity_m_per_s, angular_velocity_rad_per_s)
    SetVelocity(f32, f32),

    /// Set the target velocity for a specific duration.
    /// After the duration, a Stop command will be issued.
    /// (linear_velocity_m_per_s, angular_velocity_rad_per_s, duration_ms)
    SetVelocityTimed(f32, f32, u64),

    /// Stop the robot immediately.
    Stop,
}

// In the future, this could carry status information like current speed, errors, etc.
#[derive(Debug)]
pub enum MotorMessage {
    Status(String),
}

pub fn start_modbus_motor_thread(
    config: MotorConfig,
    command_receiver: mpsc::Receiver<MotorCommand>,
    message_sender: mpsc::Sender<MotorMessage>,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        println!("[Motor Thread] Thread started with config: {:?}", config);
        let mut timer_end: Option<Instant> = None;

        loop {
            // Check if a timed command has expired
            if let Some(end_time) = timer_end {
                if Instant::now() >= end_time {
                    println!("[Motor Thread] Timer expired. Issuing Stop.");
                    // In a real implementation, send a Modbus stop command here.
                    let _ = message_sender.send(MotorMessage::Status("Timed move stopped.".to_string()));
                    timer_end = None;
                }
            }

            // Check for new commands without blocking
            match command_receiver.try_recv() {
                Ok(command) => {
                    println!("[Motor Thread] Received command: {:?}", command);
                    match command {
                        MotorCommand::SetVelocity(v, w) => {
                            timer_end = None; // Manual velocity command cancels any active timer
                            println!("[Motor Thread] Setting velocity to ({}, {})", v, w);
                            // In a real implementation, convert (v, w) to motor RPMs
                            // and send the corresponding Modbus command.
                        }
                        MotorCommand::SetVelocityTimed(v, w, ms) => {
                            println!("[Motor Thread] Setting velocity to ({}, {}) for {} ms", v, w, ms);
                            // Set the timer to issue a stop command later
                            timer_end = Some(Instant::now() + Duration::from_millis(ms));
                        }
                        MotorCommand::Stop => {
                            timer_end = None; // Stop command cancels any active timer
                            println!("[Motor Thread] Setting velocity to (0, 0)");
                        }
                    }
                     let _ = message_sender.send(MotorMessage::Status(format!("Command Executed: {:?}", command)));
                }
                Err(mpsc::TryRecvError::Empty) => {
                    // No command, do nothing
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    println!("[Motor Thread] Channel disconnected. Terminating.");
                    break;
                }
            }

            thread::sleep(Duration::from_millis(10)); // Polling interval
        }
    })
}
