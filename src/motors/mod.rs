use crate::config::MotorConfig;
use crc::{Crc, CRC_16_MODBUS};
use serialport::SerialPort;
use std::f32::consts::PI;
use std::io::Write;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy)]
pub enum MotorCommand {
    SetVelocity(f32, f32),
    SetVelocityTimed(f32, f32, u64),
    Stop,
    EnableIdShare,
    ServoOn,
    ServoOff,
    ServoFree,
}

#[derive(Debug)]
pub enum MotorMessage {
    Status(String),
}

const QUERY_NET_ID_WRITE_TEMPLATE: [u8; 57] = [
    0x0F, 0x10, 0x00, 0x00, 0x00, 0x18, 0x30,
    // Data for ID=1 (Right Motor)
    0x00, 0x00, 0x00, 0x10, // Drive Mode
    0x00, 0x00, 0x00, 0x00, // Position
    0x00, 0x00, 0x00, 0x00, // Speed (RPM)
    0x00, 0x00, 0x07, 0xD0, // Acceleration Rate
    0x00, 0x00, 0x03, 0xE8, // Deceleration Rate
    0x00, 0x00, 0x00, 0x01, // Trigger
    // Data for ID=2 (Left Motor)
    0x00, 0x00, 0x00, 0x10, // Drive Mode
    0x00, 0x00, 0x00, 0x00, // Position
    0x00, 0x00, 0x00, 0x00, // Speed (RPM)
    0x00, 0x00, 0x07, 0xD0, // Acceleration Rate
    0x00, 0x00, 0x03, 0xE8, // Deceleration Rate
    0x00, 0x00, 0x00, 0x01, // Trigger
    0x00, 0x00, // CRC placeholder
];

const QUERY_WRITE_SON_R: [u8; 13] = [0x01, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00];
const QUERY_WRITE_SON_L: [u8; 13] = [0x02, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00];
const QUERY_WRITE_SOFF_R: [u8; 13] = [0x01, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
const QUERY_WRITE_SOFF_L: [u8; 13] = [0x02, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
const QUERY_WRITE_FREE_R: [u8; 13] = [0x01, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00];
const QUERY_WRITE_FREE_L: [u8; 13] = [0x02, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00];

// --- Initialization Commands from coyomi2/query.h ---
const QUERY_IDSHARE_R: [u8; 21] = [ 0x01, 0x10, 0x09, 0x80, 0x00, 0x06, 0x0C, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, ];
const QUERY_IDSHARE_L: [u8; 21] = [ 0x02, 0x10, 0x09, 0x80, 0x00, 0x06, 0x0C, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, ];
const QUERY_READ_R: [u8; 33] = [ 0x01, 0x10, 0x09, 0x90, 0x00, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x9E, 0x00, 0x00, 0x00, 0xA4, 0x00, 0x00, ];
const QUERY_READ_L: [u8; 33] = [ 0x02, 0x10, 0x09, 0x90, 0x00, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x9E, 0x00, 0x00, 0x00, 0xA4, 0x00, 0x00, ];
const QUERY_WRITE_R: [u8; 33] = [ 0x01, 0x10, 0x09, 0xA8, 0x00, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x2D, 0x00, 0x00, 0x00, 0x2E, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x31, 0x00, 0x00, 0x00, 0x33, 0x00, 0x00, ];
const QUERY_WRITE_L: [u8; 33] = [ 0x02, 0x10, 0x09, 0xA8, 0x00, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x2D, 0x00, 0x00, 0x00, 0x2E, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x31, 0x00, 0x00, 0x00, 0x33, 0x00, 0x00, ];

fn calculate_crc16_modbus(data: &[u8]) -> [u8; 2] {
    let crc_driver = Crc::<u16>::new(&CRC_16_MODBUS);
    let crc_value = crc_driver.checksum(data);
    crc_value.to_le_bytes()
}

fn calc_vw_to_motor_rpms(v: f32, w: f32, config: &MotorConfig) -> (i32, i32) {
    let wheel_radius = config.wheel_diameter / 2.0;
    let wr = v / wheel_radius + w * config.tread_width / (2.0 * wheel_radius);
    let wl = v / wheel_radius - w * config.tread_width / (2.0 * wheel_radius);
    let motor_wr_rpm = (-wr / (2.0 * PI) * config.gear_ratio * 60.0) as i32;
    let motor_wl_rpm = (wl / (2.0 * PI) * config.gear_ratio * 60.0) as i32;
    (motor_wr_rpm, motor_wl_rpm)
}

fn send_command(port: &mut Box<dyn SerialPort>, command_with_crc_space: &mut [u8]) -> Result<(), String> {
    // CRC計算対象は、コマンドの長さ-2バイト
    let data_to_crc = &command_with_crc_space[..(command_with_crc_space.len() - 2)];
    let crc = calculate_crc16_modbus(data_to_crc);

    // 計算されたCRCをコマンドの末尾2バイトに直接書き込む
    let len = command_with_crc_space.len();
    command_with_crc_space[len - 2] = crc[0];
    command_with_crc_space[len - 1] = crc[1];

    let sending_hex_string: String = command_with_crc_space
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<String>>()
        .join(" ");
    //println!("[Motor Thread] Sending command: {}", sending_hex_string);
    if let Err(e) = port.write_all(command_with_crc_space) {
        Err(format!("Failed to write to port: {}", e))
    } else {
        // レスポンスを待つために少し待機
        thread::sleep(Duration::from_millis(15));

        let mut response_buf = [0; 8];
        match port.read_exact(&mut response_buf) {
            Ok(_) => {
                let hex_string: String = response_buf
                    .iter()
                    .map(|b| format!("{:02X}", b)) // 'h'を削除
                    .collect::<Vec<String>>()
                    .join(" ");
                //println!("[Motor Thread] Received response: {}", hex_string);
            }
            Err(e) => {
                // タイムアウトは警告としてログ出力し、エラーとはしない
                if e.kind() == std::io::ErrorKind::TimedOut {
                    println!("[Motor Thread] Warning: Timed out waiting for response.");
                } else {
                    // その他の読み取りエラー
                    eprintln!("[Motor Thread] Error reading response: {}", e);
                }
            }
        }
        Ok(())
    }
}

pub fn start_modbus_motor_thread(
    config: MotorConfig,
    command_receiver: mpsc::Receiver<MotorCommand>,
    message_sender: mpsc::Sender<MotorMessage>,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        println!("[Motor Thread] Thread started with config: {:?}", config);

        let mut port = match serialport::new(&config.port, config.baud_rate)
            .timeout(Duration::from_millis(100))
            .data_bits(serialport::DataBits::Eight)
            .stop_bits(serialport::StopBits::One)
            .parity(serialport::Parity::Even)
            .flow_control(serialport::FlowControl::None)
            .open()
        {
            Ok(p) => {
                println!(
                    "[Motor Thread] Successfully opened serial port: {}",
                    &config.port
                );
                message_sender
                    .send(MotorMessage::Status(format!(
                        "Serial port {} opened.",
                        &config.port
                    )))
                    .unwrap_or_default();
                p
            }
            Err(e) => {
                eprintln!(
                    "[Motor Thread] Failed to open serial port {}: {}",
                    &config.port, e
                );
                message_sender
                    .send(MotorMessage::Status(format!(
                        "Error: Failed to open serial port {}.",
                        &config.port
                    )))
                    .unwrap_or_default();
                return;
            }
        };

        let mut timer_end: Option<Instant> = None;

        loop {
            // Check for a timed command expiration
            if let Some(end_time) = timer_end {
                if Instant::now() >= end_time {
                    println!("[Motor Thread] Timer expired. Issuing Stop.");
                    let (motor_wr_rpm, motor_wl_rpm): (i32, i32) = (0, 0);
                    let mut stop_command = QUERY_NET_ID_WRITE_TEMPLATE.to_vec();
                    stop_command[15..19].copy_from_slice(&motor_wr_rpm.to_be_bytes());
                    stop_command[39..43].copy_from_slice(&motor_wl_rpm.to_be_bytes());

                    if let Err(msg) = send_command(&mut port, &mut stop_command) {
                        eprintln!("[Motor Thread] Error sending timed stop command: {}", msg);
                    }
                    message_sender
                        .send(MotorMessage::Status("Timed move stopped.".to_string()))
                        .unwrap_or_default();
                    timer_end = None;
                }
            }

            // Check for new commands without blocking
            match command_receiver.try_recv() {
                Ok(command) => {
                    println!("[Motor Thread] Received command: {:?}", command);
                    let (v, w) = match command {
                        MotorCommand::SetVelocity(v, w) => {
                            timer_end = None;
                            (v, w)
                        }
                        MotorCommand::SetVelocityTimed(v, w, ms) => {
                            timer_end = Some(Instant::now() + Duration::from_millis(ms));
                            (v, w)
                        }
                        MotorCommand::Stop => {
                            timer_end = None;
                            (0.0, 0.0)
                        }
                        MotorCommand::EnableIdShare => {
                            // --- Initialization Sequence ---
                            println!("[Motor Thread] Sending initialization commands...");
                            let init_commands: [&'static [u8]; 6] = [
                                &QUERY_IDSHARE_R,
                                &QUERY_IDSHARE_L,
                                &QUERY_READ_R,
                                &QUERY_READ_L,
                                &QUERY_WRITE_R,
                                &QUERY_WRITE_L,
                            ];

                            for cmd in init_commands.iter() {
                                let mut cmd_mut = cmd.to_vec();
                                if let Err(msg) = send_command(&mut port, &mut cmd_mut) {
                                    eprintln!("[Motor Thread] Error sending init command: {}", msg);
                                    message_sender
                                        .send(MotorMessage::Status(format!("Error on init: {}", msg)))
                                        .unwrap_or_default();
                                }
                            }
                            message_sender
                                .send(MotorMessage::Status("Motor ID Share Enabled.".to_string()))
                                .unwrap_or_default();
                            continue;
                        }
                        MotorCommand::ServoOn => {
                            // --- Servo ON ---
                            println!("[Motor Thread] Sending Servo ON commands.");
                            let mut son_r = QUERY_WRITE_SON_R.to_vec();
                            if let Err(msg) = send_command(&mut port, &mut son_r) {
                                eprintln!("[Motor Thread] Error sending servo ON (R): {}", msg);
                            }
                            let mut son_l = QUERY_WRITE_SON_L.to_vec();
                            if let Err(msg) = send_command(&mut port, &mut son_l) {
                                eprintln!("[Motor Thread] Error sending servo ON (L): {}", msg);
                            }
                            message_sender
                                .send(MotorMessage::Status("Motor Servo ON.".to_string()))
                                .unwrap_or_default();
                            continue; // No velocity command to send
                        }
                        MotorCommand::ServoOff => {
                            println!("[Motor Thread] Sending Servo OFF commands.");
                            let mut soff_r = QUERY_WRITE_SOFF_R.to_vec();
                            if let Err(msg) = send_command(&mut port, &mut soff_r) {
                                eprintln!("[Motor Thread] Error sending servo OFF (R): {}", msg);
                            }
                            let mut soff_l = QUERY_WRITE_SOFF_L.to_vec();
                            if let Err(msg) = send_command(&mut port, &mut soff_l) {
                                eprintln!("[Motor Thread] Error sending servo OFF (L): {}", msg);
                            }
                            message_sender
                                .send(MotorMessage::Status("Motor Servo OFF.".to_string()))
                                .unwrap_or_default();
                            continue; // No velocity command to send
                        }
                        MotorCommand::ServoFree => {
                            println!("[Motor Thread] Sending Servo FREE commands.");
                            let mut sfree_r = QUERY_WRITE_FREE_R.to_vec();
                            if let Err(msg) = send_command(&mut port, &mut sfree_r) {
                                eprintln!("[Motor Thread] Error sending servo FREE (R): {}", msg);
                            }
                            let mut sfree_l = QUERY_WRITE_FREE_L.to_vec();
                            if let Err(msg) = send_command(&mut port, &mut sfree_l) {
                                eprintln!("[Motor Thread] Error sending servo FREE (L): {}", msg);
                            }
                            message_sender
                                .send(MotorMessage::Status("Motor Servo FREE.".to_string()))
                                .unwrap_or_default();
                            continue; // No velocity command to send
                        }
                    };

                    let (motor_wr_rpm, motor_wl_rpm) = calc_vw_to_motor_rpms(v, w, &config);
                    println!(
                        "[Motor Thread] Calculated RPMs: R={}, L={}",
                        motor_wr_rpm, motor_wl_rpm
                    );

                    let mut vel_command = QUERY_NET_ID_WRITE_TEMPLATE.to_vec();
                    vel_command[15..19].copy_from_slice(&motor_wr_rpm.to_be_bytes());
                    vel_command[39..43].copy_from_slice(&motor_wl_rpm.to_be_bytes());

                    if let Err(msg) = send_command(&mut port, &mut vel_command) {
                        eprintln!("[Motor Thread] Error sending command: {}", msg);
                        message_sender
                            .send(MotorMessage::Status(format!("Error: {}", msg)))
                            .unwrap_or_default();
                    } else {
                        message_sender
                            .send(MotorMessage::Status(format!(
                                "Command Executed: {:?}",
                                command
                            )))
                            .unwrap_or_default();
                    }
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
