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
    ReadState,
}

#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    pub alarm_code_r: i32,
    pub temp_driver_r: f32,
    pub temp_motor_r: f32,
    pub position_r: i32,
    pub power_r: i32,
    pub voltage_r: f32,
    pub alarm_code_l: i32,
    pub temp_driver_l: f32,
    pub temp_motor_l: f32,
    pub position_l: i32,
    pub power_l: i32,
    pub voltage_l: f32,
}

#[derive(Debug)]
pub enum MotorMessage {
    Status(String),
    StateUpdate(MotorState),
    OdometryUpdate { x: f32, y: f32, angle: f32 },
}

#[derive(Debug, Default)]
struct OdoState {
    initialized: bool,
    last_pos_r: i32,
    last_pos_l: i32,
    x: f32,
    y: f32,
    angle: f32, // radians
}

const QUERY_NET_ID_WRITE_TEMPLATE: [u8; 57] = [
    0x0F, 0x10, 0x00, 0x00, 0x00, 0x18, 0x30, // Data for ID=1 (Right Motor)
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

const QUERY_WRITE_SON_R: [u8; 13] = [
    0x01, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
];
const QUERY_WRITE_SON_L: [u8; 13] = [
    0x02, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
];
const QUERY_WRITE_SOFF_R: [u8; 13] = [
    0x01, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];
const QUERY_WRITE_SOFF_L: [u8; 13] = [
    0x02, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];
const QUERY_WRITE_FREE_R: [u8; 13] = [
    0x01, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00,
];
const QUERY_WRITE_FREE_L: [u8; 13] = [
    0x02, 0x10, 0x00, 0x7C, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00,
];

// --- Initialization Commands from coyomi2/query.h ---
const QUERY_IDSHARE_R: [u8; 21] = [
    0x01, 0x10, 0x09, 0x80, 0x00, 0x06, 0x0C, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x02, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00,
];
const QUERY_IDSHARE_L: [u8; 21] = [
    0x02, 0x10, 0x09, 0x80, 0x00, 0x06, 0x0C, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x02, 0x00,
    0x00, 0x00, 0x02, 0x00, 0x00,
];
const QUERY_READ_R: [u8; 33] = [
    0x01, 0x10, 0x09, 0x90, 0x00, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x7C, 0x00,
    0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x9E, 0x00, 0x00, 0x00, 0xA4, 0x00,
    0x00,
];
const QUERY_READ_L: [u8; 33] = [
    0x02, 0x10, 0x09, 0x90, 0x00, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x7C, 0x00,
    0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x9E, 0x00, 0x00, 0x00, 0xA4, 0x00,
    0x00,
];
const QUERY_WRITE_R: [u8; 33] = [
    0x01, 0x10, 0x09, 0xA8, 0x00, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x2D, 0x00, 0x00, 0x00, 0x2E, 0x00,
    0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x31, 0x00, 0x00, 0x00, 0x33, 0x00,
    0x00,
];
const QUERY_WRITE_L: [u8; 33] = [
    0x02, 0x10, 0x09, 0xA8, 0x00, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x2D, 0x00, 0x00, 0x00, 0x2E, 0x00,
    0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x31, 0x00, 0x00, 0x00, 0x33, 0x00,
    0x00,
];
const QUERY_NET_ID_READ: [u8; 8] = [0x0F, 0x03, 0x00, 0x00, 0x00, 0x1A, 0x00, 0x00];

fn circular_diff32(curr: i32, prev: i32) -> i32 {
    (curr as u32).wrapping_sub(prev as u32) as i32
}

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

fn send_and_read(
    port: &mut Box<dyn SerialPort>,
    command_with_crc_space: &mut [u8],
    expected_len: usize,
) -> Result<Vec<u8>, String> {
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
        return Err(format!("Failed to write to port: {}", e));
    }

    // レスポンスを待つために少し待機
    thread::sleep(Duration::from_millis(15));

    if expected_len > 0 {
        let mut response_buf = vec![0; expected_len];
        match port.read_exact(&mut response_buf) {
            Ok(_) => Ok(response_buf),
            Err(e) => {
                if e.kind() == std::io::ErrorKind::TimedOut {
                    println!("[Motor Thread] Warning: Timed out waiting for response.");
                    Err("Timed out".to_string())
                } else {
                    eprintln!("[Motor Thread] Error reading response: {}", e);
                    Err(format!("Read error: {}", e))
                }
            }
        }
    } else {
        Ok(Vec::new())
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
        let mut odo_state = OdoState::default();

        loop {
            // Check for a timed command expiration
            if let Some(end_time) = timer_end {
                if Instant::now() >= end_time {
                    println!("[Motor Thread] Timer expired. Issuing Stop.");
                    let (motor_wr_rpm, motor_wl_rpm): (i32, i32) = (0, 0);
                    let mut stop_command = QUERY_NET_ID_WRITE_TEMPLATE.to_vec();
                    stop_command[15..19].copy_from_slice(&motor_wr_rpm.to_be_bytes());
                    stop_command[39..43].copy_from_slice(&motor_wl_rpm.to_be_bytes());

                    if let Err(msg) = send_and_read(&mut port, &mut stop_command, 8) {
                        eprintln!("[Motor Thread] Error sending timed stop command: {}", msg);
                    }
                    message_sender
                        .send(MotorMessage::Status("Timed move stopped.".to_string()))
                        .unwrap_or_default();
                    timer_end = None;
                }
            }

            // Drain the command queue to get the most recent command, ensuring responsiveness.
            let mut latest_command: Option<MotorCommand> = None;
            while let Ok(cmd) = command_receiver.try_recv() {
                latest_command = Some(cmd);
            }

            if let Some(command) = latest_command {
                let mut should_send_velocity = true;
                let mut v = 0.0;
                let mut w = 0.0;

                // Helper for simple commands expecting an 8-byte response
                let handle_simple_command =
                    |port: &mut Box<dyn SerialPort>, cmd_bytes: &[u8]| -> Result<(), String> {
                        send_and_read(port, &mut cmd_bytes.to_vec(), 8).map(|_| ())
                    };

                match command {
                    MotorCommand::SetVelocity(vel, ang_w) => {
                        v = vel;
                        w = ang_w;
                        timer_end = None;
                    }
                    MotorCommand::SetVelocityTimed(vel, ang_w, ms) => {
                        v = vel;
                        w = ang_w;
                        timer_end = Some(Instant::now() + Duration::from_millis(ms));
                    }
                    MotorCommand::Stop => {
                        v = 0.0;
                        w = 0.0;
                        timer_end = None;
                    }
                    MotorCommand::EnableIdShare => {
                        should_send_velocity = false;
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
                            if let Err(msg) = send_and_read(&mut port, &mut cmd.to_vec(), 8) {
                                let err_msg = format!("Error on init command: {}", msg);
                                eprintln!("[Motor Thread] {}", err_msg);
                                message_sender
                                    .send(MotorMessage::Status(err_msg))
                                    .unwrap_or_default();
                            }
                        }
                        message_sender
                            .send(MotorMessage::Status("Motor ID Share Enabled.".to_string()))
                            .unwrap_or_default();
                    }
                    MotorCommand::ServoOn => {
                        should_send_velocity = false;
                        println!("[Motor Thread] Sending Servo ON commands.");
                        if let Err(e) = handle_simple_command(&mut port, &QUERY_WRITE_SON_R) {
                            eprintln!("[Motor Thread] Error sending servo ON (R): {}", e);
                        }
                        if let Err(e) = handle_simple_command(&mut port, &QUERY_WRITE_SON_L) {
                            eprintln!("[Motor Thread] Error sending servo ON (L): {}", e);
                        }
                        message_sender
                            .send(MotorMessage::Status("Motor Servo ON.".to_string()))
                            .unwrap_or_default();
                    }
                    MotorCommand::ServoOff => {
                        should_send_velocity = false;
                        println!("[Motor Thread] Sending Servo OFF commands.");
                        if let Err(e) = handle_simple_command(&mut port, &QUERY_WRITE_SOFF_R) {
                            eprintln!("[Motor Thread] Error sending servo OFF (R): {}", e);
                        }
                        if let Err(e) = handle_simple_command(&mut port, &QUERY_WRITE_SOFF_L) {
                            eprintln!("[Motor Thread] Error sending servo OFF (L): {}", e);
                        }
                        message_sender
                            .send(MotorMessage::Status("Motor Servo OFF.".to_string()))
                            .unwrap_or_default();
                    }
                    MotorCommand::ServoFree => {
                        should_send_velocity = false;
                        println!("[Motor Thread] Sending Servo FREE commands.");
                        if let Err(e) = handle_simple_command(&mut port, &QUERY_WRITE_FREE_R) {
                            eprintln!("[Motor Thread] Error sending servo FREE (R): {}", e);
                        }
                        if let Err(e) = handle_simple_command(&mut port, &QUERY_WRITE_FREE_L) {
                            eprintln!("[Motor Thread] Error sending servo FREE (L): {}", e);
                        }
                        message_sender
                            .send(MotorMessage::Status("Motor Servo FREE.".to_string()))
                            .unwrap_or_default();
                    }
                    MotorCommand::ReadState => {
                        should_send_velocity = false;
                        println!("[Motor Thread] Reading motor state...");
                        match send_and_read(&mut port, &mut QUERY_NET_ID_READ.to_vec(), 57) {
                            Ok(buf) => {
                                // --- Parse Full Motor State ---
                                let get_i32 = |b: &[u8]| i32::from_be_bytes(b.try_into().unwrap());
                                let get_f32 = |b: &[u8]| {
                                    (i32::from_be_bytes(b.try_into().unwrap()) as f32) * 0.1
                                };

                                const OFFSET: usize = 26;
                                let current_pos_r = get_i32(&buf[15..19]);
                                let current_pos_l = get_i32(&buf[15 + OFFSET..19 + OFFSET]);

                                let motor_state = MotorState {
                                    alarm_code_r: get_i32(&buf[3..7]),
                                    temp_driver_r: get_f32(&buf[7..11]),
                                    temp_motor_r: get_f32(&buf[11..15]),
                                    position_r: current_pos_r,
                                    power_r: get_i32(&buf[19..23]),
                                    voltage_r: get_f32(&buf[23..27]),
                                    alarm_code_l: get_i32(&buf[3 + OFFSET..7 + OFFSET]),
                                    temp_driver_l: get_f32(&buf[7 + OFFSET..11 + OFFSET]),
                                    temp_motor_l: get_f32(&buf[11 + OFFSET..15 + OFFSET]),
                                    position_l: current_pos_l,
                                    power_l: get_i32(&buf[19 + OFFSET..23 + OFFSET]),
                                    voltage_l: get_f32(&buf[23 + OFFSET..27 + OFFSET]),
                                };
                                message_sender
                                    .send(MotorMessage::StateUpdate(motor_state))
                                    .unwrap_or_default();

                                // --- Odometry Calculation ---
                                if !odo_state.initialized {
                                    odo_state.last_pos_r = current_pos_r;
                                    odo_state.last_pos_l = current_pos_l;
                                    odo_state.initialized = true;
                                    println!("[Motor Thread] Odometry initialized.");
                                    message_sender
                                        .send(MotorMessage::OdometryUpdate {
                                            x: odo_state.x,
                                            y: odo_state.y,
                                            angle: odo_state.angle,
                                        })
                                        .unwrap_or_default();
                                } else {
                                    let delta_pos_r =
                                        circular_diff32(current_pos_r, odo_state.last_pos_r);
                                    let delta_pos_l =
                                        circular_diff32(current_pos_l, odo_state.last_pos_l);

                                    let step_res_rad = config.step_resolution_deg.to_radians();
                                    let wheel_d = config.wheel_diameter;
                                    let gear_ratio = config.gear_ratio;
                                    let wheel_t = config.tread_width;

                                    let dist_l =
                                        (delta_pos_l as f32) * step_res_rad * 0.5 * wheel_d
                                            / gear_ratio;
                                    let dist_r =
                                        -(delta_pos_r as f32) * step_res_rad * 0.5 * wheel_d
                                            / gear_ratio;

                                    let dl = (dist_l + dist_r) / 2.0;
                                    let dth = (dist_r - dist_l) / wheel_t;

                                    odo_state.x += dl * odo_state.angle.cos();
                                    odo_state.y += dl * odo_state.angle.sin();
                                    odo_state.angle += dth;

                                    // Normalize angle to [-PI, PI]
                                    if odo_state.angle > PI {
                                        odo_state.angle -= 2.0 * PI;
                                    } else if odo_state.angle < -PI {
                                        odo_state.angle += 2.0 * PI;
                                    }

                                    odo_state.last_pos_r = current_pos_r;
                                    odo_state.last_pos_l = current_pos_l;

                                    message_sender
                                        .send(MotorMessage::OdometryUpdate {
                                            x: odo_state.x,
                                            y: odo_state.y,
                                            angle: odo_state.angle,
                                        })
                                        .unwrap_or_default();
                                }
                            }
                            Err(e) => {
                                let err_msg = format!("Failed to read motor state: {}", e);
                                eprintln!("[Motor Thread] {}", err_msg);
                                message_sender
                                    .send(MotorMessage::Status(err_msg))
                                    .unwrap_or_default();
                            }
                        }
                    }
                };

                if should_send_velocity {
                    let (motor_wr_rpm, motor_wl_rpm) = calc_vw_to_motor_rpms(v, w, &config);
                    // println!(
                    //     "[Motor Thread] Calculated RPMs: R={}, L={}",
                    //     motor_wr_rpm, motor_wl_rpm
                    // );

                    let mut vel_command = QUERY_NET_ID_WRITE_TEMPLATE.to_vec();
                    vel_command[15..19].copy_from_slice(&motor_wr_rpm.to_be_bytes());
                    vel_command[39..43].copy_from_slice(&motor_wl_rpm.to_be_bytes());

                    if let Err(msg) = send_and_read(&mut port, &mut vel_command, 8) {
                        eprintln!("[Motor Thread] Error sending velocity command: {}", msg);
                        message_sender
                            .send(MotorMessage::Status(format!("Error: {}", msg)))
                            .unwrap_or_default();
                    }
                }
            }

            // Handle disconnected state
            if let Err(mpsc::TryRecvError::Disconnected) = command_receiver.try_recv() {
                println!("[Motor Thread] Channel disconnected. Terminating.");
                break;
            }

            thread::sleep(Duration::from_millis(10)); // Polling interval
        }
    })
}
