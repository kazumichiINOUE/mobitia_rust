use anyhow::Result;
use serialport;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use crate::app::LidarMessage;

use super::comm::LidarConnection;
use super::protocol::{decode_scip_2_0_3char, decode_scip_2_0_4char};

#[derive(Debug, Clone)]
pub struct LidarInfo {
    pub lidar_path: String,
    pub baud_rate: u32,
}

pub struct LidarDriver {
    connection: LidarConnection,
}

impl LidarDriver {
    pub fn new(info: &LidarInfo) -> Result<Self> {
        // ここからシリアルポートの存在チェックロジックを追加します
        let available_ports = serialport::available_ports()?;
        let port_exists = available_ports
            .iter()
            .any(|p| p.port_name == info.lidar_path);

        if !port_exists {
            return Err(anyhow::anyhow!(
                "\n指定されたポート '{}' が見つかりません。",
                info.lidar_path
            ));
        }
        // ここまで追加してください

        let connection = LidarConnection::new(&info.lidar_path, info.baud_rate)?;
        Ok(Self { connection })
    }

    pub fn initialize(
        &mut self,
        lidar_id: usize,
        message_sender: &mpsc::Sender<LidarMessage>,
    ) -> Result<()> {
        let init_commands: &[&[u8]] = &[b"VV\n", b"PP\n", b"II\n"];
        for cmd in init_commands {
            message_sender
                .send(LidarMessage::StatusUpdate {
                    id: lidar_id,
                    message: format!(
                        "INFO: Sending command: {}",
                        String::from_utf8_lossy(*cmd).trim()
                    ),
                })
                .unwrap_or_default();
            let response = self.connection.send_and_receive(*cmd)?;
            message_sender
                .send(LidarMessage::StatusUpdate {
                    id: lidar_id,
                    message: format!(
                        "INFO: Response for {}: {}",
                        String::from_utf8_lossy(*cmd).trim(),
                        response.trim()
                    ),
                })
                .unwrap_or_default();
            thread::sleep(Duration::from_millis(50));
        }

        message_sender
            .send(LidarMessage::StatusUpdate {
                id: lidar_id,
                message: format!("INFO: Sending command: BM"),
            })
            .unwrap_or_default();
        let response_bm = self.connection.send_and_receive(b"BM\n")?;
        message_sender
            .send(LidarMessage::StatusUpdate {
                id: lidar_id,
                message: format!("INFO: Response for BM: {}", response_bm.trim()),
            })
            .unwrap_or_default();
        thread::sleep(Duration::from_millis(50));
        message_sender
            .send(LidarMessage::StatusUpdate {
                id: lidar_id,
                message: "INFO: LiDAR initialized. Laser is ON.".to_string(),
            })
            .unwrap_or_default();
        Ok(())
    }

    pub fn get_distance_data(&mut self) -> Result<Vec<(f32, f32, f32, f32, f32)>> {
        let command = b"GD0000108001\n";
        let response = self.connection.send_and_receive(command)?;

        let lines: Vec<&str> = response.trim().lines().collect();
        if lines.len() < 3 {
            return Err(anyhow::anyhow!(
                "Malformed GD response (expected at least 3 lines)"
            ));
        }

        // Decode timestamp (for logging/debugging, not used in calculation here)
        let timestamp_line = lines[2];
        if timestamp_line.len() >= 4 {
            let encoded_timestamp = &timestamp_line[0..4];
            let _timestamp_value = decode_scip_2_0_4char(encoded_timestamp)?;
        }

        // Parse data
        let mut lidar_points_current_scan: Vec<(f32, f32, f32, f32, f32)> = Vec::new();
        let gd_params = &command[2..10];
        let start_step =
            u32::from_str_radix(&String::from_utf8_lossy(&gd_params[0..4]), 10).unwrap_or(0);
        let end_step =
            u32::from_str_radix(&String::from_utf8_lossy(&gd_params[4..8]), 10).unwrap_or(0);

        let max_angle = (end_step as f32) * 0.25_f32 - 135.0;
        let angle_increment = 0.25_f32;
        let mut current_angle = (start_step as f32) * 0.25_f32 - 135.0;

        let mut all_data_chars = String::new();
        for data_line_with_checksum in lines.iter().skip(3) {
            if data_line_with_checksum.len() > 0 {
                let data_line = &data_line_with_checksum[0..data_line_with_checksum.len() - 1];
                all_data_chars.push_str(data_line);
            }
        }

        for chunk in all_data_chars.as_bytes().chunks(3) {
            let encoded_distance = String::from_utf8_lossy(chunk);
            if encoded_distance.len() != 3 {
                current_angle += angle_increment;
                continue;
            }
            match decode_scip_2_0_3char(&encoded_distance) {
                Ok(distance_mm) => {
                    if distance_mm > 1 && distance_mm < 30000 && current_angle <= max_angle {
                        let distance_m = distance_mm as f32 / 1000.0;
                        let angle_rad = current_angle.to_radians();
                        let x = distance_m * angle_rad.cos();
                        let y = distance_m * angle_rad.sin();
                        lidar_points_current_scan.push((x, y, distance_m, angle_rad, 0.0)); // 0.0をfeatureの初期値として追加
                    }
                }
                Err(_) => {
                    // Ignore decode errors for now
                }
            }
            current_angle += angle_increment;
        }

        Ok(lidar_points_current_scan)
    }

    pub fn stop_laser(&mut self) {
        let _ = self.connection.send_and_receive(b"QT\n");
    }
}

/// Spawns a thread to continuously fetch data from the LiDAR.
pub fn start_lidar_thread(
    lidar_id: usize, // Lidar IDを追加
    lidar_config: LidarInfo,
    message_sender: mpsc::Sender<LidarMessage>, // 単一のメッセージセンダー
) {
    thread::spawn(move || {
        // Lidar Path と Baud Rate をLidarMessageとして送信
        message_sender
            .send(LidarMessage::StatusUpdate {
                id: lidar_id,
                message: format!("LiDAR Path: {}", lidar_config.lidar_path),
            })
            .unwrap_or_default();
        message_sender
            .send(LidarMessage::StatusUpdate {
                id: lidar_id,
                message: format!("Baud Rate: {}", lidar_config.baud_rate),
            })
            .unwrap_or_default();

        let mut driver = match LidarDriver::new(&lidar_config) {
            Ok(d) => {
                message_sender
                    .send(LidarMessage::StatusUpdate {
                        id: lidar_id,
                        message: format!(
                            "INFO: Successfully opened port {}",
                            lidar_config.lidar_path
                        ),
                    })
                    .unwrap_or_default();
                d
            }
            Err(e) => {
                message_sender
                    .send(LidarMessage::StatusUpdate {
                        id: lidar_id,
                        message: format!(
                            "ERROR: Failed to open port '{}': {}",
                            lidar_config.lidar_path, e
                        ),
                    })
                    .unwrap_or_default();
                return;
            }
        };

        // initialize関数にstatus_senderを渡す代わりに、message_senderを使うように変更
        // driver.initialize()のシグネチャも変更する必要がある
        if let Err(e) = driver.initialize(lidar_id, &message_sender) {
            message_sender
                .send(LidarMessage::StatusUpdate {
                    id: lidar_id,
                    message: format!("ERROR: Failed to initialize LiDAR: {}", e),
                })
                .unwrap_or_default();
            return;
        }

        let mut was_connected = true; // 接続状態を追跡

        loop {
            match driver.get_distance_data() {
                Ok(points) => {
                    if !was_connected {
                        // エラー状態から回復した場合、ステータスを更新
                        message_sender
                            .send(LidarMessage::StatusUpdate {
                                id: lidar_id,
                                message: "INFO: LiDAR initialized. Laser is ON.".to_string(), // 接続成功を示すメッセージ
                            })
                            .unwrap_or_default();
                        was_connected = true;
                    }

                    if message_sender
                        .send(LidarMessage::ScanUpdate {
                            id: lidar_id,
                            scan: points,
                        })
                        .is_err()
                    {
                        // メインスレッドが閉じられた可能性
                        break;
                    }
                }
                Err(e) => {
                    // データ取得に失敗
                    message_sender
                        .send(LidarMessage::StatusUpdate {
                            id: lidar_id,
                            message: format!("ERROR: Failed to get distance data: {}", e),
                        })
                        .unwrap_or_default();
                    was_connected = false;
                }
            }
            thread::sleep(Duration::from_millis(100));
        }

        driver.stop_laser();
        message_sender
            .send(LidarMessage::StatusUpdate {
                id: lidar_id,
                message: "INFO: LiDAR thread stopped. Laser is OFF.".to_string(),
            })
            .unwrap_or_default();
    });
}
