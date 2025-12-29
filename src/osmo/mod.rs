use eframe::egui;
use std::io::{BufRead, BufReader, Read, Write};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;

// Osmoからのメッセージを定義
pub enum OsmoMessage {
    Frame { id: usize, image: egui::ColorImage },
    Status { id: usize, message: String },
}

// Osmoデバイスの情報を保持する構造体
#[derive(Clone)]
pub struct OsmoInfo {
    pub id: usize,
    pub name: String,
    // Pythonスクリプトのパスなど、将来的な設定項目
}

pub fn start_osmo_thread(
    info: OsmoInfo,
    sender: mpsc::Sender<OsmoMessage>,
    stop_flag: Arc<AtomicBool>,
) {
    thread::spawn(move || {
        sender
            .send(OsmoMessage::Status {
                id: info.id,
                message: format!(
                    "[Osmo {}] Thread started. Spawning python script...",
                    info.name
                ),
            })
            .unwrap_or_default();

        let mut child = match Command::new("python3")
            .arg("./src/osmo/osmo_capture.py")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
        {
            Ok(child) => {
                sender
                    .send(OsmoMessage::Status {
                        id: info.id,
                        message: "[Osmo] Python script spawned successfully.".to_string(),
                    })
                    .unwrap_or_default();
                child
            }
            Err(e) => {
                sender
                    .send(OsmoMessage::Status {
                        id: info.id,
                        message: format!("[Osmo] ERROR: Failed to spawn Python script: {}", e),
                    })
                    .unwrap_or_default();
                return;
            }
        };

        let mut stdin = child.stdin.take().unwrap();
        let mut stdout = child.stdout.take().unwrap();
        let stderr = child.stderr.take().unwrap();

        // Stderrを別スレッドで読み取り、UIに送信
        let sender_clone = sender.clone();
        let info_clone = info.clone();
        thread::spawn(move || {
            let reader = BufReader::new(stderr);
            for line in reader.lines() {
                match line {
                    Ok(line) => sender_clone
                        .send(OsmoMessage::Status {
                            id: info_clone.id,
                            message: format!("[Osmo stderr] {}", line),
                        })
                        .unwrap_or_default(),
                    Err(e) => sender_clone
                        .send(OsmoMessage::Status {
                            id: info_clone.id,
                            message: format!("[Osmo] ERROR reading stderr: {}", e),
                        })
                        .unwrap_or_default(),
                }
            }
        });

        // メインのフレーム取得ループ
        while !stop_flag.load(Ordering::SeqCst) {
            // Pythonにキャプチャを要求
            if let Err(e) = stdin.write_all(b"capture\n") {
                sender
                    .send(OsmoMessage::Status {
                        id: info.id,
                        message: format!("[Osmo] ERROR writing to python stdin: {}", e),
                    })
                    .unwrap_or_default();
                break;
            }
            if let Err(e) = stdin.flush() {
                sender
                    .send(OsmoMessage::Status {
                        id: info.id,
                        message: format!("[Osmo] ERROR flushing stdin: {}", e),
                    })
                    .unwrap_or_default();
                break;
            }

            // Pythonからフレームデータを読み取り
            // 1. メタデータ (width, height, channels)
            let mut meta_buf = [0u8; 12];
            if stdout.read_exact(&mut meta_buf).is_err() {
                // INFOレベルの終了メッセージはPython側で出力されるはず
                // sender.send(OsmoMessage::Status { id: info.id, message: "[Osmo] Failed to read metadata. Exiting.".to_string() }).unwrap_or_default();
                break;
            }
            let width = u32::from_le_bytes(meta_buf[0..4].try_into().unwrap()) as usize;
            let height = u32::from_le_bytes(meta_buf[4..8].try_into().unwrap()) as usize;
            let channels = u32::from_le_bytes(meta_buf[8..12].try_into().unwrap()) as usize;

            // 2. データ長
            let mut len_buf = [0u8; 8];
            if stdout.read_exact(&mut len_buf).is_err() {
                // sender.send(OsmoMessage::Status { id: info.id, message: "[Osmo] Failed to read data length. Exiting.".to_string() }).unwrap_or_default();
                break;
            }
            let data_len = u64::from_le_bytes(len_buf) as usize;

            if data_len != width * height * channels {
                sender
                    .send(OsmoMessage::Status {
                        id: info.id,
                        message: format!(
                            "[Osmo] ERROR: Data length mismatch. Expected {}, got {}",
                            width * height * channels,
                            data_len
                        ),
                    })
                    .unwrap_or_default();
                continue;
            }

            // 3. フレームデータ
            let mut frame_data = vec![0u8; data_len];
            if stdout.read_exact(&mut frame_data).is_err() {
                // sender.send(OsmoMessage::Status { id: info.id, message: "[Osmo] Failed to read frame data. Exiting.".to_string() }).unwrap_or_default();
                break;
            }

            // BGRからRGBに変換し、egui::ColorImageを作成
            let rgb_data: Vec<u8> = frame_data
                .chunks_exact(3)
                .flat_map(|bgr| [bgr[2], bgr[1], bgr[0]]) // BGR -> RGB
                .collect();

            let image = egui::ColorImage::from_rgb([width, height], &rgb_data);

            // UIスレッドに送信
            if sender
                .send(OsmoMessage::Frame { id: info.id, image })
                .is_err()
            {
                // UIスレッドが終了した場合
                break;
            }
        }

        sender
            .send(OsmoMessage::Status {
                id: info.id,
                message: "[Osmo] Thread finished. Killing python script.".to_string(),
            })
            .unwrap_or_default();

        // 子プロセスを終了させる
        child.kill().ok();
        child.wait().ok();
    });
}
