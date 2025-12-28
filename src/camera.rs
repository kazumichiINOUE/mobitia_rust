use std::{
    sync::{mpsc, Arc},
    thread,
    time::Duration,
};

use eframe::egui;
use image::ImageBuffer;
use nokhwa::{
    pixel_format::RgbFormat,
    utils::{ApiBackend, CameraIndex, RequestedFormat, RequestedFormatType},
    Camera,
};

// メインスレッドに送信するメッセージ
pub enum CameraMessage {
    Frame { id: usize, image: egui::ColorImage },
    Status { id: usize, message: String },
}

// カメラごとの情報
pub struct CameraInfo {
    pub id: usize,
    pub index: CameraIndex,
    pub name: String,
}

// カメラを起動し、フレームをチャネル経由で送信するスレッドを開始する
pub fn start_camera_thread(
    info: CameraInfo,
    sender: mpsc::Sender<CameraMessage>,
    stop_flag: Arc<std::sync::atomic::AtomicBool>,
) {
    thread::spawn(move || {
        sender
            .send(CameraMessage::Status {
                id: info.id,
                message: format!("INFO: Trying to connect to {} ({})", info.name, info.index),
            })
            .unwrap_or_default();

        let format =
            RequestedFormat::new::<RgbFormat>(RequestedFormatType::AbsoluteHighestFrameRate);
        let mut camera = match Camera::new(info.index.clone(), format) {
            Ok(cam) => {
                sender
                    .send(CameraMessage::Status {
                        id: info.id,
                        message: format!("INFO: Connected to {}", info.name),
                    })
                    .unwrap_or_default();
                cam
            }
            Err(e) => {
                sender
                    .send(CameraMessage::Status {
                        id: info.id,
                        message: format!("ERROR: Failed to connect to {}: {}", info.name, e),
                    })
                    .unwrap_or_default();
                return;
            }
        };

        if let Err(e) = camera.open_stream() {
            sender
                .send(CameraMessage::Status {
                    id: info.id,
                    message: format!("ERROR: Failed to open stream for {}: {}", info.name, e),
                })
                .unwrap_or_default();
            return;
        }

        loop {
            // 停止フラグをチェック
            if stop_flag.load(std::sync::atomic::Ordering::SeqCst) {
                sender
                    .send(CameraMessage::Status {
                        id: info.id,
                        message: "INFO: Camera thread shutting down.".to_string(),
                    })
                    .unwrap_or_default();
                break;
            }

            match camera.frame() {
                Ok(frame) => {
                    let resolution = frame.resolution();
                    if let Ok(decoded_image) = frame.decode_image::<RgbFormat>() {
                        let image_buffer: ImageBuffer<image::Rgb<u8>, Vec<u8>> = decoded_image;
                        let size = [resolution.width() as _, resolution.height() as _];
                        let color_image = egui::ColorImage::from_rgb(size, image_buffer.as_raw());
                        sender
                            .send(CameraMessage::Frame {
                                id: info.id,
                                image: color_image,
                            })
                            .unwrap_or_default();
                    } else {
                        sender
                            .send(CameraMessage::Status {
                                id: info.id,
                                message: "ERROR: Failed to decode frame.".to_string(),
                            })
                            .unwrap_or_default();
                    }
                }
                Err(e) => {
                    sender
                        .send(CameraMessage::Status {
                            id: info.id,
                            message: format!("ERROR: Failed to get frame: {}", e),
                        })
                        .unwrap_or_default();
                    // エラーが起きたら少し待つ
                    thread::sleep(Duration::from_millis(500));
                }
            }
            // 1秒待つ
            thread::sleep(Duration::from_secs(1));
        }
    });
}
