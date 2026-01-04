use anyhow::Result;
use chrono::Local;
use clap::Parser;
use eframe::egui;
use egui::Vec2;
use nalgebra::{Isometry2, Point2};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

use crate::camera::{start_camera_thread, CameraInfo, CameraMessage};
use crate::cli::Cli;
use crate::demo::DemoManager;
pub use crate::demo::DemoMode;
use crate::lidar::{load_lidar_configurations, start_lidar_thread, LidarInfo};
use crate::osmo::{start_osmo_thread, OsmoInfo, OsmoMessage};
use crate::slam::SlamManager; // Add SlamManager
use crate::xppen::{start_xppen_thread, XppenMessage};

// Camera一台分の状態を保持する構造体
#[derive(Clone)]
pub(crate) struct CameraState {
    pub(crate) id: usize,
    pub(crate) name: String,
    pub(crate) texture: Option<egui::TextureHandle>,
    pub(crate) connection_status: String,
    pub(crate) stop_flag: Arc<AtomicBool>,
}

// Osmo一台分の状態を保持する構造体
#[derive(Clone)]
pub(crate) struct OsmoState {
    pub(crate) id: usize,
    pub(crate) name: String,
    pub(crate) texture: Option<egui::TextureHandle>,
    pub(crate) connection_status: String,
    pub(crate) stop_flag: Arc<AtomicBool>,
    pub(crate) latest_image: Option<egui::ColorImage>,
}

// XPPenデバイスの状態を保持する構造体
#[derive(Clone)]
pub(crate) struct XppenState {
    pub(crate) connection_status: String,
}

// Lidar一台分の状態を保持する構造体
#[derive(Clone)]
pub(crate) struct LidarState {
    pub(crate) id: usize,
    pub(crate) path: String,
    pub(crate) baud_rate: u32,
    pub(crate) points: Vec<(f32, f32, f32, f32, f32, f32, f32)>,
    pub(crate) connection_status: String,
    pub(crate) status_messages: Vec<String>,
    // ワールド座標におけるこのLidarの原点オフセット
    pub(crate) origin: Vec2,
    // ワールド座標におけるこのLidarの回転オフセット (ラジアン)
    pub(crate) rotation: f32,
    // Lidarごとのデータフィルタリング範囲
    pub(crate) data_filter_angle_min: f32, // LiDARデータフィルタリングの最小角度 (度)
    pub(crate) data_filter_angle_max: f32, // LiDARデータフィルタリングの最大角度 (度)
    // SLAM計算にこのLidarを含めるか
    pub(crate) is_active_for_slam: bool,
}

// アプリケーション全体のの状態を管理する構造体
#[derive(PartialEq)]
pub enum AppMode {
    Lidar,
    Slam,
    Demo,
    Camera,
    Osmo,
    LidarAnalysis,
}

#[derive(PartialEq)]
pub enum SlamMode {
    Manual,
    Continuous,
    Paused,
}

pub enum LidarMessage {
    ScanUpdate {
        id: usize,
        scan: Vec<(f32, f32, f32, f32, f32, f32, f32)>,
    },
    StatusUpdate {
        id: usize,
        message: String,
    },
}

pub enum SuggestionNavigationDirection {
    Up,
    Down,
}

#[allow(dead_code)]
pub enum SlamThreadCommand {
    StartContinuous,
    Pause,
    Resume,
    UpdateScan {
        raw_scan: Vec<(f32, f32, f32, f32, f32, f32, f32)>,
        interpolated_scan: Vec<(f32, f32, f32, f32, f32, f32, f32)>,
        timestamp: u128,
    }, // LiDARからの新しいスキャンデータ
    ProcessSingleScan {
        raw_scan: Vec<(f32, f32, f32, f32, f32, f32, f32)>,
        interpolated_scan: Vec<(f32, f32, f32, f32, f32, f32, f32)>,
        timestamp: u128,
    }, // 単一スキャン処理要求
    Shutdown,
}

pub struct SlamThreadResult {
    pub map_points: Vec<(nalgebra::Point2<f32>, f64)>,
    pub robot_pose: nalgebra::Isometry2<f32>,
    pub scan_used: Vec<(f32, f32, f32, f32, f32, f32, f32)>,
}

/// サブマップのメタデータ構造体 (app.rsに移動)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Submap {
    pub id: usize,
    pub pose_x: f32,
    pub pose_y: f32,
    pub pose_theta: f32,
    pub timestamp_ms: u128,
    pub points_file: String,
    pub info_file: String,
}

// アプリケーション全体の状態を管理する構造体
pub struct ConsoleOutputEntry {
    pub text: String,
    pub group_id: usize, // どのコマンド実行に属するか
}

pub struct MyApp {
    pub(crate) input_string: String, // コンソールの入力文字列
    pub(crate) command_history: Vec<ConsoleOutputEntry>, // コンソールの表示履歴
    pub(crate) user_command_history: Vec<String>, // ユーザーのコマンド入力履歴
    pub(crate) history_index: usize, // コマンド履歴のインデックス
    pub(crate) current_suggestions: Vec<String>, // 現在のサジェスト候補
    pub(crate) suggestion_selection_index: Option<usize>, // サジェスト候補の選択インデックス
    pub(crate) lidars: Vec<LidarState>, // 複数Lidarの状態を管理
    pub(crate) cameras: Vec<CameraState>, // 複数Cameraの状態を管理
    pub(crate) osmo: OsmoState,      // Osmoの状態を管理
    pub(crate) xppen: XppenState,    // XPPenの状態を管理

    // スレッドからのデータ/ステータス受信を統合
    pub(crate) lidar_message_receiver: mpsc::Receiver<LidarMessage>,
    pub(crate) camera_message_receiver: mpsc::Receiver<CameraMessage>,
    pub(crate) osmo_message_receiver: mpsc::Receiver<OsmoMessage>,
    pub(crate) xppen_message_receiver: mpsc::Receiver<XppenMessage>,
    pub(crate) xppen_status_receiver: mpsc::Receiver<String>,
    pub(crate) xppen_trigger_sender: mpsc::Sender<()>,

    // SLAM用に各Lidarの最新スキャンを保持
    pub(crate) pending_scans: Vec<Option<Vec<(f32, f32, f32, f32, f32, f32, f32)>>>,

    // UI関連
    pub(crate) command_output_receiver: mpsc::Receiver<String>,

    pub(crate) command_output_sender: mpsc::Sender<String>,

    pub(crate) lidar_draw_rect: Option<egui::Rect>, // 描画エリアは共通

    pub(crate) app_mode: AppMode,

    pub(crate) demo_manager: DemoManager,

    pub(crate) show_command_window: bool,

    pub(crate) focus_console_requested: bool,

    pub(crate) requested_point_save_path: Option<String>, // TODO: これもLidarごとになる可能性

    pub(crate) next_group_id: usize,

    pub(crate) slam_mode: SlamMode,

    // SLAMスレッドとの通信用チャネル
    pub(crate) slam_command_sender: mpsc::Sender<SlamThreadCommand>,

    pub(crate) slam_result_receiver: mpsc::Receiver<SlamThreadResult>,

    // SLAMが処理中かどうかを示す共有フラグ
    pub(crate) is_slam_processing: Arc<AtomicBool>,

    // SLAMスレッドからの最新結果を保持 (描画用)
    pub(crate) current_map_points: Vec<(nalgebra::Point2<f32>, f64)>,

    pub(crate) current_robot_pose: nalgebra::Isometry2<f32>,

    pub(crate) single_scan_requested_by_ui: bool,

    pub(crate) latest_scan_for_draw: Vec<(f32, f32, f32, f32, f32, f32, f32)>,

    pub(crate) robot_trajectory: Vec<(egui::Pos2, f32)>,

    // --- サブマップ関連のフィールド (MyAppに移行) ---
    pub(crate) submap_counter: usize,
    pub(crate) current_submap_scan_buffer: Vec<Vec<(f32, f32)>>,
    pub(crate) current_submap_robot_poses: Vec<nalgebra::Isometry2<f32>>,
    pub(crate) submaps: HashMap<usize, Submap>,
    pub(crate) slam_map_bounding_box: Option<egui::Rect>,

    /// `list-and-load`コマンドで読み込むサブマップのパスのキュー
    pub(crate) submap_load_queue: Option<Vec<PathBuf>>,
    /// 最後にサブマップを読み込んだ時刻
    pub(crate) last_submap_load_time: Option<Instant>,

    /// F6キーによるサジェスト補完が要求されたか
    pub(crate) suggestion_completion_requested: bool,

    /// F11キーによるコマンド実行が要求されたか
    pub(crate) command_submission_requested: bool,

    /// F10キーによるクリアコマンドが要求されたか
    pub(crate) clear_command_requested: bool,

    pub(crate) osmo_capture_session_path: Option<PathBuf>,

    pub(crate) config: crate::config::Config, // 追加
}

impl MyApp {
    /// Creates a new instance of the application.
    pub fn new(cc: &eframe::CreationContext, config: crate::config::Config) -> Self {
        // Lidar の初期設定を読み込む
        let lidars = load_lidar_configurations(|id| {
            cc.storage
                .as_deref()
                .and_then(|s| s.get_string(&format!("lidar_path_{}", id)))
        });

        // pending_scansベクターをLidarの総数で初期化
        let pending_scans = vec![None; lidars.len()];

        // Lidarメッセージング用の単一チャネルを作成
        let (lidar_message_sender, lidar_message_receiver) = mpsc::channel();
        // Cameraメッセージング用の単一チャネルを作成
        let (camera_message_sender, camera_message_receiver) = mpsc::channel();
        // Osmoメッセージング用の単一チャネルを作成
        let (osmo_message_sender, osmo_message_receiver) = mpsc::channel();
        // XPPenメッセージング用の単一チャネルを作成
        let (xppen_message_sender, xppen_message_receiver) = mpsc::channel();
        // XPPenトリガー用のチャネルを作成
        let (xppen_trigger_sender, xppen_trigger_receiver) = mpsc::channel();
        // XPPenステータス用のチャネルを作成
        let (xppen_status_sender, xppen_status_receiver) = mpsc::channel();

        // コンソールコマンド出力用のチャネルを作成
        let (command_output_sender, command_output_receiver) = mpsc::channel();

        // XPPenスレッドを起動
        start_xppen_thread(
            xppen_message_sender,
            xppen_status_sender.clone(),
            xppen_trigger_receiver,
        );

        // 各Lidarに対してスレッドを起動
        for lidar_state in &lidars {
            let lidar_config = LidarInfo {
                lidar_path: lidar_state.path.clone(),
                baud_rate: lidar_state.baud_rate,
            };
            start_lidar_thread(
                lidar_state.id,
                lidar_config.clone(),
                lidar_message_sender.clone(),
            );
        }

        // カメラの初期化
        let mut cameras = Vec::new();
        let camera_devices = nokhwa::query(nokhwa::utils::ApiBackend::Auto).unwrap_or_else(|e| {
            command_output_sender
                .send(format!("ERROR: Failed to query cameras: {}", e))
                .unwrap_or_default();
            vec![]
        });

        // 内蔵カメラを特定して優先的に使用
        let mut selected_camera_device = None;
        for device in camera_devices.iter() {
            // macOSでは一般的に内蔵カメラの名前には"FaceTime"が含まれることが多い
            // または、最初のデバイスをデフォルトとする
            if device.human_name().contains("FaceTime") || selected_camera_device.is_none() {
                selected_camera_device = Some(device);
                if device.human_name().contains("FaceTime") {
                    // FaceTimeカメラを強く優先
                    break;
                }
            }
        }

        if let Some(device) = selected_camera_device {
            let i = 0; // 内蔵カメラは常にID 0 とする
            let stop_flag = Arc::new(AtomicBool::new(false));
            let camera_state = CameraState {
                id: i,
                name: device.human_name(),
                texture: None,
                connection_status: "Disconnected".to_string(),
                stop_flag: stop_flag.clone(),
            };
            cameras.push(camera_state);

            let info = CameraInfo {
                id: i,
                index: device.index().clone(),
                name: device.human_name(),
            };

            // ここでスレッドを開始
            start_camera_thread(info, camera_message_sender.clone(), stop_flag);
        } else {
            command_output_sender
                .send("WARNING: No camera found or internal camera not identified.".to_string())
                .unwrap_or_default();
        }

        // Osmoの初期化
        let osmo_stop_flag = Arc::new(AtomicBool::new(false));
        let osmo_state = OsmoState {
            id: 0,
            name: "Osmo Pocket 3".to_string(),
            texture: None,
            connection_status: "Disconnected".to_string(),
            stop_flag: osmo_stop_flag.clone(),
            latest_image: None,
        };
        let osmo_info = OsmoInfo {
            id: 0,
            name: "Osmo Pocket 3".to_string(),
        };
        start_osmo_thread(osmo_info, osmo_message_sender.clone(), osmo_stop_flag);

        // SLAM処理中フラグの作成
        let is_slam_processing = Arc::new(AtomicBool::new(false));

        // SLAMスレッドとの通信チャネル
        let (slam_command_sender, slam_command_receiver) = mpsc::channel();
        let (slam_result_sender, slam_result_receiver) = mpsc::channel();
        let is_slam_processing_for_thread = is_slam_processing.clone();
        let slam_config_for_thread = config.slam.clone();

        // SLAMスレッドの起動
        // タイムスタンプに基づいたSLAM結果保存ディレクトリのパスを決定
        let now = Local::now();
        let timestamp_str = now.format("%Y%m%d-%H%M%S").to_string();
        let slam_results_base_path = PathBuf::from("./slam_results");
        let slam_results_path =
            slam_results_base_path.join(format!("slam_result_{}", timestamp_str));

        thread::spawn(move || {
            let mut slam_manager = SlamManager::new(
                slam_results_path,
                slam_config_for_thread.clone(), // Pass slam config
            );
            let mut current_slam_mode = SlamMode::Manual;
            let mut last_slam_update_time = web_time::Instant::now(); // std::time::Instant の代わりにweb_time::Instantを使用
            const SLAM_UPDATE_INTERVAL_DURATION: web_time::Duration =
                web_time::Duration::from_millis(1000); // 500ミリ秒に変更

            loop {
                // UIスレッドからのコマンドを処理
                if let Ok(cmd) = slam_command_receiver.try_recv() {
                    match cmd {
                        SlamThreadCommand::StartContinuous => {
                            current_slam_mode = SlamMode::Continuous;
                            slam_result_sender
                                .send(SlamThreadResult {
                                    map_points: slam_manager.get_map_points().clone(),
                                    robot_pose: *slam_manager.get_robot_pose(),
                                    scan_used: Vec::new(),
                                })
                                .unwrap_or_default();
                        }
                        SlamThreadCommand::Pause => current_slam_mode = SlamMode::Paused,
                        SlamThreadCommand::Resume => {
                            current_slam_mode = SlamMode::Continuous;
                            last_slam_update_time = web_time::Instant::now(); // リセット
                            slam_result_sender
                                .send(SlamThreadResult {
                                    map_points: slam_manager.get_map_points().clone(),
                                    robot_pose: *slam_manager.get_robot_pose(),
                                    scan_used: Vec::new(),
                                })
                                .unwrap_or_default();
                        }
                        SlamThreadCommand::UpdateScan {
                            raw_scan,
                            interpolated_scan,
                            timestamp,
                        } => {
                            if current_slam_mode == SlamMode::Continuous {
                                let now = web_time::Instant::now();
                                if now.duration_since(last_slam_update_time)
                                    >= SLAM_UPDATE_INTERVAL_DURATION
                                {
                                    is_slam_processing_for_thread.store(true, Ordering::SeqCst);

                                    let slam_start_time = web_time::Instant::now();
                                    slam_manager.update(&raw_scan, &interpolated_scan, timestamp);
                                    let slam_duration = slam_start_time.elapsed();
                                    println!("[SLAM Thread] Update took: {:?}", slam_duration);

                                    slam_result_sender
                                        .send(SlamThreadResult {
                                            map_points: slam_manager.get_map_points().clone(),
                                            robot_pose: *slam_manager.get_robot_pose(),
                                            scan_used: raw_scan.clone(),
                                        })
                                        .unwrap_or_default();
                                    last_slam_update_time = now;
                                    is_slam_processing_for_thread.store(false, Ordering::SeqCst);
                                }
                            }
                        }
                        SlamThreadCommand::ProcessSingleScan {
                            raw_scan,
                            interpolated_scan,
                            timestamp,
                        } => {
                            is_slam_processing_for_thread.store(true, Ordering::SeqCst);

                            slam_manager.update(&raw_scan, &interpolated_scan, timestamp);

                            slam_result_sender
                                .send(SlamThreadResult {
                                    map_points: slam_manager.get_map_points().clone(),
                                    robot_pose: *slam_manager.get_robot_pose(),
                                    scan_used: raw_scan.clone(),
                                })
                                .unwrap_or_default();
                            is_slam_processing_for_thread.store(false, Ordering::SeqCst);
                        }
                        SlamThreadCommand::Shutdown => break,
                    }
                }

                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        });

        let command_history = vec![
            ConsoleOutputEntry {
                text: "Welcome to the interactive console!".to_string(),
                group_id: 0,
            },
            ConsoleOutputEntry {
                text: "Press Ctrl+P (Cmd+P on macOS) to toggle the floating console.".to_string(),
                group_id: 0,
            },
            ConsoleOutputEntry {
                text: "Type 'help' for a list of commands.".to_string(),
                group_id: 0,
            },
        ];
        let user_command_history = Vec::new();
        let history_index = user_command_history.len();

        Self {
            input_string: String::new(),
            command_history,
            user_command_history,
            history_index,
            current_suggestions: Vec::new(),
            suggestion_selection_index: None,
            lidars,
            cameras,
            osmo: osmo_state,
            lidar_message_receiver,
            camera_message_receiver,
            osmo_message_receiver,
            xppen_message_receiver,
            xppen_status_receiver,
            xppen_trigger_sender,
            pending_scans,
            command_output_receiver,
            command_output_sender,
            lidar_draw_rect: None,
            app_mode: AppMode::Lidar,
            demo_manager: DemoManager::new(),
            show_command_window: true,
            focus_console_requested: true,
            requested_point_save_path: None,
            next_group_id: 0,
            slam_mode: SlamMode::Manual,
            slam_command_sender,
            slam_result_receiver,
            is_slam_processing: is_slam_processing.clone(),
            current_map_points: Vec::new(),
            current_robot_pose: nalgebra::Isometry2::identity(),
            single_scan_requested_by_ui: false,
            latest_scan_for_draw: Vec::new(),
            robot_trajectory: Vec::new(),

            submap_counter: 0,
            current_submap_scan_buffer: Vec::new(),
            current_submap_robot_poses: Vec::new(),
            submaps: HashMap::new(),
            slam_map_bounding_box: None,

            submap_load_queue: None,
            last_submap_load_time: None,
            suggestion_completion_requested: false,
            command_submission_requested: false,
            clear_command_requested: false,
            osmo_capture_session_path: None,
            xppen: XppenState {
                connection_status: "Disconnected".to_string(),
            },
            config,
        }
    }

    pub fn request_save_points(&mut self, path: String) {
        self.requested_point_save_path = Some(path);
    }

    pub fn capture_osmo_image(&mut self) {
        let sender_clone = self.command_output_sender.clone();

        let image_to_save = match self.osmo.latest_image.clone() {
            Some(img) => img,
            None => {
                sender_clone
                    .send("ERROR: No Osmo image available to capture.".to_string())
                    .unwrap_or_default();
                return;
            }
        };

        // セッションパスが未設定の場合のみ、新しいパスを生成・設定する
        if self.osmo_capture_session_path.is_none() {
            let now = Local::now();
            let timestamp_dir_str = now.format("%Y%m%d-%H%M%S").to_string();
            let base_path = PathBuf::from("./osmo_images");
            let new_path = base_path.join(format!("osmo_result_{}", timestamp_dir_str));

            if let Err(e) = std::fs::create_dir_all(&new_path) {
                sender_clone
                    .send(format!(
                        "ERROR: Failed to create directory '{}': {}",
                        new_path.display(),
                        e
                    ))
                    .unwrap_or_default();
                return; // ディレクトリ作成に失敗したらここで終了
            }
            self.osmo_capture_session_path = Some(new_path);
        }

        // `osmo_capture_session_path` が Some であることを確信して unwrap
        let result_path = self.osmo_capture_session_path.clone().unwrap();

        // 別スレッドでファイルI/Oを実行
        thread::spawn(move || {
            // egui::ColorImage を image::RgbaImage に変換
            // この変換は egui の Color32 (RGBA a=premultiplied) から image の Rgba (straight alpha) への変換
            let pixels: Vec<u8> = image_to_save
                .pixels
                .iter()
                .flat_map(|color| {
                    if color.a() == 0 {
                        [0, 0, 0, 0]
                    } else {
                        // Un-premultiply alpha
                        let r = (color.r() as f32 * 255.0 / color.a() as f32) as u8;
                        let g = (color.g() as f32 * 255.0 / color.a() as f32) as u8;
                        let b = (color.b() as f32 * 255.0 / color.a() as f32) as u8;
                        [r, g, b, color.a()]
                    }
                })
                .collect();

            let buffer: image::ImageBuffer<image::Rgba<u8>, Vec<u8>> =
                match image::ImageBuffer::from_raw(
                    image_to_save.width() as u32,
                    image_to_save.height() as u32,
                    pixels,
                ) {
                    Some(buffer) => buffer,
                    None => {
                        sender_clone
                            .send("ERROR: Failed to create image buffer.".to_string())
                            .unwrap_or_default();
                        return;
                    }
                };

            // ファイルパスを生成
            let timestamp_file_str = Local::now().format("%Y%m%d_%H%M%S").to_string();
            let filename = format!("osmo_{}.png", timestamp_file_str);
            let save_path = result_path.join(filename);

            // ファイルに保存
            match buffer.save(&save_path) {
                Ok(_) => sender_clone
                    .send(format!("Osmo image saved to: {}", save_path.display()))
                    .unwrap_or_default(),
                Err(e) => sender_clone
                    .send(format!("ERROR: Failed to save Osmo image: {}", e))
                    .unwrap_or_default(),
            }
        });
    }

    pub fn load_single_submap(&mut self, ctx: &egui::Context, submap_path_str: &str) -> Result<()> {
        let path = std::path::PathBuf::from(submap_path_str);
        let info_path = path.join("info.yaml");
        let points_path = path.join("points.txt");

        if !info_path.exists() || !points_path.exists() {
            return Err(anyhow::anyhow!(
                "Submap info.yaml or points.txt not found in: {}",
                path.display()
            ));
        }

        // メタデータを読み込み
        let info_file = fs::File::open(info_path)?;
        let submap_info: Submap = serde_yaml::from_reader(info_file)?;

        // Isometry2を復元
        let global_pose = Isometry2::new(
            nalgebra::Vector2::new(submap_info.pose_x, submap_info.pose_y),
            submap_info.pose_theta,
        );

        // 点群データを読み込み
        let points_data = fs::read_to_string(points_path)?;
        let submap_points_local: Vec<Point2<f32>> = points_data
            .lines()
            .filter_map(|line| {
                let mut parts = line.split_whitespace();
                let x = parts.next()?.parse::<f32>().ok()?;
                let y = parts.next()?.parse::<f32>().ok()?;
                Some(Point2::new(x, y))
            })
            .collect();

        // ワールド座標に変換
        let transformed_points: Vec<Point2<f32>> = submap_points_local
            .into_iter()
            .map(|p| global_pose * p)
            .collect();

        // 描画用のcurrent_map_pointsに現在のサブマップの点群を追加
        self.current_map_points.extend(
            transformed_points.into_iter().map(|p| (p, 1.0)), // 占有確率1.0として追加
        );

        // サブマップリストと軌跡リストに追加
        self.submaps.insert(submap_info.id, submap_info.clone());
        self.robot_trajectory.push((
            egui::pos2(global_pose.translation.x, global_pose.translation.y),
            global_pose.rotation.angle(),
        ));

        // UIの更新をリクエスト
        ctx.request_repaint();

        Ok(())
    }

    fn compute_features(
        &self,
        scan: &Vec<(f32, f32, f32, f32, f32, f32, f32)>,
    ) -> Vec<(f32, f32, f32, f32, f32, f32, f32)> {
        let mut scan_with_features = scan.clone();
        let neighborhood_size = 5; // 片側5点、合計11点を近傍とする
        if scan.len() < (neighborhood_size * 2 + 1) {
            return scan_with_features;
        }

        for i in neighborhood_size..(scan.len() - neighborhood_size) {
            // 1. 近傍点を収集 (iと、その前後5点ずつ)
            let neighborhood: Vec<_> = (i - neighborhood_size..=i + neighborhood_size)
                .map(|j| nalgebra::Point2::new(scan[j].0, scan[j].1))
                .collect();

            // 2. 重心を計算
            let sum_vec: nalgebra::Vector2<f32> = neighborhood.iter().map(|p| p.coords).sum();
            let mean = nalgebra::Point2::from(sum_vec / (neighborhood.len() as f32));

            // 3. 共分散行列を計算
            let mut covariance_matrix = nalgebra::Matrix2::<f32>::zeros();
            for point in &neighborhood {
                let centered_point = point - mean;
                covariance_matrix += centered_point * centered_point.transpose();
            }
            covariance_matrix /= neighborhood.len() as f32;

            // 4. 固有値と固有ベクトルを計算
            let eigen = nalgebra::SymmetricEigen::new(covariance_matrix);
            let eigenvalues = eigen.eigenvalues;
            let eigenvectors = eigen.eigenvectors;

            // 固有値をソートして、どちらが大きいか小さいか判断
            let (lambda_1, lambda_2, normal_vector) = if eigenvalues[0] > eigenvalues[1] {
                (eigenvalues[0], eigenvalues[1], eigenvectors.column(1))
            } else {
                (eigenvalues[1], eigenvalues[0], eigenvectors.column(0))
            };

            // ここに法線ベクトルの向きを修正するロジックを挿入
            // 点 (px, py) と法線ベクトル (nx, ny) の内積を計算し、Lidar側を向くように調整
            let px = scan[i].0;
            let py = scan[i].1;
            let nx = normal_vector[0];
            let ny = normal_vector[1];

            // Lidarの原点 (0,0) から点 (px, py) へのベクトルと法線ベクトルの内積を計算
            // 内積が負の場合、法線ベクトルはLidarと反対方向を向いているため反転させる
            let dot_product = px * nx + py * ny;

            let (corrected_nx, corrected_ny) = if dot_product < 0.0 {
                (-nx, -ny)
            } else {
                (nx, ny)
            };

            // 5. 特徴量（直線らしさ）を計算
            let linearity = if lambda_1 > 1e-9 {
                (lambda_1 - lambda_2) / lambda_1
            } else {
                0.0
            };

            // 6. シグモイド関数で平滑化し、「エッジらしさ」を計算
            let sharpness = 10.0;
            let sensitivity = 0.7;
            let edge_ness = 1.0 - (1.0 / (1.0 + (-sharpness * (linearity - sensitivity)).exp()));

            // 結果を格納
            scan_with_features[i].4 = edge_ness;
            scan_with_features[i].5 = corrected_nx; // 修正後の法線ベクトルを格納
            scan_with_features[i].6 = corrected_ny; // 修正後の法線ベクトルを格納
        }

        scan_with_features
    }

    /// 隣接点間距離が一定以上離れている場合に、線形補間して点を追加する
    /// scan: 各点の (x, y, r, theta)
    /// angle_threshold: 補間を開始する角度差の閾値 (ラジアン)
    /// interpolation_interval_angle: 補間された点の角度間隔 (ラジアン)
    fn interpolate_lidar_scan(
        &self,
        scan: &Vec<(f32, f32, f32, f32, f32, f32, f32)>,
        min_dist_threshold: f32,
        max_dist_threshold: f32,
        interpolation_interval: f32,
    ) -> Vec<(f32, f32, f32, f32, f32, f32, f32)> {
        if scan.is_empty() {
            return Vec::new();
        }

        // 閾値を2乗しておくことで、ループ内のsqrt()計算を削減
        let min_dist_threshold_sq = min_dist_threshold * min_dist_threshold;
        let max_dist_threshold_sq = max_dist_threshold * max_dist_threshold;

        // ステップ1: 間引き
        let mut thinned_scan = Vec::new();
        thinned_scan.push(scan[0]);
        let mut last_point_thinned = scan[0]; // 最後に追加した間引き点

        for i in 1..scan.len() {
            let current_point = scan[i];
            let dx = current_point.0 - last_point_thinned.0;
            let dy = current_point.1 - last_point_thinned.1;
            let distance_xy_sq = dx * dx + dy * dy;

            if distance_xy_sq >= min_dist_threshold_sq {
                thinned_scan.push(current_point);
                last_point_thinned = current_point;
            }
        }

        // ステップ2: 補間
        let mut final_scan = Vec::new();
        if thinned_scan.is_empty() {
            return final_scan;
        }

        // 最初の間引き点を追加
        final_scan.push(thinned_scan[0]);

        // .windows(2) を使う
        for window in thinned_scan.windows(2) {
            let p1 = window[0];
            let p2 = window[1];

            let dx = p2.0 - p1.0;
            let dy = p2.1 - p1.1;
            let distance_xy_sq = dx * dx + dy * dy;

            // 補間条件をチェック (2乗で比較)
            if distance_xy_sq < max_dist_threshold_sq {
                let distance_xy = distance_xy_sq.sqrt(); // ここで初めて平方根を計算
                let num_steps = (distance_xy / interpolation_interval).floor() as usize;
                if num_steps > 0 {
                    for step in 1..=num_steps {
                        let fraction = step as f32 * interpolation_interval / distance_xy;
                        let interpolated_x = p1.0 + dx * fraction;
                        let interpolated_y = p1.1 + dy * fraction;
                        let interpolated_r = (interpolated_x * interpolated_x
                            + interpolated_y * interpolated_y)
                            .sqrt();
                        let interpolated_theta = interpolated_y.atan2(interpolated_x);
                        final_scan.push((
                            interpolated_x,
                            interpolated_y,
                            interpolated_r,
                            interpolated_theta,
                            0.0, // edge_ness
                            0.0, // nx
                            0.0, // ny
                        ));
                    }
                }
            }

            // p2 を追加 (thinned_scan の各点を最終結果に追加)
            final_scan.push(p2);
        }

        final_scan
    }

    /// Updates the suggestion list based on the current input string.
    fn update_suggestions(&mut self) {
        let input = self.input_string.trim_start();
        self.current_suggestions.clear(); // 毎回クリアしてから再計算
        self.suggestion_selection_index = None; // 選択もリセット

        let all_commands = vec![
            "help",
            "lidar",
            "slam",
            "demo",
            "osmo",
            "set",
            "serial",
            "debug-storage",
            "version",
            "quit",
            "clear",
            "save",
            "map",
            "fkeys",
        ];

        if input.is_empty() {
            self.current_suggestions = all_commands.into_iter().map(String::from).collect();
        } else {
            let parts: Vec<&str> = input.split_whitespace().collect();
            let ends_with_space = input.ends_with(' ');

            match parts.as_slice() {
                ["demo"] if ends_with_space => {
                    self.current_suggestions = vec![
                        "scan".to_string(),
                        "ripple".to_string(),
                        "breathing".to_string(),
                        "table".to_string(),
                    ];
                }
                ["demo", partial_arg] => {
                    let options = vec!["scan", "ripple", "breathing", "table"];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|opt| opt.starts_with(partial_arg))
                        .map(|s| s.to_string())
                        .collect();
                }
                ["lidar", "set", "path", _id_str] if ends_with_space => {
                    self.current_suggestions = vec!["<path>".to_string()];
                }
                ["lidar", "set", "path"] if ends_with_space => {
                    self.current_suggestions = vec!["<id>".to_string()];
                }
                ["lidar", "set", partial_subcommand] => {
                    let options = vec!["path"];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|opt| opt.starts_with(partial_subcommand))
                        .map(|s| s.to_string())
                        .collect();
                }
                ["lidar", "set"] if ends_with_space => {
                    self.current_suggestions = vec!["path".to_string()];
                }
                ["lidar", "slam-toggle"] if ends_with_space => {
                    self.current_suggestions = vec!["<id>".to_string()];
                }
                ["lidar", "slam-toggle", _id_str] if ends_with_space => {
                    self.current_suggestions.clear();
                }
                ["lidar", partial_subcommand] => {
                    let options = vec!["entermode", "mode", "set", "slam-toggle", "analyze"];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|opt| opt.starts_with(partial_subcommand))
                        .map(|s| s.to_string())
                        .collect();
                }
                ["lidar"] if ends_with_space => {
                    self.current_suggestions = vec![
                        "entermode".to_string(),
                        "mode".to_string(),
                        "set".to_string(),
                        "slam-toggle".to_string(),
                        "analyze".to_string(),
                    ];
                }
                ["osmo"] if ends_with_space => {
                    self.current_suggestions = vec!["capture".to_string()];
                }
                ["osmo", partial_subcommand] => {
                    let options = vec!["capture"];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|opt| opt.starts_with(partial_subcommand))
                        .map(|s| s.to_string())
                        .collect();
                }
                ["slam"] if ends_with_space => {
                    self.current_suggestions = vec![
                        "getlidar".to_string(),
                        "continuous".to_string(),
                        "pause".to_string(),
                    ];
                }
                ["slam", partial_subcommand] => {
                    let options = vec!["getlidar", "continuous", "pause"];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|opt| opt.starts_with(partial_subcommand))
                        .map(|s| s.to_string())
                        .collect();
                }

                ["map" | "m", "load"] if ends_with_space => {
                    // map load の場合、デフォルトで ./slam_results/ をサジェスト
                    self.current_suggestions = vec!["./slam_results/".to_string()];
                }
                ["map" | "m", "load", partial_path] => {
                    let path_buf = PathBuf::from(partial_path);
                    let (dir_to_read, prefix) = if partial_path.ends_with('/')
                        || (path_buf.is_dir() && path_buf.exists())
                    {
                        (path_buf, "".to_string())
                    } else {
                        let parent = path_buf.parent().unwrap_or(Path::new(""));
                        let file_name = path_buf
                            .file_name()
                            .unwrap_or_default()
                            .to_string_lossy()
                            .to_string();
                        (parent.to_path_buf(), file_name)
                    };
                    self.current_suggestions =
                        get_path_suggestions(&dir_to_read.to_string_lossy(), &prefix);
                }
                ["map" | "m", partial_arg] => {
                    let options = vec!["load"];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|opt| opt.starts_with(partial_arg))
                        .map(|s| s.to_string())
                        .collect();
                }
                ["map" | "m"] if ends_with_space => {
                    self.current_suggestions = vec!["load".to_string()];
                }

                ["serial", _sub @ ("list" | "ls"), partial_arg] => {
                    let options = vec!["--detail", "-d"];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|opt| opt.starts_with(partial_arg))
                        .map(|s| s.to_string())
                        .collect();
                }
                ["serial", _sub @ ("list" | "ls")] if ends_with_space => {
                    self.current_suggestions = vec![
                        "--detail".to_string(),
                        "-d".to_string(),
                        "<path>".to_string(),
                    ];
                }
                ["serial", partial_subcommand] => {
                    let options = vec![("list", "list (ls)")];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|(cmd, _)| {
                            cmd.starts_with(partial_subcommand)
                                || "ls".starts_with(partial_subcommand)
                        })
                        .map(|(cmd, _)| cmd.to_string())
                        .collect();
                }
                ["serial"] if ends_with_space => {
                    self.current_suggestions = vec!["list".to_string()];
                }
                ["save", _sub @ ("points" | "p"), partial_arg] => {
                    let options = vec!["--output", "-o"];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|opt| opt.starts_with(partial_arg))
                        .map(|s| s.to_string())
                        .collect();
                }
                ["save", _sub @ ("points" | "p")] if ends_with_space => {
                    self.current_suggestions = vec![
                        "--output".to_string(),
                        "-o".to_string(),
                        "<file path>".to_string(),
                    ];
                }
                ["save", partial_subcommand] => {
                    let options = vec![("image", "image (i)"), ("points", "points (p)")];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|(cmd, _)| {
                            cmd.starts_with(partial_subcommand)
                                || "i".starts_with(partial_subcommand)
                                || "p".starts_with(partial_subcommand)
                        })
                        .map(|(cmd, _)| cmd.to_string())
                        .collect();
                }
                ["save"] if ends_with_space => {
                    self.current_suggestions = vec!["image".to_string(), "points".to_string()];
                }
                [partial_command] => {
                    self.current_suggestions = all_commands
                        .into_iter()
                        .filter(|cmd| cmd.starts_with(partial_command))
                        .map(|s| s.to_string())
                        .collect();
                }
                _ => {}
            }
        }
    }

    fn navigate_suggestions(&mut self, direction: SuggestionNavigationDirection) {
        // If input is empty and suggestions are not shown, populate them first.
        if self.input_string.is_empty() && self.current_suggestions.is_empty() {
            self.update_suggestions();
        }

        if self.show_command_window && !self.current_suggestions.is_empty() {
            let num_suggestions = self.current_suggestions.len();
            if num_suggestions > 0 {
                let mut current_index = self.suggestion_selection_index.unwrap_or(usize::MAX);
                match direction {
                    SuggestionNavigationDirection::Down => {
                        current_index = if current_index == usize::MAX {
                            0
                        } else {
                            (current_index + 1) % num_suggestions
                        };
                    }
                    SuggestionNavigationDirection::Up => {
                        current_index = if current_index == usize::MAX || current_index == 0 {
                            num_suggestions - 1
                        } else {
                            current_index - 1
                        };
                    }
                }
                self.suggestion_selection_index = Some(current_index);
            }
        }
    }
}

impl eframe::App for MyApp {
    /// Called to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        for lidar_state in &self.lidars {
            let key = format!("lidar_path_{}", lidar_state.id);
            storage.set_string(&key, lidar_state.path.clone());
        }
    }

    /// フレームごとに呼ばれ、UIを描画する
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // --- Trigger XPPen connection on first update ---
        if self.xppen.connection_status == "Disconnected".to_string() {
            if let Err(e) = self.xppen_trigger_sender.send(()) {
                self.command_output_sender
                    .send(format!("ERROR: Failed to trigger XPPen thread: {}", e))
                    .unwrap_or_default();
            } else {
                self.xppen.connection_status = "Connecting...".to_string();
            }
        }

        // --- サブマップの逐次読み込み処理 ---
        let mut path_to_load: Option<PathBuf> = None; // ロードするパスを保持する変数

        if let Some(queue) = &mut self.submap_load_queue {
            let should_load = self.last_submap_load_time.map_or(true, |last_load| {
                last_load.elapsed() >= Duration::from_millis(self.config.map.submap_load_delay_ms)
            });

            if should_load && !queue.is_empty() {
                path_to_load = Some(queue.remove(0)); // パスを取り出して変数に格納
                self.last_submap_load_time = Some(Instant::now());
            }
        }

        if let Some(submap_path) = path_to_load {
            let submap_path_str = submap_path.to_string_lossy().into_owned();

            // どのくらい進んだかのメッセージはここで生成
            /*
            if let Some(queue) = &self.submap_load_queue {
                let total_count = self.submaps.len() + queue.len() + 1;
                let current_count = self.submaps.len() + 1;
                let msg = format!(
                    "Loading submap {}/{} from '{}'...",
                    current_count,
                    total_count,
                    submap_path.display()
                );
                self.command_history.push(ConsoleOutputEntry {
                    text: msg,
                    group_id: self.next_group_id,
                });
            }
            */

            match self.load_single_submap(ctx, &submap_path_str) {
                Ok(_) => {
                    // バウンディングボックスを更新
                    if !self.current_map_points.is_empty() {
                        let egui_points: Vec<egui::Pos2> = self
                            .current_map_points
                            .iter()
                            .map(|(p, _prob)| egui::pos2(p.x, p.y))
                            .collect();
                        self.slam_map_bounding_box = Some(egui::Rect::from_points(&egui_points));
                    }
                }
                Err(e) => {
                    let error_msg = format!(
                        "ERROR: Failed to load submap '{}': {}",
                        submap_path.display(),
                        e
                    );
                    self.command_history.push(ConsoleOutputEntry {
                        text: error_msg,
                        group_id: self.next_group_id,
                    });
                }
            }

            // キューが空になったかチェック
            if let Some(queue) = &mut self.submap_load_queue {
                if queue.is_empty() {
                    self.submap_load_queue = None; // キューが空になったらNoneにする
                    self.command_history.push(ConsoleOutputEntry {
                        text: "All submap loading attempts completed.".to_string(),
                        group_id: self.next_group_id,
                    });
                }
            }
        }

        // --- データ更新 ---
        while let Ok(xppen_message) = self.xppen_message_receiver.try_recv() {
            match xppen_message {
                XppenMessage::ToggleF1 => {
                    self.navigate_suggestions(SuggestionNavigationDirection::Up)
                }
                XppenMessage::ToggleF2 => {
                    self.navigate_suggestions(SuggestionNavigationDirection::Down)
                }
                XppenMessage::ToggleF3 => {
                    self.command_history.push(ConsoleOutputEntry {
                        text: "F3キーが押されました！".to_string(),
                        group_id: self.next_group_id,
                    });
                }
                XppenMessage::ToggleF4 => {
                    self.command_history.push(ConsoleOutputEntry {
                        text: "F4キーが押されました！".to_string(),
                        group_id: self.next_group_id,
                    });
                }
                XppenMessage::ToggleF5 => {
                    self.command_history.push(ConsoleOutputEntry {
                        text: "F5キーが押されました！".to_string(),
                        group_id: self.next_group_id,
                    });
                }
                XppenMessage::ToggleF6 => {
                    self.suggestion_completion_requested = true;
                }
                XppenMessage::ToggleF7 => {
                    self.command_history.push(ConsoleOutputEntry {
                        text: "F7キーが押されました！".to_string(),
                        group_id: self.next_group_id,
                    });
                }
                XppenMessage::ToggleF8 => {
                    self.command_history.push(ConsoleOutputEntry {
                        text: "F8キーが押されました！".to_string(),
                        group_id: self.next_group_id,
                    });
                }
                XppenMessage::ToggleF9 => {
                    self.input_string.clear();
                    self.update_suggestions();
                    self.suggestion_selection_index = None;
                    self.command_history.push(ConsoleOutputEntry {
                        text: "Clear Input (F9)".to_string(),
                        group_id: self.next_group_id,
                    });
                }
                XppenMessage::ToggleF10 => {
                    self.clear_command_requested = true;
                    self.command_history.push(ConsoleOutputEntry {
                        text: "Clear History (F10)".to_string(),
                        group_id: self.next_group_id,
                    });
                }
                XppenMessage::ToggleF11 => {
                    self.command_submission_requested = true;
                }
                XppenMessage::ToggleF12 => {
                    self.show_command_window = !self.show_command_window;
                }
            }
            ctx.request_repaint();
        }

        // XPPenからのステータスメッセージを処理
        while let Ok(status_message) = self.xppen_status_receiver.try_recv() {
            self.xppen.connection_status = status_message;
            ctx.request_repaint();
        }

        while let Ok(lidar_message) = self.lidar_message_receiver.try_recv() {
            match lidar_message {
                LidarMessage::ScanUpdate { id, scan } => {
                    let mut filtered_scan: Vec<(f32, f32, f32, f32, f32, f32, f32)> = Vec::new();
                    // 該当するLidarStateからフィルタリング角度を取得
                    let (min_angle_rad, max_angle_rad) =
                        if let Some(lidar_state_for_filter) = self.lidars.get(id) {
                            (
                                lidar_state_for_filter.data_filter_angle_min.to_radians(),
                                lidar_state_for_filter.data_filter_angle_max.to_radians(),
                            )
                        } else {
                            // LidarStateが見つからない場合はフィルタリングしない (またはエラー処理)
                            (f32::NEG_INFINITY, f32::INFINITY)
                        };

                    for &(x, y, r, theta, feature, nx, ny) in &scan {
                        // フィルタリング範囲内にあるかチェック (角度は受信データから直接利用)
                        if theta >= min_angle_rad && theta <= max_angle_rad {
                            filtered_scan.push((x, y, r, theta, feature, nx, ny));
                        }
                    }

                    // フィルタリング後のスキャンに対して特徴量を計算
                    let scan_with_features = self.compute_features(&filtered_scan);

                    // UI用に点群を更新 (特徴量計算済み)
                    if let Some(lidar_state) = self.lidars.get_mut(id) {
                        lidar_state.points = scan_with_features.clone();
                    }

                    // SLAMモードの場合、スキャンデータを統合する (特徴量計算済み)
                    if self.app_mode == AppMode::Slam {
                        if id < self.pending_scans.len() {
                            self.pending_scans[id] = Some(scan_with_features);
                        }

                        // SLAMが有効なLidarがすべてデータを受信したか確認
                        let active_lidars: Vec<_> = self
                            .lidars
                            .iter()
                            .filter(|l| l.is_active_for_slam)
                            .collect();

                        let all_active_scans_received = active_lidars.iter().all(|l| {
                            self.pending_scans
                                .get(l.id)
                                .and_then(|o| o.as_ref())
                                .is_some()
                        });

                        if all_active_scans_received {
                            let mut raw_combined_scan: Vec<(f32, f32, f32, f32, f32, f32, f32)> =
                                Vec::new();

                            // SLAMが有効な各Lidarのスキャンをロボット座標系に変換して結合
                            for lidar_state in active_lidars {
                                if let Some(Some(points)) = self.pending_scans.get(lidar_state.id) {
                                    let rotation = lidar_state.rotation;
                                    let origin = lidar_state.origin;

                                    for point in points {
                                        // Lidar座標系での点 (px, py, r, theta, feature)
                                        let px_raw = point.0;
                                        let py_raw = point.1;
                                        let r_val = point.2; // 距離
                                        let theta_val = point.3; // 角度
                                        let feature_val = point.4; // 特徴量
                                        let nx_raw = point.5;
                                        let ny_raw = point.6;

                                        // Lidarの回転を適用
                                        let px_rotated =
                                            px_raw * rotation.cos() - py_raw * rotation.sin();
                                        let py_rotated =
                                            px_raw * rotation.sin() + py_raw * rotation.cos();

                                        let nx_rotated =
                                            nx_raw * rotation.cos() - ny_raw * rotation.sin();
                                        let ny_rotated =
                                            nx_raw * rotation.sin() + ny_raw * rotation.cos();

                                        // ワールド座標に変換 (Lidarの原点オフセットを加える)
                                        let world_x = px_rotated + origin.x;
                                        let world_y = py_rotated + origin.y;

                                        raw_combined_scan.push((
                                            world_x,
                                            world_y,
                                            r_val,
                                            theta_val,
                                            feature_val,
                                            nx_rotated,
                                            ny_rotated,
                                        ));
                                    }
                                }
                            }

                            // 結合した生スキャンデータを補間
                            // FIXME: 閾値と間隔は調整が必要
                            let interpolated_combined_scan = self.interpolate_lidar_scan(
                                &raw_combined_scan,
                                0.1,  // min_dist_threshold (10cm)
                                2.0,  // max_dist_threshold (2m)
                                0.05, // interpolation_interval (5cm)
                            );

                            // 結合した点群をSLAMスレッドに送信
                            if !self.is_slam_processing.load(Ordering::SeqCst) {
                                let current_timestamp = SystemTime::now()
                                    .duration_since(SystemTime::UNIX_EPOCH)
                                    .expect("Time went backwards")
                                    .as_millis();

                                if self.slam_mode == SlamMode::Continuous {
                                    self.slam_command_sender
                                        .send(SlamThreadCommand::UpdateScan {
                                            raw_scan: raw_combined_scan,                   // raw_scanを送信
                                            interpolated_scan: interpolated_combined_scan, // 補間済みscanを送信
                                            timestamp: current_timestamp,
                                        })
                                        .unwrap_or_default();
                                } else if self.single_scan_requested_by_ui {
                                    self.slam_command_sender
                                        .send(SlamThreadCommand::ProcessSingleScan {
                                            raw_scan: raw_combined_scan,                   // raw_scanを送信
                                            interpolated_scan: interpolated_combined_scan, // 補間済みscanを送信
                                            timestamp: current_timestamp,
                                        })
                                        .unwrap_or_default();
                                    self.single_scan_requested_by_ui = false; // フラグをリセット
                                }
                            }

                            // 統合後は処理したLidarの保留中スキャンをクリア
                            for lidar_state in &self.lidars {
                                if lidar_state.is_active_for_slam {
                                    if let Some(scan_option) =
                                        self.pending_scans.get_mut(lidar_state.id)
                                    {
                                        *scan_option = None;
                                    }
                                }
                            }
                        }
                    }
                }
                LidarMessage::StatusUpdate { id, message } => {
                    if let Some(lidar_state) = self.lidars.get_mut(id) {
                        let mut handled_as_status = false;
                        if message.starts_with("ERROR: Failed to open port") {
                            lidar_state.connection_status = message.clone();
                            handled_as_status = true;
                        } else if message.starts_with("ERROR: Failed to get distance data") {
                            lidar_state.connection_status = "Connection Lost".to_string();
                        } else if message.contains("Successfully opened port") {
                            lidar_state.connection_status = "Connected".to_string();
                            handled_as_status = true;
                        } else if message.contains("INFO: LiDAR initialized. Laser is ON.") {
                            lidar_state.connection_status = "Connected (Laser ON)".to_string();
                            handled_as_status = true;
                        }

                        if !handled_as_status
                            && !message.starts_with("LiDAR Path:")
                            && !message.starts_with("Baud Rate:")
                        {
                            lidar_state.status_messages.push(message);
                            if lidar_state.status_messages.len() > 10 {
                                lidar_state.status_messages.remove(0);
                            }
                        }
                    } else {
                        // 不明なLidar IDからのメッセージはコンソールに出力
                        self.command_history.push(ConsoleOutputEntry {
                            text: format!(
                                "ERROR: Message from unknown Lidar ID {}: {}",
                                id, message
                            ),
                            group_id: self.next_group_id,
                        });
                    }
                }
            }
            ctx.request_repaint();
        }

        while let Ok(camera_message) = self.camera_message_receiver.try_recv() {
            match camera_message {
                CameraMessage::Frame { id, image } => {
                    if let Some(camera_state) = self.cameras.get_mut(id) {
                        if let Some(texture) = &mut camera_state.texture {
                            texture.set(image, egui::TextureOptions::LINEAR);
                        } else {
                            camera_state.texture = Some(ctx.load_texture(
                                format!("camera_{}", id),
                                image,
                                egui::TextureOptions::LINEAR,
                            ));
                        }
                    }
                }
                CameraMessage::Status { id, message } => {
                    if let Some(camera_state) = self.cameras.get_mut(id) {
                        camera_state.connection_status = message.clone();
                        self.command_history.push(ConsoleOutputEntry {
                            text: message,
                            group_id: self.next_group_id,
                        });
                    }
                }
            }
            ctx.request_repaint();
        }

        while let Ok(osmo_message) = self.osmo_message_receiver.try_recv() {
            match osmo_message {
                OsmoMessage::Frame { id, image } => {
                    if self.osmo.id == id {
                        self.osmo.latest_image = Some(image.clone());
                        if let Some(texture) = &mut self.osmo.texture {
                            texture.set(image, egui::TextureOptions::LINEAR);
                        } else {
                            self.osmo.texture = Some(ctx.load_texture(
                                format!("osmo_{}", id),
                                image,
                                egui::TextureOptions::LINEAR,
                            ));
                        }
                    }
                }
                OsmoMessage::Status { id, message } => {
                    if self.osmo.id == id {
                        self.osmo.connection_status = message.clone();
                        self.command_history.push(ConsoleOutputEntry {
                            text: message,
                            group_id: self.next_group_id,
                        });
                    }
                }
            }
            ctx.request_repaint();
        }

        // SLAMスレッドからの結果を受け取る
        while let Ok(slam_result) = self.slam_result_receiver.try_recv() {
            self.current_map_points = slam_result.map_points;
            self.current_robot_pose = slam_result.robot_pose;
            self.latest_scan_for_draw = slam_result.scan_used;

            // 軌跡に新しい位置と向きを追加
            let new_pos = egui::pos2(
                self.current_robot_pose.translation.x,
                self.current_robot_pose.translation.y,
            );
            let new_angle = self.current_robot_pose.rotation.angle();
            self.robot_trajectory.push((new_pos, new_angle));

            ctx.request_repaint();
        }
        while let Ok(msg) = self.command_output_receiver.try_recv() {
            println!("{}", msg); // ここで標準出力にメッセージを出力
            self.command_history.push(ConsoleOutputEntry {
                text: msg,
                group_id: self.next_group_id,
            });
            ctx.request_repaint();
        }

        // --- 点群データ保存リクエストの処理 ---
        if let Some(path) = self.requested_point_save_path.take() {
            // TODO: どのLidarの点群を保存するかIDで指定できるようにする
            // 現在は仮に最初のLidar(id=0)の点群を保存する
            if let Some(lidar_state) = self.lidars.get(0) {
                let lidar_points_clone = lidar_state.points.clone();
                let command_output_sender_clone = self.command_output_sender.clone();

                thread::spawn(move || {
                    let file_path = std::path::PathBuf::from(&path);
                    match std::fs::File::create(&file_path) {
                        Ok(mut file) => {
                            let mut content = String::new();
                            for point in lidar_points_clone {
                                content.push_str(&format!("{} {}\n", point.0, point.1));
                            }
                            match file.write_all(content.as_bytes()) {
                                Ok(_) => command_output_sender_clone
                                    .send(format!("Point cloud saved to '{}'.", path))
                                    .unwrap_or_default(),
                                Err(e) => command_output_sender_clone
                                    .send(format!(
                                        "ERROR: Failed to write to file '{}': {}",
                                        path, e
                                    ))
                                    .unwrap_or_default(),
                            }
                        }
                        Err(e) => {
                            command_output_sender_clone
                                .send(format!("ERROR: Failed to create file '{}': {}", path, e))
                                .unwrap_or_default();
                        }
                    }
                });
                ctx.request_repaint(); // 保存リクエスト処理のために再描画
            } else {
                // Lidarが設定されていない場合のメッセージ
                self.command_output_sender
                    .send("ERROR: No LiDAR 0 configured to save points from.".to_string())
                    .unwrap_or_default();
            }
        }

        let console_input_id = egui::Id::new("console_input");
        let enter_pressed_while_unfocused =
            ctx.input(|i| i.key_pressed(egui::Key::Enter)) && !ctx.wants_keyboard_input();

        // Enterキーでフローティングコンソールにフォーカスを当てる
        if self.show_command_window && enter_pressed_while_unfocused {
            let console_input_id_in_window = egui::Id::new("console_input_window"); // フローティングコンソールのテキスト入力ID
            ctx.memory_mut(|m| m.request_focus(console_input_id_in_window));
            ctx.request_repaint(); // フォーカス変更のために再描画
        }

        // Ctrl/Cmd + P でコマンドウィンドウの表示/非表示を切り替える
        if ctx.input(|i| i.modifiers.command && i.key_pressed(egui::Key::P)) {
            self.show_command_window = !self.show_command_window;
            // ウィンドウが表示されるときにフォーカスを要求
            if self.show_command_window {
                self.focus_console_requested = true;
            }
            ctx.request_repaint(); // 表示が変わるので再描画
        }

        if (ctx.input(|i| (i.modifiers.ctrl || i.modifiers.command) && i.key_pressed(egui::Key::L)))
            || self.clear_command_requested
        {
            self.command_history.clear();
            ctx.request_repaint();
            self.clear_command_requested = false; // Reset the flag
        }

        if self.show_command_window {
            egui::Window::new("Console")
                .default_pos(egui::pos2(20.0, 500.0)) // 初期位置
                .default_size(egui::vec2(
                    ctx.input(|i| i.screen_rect()).width() / 3.0,
                    400.0,
                )) // 初期サイズ
                .resizable(true)
                .collapsible(true)
                .show(ctx, |ui| {
                    let console_input_id = egui::Id::new("console_input_window");

                    // コマンド履歴
                    egui::ScrollArea::vertical()
                        .id_source("console_history_scroll") // ユニークなIDを追加
                        .stick_to_bottom(true)
                        .max_height(
                            ui.available_height()
                                - ui.text_style_height(&egui::TextStyle::Monospace) * 2.0
                                - 10.0,
                        ) // 入力欄とマージンを考慮
                        .show(ui, |ui| {
                            for line_entry in &self.command_history {
                                let is_current_group = line_entry.group_id == self.next_group_id;

                                let final_color = if line_entry.text.starts_with("> ") {
                                    egui::Color32::GRAY // 確定後のユーザー入力は常にGRAY
                                } else if line_entry.text.starts_with("ERROR:") {
                                    egui::Color32::RED // エラーは常にRED
                                } else {
                                    if is_current_group {
                                        egui::Color32::WHITE // 最新の通常の出力はWHITE
                                    } else {
                                        egui::Color32::DARK_GRAY // 過去の通常の出力はDARK_GRAY
                                    }
                                };
                                ui.monospace(
                                    egui::RichText::new(&line_entry.text).color(final_color),
                                );
                            }
                        });

                    // 入力欄
                    let text_edit_response = ui
                        .horizontal(|ui| {
                            ui.monospace(">");
                            ui.add(
                                egui::TextEdit::singleline(&mut self.input_string)
                                    .id(console_input_id)
                                    .frame(false)
                                    .hint_text("Enter command...")
                                    .font(egui::TextStyle::Monospace)
                                    .lock_focus(true)
                                    .desired_width(f32::INFINITY) // 幅を最大化
                                    .text_color(egui::Color32::LIGHT_GREEN), // ここを追加
                            )
                        })
                        .inner;

                    if self.focus_console_requested {
                        text_edit_response.request_focus();
                        self.focus_console_requested = false;
                    }

                    // --- サジェスト候補の生成 ---
                    if text_edit_response.changed()
                        || (text_edit_response.gained_focus() && self.input_string.is_empty())
                    {
                        self.update_suggestions();
                    }

                    // --- サジェスト候補の表示 ---
                    if !self.current_suggestions.is_empty() {
                        egui::Frame::none()
                            .fill(egui::Color32::from_rgb(40, 40, 40))
                            .inner_margin(egui::Margin::symmetric(5.0, 2.0))
                            .show(ui, |ui| {
                                ui.set_width(ui.available_width());
                                ui.label("Suggestions:");
                                ui.separator();
                                for (i, suggestion) in self.current_suggestions.iter().enumerate() {
                                    let is_selected = self.suggestion_selection_index == Some(i);
                                    let label = egui::SelectableLabel::new(
                                        is_selected,
                                        egui::RichText::new(suggestion).monospace(),
                                    );
                                    if ui.add(label).clicked() {
                                        self.suggestion_selection_index = Some(i);
                                        // Optional: complete on click
                                    }
                                }
                            });
                    }

                    // --- コマンド履歴ナビゲーション & サジェスト選択 ---
                    if text_edit_response.has_focus() {
                        let i = ctx.input(|i| i.clone()); // Capture input state

                        let ctrl_n_pressed = i.modifiers.ctrl && i.key_pressed(egui::Key::N);
                        let ctrl_p_pressed = i.modifiers.ctrl && i.key_pressed(egui::Key::P);
                        let up_pressed = i.key_pressed(egui::Key::ArrowUp);
                        let down_pressed = i.key_pressed(egui::Key::ArrowDown);
                        let tab_pressed = i.key_pressed(egui::Key::Tab);

                        // Suggestion navigation (Ctrl+n/p)
                        if ctrl_n_pressed {
                            self.navigate_suggestions(SuggestionNavigationDirection::Down);
                            ctx.input_mut(|i| i.consume_key(egui::Modifiers::CTRL, egui::Key::N));
                        }
                        if ctrl_p_pressed {
                            self.navigate_suggestions(SuggestionNavigationDirection::Up);
                            ctx.input_mut(|i| i.consume_key(egui::Modifiers::CTRL, egui::Key::P));
                        }

                        // Tab key handling (completion) by Tab or F6
                        if tab_pressed || self.suggestion_completion_requested {
                            if !self.current_suggestions.is_empty()
                                && self.suggestion_selection_index.is_some()
                            {
                                // Case 1: Suggestion selected. COMPLETE it.
                                if let Some(selected_index) = self.suggestion_selection_index {
                                    if let Some(selected_suggestion) =
                                        self.current_suggestions.get(selected_index).cloned()
                                    {
                                        let mut completed_text = selected_suggestion;
                                        // 補完候補がディレクトリ（末尾が /）でないならスペースを追加
                                        if !completed_text.ends_with('/') {
                                            completed_text.push(' ');
                                        }

                                        let last_space_idx = self
                                            .input_string
                                            .rfind(char::is_whitespace)
                                            .map(|i| i + 1)
                                            .unwrap_or(0);
                                        let last_slash_idx = self
                                            .input_string
                                            .rfind('/')
                                            .map(|i| i + 1)
                                            .unwrap_or(0);
                                        let replace_from_idx =
                                            std::cmp::max(last_space_idx, last_slash_idx);

                                        self.input_string = format!(
                                            "{}{}",
                                            &self.input_string[..replace_from_idx],
                                            completed_text
                                        );

                                        let id = text_edit_response.id;
                                        if let Some(mut state) =
                                            egui::widgets::text_edit::TextEditState::load(ctx, id)
                                        {
                                            let new_cursor_pos = self.input_string.len();
                                            state.set_ccursor_range(Some(
                                                egui::widgets::text_edit::CCursorRange::one(
                                                    egui::text::CCursor::new(new_cursor_pos),
                                                ),
                                            ));
                                            state.store(ctx, id);
                                        }
                                        self.update_suggestions();
                                        self.suggestion_selection_index = None;
                                        if tab_pressed {
                                            ctx.input_mut(|i| {
                                                i.consume_key(egui::Modifiers::NONE, egui::Key::Tab)
                                            });
                                        }
                                    }
                                }
                            }
                            // Reset the flag since the action has been handled
                            self.suggestion_completion_requested = false;
                        }

                        // History navigation (ArrowUp/Down) - separate from suggestions
                        if up_pressed {
                            if self.history_index > 0 {
                                self.history_index -= 1;
                                self.input_string = self
                                    .user_command_history
                                    .get(self.history_index)
                                    .cloned()
                                    .unwrap_or_default();
                            }
                            ctx.input_mut(|i| {
                                i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp)
                            });
                        } else if down_pressed {
                            if self.history_index < self.user_command_history.len() {
                                self.history_index += 1;
                                if self.history_index == self.user_command_history.len() {
                                    self.input_string.clear();
                                } else {
                                    self.input_string = self
                                        .user_command_history
                                        .get(self.history_index)
                                        .cloned()
                                        .unwrap_or_default();
                                }
                            }
                            ctx.input_mut(|i| {
                                i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown)
                            });
                        }
                    }

                    // Enter key handling (submission) - must be outside the `has_focus` block
                    let enter_pressed = text_edit_response.lost_focus()
                        && ctx.input(|i| i.key_pressed(egui::Key::Enter));
                    if enter_pressed || self.command_submission_requested {
                        let full_command_line_owned = self.input_string.trim().to_owned();
                        if !full_command_line_owned.is_empty() {
                            self.command_history.push(ConsoleOutputEntry {
                                text: format!("> {}", full_command_line_owned),
                                group_id: self.next_group_id,
                            });
                            self.user_command_history
                                .push(full_command_line_owned.clone());
                            self.history_index = self.user_command_history.len();
                            match shlex::split(&full_command_line_owned) {
                                Some(args) => match Cli::try_parse_from(args.into_iter()) {
                                    Ok(cli_command) => {
                                        crate::cli::handle_command(self, ctx, cli_command);
                                    }
                                    Err(e) => {
                                        for line in e.to_string().lines() {
                                            self.command_history.push(ConsoleOutputEntry {
                                                text: line.to_string(),
                                                group_id: self.next_group_id,
                                            });
                                        }
                                    }
                                },
                                None => {
                                    self.command_history.push(ConsoleOutputEntry {
                                        text: "ERROR: Failed to parse command line.".to_string(),
                                        group_id: self.next_group_id,
                                    });
                                }
                            }
                        }
                        self.input_string.clear();
                        self.current_suggestions.clear();
                        self.suggestion_selection_index = None;
                        text_edit_response.request_focus();
                        // Reset the F11 flag
                        self.command_submission_requested = false;
                    }
                });
        }

        // 1. 左側のパネル（コンソール）
        egui::SidePanel::left("terminal")
            .exact_width(ctx.input(|i| i.screen_rect()).width() / 3.0)
            .resizable(true)
            .min_width(150.0)
            .show(ctx, |ui| {
                let background_response = ui.interact(
                    ui.available_rect_before_wrap(),
                    ui.id().with("terminal_background"),
                    egui::Sense::click(),
                );
                if background_response.clicked() {
                    ctx.memory_mut(|m| m.request_focus(console_input_id));
                }
                ui.heading("Console");

                // Osmo Status
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.label(format!("[OSMO {}]", self.osmo.id));

                    // ライブ映像の表示
                    if let Some(texture) = &self.osmo.texture {
                        let image_size = texture.size_vec2();
                        if image_size.x > 0.0 && image_size.y > 0.0 {
                            let available_width = ui.available_width();
                            let image_aspect = image_size.x / image_size.y;
                            let new_height = available_width / image_aspect;
                            let new_size = egui::vec2(available_width, new_height);
                            ui.add(egui::Image::new(texture).fit_to_exact_size(new_size));
                        }
                    }

                    ui.label(format!("  Name: {}", self.osmo.name));
                    let status_text = format!("  Status: {}", self.osmo.connection_status);
                    let status_color = if self.osmo.connection_status.contains("successfully") {
                        egui::Color32::GREEN
                    } else if self.osmo.connection_status.contains("Spawning") {
                        egui::Color32::YELLOW
                    } else if self.osmo.connection_status.contains("ERROR") {
                        egui::Color32::RED
                    } else {
                        egui::Color32::GRAY
                    };
                    ui.label(egui::RichText::new(status_text).color(status_color));
                });
                ui.separator();

                // XPPen Status
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.label("[XPPen Shortcut Device]");
                    let status_text = format!("  Status: {}", self.xppen.connection_status);
                    let status_color = if self.xppen.connection_status.contains("connected") {
                        egui::Color32::GREEN
                    } else if self.xppen.connection_status.contains("Connecting") {
                        egui::Color32::YELLOW
                    } else if self.xppen.connection_status.contains("Failed")
                        || self.xppen.connection_status.contains("ERROR")
                        || self.xppen.connection_status.contains("exiting")
                    {
                        egui::Color32::RED
                    } else {
                        egui::Color32::GRAY
                    };
                    ui.label(egui::RichText::new(status_text).color(status_color));
                });
                ui.separator();

                for camera_state in &self.cameras {
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        ui.label(format!("[CAMERA {}]", camera_state.id));
                        ui.label(format!("  Name: {}", camera_state.name));
                        let status_text = format!("  Status: {}", camera_state.connection_status);
                        let status_color = if camera_state.connection_status.contains("Connected") {
                            egui::Color32::GREEN
                        } else if camera_state.connection_status.contains("Trying") {
                            egui::Color32::YELLOW
                        } else {
                            egui::Color32::RED
                        };
                        ui.label(egui::RichText::new(status_text).color(status_color));
                    });
                }
                ui.separator();

                for lidar_state in &self.lidars {
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        ui.label(format!("[LIDAR {}]", lidar_state.id));
                        ui.label(format!("  Path: {}", lidar_state.path));
                        ui.label(format!("  Baud: {}", lidar_state.baud_rate));
                        let status_text = format!("  Status: {}", lidar_state.connection_status);
                        let status_color = if lidar_state.connection_status.starts_with("Connected")
                        {
                            egui::Color32::GREEN
                        } else if lidar_state.connection_status.starts_with("Connecting") {
                            egui::Color32::YELLOW
                        } else {
                            egui::Color32::RED
                        };
                        ui.label(egui::RichText::new(status_text).color(status_color));
                        let slam_status_text = if lidar_state.is_active_for_slam {
                            "  SLAM: Active"
                        } else {
                            "  SLAM: Inactive"
                        };
                        let slam_status_color = if lidar_state.is_active_for_slam {
                            egui::Color32::GREEN
                        } else {
                            egui::Color32::GRAY
                        };
                        ui.label(egui::RichText::new(slam_status_text).color(slam_status_color));
                    });

                    ui.group(|ui| {
                        ui.set_height(ui.text_style_height(&egui::TextStyle::Monospace) * 5.0); // 高さを調整
                        egui::ScrollArea::vertical()
                            .id_source(format!("lidar_status_scroll_{}", lidar_state.id))
                            .show(ui, |ui| {
                                for line in &lidar_state.status_messages {
                                    ui.monospace(line);
                                }
                            });
                    });
                    ui.separator();
                }
                ui.separator();
            });

        // 2. 右側のパネル（グラフィック表示）
        egui::CentralPanel::default().show(ctx, |ui| match self.app_mode {
            AppMode::Lidar => {
                ui.heading("LiDAR Data Visualization");
                let (response, painter) =
                    ui.allocate_painter(ui.available_size(), egui::Sense::hover());
                let rect = response.rect;
                self.lidar_draw_rect = Some(rect);

                // --- 全Lidar共通の描画設定 ---
                let side = rect.height();
                let square_rect =
                    egui::Rect::from_center_size(rect.center(), egui::vec2(side, side));

                let half_view_size = self.config.slam.online_slam_view_size / 2.0;
                // ワールド座標からスクリーン座標への変換を定義
                // Y軸を反転させるため、fromに渡すRectのYのmin/maxを入れ替える
                let world_to_screen_rect = egui::Rect::from_min_max(
                    egui::pos2(-half_view_size, half_view_size), // ワールドの左上 (min_x, max_y)
                    egui::pos2(half_view_size, -half_view_size), // ワールドの右下 (max_x, min_y)
                );
                let to_screen =
                    egui::emath::RectTransform::from_to(world_to_screen_rect, square_rect);

                // 背景とグリッドを描画
                painter.rect_filled(square_rect, 0.0, egui::Color32::from_rgb(20, 20, 20));
                painter.hline(
                    square_rect.x_range(),
                    to_screen.transform_pos(egui::pos2(0.0, 0.0)).y,
                    egui::Stroke::new(0.5, egui::Color32::DARK_GRAY),
                );
                painter.vline(
                    to_screen.transform_pos(egui::pos2(0.0, 0.0)).x,
                    square_rect.y_range(),
                    egui::Stroke::new(0.5, egui::Color32::DARK_GRAY),
                );

                let mut any_lidar_connected = false;

                // --- 各Lidarのデータを描画 ---
                for (i, lidar_state) in self.lidars.iter().enumerate() {
                    if lidar_state.connection_status.starts_with("Connected") {
                        any_lidar_connected = true;

                        let lidar_origin_world = lidar_state.origin;
                        let lidar_color = match i {
                            0 => egui::Color32::GREEN,
                            1 => egui::Color32::YELLOW,
                            2 => egui::Color32::BLUE,
                            _ => egui::Color32::WHITE,
                        };

                        // Lidarの原点を描画
                        let lidar_origin_screen =
                            to_screen.transform_pos(lidar_origin_world.to_pos2());
                        painter.circle_filled(lidar_origin_screen, 5.0, lidar_color);
                        painter.text(
                            lidar_origin_screen + egui::vec2(10.0, -10.0),
                            egui::Align2::LEFT_BOTTOM,
                            format!("LIDAR {}", lidar_state.id),
                            egui::FontId::default(),
                            egui::Color32::WHITE,
                        );

                        // Lidarの点群を描画
                        for point in &lidar_state.points {
                            // Lidar座標系での点 (px, py)
                            let pxx = point.0;
                            let pyy = point.1;
                            let px_raw = pxx;
                            let py_raw = pyy;
                            //let px_raw = -pyy;
                            //let py_raw = -pxx;

                            // Lidarの回転を適用
                            let rotation = lidar_state.rotation;
                            let px_rotated = px_raw * rotation.cos() - py_raw * rotation.sin();
                            let py_rotated = px_raw * rotation.sin() + py_raw * rotation.cos();

                            // ワールド座標に変換 (Lidarの原点オフセットを加える)
                            let world_pos = egui::pos2(px_rotated, py_rotated) + lidar_origin_world;

                            // スクリーン座標に変換
                            let screen_pos = to_screen.transform_pos(world_pos);

                            if square_rect.contains(screen_pos) {
                                painter.circle_filled(screen_pos, 2.0, lidar_color);
                            }
                        }
                    }
                }

                // どのLidarも接続されていない場合にメッセージを表示
                if !any_lidar_connected {
                    ui.allocate_ui_at_rect(rect, |ui| {
                        ui.centered_and_justified(|ui| {
                            // 最初のLidarのステータスを代表として表示（暫定）
                            let status = self
                                .lidars
                                .get(0)
                                .map_or("No Lidars configured", |l| &l.connection_status);
                            let text = egui::RichText::new(status)
                                .color(egui::Color32::WHITE)
                                .font(egui::FontId::proportional(24.0));
                            ui.add(egui::Label::new(text).wrap(true));
                        });
                    });
                }
            }
            AppMode::LidarAnalysis => {
                ui.heading("LiDAR Feature Analysis");
                let (response, painter) =
                    ui.allocate_painter(ui.available_size(), egui::Sense::hover());
                let rect = response.rect;
                self.lidar_draw_rect = Some(rect);

                // --- 描画設定 ---
                let side = rect.height();
                let square_rect =
                    egui::Rect::from_center_size(rect.center(), egui::vec2(side, side));

                let half_view_size = self.config.slam.online_slam_view_size / 2.0;
                let world_to_screen_rect = egui::Rect::from_min_max(
                    egui::pos2(-half_view_size, half_view_size), // ワールドの左上 (min_x, max_y)
                    egui::pos2(half_view_size, -half_view_size), // ワールドの右下 (max_x, min_y)
                );
                let to_screen =
                    egui::emath::RectTransform::from_to(world_to_screen_rect, square_rect);

                // 背景とグリッドを描画
                painter.rect_filled(square_rect, 0.0, egui::Color32::from_rgb(20, 20, 20));
                painter.hline(
                    square_rect.x_range(),
                    to_screen.transform_pos(egui::pos2(0.0, 0.0)).y,
                    egui::Stroke::new(0.5, egui::Color32::DARK_GRAY),
                );
                painter.vline(
                    to_screen.transform_pos(egui::pos2(0.0, 0.0)).x,
                    square_rect.y_range(),
                    egui::Stroke::new(0.5, egui::Color32::DARK_GRAY),
                );

                // ロボットの原点 (0,0) を描画
                let robot_origin_screen = to_screen.transform_pos(egui::pos2(0.0, 0.0));
                painter.circle_filled(
                    robot_origin_screen,
                    5.0,
                    egui::Color32::from_rgb(255, 0, 0),
                );

                // 各LiDARの点群を描画
                for lidar_state in &self.lidars {
                    let rotation = lidar_state.rotation;
                    let origin = lidar_state.origin;

                    for point in &lidar_state.points {
                        // Lidar座標系での点 (px, py, r, theta, feature, nx, ny)
                        let px_raw = point.0;
                        let py_raw = point.1;
                        let edge_ness = point.4;
                        let nx_raw = point.5;
                        let ny_raw = point.6;

                        // Lidarの回転を適用
                        let px_rotated = px_raw * rotation.cos() - py_raw * rotation.sin();
                        let py_rotated = px_raw * rotation.sin() + py_raw * rotation.cos();

                        let nx_rotated = nx_raw * rotation.cos() - ny_raw * rotation.sin();
                        let ny_rotated = nx_raw * rotation.sin() + ny_raw * rotation.cos();

                        // ワールド座標に変換 (Lidarの原点オフセットを加える)
                        let world_x = px_rotated + origin.x;
                        let world_y = py_rotated + origin.y;

                        let screen_pos = to_screen.transform_pos(egui::pos2(world_x, world_y));

                        // edge_ness に基づいて色を決定
                        let color = egui::Color32::from_rgb(
                            (edge_ness * 255.0) as u8,      // エッジらしさが高いほど赤が強く
                            ((1.0 - edge_ness) * 255.0) as u8, // 低いほど緑が強く
                            0,                                // 青は常に0
                        );
                        painter.circle_filled(screen_pos, 2.0, color);

                        // 法線ベクトルを描画 (edge_ness が閾値以上の場合のみ)
                        if edge_ness > 0.1 { // ある程度エッジらしい点のみ法線を描画
                            let normal_len = 0.1; // 法線ベクトルの長さ (ワールド座標)
                            let normal_end_x = world_x + nx_rotated * normal_len;
                            let normal_end_y = world_y + ny_rotated * normal_len;
                            let normal_end_screen = to_screen.transform_pos(egui::pos2(normal_end_x, normal_end_y));
                            painter.line_segment(
                                [screen_pos, normal_end_screen],
                                egui::Stroke::new(1.0, egui::Color32::YELLOW),
                            );
                        }
                    }
                }
            }
            AppMode::Slam => {
                ui.heading("SLAM Mode");
                let (response, painter) =
                    ui.allocate_painter(ui.available_size(), egui::Sense::hover());
                let rect = response.rect;
                self.lidar_draw_rect = Some(rect);
                painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 20, 20));

                // Draw robot pose
                let robot_pose = &self.current_robot_pose;
                // 描画範囲を決定
                let map_view_rect = if let Some(bounds) = self.slam_map_bounding_box {
                    // ロードした地図がある場合、そのバウンディングボックスを表示範囲とする
                    let new_bounds = bounds.expand(bounds.width() * 0.1); // 先に少しマージンを追加

                    let screen_aspect = rect.width() / rect.height();
                    if !screen_aspect.is_nan() {
                        let bounds_aspect = new_bounds.width() / new_bounds.height();
                        let center = new_bounds.center();
                        let (width, height) = if bounds_aspect > screen_aspect {
                            // バウンディングボックスの方が横長 -> 高さを増やす
                            (new_bounds.width(), new_bounds.width() / screen_aspect)
                        } else {
                            // バウンディングボックスの方が縦長 -> 幅を増やす
                            (new_bounds.height() * screen_aspect, new_bounds.height())
                        };
                        egui::Rect::from_center_size(center, egui::vec2(width, height))
                    } else {
                        new_bounds
                    }
                } else {
                    // オンラインSLAM中の場合、これまで通りロボットを中心に表示
                    let robot_center_world =
                        egui::pos2(robot_pose.translation.x, robot_pose.translation.y);
                    let map_view_size = self.config.slam.online_slam_view_size; // 追従時の表示サイズ
                    egui::Rect::from_center_size(
                        robot_center_world,
                        egui::vec2(map_view_size, map_view_size),
                    )
                };

                // Y軸を反転させるため、map_view_rectのYのmin/maxを入れ替える
                let mut inverted_map_view_rect = map_view_rect;
                inverted_map_view_rect.min.y = map_view_rect.max.y;
                inverted_map_view_rect.max.y = map_view_rect.min.y;
                let to_screen = egui::emath::RectTransform::from_to(
                    inverted_map_view_rect,
                    rect, // 実際の描画エリア
                );

                // 軌跡の線と向き（三角形）を描画
                if self.robot_trajectory.len() > 1 {
                    // 軌跡の線
                    let trajectory_line_points: Vec<egui::Pos2> = self
                        .robot_trajectory
                        .iter()
                        .map(|(world_pos, _angle)| to_screen.transform_pos(*world_pos))
                        .collect();
                    let line_stroke = egui::Stroke::new(1.0, egui::Color32::DARK_GRAY);
                    painter.add(egui::Shape::line(trajectory_line_points, line_stroke));

                    // 向きを示す三角形
                    let triangle_color = egui::Color32::GRAY;
                    for (i, (world_pos, angle)) in self.robot_trajectory.iter().enumerate() {
                        if i > 0 {
                            // 始点を除くすべての点で描画
                            let center_screen = to_screen.transform_pos(*world_pos);

                            // 三角形の頂点を定義
                            let triangle_size = 20.0; // 三角形の大きさ
                            let p1 = center_screen
                                + egui::vec2(angle.cos(), -angle.sin()) * triangle_size; // 先端
                            let angle_left = *angle + (150.0f32).to_radians();
                            let p2 = center_screen
                                + egui::vec2(angle_left.cos(), -angle_left.sin())
                                    * triangle_size
                                    * 0.7;
                            let angle_right = *angle - (150.0f32).to_radians();
                            let p3 = center_screen
                                + egui::vec2(angle_right.cos(), -angle_right.sin())
                                    * triangle_size
                                    * 0.7;

                            painter.add(egui::Shape::convex_polygon(
                                vec![p1, p2, p3],
                                triangle_color,
                                egui::Stroke::NONE,
                            ));
                        }
                    }
                }

                // Draw the map points
                for (point, probability) in &self.current_map_points {
                    // Right = +X, Up = +Y
                    let screen_pos = to_screen.transform_pos(egui::pos2(point.x, point.y));
                    if rect.contains(screen_pos) {
                        // 占有確率に基づいて色を調整
                        let intensity = (*probability as f32 * 255.0).min(255.0).max(0.0);
                        let color = egui::Color32::from_rgb(
                            (intensity * 0.4).min(255.0) as u8, // 青みを強くするためR,Gを低めに
                            (intensity * 0.4).min(255.0) as u8,
                            intensity as u8,
                        );
                        painter.circle_filled(screen_pos, 2.0, color);
                    }
                }

                // 最新のスキャンデータを別の色で描画
                for point in &self.latest_scan_for_draw {
                    let local_point = nalgebra::Point2::new(point.0, point.1);
                    let world_point = self.current_robot_pose * local_point;
                    let screen_pos =
                        to_screen.transform_pos(egui::pos2(world_point.x, world_point.y));
                    if rect.contains(screen_pos) {
                        painter.circle_filled(
                            screen_pos,
                            2.5, // 少し大きくして目立たせる
                            egui::Color32::YELLOW,
                        );
                    }
                }

                // Draw robot pose (この部分は変更しない)
                let robot_pos_on_screen = to_screen.transform_pos(egui::pos2(
                    robot_pose.translation.x,
                    robot_pose.translation.y,
                ));
                let angle = robot_pose.rotation.angle();
                painter.circle_filled(robot_pos_on_screen, 5.0, egui::Color32::RED);
                painter.line_segment(
                    [
                        robot_pos_on_screen,
                        robot_pos_on_screen + egui::vec2(angle.cos(), -angle.sin()) * 20.0,
                    ],
                    egui::Stroke::new(2.0, egui::Color32::RED),
                );
            }
            AppMode::Demo => {
                self.demo_manager.update_and_draw(ui);
            }
            AppMode::Osmo => {
                ui.heading("Osmo Mode");
                if let Some(texture) = &self.osmo.texture {
                    // 利用可能な描画領域のサイズを取得
                    let available_size = ui.available_size();
                    let image_size = texture.size_vec2();

                    if image_size.y > 0.0 {
                        let image_aspect = image_size.x / image_size.y;
                        let available_aspect = available_size.x / available_size.y;

                        let new_size = if image_aspect > available_aspect {
                            // 画像が描画領域より横長 -> 幅に合わせる
                            let new_height = available_size.x / image_aspect;
                            egui::vec2(available_size.x, new_height)
                        } else {
                            // 画像が描画領域より縦長 -> 高さに合わせる
                            let new_width = available_size.y * image_aspect;
                            egui::vec2(new_width, available_size.y)
                        };

                        // 画像を中央揃えで描画
                        ui.centered_and_justified(|ui| {
                            ui.add(egui::Image::new(texture).fit_to_exact_size(new_size));
                        });
                    }
                } else {
                    ui.centered_and_justified(|ui| {
                        ui.label(&self.osmo.connection_status);
                    });
                }
            }
            AppMode::Camera => {
                ui.heading("Camera Mode");
                if let Some(camera_state) = self.cameras.get(0) {
                    if let Some(texture) = &camera_state.texture {
                        // 利用可能な描画領域のサイズを取得
                        let available_size = ui.available_size();
                        let image_size = texture.size_vec2();

                        if image_size.y > 0.0 {
                            let image_aspect = image_size.x / image_size.y;
                            let available_aspect = available_size.x / available_size.y;

                            let new_size = if image_aspect > available_aspect {
                                // 画像が描画領域より横長 -> 幅に合わせる
                                let new_height = available_size.x / image_aspect;
                                egui::vec2(available_size.x, new_height)
                            } else {
                                // 画像が描画領域より縦長 -> 高さに合わせる
                                let new_width = available_size.y * image_aspect;
                                egui::vec2(new_width, available_size.y)
                            };

                            // 画像を中央揃えで描画
                            ui.centered_and_justified(|ui| {
                                ui.add(egui::Image::new(texture).fit_to_exact_size(new_size));
                            });
                        }
                    } else {
                        ui.centered_and_justified(|ui| {
                            ui.label(&camera_state.connection_status);
                        });
                    }
                } else {
                    ui.centered_and_justified(|ui| {
                        ui.label("No camera available.");
                    });
                }
            }
        });

        ctx.request_repaint();
    }
}

impl Drop for MyApp {
    fn drop(&mut self) {
        // アプリケーション終了時にカメラのスレッドに停止を通知
        for camera_state in &self.cameras {
            camera_state.stop_flag.store(true, Ordering::SeqCst);
        }
        // Osmoスレッドにも停止を通知
        self.osmo.stop_flag.store(true, Ordering::SeqCst);
    }
}

/// Helper function for path suggestions.
/// Lists entries in `dir_path` that start with `prefix`.
fn get_path_suggestions(dir_path: &str, prefix: &str) -> Vec<String> {
    let mut suggestions = Vec::new();
    let dir_to_read = if dir_path.is_empty() {
        Path::new(".")
    } else {
        Path::new(dir_path)
    };

    if let Ok(entries) = fs::read_dir(dir_to_read) {
        for entry in entries.filter_map(|e| e.ok()) {
            if let Some(file_name_os) = entry.file_name().to_str() {
                if file_name_os.starts_with(prefix) {
                    let mut suggestion_str = file_name_os.to_string();
                    if entry.path().is_dir() {
                        suggestion_str.push('/');
                    }
                    suggestions.push(suggestion_str);
                }
            }
        }
    }
    suggestions.sort();
    suggestions
}
