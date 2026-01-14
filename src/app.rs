use anyhow::Result;
use chrono::Local;
use clap::Parser;
use eframe::egui;
use egui::Vec2;
use nalgebra::Isometry2;
use std::collections::HashMap;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::{Instant, SystemTime};

use crate::camera::{start_camera_thread, CameraInfo, CameraMessage};
use crate::cli::Cli;
use crate::config::Config;
use crate::demo::DemoManager;
pub use crate::demo::DemoMode;
use crate::lidar::features::{compute_features, interpolate_lidar_scan};
use crate::lidar::{start_lidar_thread, LidarInfo};
use crate::motors::{start_modbus_motor_thread, MotorCommand, MotorMessage};
use crate::osmo::{start_osmo_thread, OsmoInfo, OsmoMessage};
use crate::slam::{OccupancyGrid, SlamManager, Submap};
use crate::xppen::{start_xppen_thread, XppenMessage};

mod app_map_loading;
use crate::ui::get_path_suggestions::get_path_suggestions;

// Camera一台分の状態を保持する構造体
#[derive(Clone)]
pub struct CameraState {
    pub(crate) id: usize,
    pub(crate) name: String,
    pub(crate) texture: Option<egui::TextureHandle>,
    pub(crate) connection_status: String,
    pub(crate) stop_flag: Arc<AtomicBool>,
}

// Osmo一台分の状態を保持する構造体
#[derive(Clone)]
pub struct OsmoState {
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
pub struct LidarState {
    pub(crate) id: usize,
    pub(crate) path: String,
    pub(crate) baud_rate: u32,
    pub(crate) points: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
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
    Map,
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
        scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
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
        raw_scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
        interpolated_scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
        timestamp: u128,
        odom_guess: Option<(f32, f32, f32)>,
    }, // LiDARからの新しいスキャンデータ
    ProcessSingleScan {
        raw_scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
        interpolated_scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
        timestamp: u128,
        odom_guess: Option<(f32, f32, f32)>,
    }, // 単一スキャン処理要求
    Shutdown,
}

pub struct SlamThreadResult {
    pub map_points: Vec<(nalgebra::Point2<f32>, f64)>,
    pub robot_pose: nalgebra::Isometry2<f32>,
    pub scan_used: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
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
    pub(crate) motor_message_receiver: mpsc::Receiver<MotorMessage>,
    pub(crate) xppen_message_receiver: mpsc::Receiver<XppenMessage>,
    pub(crate) xppen_status_receiver: mpsc::Receiver<String>,
    pub(crate) xppen_trigger_sender: mpsc::Sender<()>,

    // SLAM用に各Lidarの最新スキャンを保持
    pub(crate) pending_scans: Vec<Option<Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>>>,

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

    // Motorスレッドとの通信用チャネル
    pub(crate) motor_command_sender: mpsc::Sender<MotorCommand>,

    // SLAMが処理中かどうかを示す共有フラグ
    pub(crate) is_slam_processing: Arc<AtomicBool>,

    // SLAMスレッドからの最新結果を保持 (描画用)
    pub(crate) current_map_points: Vec<(nalgebra::Point2<f32>, f64)>,

    pub(crate) current_robot_pose: nalgebra::Isometry2<f32>,

    pub(crate) single_scan_requested_by_ui: bool,

    pub(crate) latest_scan_for_draw: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,

    pub(crate) robot_trajectory: Vec<(egui::Pos2, f32)>,

    // --- サブマップ関連のフィールド (MyAppに移行) ---
    pub(crate) submap_counter: usize,
    pub(crate) current_submap_scan_buffer: Vec<Vec<(f32, f32)>>,
    pub(crate) current_submap_robot_poses: Vec<nalgebra::Isometry2<f32>>,
    pub(crate) submaps: HashMap<usize, Submap>,
    pub(crate) slam_map_bounding_box: Option<egui::Rect>,

    /// `list-and-load`コマンドで読み込むサブマップのパスのキュー
    pub(crate) submap_load_queue: Option<Vec<PathBuf>>,
    /// 現在読み込み中のサブマップの進捗
    pub(crate) current_submap_load_progress: Option<app_map_loading::SubmapLoadProgress>,
    /// 最後にサブマップを読み込んだ時刻
    pub(crate) last_submap_load_time: Option<Instant>,
    /// 地図読み込みセッション中に使用される共有占有グリッド
    pub(crate) offline_map: Option<OccupancyGrid>,

    /// F6キーによるサジェスト補完が要求されたか
    pub(crate) suggestion_completion_requested: bool,

    /// F11キーによるコマンド実行が要求されたか
    pub(crate) command_submission_requested: bool,

    /// F10キーによるクリアコマンドが要求されたか
    pub(crate) clear_command_requested: bool,

    pub(crate) osmo_capture_session_path: Option<PathBuf>,

    pub(crate) config: crate::config::Config, // 追加

    // モーター制御スレッドがアクティブかどうか
    pub(crate) motor_thread_active: bool,
    // モーターオドメトリによる姿勢
    pub(crate) motor_odometry: (f32, f32, f32),
    // 前回のSLAM更新時のオドメトリ
    pub(crate) last_slam_odom: (f32, f32, f32),
    // モータースレッドのハンドル
    pub(crate) motor_thread_handle: Option<thread::JoinHandle<()>>,
    // 共有オドメトリデータ
    pub(crate) shared_odometry: Arc<Mutex<(f32, f32, f32)>>,

    // --- Shutdown process ---
    pub(crate) is_shutting_down: bool,
    pub(crate) slam_thread_handle: Option<thread::JoinHandle<()>>,

    // --- UI Screen States ---
    pub(crate) lidar_screen: crate::ui::lidar_screen::LidarScreen,
    pub(crate) lidar_analysis_screen: crate::ui::lidar_analysis_screen::LidarAnalysisScreen,
    pub(crate) slam_screen: crate::ui::slam_screen::SlamScreen,
    pub(crate) map_screen: crate::ui::map_screen::MapScreen,
    pub(crate) demo_screen: crate::ui::demo_screen::DemoScreen,
    pub(crate) osmo_screen: crate::ui::osmo_screen::OsmoScreen,
    pub(crate) camera_screen: crate::ui::camera_screen::CameraScreen,
}

impl MyApp {
    /// Creates a new instance of the application.
    pub fn new(cc: &eframe::CreationContext, config: crate::config::Config) -> Self {
        // Lidar の初期設定を読み込む
        let lidars = config.get_lidar_states();

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

        // モーター制御用のチャネルを作成
        let (motor_command_sender, motor_command_receiver) = mpsc::channel();
        let (motor_message_sender, motor_message_receiver) = mpsc::channel();

        // コンソールコマンド出力用のチャネルを作成
        let (command_output_sender, command_output_receiver) = mpsc::channel();

        // モーター制御スレッドを起動
        let (motor_thread_handle, shared_odometry) = start_modbus_motor_thread(
            config.motor.clone(),
            motor_command_receiver,
            motor_message_sender,
        );

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

        let slam_thread_handle = Some(thread::spawn(move || {
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
                            odom_guess,
                        } => {
                            if current_slam_mode == SlamMode::Continuous {
                                let now = web_time::Instant::now();
                                if now.duration_since(last_slam_update_time)
                                    >= SLAM_UPDATE_INTERVAL_DURATION
                                {
                                    is_slam_processing_for_thread.store(true, Ordering::SeqCst);

                                    let slam_start_time = web_time::Instant::now();
                                    slam_manager.update(
                                        &raw_scan,
                                        &interpolated_scan,
                                        timestamp,
                                        odom_guess,
                                    );
                                    let _slam_duration = slam_start_time.elapsed();
                                    //println!("[SLAM Thread] Update took: {:?}", slam_duration);

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
                            odom_guess,
                        } => {
                            is_slam_processing_for_thread.store(true, Ordering::SeqCst);

                            slam_manager.update(
                                &raw_scan,
                                &interpolated_scan,
                                timestamp,
                                odom_guess,
                            );

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
        }));

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
            motor_message_receiver,
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
            motor_command_sender,
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
            current_submap_load_progress: None,
            last_submap_load_time: None,
            offline_map: None,
            suggestion_completion_requested: false,
            command_submission_requested: false,
            clear_command_requested: false,
            osmo_capture_session_path: None,
            xppen: XppenState {
                connection_status: "Disconnected".to_string(),
            },
            config,
            motor_thread_handle: Some(motor_thread_handle),
            shared_odometry,

            // --- Shutdown process ---
            is_shutting_down: false,
            slam_thread_handle,

            // --- UI Screen States ---
            lidar_screen: crate::ui::lidar_screen::LidarScreen::new(),
            lidar_analysis_screen: crate::ui::lidar_analysis_screen::LidarAnalysisScreen::new(),
            slam_screen: crate::ui::slam_screen::SlamScreen::new(),
            map_screen: crate::ui::map_screen::MapScreen::new(),
            demo_screen: crate::ui::demo_screen::DemoScreen::new(),
            osmo_screen: crate::ui::osmo_screen::OsmoScreen::new(),
            camera_screen: crate::ui::camera_screen::CameraScreen::new(),
            motor_thread_active: true, // Initialize to true
            motor_odometry: (0.0, 0.0, 0.0),
            last_slam_odom: (0.0, 0.0, 0.0),
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
            "motor",
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

                ["motor", "set-timed", ..] | ["motor", "tm", ..] => {
                    let mut suggestions = Vec::new();
                    if !parts.contains(&"--velocity") {
                        suggestions.push("--velocity ".to_string());
                    }
                    if !parts.contains(&"--omega") {
                        suggestions.push("--omega ".to_string());
                    }
                    if !parts.contains(&"--ms") {
                        suggestions.push("--ms ".to_string());
                    }
                    self.current_suggestions = suggestions;
                }

                ["motor", "set", ..] => {
                    let mut suggestions = Vec::new();
                    if !parts.contains(&"--velocity") {
                        suggestions.push("--velocity ".to_string());
                    }
                    if !parts.contains(&"--omega") {
                        suggestions.push("--omega ".to_string());
                    }
                    self.current_suggestions = suggestions;
                }

                ["motor", partial_subcommand] => {
                    let options = vec![
                        "set",
                        "set-timed",
                        "tm",
                        "stop",
                        "enable-id-share",
                        "eidshare",
                        "servo-on",
                        "servo-off",
                        "servo-free",
                        "read-state",
                    ];
                    self.current_suggestions = options
                        .into_iter()
                        .filter(|opt| opt.starts_with(partial_subcommand))
                        .map(|s| s.to_string())
                        .collect();
                }
                ["motor"] if ends_with_space => {
                    self.current_suggestions = vec![
                        "set".to_string(),
                        "set-timed".to_string(),
                        "tm".to_string(),
                        "stop".to_string(),
                        "enable-id-share".to_string(),
                        "eidshare".to_string(),
                        "servo-on".to_string(),
                        "servo-off".to_string(),
                        "servo-free".to_string(),
                        "read-state".to_string(),
                    ];
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
        // --- Update motor odometry from shared data ---
        if let Ok(odometry) = self.shared_odometry.lock() {
            self.motor_odometry = *odometry;
        }

        // --- Shutdown sequence ---
        if self.is_shutting_down {
            if let Some(handle) = &self.slam_thread_handle {
                if handle.is_finished() {
                    if let Some(handle) = self.slam_thread_handle.take() {
                        let _ = handle.join();
                    }
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                } else {
                    // SLAM thread is still running, repaint to re-check
                }
            } else {
                // SLAM thread already joined or never existed, just close
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            ctx.request_repaint(); // Request repaint to re-check the thread status
            return;
        }

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
        if self.current_submap_load_progress.is_some() {
            match self.process_next_scan_in_submap(ctx) {
                Ok(true) => {
                    // A scan was processed, update the map points for drawing
                    self.update_map_points_from_grid();
                    self.update_slam_map_bounding_box();
                    ctx.request_repaint();
                }
                Ok(false) => {
                    // Finished processing a submap
                    ctx.request_repaint();
                }
                Err(e) => {
                    self.command_history.push(ConsoleOutputEntry {
                        text: format!("ERROR during submap processing: {}", e),
                        group_id: self.next_group_id,
                    });
                    self.current_submap_load_progress = None; // Stop processing on error
                }
            }
        } else if let Some(path_to_load) = self.submap_load_queue.as_mut().and_then(|q| {
            if q.is_empty() {
                None
            } else {
                Some(q.remove(0))
            }
        }) {
            let submap_path_str = path_to_load.to_string_lossy().into_owned();
            if let Err(e) = self.start_submap_loading_session(&submap_path_str) {
                self.command_history.push(ConsoleOutputEntry {
                    text: format!(
                        "ERROR starting to load submap '{}': {}",
                        path_to_load.display(),
                        e
                    ),
                    group_id: self.next_group_id,
                });
            }

            if self
                .submap_load_queue
                .as_ref()
                .map_or(false, |q| q.is_empty())
            {
                self.command_history.push(ConsoleOutputEntry {
                    text: "All submap loading sessions initiated.".to_string(),
                    group_id: self.next_group_id,
                });
                self.submap_load_queue = None;
            }
            ctx.request_repaint();
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
                    let mut filtered_scan: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)> =
                        Vec::new();
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

                    for &(x, y, r, theta, feature, nx, ny, corner) in &scan {
                        // フィルタリング範囲内にあるかチェック (角度は受信データから直接利用)
                        if theta >= min_angle_rad && theta <= max_angle_rad {
                            filtered_scan.push((x, y, r, theta, feature, nx, ny, corner));
                        }
                    }

                    // フィルタリング後のスキャンに対して特徴量を計算
                    let scan_with_features = compute_features(&filtered_scan);

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
                            let mut raw_combined_scan: Vec<(
                                f32,
                                f32,
                                f32,
                                f32,
                                f32,
                                f32,
                                f32,
                                f32,
                            )> = Vec::new();

                            // Calculate odometry guess for SLAM
                            let odom_guess = if self.config.slam.use_odometry_as_initial_guess {
                                let (curr_odom_x, curr_odom_y, curr_odom_angle) =
                                    self.motor_odometry;
                                let (last_slam_odom_x, last_slam_odom_y, last_slam_odom_angle) =
                                    self.last_slam_odom;

                                // ロボットのローカル座標系での移動量を計算
                                let delta_x_global = curr_odom_x - last_slam_odom_x;
                                let delta_y_global = curr_odom_y - last_slam_odom_y;
                                let delta_angle = curr_odom_angle - last_slam_odom_angle;

                                // グローバル座標の差分を前回のSLAM姿勢（last_slam_odom_angle）でローカル座標に回転
                                let cos_angle = last_slam_odom_angle.cos();
                                let sin_angle = last_slam_odom_angle.sin();
                                let odom_dx =
                                    delta_x_global * cos_angle + delta_y_global * sin_angle;
                                let odom_dy =
                                    -delta_x_global * sin_angle + delta_y_global * cos_angle;

                                self.last_slam_odom = self.motor_odometry; // Update last SLAM odometry

                                Some((odom_dx, odom_dy, delta_angle))
                            } else {
                                self.last_slam_odom = self.motor_odometry; // Still update, even if not used
                                None
                            };

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
                                            0.0, // corner_nessの初期値として0.0を追加
                                        ));
                                    }
                                }
                            }

                            // 結合した生スキャンデータを補間
                            // FIXME: 閾値と間隔は調整が必要
                            let interpolated_combined_scan = interpolate_lidar_scan(
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
                                            odom_guess,
                                        })
                                        .unwrap_or_default();
                                } else if self.single_scan_requested_by_ui {
                                    self.slam_command_sender
                                        .send(SlamThreadCommand::ProcessSingleScan {
                                            raw_scan: raw_combined_scan,                   // raw_scanを送信
                                            interpolated_scan: interpolated_combined_scan, // 補間済みscanを送信
                                            timestamp: current_timestamp,
                                            odom_guess,
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

        while let Ok(motor_message) = self.motor_message_receiver.try_recv() {
            match motor_message {
                MotorMessage::Status(text) => {
                    // Check for the specific confirmation to trigger the next command in the sequence.
                    if text == "Motor ID Share Enabled." {
                        if self.motor_thread_active {
                            if let Err(e) = self.motor_command_sender.send(MotorCommand::ReadState)
                            {
                                let error_msg = format!(
                                    "ERROR: Failed to send follow-up ReadState command: {}",
                                    e
                                );
                                self.command_output_sender
                                    .send(error_msg)
                                    .unwrap_or_default();
                                self.motor_thread_active = false;
                            }
                        }
                    }
                    self.command_history.push(ConsoleOutputEntry {
                        text,
                        group_id: self.next_group_id,
                    });
                }
                MotorMessage::StateUpdate(state) => {
                    let mut output_lines = Vec::new();
                    output_lines
                        .push("--- Motor State -----------------------------------".to_string());
                    output_lines.push(format!(
                        "{:<15} {:<15} {:<15}",
                        "Parameter", "Left Motor", "Right Motor"
                    ));
                    output_lines
                        .push("---------------------------------------------------".to_string());

                    output_lines.push(format!(
                        "{:<15} {:<15} {:<15}",
                        "Alarm", state.alarm_code_l, state.alarm_code_r
                    ));
                    output_lines.push(format!(
                        "{:<15} {:<15} {:<15}",
                        "Driver Temp",
                        format!("{:.1}°C", state.temp_driver_l),
                        format!("{:.1}°C", state.temp_driver_r)
                    ));
                    output_lines.push(format!(
                        "{:<15} {:<15} {:<15}",
                        "Motor Temp",
                        format!("{:.1}°C", state.temp_motor_l),
                        format!("{:.1}°C", state.temp_motor_r)
                    ));
                    output_lines.push(format!(
                        "{:<15} {:<15} {:<15}",
                        "Position", state.position_l, state.position_r
                    ));
                    output_lines.push(format!(
                        "{:<15} {:<15} {:<15}",
                        "Power", state.power_l, state.power_r
                    ));
                    output_lines.push(format!(
                        "{:<15} {:<15} {:<15}",
                        "Voltage",
                        format!("{:.1}V", state.voltage_l),
                        format!("{:.1}V", state.voltage_r)
                    ));
                    output_lines
                        .push("---------------------------------------------------".to_string());

                    for line in output_lines {
                        self.command_history.push(ConsoleOutputEntry {
                            text: line,
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
            //println!("{}", msg); // ここで標準出力にメッセージを出力
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
                if self.config.ui.show_osmo_panel {
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
                }

                // XPPen Status
                if self.config.ui.show_xppen_panel {
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
                }

                if self.config.ui.show_camera_panel {
                    for camera_state in &self.cameras {
                        egui::Frame::group(ui.style()).show(ui, |ui| {
                            ui.set_width(ui.available_width());
                            ui.label(format!("[CAMERA {}]", camera_state.id));
                            ui.label(format!("  Name: {}", camera_state.name));
                            let status_text =
                                format!("  Status: {}", camera_state.connection_status);
                            let status_color =
                                if camera_state.connection_status.contains("Connected") {
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
                }

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
                self.lidar_screen
                    .draw(ui, &self.config, &self.lidars, &mut self.lidar_draw_rect);
            }
            AppMode::LidarAnalysis => {
                self.lidar_analysis_screen.draw(
                    ui,
                    &self.config,
                    &self.lidars,
                    &mut self.lidar_draw_rect,
                );
            }
            AppMode::Slam => {
                self.slam_screen.draw(
                    ui,
                    &self.config,
                    &mut self.lidar_draw_rect,
                    &self.current_robot_pose,
                    &self.robot_trajectory,
                    &self.current_map_points,
                    &self.latest_scan_for_draw,
                    &self.motor_odometry,
                );
            }
            AppMode::Map => {
                let map_loading_complete = self.current_submap_load_progress.is_none()
                    && (self.submap_load_queue.is_none()
                        || self.submap_load_queue.as_ref().unwrap().is_empty());
                self.map_screen.draw(
                    ui,
                    &self.config,
                    &mut self.lidar_draw_rect,
                    &self.current_robot_pose,
                    &self.slam_map_bounding_box,
                    &self.robot_trajectory,
                    &self.current_map_points,
                    map_loading_complete,
                );
            }
            AppMode::Demo => {
                self.demo_screen.draw(ui, &mut self.demo_manager);
            }
            AppMode::Osmo => {
                self.osmo_screen.draw(ui, &self.osmo);
            }
            AppMode::Camera => {
                self.camera_screen.draw(ui, &self.cameras);
            }
        });

        // --- Motor control via keyboard for testing ---
        // コンソールにフォーカスがない場合にのみ、矢印キーによるモーター制御を有効にする
        if !ctx.wants_keyboard_input() && self.motor_thread_active {
            // Add check for motor_thread_active
            let input = ctx.input(|i| i.clone());
            let mut v = 0.0;
            let mut w = 0.0;
            // No need for velocity_changed, just determine the command
            let mut command_to_send = MotorCommand::Stop;

            let mut is_moving_key_down = false;

            if input.key_down(egui::Key::ArrowUp) {
                v = 0.3; // m/s
                is_moving_key_down = true;
            }
            if input.key_down(egui::Key::ArrowDown) {
                v = -0.3; // m/s
                is_moving_key_down = true;
            }
            if input.key_down(egui::Key::ArrowLeft) {
                w = 0.5; // rad/s
                is_moving_key_down = true;
            }
            if input.key_down(egui::Key::ArrowRight) {
                w = -0.5; // rad/s
                is_moving_key_down = true;
            }

            if is_moving_key_down {
                command_to_send = MotorCommand::SetVelocity(v, w);
            }

            if let Err(e) = self.motor_command_sender.send(command_to_send) {
                let error_msg = format!("ERROR: Failed to send motor command: {}", e);
                // mpsc::SendErrorが発生した場合、レシーバがドロップされたことを意味するため、
                // これ以上コマンドを送信しようとしない。
                self.command_output_sender
                    .send(error_msg)
                    .unwrap_or_default();
                self.motor_thread_active = false; // 以降のコマンド送信を停止
            }
        }

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
