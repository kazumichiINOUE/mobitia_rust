#### `src/app.rs` (アプリケーションコア)
- `MyApp` 構造体がアプリケーション全体の状態を保持し、UIのメインループを管理します。
- 各種ハードウェアとの通信や重い計算処理はバックグラウンドスレッドにオフロードされ、`MyApp` はこれらスレッドからのメッセージ受信や共有データ（`Arc<Mutex<...>>`）の参照を通じて状態を統合・更新します。

##### 関連する型定義

```rust
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

// アプリケーション全体のの状態を管理するenum
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

pub struct ConsoleOutputEntry {
    pub text: String,
    pub group_id: usize, // どのコマンド実行に属するか
}
```

##### `MyApp` 構造体 (アプリケーション全体の状態)
アプリケーションのすべての状態と、スレッド間通信のためのチャネルの送受信エンドポイントを保持します。

```rust
pub struct MyApp {
    pub(crate) input_string: String,
    pub(crate) command_history: Vec<ConsoleOutputEntry>,
    pub(crate) user_command_history: Vec<String>,
    pub(crate) history_index: usize,
    pub(crate) current_suggestions: Vec<String>,
    pub(crate) suggestion_selection_index: Option<usize>,
    pub(crate) lidars: Vec<LidarState>,
    pub(crate) cameras: Vec<CameraState>,
    pub(crate) osmo: OsmoState,
    pub(crate) xppen: XppenState,

    pub(crate) lidar_message_receiver: mpsc::Receiver<LidarMessage>,
    pub(crate) camera_message_receiver: mpsc::Receiver<CameraMessage>,
    pub(crate) osmo_message_receiver: mpsc::Receiver<OsmoMessage>,
    pub(crate) motor_message_receiver: mpsc::Receiver<MotorMessage>,
    pub(crate) xppen_message_receiver: mpsc::Receiver<XppenMessage>,
    pub(crate) xppen_status_receiver: mpsc::Receiver<String>,
    pub(crate) xppen_trigger_sender: mpsc::Sender<()>,

    pub(crate) pending_scans: Vec<Option<Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>>>,

    pub(crate) command_output_receiver: mpsc::Receiver<String>,
    pub(crate) command_output_sender: mpsc::Sender<String>,

    pub(crate) lidar_draw_rect: Option<egui::Rect>,

    pub(crate) app_mode: AppMode,

    pub(crate) demo_manager: DemoManager,

    pub(crate) show_command_window: bool,
    pub(crate) focus_console_requested: bool,
    pub(crate) requested_point_save_path: Option<String>,
    pub(crate) next_group_id: usize,
    pub(crate) slam_mode: SlamMode,

    pub(crate) slam_command_sender: mpsc::Sender<SlamThreadCommand>,
    pub(crate) slam_result_receiver: mpsc::Receiver<SlamThreadResult>,

    pub(crate) motor_command_sender: mpsc::Sender<MotorCommand>,

    pub(crate) is_slam_processing: Arc<AtomicBool>,
    pub(crate) current_map_points: Vec<(nalgebra::Point2<f32>, f64)>,
    pub(crate) current_robot_pose: nalgebra::Isometry2<f32>,
    pub(crate) single_scan_requested_by_ui: bool,
    pub(crate) latest_scan_for_draw: Vec<(f32, f32, f32, f32, f32, f32, f32, f32)>,
    pub(crate) robot_trajectory: Vec<(egui::Pos2, f32)>,
    pub(crate) last_map_update_pose: Option<nalgebra::Isometry2<f32>>,

    pub(crate) submap_counter: usize,
    pub(crate) current_submap_scan_buffer: Vec<Vec<(f32, f32)>>,
    pub(crate) current_submap_robot_poses: Vec<nalgebra::Isometry2<f32>>,
    pub(crate) submaps: HashMap<usize, Submap>,
    pub(crate) slam_map_bounding_box: Option<egui::Rect>,
    pub(crate) grid_world_bounds: Option<egui::Rect>,

    pub(crate) submap_load_queue: Option<Vec<PathBuf>>,
    pub(crate) current_submap_load_progress: Option<app_map_loading::SubmapLoadProgress>,
    pub(crate) last_submap_load_time: Option<Instant>,
    pub(crate) offline_map: Option<OccupancyGrid>,
    pub(crate) map_texture: Option<egui::TextureHandle>,

    pub(crate) suggestion_completion_requested: bool,
    pub(crate) command_submission_requested: bool,
    pub(crate) clear_command_requested: bool,

    pub(crate) osmo_capture_session_path: Option<PathBuf>,
    pub(crate) trajectory_save_path: Option<PathBuf>,
    pub(crate) map_image_save_path: Option<PathBuf>,
    pub(crate) map_info_save_path: Option<PathBuf>,

    pub(crate) config: crate::config::Config,

    pub(crate) motor_thread_active: bool,
    pub(crate) motor_odometry: (f32, f32, f32),
    pub(crate) last_slam_odom: (f32, f32, f32),
    pub(crate) is_motor_initialized: bool,
    pub(crate) motor_thread_handle: Option<thread::JoinHandle<()>>,
    pub(crate) shared_odometry: Arc<Mutex<(f32, f32, f32)>>,

    pub(crate) is_shutting_down: bool,
    pub(crate) slam_thread_handle: Option<thread::JoinHandle<()>>,

    pub(crate) lidar_screen: crate::ui::lidar_screen::LidarScreen,
    pub(crate) lidar_analysis_screen: crate::ui::lidar_analysis_screen::LidarAnalysisScreen,
    pub(crate) slam_screen: crate::ui::slam_screen::SlamScreen,
    pub(crate) map_screen: crate::ui::map_screen::MapScreen,
    pub(crate) demo_screen: crate::ui::demo_screen::DemoScreen,
    pub(crate) osmo_screen: crate::ui::osmo_screen::OsmoScreen,
    pub(crate) camera_screen: crate::ui::camera_screen::CameraScreen,
}
```

##### `MyApp::new` (アプリケーションの初期化)
アプリケーション起動時に一度だけ呼び出され、`MyApp`のインスタンスを生成します。
各種サブシステム用のチャネルを設定し、バックグラウンドスレッドを起動します。

```rust
impl MyApp {
    /// Creates a new instance of the application.
    pub fn new(cc: &eframe::CreationContext, config: crate::config::Config) -> Self {
        let lidars = config.get_lidar_states();
        let pending_scans = vec![None; lidars.len()];

        // 各サブシステム用のmpscチャネルをセットアップ
        let (lidar_message_sender, lidar_message_receiver) = mpsc::channel();
        let (camera_message_sender, camera_message_receiver) = mpsc::channel();
        let (osmo_message_sender, osmo_message_receiver) = mpsc::channel();
        let (xppen_message_sender, xppen_message_receiver) = mpsc::channel();
        let (xppen_trigger_sender, xppen_trigger_receiver) = mpsc::channel();
        let (xppen_status_sender, xppen_status_receiver) = mpsc::channel();
        let (motor_command_sender, motor_command_receiver) = mpsc::channel();
        let (motor_message_sender, motor_message_receiver) = mpsc::channel();
        let (command_output_sender, command_output_receiver) = mpsc::channel();
        let (slam_command_sender, slam_command_receiver) = mpsc::channel();
        let (slam_result_sender, slam_result_receiver) = mpsc::channel();

        // モーター制御スレッドを起動 (shared_odometryを返す)
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

        // カメラの初期化とスレッド起動 (一部省略)
        let mut cameras = Vec::new();
        let camera_devices = nokhwa::query(nokhwa::utils::ApiBackend::Auto).unwrap_or_else(|e| { /* ... */ vec![] });
        if let Some(device) = selected_camera_device {
            // ... (カメラ状態のセットアップとstart_camera_thread呼び出し) ...
        } else { /* ... */ }

        // Osmoの初期化とスレッド起動
        let osmo_stop_flag = Arc::new(AtomicBool::new(false));
        let osmo_state = OsmoState { /* ... */ };
        let osmo_info = OsmoInfo { /* ... */ };
        start_osmo_thread(osmo_info, osmo_message_sender.clone(), osmo_stop_flag);

        // SLAMスレッドの起動
        let is_slam_processing = Arc::new(AtomicBool::new(false));
        let is_slam_processing_for_thread = is_slam_processing.clone();
        let slam_config_for_thread = config.slam.clone();
        let slam_thread_handle = Some(thread::spawn(move || {
            let mut slam_manager = SlamManager::new( /* ... */ );
            // ... (SLAMスレッドのループロジック) ...
        }));

        // MyApp構造体の初期化
        Self {
            input_string: String::new(),
            command_history: vec![/* ... Welcome messages ... */],
            user_command_history: Vec::new(),
            history_index: 0,
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
            is_slam_processing,
            current_map_points: Vec::new(),
            current_robot_pose: nalgebra::Isometry2::identity(),
            single_scan_requested_by_ui: false,
            latest_scan_for_draw: Vec::new(),
            robot_trajectory: Vec::new(),
            last_map_update_pose: None,
            submap_counter: 0,
            current_submap_scan_buffer: Vec::new(),
            current_submap_robot_poses: Vec<nalgebra::Isometry2<f32>>,
            submaps: HashMap::new(),
            slam_map_bounding_box: None,
            grid_world_bounds: None,
            submap_load_queue: None,
            current_submap_load_progress: None,
            last_submap_load_time: None,
            offline_map: None,
            map_texture: None,
            suggestion_completion_requested: false,
            command_submission_requested: false,
            clear_command_requested: false,
            osmo_capture_session_path: None,
            trajectory_save_path: None,
            map_image_save_path: None,
            map_info_save_path: None,
            xppen: XppenState {
                connection_status: "Disconnected".to_string(),
            },
            config,
            motor_thread_handle: Some(motor_thread_handle),
            shared_odometry,
            is_shutting_down: false,
            slam_thread_handle,
            lidar_screen: crate::ui::lidar_screen::LidarScreen::new(),
            lidar_analysis_screen: crate::ui::lidar_analysis_screen::LidarAnalysisScreen::new(),
            slam_screen: crate::ui::slam_screen::SlamScreen::new(),
            map_screen: crate::ui::map_screen::MapScreen::new(),
            demo_screen: crate::ui::demo_screen::DemoScreen::new(),
            osmo_screen: crate::ui::osmo_screen::OsmoScreen::new(),
            camera_screen: crate::ui::camera_screen::CameraScreen::new(),
            motor_thread_active: true,
            motor_odometry: (0.0, 0.0, 0.0),
            last_slam_odom: (0.0, 0.0, 0.0),
            is_motor_initialized: false,
        }
    }
}
```

##### `MyApp::update` (メインUI更新ループ)
毎フレーム呼び出され、アプリケーションの状態を更新し、UIを描画します。