use clap::Parser;
use eframe::egui;
use egui::Vec2;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;


use crate::cli::Cli;
use crate::lidar::{start_lidar_thread, LidarInfo};
use crate::slam::SlamManager;
use crate::demo::DemoManager;
pub use crate::demo::DemoMode;

// Lidar一台分の状態を保持する構造体
#[derive(Clone)]
pub(crate) struct LidarState {
    pub(crate) id: usize,
    pub(crate) path: String,
    pub(crate) baud_rate: u32,
    pub(crate) points: Vec<(f32, f32)>,
    pub(crate) connection_status: String,
    pub(crate) status_messages: Vec<String>,
    // ワールド座標におけるこのLidarの原点オフセット
    pub(crate) origin: Vec2,
    // ワールド座標におけるこのLidarの回転オフセット (ラジアン)
    pub(crate) rotation: f32,
}

// アプリケーション全体のの状態を管理する構造体
#[derive(PartialEq)]
pub enum AppMode {
    Lidar,
    Slam,
    Demo,
}

#[derive(PartialEq)]
pub enum SlamMode {
    Manual,
    Continuous,
    Paused,
}

pub enum LidarMessage {
    ScanUpdate { id: usize, scan: Vec<(f32, f32)> },
    StatusUpdate { id: usize, message: String },
}

#[allow(dead_code)]
pub enum SlamThreadCommand {
    StartContinuous,
    Pause,
    Resume,
    UpdateScan { scan: Vec<(f32, f32)> }, // LiDARからの新しいスキャンデータ
    ProcessSingleScan { scan: Vec<(f32, f32)> }, // 単一スキャン処理要求
    Shutdown,
}

pub struct SlamThreadResult {
    pub map_points: Vec<nalgebra::Point2<f32>>,
    pub robot_pose: nalgebra::Isometry2<f32>,
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
    pub(crate) lidars: Vec<LidarState>, // 複数Lidarの状態を管理

    // スレッドからのデータ/ステータス受信を統合
    pub(crate) lidar_message_receiver: mpsc::Receiver<LidarMessage>,

    // SLAM用に各Lidarの最新スキャンを保持
    pub(crate) pending_scans: [Option<Vec<(f32, f32)>>; 2],

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
    pub(crate) current_map_points: Vec<nalgebra::Point2<f32>>,

    pub(crate) current_robot_pose: nalgebra::Isometry2<f32>,

    pub(crate) single_scan_requested_by_ui: bool, // 追加
}

impl MyApp {
    /// Creates a new instance of the application.
    pub fn new(cc: &eframe::CreationContext) -> Self {
        // 2台のLidarの初期設定
        // TODO: 将来的には設定ファイルなどから読み込む
        let lidar_defs = vec![
            (
                0, // 進行方向右手のlidar
                "/dev/cu.usbmodem1201",
                115200,
                Vec2::new(0.0,-0.14), -std::f32::consts::FRAC_PI_2), // - 90 deg
            (
                1, // 進行方向左手のliar
                "/dev/cu.usbmodem1301",
                115200,
                Vec2::new(0.0, 0.14), std::f32::consts::FRAC_PI_2), // 90 deg
        ];
        let mut lidars = Vec::new();
        for (id, path, baud_rate, origin, rotation) in lidar_defs {
            // eframe::Storageから対応するlidar_pathを読み込む試み
            let storage_key = format!("lidar_path_{}", id);
            let device_path = cc
                .storage
                .and_then(|storage| storage.get_string(&storage_key))
                .unwrap_or_else(|| path.to_string());

            lidars.push(LidarState {
                id,
                path: device_path,
                baud_rate,
                points: Vec::new(),
                connection_status: "Connecting...".to_string(),
                status_messages: Vec::new(),
                origin,
                rotation,
            });
        }

        // Lidarメッセージング用の単一チャネルを作成
        let (lidar_message_sender, lidar_message_receiver) = mpsc::channel();

        // コンソールコマンド出力用のチャネルを作成
        let (command_output_sender, command_output_receiver) = mpsc::channel();


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

        // SLAM処理中フラグの作成
        let is_slam_processing = Arc::new(AtomicBool::new(false));

        // SLAMスレッドとの通信チャネル
        let (slam_command_sender, slam_command_receiver) = mpsc::channel();
        let (slam_result_sender, slam_result_receiver) = mpsc::channel();
        let is_slam_processing_for_thread = is_slam_processing.clone();

        // SLAMスレッドの起動
        thread::spawn(move || {
            let mut slam_manager = SlamManager::new();
            let mut current_slam_mode = SlamMode::Manual;
            let mut last_slam_update_time = web_time::Instant::now(); // std::time::Instant の代わりにweb_time::Instantを使用
            const SLAM_UPDATE_INTERVAL_DURATION: web_time::Duration =
                web_time::Duration::from_secs(1); // Duration型に変更

            loop {
                // UIスレッドからのコマンドを処理
                if let Ok(cmd) = slam_command_receiver.try_recv() {
                    match cmd {
                        SlamThreadCommand::StartContinuous => {
                            current_slam_mode = SlamMode::Continuous;
                            last_slam_update_time = web_time::Instant::now(); // モード切り替え時にリセット
                            slam_result_sender
                                .send(SlamThreadResult {
                                    map_points: slam_manager.get_map_points().clone(),
                                    robot_pose: *slam_manager.get_robot_pose(),
                                })
                                .unwrap_or_default();
                        }
                        SlamThreadCommand::Pause => current_slam_mode = SlamMode::Paused,
                        SlamThreadCommand::Resume => {
                            // Resumeを追加
                            current_slam_mode = SlamMode::Continuous;
                            last_slam_update_time = web_time::Instant::now(); // リセット
                            slam_result_sender
                                .send(SlamThreadResult {
                                    map_points: slam_manager.get_map_points().clone(),
                                    robot_pose: *slam_manager.get_robot_pose(),
                                })
                                .unwrap_or_default();
                        }
                        SlamThreadCommand::UpdateScan { scan } => {
                            // UIからLiDARデータが届いたら、モードとタイマーをチェックして更新
                            if current_slam_mode == SlamMode::Continuous {
                                let now = web_time::Instant::now();
                                if now.duration_since(last_slam_update_time)
                                    >= SLAM_UPDATE_INTERVAL_DURATION
                                {
                                    is_slam_processing_for_thread.store(true, Ordering::SeqCst);

                                    slam_manager.update(&scan);

                                    slam_result_sender
                                        .send(SlamThreadResult {
                                            map_points: slam_manager.get_map_points().clone(),
                                            robot_pose: *slam_manager.get_robot_pose(),
                                        })
                                        .unwrap_or_default();
                                    last_slam_update_time = now; // 更新時間を記録
                                    is_slam_processing_for_thread.store(false, Ordering::SeqCst);
                                } else {
                                    //
                                }
                            }
                        }
                        SlamThreadCommand::ProcessSingleScan { scan } => {
                            // 単一スキャン要求はモードに関わらずすぐに処理
                            is_slam_processing_for_thread.store(true, Ordering::SeqCst);

                            slam_manager.update(&scan);

                            slam_result_sender
                                .send(SlamThreadResult {
                                    map_points: slam_manager.get_map_points().clone(),
                                    robot_pose: *slam_manager.get_robot_pose(),
                                })
                                .unwrap_or_default();
                            is_slam_processing_for_thread.store(false, Ordering::SeqCst);
                        }
                        SlamThreadCommand::Shutdown => break, // スレッドを終了
                    }
                }

                // 負荷軽減のため短時間スリープ
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
            lidars,                 // 新しいlidarsフィールドを初期化
            lidar_message_receiver, // 単一のLidarメッセージ受信
            pending_scans: [None, None],
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
        }
    }

    pub fn request_save_points(&mut self, path: String) {
        self.requested_point_save_path = Some(path);
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
        // --- データ更新 ---
        while let Ok(lidar_message) = self.lidar_message_receiver.try_recv() {
            match lidar_message {
                LidarMessage::ScanUpdate { id, scan } => {
                    // UI用に点群を更新
                    if let Some(lidar_state) = self.lidars.get_mut(id) {
                        lidar_state.points = scan.clone();
                    }

                    // SLAMモードの場合、スキャンデータを統合する
                    if self.app_mode == AppMode::Slam {
                        if id < self.pending_scans.len() {
                            self.pending_scans[id] = Some(scan);
                        }

                        // 両方のLidarからスキャンデータが届いているか確認
                        if self.pending_scans.iter().all(Option::is_some) {
                            let mut combined_scan = Vec::new();

                            // 各Lidarのスキャンをロボット座標系に変換して結合
                            for (lidar_id, scan_option) in self.pending_scans.iter().enumerate() {
                                if let Some(points) = scan_option {
                                    if let Some(lidar_state) = self.lidars.get(lidar_id) {
                                        let rotation = lidar_state.rotation;
                                        let origin = lidar_state.origin;

                                        for point in points {
                                            // Lidar座標系での点 (px, py)
                                            let pxx = point.0;
                                            let pyy = point.1;
                                            let px_raw = pxx;
                                            let py_raw = pyy;

                                            // Lidarの回転を適用
                                            let px_rotated =
                                                px_raw * rotation.cos() - py_raw * rotation.sin();
                                            let py_rotated =
                                                px_raw * rotation.sin() + py_raw * rotation.cos();

                                            // ワールド座標に変換 (Lidarの原点オフセットを加える)
                                            // SLAMマネージャーは (f32, f32) を期待しているので、egui::Pos2にはしない
                                            let world_x = px_rotated + origin.x;
                                            let world_y = py_rotated + origin.y;

                                            combined_scan.push((world_x, world_y));
                                        }
                                    }
                                }
                            }

                            // 結合した点群をSLAMスレッドに送信

                            // SLAMが処理中でない場合のみ送信

                            if !self.is_slam_processing.load(Ordering::SeqCst) {
                                if self.slam_mode == SlamMode::Continuous {
                                    self.slam_command_sender
                                        .send(SlamThreadCommand::UpdateScan {
                                            scan: combined_scan,
                                        })
                                        .unwrap_or_default();
                                } else if self.single_scan_requested_by_ui {
                                    self.slam_command_sender
                                        .send(SlamThreadCommand::ProcessSingleScan {
                                            scan: combined_scan,
                                        })
                                        .unwrap_or_default();

                                    self.single_scan_requested_by_ui = false; // フラグをリセット
                                }
                            }

                            // 統合後は保留中のスキャンをクリア
                            self.pending_scans = [None, None];
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

        // SLAMスレッドからの結果を受け取る
        while let Ok(slam_result) = self.slam_result_receiver.try_recv() {
            self.current_map_points = slam_result.map_points;
            self.current_robot_pose = slam_result.robot_pose;
            ctx.request_repaint();
        }
        while let Ok(msg) = self.command_output_receiver.try_recv() {
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

        if ctx.input(|i| (i.modifiers.ctrl || i.modifiers.command) && i.key_pressed(egui::Key::L)) {
            self.command_history.clear();
            ctx.request_repaint();
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
                            ui.monospace("> ");
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
                    if text_edit_response.changed() || self.input_string.is_empty() {
                        let input = self.input_string.trim_start();
                        self.current_suggestions.clear(); // 毎回クリアしてから再計算

                        if !input.is_empty() {
                            let parts: Vec<&str> = input.split_whitespace().collect();
                            let ends_with_space = input.ends_with(' ');

                            match parts.as_slice() {
                                ["set", "demo"] if ends_with_space => {
                                    self.current_suggestions = vec![
                                        "scan".to_string(),
                                        "ripple".to_string(),
                                        "breathing".to_string(),
                                        "table".to_string(),
                                    ];
                                }
                                ["set", "demo", partial_arg] => {
                                    let options = vec!["scan", "ripple", "breathing", "table"];
                                    self.current_suggestions = options
                                        .into_iter()
                                        .filter(|opt| opt.starts_with(partial_arg))
                                        .map(|s| s.to_string())
                                        .collect();
                                }
                                ["set"] if ends_with_space => {
                                    self.current_suggestions =
                                        vec!["path".to_string(), "demo".to_string()];
                                }
                                ["set", partial_subcommand] => {
                                    let options = vec!["path", "demo"];
                                    self.current_suggestions = options
                                        .into_iter()
                                        .filter(|opt| opt.starts_with(partial_subcommand))
                                        .map(|s| s.to_string())
                                        .collect();
                                }
                                ["serial"] if ends_with_space => {
                                    self.current_suggestions = vec!["list".to_string()];
                                }
                                ["serial", partial_subcommand] => {
                                    let options = vec!["list"];
                                    self.current_suggestions = options
                                        .into_iter()
                                        .filter(|opt| opt.starts_with(partial_subcommand))
                                        .map(|s| s.to_string())
                                        .collect();
                                }
                                ["save"] if ends_with_space => {
                                    self.current_suggestions =
                                        vec!["image".to_string(), "points".to_string()];
                                }
                                ["save", partial_subcommand] => {
                                    let options = vec!["image", "points"];
                                    self.current_suggestions = options
                                        .into_iter()
                                        .filter(|opt| opt.starts_with(partial_subcommand))
                                        .map(|s| s.to_string())
                                        .collect();
                                }
                                [partial_command] => {
                                    let all_commands = vec![
                                        "help",
                                        "set",
                                        "debug-storage",
                                        "quit",
                                        "q",
                                        "clear",
                                        "serial",
                                        "save",
                                    ];
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

                    // --- サジェスト候補の表示 ---
                    if !self.current_suggestions.is_empty() {
                        let suggestion_text =
                            format!("Suggestions: {}", self.current_suggestions.join(", "));
                        egui::Frame::none()
                            .fill(egui::Color32::from_rgb(40, 40, 40))
                            .inner_margin(egui::Margin::symmetric(5.0, 2.0))
                            .show(ui, |ui| {
                                ui.set_width(ui.available_width());
                                ui.monospace(suggestion_text);
                            });
                    }

                    // --- コマンド履歴ナビゲーション ---
                    if text_edit_response.has_focus() {
                        let up_pressed = ctx.input(|i| i.key_pressed(egui::Key::ArrowUp));
                        if up_pressed && self.history_index > 0 {
                            self.history_index -= 1;
                            self.input_string = self
                                .user_command_history
                                .get(self.history_index)
                                .cloned()
                                .unwrap_or_default();
                        }
                        let down_pressed = ctx.input(|i| i.key_pressed(egui::Key::ArrowDown));
                        if down_pressed && self.history_index < self.user_command_history.len() {
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
                    }

                    // --- コマンド実行 ---
                    if text_edit_response.lost_focus()
                        && ctx.input(|i| i.key_pressed(egui::Key::Enter))
                    {
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
                        text_edit_response.request_focus();
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

                // ワールド座標からスクリーン座標への変換を定義
                // Y軸を反転させるため、fromに渡すRectのYのmin/maxを入れ替える
                let world_to_screen_rect = egui::Rect::from_min_max(
                    egui::pos2(-8.0, 8.0),  // ワールドの左上 (min_x, max_y)
                    egui::pos2(8.0, -8.0)   // ワールドの右下 (max_x, min_y)
                );
                let to_screen = egui::emath::RectTransform::from_to(
                    world_to_screen_rect,
                    square_rect,
                );

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
                        let lidar_color = if i == 0 {
                            egui::Color32::GREEN
                        } else {
                            egui::Color32::YELLOW
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
            AppMode::Slam => {
                ui.heading("SLAM Mode");
                let (response, painter) =
                    ui.allocate_painter(ui.available_size(), egui::Sense::hover());
                let rect = response.rect;
                self.lidar_draw_rect = Some(rect);
                painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 20, 20));

                // Draw robot pose
                let robot_pose = &self.current_robot_pose;
                // ロボットの現在のXY座標を取得 (方向は無視)
                let robot_center_world =
                    egui::pos2(robot_pose.translation.x, robot_pose.translation.y);

                // 描画エリアのワールド座標の範囲 (例: 中心から±5メートル)
                // この範囲の中心をロボットの現在位置に設定
                let map_view_size = 30.0; // ワールド座標で表示する領域のサイズ (例: 30m x 30m)
                let map_view_rect = egui::Rect::from_center_size(
                    robot_center_world,
                    egui::vec2(map_view_size, map_view_size),
                );

                // Y軸を反転させるため、map_view_rectのYのmin/maxを入れ替える
                let mut inverted_map_view_rect = map_view_rect;
                inverted_map_view_rect.min.y = map_view_rect.max.y;
                inverted_map_view_rect.max.y = map_view_rect.min.y;
                let to_screen = egui::emath::RectTransform::from_to(
                    inverted_map_view_rect,
                    rect,          // 実際の描画エリア
                );

                // Draw the map points
                for point in &self.current_map_points {
                    // Right = +X, Up = +Y
                    let screen_pos = to_screen.transform_pos(egui::pos2(point.x, point.y));
                    if rect.contains(screen_pos) {
                        painter.circle_filled(
                            screen_pos,
                            2.0,
                            egui::Color32::from_rgb(100, 100, 255),
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
        });

        ctx.request_repaint();
    }
}
