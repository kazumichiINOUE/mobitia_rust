use anyhow::Result;
use clap::Parser;
use eframe::egui;
use eframe::egui::ecolor::Hsva;
use rand::Rng;
use std::io::Write;
use std::sync::mpsc;
use std::thread;

use crate::cli::Cli;
use crate::lidar::{start_lidar_thread, LidarInfo};
use crate::slam::SlamManager;

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

// アプリケーション全体の状態を管理する構造体
pub enum DemoMode {
    RotatingScan,
    ExpandingRipple,
    BreathingCircle,
    Table,
}

pub struct ConsoleOutputEntry {
    pub text: String,
    pub group_id: usize, // どのコマンド実行に属するか
}
pub struct Ripple {
    center: egui::Pos2,
    spawn_time: f64,
    max_radius: f32,
    duration: f32, // 波紋の寿命
}

pub struct MyApp {
    pub(crate) input_string: String, // コンソールの入力文字列
    pub(crate) command_history: Vec<ConsoleOutputEntry>, // コンソールの表示履歴
    pub(crate) user_command_history: Vec<String>, // ユーザーのコマンド入力履歴
    pub(crate) history_index: usize, // コマンド履歴のインデックス
    pub(crate) current_suggestions: Vec<String>, // 現在のサジェスト候補
    pub(crate) lidar_points: Vec<(f32, f32)>, // 描画用のLiDAR点群データ (x, y)
    pub(crate) receiver: mpsc::Receiver<Result<Vec<(f32, f32)>>>, // データ受信用のレシーバー
    pub(crate) lidar_status_messages: Vec<String>, // LiDARの状態表示用メッセージ
    pub(crate) status_receiver: mpsc::Receiver<String>, // LiDARの状態メッセージ受信用のレシーバー
    pub(crate) command_output_receiver: mpsc::Receiver<String>, // コンソールコマンド出力受信用のレシーバー
    pub(crate) command_output_sender: mpsc::Sender<String>, // コンソールコマンド出力送信用のセンダー
    pub(crate) lidar_draw_rect: Option<egui::Rect>,         // LiDAR描画エリアのRect
    pub(crate) lidar_path: String,
    pub(crate) lidar_baud_rate: u32,
    pub(crate) lidar_connection_status: String,
    pub(crate) app_mode: AppMode,
    pub(crate) demo_mode: DemoMode,
    pub(crate) slam_manager: SlamManager,
    pub(crate) slam_request_scan: bool,
    ripples: Vec<Ripple>,
    last_ripple_spawn_time: f64,
    pub(crate) show_command_window: bool,
    pub(crate) focus_console_requested: bool,
    pub(crate) requested_point_save_path: Option<String>,
    pub(crate) next_group_id: usize,

    pub(crate) slam_mode: SlamMode,
    pub(crate) last_slam_update: f64,
}

impl MyApp {
    /// Creates a new instance of the application.
    pub fn new(cc: &eframe::CreationContext) -> Self {
        // eframe::Storageからlidar_pathを読み込む
        let lidar_path: String = cc
            .storage
            .and_then(|storage| storage.get_string("lidar_path"))
            .unwrap_or_else(|| "/dev/cu.usbmodem2101".to_string());

        // チャネルを作成し、データ生成スレッドを開始する
        let (sender, receiver) = mpsc::channel();
        let (status_sender, status_receiver) = mpsc::channel();
        let (command_output_sender, command_output_receiver) = mpsc::channel();

        let lidar_config = LidarInfo {
            lidar_path: lidar_path.clone(), // 読み込んだパスを使用
            baud_rate: 115200,
        };

        start_lidar_thread(lidar_config.clone(), sender.clone(), status_sender.clone());

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
            current_suggestions: Vec::new(), // 初期化
            lidar_points: Vec::new(),
            receiver,
            lidar_status_messages: Vec::new(),
            status_receiver,
            command_output_receiver,
            command_output_sender,
            lidar_draw_rect: None,
            lidar_path: lidar_config.lidar_path,
            lidar_baud_rate: lidar_config.baud_rate,
            lidar_connection_status: "Connecting...".to_string(),
            app_mode: AppMode::Lidar, // Default to Lidar mode
            demo_mode: DemoMode::RotatingScan,
            slam_manager: SlamManager::new(),
            slam_request_scan: false,
            ripples: Vec::new(),
            last_ripple_spawn_time: 0.0,
            show_command_window: true,
            focus_console_requested: true,
            requested_point_save_path: None,
            next_group_id: 0,
            slam_mode: SlamMode::Manual,
            last_slam_update: 0.0,
        }
    }

    pub fn request_save_points(&mut self, path: String) {
        self.requested_point_save_path = Some(path);
    }
}

impl eframe::App for MyApp {
    /// Called to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        storage.set_string("lidar_path", self.lidar_path.clone());
    }

    /// フレームごとに呼ばれ、UIを描画する
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // --- データ更新 ---
        if let Ok(result) = self.receiver.try_recv() {
            match result {
                Ok(points) => {
                    self.lidar_points = points;
                }
                Err(e) => {
                    self.command_history.push(ConsoleOutputEntry {
                        text: format!("INFO: {}", e),
                        group_id: self.next_group_id,
                    });
                }
            }
            ctx.request_repaint();
        }

        if self.slam_request_scan {
            if !self.lidar_points.is_empty() {
                self.slam_manager.update(&self.lidar_points);
            }
            self.slam_request_scan = false;
        }

        if self.app_mode == AppMode::Slam && self.slam_mode == SlamMode::Continuous {
            let current_time = ctx.input(|i| i.time);
            const UPDATE_INTERVAL: f64 = 1.0; // 1秒ごとに更新

            if current_time - self.last_slam_update >= UPDATE_INTERVAL {
                if !self.lidar_points.is_empty() {
                    self.slam_manager.update(&self.lidar_points);
                    self.last_slam_update = current_time; // 更新時間を記録
                }
                ctx.request_repaint(); // SLAM更新後に再描画を要求
            }
        }

        while let Ok(msg) = self.status_receiver.try_recv() {
            let mut handled_as_status = false;
            if msg.starts_with("ERROR: Failed to open port") {
                self.lidar_connection_status = msg.clone();
                handled_as_status = true;
            } else if msg.starts_with("ERROR: Failed to get distance data") {
                self.lidar_connection_status = "Connection Lost".to_string();
                // handled_as_status は false のままにして、詳細ログにもメッセージが表示されるようにする
            } else if msg.contains("Successfully opened port") {
                self.lidar_connection_status = "Connected".to_string();
                handled_as_status = true;
            } else if msg.contains("INFO: LiDAR initialized. Laser is ON.") {
                self.lidar_connection_status = "Connected (Laser ON)".to_string();
                handled_as_status = true;
            }

            if !handled_as_status
                && !msg.starts_with("LiDAR Path:")
                && !msg.starts_with("Baud Rate:")
            {
                self.lidar_status_messages.push(msg);
                if self.lidar_status_messages.len() > 10 {
                    self.lidar_status_messages.remove(0);
                }
            }
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
            let lidar_points_clone = self.lidar_points.clone();
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
                                .send(format!("ERROR: Failed to write to file '{}': {}", path, e))
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

                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.label(format!("Device Path: {}", self.lidar_path));
                    ui.label(format!("Baud Rate: {}", self.lidar_baud_rate));
                    let status_text = format!("Status: {}", self.lidar_connection_status);
                    let status_color = if self.lidar_connection_status.starts_with("Connected") {
                        egui::Color32::GREEN
                    } else if self.lidar_connection_status.starts_with("Connecting") {
                        egui::Color32::YELLOW
                    } else {
                        egui::Color32::RED
                    };
                    ui.label(egui::RichText::new(status_text).color(status_color));
                });
                ui.separator();

                ui.group(|ui| {
                    ui.set_width(ui.available_width());
                    ui.set_height(ui.text_style_height(&egui::TextStyle::Monospace) * 10.0);
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        for line in &self.lidar_status_messages {
                            ui.monospace(line);
                        }
                    });
                });
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

                if self.lidar_connection_status.starts_with("Connected") {
                    // --- LiDAR接続時：点群を描画 (Painterベース) ---
                    let side = rect.height();
                    let square_rect =
                        egui::Rect::from_center_size(rect.center(), egui::vec2(side, side));
                    let to_screen = egui::emath::RectTransform::from_to(
                        egui::Rect::from_center_size(egui::Pos2::ZERO, egui::vec2(16.0, 16.0)),
                        square_rect,
                    );
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
                    for point in &self.lidar_points {
                        let pxx = point.0;
                        let pyy = point.1;
                        let px = -pyy;
                        let py = -pxx;
                        let screen_pos = to_screen.transform_pos(egui::pos2(px, py));
                        if square_rect.contains(screen_pos) {
                            painter.circle_filled(screen_pos, 2.0, egui::Color32::GREEN);
                        }
                    }
                    let robot_pos = to_screen.transform_pos(egui::Pos2::ZERO);
                    painter.circle_filled(robot_pos, 5.0, egui::Color32::RED);
                } else {
                    // --- LiDAR未接続時：メッセージを表示 ---
                    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 20, 20));
                    ui.allocate_ui_at_rect(rect, |ui| {
                        ui.centered_and_justified(|ui| {
                            let text = egui::RichText::new(&self.lidar_connection_status)
                                .color(egui::Color32::WHITE)
                                .font(egui::FontId::proportional(24.0)); // Reduced font size
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

                let to_screen = egui::emath::RectTransform::from_to(
                    egui::Rect::from_center_size(egui::Pos2::ZERO, egui::vec2(10.0, 10.0)),
                    rect,
                );

                // Draw the map points
                for point in self.slam_manager.get_map_points() {
                    // Right = +X, Up = +Y
                    let screen_pos = to_screen.transform_pos(egui::pos2(point.x, -point.y));
                    if rect.contains(screen_pos) {
                        painter.circle_filled(screen_pos, 2.0, egui::Color32::from_rgb(100, 100, 255));
                    }
                }

                // Draw robot pose
                let robot_pose = self.slam_manager.get_robot_pose();
                let robot_pos = to_screen.transform_pos(egui::pos2(robot_pose.translation.x, -robot_pose.translation.y));
                let angle = robot_pose.rotation.angle();
                painter.circle_filled(robot_pos, 5.0, egui::Color32::RED);
                painter.line_segment(
                    [
                        robot_pos,
                        robot_pos + egui::vec2(angle.cos(), -angle.sin()) * 20.0,
                    ],
                    egui::Stroke::new(2.0, egui::Color32::RED),
                );
            }
            AppMode::Demo => {
                let heading_text = match self.demo_mode {
                    DemoMode::RotatingScan => "Rotating Scan Demo",
                    DemoMode::ExpandingRipple => "Expanding Ripple Demo",
                    DemoMode::BreathingCircle => "Breathing Circle Demo",
                    DemoMode::Table => "Table Demo",
                };
                ui.heading(heading_text);

                ui.painter().rect_filled(
                    ui.available_rect_before_wrap(),
                    0.0,
                    egui::Color32::from_rgb(20, 20, 20),
                );

                match self.demo_mode {
                    DemoMode::Table => {
                        // --- Widgetベースのデモ ---
                        ui.centered_and_justified(|ui| {
                            egui::Grid::new("demo_table_grid")
                                .num_columns(2)
                                .spacing([40.0, 4.0])
                                .striped(true)
                                .show(ui, |ui| {
                                    ui.label("Parameter 1:");
                                    ui.label("Value A");
                                    ui.end_row();

                                    ui.label("Parameter 2:");
                                    ui.label("Value B");
                                    ui.end_row();

                                    ui.label("A much longer parameter name:");
                                    ui.label("Some other value C");
                                    ui.end_row();

                                    ui.label("Status:");
                                    ui.label(egui::RichText::new("OK").color(egui::Color32::GREEN));
                                    ui.end_row();
                                });
                        });
                    }
                    DemoMode::RotatingScan => {
                        let (response, painter) =
                            ui.allocate_painter(ui.available_size(), egui::Sense::hover());
                        let rect = response.rect;
                        self.lidar_draw_rect = Some(rect);

                        let time = ui.input(|i| i.time);
                        let center = rect.center();
                        let radius = rect.height().min(rect.width()) * 0.4;
                        let angle = time as f32 * std::f32::consts::TAU / 4.0;

                        let scan_angle_width = std::f32::consts::FRAC_PI_3;
                        let num_segments = 60;
                        let inner_radius_for_fan = radius * 0.25;

                        let start_scan_angle = angle - scan_angle_width;
                        let mut current_segment_angle = start_scan_angle;

                        for i in 0..num_segments {
                            let next_segment_angle = start_scan_angle
                                + scan_angle_width * ((i + 1) as f32 / num_segments as f32);

                            let p1 = center
                                + radius
                                    * egui::vec2(
                                        current_segment_angle.cos(),
                                        -current_segment_angle.sin(),
                                    );
                            let p2 = center
                                + radius
                                    * egui::vec2(
                                        next_segment_angle.cos(),
                                        -next_segment_angle.sin(),
                                    );

                            let inner_p1 = center
                                + inner_radius_for_fan
                                    * egui::vec2(
                                        current_segment_angle.cos(),
                                        -current_segment_angle.sin(),
                                    );
                            let inner_p2 = center
                                + inner_radius_for_fan
                                    * egui::vec2(
                                        next_segment_angle.cos(),
                                        -next_segment_angle.sin(),
                                    );

                            let hue = current_segment_angle.rem_euclid(std::f32::consts::TAU)
                                / std::f32::consts::TAU;

                            let color: egui::Color32 = Hsva {
                                h: hue,
                                s: 1.0,
                                v: 1.0,
                                a: 0.1,
                            }
                            .into();

                            painter.add(egui::Shape::convex_polygon(
                                vec![inner_p1, p1, p2],
                                color,
                                egui::Stroke::NONE,
                            ));
                            painter.add(egui::Shape::convex_polygon(
                                vec![inner_p1, p2, inner_p2],
                                color,
                                egui::Stroke::NONE,
                            ));
                            current_segment_angle = next_segment_angle;
                        }

                        painter.text(
                            center,
                            egui::Align2::CENTER_CENTER,
                            "Signal Lost",
                            egui::FontId::proportional(40.0),
                            egui::Color32::WHITE,
                        );
                    }
                    DemoMode::ExpandingRipple => {
                        let (response, painter) =
                            ui.allocate_painter(ui.available_size(), egui::Sense::hover());
                        let rect = response.rect;
                        self.lidar_draw_rect = Some(rect);

                        let time = ui.input(|i| i.time);
                        let mut rng = rand::thread_rng();

                        if time - self.last_ripple_spawn_time > 0.4 {
                            self.last_ripple_spawn_time = time;
                            let center_x = rng.gen_range(rect.left()..=rect.right());
                            let center_y = rng.gen_range(rect.top()..=rect.bottom());
                            let center = egui::pos2(center_x, center_y);

                            self.ripples.push(Ripple {
                                center,
                                spawn_time: time,
                                max_radius: rng.gen_range(50.0..=300.0),
                                duration: rng.gen_range(2.0..=5.0),
                            });
                        }

                        let current_ripples = std::mem::take(&mut self.ripples);
                        self.ripples = current_ripples
                            .into_iter()
                            .filter_map(|ripple| {
                                let elapsed_time = (time - ripple.spawn_time) as f32;
                                if elapsed_time > ripple.duration {
                                    return None;
                                }

                                let progress = elapsed_time / ripple.duration;
                                let current_radius = ripple.max_radius * progress;

                                let alpha = (1.0 - progress).powf(2.0) * 1.0;
                                let stroke_alpha = (alpha * 255.0) as u8;

                                let color = egui::Color32::from_rgba_unmultiplied(
                                    0,
                                    200,
                                    255,
                                    stroke_alpha,
                                );

                                let stroke_width = (1.0 - progress) * 3.5 + 0.5;

                                painter.circle_stroke(
                                    ripple.center,
                                    current_radius,
                                    egui::Stroke::new(stroke_width, color),
                                );
                                ui.ctx().request_repaint();

                                Some(ripple)
                            })
                            .collect();

                        painter.text(
                            rect.center(),
                            egui::Align2::CENTER_CENTER,
                            "Expanding Ripple Demo",
                            egui::FontId::proportional(40.0),
                            egui::Color32::WHITE,
                        );
                    }
                    DemoMode::BreathingCircle => {
                        let (response, painter) =
                            ui.allocate_painter(ui.available_size(), egui::Sense::hover());
                        let rect = response.rect;
                        self.lidar_draw_rect = Some(rect);

                        let time = ui.input(|i| i.time) as f32;
                        let center = rect.center();

                        let cycle_duration = 8.0;
                        let progress =
                            (time / cycle_duration * std::f32::consts::TAU).cos() * -0.5 + 0.5;

                        let min_radius_ratio = 0.10;
                        let max_radius_ratio = 0.40;
                        let radius_ratio =
                            min_radius_ratio + (max_radius_ratio - min_radius_ratio) * progress;
                        let radius = rect.height().min(rect.width()) * radius_ratio;

                        let min_alpha = 0.3;
                        let max_alpha = 0.8;
                        let alpha = min_alpha + (max_alpha - min_alpha) * progress;

                        let color = egui::Color32::from_rgba_unmultiplied(
                            128,
                            128,
                            128,
                            (alpha * 255.0) as u8,
                        );

                        painter.circle_filled(center, radius, color);
                        ui.ctx().request_repaint();

                        painter.text(
                            center,
                            egui::Align2::CENTER_CENTER,
                            "Breathing Circle Demo",
                            egui::FontId::proportional(40.0),
                            egui::Color32::WHITE,
                        );
                    }
                }
            }
        });

        ctx.request_repaint();
    }
}
