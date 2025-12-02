use anyhow::Result;
use eframe::egui;
use eframe::egui::ecolor::Hsva;
use std::sync::mpsc;

use crate::lidar::{LidarInfo, start_lidar_thread};

// アプリケーション全体の状態を管理する構造体
pub enum DemoMode {
    RotatingScan,
}

pub struct MyApp {
    pub(crate) input_string: String,                              // コンソールの入力文字列
    pub(crate) command_history: Vec<String>,                      // コンソールのコマンド履歴
    pub(crate) lidar_points: Vec<(f32, f32)>,                     // 描画用のLiDAR点群データ (x, y)
    pub(crate) receiver: mpsc::Receiver<Result<Vec<(f32, f32)>>>, // データ受信用のレシーバー
    pub(crate) lidar_status_messages: Vec<String>,                // LiDARの状態表示用メッセージ
    pub(crate) status_receiver: mpsc::Receiver<String>,           // LiDARの状態メッセージ受信用のレシーバー
    pub(crate) command_output_receiver: mpsc::Receiver<String>,   // コンソールコマンド出力受信用のレシーバー
    pub(crate) command_output_sender: mpsc::Sender<String>,       // コンソールコマンド出力送信用のセンダー
    pub(crate) lidar_draw_rect: Option<egui::Rect>,               // LiDAR描画エリアのRect
    pub(crate) lidar_path: String,
    pub(crate) lidar_baud_rate: u32,
    pub(crate) lidar_connection_status: String,
    demo_mode: DemoMode,
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

        Self {
            input_string: String::new(),
            command_history: vec!["Welcome to the interactive console!".to_string()],
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
            demo_mode: DemoMode::RotatingScan,
        }
    }

    fn draw_demo_mode(&self, ui: &mut egui::Ui, painter: &egui::Painter, rect: egui::Rect) {
        match self.demo_mode {
            DemoMode::RotatingScan => {
                let time = ui.input(|i| i.time);
                let center = rect.center();
                // 描画半径を矩形の高さの40%に設定
                let radius = rect.height().min(rect.width()) * 0.4;
                // 4秒で1周する角度
                let angle = time as f32 * std::f32::consts::TAU / 4.0; 
    

    
                // 2. グラデーション付きのドーナツ状扇形（スキャン軌跡）
                let scan_angle_width = std::f32::consts::FRAC_PI_3; // 60度
                let num_segments = 60; // 滑らかさのためのセグメント数
                let inner_radius_for_fan = radius * 0.25; // ドーナツの内側の半径

                // スキャン開始角度
                let start_scan_angle = angle - scan_angle_width;
                let mut current_segment_angle = start_scan_angle;

                for i in 0..num_segments {
                    let next_segment_angle = start_scan_angle + scan_angle_width * ((i + 1) as f32 / num_segments as f32);
                    
                    // 外側の円周上の点
                    let p1 = center + radius * egui::vec2(current_segment_angle.cos(), -current_segment_angle.sin());
                    let p2 = center + radius * egui::vec2(next_segment_angle.cos(), -next_segment_angle.sin());

                    // 内側の円周上の点
                    let inner_p1 = center + inner_radius_for_fan * egui::vec2(current_segment_angle.cos(), -current_segment_angle.sin());
                    let inner_p2 = center + inner_radius_for_fan * egui::vec2(next_segment_angle.cos(), -next_segment_angle.sin());

                    // 角度を色相 (0.0..=1.0) にマッピング
                    let hue = current_segment_angle.rem_euclid(std::f32::consts::TAU) / std::f32::consts::TAU;

                    // egui::Color32::from_hsva は存在しないため、Hsva構造体からColor32に変換
                    let color: egui::Color32 = Hsva { h: hue, s: 1.0, v: 1.0, a: 0.1 }.into();

                    // 各セグメント（台形）を2つの三角形で描画
                    painter.add(egui::Shape::convex_polygon(
                        vec![inner_p1, p1, p2], // 外側の三角形
                        color,
                        egui::Stroke::NONE,
                    ));
                    painter.add(egui::Shape::convex_polygon(
                        vec![inner_p1, p2, inner_p2], // 内側の三角形 (台形を構成するもう一つの三角形)
                        color,
                        egui::Stroke::NONE,
                    ));
                    current_segment_angle = next_segment_angle;
                }
                
                // 3. 中央のテキスト
                painter.text(
                    center,
                    egui::Align2::CENTER_CENTER,
                    "Signal Lost",
                    egui::FontId::proportional(40.0),
                    egui::Color32::WHITE,
                );
            }
        }
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
                    self.command_history.push(format!("INFO: {}", e));
                }
            }
            ctx.request_repaint();
        }

        while let Ok(msg) = self.status_receiver.try_recv() {
            let mut handled_as_status = false;
            if msg.starts_with("ERROR: Failed to open port") {
                self.lidar_connection_status = msg.clone();
                handled_as_status = true;
            } else if msg.contains("Successfully opened port") {
                self.lidar_connection_status = "Connected".to_string();
                handled_as_status = true;
            } else if msg.contains("INFO: LiDAR initialized. Laser is ON.") {
                self.lidar_connection_status = "Connected (Laser ON)".to_string();
                handled_as_status = true;
            }

            if !handled_as_status && !msg.starts_with("LiDAR Path:") && !msg.starts_with("Baud Rate:") {
                self.lidar_status_messages.push(msg);
                if self.lidar_status_messages.len() > 10 {
                    self.lidar_status_messages.remove(0);
                }
            }
            ctx.request_repaint();
        }

        while let Ok(msg) = self.command_output_receiver.try_recv() {
            self.command_history.push(msg);
            ctx.request_repaint();
        }

        let console_input_id = egui::Id::new("console_input");
        let enter_pressed_while_unfocused = ctx.input(|i| i.key_pressed(egui::Key::Enter)) && !ctx.wants_keyboard_input();

        if ctx.input(|i| (i.modifiers.ctrl || i.modifiers.command) && i.key_pressed(egui::Key::L)) {
            self.command_history.clear();
            ctx.request_repaint();
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
                if background_response.clicked() || enter_pressed_while_unfocused {
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
                egui::ScrollArea::vertical()
                    .stick_to_bottom(true)
                    .max_height(ui.available_height() * 0.8)
                    .show(ui, |ui| {
                        for line in &self.command_history {
                            ui.monospace(line);
                        }
                    });
                ui.horizontal(|ui| {
                    ui.monospace("> ");
                    let text_edit_response = ui.add(
                        egui::TextEdit::singleline(&mut self.input_string)
                            .id(console_input_id)
                            .frame(false)
                            .hint_text("Enter command...")
                            .font(egui::TextStyle::Monospace)
                            .lock_focus(true),
                    );
                    if text_edit_response.lost_focus() && ctx.input(|i| i.key_pressed(egui::Key::Enter)) {
                        let full_command_line_owned = self.input_string.trim().to_owned(); // Fix: Clone to String
                        if !full_command_line_owned.is_empty() {
                            self.command_history.push(format!("> {}", full_command_line_owned));
                            crate::cli::handle_command(self, ctx, &full_command_line_owned); // Fix: Pass reference
                        }
                        self.input_string.clear();
                        text_edit_response.request_focus();
                    }
                });
            });

        // 2. 右側のパネル（グラフィック表示）
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("LiDAR Data Visualization");
            let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
            self.lidar_draw_rect = Some(response.rect);

            // LiDARの接続状態で描画を切り替える
            if self.lidar_connection_status.starts_with("Connected") {
                // --- LiDAR接続時：点群を描画 ---
                let side = response.rect.height();
                let square_rect = egui::Rect::from_center_size(response.rect.center(), egui::vec2(side, side));
                let to_screen = egui::emath::RectTransform::from_to(
                    egui::Rect::from_center_size(egui::Pos2::ZERO, egui::vec2(16.0, 16.0)),
                    square_rect,
                );
                painter.rect_filled(square_rect, 0.0, egui::Color32::from_rgb(20, 20, 20));
                painter.hline(square_rect.x_range(), to_screen.transform_pos(egui::pos2(0.0, 0.0)).y, egui::Stroke::new(0.5, egui::Color32::DARK_GRAY));
                painter.vline(to_screen.transform_pos(egui::pos2(0.0, 0.0)).x, square_rect.y_range(), egui::Stroke::new(0.5, egui::Color32::DARK_GRAY));
                for point in &self.lidar_points {
                    let screen_pos = to_screen.transform_pos(egui::pos2(point.0, point.1));
                    if square_rect.contains(screen_pos) {
                        painter.circle_filled(screen_pos, 2.0, egui::Color32::GREEN);
                    }
                }
                let robot_pos = to_screen.transform_pos(egui::Pos2::ZERO);
                painter.circle_filled(robot_pos, 5.0, egui::Color32::RED);
            } else {
                // --- LiDAR未接続時：デモ画面を描画 ---
                painter.rect_filled(response.rect, 0.0, egui::Color32::from_rgb(20, 20, 20));
                self.draw_demo_mode(ui, &painter, response.rect);
            }
        });

        ctx.request_repaint();
    }
}
