use anyhow::Result;
use eframe::egui;
use std::sync::mpsc;

use crate::lidar::{LidarInfo, start_lidar_thread};

// アプリケーション全体の状態を管理する構造体
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
}

// Defaultを実装すると、`new`関数内で MyApp::default() が使え、コードが少しきれいになる
impl Default for MyApp {
    fn default() -> Self {
        // チャネルを作成し、データ生成スレッドを開始する
        let (sender, receiver) = mpsc::channel();
        let (status_sender, status_receiver) = mpsc::channel(); // LiDARステータス表示用
        let (command_output_sender, command_output_receiver) = mpsc::channel(); // コマンド出力用

        let lidar_config = LidarInfo {
            lidar_path: "/dev/cu.usbmodem2101".to_string(),
            baud_rate: 115200,
        };
        
        start_lidar_thread(lidar_config.clone(), sender.clone(), status_sender.clone());

        Self {
            input_string: String::new(),
            command_history: vec!["Welcome to the interactive console!".to_string()],
            lidar_points: Vec::new(), // 最初は空
            receiver,                // 生成したレシーバーを格納
            lidar_status_messages: Vec::new(),
            status_receiver,
            command_output_receiver,
            command_output_sender,
            lidar_draw_rect: None, // 初期値はNone
            lidar_path: lidar_config.lidar_path,
            lidar_baud_rate: lidar_config.baud_rate,
            lidar_connection_status: "Connecting...".to_string(),
        }    
    }
}

impl eframe::App for MyApp {
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
            .exact_width(ctx.input(|i| i.screen_rect()).width() / 4.0)
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
        });

        ctx.request_repaint();
    }
}
