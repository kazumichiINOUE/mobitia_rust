use anyhow::Result;
use eframe::egui;
use serialport::ClearBuffer;
use std::io::{self, Write};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

#[derive(Debug, Clone)]
pub struct LidarInfo {
    pub lidar_path: String,
    pub baud_rate: u32,
}

fn send_and_receive(port: &mut Box<dyn serialport::SerialPort>, command: &[u8]) -> Result<String> {
    port.write_all(command)?;
    port.clear(ClearBuffer::Input)?;
    let mut buf = Vec::new();
    let mut serial_buf = [0; 1];
    loop {
        match port.read(&mut serial_buf) {
            Ok(bytes) => {
                if bytes > 0 {
                    buf.extend_from_slice(&serial_buf[..bytes]);
                    // 応答の終わりを判定する簡易的な方法。実際のプロトコルに合わせて調整が必要。
                    if buf.ends_with(b"\n\n") {
                        break;
                    }
                } else {
                    // データがもう来ない場合
                    break;
                }
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                // タイムアウトはエラーではなく、単にデータが来ていないだけ
                break;
            }
            Err(e) => return Err(e.into()),
        }
    }
    // ★ここから追加
    Ok(String::from_utf8_lossy(&buf).to_string())
}

// Helper function to decode a 4-character SCIP2.0 encoded value
fn decode_scip_2_0_4char(encoded_val: &str) -> Result<u32, anyhow::Error> {
    if encoded_val.len() != 4 {
        return Err(anyhow::anyhow!("Invalid 4-character encoded value length: {}", encoded_val.len()));
    }
    let mut decoded_u32: u32 = 0;
    for (i, c) in encoded_val.chars().enumerate() {
        let val = (c as u32).saturating_sub(0x30); // Subtract ASCII '0'
        if val > 0x3F { // Max 6-bit value
            return Err(anyhow::anyhow!("Invalid SCIP2.0 character: {}", c));
        }
        decoded_u32 |= val << (6 * (3 - i)); // Shift according to position (MSB first)
    }
    Ok(decoded_u32)
}

// Helper function to decode a 3-character SCIP2.0 encoded value (18-bit)
fn decode_scip_2_0_3char(encoded_val: &str) -> Result<u32, anyhow::Error> {
    if encoded_val.len() != 3 {
        return Err(anyhow::anyhow!("Invalid 3-character encoded value length: {}", encoded_val.len()));
    }
    let mut decoded_u32: u32 = 0;
    for (i, c) in encoded_val.chars().enumerate() {
        let val = (c as u32).saturating_sub(0x30); // Subtract ASCII '0'
        if val > 0x3F { // Max 6-bit value
            return Err(anyhow::anyhow!("Invalid SCIP2.0 character: {}", c));
        }
        decoded_u32 |= val << (6 * (2 - i)); // Shift according to position (MSB first)
    }
    Ok(decoded_u32)
}

/// アプリケーション全体の状態を管理する構造体
struct MyApp {
    // コンソールの入力文字列
    input_string: String,
    // コンソールのコマンド履歴
    command_history: Vec<String>,
    // 描画用のLiDAR点群データ (x, y)
    lidar_points: Vec<(f32, f32)>,
    // データ受信用のレシーバー
    receiver: mpsc::Receiver<Result<Vec<(f32, f32)>>>,
    // 初期サイズ設定を一度だけ行うためのフラグ
    should_set_initial_size: bool,
}

// Defaultを実装すると、`new`関数内で MyApp::default() が使え、コードが少しきれいになる
impl Default for MyApp {
    fn default() -> Self {
        // チャネルを作成し、データ生成スレッドを開始する
        let (sender, receiver) = mpsc::channel();
        thread::spawn(move || {
            // --- LiDAR初期接続ロジック ---
            let lidar_config = LidarInfo {
                lidar_path: "/dev/cu.usbmodem2101".to_string(), // 注意: 環境に合わせて変更が必要
                baud_rate: 115200,
            };

            let mut port = match serialport::new(&lidar_config.lidar_path, lidar_config.baud_rate)
                .timeout(Duration::from_millis(100)) // タイムアウトを短めに設定
                .open()
            {
                Ok(p) => {
                    let msg = format!("INFO: Successfully opened port {}", lidar_config.lidar_path);
                    sender.send(Err(anyhow::anyhow!(msg))).unwrap_or_default();
                    p
                }
                Err(e) => {
                    sender
                        .send(Err(anyhow::anyhow!(
                            "ERROR: Failed to open port '{}': {}",
                            lidar_config.lidar_path,
                            e
                        )))
                        .unwrap_or_default();
                    // ポートが開けなかったらスレッド終了
                    return;
                }
            };

            // 初期化コマンドを送信
            let init_commands: &[&[u8]] = &[b"VV\n", b"PP\n", b"II\n"];
            for cmd in init_commands {
                sender.send(Err(anyhow::anyhow!(format!("INFO: Sending command: {}", String::from_utf8_lossy(*cmd).trim())))).unwrap_or_default();
                let response = match send_and_receive(&mut port, *cmd) {
                    Ok(resp) => resp,
                    Err(e) => {
                        sender
                            .send(Err(anyhow::anyhow!(format!("ERROR: Failed to send command {}: {}", String::from_utf8_lossy(*cmd).trim(), e))))
                            .unwrap_or_default();
                        return; // 初期化失敗時もスレッド終了
                    }
                };
                sender.send(Err(anyhow::anyhow!(format!("INFO: Response for {}: {}", String::from_utf8_lossy(*cmd).trim(), response.trim())))).unwrap_or_default();
                thread::sleep(Duration::from_millis(50)); // コマンド間に少し待機
            }

            // BM コマンドを別に送信
            sender.send(Err(anyhow::anyhow!(format!("INFO: Sending command: {}", String::from_utf8_lossy(b"BM\n").trim())))).unwrap_or_default();
            let response_bm = match send_and_receive(&mut port, b"BM\n") {
                Ok(resp) => resp,
                Err(e) => {
                    sender
                        .send(Err(anyhow::anyhow!(format!("ERROR: Failed to send command BM: {}", e))))
                        .unwrap_or_default();
                    return;
                }
            };
            sender.send(Err(anyhow::anyhow!(format!("INFO: Response for BM: {}", response_bm.trim())))).unwrap_or_default();
            thread::sleep(Duration::from_millis(50)); // コマンド間に少し待機
            sender.send(Err(anyhow::anyhow!("INFO: LiDAR initialized. Laser is ON.")))
                .unwrap_or_default();
            // --- ここまでLiDAR初期接続ロジック ---

            // --- ここからLiDARデータ取得ロジック (GDコマンド) ---
            loop {
                // GDコマンドを送信してLiDARデータを取得
                let command = b"GD0000108001\n";
                sender.send(Err(anyhow::anyhow!(format!("INFO: Sending command: {}", String::from_utf8_lossy(command).trim())))).unwrap_or_default();

                let response = match send_and_receive(&mut port, command) {
                    Ok(resp) => resp,
                    Err(e) => {
                        sender
                            .send(Err(anyhow::anyhow!(format!("ERROR: Failed to send GD command or receive response: {}", e))))
                            .unwrap_or_default();
                        break;
                    }
                };

                // 応答を解析してタイムスタンプを抽出
                let lines: Vec<&str> = response.trim().lines().collect();
                // GDコマンドのエコー、ステータス行、タイムスタンプ行が最低限必要
                if lines.len() >= 3 { 
                    // GDコマンドのエコーバック
                    sender.send(Err(anyhow::anyhow!(format!("GD Echo: {}", lines[0]))))
                        .unwrap_or_default();

                    // ステータス行 (2バイト)
                    let status_line = lines[1];
                    if status_line.len() >= 2 { // ステータスは2文字
                        sender.send(Err(anyhow::anyhow!(format!("Status: {}", &status_line[0..2]))))
                            .unwrap_or_default();
                    } else {
                        sender.send(Err(anyhow::anyhow!(format!("ERROR: Status line too short: '{}'", status_line))))
                            .unwrap_or_default();
                    }


                    // タイムスタンプ行からタイムスタンプ部分を抽出
                    let timestamp_line = lines[2]; // 3行目 (index 2) がタイムスタンプ
                    if timestamp_line.len() >= 4 { // タイムスタンプは4文字
                        let encoded_timestamp = &timestamp_line[0..4];
                        match decode_scip_2_0_4char(encoded_timestamp) {
                            Ok(timestamp_value) => {
                                sender.send(Err(anyhow::anyhow!(format!("Decoded Timestamp: {}", timestamp_value))))
                                    .unwrap_or_default();
                            }
                            Err(e) => {
                                sender.send(Err(anyhow::anyhow!(format!("ERROR: Failed to decode timestamp: {}", e))))
                                    .unwrap_or_default();
                            }
                        }
                    } else {
                        sender.send(Err(anyhow::anyhow!(format!("ERROR: Timestamp line too short: '{}'", timestamp_line))))
                            .unwrap_or_default();
                        continue; // 次のループへ
                    }

                    // データブロックの解析
                    let mut lidar_points_current_scan: Vec<(f32, f32)> = Vec::new();
                    // GDコマンドの応答からStart/End Angle, Cluster Countを解析
                    // GDxxxxSTEECC, ST:Start Step, EE:End Step, CC:Cluster Count
                    // 今回はGD0000108001なので、0000 -> 0, 1080 -> 270度, 01 -> 1クラスタ
                    // Start Angle: (Start Step - 384) / 4 (degrees)
                    // End Angle: (End Step - 384) / 4 (degrees)
                    // Angle Increment: 360 / Total Steps (degrees)
                    // HOKUYO-UXM-30LNの場合、スキャン範囲は270度、0.25度ステップ

                    // GDコマンドのパラメータを解析
                    let gd_params = &command[2..10]; // "00001080"
                    let start_step = u32::from_str_radix(&String::from_utf8_lossy(&gd_params[0..4]), 10).unwrap_or(0);
                    let end_step = u32::from_str_radix(&String::from_utf8_lossy(&gd_params[4..8]), 10).unwrap_or(0);
                    
                    // Lidarの仕様から角度範囲を計算
                    // (ステップ番号 - 384) * 0.25
                    let min_angle = (start_step as f32 - 384.0) * 0.25_f32;
                    let max_angle = (end_step as f32 - 384.0) * 0.25_f32;
                    
                    let angle_increment = 0.25_f32; // 0.25度 per step

                    let mut current_angle = -135.0_f32; // 最初の角度はmin_angleから開始

                    // タイムスタンプ以降の行からデータ部分を抽出し、一つの文字列に結合
                    let mut all_data_chars = String::new();
                    for data_line_with_checksum in lines.iter().skip(3) { // Skip echo, status, timestamp
                        // 各データラインの最後の1バイトはチェックサムなので無視
                        let data_line = &data_line_with_checksum[0..data_line_with_checksum.len() - 1];
                        all_data_chars.push_str(data_line);
                    }

                    // 結合されたデータ文字列を3文字ずつデコード
                    for (i, chunk) in all_data_chars.as_bytes().chunks(3).enumerate() {

                        let encoded_distance = String::from_utf8_lossy(chunk);
                        if encoded_distance.len() != 3 {
                            sender.send(Err(anyhow::anyhow!(format!("ERROR: Invalid 3-char encoded distance length: '{}' at chunk index {}", encoded_distance, i))))
                                .unwrap_or_default();
                            current_angle += angle_increment; // ここでも角度をインクリメントしないとずれる
                            continue;
                        }
                        match decode_scip_2_0_3char(&encoded_distance) {
                            Ok(distance_mm) => {
                                //println!("{} {:?} {:?} {}", i, chunk, encoded_distance, distance_mm);
                                //std::process::exit(0);
                                // 距離0は測定不能または無効なデータと判断
                                // また、最大角度を超えた場合は終了 (LiDARの範囲外データ)
                                if distance_mm == 0 || current_angle > max_angle {
                                    current_angle += angle_increment;
                                    continue;
                                }

                                if distance_mm > 300000 {
                                    current_angle += angle_increment;
                                    continue;
                                }

                                let distance_m = distance_mm as f32 / 1000.0; // mmをmに変換
                                let angle_rad = current_angle.to_radians();

                                let xx = distance_m * angle_rad.cos();
                                let yy = distance_m * angle_rad.sin();
                                
                                let x = -yy;
                                let y = -xx;

                                lidar_points_current_scan.push((x, y));
                            }
                            Err(e) => {
                                sender.send(Err(anyhow::anyhow!(format!("ERROR: Failed to decode distance '{}' at chunk index {}: {}", encoded_distance, i, e))))
                                    .unwrap_or_default();
                            }
                        }
                        current_angle += angle_increment;
                    }

                    sender.send(Ok(lidar_points_current_scan.clone()))
                        .unwrap_or_default();

                    // ★ここから追加: 初回の点群データをファイルに保存し、終了する一時的なコード
                    let filename = "/Users/kaz/src/mobitia/lidar_points_capture.txt"; // プロジェクトルートディレクトリに保存
                    let mut file = match std::fs::File::create(filename) {
                        Ok(f) => f,
                        Err(e) => {
                            sender.send(Err(anyhow::anyhow!(format!("ERROR: Failed to create file {}: {}", filename, e))))
                                .unwrap_or_default();
                            break; // エラーが発生した場合はループを抜ける
                        }
                    };

                    for point in &lidar_points_current_scan {
                        if let Err(e) = writeln!(file, "{} {}", point.0, point.1) {
                            sender.send(Err(anyhow::anyhow!(format!("ERROR: Failed to write to file {}: {}", filename, e))))
                                .unwrap_or_default();
                            break; // エラーが発生した場合はループを抜ける
                        }
                    }
                    sender.send(Err(anyhow::anyhow!(format!("INFO: First point cloud saved to {}. Terminating.", filename))))
                        .unwrap_or_default();
                    
                    // 生データの出力はコメントアウト
                    // sender.send(Err(anyhow::anyhow!(format!("RAW GD RESPONSE: {}", response))))
                    //       .unwrap_or_default();

                } else {
                    sender.send(Err(anyhow::anyhow!(format!("ERROR: Malformed GD response (expected at least 3 lines): {}", response))))
                        .unwrap_or_default();
                }
                thread::sleep(Duration::from_millis(100)); 
            }

            // スレッド終了前にレーザーをオフにする
            if let Err(e) = send_and_receive(&mut port, b"QT\n") {
                eprintln!("Failed to send QT command: {}", e);
            }
        });

        Self {
            input_string: String::new(),
            command_history: vec!["Welcome to the interactive console!".to_string()],
            lidar_points: Vec::new(), // 最初は空
            receiver,                // 生成したレシーバーを格納
            should_set_initial_size: true,
        }
    }
}

impl eframe::App for MyApp {
    /// フレームごとに呼ばれ、UIを描画する
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // --- 初期サイズを一度だけ設定 ---
        if self.should_set_initial_size {
            // 先にコンソールパネルを描画して、その高さを取得する
            let panel_response = egui::TopBottomPanel::bottom("terminal")
                .resizable(true)
                .min_height(200.0)
                .show(ctx, |_ui| {
                    // この時点では、サイズ計算のためにダミーで描画するだけ
                });

            let console_height = panel_response.response.rect.height();
            let client_rect = ctx.input(|i| i.screen_rect());

            // 理想的なウィンドウ幅を計算 (クライアント領域の高さ - コンソールの高さ)
            let ideal_width = client_rect.height() - console_height;
            let new_size = egui::vec2(ideal_width, client_rect.height());

            // ウィンドウをリサイズするコマンドを送信
            ctx.send_viewport_cmd(egui::ViewportCommand::InnerSize(new_size));
            // ウィンドウを画面中央に配置するコマンドを送信
            if let Some(cmd) = egui::ViewportCommand::center_on_screen(ctx) {
                ctx.send_viewport_cmd(cmd);
            }

            self.should_set_initial_size = false;
            // リサイズが適用される次のフレームで正しく描画するため、このフレームの描画はスキップ
            return;
        }

        // --- データ更新 ---
        // バックグラウンドスレッドから新しいデータが届いていれば、描画データを更新
        if let Ok(result) = self.receiver.try_recv() {
            match result {
                Ok(points) => {
                    self.lidar_points = points;
                }
                Err(e) => {
                    // エラーやステータスメッセージをコンソールに表示
                    self.command_history.push(format!("INFO: {}", e));
                }
            }
            // 新しいデータが来たら再描画を要求
            ctx.request_repaint();
        }

        // TextEditウィジェットに割り当てる一意のID
        let console_input_id = egui::Id::new("console_input");

        // テキスト入力以外の場所でEnterが押されたか
        let enter_pressed_while_unfocused =
        ctx.input(|i| i.key_pressed(egui::Key::Enter)) && !ctx.wants_keyboard_input();

        // 1. 下側のパネル（コンソール）
        egui::TopBottomPanel::bottom("terminal")
            .resizable(true)
            .min_height(300.0)
            .show(ctx, |ui| {
                // パネル背景のクリックを検知するためのインタラクション領域
                let background_response = ui.interact(
                    ui.available_rect_before_wrap(),
                    ui.id().with("terminal_background"),
                    egui::Sense::click(),
                );
                // パネル背景がクリックされたか、またはEnterが押された場合にフォーカスを要求
                if background_response.clicked() || enter_pressed_while_unfocused {
                    ctx.memory_mut(|m| m.request_focus(console_input_id));
                }
                ui.heading("Console");
                egui::ScrollArea::vertical()
                    .stick_to_bottom(true)
                    .max_height(ui.available_height() * 0.8)
                    .show(ui, |ui| {
                        ui.with_layout(egui::Layout::top_down(egui::Align::LEFT), |ui| {
                            for line in &self.command_history {
                                ui.monospace(line);
                            }
                        });
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
                        let command = self.input_string.trim();
                        if !command.is_empty() {
                            self.command_history.push(format!("> {}", command));
                            match command {
                                "q" => {
                                    self.command_history.push("Exiting application...".to_string());
                                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                                }
                                _ => {
                                    self.command_history.push(format!("Unknown command: '{}'", command));
                                }
                            }
                        }
                        self.input_string.clear();
                        text_edit_response.request_focus();
                    }
                });
            });

        // 2. 上側のパネル（グラフィック表示）
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("LiDAR Data Visualization");
            let (response, painter) =
            ui.allocate_painter(ui.available_size(), egui::Sense::hover());
            let side = response.rect.width().min(response.rect.height());
            let square_rect = egui::Rect::from_center_size(response.rect.center(), egui::vec2(side, side));
            let to_screen = egui::emath::RectTransform::from_to(
                egui::Rect::from_center_size(egui::Pos2::ZERO, egui::vec2(16.0, 16.0)),
                //egui::Rect::from_center_size(egui::Pos2::ZERO, egui::vec2(66.0, 66.0)),
                square_rect,
            );
            painter.rect_filled(square_rect, 0.0, egui::Color32::from_rgb(20, 20, 20));
            painter.hline(square_rect.x_range(), to_screen.transform_pos(egui::pos2(0.0, 0.0)).y, egui::Stroke::new(0.5, egui::Color32::DARK_GRAY));
            painter.vline(to_screen.transform_pos(egui::pos2(0.0, 0.0)).x, square_rect.y_range(), egui::Stroke::new(0.5, egui::Color32::DARK_GRAY));
            for point in &self.lidar_points { // lidar_points を使うように変更
                let screen_pos = to_screen.transform_pos(egui::pos2(point.0, point.1));
                if square_rect.contains(screen_pos) {
                    painter.circle_filled(screen_pos, 2.0, egui::Color32::GREEN);
                }
            }
            // 画面中心にロボットの円を描画
            let robot_pos = to_screen.transform_pos(egui::Pos2::ZERO);
            painter.circle_filled(robot_pos, 5.0, egui::Color32::RED);
        });

        // UIイベントがない場合でも再描画を要求し、アニメーションを滑らかにする
        ctx.request_repaint();
    }
}

fn main() -> Result<(), eframe::Error> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_maximized(true)
            .with_min_inner_size([400.0, 300.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Mobitia 2-Pane Prototype",
        native_options,
        Box::new(|cc| {
            let mut style = (*cc.egui_ctx.style()).clone();
            style.text_styles.insert(
                egui::TextStyle::Monospace,
                egui::FontId::proportional(18.0), // ここでサイズを調整
            );
            cc.egui_ctx.set_style(style);
            Box::new(MyApp::default())
        }),
    )
}
