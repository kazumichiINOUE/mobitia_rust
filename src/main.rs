use anyhow::Result;
use eframe::egui;
use serialport::ClearBuffer;
use std::io::{self, Write};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;
use chrono::Local;

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
                    // 応答の終わりを判定する．SCIP2.0では，すべての応答の終端はこの形式．
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

// Helper function to decode a 3-character SCIP2.0 encoded value
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

// アプリケーション全体の状態を管理する構造体
struct MyApp {
    input_string: String,                              // コンソールの入力文字列
    command_history: Vec<String>,                      // コンソールのコマンド履歴
    lidar_points: Vec<(f32, f32)>,                     // 描画用のLiDAR点群データ (x, y)
    receiver: mpsc::Receiver<Result<Vec<(f32, f32)>>>, // データ受信用のレシーバー
    lidar_status_messages: Vec<String>,                // LiDARの状態表示用メッセージ
    status_receiver: mpsc::Receiver<String>,           // LiDARの状態メッセージ受信用のレシーバー
    command_output_receiver: mpsc::Receiver<String>,   // コンソールコマンド出力受信用のレシーバー
    command_output_sender: mpsc::Sender<String>,       // コンソールコマンド出力送信用のセンダー
    lidar_draw_rect: Option<egui::Rect>,               // LiDAR描画エリアのRect
    lidar_path: String,
    lidar_baud_rate: u32,
    lidar_connection_status: String,
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
        let lidar_config_clone = lidar_config.clone(); // スレッド用にクローン

        thread::spawn(move || {
            let lidar_config = lidar_config_clone;
            // --- LiDAR初期接続ロジック ---
            
            status_sender.send(format!("LiDAR Path: {}", lidar_config.lidar_path)).unwrap_or_default();
            status_sender.send(format!("Baud Rate: {}", lidar_config.baud_rate)).unwrap_or_default();

            let mut port = match serialport::new(&lidar_config.lidar_path, lidar_config.baud_rate)
                .timeout(Duration::from_millis(100)) // タイムアウトを短めに設定
                .open()
            {
                Ok(p) => {
                    status_sender.send(format!("INFO: Successfully opened port {}", lidar_config.lidar_path)).unwrap_or_default();
                    p
                }
                Err(e) => {
                    status_sender
                        .send(format!(
                            "ERROR: Failed to open port '{}': {}",
                            lidar_config.lidar_path,
                            e
                        ))
                        .unwrap_or_default();
                    // ポートが開けなかったらスレッド終了
                    return;
                }
            };

            // 初期化コマンドを送信
            let init_commands: &[&[u8]] = &[b"VV\n", b"PP\n", b"II\n"];
            for cmd in init_commands {
                status_sender.send(format!("INFO: Sending command: {}", String::from_utf8_lossy(*cmd).trim())).unwrap_or_default();
                let response = match send_and_receive(&mut port, *cmd) {
                    Ok(resp) => resp,
                    Err(e) => {
                        status_sender
                            .send(format!("ERROR: Failed to send command {}: {}", String::from_utf8_lossy(*cmd).trim(), e))
                            .unwrap_or_default();
                        return; // 初期化失敗時もスレッド終了
                    }
                };
                status_sender.send(format!("INFO: Response for {}: {}", String::from_utf8_lossy(*cmd).trim(), response.trim())).unwrap_or_default();
                thread::sleep(Duration::from_millis(50)); // コマンド間に少し待機
            }

            // BM コマンドを別に送信
            status_sender.send(format!("INFO: Sending command: {}", String::from_utf8_lossy(b"BM\n").trim())).unwrap_or_default();
            let response_bm = match send_and_receive(&mut port, b"BM\n") {
                Ok(resp) => resp,
                Err(e) => {
                    status_sender
                        .send(format!("ERROR: Failed to send command BM: {}", e))
                        .unwrap_or_default();
                    return;
                }
            };
            status_sender.send(format!("INFO: Response for BM: {}", response_bm.trim())).unwrap_or_default();
            thread::sleep(Duration::from_millis(50)); // コマンド間に少し待機
            status_sender.send("INFO: LiDAR initialized. Laser is ON.".to_string())
                .unwrap_or_default();
            // --- ここまでLiDAR初期接続ロジック ---

            // --- ここからLiDARデータ取得ロジック (GDコマンド) ---
            loop {
                // GDコマンドを送信してLiDARデータを取得
                let command = b"GD0000108001\n";
                status_sender.send(format!("INFO: Sending command: {}", String::from_utf8_lossy(command).trim())).unwrap_or_default();

                let response = match send_and_receive(&mut port, command) {
                    Ok(resp) => resp,
                    Err(e) => {
                        status_sender
                            .send(format!("ERROR: Failed to send GD command or receive response: {}", e))
                            .unwrap_or_default();
                        break;
                    }
                };

                // 応答を解析してタイムスタンプを抽出
                let lines: Vec<&str> = response.trim().lines().collect();
                // GDコマンドのエコー、ステータス行、タイムスタンプ行が最低限必要
                if lines.len() >= 3 { 
                    // GDコマンドのエコーバック
                    status_sender.send(format!("GD Echo: {}", lines[0]))
                        .unwrap_or_default();

                    // ステータス行 (2バイト)
                    let status_line = lines[1];
                    if status_line.len() >= 2 { // ステータスは2文字
                        status_sender.send(format!("Status: {}", &status_line[0..2]))
                            .unwrap_or_default();
                    } else {
                        status_sender.send(format!("ERROR: Status line too short: '{}'", status_line))
                            .unwrap_or_default();
                    }


                    // タイムスタンプ行からタイムスタンプ部分を抽出
                    let timestamp_line = lines[2]; // 3行目 (index 2) がタイムスタンプ
                    if timestamp_line.len() >= 4 { // タイムスタンプは4文字
                        let encoded_timestamp = &timestamp_line[0..4];
                        match decode_scip_2_0_4char(encoded_timestamp) {
                            Ok(timestamp_value) => {
                                status_sender.send(format!("Decoded Timestamp: {}", timestamp_value))
                                    .unwrap_or_default();
                            }
                            Err(e) => {
                                status_sender.send(format!("ERROR: Failed to decode timestamp: {}", e))
                                    .unwrap_or_default();
                            }
                        }
                    } else {
                        status_sender.send(format!("ERROR: Timestamp line too short: '{}'", timestamp_line))
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
                            status_sender.send(format!("ERROR: Invalid 3-char encoded distance length: '{}' at chunk index {}", encoded_distance, i))
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

                                // 距離30m以上は測定不能または無効なデータと判断
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
                                status_sender.send(format!("ERROR: Failed to decode distance '{}' at chunk index {}: {}", encoded_distance, i, e))
                                    .unwrap_or_default();
                            }
                        }
                        current_angle += angle_increment;
                    }

                    sender.send(Ok(lidar_points_current_scan.clone()))
                        .unwrap_or_default();

                    /*
                    // 点群データをファイルに保存するテストコード（不使用）
                    let filename = "/Users/kaz/src/mobitia/lidar_points_capture.txt"; // プロジェクトルートディレクトリに保存
                    let mut file = match std::fs::File::create(filename) {
                        Ok(f) => f,
                        Err(e) => {
                            status_sender.send(format!("ERROR: Failed to create file {}: {}", filename, e))
                                .unwrap_or_default();
                            break; // エラーが発生した場合はループを抜ける
                        }
                    };

                    for point in &lidar_points_current_scan {
                        if let Err(e) = writeln!(file, "{} {}", point.0, point.1) {
                            status_sender.send(format!("ERROR: Failed to write to file {}: {}", filename, e))
                                .unwrap_or_default();
                            break; // エラーが発生した場合はループを抜ける
                        }
                    }
                    status_sender.send(format!("INFO: First point cloud saved to {}. Terminating.", filename))
                        .unwrap_or_default();
                    */

                    // 生データの出力はコメントアウト
                    // sender.send(Err(anyhow::anyhow!(format!("RAW GD RESPONSE: {}", response))))
                    //       .unwrap_or_default();

                } else {
                    status_sender.send(format!("ERROR: Malformed GD response (expected at least 3 lines): {}", response))
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

        // LiDARステータスメッセージの受信と更新
        while let Ok(msg) = self.status_receiver.try_recv() {
            let mut handled_as_status = false;

            // 特定のメッセージを解析して固定ステータスを更新
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

            // 固定ステータスとして扱われなかったメッセージ、かつログから除外したい情報を除く
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

        // コマンド出力メッセージの受信と更新
        while let Ok(msg) = self.command_output_receiver.try_recv() {
            self.command_history.push(msg);
            ctx.request_repaint();
        }

        // TextEditウィジェットに割り当てる一意のID
        let console_input_id = egui::Id::new("console_input");

        // テキスト入力以外の場所でEnterが押されたか
        let enter_pressed_while_unfocused =
        ctx.input(|i| i.key_pressed(egui::Key::Enter)) && !ctx.wants_keyboard_input();

        // Ctrl + L (または Cmd + L) でコンソールをクリア
        if ctx.input(|i| (i.modifiers.ctrl || i.modifiers.command) && i.key_pressed(egui::Key::L)) {
            self.command_history.clear();
            ctx.request_repaint(); // 画面を再描画してクリアを反映
        }

        // 1. 左側のパネル（コンソール）
        egui::SidePanel::left("terminal")
            .exact_width(ctx.input(|i| i.screen_rect()).width() / 4.0)
            .resizable(true)
            .min_width(150.0) // 最小幅を設定
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

                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.label(format!("Device Path: {}", self.lidar_path));
                    ui.label(format!("Baud Rate: {}", self.lidar_baud_rate));

                    // ステータスの内容に応じて色を決定
                    let status_text = format!("Status: {}", self.lidar_connection_status);
                    let status_color = if self.lidar_connection_status.starts_with("Connected") {
                        egui::Color32::GREEN // 接続成功時は緑
                    } else if self.lidar_connection_status.starts_with("Connecting") {
                        egui::Color32::YELLOW // 接続中は黄色
                    } else {
                        egui::Color32::RED // それ以外（エラーなど）は赤
                    };
                    
                    // RichTextを使って色付きのラベルを表示
                    ui.label(egui::RichText::new(status_text).color(status_color));
                });
                ui.separator();

                // LiDAR状態表示用の固定領域
                ui.group(|ui| {
                    ui.set_width(ui.available_width()); // 利用可能な全幅を使う
                    ui.set_height(ui.text_style_height(&egui::TextStyle::Monospace) * 10.0); // 10行分の高さを確保
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        for line in &self.lidar_status_messages {
                            ui.monospace(line);
                        }
                    });
                });
                ui.separator(); // 固定領域とコマンド履歴の間に区切り線
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
                        let full_command_line = self.input_string.trim();
                        if !full_command_line.is_empty() {
                            self.command_history.push(format!("> {}", full_command_line));

                            let parts: Vec<&str> = full_command_line.split_whitespace().collect();
                            let command_name = parts[0];
                            let args = &parts[1..];

                            match command_name {
                                "help" => {
                                    self.command_history.push("Available commands:".to_string());
                                    self.command_history.push("  help      - Show this help message".to_string());
                                    self.command_history.push("  q         - Quit the application".to_string());
                                    self.command_history.push("  ls [path] - List directory contents (-l) of [path]".to_string());
                                    self.command_history.push("  save      - Save current LiDAR visualization as image".to_string());
                                    self.command_history.push("  zi        - Zoom in (not yet implemented)".to_string());
                                    self.command_history.push("  zo        - Zoom out (not yet implemented)".to_string());
                                    self.command_history.push("  Ctrl+L/Cmd+L - Clear console history (anywhere)".to_string());
                                }
                                "q" => {
                                    self.command_history.push("Exiting application...".to_string());
                                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                                }
                                "save" => {
                                    let sender_clone = self.command_output_sender.clone();
                                    self.command_history.push("Saving LiDAR visualization...".to_string());

                                    let lidar_rect = self.lidar_draw_rect; // Option<egui::Rect>
                                    if lidar_rect.is_none() {
                                        sender_clone.send("ERROR: LiDAR drawing area not available for saving.".to_string()).unwrap_or_default();
                                        return;
                                    }
                                    let lidar_rect = lidar_rect.unwrap();
                                    let width = lidar_rect.width() as u32;
                                    let height = lidar_rect.height() as u32;

                                    if width == 0 || height == 0 {
                                        sender_clone.send(format!("ERROR: Invalid LiDAR drawing area size: {}x{}.", width, height)).unwrap_or_default();
                                        return;
                                    }

                                    // 現在のLiDAR点群データを取得
                                    let lidar_points_clone = self.lidar_points.clone();

                                    thread::spawn(move || {
                                        // egui::ColorImage は RGB を想定しているので、RGB8を直接操作
                                        let mut img = image::RgbImage::new(width, height);

                                        // 描画エリアの座標変換 (CentralPanel のコードから再利用)
                                        let to_screen = egui::emath::RectTransform::from_to(
                                            egui::Rect::from_center_size(egui::Pos2::ZERO, egui::vec2(16.0, 16.0)), // 元の描画範囲
                                            egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width as f32, height as f32)), // Image の座標系
                                        );

                                        // 背景を黒で塗りつぶし
                                        for x in 0..width {
                                            for y in 0..height {
                                                img.put_pixel(x, y, image::Rgb([20, 20, 20]));
                                            }
                                        }

                                        // LiDAR点群を描画
                                        for point in &lidar_points_clone {
                                            let screen_pos = to_screen.transform_pos(egui::pos2(point.0, point.1));
                                            let x = screen_pos.x.round() as u32;
                                            let y = screen_pos.y.round() as u32;

                                            // 画像範囲内であればピクセルをセット
                                            if x < width && y < height {
                                                img.put_pixel(x, y, image::Rgb([0, 255, 0])); // 緑色の点
                                            }
                                        }

                                        // ファイル名を生成 (例: lidar_capture_YYYYMMDD_HHMMSS.png)
                                        let timestamp = Local::now().format("%Y%m%d_%H%M%S").to_string();
                                        let filename = format!("lidar_capture_{}.png", timestamp);
                                        let save_path = std::path::PathBuf::from(filename);

                                        match img.save(&save_path) {
                                            Ok(_) => {
                                                sender_clone.send(format!("Image saved to: {}", save_path.display())).unwrap_or_default();
                                            }
                                            Err(e) => {
                                                sender_clone.send(format!("ERROR: Failed to save image: {}", e)).unwrap_or_default();
                                            }
                                        }
                                        sender_clone.send("Finished saving LiDAR visualization.".to_string()).unwrap_or_default();
                                    });
                                }
                                "ls" => {
                                    let sender_clone = self.command_output_sender.clone();
                                    let path_arg = if args.is_empty() {
                                        ".".to_string() // 引数がない場合はカレントディレクトリ
                                    } else {
                                        args[0].to_string() // 最初の引数をパスとみなす
                                    };
                                    self.command_history.push(format!("Executing 'sh -c \"ls -1 {}\"' in background...", path_arg));
                                    thread::spawn(move || {
                                        let output = std::process::Command::new("sh")
                                            .arg("-c")
                                            .arg(format!("ls -1 {}", path_arg)) // シェルを介して実行
                                            .output();

                                        match output {
                                            Ok(output) => {
                                                if !output.stdout.is_empty() {
                                                    let stdout = String::from_utf8_lossy(&output.stdout);
                                                    for line in stdout.lines() {
                                                        sender_clone.send(line.to_string()).unwrap_or_default();
                                                    }
                                                }
                                                if !output.stderr.is_empty() {
                                                    let stderr = String::from_utf8_lossy(&output.stderr);
                                                    for line in stderr.lines() {
                                                        sender_clone.send(format!("ERROR: {}", line)).unwrap_or_default();
                                                    }
                                                }
                                            },
                                            Err(e) => {
                                                sender_clone.send(format!("ERROR: Failed to execute 'ls': {}", e)).unwrap_or_default();
                                            }
                                        }
                                        sender_clone.send(format!("Finished 'ls -1 {}.", path_arg)).unwrap_or_default();
                                    });
                                }
                                "zi" => {
                                    self.command_history.push("Zoom in".to_string());
                                }
                                "zo" => {
                                    self.command_history.push("Zoom out".to_string());
                                }
                                _ => {
                                    self.command_history.push(format!("Unknown command: '{}'", full_command_line));
                                }
                            }
                        }
                        self.input_string.clear();
                        text_edit_response.request_focus();
                    }
                });
            });

        // 2. 右側のパネル（グラフィック表示）
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("LiDAR Data Visualization");
            let (response, painter) =
            ui.allocate_painter(ui.available_size(), egui::Sense::hover());

            // ここで lidar_draw_rect を更新
            self.lidar_draw_rect = Some(response.rect); 

            let side = response.rect.height();
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
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
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
