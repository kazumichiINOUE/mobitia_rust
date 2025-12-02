use crate::app::MyApp;
use chrono::Local;
use eframe::egui;
use std::thread;

pub fn handle_command(app: &mut MyApp, ctx: &egui::Context, full_command_line: &str) {
    let parts: Vec<&str> = full_command_line.split_whitespace().collect();
    let command_name = parts[0];
    let args = &parts[1..];

    match command_name {
        "help" => {
            app.command_history.push("Available commands:".to_string());
            app.command_history.push("  help      - Show this help message".to_string());
            app.command_history.push("  q         - Quit the application".to_string());
            app.command_history.push("  ls [path] - List directory contents of [path]".to_string());
            app.command_history.push("  save      - Save current LiDAR visualization as image".to_string());
            app.command_history.push("  clear     - Clear console history".to_string());
            app.command_history.push("  Ctrl+L/Cmd+L - Also clear console history".to_string());
        }
        "q" => {
            app.command_history.push("Exiting application...".to_string());
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        "clear" => {
            app.command_history.clear();
        }
        "save" => {
            let sender_clone = app.command_output_sender.clone();
            app.command_history.push("Saving LiDAR visualization...".to_string());

            let lidar_rect = match app.lidar_draw_rect {
                Some(rect) => rect,
                None => {
                    sender_clone.send("ERROR: LiDAR drawing area not available.".to_string()).unwrap_or_default();
                    return;
                }
            };
            
            let width = lidar_rect.width() as u32;
            let height = lidar_rect.height() as u32;

            if width == 0 || height == 0 {
                sender_clone.send(format!("ERROR: Invalid drawing area size: {}x{}.", width, height)).unwrap_or_default();
                return;
            }

            let lidar_points_clone = app.lidar_points.clone();

            thread::spawn(move || {
                let mut img = image::RgbImage::new(width, height);
                let to_screen = egui::emath::RectTransform::from_to(
                    egui::Rect::from_center_size(egui::Pos2::ZERO, egui::vec2(16.0, 16.0)),
                    egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width as f32, height as f32)),
                );

                for x in 0..width {
                    for y in 0..height {
                        img.put_pixel(x, y, image::Rgb([20, 20, 20]));
                    }
                }

                for point in &lidar_points_clone {
                    let screen_pos = to_screen.transform_pos(egui::pos2(point.0, point.1));
                    let x = screen_pos.x.round() as u32;
                    let y = screen_pos.y.round() as u32;
                    if x < width && y < height {
                        img.put_pixel(x, y, image::Rgb([0, 255, 0]));
                    }
                }

                let timestamp = Local::now().format("%Y%m%d_%H%M%S").to_string();
                let filename = format!("lidar_capture_{}.png", timestamp);
                let save_path = std::path::PathBuf::from(&filename);

                match img.save(&save_path) {
                    Ok(_) => sender_clone.send(format!("Image saved to: {}", filename)).unwrap_or_default(),
                    Err(e) => sender_clone.send(format!("ERROR: Failed to save image: {}", e)).unwrap_or_default(),
                }
            });
        }
        "ls" => {
            let sender_clone = app.command_output_sender.clone();
            let path_arg = if args.is_empty() { ".".to_string() } else { args[0].to_string() };
            app.command_history.push(format!("Executing 'ls -1 {}'...", path_arg));
            thread::spawn(move || {
                match std::process::Command::new("ls").arg("-1").arg(&path_arg).output() {
                    Ok(output) => {
                        if !output.stdout.is_empty() {
                            String::from_utf8_lossy(&output.stdout)
                                .lines()
                                .for_each(|line| { sender_clone.send(line.to_string()).unwrap_or_default(); });
                        }
                        if !output.stderr.is_empty() {
                            String::from_utf8_lossy(&output.stderr)
                                .lines()
                                .for_each(|line| { sender_clone.send(format!("ERROR: {}", line)).unwrap_or_default(); });
                        }
                    },
                    Err(e) => {
                        sender_clone.send(format!("ERROR: Failed to execute 'ls': {}", e)).unwrap_or_default();
                    }
                }
            });
        }
        _ => {
            app.command_history.push(format!("Unknown command: '{}'", full_command_line));
        }
    }
}
