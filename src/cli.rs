use crate::app::{MyApp, DemoMode};
use chrono::Local;
use eframe::egui;
use std::thread;
use clap::{Parser, Subcommand}; // clapをインポート

/// CLI Commands for Mobitia application
#[derive(Parser, Debug)]
#[command(name = "mobitia", no_binary_name(true), version, about, long_about = None, disable_help_flag = true, disable_help_subcommand = true)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand, Debug)]
pub enum Commands {
    /// Show help for commands.
    #[command(alias = "h")]
    Help,
    /// Set a configuration value.
    Set {
        #[command(subcommand)]
        command: SetCommands,
    },

    /// Show the path of the storage file.
    #[command(name = "debug-storage")] // Explicitly set name for hyphenated command
    DebugStorage,
    /// Quit the application.
    #[command(alias = "q")]
    Quit,
    /// Clear console history.
    Clear,
    /// List directory contents.
    Ls {
        /// Path to list (defaults to current directory).
        path: Option<String>,
    },
    /// Save current LiDAR visualization as image.
    Save,
}

#[derive(Subcommand, Debug)]
pub enum SetCommands {
    /// Set the demo mode.
    Demo {
        /// Demo mode to set ("scan" or "ripple").
        #[arg(value_parser = ["scan", "ripple", "breathing"])]
        mode: String,
    },
    /// Set the LiDAR device path.
    Path {
        /// New LiDAR device path.
        path: String,
    },
}

pub fn handle_command(app: &mut MyApp, ctx: &egui::Context, cli: Cli) {
    match cli.command {
        Commands::Help => {
            app.command_history.push("Available commands:".to_string());
            app.command_history.push("  help                         - Show this help message".to_string());
            app.command_history.push("  set demo <scan|ripple|breathing> - Set the demo mode".to_string());
            app.command_history.push("  set path <path>              - Set the LiDAR device path".to_string());
            app.command_history.push("  debug-storage                - Show the path of the storage file".to_string());
            app.command_history.push("  quit (or q)                  - Quit the application".to_string());
            app.command_history.push("  clear                        - Clear console history (or Ctrl+L/Cmd+L)".to_string());
            app.command_history.push("  ls [path]                    - List directory contents".to_string());
            app.command_history.push("  save                         - Save current LiDAR visualization as image".to_string());
        }
        Commands::Quit => {
            app.command_history.push("Exiting application...".to_string());
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        Commands::Clear => {
            app.command_history.clear();
        }
        Commands::DebugStorage => {
            let app_name = "Mobitia 2-Pane Prototype";
            if let Some(dir) = eframe::storage_dir(app_name) {
                app.command_history.push(format!("Storage directory: {}", dir.display()));
            } else {
                app.command_history.push("Could not determine storage directory.".to_string());
            }
        }
        Commands::Set { command } => match command {
            SetCommands::Demo { mode } => {
                match mode.as_str() {
                    "scan" => {
                        app.demo_mode = DemoMode::RotatingScan;
                        app.command_history.push("Demo mode set to Rotating Scan.".to_string());
                    }
                    "ripple" => {
                        app.demo_mode = DemoMode::ExpandingRipple;
                        app.command_history.push("Demo mode set to Expanding Ripple.".to_string());
                    }
                    "breathing" => {
                        app.demo_mode = DemoMode::BreathingCircle;
                        app.command_history.push("Demo mode set to Breathing Circle.".to_string());
                    }
                    _ => {
                        app.command_history.push(format!("ERROR: Unknown demo mode: '{}'. Use 'scan', 'ripple', or 'breathing'.", mode));
                    }
                }
            }
            SetCommands::Path { path } => {
                app.lidar_path = path.clone();
                app.command_history.push(format!("LiDAR path set to: {}", path));
                app.command_history.push("NOTE: Restart the application to apply the new path.".to_string());
            }
        },
        Commands::Save => {
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
        Commands::Ls { path } => {
            let sender_clone = app.command_output_sender.clone();
            let path_arg = path.unwrap_or_else(|| ".".to_string());
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
    }
}
