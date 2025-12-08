use crate::app::{ConsoleOutputEntry, DemoMode, MyApp};
use chrono::Local;
use clap::{Parser, Subcommand};
use dirs;
use eframe::egui;
use std::thread;

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
    /// Manage serial port functions.
    Serial {
        #[command(subcommand)]
        command: SerialCommands,
    },
    /// Show the path of the storage file.
    #[command(name = "debug-storage")] // Explicitly set name for hyphenated command
    DebugStorage,
    /// Quit the application.
    #[command(alias = "q")]
    Quit,
    /// Clear console history.
    Clear,
    /// Save data (image or points).
    Save {
        #[command(subcommand)]
        command: SaveCommands,
    },
}

#[derive(Subcommand, Debug)]
pub enum SaveCommands {
    /// Save current LiDAR visualization as an image.
    #[command(alias = "i")]
    Image,
    /// Save current LiDAR point cloud data to a file.
    #[command(alias = "p")]
    Points {
        /// Optional output file path (e.g., "output.lsp"). Defaults to "lidar_points_YYYYMMDD_HHMMSS.lsp".
        #[arg(short, long)]
        output: Option<String>,
    },
}

#[derive(Subcommand, Debug)]
pub enum SerialCommands {
    /// List available serial ports.
    #[command(alias = "ls")]
    List {
        /// Optional: Show details for a specific port path.
        path: Option<String>,
        /// Show detailed information for each port.
        #[arg(long, short)]
        detail: bool,
    },
}

#[derive(Subcommand, Debug)]
pub enum SetCommands {
    /// Set the demo mode.
    Demo {
        /// Demo mode to set ("scan" or "ripple").
        #[arg(value_parser = ["scan", "ripple", "breathing", "table"])]
        mode: String,
    },
    /// Set the LiDAR device path.
    Path {
        /// New LiDAR device path.
        path: String,
    },
}

pub fn handle_command(app: &mut MyApp, ctx: &egui::Context, cli: Cli) {
    // 新しいコマンド実行のグループIDを生成
    app.next_group_id += 1;
    let current_group_id = app.next_group_id;

    match cli.command {
        Commands::Help => {
            app.command_history.push(ConsoleOutputEntry {
                text: "Available commands:".to_string(),
                group_id: current_group_id,
            });
            app.command_history.push(ConsoleOutputEntry {
                text: "  help (or h)                  - Show this help message".to_string(),
                group_id: current_group_id,
            });
            app.command_history.push(ConsoleOutputEntry {
                text: "  set demo <scan|ripple|breathing|table> - Set the demo mode".to_string(),
                group_id: current_group_id,
            });
            app.command_history.push(ConsoleOutputEntry {
                text: "  set path <path>              - Set the LiDAR device path".to_string(),
                group_id: current_group_id,
            });
            app.command_history.push(ConsoleOutputEntry {
                text: "  serial list [--detail] [path] - List available serial ports".to_string(),
                group_id: current_group_id,
            });
            app.command_history.push(ConsoleOutputEntry {
                text: "  debug-storage                - Show the path of the storage file"
                    .to_string(),
                group_id: current_group_id,
            });
            app.command_history.push(ConsoleOutputEntry {
                text: "  quit (or q)                  - Quit the application".to_string(),
                group_id: current_group_id,
            });
            app.command_history.push(ConsoleOutputEntry {
                text: "  clear                        - Clear console history (or Ctrl+L/Cmd+L)"
                    .to_string(),
                group_id: current_group_id,
            });
            app.command_history.push(ConsoleOutputEntry {
                text:
                    "  save image                   - Save current LiDAR visualization as an image"
                    .to_string(),
                group_id: current_group_id,
            });
            app.command_history.push(ConsoleOutputEntry { 
                text: "  save points (or p) [--output <file>] - Save current LiDAR point cloud to a file (.lsp format)"
                    .to_string(), group_id: current_group_id });
        }
        Commands::Quit => {
            app.command_history.push(ConsoleOutputEntry {
                text: "Exiting application...".to_string(),
                group_id: current_group_id,
            });
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        Commands::Clear => {
            app.command_history.clear();
        }
        Commands::DebugStorage => {
            let app_name = "Mobitia 2-Pane Prototype";
            if let Some(dir) = eframe::storage_dir(app_name) {
                app.command_history.push(ConsoleOutputEntry {
                    text: format!("Storage directory: {}", dir.display()),
                    group_id: current_group_id,
                });
            } else {
                app.command_history.push(ConsoleOutputEntry {
                    text: "Could not determine storage directory.".to_string(),
                    group_id: current_group_id,
                });
            }
        }
        Commands::Set { command } => match command {
            SetCommands::Demo { mode } => match mode.as_str() {
                "scan" => {
                    app.demo_mode = DemoMode::RotatingScan;
                    app.command_history.push(ConsoleOutputEntry {
                        text: "Demo mode set to Rotating Scan.".to_string(),
                        group_id: current_group_id,
                    });
                }
                "ripple" => {
                    app.demo_mode = DemoMode::ExpandingRipple;
                    app.command_history.push(ConsoleOutputEntry {
                        text: "Demo mode set to Expanding Ripple.".to_string(),
                        group_id: current_group_id,
                    });
                }
                "breathing" => {
                    app.demo_mode = DemoMode::BreathingCircle;
                    app.command_history.push(ConsoleOutputEntry {
                        text: "Demo mode set to Breathing Circle.".to_string(),
                        group_id: current_group_id,
                    });
                }
                "table" => {
                    app.demo_mode = DemoMode::Table;
                    app.command_history.push(ConsoleOutputEntry {
                        text: "Demo mode set to Table.".to_string(),
                        group_id: current_group_id,
                    });
                }
                _ => {
                    app.command_history.push(ConsoleOutputEntry { 
                        text: format!("ERROR: Unknown demo mode: '{}'. Use 'scan', 'ripple', 'breathing', or 'table'.", 
                            mode), group_id: current_group_id });
                }
            },
            SetCommands::Path { path } => {
                app.lidar_path = path.clone();
                app.command_history.push(ConsoleOutputEntry {
                    text: format!("LiDAR path set to: {}", path),
                    group_id: current_group_id,
                });
                app.command_history.push(ConsoleOutputEntry {
                    text: "NOTE: Restart the application to apply the new path.".to_string(),
                    group_id: current_group_id,
                });
            }
        },
        Commands::Serial { command } => match command {
            SerialCommands::List { path, detail } => {
                app.command_history.push(ConsoleOutputEntry {
                    text: "Searching for available serial ports...".to_string(),
                    group_id: current_group_id,
                });
                match serialport::available_ports() {
                    Ok(ports) => {
                        if ports.is_empty() {
                            app.command_history.push(ConsoleOutputEntry {
                                text: "No serial ports found.".to_string(),
                                group_id: current_group_id,
                            });
                            return;
                        }

                        let mut found_any = false;
                        for p in ports {
                            if let Some(ref path_filter) = path {
                                if p.port_name != *path_filter {
                                    continue;
                                }
                            }
                            found_any = true;

                            if detail {
                                app.command_history.push(ConsoleOutputEntry {
                                    text: format!("Port: {}", p.port_name),
                                    group_id: current_group_id,
                                });
                                match &p.port_type {
                                    serialport::SerialPortType::UsbPort(info) => {
                                        app.command_history.push(ConsoleOutputEntry {
                                            text: "  Type: USB".to_string(),
                                            group_id: current_group_id,
                                        });
                                        app.command_history.push(ConsoleOutputEntry {
                                            text: format!(
                                                "  VID: {:04x}, PID: {:04x}",
                                                info.vid, info.pid
                                            ),
                                            group_id: current_group_id,
                                        });
                                        if let Some(sn) = &info.serial_number {
                                            app.command_history.push(ConsoleOutputEntry {
                                                text: format!("  Serial Number: {}", sn),
                                                group_id: current_group_id,
                                            });
                                        }
                                        if let Some(manufacturer) = &info.manufacturer {
                                            app.command_history.push(ConsoleOutputEntry {
                                                text: format!("  Manufacturer: {}", manufacturer),
                                                group_id: current_group_id,
                                            });
                                        }
                                        if let Some(product) = &info.product {
                                            app.command_history.push(ConsoleOutputEntry {
                                                text: format!("  Product: {}", product),
                                                group_id: current_group_id,
                                            });
                                        }
                                    }
                                    serialport::SerialPortType::BluetoothPort => {
                                        app.command_history.push(ConsoleOutputEntry {
                                            text: "  Type: Bluetooth".to_string(),
                                            group_id: current_group_id,
                                        });
                                    }
                                    serialport::SerialPortType::PciPort => {
                                        app.command_history.push(ConsoleOutputEntry {
                                            text: "  Type: PCI".to_string(),
                                            group_id: current_group_id,
                                        });
                                    }
                                    serialport::SerialPortType::Unknown => {
                                        app.command_history.push(ConsoleOutputEntry {
                                            text: "  Type: Unknown".to_string(),
                                            group_id: current_group_id,
                                        });
                                    }
                                }
                            } else {
                                app.command_history.push(ConsoleOutputEntry {
                                    text: p.port_name,
                                    group_id: current_group_id,
                                });
                            }
                        }

                        if let Some(path_filter) = path {
                            if !found_any {
                                app.command_history.push(ConsoleOutputEntry {
                                    text: format!("Port '{}' not found.", path_filter),
                                    group_id: current_group_id,
                                });
                            }
                        }
                    }
                    Err(e) => {
                        app.command_history.push(ConsoleOutputEntry {
                            text: format!("ERROR: Failed to list serial ports: {}", e),
                            group_id: current_group_id,
                        });
                    }
                }
            }
        },
        Commands::Save { command } => match command {
            SaveCommands::Image => {
                let sender_clone = app.command_output_sender.clone();
                let save_dir = match dirs::download_dir() {
                    Some(path) => path.join(Local::now().format("%Y-%m-%d").to_string()),
                    None => {
                        sender_clone
                            .send("ERROR: Could not find download directory.".to_string())
                            .unwrap_or_default();
                        return;
                    }
                };
                if let Err(e) = std::fs::create_dir_all(&save_dir) {
                    sender_clone
                        .send(format!(
                            "ERROR: Failed to create directory '{}': {}",
                            save_dir.display(),
                            e
                        ))
                        .unwrap_or_default();
                    return;
                }

                app.command_history.push(ConsoleOutputEntry {
                    text: "Saving LiDAR visualization...".to_string(),
                    group_id: current_group_id,
                });

                let lidar_rect = match app.lidar_draw_rect {
                    Some(rect) => rect,
                    None => {
                        sender_clone
                            .send("ERROR: LiDAR drawing area not available.".to_string())
                            .unwrap_or_default();
                        return;
                    }
                };

                let width = lidar_rect.width() as u32;
                let height = lidar_rect.height() as u32;

                if width == 0 || height == 0 {
                    sender_clone
                        .send(format!(
                            "ERROR: Invalid drawing area size: {}x{}.",
                            width, height
                        ))
                        .unwrap_or_default();
                    return;
                }

                let lidar_points_clone = app.lidar_points.clone();

                thread::spawn(move || {
                    let mut img = image::RgbImage::new(width, height);
                    let to_screen = egui::emath::RectTransform::from_to(
                        egui::Rect::from_center_size(egui::Pos2::ZERO, egui::vec2(16.0, 16.0)),
                        egui::Rect::from_min_size(
                            egui::Pos2::ZERO,
                            egui::vec2(width as f32, height as f32),
                        ),
                    );

                    for x in 0..width {
                        for y in 0..height {
                            img.put_pixel(x, y, image::Rgb([20, 20, 20]));
                        }
                    }

                    for point in &lidar_points_clone {
                        let screen_pos = to_screen.transform_pos(egui::pos2(point.0, -point.1));
                        let x = screen_pos.x.round() as u32;
                        let y = screen_pos.y.round() as u32;
                        if x < width && y < height {
                            img.put_pixel(x, y, image::Rgb([0, 255, 0]));
                        }
                    }

                    let timestamp = Local::now().format("%H%M%S").to_string();
                    let filename = format!("lidar_capture_{}.png", timestamp);
                    let save_path = save_dir.join(filename);

                    match img.save(&save_path) {
                        Ok(_) => sender_clone
                            .send(format!("Image saved to: {}", save_path.display()))
                            .unwrap_or_default(),
                        Err(e) => sender_clone
                            .send(format!("ERROR: Failed to save image: {}", e))
                            .unwrap_or_default(),
                    }
                });
            }
            SaveCommands::Points { output } => {
                let path_to_save = if let Some(path_str) = output {
                    path_str
                } else {
                    let save_dir = match dirs::download_dir() {
                        Some(path) => path.join(Local::now().format("%Y-%m-%d").to_string()),
                        None => {
                            app.command_history.push(ConsoleOutputEntry {
                                text: "ERROR: Could not find download directory.".to_string(),
                                group_id: current_group_id,
                            });
                            return;
                        }
                    };
                    if let Err(e) = std::fs::create_dir_all(&save_dir) {
                        app.command_history.push(ConsoleOutputEntry {
                            text: format!(
                                "ERROR: Failed to create directory '{}': {}",
                                save_dir.display(),
                                e
                            ),
                            group_id: current_group_id,
                        });
                        return;
                    }
                    let timestamp = Local::now().format("%H%M%S").to_string();
                    let filename_only = format!("lidar_points_{}.lsp", timestamp);
                    save_dir.join(filename_only).to_string_lossy().into_owned()
                };

                app.command_history.push(ConsoleOutputEntry {
                    text: format!("Requesting save of point cloud to '{}'...", path_to_save),
                    group_id: current_group_id,
                });
                app.request_save_points(path_to_save);
                ctx.request_repaint(); // ファイル保存リクエストのために再描画
            }
        },
    }
}
