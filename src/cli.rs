use crate::app::{AppMode, ConsoleOutputEntry, DemoMode, MyApp};
use chrono::Local;
use clap::{CommandFactory, Parser, Subcommand};
use dirs;
use eframe::egui;
use std::path::PathBuf;
use std::thread;

/// CLI Commands for Mobitia application
#[derive(Parser, Debug)]
#[command(name = "mobitia", no_binary_name(true), about, long_about = None, disable_version_flag = true, disable_help_flag = true, disable_help_subcommand = true)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand, Debug)]
pub enum Commands {
    /// Show help for commands.
    #[command(alias = "h")]
    Help,
    /// Manage Camera related commands and settings.
    Camera {
        #[command(subcommand)]
        command: Option<CameraCommands>,
    },
    /// Manage LiDAR related commands and settings.
    Lidar {
        #[command(subcommand)]
        command: Option<LidarCommands>,
    },
    /// Enter SLAM mode.
    Slam {
        #[command(subcommand)]
        command: Option<SlamCommands>,
    },
    /// Enter demo mode.
    Demo {
        /// Demo mode to set ("scan", "ripple", "breathing", or "table"). Defaults to "scan" if omitted.
        #[arg(value_parser = ["scan", "ripple", "breathing", "table"])]
        mode: Option<String>,
    },
    /// Manage serial port functions.
    Serial {
        #[command(subcommand)]
        command: SerialCommands,
    },
    /// Show the path of the storage file.
    #[command(name = "debug-storage")] // Explicitly set name for hyphenated command
    DebugStorage,
    /// Show the application version.
    Version,
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
    /// Manage map data.
    #[command(alias = "m")]
    Map {
        #[command(subcommand)]
        command: MapCommands,
    },
}

#[derive(Subcommand, Debug)]
pub enum CameraCommands {
    /// Enter Camera visualization mode.
    #[command(alias = "mode")]
    EnterMode,
}

#[derive(Subcommand, Debug)]
pub enum MapCommands {
    /// Load a map from a specified directory, loading submaps sequentially.
    Load {
        /// Path to the directory containing map data (e.g., ./slam_results/slam_result_20251220-211047).
        path: PathBuf,
    },
}

#[derive(Subcommand, Debug)]
pub enum LidarCommands {
    /// Enter LiDAR visualization mode.
    #[command(alias = "mode")]
    EnterMode,
    /// Toggle whether a Lidar is used for SLAM.
    SlamToggle {
        /// Lidar ID (e.g., 0, 1).
        id: usize,
    },
    /// Set LiDAR related configurations.
    #[command(subcommand)]
    Set(SetLidarCommands),
}

#[derive(Subcommand, Debug)]
pub enum SetLidarCommands {
    /// Set the LiDAR device path for a specific Lidar ID.
    Path {
        /// Lidar ID (e.g., 0, 1).
        id: usize,
        /// New LiDAR device path.
        path: String,
    },
    // ここに将来的にbaudrateなどの設定を追加できる
}

#[derive(Subcommand, Debug)]
pub enum SlamCommands {
    /// Get a single LiDAR scan for the SLAM map.
    GetLidar,
    /// Start continuous SLAM updates.
    #[command(alias = "c")]
    Continuous,
    /// Pause continuous SLAM updates.
    Pause,
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

pub fn handle_command(app: &mut MyApp, ctx: &egui::Context, cli: Cli) {
    // 新しいコマンド実行のグループIDを生成
    app.next_group_id += 1;
    let current_group_id = app.next_group_id;

    match cli.command {
        Commands::Help => {
            // clapのヘルプ文字列を生成
            let mut cmd = Cli::command();
            // Usageを含まないようにヘルプのテンプレートをカスタマイズ
            cmd = cmd.help_template("{about-with-newline}\n{all-args}");
            let help_text = cmd.render_help().to_string();

            // 生成されたヘルプ文字列をコンソールに1行ずつ追加
            for line in help_text.lines() {
                app.command_history.push(ConsoleOutputEntry {
                    text: line.to_string(),
                    group_id: current_group_id,
                });
            }
        }
        Commands::Camera { command } => {
            if let Some(camera_command) = command {
                match camera_command {
                    CameraCommands::EnterMode => {
                        app.app_mode = AppMode::Camera;
                        app.command_history.push(ConsoleOutputEntry {
                            text: "Mode set to Camera.".to_string(),
                            group_id: current_group_id,
                        });
                    }
                }
            } else {
                app.app_mode = AppMode::Camera;
                app.command_history.push(ConsoleOutputEntry {
                    text: "Mode set to Camera.".to_string(),
                    group_id: current_group_id,
                });
            }
        }
        Commands::Lidar { command } => {
            if let Some(lidar_command) = command {
                match lidar_command {
                    LidarCommands::EnterMode => {
                        app.app_mode = AppMode::Lidar;
                        app.command_history.push(ConsoleOutputEntry {
                            text: "Mode set to LiDAR.".to_string(),
                            group_id: current_group_id,
                        });
                    }
                    LidarCommands::SlamToggle { id } => {
                        if let Some(lidar_state) = app.lidars.get_mut(id) {
                            lidar_state.is_active_for_slam = !lidar_state.is_active_for_slam;
                            let status = if lidar_state.is_active_for_slam {
                                "active"
                            } else {
                                "inactive"
                            };
                            app.command_output_sender
                                .send(format!("LiDAR {} is now {} for SLAM.", id, status))
                                .unwrap_or_default();
                        } else {
                            app.command_output_sender
                                .send(format!("ERROR: LiDAR ID {} not found.", id))
                                .unwrap_or_default();
                        }
                    }
                    LidarCommands::Set(set_lidar_command) => match set_lidar_command {
                        SetLidarCommands::Path { id, path } => {
                            if let Some(lidar_state) = app.lidars.get_mut(id) {
                                lidar_state.path = path.clone();
                                app.command_output_sender
                                    .send(format!("LiDAR {} path set to: {}", id, path))
                                    .unwrap_or_default();
                            } else {
                                app.command_output_sender
                                    .send(format!(
                                        "ERROR: No LiDAR {} configured or found to set path for.",
                                        id
                                    ))
                                    .unwrap_or_default();
                            }
                        }
                    },
                }
            } else {
                // If 'lidar' is run without subcommands, switch to Lidar mode.
                app.app_mode = AppMode::Lidar;
                app.command_history.push(ConsoleOutputEntry {
                    text: "Mode set to LiDAR.".to_string(),
                    group_id: current_group_id,
                });
            }
        }
        Commands::Slam { command } => {
            app.app_mode = AppMode::Slam;
            if let Some(slam_command) = command {
                match slam_command {
                    SlamCommands::GetLidar => {
                        app.slam_map_bounding_box = None; // 表示範囲をリセット
                        app.single_scan_requested_by_ui = true;
                        app.command_history.push(ConsoleOutputEntry {
                            text: "Requesting single scan for SLAM.".to_string(),
                            group_id: current_group_id,
                        });
                    }
                    SlamCommands::Continuous => {
                        app.slam_map_bounding_box = None; // 表示範囲をリセット
                        app.slam_mode = crate::app::SlamMode::Continuous;
                        app.slam_command_sender
                            .send(crate::app::SlamThreadCommand::StartContinuous)
                            .unwrap_or_default();
                        app.command_history.push(ConsoleOutputEntry {
                            text: "SLAM set to continuous mode.".to_string(),
                            group_id: current_group_id,
                        });
                    }
                    SlamCommands::Pause => {
                        app.slam_mode = crate::app::SlamMode::Paused;
                        app.slam_command_sender
                            .send(crate::app::SlamThreadCommand::Pause)
                            .unwrap_or_default();
                        app.command_history.push(ConsoleOutputEntry {
                            text: "SLAM paused.".to_string(),
                            group_id: current_group_id,
                        });
                    }
                }
            } else {
                app.command_history.push(ConsoleOutputEntry {
                    text: "Switched to SLAM mode.".to_string(),
                    group_id: current_group_id,
                });
            }
        }
        Commands::Demo { mode } => {
            app.app_mode = AppMode::Demo;
            let mode_str = mode.as_deref().unwrap_or("scan");
            match mode_str {
                "scan" => {
                    app.demo_manager.set_mode(DemoMode::RotatingScan);
                    app.command_history.push(ConsoleOutputEntry {
                        text: "Mode set to Demo (Rotating Scan).".to_string(),
                        group_id: current_group_id,
                    });
                }
                "ripple" => {
                    app.demo_manager.set_mode(DemoMode::ExpandingRipple);
                    app.command_history.push(ConsoleOutputEntry {
                        text: "Mode set to Demo (Expanding Ripple).".to_string(),
                        group_id: current_group_id,
                    });
                }
                "breathing" => {
                    app.demo_manager.set_mode(DemoMode::BreathingCircle);
                    app.command_history.push(ConsoleOutputEntry {
                        text: "Mode set to Demo (Breathing Circle).".to_string(),
                        group_id: current_group_id,
                    });
                }
                "table" => {
                    app.demo_manager.set_mode(DemoMode::Table);
                    app.command_history.push(ConsoleOutputEntry {
                        text: "Mode set to Demo (Table).".to_string(),
                        group_id: current_group_id,
                    });
                }
                _ => {
                    // This case should not be reached due to clap's value_parser
                    app.command_history.push(ConsoleOutputEntry {
                        text: format!("ERROR: Unknown demo mode: '{}'. Use 'scan', 'ripple', 'breathing', or 'table'.",
                            mode_str), group_id: current_group_id });
                }
            }
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
        Commands::Version => {
            app.command_history.push(ConsoleOutputEntry {
                text: format!("Mobitia Version: {}", env!("CARGO_PKG_VERSION")),
                group_id: current_group_id,
            });
        }

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

                // TODO: どのLidarの点群を保存するか指定できるようにする
                let lidar_points_clone = if let Some(lidar_state) = app.lidars.get(0) {
                    lidar_state.points.clone()
                } else {
                    sender_clone
                        .send("ERROR: No LiDAR 0 configured to save image from.".to_string())
                        .unwrap_or_default();
                    return; // 点群がない場合は処理を中断
                };

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
        Commands::Map { command } => match command {
            MapCommands::Load { path } => {
                /*
                let msg = format!(
                    "Queueing up individual submaps from '{}'...",
                    path.display()
                );
                println!("{}", msg);
                app.command_history.push(ConsoleOutputEntry {
                    text: msg,
                    group_id: current_group_id,
                });
                */

                // ロード開始時にマップをクリア
                app.current_map_points.clear();
                app.submaps.clear();
                app.robot_trajectory.clear();
                app.slam_map_bounding_box = None;

                let submaps_base_path = path.join("submaps");
                if !submaps_base_path.exists() || !submaps_base_path.is_dir() {
                    let msg_error = format!(
                        "ERROR: Submaps directory not found at: {}",
                        submaps_base_path.display()
                    );
                    println!("{}", msg_error);
                    app.command_history.push(ConsoleOutputEntry {
                        text: msg_error,
                        group_id: current_group_id,
                    });
                    return;
                }

                let pattern = regex::Regex::new(r"submap_\d{3}").unwrap();
                let mut found_submaps = Vec::new();

                match std::fs::read_dir(&submaps_base_path) {
                    Ok(entries) => {
                        for entry in entries.filter_map(|entry| entry.ok()) {
                            if entry.file_type().map_or(false, |ft| ft.is_dir()) {
                                let file_name = entry.file_name();
                                if let Some(name) = file_name.to_str() {
                                    if pattern.is_match(name) {
                                        found_submaps.push(entry.path());
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => {
                        let msg_error = format!(
                            "ERROR: Failed to read submaps directory '{}': {}",
                            submaps_base_path.display(),
                            e
                        );
                        println!("{}", msg_error);
                        app.command_history.push(ConsoleOutputEntry {
                            text: msg_error,
                            group_id: current_group_id,
                        });
                        return;
                    }
                }

                if found_submaps.is_empty() {
                    let msg = format!(
                        "No submaps found matching 'submap_NNN' in '{}'.",
                        submaps_base_path.display()
                    );
                    println!("{}", msg);
                    app.command_history.push(ConsoleOutputEntry {
                        text: msg,
                        group_id: current_group_id,
                    });
                    return;
                }

                found_submaps.sort(); // 名前順にソート (submap_000, submap_001, ...)

                /*
                let msg_found_count = format!("Found and queued {} submaps.", found_submaps.len());
                println!("{}", msg_found_count);
                app.command_history.push(ConsoleOutputEntry {
                    text: msg_found_count,
                    group_id: current_group_id,
                });
                */

                // 実際のロード処理は update ループに任せる
                app.submap_load_queue = Some(found_submaps);
                app.last_submap_load_time = None; // タイマーをリセット
                app.app_mode = AppMode::Slam; // 先にSlamモードに切り替え
            }
        },
    }
}
