use anyhow::Result;
use clap::Parser;
use eframe::egui;
use std::fs;
use std::path::Path;

// Define application modules
mod app;
mod camera;
mod cli;
mod config; // Add config module
mod demo;
mod experiment; // Add experiment module
mod lidar;
mod motors;
mod navigation;
mod osmo;
mod slam;

mod ui;
mod xppen;

use crate::config::Config;
use app::MyApp; // 追加
use tracing_subscriber::prelude::*;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Run in experiment mode (headless)
    #[arg(long)]
    experiment: bool,

    /// Experiment mode: "baseline", "proposed", or "ground_truth"
    #[arg(long, default_value = "baseline")]
    mode: String,

    /// Path to input directory containing submaps (e.g. slam_results/slam_result_.../)
    #[arg(long)]
    input: Option<std::path::PathBuf>,

    /// Path to output directory
    #[arg(long, default_value = "experiments")]
    output: std::path::PathBuf,

    /// Path to anchor_log.csv for degenerate environment analysis (Optional)
    #[arg(long)]
    anchors: Option<std::path::PathBuf>,
}

fn main() -> Result<(), eframe::Error> {
    let args = Args::parse();

    if args.experiment {
        if let Some(input_dir) = args.input {
            let exp_args = experiment::ExperimentArgs {
                mode: args.mode,
                input_dir,
                output_dir: args.output,
                anchors_path: args.anchors,
            };
            match experiment::run_experiment(exp_args) {
                Ok(_) => {
                    println!("Experiment finished successfully.");
                    std::process::exit(0);
                }
                Err(e) => {
                    eprintln!("Experiment failed: {}", e);
                    std::process::exit(1);
                }
            }
        } else {
            eprintln!("Error: --input is required for experiment mode.");
            std::process::exit(1);
        }
    }

    // --- Tracing Setup ---
    // Create traces directory if it doesn't exist
    if let Err(e) = std::fs::create_dir_all("./traces") {
        eprintln!("Failed to create traces directory: {}", e);
    }

    // Create a new layer that writes a Chrome Trace file
    let (chrome_layer, _guard) = tracing_chrome::ChromeLayerBuilder::new()
        .file("./traces/trace.json") // Save to ./traces/trace.json (rotated)
        .include_args(true)
        .build();

    tracing_subscriber::registry().with(chrome_layer).init();
    // --- End of Tracing Setup ---

    // --- Config loading ---
    let config_path = Path::new("config.toml");
    let config: Config = if config_path.exists() {
        let config_str = fs::read_to_string(config_path).expect("Failed to read config.toml");
        toml::from_str(&config_str).expect("Failed to parse config.toml")
    } else {
        let default_config_toml = r###"
# Mobitia Configuration File

[map]
submap_load_delay_ms = 500

[ui]
show_xppen_panel = true
show_camera_panel = true
show_osmo_panel = true

[slam]
# --- Map Representation ---
csize = 0.025
map_width = 3200
map_height = 3200
map_update_method = "Probabilistic" # "Binary", "Probabilistic", "Hybrid"
point_representation = "Centroid" # "CellCenter", "Centroid"
log_odds_clamp_max = 5.0
log_odds_clamp_min = -5.0

# --- Probabilistic Model ---
prob_occupied = 0.6
prob_free = 0.4
decay_rate = 1.0

# --- Scan Matching ---
max_matching_dist = 0.5
match_sigma = 0.1
gaussian_kernel_sigma = 0.8
gaussian_kernel_radius = 1

# --- Scoring Weights ---
position_score_weight = 0.1
feature_score_weight = 0.4
normal_alignment_score_weight = 0.5

# --- Pose Update Penalties ---
translation_penalty_weight = 100.0
rotation_penalty_weight = 1000.0
penalty_log_odds_threshold = -0.2
penalty_factor = 1.0

# --- Differential Evolution ---
population_size = 200
generations = 100
wxy = 0.8
wa_degrees = 35.0
f_de = 0.1
cr = 0.6

# --- Submap Generation ---
num_scans_per_submap = 20
online_slam_view_size = 30.0
"###;
        fs::write(config_path, default_config_toml).expect("Failed to write default config.toml");
        toml::from_str(default_config_toml).expect("Failed to parse default config from string")
    };
    // --- End of Config loading ---

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
            // --- Font setup ---
            let mut fonts = egui::FontDefinitions::default();

            // Install Source Han Sans font
            fonts.font_data.insert(
                "source_han_sans".to_owned(),
                egui::FontData::from_static(include_bytes!(
                    "../assets/fonts/SourceHanSans-Regular.otf"
                )),
            );

            // Install Hack Nerd Font
            fonts.font_data.insert(
                "hack_nerd".to_owned(),
                egui::FontData::from_static(include_bytes!(
                    "../assets/fonts/HackNerdFont-Regular.ttf"
                )),
            );

            // Set Hack Nerd Font as the first priority for both proportional and monospace fonts
            fonts
                .families
                .entry(egui::FontFamily::Proportional)
                .or_default()
                .insert(0, "hack_nerd".to_owned());
            fonts
                .families
                .entry(egui::FontFamily::Monospace)
                .or_default()
                .insert(0, "hack_nerd".to_owned());

            // Set Source Han Sans as a fallback font
            fonts
                .families
                .entry(egui::FontFamily::Proportional)
                .or_default()
                .push("source_han_sans".to_owned());
            fonts
                .families
                .entry(egui::FontFamily::Monospace)
                .or_default()
                .push("source_han_sans".to_owned());

            cc.egui_ctx.set_fonts(fonts);
            // --- End of font setup ---

            // Light/Dark mode setup
            cc.egui_ctx.set_visuals(egui::Visuals::dark());

            // style setup
            let mut style = (*cc.egui_ctx.style()).clone();
            style.text_styles.insert(
                egui::TextStyle::Monospace,
                egui::FontId::proportional(18.0), // Adjust font size here
            );
            cc.egui_ctx.set_style(style);

            Box::new(MyApp::new(cc, config)) // app_config を config に修正
        }),
    )
}
