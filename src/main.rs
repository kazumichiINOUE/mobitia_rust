use anyhow::Result;
use eframe::egui;
use std::fs;
use std::path::Path;

// Define application modules
mod app;
mod camera;
mod cli;
mod config; // Add config module
mod demo;
mod lidar;
mod osmo;
mod slam;
mod xppen;

use app::MyApp;

fn main() -> Result<(), eframe::Error> {
    // --- Config loading ---
    let config_path = Path::new("config.toml");
    let app_config = if config_path.exists() {
        let config_str = fs::read_to_string(config_path).expect("Failed to read config.toml");
        toml::from_str(&config_str).expect("Failed to parse config.toml")
    } else {
        let default_config_toml = r###"
# Mobitia Configuration File

# --- General Application Settings ---
# [general]
# ui_theme = "Dark" # Possible values: "Dark", "Light"

[slam]
# --- Map Representation ---
# The size of each grid cell in meters.
csize = 0.025

# The width of the occupancy grid in cells.
map_width = 3200

# The height of the occupancy grid in cells.
map_height = 3200

# The method used to update the occupancy grid.
# Possible values are: "Binary", "Probabilistic", "Hybrid"
map_update_method = "Probabilistic"

# How a point is represented within a cell.
# Possible values are: "CellCenter", "Centroid"
point_representation = "Centroid"

# Maximum log-odds value for a cell, preventing extreme confidence.
log_odds_clamp_max = 5.0

# Minimum log-odds value for a cell.
log_odds_clamp_min = -5.0

# --- Probabilistic Model (for "Probabilistic" or "Hybrid" map update methods) ---
# Probability assigned to a cell when a laser hits it.
prob_occupied = 0.6

# Probability assigned to a cell when a laser passes through it.
prob_free = 0.4

# Decay rate for log-odds values over time. A value of 1.0 means no decay.
decay_rate = 1.0

# --- Scan Matching (Pose Estimation) ---
# The maximum distance between a scan point and a map point to be considered a valid match.
max_matching_dist = 0.5

# The standard deviation (sigma) of the Gaussian function used for calculating the distance penalty in scan matching.
match_sigma = 0.1

# Sigma for the Gaussian kernel used in gaussian_match_count.
gaussian_kernel_sigma = 0.8

# Radius for the Gaussian kernel.
gaussian_kernel_radius = 1

# --- Scoring Weights (for Scan Matching) ---
# Weight for the position alignment component of the score.
position_score_weight = 0.1

# Weight for the feature (e.g., edge-ness) similarity component.
feature_score_weight = 0.4

# Weight for the alignment of normal vectors.
normal_alignment_score_weight = 0.5

# --- Pose Update Penalties (for Scan Matching) ---
# Penalty applied for translational movement of the robot.
translation_penalty_weight = 100.0

# Penalty applied for rotational movement of the robot.
rotation_penalty_weight = 1000.0

# Log-odds threshold below which a penalty is applied.
penalty_log_odds_threshold = -0.2

# Factor to scale the penalty.
penalty_factor = 1.0

# --- Differential Evolution (Optimizer for Scan Matching) ---
# Number of individuals in the DE population.
population_size = 200

# Number of generations the DE algorithm runs for.
generations = 100

# Search window size for translation (x, y) in meters.
wxy = 0.8

# Search window size for rotation (angle) in degrees.
wa_degrees = 35.0

# Mutation factor for Differential Evolution.
f_de = 0.1

# Crossover rate for Differential Evolution.
cr = 0.6

# --- Submap Generation ---
# Number of laser scans to be collected before generating a new submap.
num_scans_per_submap = 20

# The size of the viewing area (in meters) when running online SLAM.
online_slam_view_size = 30.0

[map]
# Delay in milliseconds between loading each submap for 'list-and-load'.
submap_load_delay_ms = 500
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

            Box::new(MyApp::new(cc, app_config))
        }),
    )
}
