use crate::app::LidarState;
use eframe::egui::Vec2;
use serde::{Deserialize, Serialize};

// --- LiDAR Configuration (for config.toml) ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LidarTomlConfig {
    pub id: usize,
    pub path: String,
    pub baud_rate: u32,
    pub origin_x: f32,
    pub origin_y: f32,
    pub rotation_deg: f32,
    pub data_filter_angle_min_deg: f32,
    pub data_filter_angle_max_deg: f32,
    pub is_active_for_slam: bool,
}

// --- Map Update Method Selection ---
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum MapUpdateMethod {
    Binary,
    Probabilistic,
    Hybrid,
}

impl Default for MapUpdateMethod {
    fn default() -> Self {
        MapUpdateMethod::Probabilistic
    }
}

// --- Point Representation Method ---
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum PointRepresentationMethod {
    CellCenter,
    Centroid,
}

impl Default for PointRepresentationMethod {
    fn default() -> Self {
        PointRepresentationMethod::Centroid
    }
}

// --- Application-wide Configuration ---
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct Config {
    #[serde(default)]
    pub slam: SlamConfig,
    #[serde(default)]
    pub map: MapConfig,
    #[serde(default)]
    pub ui: UiConfig,
    #[serde(default = "default_lidars")]
    pub lidar: Vec<LidarTomlConfig>,
}

impl Config {
    /// Converts LiDAR configurations from TOML format to the application's LidarState.
    pub fn get_lidar_states(&self) -> Vec<LidarState> {
        let mut lidars: Vec<LidarState> = self
            .lidar
            .iter()
            .map(|toml_config| LidarState {
                id: toml_config.id,
                path: toml_config.path.clone(),
                baud_rate: toml_config.baud_rate,
                points: Vec::new(),
                connection_status: "Connecting...".to_string(),
                status_messages: Vec::new(),
                origin: Vec2::new(toml_config.origin_x, toml_config.origin_y),
                rotation: toml_config.rotation_deg.to_radians(),
                data_filter_angle_min: toml_config.data_filter_angle_min_deg,
                data_filter_angle_max: toml_config.data_filter_angle_max_deg,
                is_active_for_slam: toml_config.is_active_for_slam,
            })
            .collect();

        // Sort by ID to ensure a stable order
        lidars.sort_by_key(|l| l.id);
        lidars
    }
}

fn default_lidars() -> Vec<LidarTomlConfig> {
    // Returns an empty vector if [[lidar]] is not present in config.toml
    Vec::new()
}

// --- UI-related Parameters ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct UiConfig {
    pub show_xppen_panel: bool,
    pub show_camera_panel: bool,
    pub show_osmo_panel: bool,
}

// --- Map-related Parameters ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MapConfig {
    pub submap_load_delay_ms: u64,
}

// --- SLAM-related Parameters ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SlamConfig {
    // --- Map Representation ---
    pub csize: f32,
    pub map_width: usize,
    pub map_height: usize,
    #[serde(default)]
    pub map_update_method: MapUpdateMethod,
    #[serde(default)]
    pub point_representation: PointRepresentationMethod,
    pub log_odds_clamp_max: f64,
    pub log_odds_clamp_min: f64,

    // --- Probabilistic Model ---
    pub prob_occupied: f64,
    pub prob_free: f64,
    pub decay_rate: f64,

    // --- Scan Matching ---
    pub max_matching_dist: f32,
    pub match_sigma: f32,
    pub gaussian_kernel_sigma: f64,
    pub gaussian_kernel_radius: i32,

    // --- Scoring Weights ---
    pub position_score_weight: f64,
    pub feature_score_weight: f64,
    pub normal_alignment_score_weight: f64,
    pub corner_score_weight: f64,

    // --- Pose Update Penalties ---
    pub translation_penalty_weight: f64,
    pub rotation_penalty_weight: f64,
    pub penalty_log_odds_threshold: f64,
    pub penalty_factor: f64,

    // --- Differential Evolution ---
    pub population_size: usize,
    pub generations: usize,
    pub wxy: f32,
    pub wa_degrees: f32,
    pub f_de: f32, // Renamed to avoid conflict with `std::f32::consts::F`
    pub cr: f32,

    // --- Submap Generation ---
    pub num_scans_per_submap: usize,
    pub online_slam_view_size: f32,
}

// --- Default Implementations ---

impl Default for UiConfig {
    fn default() -> Self {
        Self {
            show_xppen_panel: true,
            show_camera_panel: true,
            show_osmo_panel: true,
        }
    }
}

impl Default for MapConfig {
    fn default() -> Self {
        Self {
            submap_load_delay_ms: 500,
        }
    }
}

impl Default for SlamConfig {
    fn default() -> Self {
        Self {
            // --- Map Representation ---
            csize: 0.025,
            map_width: 3200,
            map_height: 3200,
            map_update_method: MapUpdateMethod::default(),
            point_representation: PointRepresentationMethod::default(),
            log_odds_clamp_max: 5.0,
            log_odds_clamp_min: -5.0,

            // --- Probabilistic Model ---
            prob_occupied: 0.6,
            prob_free: 0.4,
            decay_rate: 1.0,

            // --- Scan Matching ---
            max_matching_dist: 0.5,
            match_sigma: 0.1,
            gaussian_kernel_sigma: 0.8,
            gaussian_kernel_radius: 1,

            // --- Scoring Weights ---
            position_score_weight: 0.1,
            feature_score_weight: 0.4,
            normal_alignment_score_weight: 0.5,
            corner_score_weight: 0.3,

            // --- Pose Update Penalties ---
            translation_penalty_weight: 100.0,
            rotation_penalty_weight: 1000.0,
            penalty_log_odds_threshold: -0.2,
            penalty_factor: 1.0,

            // --- Differential Evolution ---
            population_size: 200,
            generations: 100,
            wxy: 0.8,
            wa_degrees: 35.0,
            f_de: 0.1,
            cr: 0.6,

            // --- Submap Generation ---
            num_scans_per_submap: 20,
            online_slam_view_size: 30.0,
        }
    }
}
