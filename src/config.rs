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

// --- SLAM-related Parameters ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SlamConfig {
    pub csize: f32,
    pub map_width: usize,
    pub map_height: usize,
    #[serde(default)]
    pub map_update_method: MapUpdateMethod,
    #[serde(default)]
    pub point_representation: PointRepresentationMethod,
    pub log_odds_clamp_max: f64,
    pub log_odds_clamp_min: f64,
    pub prob_occupied: f64,
    pub prob_free: f64,
    pub decay_rate: f64,
    pub max_matching_dist: f32,
    pub match_sigma: f32,
    pub gaussian_kernel_sigma: f64,
    pub gaussian_kernel_radius: i32,
    pub position_score_weight: f64,
    pub feature_score_weight: f64,
    pub normal_alignment_score_weight: f64,
    pub corner_score_weight: f64,
    pub translation_penalty_weight: f64,
    pub rotation_penalty_weight: f64,
    pub penalty_log_odds_threshold: f64,
    pub penalty_factor: f64,
    pub population_size: usize,
    pub generations: usize,
    pub wxy: f32,
    pub wa_degrees: f32,
    pub f_de: f32,
    pub cr: f32,
    #[serde(default)]
    pub use_odometry_as_initial_guess: bool,
    pub num_scans_per_submap: usize,
    pub online_slam_view_size: f32,
    #[serde(default)]
    pub min_valid_points_for_de: usize,
    #[serde(default)]
    pub update_interval_ms: u64,
}

// --- Map-related Parameters ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MapConfig {
    pub submap_load_delay_ms: u64,
    pub map_load_update_distance: f32,
}

// --- UI-related Parameters ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct UiConfig {
    pub show_xppen_panel: bool,
    pub show_camera_panel: bool,
    pub show_osmo_panel: bool,
    #[serde(default = "default_nav_zoom_view_size")]
    pub nav_zoom_view_size: f32,
}

fn default_nav_zoom_view_size() -> f32 {
    4.0
}

// --- Motor Configuration ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MotorConfig {
    pub port: String,
    pub baud_rate: u32,
    pub step_resolution_deg: f32,
    pub wheel_diameter: f32,
    pub tread_width: f32,
    pub gear_ratio: f32,
    pub max_linear_velocity: f32,
    pub max_angular_velocity: f32,
}
// ... (rest of the file)
impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            port: "/dev/tty.usbserial-default".to_string(),
            baud_rate: 230400,
            step_resolution_deg: 0.01,
            wheel_diameter: 0.311,
            tread_width: 0.461,
            gear_ratio: 50.0,
            max_linear_velocity: 1.5,
            max_angular_velocity: 1.0,
        }
    }
}

// --- Robot Configuration ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RobotConfig {
    pub width: f32,
    pub length: f32,
    pub min_mapping_dist: f32,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            width: 0.4,
            length: 0.5,
            min_mapping_dist: 0.3,
        }
    }
}

// --- DWA Configuration ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct DwaConfig {
    pub max_accel_v: f32, // m/s^2
    pub max_accel_w: f32, // rad/s^2
    pub max_speed_v: f32, // m/s
    pub max_speed_w: f32, // rad/s
    pub predict_time: f32, // s
    pub dt: f32,          // s (simulation step)
    pub v_samples: usize,
    pub w_samples: usize,
}

impl Default for DwaConfig {
    fn default() -> Self {
        Self {
            max_accel_v: 0.5,
            max_accel_w: 1.0,
            max_speed_v: 0.5,
            max_speed_w: 1.0,
            predict_time: 2.0,
            dt: 0.1,
            v_samples: 6,
            w_samples: 11,
        }
    }
}

// --- Elastic Band Configuration ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ElasticBandConfig {
    pub internal_force_gain: f32,
    pub external_force_gain: f32,
    pub obstacle_safety_dist: f32,
    pub num_iterations: usize,
    pub global_path_gain: f32,
}

impl Default for ElasticBandConfig {
    fn default() -> Self {
        Self {
            internal_force_gain: 0.2,
            external_force_gain: 0.1,
            obstacle_safety_dist: 0.8,
            num_iterations: 10,
            global_path_gain: 0.05,
        }
    }
}

// --- Navigation Configuration ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct NavConfig {
    #[serde(default = "default_initial_pose")]
    pub initial_pose: [f32; 3], // [x, y, angle_degrees]
    #[serde(default)]
    pub debug_use_dummy_scan: bool,
    #[serde(default = "default_initial_localization_interval_frames")]
    pub initial_localization_interval_frames: usize,
    #[serde(default = "default_tracking_update_interval_frames")]
    pub tracking_update_interval_frames: usize,
    #[serde(default = "default_tracking_wxy")]
    pub tracking_wxy: f32,
    #[serde(default = "default_tracking_wa_degrees")]
    pub tracking_wa_degrees: f32,
    #[serde(default = "default_tracking_population_size")]
    pub tracking_population_size: usize,
    #[serde(default = "default_tracking_generations")]
    pub tracking_generations: usize,
    #[serde(default = "default_lookahead_distance")]
    pub lookahead_distance: f32,
    #[serde(default = "default_target_velocity")]
    pub target_velocity: f32,
    #[serde(default = "default_goal_tolerance")]
    pub goal_tolerance: f32,
    #[serde(default)]
    pub debug_show_corner_cells: bool,
    #[serde(default = "default_lidar_avoid_dist")]
    pub lidar_avoid_dist: f32,
    #[serde(default = "default_collision_check_predict_time")]
    pub collision_check_predict_time: f32,
    #[serde(default)]
    pub dwa: DwaConfig,
    #[serde(default)]
    pub elastic_band: ElasticBandConfig,
    #[serde(default = "default_recovery_enabled")]
    pub recovery_enabled: bool,
    #[serde(default = "default_recovery_trigger_dist")]
    pub recovery_trigger_dist: f32,
    #[serde(default = "default_recovery_target_dist")]
    pub recovery_target_dist: f32,
    #[serde(default = "default_recovery_speed")]
    pub recovery_speed: f32,
}

fn default_initial_pose() -> [f32; 3] {
    [0.0, 0.0, 0.0]
}

fn default_initial_localization_interval_frames() -> usize {
    30
}

fn default_tracking_update_interval_frames() -> usize {
    60
}

fn default_tracking_wxy() -> f32 {
    0.05
}

fn default_tracking_wa_degrees() -> f32 {
    2.0
}

fn default_tracking_population_size() -> usize {
    30
}

fn default_tracking_generations() -> usize {
    5
}

fn default_lookahead_distance() -> f32 {
    0.5
}

fn default_target_velocity() -> f32 {
    0.3
}

fn default_goal_tolerance() -> f32 {
    0.2
}

fn default_lidar_avoid_dist() -> f32 {
    0.4
}

fn default_collision_check_predict_time() -> f32 {
    1.0
}

fn default_recovery_enabled() -> bool {
    true
}

fn default_recovery_trigger_dist() -> f32 {
    0.2
}

fn default_recovery_target_dist() -> f32 {
    0.8
}

fn default_recovery_speed() -> f32 {
    0.15
}

impl Default for NavConfig {
    fn default() -> Self {
        Self {
            initial_pose: default_initial_pose(),
            debug_use_dummy_scan: false,
            initial_localization_interval_frames: default_initial_localization_interval_frames(),
            tracking_update_interval_frames: default_tracking_update_interval_frames(),
            tracking_wxy: default_tracking_wxy(),
            tracking_wa_degrees: default_tracking_wa_degrees(),
            tracking_population_size: default_tracking_population_size(),
            tracking_generations: default_tracking_generations(),
            lookahead_distance: default_lookahead_distance(),
            target_velocity: default_target_velocity(),
            goal_tolerance: default_goal_tolerance(),
            debug_show_corner_cells: false,
            lidar_avoid_dist: default_lidar_avoid_dist(),
            collision_check_predict_time: default_collision_check_predict_time(),
            dwa: DwaConfig::default(),
            elastic_band: ElasticBandConfig::default(),
            recovery_enabled: default_recovery_enabled(),
            recovery_trigger_dist: default_recovery_trigger_dist(),
            recovery_target_dist: default_recovery_target_dist(),
            recovery_speed: default_recovery_speed(),
        }
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
    #[serde(default)]
    pub motor: MotorConfig,
    #[serde(default)]
    pub robot: RobotConfig,
    #[serde(default = "default_lidars")]
    pub lidar: Vec<LidarTomlConfig>,
    #[serde(default)]
    pub nav: NavConfig,
}

impl Config {
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

        lidars.sort_by_key(|l| l.id);
        lidars
    }
}

fn default_map_load_update_distance() -> f32 {
    0.1 // 10cm
}

fn default_lidars() -> Vec<LidarTomlConfig> {
    Vec::new()
}

// --- Default Implementations ---

impl Default for UiConfig {
    fn default() -> Self {
        Self {
            show_xppen_panel: true,
            show_camera_panel: true,
            show_osmo_panel: true,
            nav_zoom_view_size: default_nav_zoom_view_size(),
        }
    }
}

impl Default for MapConfig {
    fn default() -> Self {
        Self {
            submap_load_delay_ms: 500,
            map_load_update_distance: default_map_load_update_distance(),
        }
    }
}

impl Default for SlamConfig {
    fn default() -> Self {
        Self {
            csize: 0.025,
            map_width: 3200,
            map_height: 3200,
            map_update_method: MapUpdateMethod::default(),
            point_representation: PointRepresentationMethod::default(),
            log_odds_clamp_max: 5.0,
            log_odds_clamp_min: -5.0,
            prob_occupied: 0.6,
            prob_free: 0.4,
            decay_rate: 1.0,
            max_matching_dist: 0.5,
            match_sigma: 0.1,
            gaussian_kernel_sigma: 0.8,
            gaussian_kernel_radius: 1,
            position_score_weight: 0.1,
            feature_score_weight: 0.4,
            normal_alignment_score_weight: 0.5,
            corner_score_weight: 0.3,
            translation_penalty_weight: 100.0,
            rotation_penalty_weight: 1000.0,
            penalty_log_odds_threshold: -0.2,
            penalty_factor: 1.0,
            population_size: 200,
            generations: 100,
            wxy: 0.8,
            wa_degrees: 35.0,
            f_de: 0.1,
            cr: 0.6,
            use_odometry_as_initial_guess: false,
            num_scans_per_submap: 20,
            online_slam_view_size: 30.0,
            min_valid_points_for_de: 0,
            update_interval_ms: 1000,
        }
    }
}
