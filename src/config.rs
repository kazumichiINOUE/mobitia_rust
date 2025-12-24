use serde::{Serialize, Deserialize};

// --- Map Update Method Selection (Moved from src/slam/mod.rs) ---
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

// --- Point Representation Method (Moved from src/slam/mod.rs) ---
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

// アプリケーション全体のコンフィグ
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Config {
    #[serde(default)]
    pub slam: SlamConfig,
    #[serde(default)]
    pub map: MapConfig,
    // 将来の拡張用
    // #[serde(default)]
    // pub ui: UiConfig,
}

// map関連のパラメータ
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MapConfig {
    pub submap_load_delay_ms: u64,
}

// SLAM関連のパラメータ
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

// `Config` 全体のデフォルト値を定義
impl Default for Config {
    fn default() -> Self {
        Self {
            slam: SlamConfig::default(),
            map: MapConfig::default(),
        }
    }
}

// `MapConfig` のデフォルト値を定義
impl Default for MapConfig {
    fn default() -> Self {
        Self {
            submap_load_delay_ms: 500,
        }
    }
}

// `SlamConfig` のデフォルト値を定義
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
