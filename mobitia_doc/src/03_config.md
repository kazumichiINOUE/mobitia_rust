### 2.5 設定 (`config.toml` & `src/config.rs`)
アプリケーションの動作は、プロジェクトルートにある `config.toml` ファイルによって外部から設定可能です。このファイルが存在しない場合、アプリケーションはデフォルト値で新しい `config.toml` を生成します。

設定の読み込みと構造体のマッピングは `src/config.rs` で定義されています。

#### 設定構造体
`config.toml` の内容は、以下のRust構造体にマッピングされます。

```rust
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
    #[serde(default = "default_lidars")]
    pub lidar: Vec<LidarTomlConfig>,
}

// --- SLAM-related Parameters ---
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SlamConfig {
    pub csize: f32,
    pub map_width: usize,
    pub map_height: usize,
    pub map_update_method: MapUpdateMethod,
    pub point_representation: PointRepresentationMethod,
    pub log_odds_clamp_max: f64,
    pub log_odds_clamp_min: f64,
    // ... 他の多くのSLAMパラメータ
    pub use_odometry_as_initial_guess: bool,
    pub num_scans_per_submap: usize,
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

// --- LiDAR Configuration ---
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

// ... 他のConfig構造体 (MapConfig, UiConfig) ...
```

#### 主要なパラメータ
- **`[slam]` セクション**: SLAMアルゴリズムの振る舞いを制御します。
    - `csize`: SLAMマップの1セルのサイズ(m)。
    - `use_odometry_as_initial_guess`: SLAMの自己位置推定において、モーターオドメトリを初期推測値として利用するかどうか。`true`に設定することで、よりロバストな推定が期待できます。
    - `population_size`, `generations`: 自己位置推定に用いる最適化アルゴリズム（差分進化）のパラメータ。
    - `update_interval_ms`: SLAMの更新処理（自己位置推定とマップ更新）を実行する周期をミリ秒単位で指定します。値を小さくすると推定頻度が上がりますが、計算負荷も増加します。デフォルトは `1000` です。
    - `min_valid_points_for_de`: 自己位置推定に最適化（差分進化）を使用するために最低限必要なLiDARの有効点数を指定します。この閾値を下回った場合、オドメトリのみで姿勢更新が行われ、計算負荷を低減し頑健性を高めます。デフォルトは `0` です。

- **`[motor]` セクション**: モータードライバとの通信や、物理的なロボットのパラメータを定義します。
    - `port`, `baud_rate`: モータードライバとのシリアル通信設定。
    - `wheel_diameter`, `tread_width`: オドメトリ計算に使用されるロボットの物理寸法。

- **`[[lidar]]` セクション**: 各LiDARセンサーの設定を定義します。配列形式（`[[...]]`）であり、複数のLiDARを接続可能です。
    - `id`: 各LiDARを識別するための一意のID。
    - `path`: LiDARのシリアルポートのパス。
    - `origin_x`, `origin_y`, `rotation_deg`: ロボットの中心から見たLiDARセンサーの取り付け位置と向き。
    - `is_active_for_slam`: このLiDARからのスキャンデータをSLAM計算に使用するかどうか。
